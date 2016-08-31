/*
COPYRIGHT (C) 2007  Dan D. V. Bhanderi (dan@bhanderi.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY. See the GNU Lesser General Public License for
more details.
*/

#define S_FUNCTION_NAME		sfun_comedi_di
#define S_FUNCTION_LEVEL	2

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#endif
#include "simstruc.h"

#define NUMBER_OF_PARAMS	(3)

#define COMEDI_DEVICE_PARAM	ssGetSFcnParam(S,0)
#define COMEDI_CHANNEL_PARAM	ssGetSFcnParam(S,1)
#define SAMPLE_TIME_PARAM       ssGetSFcnParam(S,2)

#define COMEDI_DEVICE		((uint_T) mxGetPr(COMEDI_DEVICE_PARAM)[0])
#define COMEDI_CHANNEL		((uint_T) mxGetPr(COMEDI_CHANNEL_PARAM)[0])
#define SAMPLE_TIME             ((real_T) mxGetPr(SAMPLE_TIME_PARAM)[0])


#ifndef MATLAB_MEX_FILE

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>

extern void *ComediDev[];
extern int ComediDev_InUse[];
extern int ComediDev_DIOInUse[];

#endif

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
	static char_T errMsg[256];
	boolean_T allParamsOK = 1;

	if (mxGetNumberOfElements(COMEDI_CHANNEL_PARAM) != 1) {
		sprintf(errMsg, "Channel parameter must be a scalar.\n");
		allParamsOK = 0;
		goto EXIT_POINT;
	}

EXIT_POINT:
	if (!allParamsOK) {
		ssSetErrorStatus(S, errMsg);
	}
}
#endif

static void mdlInitializeSizes(SimStruct *S)
{
	uint_T i;

	ssSetNumSFcnParams(S, NUMBER_OF_PARAMS);
#if defined(MATLAB_MEX_FILE)
	if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
		mdlCheckParameters(S);
		if (ssGetErrorStatus(S) != NULL) {
			return;
		}
	} else {
		return;
	}
#endif
	for (i = 0; i < NUMBER_OF_PARAMS; i++) {
		ssSetSFcnParamNotTunable(S, i);
	}
	ssSetNumInputPorts(S, 0);
	ssSetNumOutputPorts(S, 1);
	ssSetOutputPortWidth(S, 0, 1);
	ssSetNumContStates(S, 0);
	ssSetNumDiscStates(S, 0);
	ssSetNumSampleTimes(S, 1);
	ssSetNumPWork(S,1);
	ssSetNumIWork(S,1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, SAMPLE_TIME);
	ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START 
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
	void *dev;
	int subdev_type = -1;
	int subdev;
	int index = (int)COMEDI_DEVICE - 1;
	unsigned int channel = (unsigned int)COMEDI_CHANNEL;
	int n_channels;
	char *devname[4] = {"/dev/comedi0","/dev/comedi1","/dev/comedi2","/dev/comedi3"};
	char board[50];
	static char_T errMsg[256];
	
	if (!ComediDev[index]) {
		dev = comedi_open(devname[index]);
		if (!dev) {
			sprintf(errMsg, "Comedi open failed\n");
			ssSetErrorStatus(S, errMsg);
			printf("%s", errMsg);
			return;
		}
		lnx_comedi_get_board_name(dev, board);
		printf("COMEDI %s (%s) opened.\n\n", devname[index], board);
		ComediDev[index] = dev;
		
		if ((subdev = comedi_find_subdevice_by_type(dev, COMEDI_SUBD_DI, 0)) < 0) {
			sprintf(errMsg, "Comedi find_subdevice failed (No digital Input)\n");
			//ssSetErrorStatus(S, errMsg);
			//printf("%s", errMsg);
			//comedi_close(dev);
			//return;
		}else {
			subdev_type = COMEDI_SUBD_DI;
			//printf("Subdevice %d type %d of (%s) opened.\n\n", subdev, subdev_type, board);
			}  
		if(subdev == -1){
				if ((subdev = comedi_find_subdevice_by_type(dev, COMEDI_SUBD_DIO, 0)) < 0) {
				sprintf(errMsg, "Comedi find_subdevice failed (No digital I/O)\n");
				ssSetErrorStatus(S, errMsg);
				printf("%s", errMsg);
				comedi_close(dev);
				return;
				}else{
				       subdev_type = COMEDI_SUBD_DIO;
				       //printf("Subdevice %d type %d of (%s) opened.\n\n", subdev, subdev_type, board);
				     }  
		}
		if ((comedi_lock(dev, subdev)) < 0) {
			sprintf(errMsg, "Comedi lock failed for subdevice %d\n", subdev);
			ssSetErrorStatus(S, errMsg);
			printf("%s", errMsg);
			comedi_close(dev);
			return;
		}
	} else {
		dev = ComediDev[index];
		if((subdev = comedi_find_subdevice_by_type(dev, COMEDI_SUBD_DI, 0)) < 0){
		 	subdev = comedi_find_subdevice_by_type(dev, COMEDI_SUBD_DIO, 0);
		 	subdev_type =COMEDI_SUBD_DIO;
		}else subdev_type =COMEDI_SUBD_DI; 
		
		//printf("Subdevice %d type %d opened.\n\n", subdev, subdev_type);
	}
	if ((n_channels = comedi_get_n_channels(dev, subdev)) < 0) {
		sprintf(errMsg, "Comedi get_n_channels failed for subdev %d\n", subdev);
		ssSetErrorStatus(S, errMsg);
		printf("%s", errMsg);
		comedi_unlock(dev, subdev);
		comedi_close(dev);
		return;
	}
	if (channel >= n_channels) {
		sprintf(errMsg, "Comedi channel not available for subdevice %d\n", subdev);
		ssSetErrorStatus(S, errMsg);
		printf("%s", errMsg);
		comedi_unlock(dev, subdev);
		comedi_close(dev);
		return;
	}
	if(subdev_type == COMEDI_SUBD_DIO){
		if ((comedi_dio_config(dev, subdev, channel, COMEDI_INPUT)) < 0) {
			sprintf(errMsg, "Comedi DIO config failed for subdevice %d\n", subdev);
			ssSetErrorStatus(S, errMsg);
			printf("%s", errMsg);
			comedi_unlock(dev, subdev);
			comedi_close(dev);
			return;
		}	
	}	
	ComediDev_InUse[index]++;
	ComediDev_DIOInUse[index]++;
	ssGetPWork(S)[0] = (void *)dev;
	ssGetIWork(S)[0] = subdev;
#endif
}
#endif

static void mdlOutputs(SimStruct *S, int_T tid)
{
	double *y = ssGetOutputPortRealSignal(S,0);

#ifndef MATLAB_MEX_FILE
	unsigned int channel = (unsigned int)COMEDI_CHANNEL;
	void *dev        = (void *)ssGetPWork(S)[0];
	int subdev       = ssGetIWork(S)[0];
	unsigned int bit;
	
	comedi_dio_read(dev, subdev, channel, &bit);
	*y = (double)bit;
#endif
}

static void mdlTerminate(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
	int index = (int)COMEDI_DEVICE - 1;
	void *dev        = (void *)ssGetPWork(S)[0];
	int subdev       = ssGetIWork(S)[0];
	char *devname[4] = {"/dev/comedi0","/dev/comedi1","/dev/comedi2","/dev/comedi3"};

	if (ssGetErrorStatus(S) == NULL) {
		ComediDev_InUse[index]--;
		ComediDev_DIOInUse[index]--;
		if (!ComediDev_DIOInUse[index]) {
			comedi_unlock(dev, subdev);
		}
		if (!ComediDev_InUse[index]) {
			comedi_close(dev);
			printf("\nCOMEDI %s closed.\n\n", devname[index]);
			ComediDev[index] = NULL;
		}
	}
#endif
}

#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
