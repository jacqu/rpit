/* $Revision: 1.68.4.12 $
 * Copyright 1994-2005 The MathWorks, Inc.
 *
 * File    : grt_main.c
 *
 * Abstract:
 *      A Generic "Real-Time (single tasking or pseudo-multitasking,
 *      statically allocated data)" main that runs under most
 *      operating systems.
 *
 *      This file may be a useful starting point when targeting a new
 *      processor or microcontroller.
 *
 *
 * Compiler specified defines:
 *	RT              - Required.
 *      MODEL=modelname - Required.
 *	NUMST=#         - Required. Number of sample times.
 *	NCSTATES=#      - Required. Number of continuous states.
 *      TID01EQ=1 or 0  - Optional. Only define to 1 if sample time task
 *                        id's 0 and 1 have equal rates.
 *      MULTITASKING    - Optional. (use MT for a synonym).
 *	SAVEFILE        - Optional (non-quoted) name of .mat file to create.
 *			  Default is <MODEL>.mat
 *      BORLAND         - Required if using Borland C/C++
 */

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtwtypes.h"
#include "rtmodel.h"
#include "rt_sim.h"
#include "rt_nonfinite.h"

/* Signal Handler header */
#ifdef BORLAND
#include <signal.h>
#include <float.h>
#endif

#include "ext_work.h"
#include "ecrobot_interface.h"
#include "kernel.h"
#include "kernel_id.h"

DeclareTask(OSEK_Task_Startup);

/*=========*
 * Defines *
 *=========*/

#ifndef TRUE
#define FALSE (0)
#define TRUE  (1)
#endif

#ifndef EXIT_FAILURE
#define EXIT_FAILURE  1
#endif
#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS  0
#endif

#define QUOTE1(name) #name
#define QUOTE(name) QUOTE1(name)    /* need to expand name    */

#ifndef RT
# error "must define RT"
#endif

#ifndef MODEL
# error "must define MODEL"
#endif

#ifndef NUMST
# error "must define number of sample times, NUMST"
#endif

#ifndef NCSTATES
# error "must define NCSTATES"
#endif

#ifndef SAVEFILE
# define MATFILE2(file) #file ".mat"
# define MATFILE1(file) MATFILE2(file)
# define MATFILE MATFILE1(MODEL)
#else
# define MATFILE QUOTE(SAVEFILE)
#endif

#define RUN_FOREVER -1.0

#define EXPAND_CONCAT(name1,name2) name1 ## name2
#define CONCAT(name1,name2) EXPAND_CONCAT(name1,name2)
#define RT_MODEL            CONCAT(MODEL,_rtModel)

/*====================*
 * External functions *
 *====================*/
#ifdef __cplusplus

extern "C" {

#endif

extern RT_MODEL *MODEL(void);

extern void MdlInitializeSizes(void);
extern void MdlInitializeSampleTimes(void);
extern void MdlStart(void);
extern void MdlOutputs(int_T tid);
extern void MdlUpdate(int_T tid);
extern void MdlTerminate(void);

#ifdef __cplusplus

}
#endif

#if NCSTATES > 0
#ifdef __cplusplus

extern "C" {

#endif
  extern void rt_ODECreateIntegrationData(RTWSolverInfo *si);
  extern void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
#ifdef __cplusplus

}
#endif

# define rt_CreateIntegrationData(S) \
    rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(S));
# define rt_UpdateContinuousStates(S) \
    rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(S));
# else
# define rt_CreateIntegrationData(S)  \
      rtsiSetSolverName(rtmGetRTWSolverInfo(S),"FixedStepDiscrete");
# define rt_UpdateContinuousStates(S) /* Do Nothing */
#endif


/*==================================*
 * Global data local to this module *
 *==================================*/

static struct {
  int_T    stopExecutionFlag;
  int_T    isrOverrun;
  int_T    overrunFlags[NUMST];
  int_T    eventFlags[NUMST];
  const    char_T *errmsg;
} GBLbuf;


#ifdef EXT_MODE
#  define rtExtModeSingleTaskUpload(S)                          \
   {                                                            \
        int stIdx;                                              \
        rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));   \
        for (stIdx=0; stIdx<NUMST; stIdx++) {                   \
            if (rtmIsSampleHit(S, stIdx, 0 /*unused*/)) {       \
                rtExtModeUpload(stIdx,rtmGetTaskTime(S,stIdx)); \
            }                                                   \
        }                                                       \
   }
#else
#  define rtExtModeSingleTaskUpload(S) /* Do nothing */
#endif

/*=================*
 * Local functions *
 *=================*/

#if !defined(MULTITASKING)  /* SINGLETASKING */

/* Function: rtOneStep ========================================================
 *
 * Abstract:
 *      Perform one step of the model. This function is modeled such that
 *      it could be called from an interrupt service routine (ISR) with minor
 *      modifications.
 */

static void rt_OneStep(RT_MODEL *S)
{
    real_T tnext;

    /***********************************************
     * Check and see if base step time is too fast *
     ***********************************************/
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    /***********************************************
     * Check and see if error status has been set  *
     ***********************************************/
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    /* enable interrupts here */

    /*
     * In a multi-tasking environment, this would be removed from the base rate
     * and called as a "background" task.
     */
    /*rtExtModeOneStep(rtmGetRTWExtModeInfo(S),
                     rtmGetNumSampleTimes(S),
                     (boolean_T *)&rtmGetStopRequested(S));*/

    tnext = rt_SimGetNextSampleHit();
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S),tnext);

    MdlOutputs(0);

    //rtExtModeSingleTaskUpload(S);

    //GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
    //                                    rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    //rt_UpdateSigLogVars(rtmGetRTWLogInfo(S), rtmGetTPtr(S));

    MdlUpdate(0);
    rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(S),
                                       rtmGetTimingData(S),
                                       rtmGetSampleHitPtr(S),
                                       rtmGetTPtr(S));

    if (rtmGetSampleTime(S,0) == CONTINUOUS_SAMPLE_TIME) {
        rt_UpdateContinuousStates(S);
    }

    GBLbuf.isrOverrun--;

    rtExtModeCheckEndTrigger();

} /* end rtOneStep */

#else /* MULTITASKING */

# if TID01EQ == 1
#  define FIRST_TID 1
# else
#  define FIRST_TID 0
# endif

/* Function: rtOneStep ========================================================
 *
 * Abstract:
 *      Perform one step of the model. This function is modeled such that
 *      it could be called from an interrupt service routine (ISR) with minor
 *      modifications.
 *
 *      This routine is modeled for use in a multitasking environment and
 *	therefore needs to be fully re-entrant when it is called from an
 *	interrupt service routine.
 *
 * Note:
 *      Error checking is provided which will only be used if this routine
 *      is attached to an interrupt.
 *
 */
static void rt_OneStep(RT_MODEL *S)
{
    int_T  i;
    real_T tnext;
    int_T  *sampleHit = rtmGetSampleHitPtr(S);

    /***********************************************
     * Check and see if base step time is too fast *
     ***********************************************/
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    /***********************************************
     * Check and see if error status has been set  *
     ***********************************************/
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }
    /* enable interrupts here */

    /*
     * In a multi-tasking environment, this would be removed from the base rate
     * and called as a "background" task.
     */
    rtExtModeOneStep(rtmGetRTWExtModeInfo(S),
                     rtmGetNumSampleTimes(S),
                     (boolean_T *)&rtmGetStopRequested(S));

    /***********************************************
     * Update discrete events                      *
     ***********************************************/
    tnext = rt_SimUpdateDiscreteEvents(rtmGetNumSampleTimes(S),
                                       rtmGetTimingData(S),
                                       rtmGetSampleHitPtr(S),
                                       rtmGetPerTaskSampleHitsPtr(S));
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S),tnext);
    for (i=FIRST_TID+1; i < NUMST; i++) {
        if (sampleHit[i] && GBLbuf.eventFlags[i]++) {
            GBLbuf.isrOverrun--; 
            GBLbuf.overrunFlags[i]++;    /* Are we sampling too fast for */
            GBLbuf.stopExecutionFlag=1;  /*   sample time "i"?           */
            return;
        }
    }
    /*******************************************
     * Step the model for the base sample time *
     *******************************************/
    MdlOutputs(FIRST_TID);

    rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));
    rtExtModeUpload(FIRST_TID,rtmGetTaskTime(S, FIRST_TID));

    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    rt_UpdateSigLogVars(rtmGetRTWLogInfo(S), rtmGetTPtr(S));

    MdlUpdate(FIRST_TID);

    if (rtmGetSampleTime(S,0) == CONTINUOUS_SAMPLE_TIME) {
        rt_UpdateContinuousStates(S);
    }
     else {
        rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S), 
                                     rtmGetTimingData(S), 0);
    }

#if FIRST_TID == 1
    rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S), 
                                 rtmGetTimingData(S),1);
#endif


    /************************************************************************
     * Model step complete for base sample time, now it is okay to          *
     * re-interrupt this ISR.                                               *
     ************************************************************************/

    GBLbuf.isrOverrun--;


    /*********************************************
     * Step the model for any other sample times *
     *********************************************/
    for (i=FIRST_TID+1; i<NUMST; i++) {
        /* If task "i" is running, don't run any lower priority task */
        if (GBLbuf.overrunFlags[i]) return;

        if (GBLbuf.eventFlags[i]) {
            GBLbuf.overrunFlags[i]++;

            MdlOutputs(i);
 
            rtExtModeUpload(i, rtmGetTaskTime(S,i));

            MdlUpdate(i);

            rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S), 
                                         rtmGetTimingData(S),i);

            /* Indicate task complete for sample time "i" */
            GBLbuf.overrunFlags[i]--;
            GBLbuf.eventFlags[i]--;
        }
    }

    rtExtModeCheckEndTrigger();

} /* end rtOneStep */

#endif /* MULTITASKING */

/*===================*
 * Visible functions *
 *===================*/

static int_T stepTime, clockTime;

/* Function: main =============================================================
 *
 * Abstract:
 *      Execute model on a generic target such as a workstation.
 */
TASK(OSEK_Task_Startup)
{
	RT_MODEL   *S;
    const char *status;
    real_T     finaltime = -2.0;

    /****************************
     * Initialize global memory *
     ****************************/
    (void)memset(&GBLbuf, 0, sizeof(GBLbuf));

    /************************
     * Initialize the model *
     ************************/
    rt_InitInfAndNaN(sizeof(real_T));

    S = MODEL();
    if (rtmGetErrorStatus(S) != NULL) {
        return;
    }
    if (finaltime >= 0.0 || finaltime == RUN_FOREVER) rtmSetTFinal(S,finaltime);

    MdlInitializeSizes();
    MdlInitializeSampleTimes();
    
    status = rt_SimInitTimingEngine(rtmGetNumSampleTimes(S),
                                    rtmGetStepSize(S),
                                    rtmGetSampleTimePtr(S),
                                    rtmGetOffsetTimePtr(S),
                                    rtmGetSampleHitPtr(S),
                                    rtmGetSampleTimeTaskIDPtr(S),
                                    rtmGetTStart(S),
                                    &rtmGetSimTimeStep(S),
                                    &rtmGetTimingData(S));

    if (status != NULL) {
        return;
    }

    rt_CreateIntegrationData(S);

    if (GBLbuf.errmsg != NULL) {
        return;
    }

    rtExtModeCheckInit(rtmGetNumSampleTimes(S));
    rtExtModeWaitForStartPkt(rtmGetRTWExtModeInfo(S),
                             rtmGetNumSampleTimes(S),
                             (boolean_T *)&rtmGetStopRequested(S));

    MdlStart();
    if (rtmGetErrorStatus(S) != NULL) {
      GBLbuf.stopExecutionFlag = 1;
    }

    /*************************************************************************
     * Execute the model.  You may attach rtOneStep to an ISR, if so replace *
     * the call to rtOneStep (below) with a call to a background task        *
     * application.                                                          *
     *************************************************************************/

	stepTime = rtmGetStepSize(S)*1000;
	clockTime = systick_get_ms();

    while (!GBLbuf.stopExecutionFlag &&
           (rtmGetTFinal(S) == RUN_FOREVER ||
            rtmGetTFinal(S)-rtmGetT(S) > rtmGetT(S)*DBL_EPSILON)) {

        rtExtModePauseIfNeeded(rtmGetRTWExtModeInfo(S),
                               rtmGetNumSampleTimes(S),
                               (boolean_T *)&rtmGetStopRequested(S));

		while(systick_get_ms() < clockTime) {}
		clockTime += stepTime;
		ecrobot_status_monitor("nxtOSEK Blockset");
        if (rtmGetStopRequested(S)) break;
		rt_OneStep(S);
    }

	rtExtModeShutdown(rtmGetNumSampleTimes(S));

	if (!GBLbuf.stopExecutionFlag && !rtmGetStopRequested(S)) {
        /* Execute model last time step */
        rt_OneStep(S);
    }	

    MdlTerminate();
	ecrobot_status_monitor("Finished.");
	TerminateTask();
} /* end main */

/* EOF: grt_main.c */
