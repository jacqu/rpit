/*
COPYRIGHT (C) 2007  Dan D. V. Bhanderi (dan@bhanderi.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.
*/

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <comedi.h>
#include <comedilib.h>

#define MAX_COMEDI_DEVICES        10	

void *ComediDev[MAX_COMEDI_DEVICES];
int ComediDev_InUse[MAX_COMEDI_DEVICES] = {0};
int ComediDev_AIInUse[MAX_COMEDI_DEVICES] = {0};
int ComediDev_AOInUse[MAX_COMEDI_DEVICES] = {0};
int ComediDev_DIOInUse[MAX_COMEDI_DEVICES] = {0};

char *lnx_comedi_get_board_name(void *dev, char *name)
{
#ifndef MATLAB_MEX_FILE
  void *p;  
  if ((p = comedi_get_board_name((void *)dev)) != 0) {
    strncpy(name, p, COMEDI_NAMELEN);
    return name;
  }
  return 0;
#endif
}

comedi_range *lnx_comedi_get_range(void *dev, unsigned int subdev, unsigned int chan, unsigned int range)
{
#ifndef MATLAB_MEX_FILE
  return comedi_get_range(dev, subdev, chan, range);
#endif
}
