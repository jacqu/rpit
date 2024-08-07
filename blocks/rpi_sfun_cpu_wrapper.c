/*
  *
  *   --- THIS FILE GENERATED BY S-FUNCTION BUILDER: 3.0 ---
  *
  *   This file is a wrapper S-function produced by the S-Function
  *   Builder which only recognizes certain fields.  Changes made
  *   outside these fields will be lost the next time the block is
  *   used to load, edit, and resave this file. This file will be overwritten
  *   by the S-function Builder block. If you want to edit this file by hand, 
  *   you must change it only in the area defined as:  
  *
  *        %%%-SFUNWIZ_wrapper_XXXXX_Changes_BEGIN 
  *            Your Changes go here
  *        %%%-SFUNWIZ_wrapper_XXXXXX_Changes_END
  *
  *   For better compatibility with the Real-Time Workshop, the
  *   "wrapper" S-function technique is used.  This is discussed
  *   in the Real-Time Workshop User's Manual in the Chapter titled,
  *   "Wrapper S-functions".
  *
  *   Created: Wed Jul  9 13:24:27 2014
  */


/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#if defined(MATLAB_MEX_FILE)
#include <math.h>
#else
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 
#define y_width 1
/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
#if !defined(MATLAB_MEX_FILE)
extern uint_T rpi_cpu_usage;
#endif
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output functions
 *
 */
void rpi_sfun_cpu_Outputs_wrapper(real_T *y0, 
                           const real_T  *rpi_Ts, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
#if defined(MATLAB_MEX_FILE)
  y0[0] = 0;
#else

  /* Basic checks on the sapling rate */

  if ( p_width0 > 1 )	{
    fprintf( stderr, "** Multiple rates not allowed in this block **\n" );
  }

  if ( *rpi_Ts < 0.001 )	{
    fprintf( stderr, "** Warning : sampling rates beyond 1000Hz may yield overruns  **\n" );
  }

  y0[0] = (real_T)rpi_cpu_usage;
#endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}
