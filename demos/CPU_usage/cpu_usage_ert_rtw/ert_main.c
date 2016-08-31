/*
 * File: ert_main.c
 *
 * Real-Time Workshop code generated for Simulink model cpu_usage.
 *
 * Model version                        : 1.18
 * Real-Time Workshop file version      : 7.6.2  (R2010bSP2)  02-Mar-2012
 * Real-Time Workshop file generated on : Fri Apr 22 17:15:32 2016
 * TLC version                          : 7.6 (Jul 13 2010)
 * C/C++ source code generated on       : Fri Apr 22 17:15:33 2016
 *
 * Target selection: ert_rpi.tlc
 * Embedded hardware selection: ARM Compatible->ARM 11
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#include "cpu_usage.h"

struct rpi_periodic_info
{
  int_T timer_fd;
  uint_T wakeups_missed;
};

uint_T rpi_cpu_usage = 0;
static boolean_T OverrunFlag = 0;
static int_T rpi_make_periodic(uint_T period, struct rpi_periodic_info *info)
{
  int_T ret;
  uint_T ns;
  uint_T sec;
  int_T fd;
  struct itimerspec itval;

  /* Create the timer */
  fd = timerfd_create (CLOCK_MONOTONIC, 0);
  info->wakeups_missed = 0;
  info->timer_fd = fd;
  if (fd == -1)
    return fd;

  /* Make the timer periodic */
  sec = period/1000000;
  ns = (period - (sec * 1000000)) * 1000;
  itval.it_interval.tv_sec = sec;
  itval.it_interval.tv_nsec = ns;
  itval.it_value.tv_sec = sec;
  itval.it_value.tv_nsec = ns;
  ret = timerfd_settime (fd, 0, &itval, NULL);
  return ret;
}

static int_T rpi_wait_period(struct rpi_periodic_info *info)
{
  unsigned long long missed;
  int_T ret;

  /*
     Wait for the next timer event.
     If we have missed any the
     number is written to "missed"
   */
  ret = read(info->timer_fd, &missed, sizeof (missed));
  if (ret == -1) {
    rtmSetErrorStatus(cpu_usage_M, "timer read error");
    fprintf(stderr,"** timer read error **\n");
    return -1;
  }

  if (missed > 1) {
    info->wakeups_missed += (int)(missed-1);
    fprintf(stderr,"** clock tic missed (total=%d) **\n",info->wakeups_missed);
    return -2;
  }

  return 0;
}

void rt_OneStep(void)
{
  /* Check for overun */
  if (OverrunFlag++) {
    rtmSetErrorStatus(cpu_usage_M, "Overrun");
    return;
  }

  cpu_usage_step();

  /* Get model outputs here */
  OverrunFlag--;
}

int_T main(int_T argc, const char_T *argv[])
{
  struct rpi_periodic_info info;
  struct timespec tmspc_before, tmspc_after;

  /* External mode */
  rtERTExtModeParseArgs(argc, argv);

  /* Initialize model */
  cpu_usage_initialize(1);
  rpi_make_periodic((uint_T)(0.005*1000000.0), &info);

  /* Associate rt_OneStep() with a timer that executes at the base rate of the model */

  /* Main loop : the rate is clocked by "rpi_wait_period" */
  while ((rtmGetErrorStatus(cpu_usage_M) == (NULL)) && !rtmGetStopRequested
         (cpu_usage_M)) {
    rpi_wait_period (&info);
    clock_gettime( CLOCK_MONOTONIC, &tmspc_before );
    rt_OneStep();
    clock_gettime( CLOCK_MONOTONIC, &tmspc_after );
    rpi_cpu_usage = (uint_T)( ( (real_T)( tmspc_after.tv_sec -
      tmspc_before.tv_sec )
      + (real_T)( tmspc_after.tv_nsec - tmspc_before.tv_nsec ) * 1e-9 )
      / (real_T)0.005 * 100.0 );
  }

  /* External mode */
  rtExtModeShutdown(1);
  cpu_usage_terminate();
  return 0;
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
