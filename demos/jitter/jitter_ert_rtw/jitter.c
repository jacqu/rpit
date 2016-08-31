/*
 * File: jitter.c
 *
 * Real-Time Workshop code generated for Simulink model jitter.
 *
 * Model version                        : 1.11
 * Real-Time Workshop file version      : 7.6.2  (R2010bSP2)  02-Mar-2012
 * Real-Time Workshop file generated on : Fri Apr 22 17:18:52 2016
 * TLC version                          : 7.6 (Jul 13 2010)
 * C/C++ source code generated on       : Fri Apr 22 17:18:52 2016
 *
 * Target selection: ert_rpi.tlc
 * Embedded hardware selection: ARM Compatible->ARM 11
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "jitter.h"
#include "jitter_private.h"
#include <stdio.h>
#include "jitter_dt.h"

/* Block signals (auto storage) */
BlockIO_jitter jitter_B;

/* Block states (auto storage) */
D_Work_jitter jitter_DWork;

/* Real-time model */
RT_MODEL_jitter jitter_M_;
RT_MODEL_jitter *jitter_M = &jitter_M_;

/* Model step function */
void jitter_step(void)
{
  /* local block i/o variables */
  real_T rtb_DigitalClock;

  {
    boolean_T rtmStopReq = FALSE;
    rtExtModePauseIfNeeded(jitter_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(jitter_M, TRUE);
    }

    if (rtmGetStopRequested(jitter_M) == TRUE) {
      rtmSetErrorStatus(jitter_M, "Simulation finished");
      return;
    }
  }

  /* S-Function (rpi_sfun_time): '<S1>/RPI time' */
  rpi_sfun_time_Outputs_wrapper( &jitter_B.RPItime, &jitter_P.RPItime_P1, 1);

  /* DigitalClock: '<Root>/Digital Clock' */
  rtb_DigitalClock = jitter_M->Timing.taskTime0;

  /* Sum: '<Root>/Sum' */
  jitter_B.Sum = jitter_B.RPItime - rtb_DigitalClock;

  /* external mode */
  {
    boolean_T rtmStopReq = FALSE;
    rtExtModeOneStep(jitter_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(jitter_M, TRUE);
    }
  }

  rtExtModeUploadCheckTrigger(1);

  {                                    /* Sample time: [0.01s, 0.0s] */
    rtExtModeUpload(0, jitter_M->Timing.taskTime0);
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.01s, 0.0s] */
    if ((rtmGetTFinal(jitter_M)!=-1) &&
        !((rtmGetTFinal(jitter_M)-jitter_M->Timing.taskTime0) >
          jitter_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(jitter_M, "Simulation finished");
    }

    if (rtmGetStopRequested(jitter_M)) {
      rtmSetErrorStatus(jitter_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  jitter_M->Timing.taskTime0 =
    (++jitter_M->Timing.clockTick0) * jitter_M->Timing.stepSize0;
}

/* Model initialize function */
void jitter_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)jitter_M, 0,
                sizeof(RT_MODEL_jitter));
  rtmSetTFinal(jitter_M, -1);
  jitter_M->Timing.stepSize0 = 0.01;

  /* external mode info */
  jitter_M->Sizes.checksums[0] = (843035508U);
  jitter_M->Sizes.checksums[1] = (3641226404U);
  jitter_M->Sizes.checksums[2] = (2522151300U);
  jitter_M->Sizes.checksums[3] = (3487289261U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    jitter_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(jitter_M->extModeInfo,
      &jitter_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(jitter_M->extModeInfo, jitter_M->Sizes.checksums);
    rteiSetTPtr(jitter_M->extModeInfo, rtmGetTPtr(jitter_M));
  }

  /* block I/O */
  (void) memset(((void *) &jitter_B), 0,
                sizeof(BlockIO_jitter));

  /* states (dwork) */
  (void) memset((void *)&jitter_DWork, 0,
                sizeof(D_Work_jitter));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    jitter_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }

  /* External mode */
  rtERTExtModeSetTFinal(&rtmGetTFinal(jitter_M));
  rtExtModeCheckInit(1);

  {
    boolean_T rtmStopReq = FALSE;
    rtExtModeWaitForStartPkt(jitter_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(jitter_M, TRUE);
    }
  }

  (void)printf("\n** starting the model **\n");
}

/* Model terminate function */
void jitter_terminate(void)
{
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
