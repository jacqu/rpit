/*
 * File: cpu_usage.c
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

#include "cpu_usage.h"
#include "cpu_usage_private.h"
#include <stdio.h>
#include "cpu_usage_dt.h"

/* Block signals (auto storage) */
BlockIO_cpu_usage cpu_usage_B;

/* Block states (auto storage) */
D_Work_cpu_usage cpu_usage_DWork;

/* Real-time model */
RT_MODEL_cpu_usage cpu_usage_M_;
RT_MODEL_cpu_usage *cpu_usage_M = &cpu_usage_M_;

/* Model step function */
void cpu_usage_step(void)
{
  {
    boolean_T rtmStopReq = FALSE;
    rtExtModePauseIfNeeded(cpu_usage_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(cpu_usage_M, TRUE);
    }

    if (rtmGetStopRequested(cpu_usage_M) == TRUE) {
      rtmSetErrorStatus(cpu_usage_M, "Simulation finished");
      return;
    }
  }

  /* S-Function (rpi_sfun_cpu): '<S1>/S-Function Builder' */
  rpi_sfun_cpu_Outputs_wrapper( &cpu_usage_B.SFunctionBuilder,
    &cpu_usage_P.SFunctionBuilder_P1, 1);

  /* external mode */
  {
    boolean_T rtmStopReq = FALSE;
    rtExtModeOneStep(cpu_usage_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(cpu_usage_M, TRUE);
    }
  }

  rtExtModeUploadCheckTrigger(1);

  {                                    /* Sample time: [0.005s, 0.0s] */
    rtExtModeUpload(0, cpu_usage_M->Timing.taskTime0);
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.005s, 0.0s] */
    if ((rtmGetTFinal(cpu_usage_M)!=-1) &&
        !((rtmGetTFinal(cpu_usage_M)-cpu_usage_M->Timing.taskTime0) >
          cpu_usage_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(cpu_usage_M, "Simulation finished");
    }

    if (rtmGetStopRequested(cpu_usage_M)) {
      rtmSetErrorStatus(cpu_usage_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  cpu_usage_M->Timing.taskTime0 =
    (++cpu_usage_M->Timing.clockTick0) * cpu_usage_M->Timing.stepSize0;
}

/* Model initialize function */
void cpu_usage_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)cpu_usage_M, 0,
                sizeof(RT_MODEL_cpu_usage));
  rtmSetTFinal(cpu_usage_M, -1);
  cpu_usage_M->Timing.stepSize0 = 0.005;

  /* external mode info */
  cpu_usage_M->Sizes.checksums[0] = (3904111941U);
  cpu_usage_M->Sizes.checksums[1] = (440812128U);
  cpu_usage_M->Sizes.checksums[2] = (4164639564U);
  cpu_usage_M->Sizes.checksums[3] = (1866868293U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    cpu_usage_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(cpu_usage_M->extModeInfo,
      &cpu_usage_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(cpu_usage_M->extModeInfo, cpu_usage_M->Sizes.checksums);
    rteiSetTPtr(cpu_usage_M->extModeInfo, rtmGetTPtr(cpu_usage_M));
  }

  /* block I/O */
  (void) memset(((void *) &cpu_usage_B), 0,
                sizeof(BlockIO_cpu_usage));

  /* states (dwork) */
  (void) memset((void *)&cpu_usage_DWork, 0,
                sizeof(D_Work_cpu_usage));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    cpu_usage_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }

  /* External mode */
  rtERTExtModeSetTFinal(&rtmGetTFinal(cpu_usage_M));
  rtExtModeCheckInit(1);

  {
    boolean_T rtmStopReq = FALSE;
    rtExtModeWaitForStartPkt(cpu_usage_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(cpu_usage_M, TRUE);
    }
  }

  (void)printf("\n** starting the model **\n");
}

/* Model terminate function */
void cpu_usage_terminate(void)
{
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
