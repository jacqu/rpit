/*
 * File: Polaris_demo.c
 *
 * Real-Time Workshop code generated for Simulink model Polaris_demo.
 *
 * Model version                        : 1.9
 * Real-Time Workshop file version      : 7.6.2  (R2010bSP2)  02-Mar-2012
 * Real-Time Workshop file generated on : Mon Feb 02 16:10:07 2015
 * TLC version                          : 7.6 (Jul 13 2010)
 * C/C++ source code generated on       : Mon Feb 02 16:10:07 2015
 *
 * Target selection: ert_rpi.tlc
 * Embedded hardware selection: ARM Compatible->ARM 11
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Polaris_demo.h"
#include "Polaris_demo_private.h"
#include <stdio.h>
#include "Polaris_demo_dt.h"

/* Block signals (auto storage) */
BlockIO_Polaris_demo Polaris_demo_B;

/* Block states (auto storage) */
D_Work_Polaris_demo Polaris_demo_DWork;

/* Real-time model */
RT_MODEL_Polaris_demo Polaris_demo_M_;
RT_MODEL_Polaris_demo *Polaris_demo_M = &Polaris_demo_M_;

/* Model step function */
void Polaris_demo_step(void)
{
  {
    boolean_T rtmStopReq = FALSE;
    rtExtModePauseIfNeeded(Polaris_demo_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(Polaris_demo_M, TRUE);
    }

    if (rtmGetStopRequested(Polaris_demo_M) == TRUE) {
      rtmSetErrorStatus(Polaris_demo_M, "Simulation finished");
      return;
    }
  }

  /* S-Function (rpi_sfun_cpu): '<S1>/S-Function Builder' */
  rpi_sfun_cpu_Outputs_wrapper( &Polaris_demo_B.SFunctionBuilder,
    &Polaris_demo_P.SFunctionBuilder_P1, 1);

  /* S-Function (rpi_sfun_polaris): '<S2>/Polaris' */
  rpi_sfun_polaris_Outputs_wrapper( &Polaris_demo_B.Polaris_o1,
    &Polaris_demo_B.Polaris_o2, &Polaris_demo_B.Polaris_o3,
    &Polaris_demo_B.Polaris_o4, &Polaris_demo_B.Polaris_o5,
    &Polaris_demo_B.Polaris_o6, &Polaris_demo_B.Polaris_o7,
    &Polaris_demo_B.Polaris_o8, &Polaris_demo_B.Polaris_o9,
    &Polaris_demo_P.Polaris_P1, 1);

  /* external mode */
  {
    boolean_T rtmStopReq = FALSE;
    rtExtModeOneStep(Polaris_demo_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(Polaris_demo_M, TRUE);
    }
  }

  rtExtModeUploadCheckTrigger(1);

  {                                    /* Sample time: [0.01s, 0.0s] */
    rtExtModeUpload(0, Polaris_demo_M->Timing.taskTime0);
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.01s, 0.0s] */
    if ((rtmGetTFinal(Polaris_demo_M)!=-1) &&
        !((rtmGetTFinal(Polaris_demo_M)-Polaris_demo_M->Timing.taskTime0) >
          Polaris_demo_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Polaris_demo_M, "Simulation finished");
    }

    if (rtmGetStopRequested(Polaris_demo_M)) {
      rtmSetErrorStatus(Polaris_demo_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  Polaris_demo_M->Timing.taskTime0 =
    (++Polaris_demo_M->Timing.clockTick0) * Polaris_demo_M->Timing.stepSize0;
}

/* Model initialize function */
void Polaris_demo_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)Polaris_demo_M, 0,
                sizeof(RT_MODEL_Polaris_demo));
  rtmSetTFinal(Polaris_demo_M, -1);
  Polaris_demo_M->Timing.stepSize0 = 0.01;

  /* external mode info */
  Polaris_demo_M->Sizes.checksums[0] = (3110987317U);
  Polaris_demo_M->Sizes.checksums[1] = (2347476052U);
  Polaris_demo_M->Sizes.checksums[2] = (191229939U);
  Polaris_demo_M->Sizes.checksums[3] = (494255935U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    Polaris_demo_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(Polaris_demo_M->extModeInfo,
      &Polaris_demo_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(Polaris_demo_M->extModeInfo,
                        Polaris_demo_M->Sizes.checksums);
    rteiSetTPtr(Polaris_demo_M->extModeInfo, rtmGetTPtr(Polaris_demo_M));
  }

  /* block I/O */
  (void) memset(((void *) &Polaris_demo_B), 0,
                sizeof(BlockIO_Polaris_demo));

  /* states (dwork) */
  (void) memset((void *)&Polaris_demo_DWork, 0,
                sizeof(D_Work_Polaris_demo));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    Polaris_demo_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }

  /* S-Function Block: <S2>/Polaris */
  rpitdd_server_init( );

  /* External mode */
  rtERTExtModeSetTFinal(&rtmGetTFinal(Polaris_demo_M));
  rtExtModeCheckInit(1);

  {
    boolean_T rtmStopReq = FALSE;
    rtExtModeWaitForStartPkt(Polaris_demo_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(Polaris_demo_M, TRUE);
    }
  }

  (void)printf("\n** starting the model **\n");
}

/* Model terminate function */
void Polaris_demo_terminate(void)
{
  /* S-Function Block: <S2>/Polaris */
  rpitdd_server_shutdown( );
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
