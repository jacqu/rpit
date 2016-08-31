/*
 * File: Polaris_demo.h
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

#ifndef RTW_HEADER_Polaris_demo_h_
#define RTW_HEADER_Polaris_demo_h_
#ifndef Polaris_demo_COMMON_INCLUDES_
# define Polaris_demo_COMMON_INCLUDES_
#include <float.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "dt_info.h"
#include "ext_work.h"
#endif                                 /* Polaris_demo_COMMON_INCLUDES_ */

#include "Polaris_demo_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T SFunctionBuilder;             /* '<S1>/S-Function Builder' */
  real_T Polaris_o1;                   /* '<S2>/Polaris' */
  real_T Polaris_o2;                   /* '<S2>/Polaris' */
  real_T Polaris_o3;                   /* '<S2>/Polaris' */
  real_T Polaris_o4;                   /* '<S2>/Polaris' */
  real_T Polaris_o5;                   /* '<S2>/Polaris' */
  real_T Polaris_o6;                   /* '<S2>/Polaris' */
  real_T Polaris_o7;                   /* '<S2>/Polaris' */
  real_T Polaris_o8;                   /* '<S2>/Polaris' */
  real_T Polaris_o9;                   /* '<S2>/Polaris' */
} BlockIO_Polaris_demo;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  struct {
    void *LoggedData;
  } CPU_PWORK;                         /* '<Root>/CPU%' */

  struct {
    void *LoggedData;
  } Quaternions_PWORK;                 /* '<Root>/Quaternions' */

  struct {
    void *LoggedData;
  } Translation_PWORK;                 /* '<Root>/Translation' */
} D_Work_Polaris_demo;

/* Parameters (auto storage) */
struct Parameters_Polaris_demo_ {
  real_T SFunctionBuilder_P1;          /* Expression: rpi_mask_Ts
                                        * Referenced by: '<S1>/S-Function Builder'
                                        */
  real_T Polaris_P1;                   /* Expression: rpi_mask_Ts
                                        * Referenced by: '<S2>/Polaris'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_Polaris_demo {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (auto storage) */
extern Parameters_Polaris_demo Polaris_demo_P;

/* Block signals (auto storage) */
extern BlockIO_Polaris_demo Polaris_demo_B;

/* Block states (auto storage) */
extern D_Work_Polaris_demo Polaris_demo_DWork;

/* Model entry point functions */
extern void Polaris_demo_initialize(boolean_T firstTime);
extern void Polaris_demo_step(void);
extern void Polaris_demo_terminate(void);

/* Real-time Model object */
extern struct RT_MODEL_Polaris_demo *Polaris_demo_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : Polaris_demo
 * '<S1>'   : Polaris_demo/CPU usage
 * '<S2>'   : Polaris_demo/Polaris
 */
#endif                                 /* RTW_HEADER_Polaris_demo_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
