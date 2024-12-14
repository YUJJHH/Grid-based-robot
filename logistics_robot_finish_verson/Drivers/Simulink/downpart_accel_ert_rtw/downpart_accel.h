/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: downpart_accel.h
 *
 * Code generated for Simulink model 'downpart_accel'.
 *
 * Model version                  : 1.2
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Wed Jul  3 17:27:36 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef downpart_accel_h_
#define downpart_accel_h_
#ifndef downpart_accel_COMMON_INCLUDES_
#define downpart_accel_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                                 /* downpart_accel_COMMON_INCLUDES_ */

#include "downpart_accel_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T accel_wave;                   /* '<Root>/accel_wave' */
} ExtY_downpart_accel_T;

/* Real-time Model Data Structure */
struct tag_RTM_downpart_accel_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_downpart_accel_T downpart_accel_Y;

/* Model entry point functions */
extern void downpart_accel_initialize(void);
extern void downpart_accel_step(void);
extern void downpart_accel_terminate(void);

/* Real-time Model object */
extern RT_MODEL_downpart_accel_T *const downpart_accel_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope1' : Unused code path elimination
 */

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
 * '<Root>' : 'downpart_accel'
 * '<S1>'   : 'downpart_accel/MATLAB Function'
 */
#endif                                 /* downpart_accel_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
