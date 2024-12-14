/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: downpart_accel.c
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

#include "downpart_accel.h"
#include <math.h>
#include "rtwtypes.h"
#include "downpart_accel_private.h"

#include "stm32f4xx_hal.h"
extern TIM_HandleTypeDef htim13;
/* External outputs (root outports fed by signals with default storage) */
ExtY_downpart_accel_T downpart_accel_Y;

/* Real-time model */
static RT_MODEL_downpart_accel_T downpart_accel_M_;
RT_MODEL_downpart_accel_T *const downpart_accel_M = &downpart_accel_M_;

/* Model step function */
int downpart_accel_step_max=500.0;
int downpart_accel_step_min=300.0;
real_T rtb_Clock;
void downpart_accel_step(void) {


	/* Clock: '<Root>/Clock' */
	rtb_Clock = downpart_accel_M->Timing.t[0];

	/* MATLAB Function: '<Root>/MATLAB Function' */
	if (rtb_Clock < 25) {
		/* Outport: '<Root>/accel_wave' */
		downpart_accel_Y.accel_wave = (downpart_accel_step_min-downpart_accel_step_max)/ (exp((rtb_Clock / 30 - 0.5) * -10.0) + 1.0) + downpart_accel_step_max;

		TIM13->ARR = downpart_accel_Y.accel_wave;
	} else {
		/* Outport: '<Root>/accel_wave' */
		downpart_accel_Y.accel_wave = downpart_accel_step_min;
		TIM13->ARR = downpart_accel_Y.accel_wave;
	}

	/*


	  % 파라미터 설정
	    A = 500; % 초기 값
	    B = 300; % 최종 값
	    D = 2.5; % 감속 시간 (초)

	    % 시간 기준으로 신호 생성
	    if t < D
	        % 더 부드러운 S-curve (sigmoid) 함수로 부드러운 감소
	        k = 10; % 조정 가능한 기울기 파라미터, 낮을수록 더 부드러움
	        y = A + (B - A) / (1 + exp(-k * (t / D - 0.5)));
	    else
	        y = B; % 최종 값 유지
	*/






	/* End of MATLAB Function: '<Root>/MATLAB Function' */

	/* Update absolute time for base rate */
	/* The "clockTick0" counts the number of times the code of this task has
	 * been executed. The absolute time is the multiplication of "clockTick0"
	 * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
	 * overflow during the application lifespan selected.
	 */
	downpart_accel_M->Timing.t[0] =
			((time_T) (++downpart_accel_M->Timing.clockTick0))
					* downpart_accel_M->Timing.stepSize0;

	{
		/* Update absolute timer for sample time: [0.2s, 0.0s] */
		/* The "clockTick1" counts the number of times the code of this task has
		 * been executed. The resolution of this integer timer is 0.2, which is the step size
		 * of the task. Size of "clockTick1" ensures timer will not overflow during the
		 * application lifespan selected.
		 */
		downpart_accel_M->Timing.clockTick1++;
	}
}

/* Model initialize function */
void downpart_accel_initialize(void) {
	/* Registration code */
	{
		/* Setup solver object */
		rtsiSetSimTimeStepPtr(&downpart_accel_M->solverInfo,
				&downpart_accel_M->Timing.simTimeStep);
		rtsiSetTPtr(&downpart_accel_M->solverInfo,
				&rtmGetTPtr(downpart_accel_M));
		rtsiSetStepSizePtr(&downpart_accel_M->solverInfo,
				&downpart_accel_M->Timing.stepSize0);
		rtsiSetErrorStatusPtr(&downpart_accel_M->solverInfo,
				(&rtmGetErrorStatus(downpart_accel_M)));
		rtsiSetRTModelPtr(&downpart_accel_M->solverInfo, downpart_accel_M);
	}

	rtsiSetSimTimeStep(&downpart_accel_M->solverInfo, MAJOR_TIME_STEP);
	rtsiSetIsMinorTimeStepWithModeChange(&downpart_accel_M->solverInfo, false);
	rtsiSetIsContModeFrozen(&downpart_accel_M->solverInfo, false);
	rtsiSetSolverName(&downpart_accel_M->solverInfo, "FixedStepDiscrete");
	rtmSetTPtr(downpart_accel_M, &downpart_accel_M->Timing.tArray[0]);
	downpart_accel_M->Timing.stepSize0 = 0.2;
}

/* Model terminate function */
void downpart_accel_terminate(void) {
	/* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
