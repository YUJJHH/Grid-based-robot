/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor.h"
#include "pid.h"
#include "step.h"
#include "linear.h"

extern struct flag Finish_flag;

extern float input_position_X;
extern uint32_t save_X_avg;

extern uint8_t vt_finish_flag;

uint8_t position_start = 0;
uint8_t position_start_Y = 0;
uint8_t vt_start = 0;  //?��
uint8_t vt_start_Y = 0;  //?��

///////////////////////////////ADC

uint8_t k;
float battery_V;
float battery_V_avg;
float battery_V_sum;

extern uint16_t ADC3_value[3];
uint16_t ADC3_IN12;
uint16_t ADC3_IN13;
uint16_t ADC3_IN14;
///////////////////////////////////???���? ?��?���?
uint16_t tim7_flag = 0;

uint8_t tim5_flag = 0;
uint8_t tim6_flag = 0;
uint8_t tim14_flag = 0;
uint8_t tim13_flag = 0;

uint8_t tim11_flag = 0;

//////////////////////?��류센?��

float sensitivity = 0.255;

uint8_t rawVoltage_count = 0;
float rawVoltage_avg = 0;
float rawVoltage_sum = 0;

uint16_t readValue;

float rawVoltage;

float current_mA_X;
float current_A_X;
float current_A_floor_X;

////////////////////////////////    ?��?��모터
uint32_t step_pulse_count_tim13 = 0;
uint16_t step_pulse_count_tim11 = 0;

extern uint16_t shaft_step;
extern uint16_t shaft_pulse_cycle;

extern uint16_t ball_screw_step;
extern uint16_t ball_screw_pulse_cycle;

//////////////////////////////////
extern uint32_t encoder_count_x;
extern uint16_t encoder_count_y;

//extern double p_encoder;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
int exti2_test = 0;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern float RPM_X, RPM_Y;

extern int box_floor_test_pick ;
extern int box_floor_test_drop ;

extern struct Manual Manual_mode;
extern void Box_pickup(uint8_t floor);
extern void Box_dropoff(uint8_t floor);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int qweqwe = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern bool accel_flag;
extern void downpart_accel_step();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define EXTI_GPIO_PIN GPIO_PIN_8
#define EXTI_GPIO_PORT GPIOB

uint8_t fallingEdgeHandled = 0;
uint8_t risingEdgeHandled = 0;

extern int loop;

extern float go_time;
extern float back_time;







extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART3) {

		UART3_RX_PC_to_STM32(); // UART3: STM32 <-> PC
	}

	if (huart->Instance == UART4) {

		UART4_RX_ESP_to_STM32();  // UART4: esp <-> stm32
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	if (hadc->Instance == hadc3.Instance)

	{

		//GPIOB->ODR ^= 1 << 0;  //ok

		ADC3_IN14 = ADC3_value[2];

		battery_V = 16.059 * ADC3_IN14 / 67.8 * 27.667 + 2.2;

		battery_V_sum = battery_V + battery_V_sum;

		k++;
		if (k == 2000) {
			battery_V_avg = battery_V_sum / 2001;

			k = 0;
			battery_V_sum = 0;
		}

		ADC3_IN12 = ADC3_value[0];

		/*
		 readValue = ADC3_IN13;
		 rawVoltage = (float) readValue * 3.3 * 2 / 4095;
		 // If rawVoltage is not 2.5Volt, multiply by a factor.In my case it is 1.035
		 // This is due to tolerance in voltage divider resister & ADC accuracy
		 current =(rawVoltage - 2.5)/sensitivity;
		 */
		/*
		 readValue = ADC3_value[1];
		 //  readValue=readValue*0.6870; //
		 rawVoltage_sum = rawVoltage_sum + readValue;
		 rawVoltage_count++;
		 if (rawVoltage_count == 200) {
		 rawVoltage_avg = rawVoltage_sum / (200);
		 rawVoltage_sum = 0;
		 rawVoltage_count = 0;
		 }

		 //rawVoltage_avg=rawVoltage_avg*0.6887;
		 current_A_X = (rawVoltage_avg - (3256)) * 5 / 4095 / 0.253;

		 current_A_floor_X = floor(current_A_X * 100) / 100; // ?��?��?�� ?��?��짜리까�? ?��?��
		 current_mA_X = current_A_X * 1000;
		 */

		//printf("%5d %5d %5d \n\r",ADC3_value[0],ADC3_value[1],ADC3_value[2]);
	}

}

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BALL_MID_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(DOWN_LIMIT_SW_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */
	//	tim11_flag = 1;
	//step_pulse_count_tim11++;
//	GPIOB->ODR ^= 1 << 0;
//	z_axis_step_motor(ball_screw_step, ball_screw_pulse_cycle); //(?��?��,주기) ?��?��:400?�� ?��바�??
  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(USER_Btn_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

	/*	if ((TIM13->SR & 0x01) != RESET)	// CC1 interrupt flag
	 {
	 TIM13->SR &= ~0x01;	// CC1 Interrupt Claer


	 step_pulse_count_tim13++;

	 shaft_step_motor(shaft_step, shaft_pulse_cycle); //(?��?��,주기) ?��?��:400?�� ?��바�??

	 }
	 */
  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */
	tim5_flag = 1;

	/*      V T       */
	if (vt_start == 1) {
		vt_start_Y = 0;
		VT_control_X();

		GPIOB->ODR ^= 1 << 14;  //LD3
	}
	if (vt_start_Y == 1) {
		vt_start = 0;
		VT_control_Y();

		GPIOB->ODR ^= 1 << 14;  //LD3
	}

	linear_time_count_GO(go_time); //1�? ?���?
	linear_time_count_BACK(back_time);  //1.1�? ?���? 백래?�� 고려

	if(accel_flag==TRUE)
	{

				downpart_accel_step();


	}



  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

	tim6_flag = 1;

	encoder_count_x = TIM2->CNT;  //32bit timer
	encoder_count_y = TIM4->CNT;

	/*      PID      */
	if (position_start == 1) {

		position_pid_x();

		GPIOB->ODR ^= 1 << 14;  //LD3
	}

	if (position_start_Y == 1) {
		position_pid_y();

		GPIOB->ODR ^= 1 << 14;  //LD3
	}



//	if ((Manual_mode.box_pickup == TRUE) && (RPM_X == 0) && (RPM_Y == 0)) {
//						Box_pickup(box_floor_test_pick);
//					}
//
//					if ((Manual_mode.Box_dropoff == TRUE) && (RPM_X == 0) && (RPM_Y == 0)) {
//						Box_dropoff(box_floor_test_drop);
//
//					}



	/*	 ?? ?���? 발생 ??
	 if (tim6_flag == 1) {
	 distance_sensor();  //거리 ?��?��
	 tim6_flag = 0;
	 }
	 */
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM11) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

			GPIOB->ODR ^= 1 << 0;

			step_pulse_count_tim11++;
			z_axis_step_motor(ball_screw_step, ball_screw_pulse_cycle); //(?��?��,주기) ?��?��:400?�� ?��바�??
			//printf("%d , ,\n\r",step_pulse_count_tim11);
		}
	}

	if (htim->Instance == TIM13) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

			tim13_flag++;

			step_pulse_count_tim13++;
			shaft_step_motor(shaft_step, shaft_pulse_cycle); //(?��?��,주기) ?��?��:400?�� ?��바�??
		}

	}
}

uint32_t tim4_encoder_overflow = 0;
extern uint32_t encoder_count_x;
extern uint32_t save_X_IN;

extern uint8_t step_count_for_flag__;
extern int step_count__;
extern int step_count__min;
int count_test = 0;

extern int count_arr[1000];

extern uint8_t box_floor;
float test_downpart_step=8300;
float test_downpart_distance=53.0;


 uint32_t downpart_step_1_floor_test=7800; //?��?��, �? ?�� �?
 uint32_t downpart_step_2_floor_test;
 uint32_t downpart_step_3_floor_test;

extern float RPM_X, RPM_Y;



//extern int downpart_run;

extern int pickup_run;
extern int dropoff_run;
extern int ballscrew_down_run;

extern uint16_t tim9_flag;
extern uint16_t tim10_flag;
extern bool tim12_flag;
extern int ballscrew_up_run;
extern int ballscrew_down_run;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM9) { //0.1�?,  paser ?��?��?��?��?��
			tim9_flag++;

		}
	if (htim->Instance == TIM10) { //0.1�?,  paser ?��?��?��?��?��
			tim10_flag++;

		}
	if (htim->Instance == TIM12) { //0.1�?,  paser ?��?��?��?��?��
				tim12_flag = TRUE;

		}

	if (htim->Instance == TIM14) {  //tim14 : 0.01�? 마다 rpm 측정

		tim14_flag++;

		dc_motor_RPM();

		if (tim14_flag >= 10) { //0.1�? 마다 ESP�? ?��?��?�� ?��?��

			STM32_to_ESP();
			tim14_flag = 0;
		}

	}
	if (htim->Instance == TIM7) //?��?�� ?��?�� 측정 0.0001 =0.1ms
	{
		//if(RPM_Y )
			photo_sensor_1();

			photo_sensor_2();

	//	ball_limit_sw_max();  //?��??


		 // ?��?�� 보류
//		if( (dropoff_run == 1) && (pickup_run == 1) )  // ?��?��?��?��(?��?���?)
//		{
//			TIM3->CCR1=0;
//			TIM3->CCR2=0;
//		}

		if(ballscrew_up_run ==1 || ballscrew_down_run==1)
		{
			pickup_run=0;
			dropoff_run=0;
			HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
			HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		}


		if ( (RPM_X == 0) && (RPM_Y == 0))  //모터 ?���??�� ?��?���?
		{
			if(pickup_run == 0 && dropoff_run ==0 && ballscrew_down_run ==1)  //?��?��, ?��?�� ?��?�� ?��?�� ?��  볼스?���? ?��?���? ?��?��
			{
				ball_limit_sw_min();
			}

			down_sw_max();


		}
		else  //모터 ?��?���?�? FALSE
		{
			Finish_flag.downpart_high = FALSE;
			Finish_flag.downpart_low = FALSE;
			Finish_flag.ball_high = FALSE;
			Finish_flag.ball_low = FALSE;
			Finish_flag.linear_back = FALSE;
			Finish_flag.linear_go = FALSE;
		}





		//1�? if ?��?��
	//	if(box_floor==1)  	downpart_distance_step_dintance(downpart_step_1_floor_test, 0);  //?��중에 거리�? �??���?
	//	else if(box_floor==2)  downpart_distance_step_dintance(downpart_step_2_floor_test, 0);
	//	else if(box_floor==3)  downpart_distance_step_dintance(downpart_step_3_floor_test, 0);

		tim7_flag++;

		if (step_count_for_flag__ >= 1)  // ?��?��?��?�� �??�� �?�?
				{

			if (tim7_flag >= 100 && step_count__ >= step_count__min)  //0.1�? 마다
				{
				count_test = count_arr[step_count__--];
				TIM13->ARR = count_arr[step_count__--];
				tim7_flag = 0;
			}
			//step_count_for_flag=0;
		}
		if (step_count__ <= step_count__min) {
			step_count_for_flag__ = 0;
		}

//
//		if(accel_flag==TRUE)
//		{
//			downpart_accel_step();
//		}




	}
	//tim7_flag=1000:  5초에 100 카운?�� 감소  ,    200:  1초에 100카운?��

	if (htim->Instance == TIM4) {  //tim4 : y�? ?��코더모드

//		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == 0)  //y�? 방향 go
//		{
//			tim4_encoder_overflow += 0x10000; // ?��?�� 16비트 증�? (?��버플로우 발생 ?��)
//		}
//		else
//		{
//			tim4_encoder_overflow -= 0x10000; // ?��?�� 16비트 감소 (?��버플로우 발생 ?��)
//		}
	}

}

int pc6_sw_test=0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //x stop
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);  //y stop
	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //리니?�� ?���?
	if ((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0) == 1)
			&& (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1) && (RPM_X == 0)
			&& (RPM_Y == 0) && (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4) == 1)) {
//		if(GPIO_Pin == GPIO_PIN_0)  //ball MAX
//		{
//			 HAL_TIM_OC_Stop_IT(&htim11,TIM_CHANNEL_1);  // 볼스?���? ?��?��모터 ???���? ?���?
//			 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //?���?
//			 Finish_flag.ball_high = TRUE;
//
//		}
//		if(GPIO_Pin == GPIO_PIN_13)  //?��?? 버튼, ?��?���?  ball min
//		{
//			HAL_TIM_OC_Stop_IT(&htim11,TIM_CHANNEL_1);  // 볼스?���? ?��?��모터 ???���? ?���?
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //?���?
//			Finish_flag.ball_low = TRUE;
//
//		}
//		if (GPIO_Pin == GPIO_PIN_2)  // ball min
//		{
//			if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2) == GPIO_PIN_RESET) { //
//				HAL_TIM_OC_Stop_IT(&htim13, TIM_CHANNEL_1);  // 볼스?���? ?��?��모터 ???���? ?���?
//				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //?���?
//				Finish_flag.ball_low = TRUE;
//				exti2_test++;
//
//				printf("%d\n\r", exti2_test);
//			}
//			//else {
//		//		Finish_flag.ball_low = FALSE;
//			//}
//		}
//		if (GPIO_Pin == GPIO_PIN_3)  // dawn part
//		{
//			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) { //
//				HAL_TIM_OC_Stop_IT(&htim13, TIM_CHANNEL_1);  // 볼스?���? ?��?��모터 ???���? ?���?
//				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //?���?
//				Finish_flag.downpart_high = TRUE;
//				//exti2_test++;
//
//				printf("%d\n\r", exti2_test);
//			} else {
//				Finish_flag.downpart_high = FALSE;
//			}
//		}
	}


			if (GPIO_Pin == GPIO_PIN_6)  // dawn part
			{
				pc6_sw_test++;
			}

}
//HAL_GetTick()

/* USER CODE END 1 */
