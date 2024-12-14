/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "sensor.h"
#include "step.h"
#include "linear.h"
#include "pp.h"
#include "mpu6050.h"
#include "queue.h"
//?��뮬링?��
#include "rtwtypes.h"
#include "downpart_accel.h"
//

uint8_t key_value;

uint8_t data;

uint32_t encoder_count_x = 1000000;
uint16_t encoder_count_y = 30000;
extern uint32_t tim4_encoder_overflow;

uint16_t ADC3_value[3];

extern uint8_t tim6_flag;
extern uint8_t tim14_flag;
extern uint8_t tim5_flag;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint16_t shaft_pulse_cycle;
extern uint16_t ball_screw_pulse_cycle;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM13_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
bool accel_flag;

//struct {
//	bool target_2_1;
//	bool target_3_1;
//	bool target_1_2;
//	bool target_2_2;
//	bool target_3_2;
//
//	bool box_pickup;
//	bool Box_dropoff;
//} Manual_mode = { FALSE, FALSE, FALSE };
struct Manual Manual_mode = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};


struct flag Finish_flag = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, 0,
		0, FALSE, FALSE };

float targetS_X_GO = 53;
float targetS_X_BACK = 56.0;

float targetS_Y_GO = 49;
float targetS_Y_BACK = 48;

float targetS_X_GO2 = 98;
float targetS_X_BACK2 = 103;
//

float go_time = 1.25;
float back_time = 1.4;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool photo_X_go = FALSE;
bool photo_X_back = FALSE;
bool photo_Y_go = FALSE;
bool photo_Y_back = FALSE;

int pickup_run;
int dropoff_run;

extern int pathLength;

cell path[CELL_DOMAIN_MAX];

extern float maxV;
extern float fallingN;
extern float fallingN_;
extern float raisingN;
extern float staticN;
extern float orderN;
extern uint16_t i_v_t_count;

uint8_t target_1_2_flag = 0;
uint8_t target_2_1_flag = 0;
uint8_t target_3_1_flag = 0;
uint8_t target_2_2_flag = 0;
uint8_t target_3_2_flag = 0;
uint8_t box_pickup_flag = 0;
uint8_t box_dropoff_flag = 0;
void FALSE_Init() {
	Finish_flag.x_go = FALSE;
	Finish_flag.x_back = FALSE;
	Finish_flag.y_go = FALSE;
	Finish_flag.y_back = FALSE;
	Finish_flag.ball_high = FALSE;
	Finish_flag.ball_mid = FALSE;
	Finish_flag.ball_low = FALSE;
	Finish_flag.downpart_high = FALSE;
	Finish_flag.downpart_low = FALSE;
	Finish_flag.linear_go = FALSE;
	Finish_flag.linear_back = FALSE;

	target_1_2_flag = 0;
	target_2_1_flag = 0;
	target_3_1_flag = 0;
	target_2_2_flag = 0;
	target_3_2_flag = 0;
	box_pickup_flag = 0;
	box_dropoff_flag = 0;

	pickup_run=0;
	dropoff_run= 0;
	Manual_mode.box_pickup = FALSE;
	Manual_mode.Box_dropoff = FALSE;
}
uint32_t downpart_step_1_floor = 8100; //
uint32_t downpart_step_2_floor = 6600;
uint32_t downpart_step_3_floor = 4500;  //?��?��
uint8_t box_floor;

int box_floor_test_pick = 1;
int box_floor_test_drop = 1;

//박스 ?��?�� ?��?�� : ?��?��?��?�� ?���?, 리니?�� ?���?, ?��?��?��?�� ?��?��

uint8_t box_floor;  //?��?��?��?��?��
uint32_t downpart_step_floor;

void Box_pickup(uint8_t floor) {
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);


	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	pickup_run = 1;

	box_floor = floor;

	if (floor == 1)
		downpart_step_floor = downpart_step_1_floor;
	else if (floor == 2)
		downpart_step_floor = downpart_step_2_floor;
	else if (floor == 3)
		downpart_step_floor = downpart_step_3_floor;

//	if (Manual_mode.box_pickup == TRUE) {
	if (box_pickup_flag == 0) {

		down_part_DOWN_accel(downpart_step_floor);
		box_pickup_flag = 1;
	}

	if ((Finish_flag.downpart_low == TRUE) && (box_pickup_flag == 1)) {
		box_pickup_flag = 2;
		HAL_Delay(700 - 1);

		linear_motor_GO(go_time);
		Finish_flag.downpart_low = FALSE;
	}
	if ((Finish_flag.linear_go == TRUE) && (box_pickup_flag == 2)) {
		box_pickup_flag = 3;
		HAL_Delay(700 - 1);
		down_part_UP_accel(downpart_step_floor + 10000);
		Finish_flag.linear_go = FALSE;
	}
	if ((Finish_flag.downpart_high == TRUE) && (box_pickup_flag == 3)) {
		box_pickup_flag = 4;
		HAL_Delay(120 - 1);

		down_part_DOWN(130, shaft_pulse_cycle);
		Finish_flag.downpart_high = FALSE;
	}
	if(	Finish_flag.downpart_low == TRUE && box_pickup_flag ==4)
	{
		box_pickup_flag =5;
		HAL_Delay(700 - 1);
		Finish_flag.downpart_low = FALSE;
		Manual_mode.box_pickup = FALSE;
		pickup_run = 0;
	}
//	}
}

//박스 ?��?�� ?��?�� : ?��?��?��?�� ?���?, 리니?�� ?���?, ?��?��?��?�� ?��?��
void Box_dropoff(uint8_t floor) {
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;

	dropoff_run = 1;

	box_floor = floor;

	if (floor == 1)
		downpart_step_floor = downpart_step_1_floor;
	else if (floor == 2)
		downpart_step_floor = downpart_step_2_floor;
	else if (floor == 3)
		downpart_step_floor = downpart_step_3_floor;

	//if (Manual_mode.Box_dropoff == TRUE) {
	if (box_dropoff_flag == 0) {
		down_part_DOWN_accel(downpart_step_floor);
		box_dropoff_flag = 1;
	}

	if ((Finish_flag.downpart_low == TRUE) && (box_dropoff_flag == 1)) {
		box_dropoff_flag = 2;
		HAL_Delay(700 - 1);

		linear_motor_BACK(back_time);
		Finish_flag.downpart_low = FALSE;
	}
	if ((Finish_flag.linear_back == TRUE) && (box_dropoff_flag == 2)) {
		box_dropoff_flag = 3;
		HAL_Delay(200 - 1);
		down_part_UP_accel(downpart_step_floor + 10000);
		Finish_flag.linear_back = FALSE;
	}
	if ((Finish_flag.downpart_high == TRUE) && (box_dropoff_flag == 3)) {
		box_dropoff_flag = 4;
		HAL_Delay(120 - 1);

		down_part_DOWN(130, shaft_pulse_cycle);
		Finish_flag.downpart_high = FALSE;
	}
	if(Finish_flag.downpart_low == TRUE && (box_dropoff_flag == 4))
	{
		box_dropoff_flag =5;
		HAL_Delay(700 - 1);
		Finish_flag.downpart_low = FALSE;
		Manual_mode.Box_dropoff = FALSE;
		dropoff_run = 0;
	}


//	}
}

//y�? 1�? ?���?,  볼스?���? ?���?  ?��?��?��?�� ?��?���?					                                     			1              2           3                         4              5                       6
void Manual_Mode_Y1(float row, float col, uint8_t floor_pick,
		uint8_t floor_drop) { //볼스?���? ?��?�� 먼�? >> y ?���?, ?��?��?��?�� ?���?, 리니?�� ?���?, ?��?��?��?�� ?��?��, y ?���?, ?��?��?��?�� ?���?, 리니?�� ?���? ,?��?��?��?�� ?��?��, 볼스?���? ?��?��
//																	      -------------------------------        ------------------------------
	if (Manual_mode.target_1_2 == TRUE) {

		if (target_1_2_flag == 0) {

			z_axis_DOWN(400 * 6 * 2 + 500, ball_screw_pulse_cycle);
			target_1_2_flag = 1;
		}

		if ((Finish_flag.ball_low == TRUE) && (target_1_2_flag == 1)) {
			target_1_2_flag = 2;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_GO_Y(targetS_Y_GO);  //?��?���?

			Finish_flag.ball_low = FALSE;
		}
		if ((Finish_flag.y_go == TRUE) && (target_1_2_flag == 2)) {
			target_1_2_flag = 3;
			HAL_Delay(2100 - 1);

			Manual_mode.box_pickup = TRUE;
			box_floor_test_pick = floor_pick;
			Finish_flag.y_go = FALSE;
		}
		if ((Manual_mode.box_pickup == FALSE) && (target_1_2_flag == 3)) {

			target_1_2_flag = 4;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_BACK_Y(targetS_Y_BACK);

		}
		if ((Finish_flag.y_back == TRUE) && (target_1_2_flag == 4)) {
			target_1_2_flag = 5;
			HAL_Delay(700 - 1);
			//		down_part_DOWN_accel(downpart_step_1_floor);

			Manual_mode.Box_dropoff = TRUE;
			box_floor_test_drop = floor_drop;

			Finish_flag.y_back = FALSE;
		}

		if ((Manual_mode.Box_dropoff == FALSE) && (target_1_2_flag == 5)) {

			target_1_2_flag = 6;

			HAL_Delay(900 - 1);
			z_axis_UP(400 * 6 * 2, ball_screw_pulse_cycle);

			FALSE_Init();
			Manual_mode.target_1_2 = FALSE;
		}
	}
}

//(2,1)  (3,1)    ?��기는 볼스?���? ?��?��?��?��                         //1       2          3            4       5        6          7        8
void Manual_Mode_X1(float row, float col, uint8_t floor_pick,
		uint8_t floor_drop) { // x?���? , ?��?��?��?�� ?���?, 리니?�� ?���?, ?��?��?��?�� ?��?��, x?���?, ?��?��?��?�� ?���?, 리니?�� ?���?, ?��?��?��?�� ?��?�� ?��
	//				                 -----------------------------          ----------------------------
	if (Manual_mode.target_2_1 == TRUE) {

		if (target_2_1_flag == 0) {
			v_t_graph_DIR_GO_X(targetS_X_GO);  //?��?���?

			target_2_1_flag = 1;
		}

		if ((Finish_flag.x_go == TRUE) && (target_2_1_flag == 1)) {
			target_2_1_flag = 2;
			HAL_Delay(2100 - 1);

			Manual_mode.box_pickup = TRUE;
			box_floor_test_pick = floor_pick;
			Finish_flag.x_go = FALSE;

		}

		if ((Manual_mode.box_pickup == FALSE) && (target_2_1_flag == 2)) {

			target_2_1_flag = 3;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_BACK_X(targetS_X_BACK);

		}
		if ((Finish_flag.x_back == TRUE) && (target_2_1_flag == 3)) {
			target_2_1_flag = 4;
			HAL_Delay(700 - 1);

			Manual_mode.Box_dropoff = TRUE;
			box_floor_test_drop = floor_drop;

			Finish_flag.x_back = FALSE;
		}

		if ((Manual_mode.Box_dropoff == FALSE) && (target_2_1_flag == 4)) {
			target_2_1_flag = 5;

			FALSE_Init();
			Manual_mode.target_2_1 = FALSE;
		}
	}
}

//			 1                    2                   3              4
void Manual_Mode_X2(float row, float col, uint8_t floor_pick,
		uint8_t floor_drop) { // x?���? , ?��?��?��?�� ?���?, 리니?�� ?���?, ?��?��?��?�� ?��?��, x?���?, ?��?��?��?�� ?���?, 리니?�� ?���?, ?��?��?��?�� ?��?�� ?��
	//				                                                   -----------------------------          ----------------------------
	if (Manual_mode.target_3_1 == TRUE) {

		if (target_3_1_flag == 0) {
			v_t_graph_DIR_GO_X(targetS_X_GO2);  //?��?���?

			target_3_1_flag = 1;
		}

		if ((Finish_flag.x_go == TRUE) && (target_3_1_flag == 1)) {
			target_3_1_flag = 2;
			HAL_Delay(2100 - 1);

			Manual_mode.box_pickup = TRUE;
			box_floor_test_pick = floor_pick;
			Finish_flag.x_go = FALSE;

		}

		if ((Manual_mode.box_pickup == FALSE) && (target_3_1_flag == 2)) {

			target_3_1_flag = 3;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_BACK_X(targetS_X_BACK2);

		}
		if ((Finish_flag.x_back == TRUE) && (target_3_1_flag == 3)) {
			target_3_1_flag = 4;
			HAL_Delay(700 - 1);

			Manual_mode.Box_dropoff = TRUE;
			box_floor_test_drop = floor_drop;

			Finish_flag.x_back = FALSE;
		}

		if ((Manual_mode.Box_dropoff == FALSE) && (target_3_1_flag == 4)) {
			target_3_1_flag = 5;

			FALSE_Init();
			Manual_mode.target_3_1 = FALSE;
		}
	}
}

// x�? 1�? ?���? ,볼스?���? 2�? ?��?��, y�? ?���? , y�? ?���? , 볼스?���? 2�? ?��, x�? ?���?
//?��기서 ?��?�� ?��?�� 추�?
//x 1�? ?���? , 볼스?���? ?��?��, y�? ?���? , ?��?��?��?�� ?���?, 리니?�� ?���?, ?��?��?��?�� ?��?��, y?���?, 볼스?���? ?��, x?���?, ?��?��?��?�� ?��?��, 리니?�� ?���?, ?��?��?��?�� ?��?��
//                               ------------------------------                      --------------------------------
// 1              2         3                  4                    5       6     7          8
void Manual_Mode_X1Y1(float row, float col, uint8_t floor_pick,
		uint8_t floor_drop) {

	if (Manual_mode.target_2_2 == TRUE) {

		if (target_2_2_flag == 0) {
			v_t_graph_DIR_GO_X(targetS_X_GO);  // x ?��?���?

			target_2_2_flag = 1;
		}

		if ((Finish_flag.x_go == TRUE) && (target_2_2_flag == 1)) {
			target_2_2_flag = 2;
			HAL_Delay(700 - 1);

			z_axis_DOWN(400 * 6 * 2 + 500, ball_screw_pulse_cycle); //볼스?���? 2�? ?��?��

			Finish_flag.x_go = FALSE;
		}
		if ((Finish_flag.ball_low == TRUE) && (target_2_2_flag == 2)) {
			target_2_2_flag = 3;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_GO_Y(targetS_Y_GO); //					// y�? ?���?
			Finish_flag.ball_low = FALSE;
		}
		if ((Finish_flag.y_go == TRUE) && (target_2_2_flag == 3)) {
			target_2_2_flag = 4;
			HAL_Delay(3000 - 1);

			Manual_mode.box_pickup = TRUE;
			box_floor_test_pick = floor_pick;

			Finish_flag.y_go = FALSE;
		}
		if ((Manual_mode.box_pickup == FALSE) && (target_2_2_flag == 4)) {
			target_2_2_flag = 5;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_BACK_Y(targetS_Y_BACK); //	y?���?

			//Finish_flag.y_back = FALSE;
		}
		if ((Finish_flag.y_back == TRUE) && (target_2_2_flag == 5)) {
			target_2_2_flag = 6;
			HAL_Delay(700 - 1);

			z_axis_UP(400 * 6 * 2, ball_screw_pulse_cycle); //		    볼스?���? 2�? ?��

			Finish_flag.y_back = FALSE;
		}
		if ((Finish_flag.ball_high == TRUE) && (target_2_2_flag == 6)) {
			target_2_2_flag = 7;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_BACK_X(targetS_X_BACK); //
			Finish_flag.ball_high = FALSE;
		}
		if ((Finish_flag.x_back == TRUE) && (target_2_2_flag == 7)) {
			target_2_2_flag = 8;
			HAL_Delay(700 - 1);

			Manual_mode.Box_dropoff = TRUE;
			box_floor_test_drop = floor_drop;

			Finish_flag.x_back = FALSE;
		}
		if ((Manual_mode.Box_dropoff == FALSE) && (target_2_2_flag == 8)) {
			target_2_2_flag = 9;
			HAL_Delay(700 - 1);

			FALSE_Init();
			Manual_mode.target_2_2 = FALSE;
		}

	}
}

//x 1�? ?���? , 볼스?���? ?��?��, y�? ?���? , ?��?��?��?�� ?���?, 리니?�� ?���?, ?��?��?��?�� ?��?��, y?���?, 볼스?���? ?��, x?���?, ?��?��?��?�� ?��?��, 리니?�� ?���?, ?��?��?��?�� ?��?��
//                               ------------------------------                      --------------------------------
// 1              2         3                  4                    5       6     7          8
void Manual_Mode_X2Y1(float row, float col, uint8_t floor_pick,
		uint8_t floor_drop) {

	if (Manual_mode.target_3_2 == TRUE) {

		if (target_3_2_flag == 0) {
			v_t_graph_DIR_GO_X(targetS_X_GO2);  // x 2�? ?��?���?

			target_3_2_flag = 1;
		}

		if ((Finish_flag.x_go == TRUE) && (target_3_2_flag == 1)) {
			target_3_2_flag = 2;
			HAL_Delay(700 - 1);

			z_axis_DOWN(400 * 6 * 2 + 500, ball_screw_pulse_cycle); //볼스?���? 2�? ?��?��

			Finish_flag.x_go = FALSE;
		}
		if ((Finish_flag.ball_low == TRUE) && (target_3_2_flag == 2)) {
			target_3_2_flag = 3;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_GO_Y(targetS_Y_GO); //					// y�? ?���?
			Finish_flag.ball_low = FALSE;
		}
		if ((Finish_flag.y_go == TRUE) && (target_3_2_flag == 3)) {
			target_3_2_flag = 4;
			HAL_Delay(2100 - 1);

			Manual_mode.box_pickup = TRUE;

			box_floor_test_pick = floor_pick;

			Finish_flag.y_go = FALSE;
		}
		if ((Manual_mode.box_pickup == FALSE) && (target_3_2_flag == 4)) {
			target_3_2_flag = 5;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_BACK_Y(targetS_Y_BACK); //	y?���?

			//Finish_flag.y_back = FALSE;
		}
		if ((Finish_flag.y_back == TRUE) && (target_3_2_flag == 5)) {
			target_3_2_flag = 6;
			HAL_Delay(700 - 1);

			z_axis_UP(400 * 6 * 2, ball_screw_pulse_cycle); //		    볼스?���? 2�? ?��

			Finish_flag.y_back = FALSE;
		}
		if ((Finish_flag.ball_high == TRUE) && (target_3_2_flag == 6)) {
			target_3_2_flag = 7;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_BACK_X(targetS_X_BACK2); //
			Finish_flag.ball_high = FALSE;
		}
		if ((Finish_flag.x_back == TRUE) && (target_3_2_flag == 7)) {
			target_3_2_flag = 8;
			HAL_Delay(700 - 1);

			Manual_mode.Box_dropoff = TRUE;
			box_floor_test_drop = floor_drop;

			Finish_flag.x_back = FALSE;
		}
		if ((Manual_mode.Box_dropoff == FALSE) && (target_3_2_flag == 8)) {
			target_3_2_flag = 9;
			HAL_Delay(700 - 1);

			FALSE_Init();
			Manual_mode.target_3_2 = FALSE;
		}

	}
}

int linear_init = 0;

char txData[] = "Hello, UART3!\r\n";
char rxData[100];

uint8_t rxByte;
volatile uint8_t rxIndex = 0;

#define DATA_SIZE 200

char segment[6];

char Rx_data[DATA_SIZE];
char Rx_buffer[2];  // 1�??��?�� ?��?��
int Transfer_finish = 0;  // ?��?�� ?���? ?��?���?

extern float RPM_X, RPM_Y;
extern int down_max_flag;

int plc;

int pik;

int rol = 0;

Queue str_q;
char pased_flag = '0';
char tmp;

bool paser_flag = FALSE;

void fillQueue(char *input) {
	int input_length = strlen(input);
	int i = 0;

	for (i = 0; i < input_length; i++) {
		enqueue(&str_q, input[i]);
	}
}
char cmd_[4];

char num;
/*
 void analyzer(char cmd[4], char num) {

 if (strcmp(&cmd[0], "mov") == 0) {
 //		mov++;

 HAL_Delay(2500);
 if (rol % 2==0)  //짝수: x방향  ?���?
 {
 if(num == '1')
 {
 v_t_graph_DIR_BACK_X(targetS_X_BACK);  // x ?��?���?
 }
 else if(num == '2')
 {
 v_t_graph_DIR_BACK_X(targetS_X_BACK2);  // x ?��?���?
 }
 }
 else if (rol % 2==1)  //???�� y방향  ?���?
 {
 if(num == '1')
 {
 v_t_graph_DIR_BACK_Y(targetS_Y_BACK);
 }

 }


 } else if (strcmp(&cmd[0], "pov") == 0) {
 //		pov++;

 HAL_Delay(2500);
 if (rol % 2==0)  //짝수: x방향 ?���?
 {
 if(num == '1')
 {
 v_t_graph_DIR_GO_X(targetS_X_GO);  // x ?��?���?
 }
 else if(num == '2')
 {
 v_t_graph_DIR_GO_X(targetS_X_GO2);  // x ?��?���?
 }
 }
 else if (rol % 2==1)  //???�� y방향  ?���?
 {
 if(num == '1')
 {
 v_t_graph_DIR_GO_Y(targetS_Y_GO);
 }

 }



 } else if (strcmp(&cmd[0], "rol") == 0) {

 HAL_Delay(2500);
 if(num == '0')
 {
 z_axis_DOWN(400 * 6 * 2+700, ball_screw_pulse_cycle);   //�? ?���? x -> y

 }
 else if(num == '1')
 {
 z_axis_UP(400 * 6 * 2, ball_screw_pulse_cycle);   //�? ?���? y -> x

 }
 rol++;




 } else if (strcmp(&cmd[0], "pik") == 0) {
 pik++;
 HAL_Delay(2500);
 if(num == '1')
 {
 FALSE_Init();

 Manual_mode.box_pickup = TRUE;
 box_floor_test_pick=1;
 }
 else if(num == '2')
 {
 FALSE_Init();

 Manual_mode.box_pickup = TRUE;
 box_floor_test_pick=2;
 }
 else if(num == '3')
 {
 FALSE_Init();

 Manual_mode.box_pickup = TRUE;
 box_floor_test_pick=3;
 }



 } else if (strcmp(&cmd[0], "plc") == 0) {
 plc++;
 HAL_Delay(2500);
 if(num == '1')
 {
 FALSE_Init();

 Manual_mode.Box_dropoff = TRUE;
 box_floor_test_drop=1;
 }
 else if(num == '2')
 {
 FALSE_Init();

 Manual_mode.Box_dropoff = TRUE;
 box_floor_test_drop=2;
 }
 else if(num == '3')
 {
 FALSE_Init();

 Manual_mode.Box_dropoff = TRUE;
 box_floor_test_drop=3;
 }


 }
 return;
 }
 */
AnalyzerType Analyzer = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
		FALSE, FALSE, FALSE };

void analyzer(char cmd[4], char num_) {
	num = num_;

	if (strcmp(&cmd[0], "mov") == 0) {

		Analyzer.mov = TRUE;

	} else if (strcmp(&cmd[0], "pov") == 0) {

		Analyzer.pov = TRUE;

	} else if (strcmp(&cmd[0], "rol") == 0) {

		Analyzer.rol = TRUE;

	} else if (strcmp(&cmd[0], "pik") == 0) {

		Analyzer.pik = TRUE;

	} else if (strcmp(&cmd[0], "plc") == 0) {

		Analyzer.plc = TRUE;

	}
	return;
}

void paser(char *input) {
	if (pased_flag == '0') {
		//printf("start Queue\n");
		if (is_empty(&str_q)) {
			printf("queue is empty\n");
			return;
		}
		tmp = dequeue(&str_q);
		while (tmp != '@') {
			if (is_empty(&str_q)) {
				printf("queue is empty\n");
				return;
			}
			tmp = dequeue(&str_q);
		}
		pased_flag = '1';
	}

	if (tmp == '@' || tmp == '#') {
		char cmd[4] = "   \0";
		char num;

		int i = 0;
		for (i = 0; i < 3; i++) {
			cmd[i] = dequeue(&str_q);
		}

		dequeue(&str_q);
		num = dequeue(&str_q);
		tmp = dequeue(&str_q);

		analyzer(cmd, num);
		//	paser_flag=0;
	} else if (tmp == '!') {
		printf("needed new commend\n");
		pased_flag = '0';
	}
	return;
}

int countCmd(char *input) {
	int len = strlen(input);
	int cmdLen = 0;
	int i = 0;

	for (i = 0; i < len; i++) {
		if (input[i] == '@') {
			cmdLen = 0;
		} else {
			cmdLen++;
		}
		if (input[i] == '!') {
			return cmdLen / 6;
		}
	}
}

int count____;

char *input;

int down_high_test;
int ball_min_test;
int x_go;
int x_back;

int y_go;
int y_back;
int ball_high;

int RPM_X_go;
int RPM_X_back;
int RPM_Y_go;
int RPM_Y_back;
int PID_Y_finish;
int PID_X_finish;

// int downpart_run;
int ballscrew_up_run;

int ballscrew_down_run;
uint8_t step_count_for_flag__;

uint8_t shaft_accel_flag; //0:?��?�� ,1 �??��

int DC_Y_go_sensor_flag;
int DC_Y_back_sensor_flag;

// int DC_X_now_posiotion;
int DC_Y_now_posiotion;

extern uint32_t save_X_IN;
extern uint32_t save_Y_IN;

uint16_t tim9_flag = 0;
uint16_t tim10_flag = 100;
bool tim12_flag = FALSE;

int asda = 0;

bool down_max_flag_ = FALSE;


int algo_y_go=0;
int algo_y_back=0;
int algo_x_go=0;
int algo_x_back=0;
int algo_x_go2=0;
int algo_x_back2=0;
int algo_rol_up=0;
int algo_rol_down=0;
int algo_pik1=0;
int algo_pik2=0;
int algo_pik3=0;
int algo_plc1=0;
int algo_plc2=0;
int algo_plc3=0;

bool algo_y_go_=FALSE;
bool algo_y_back_=FALSE;

char RxRX;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	downpart_accel_initialize();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	linear_init = 1;
	if (linear_init == 1) {
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //?���?
	}
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM13_Init();
  MX_ADC3_Init();
  MX_TIM12_Init();
  MX_TIM11_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_I2C2_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	linear_init = 1;
	if (linear_init == 1) {
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //?���?
	}
	TIM3->CCR4 = 0;
	MPU6050_Init();

//	HAL_UART_Receive_IT(&huart3, &rxByte, 1);
	HAL_UART_Receive_IT(&huart3, &key_value, 1);
	//HAL_UART_Receive_IT(&huart4, &data, 1);
	HAL_UART_Receive_IT(&huart4, (uint8_t*) Rx_buffer, 1);

//	HAL_UART_Receive_IT(&huart4, &RxRX, 1);

	HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC3_value, 3);

	HAL_TIM_Base_Start_IT(&htim6);  //pid
	HAL_TIM_Base_Start_IT(&htim7);  //?��?��
	HAL_TIM_Base_Start_IT(&htim14);  //RPM
	HAL_TIM_Base_Start_IT(&htim5);  //vt

	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_Base_Start_IT(&htim10);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_Base_Start_IT(&htim12);  //초음?�� ?��?��  ?��?? >
	HAL_TIM_Base_Start_IT(&htim8);  //ADC ???���?

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	setvbuf(stdin, NULL, _IONBF, 0);

	gridInit(IsEmpty);
	setDpp(0, 0, 5, 5); //0,0 ?��?�� 1,2�?
	readPath(lpp(), path);

	TIM3->CCR4 = 0;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //?���?
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); //리니?��  ?���?

	down_high_test = 0;
	ball_min_test = 0;
	x_go = 0;
	x_back = 0;
	y_go = 0;
	y_back = 0;
	ball_high = 0;

	RPM_X_go = 0;
	RPM_X_back = 0;
	RPM_Y_go = 0;
	RPM_Y_back = 0;

	PID_X_finish = 0;
	PID_Y_finish = 0;
//	downpart_run=0;
	ballscrew_up_run = 0;
	ballscrew_down_run = 0;
	pickup_run = 0;
	dropoff_run = 0;

	step_count_for_flag__ = 0;

	shaft_accel_flag = 0; //0:?��?�� ,1 �??��

	DC_Y_go_sensor_flag = 0;
	DC_Y_back_sensor_flag = 0;

	//   DC_X_now_posiotion=0;
	DC_Y_now_posiotion = 0;





	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (tim12_flag == TRUE)  //0.1s
		{
			///////////////// ?��?�� 버튼 깔짝
			if(  key_value=='>' || RxRX =='R')
			{		if (down_max_flag == 1 ) {

					tim9_flag = 0;
					down_max_flag_ = TRUE;
					down_max_flag = 0 ;
				}
				else if (down_max_flag_ == TRUE) {
					if (tim9_flag >= 2)   //0.1�? 마다++ >> 10: 1s
					{
						down_part_DOWN(130, shaft_pulse_cycle);
						down_max_flag_ = FALSE;
					}

				}
			}
////////////////////////////////  ?��?��리�?

			 if (Analyzer.mov == TRUE)
			 {
				 tim9_flag = 0;
				 Analyzer.mov_ = TRUE;

				 Analyzer.mov = FALSE;
			 }
			 else if (Analyzer.mov_ == TRUE)
			 {

				 if (tim9_flag >= 14)   //0.1�? 마다++ >> 2.5�? ??�?
				 {

					 if (rol % 2 == 0)  //짝수: x방향  ?���?
					 {

						 if (num == '1')
						 {
							 v_t_graph_DIR_BACK_X(targetS_X_BACK);
							 algo_x_back++;
						 }
						 else if (num == '2')
						 {
							 v_t_graph_DIR_BACK_X(targetS_X_BACK2);
							 algo_x_back2++;
						 }
					 }
				 else if (rol % 2 == 1)  //???�� y방향  ?���?
				 {
					 if (num == '1')
					 {
							HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
							HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

							HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
							HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
							HAL_Delay(80);
						 v_t_graph_DIR_BACK_Y(targetS_Y_BACK);
						// v_t_graph_DIR_BACK_Y(targetS_Y_BACK);
						 algo_y_back++;
					 }

				 }
					 Analyzer.mov_ = FALSE;
				 }
			 }

			 else if (Analyzer.pov == TRUE)
			 {
				 tim9_flag = 0;
				 Analyzer.pov_ = TRUE;

				 Analyzer.pov = FALSE;
			 }
			 else if (Analyzer.pov_ == TRUE)
			 {
				 if (tim9_flag >= 14)   //0.1�? 마다++ >> 2.5�? ??�?
				 {

					 if (rol % 2 == 0)  //짝수: x방향 ?���?
					 {
						 if (num == '1')
						 {
							 v_t_graph_DIR_GO_X(targetS_X_GO);  // x ?��?���?
							 algo_x_go++;
						 }
						 else if (num == '2')
						 {
							 v_t_graph_DIR_GO_X(targetS_X_GO2);  // x ?��?���?
							 algo_x_go2++;
						 }
					 }
					 else if (rol % 2 == 1)  //???�� y방향  ?���?
					 {
						 if (num == '1')
						 {
								HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
								HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

								HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
								HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
							//	HAL_Delay(80);
							 v_t_graph_DIR_GO_Y(targetS_Y_GO);
							// v_t_graph_DIR_GO_Y(targetS_Y_GO);
							 algo_y_go++;


						 }

					 }
						 Analyzer.pov_ = FALSE;
				 }
			 }

			 else if (Analyzer.rol == TRUE)
			 {
				 tim9_flag = 0;
				 Analyzer.rol_ = TRUE;

				 Analyzer.rol = FALSE;
			 }
			 else if (Analyzer.rol_ == TRUE)
			 {
				 if (tim9_flag >= 14)   //0.1�? 마다++ >> 2.5�? ??�?
				 {

					 if (num == '0')
					 {
						 z_axis_DOWN(400 * 6 * 2 + 700, ball_screw_pulse_cycle); //�? ?���? x -> y
						 algo_rol_down++;
					 }
					 else if (num == '1')
					 {
						 z_axis_UP(400 * 6 * 2, ball_screw_pulse_cycle); //�? ?���? y -> x
						 algo_rol_up++;
					 }
					 rol++;

				 	 Analyzer.rol_ = FALSE;
				 }
			 }

			 else if (Analyzer.pik == TRUE)
			 {
				 tim9_flag = 0;
				 Analyzer.pik_ = TRUE;

				 Analyzer.pik = FALSE;
			 }
			 else if (Analyzer.pik_ == TRUE)
			 {
				 if (tim9_flag >= 15)   //0.1�? 마다++ >> 2.5�? ??�?
				 {

					 pik++;  //?��?�� ???��?�� : 물건?�� ?��고있?�� ?��?��?��, 짝수: �? ?��?��
					 //			HAL_Delay(2500);
					 if (num == '1')
					 {
						 FALSE_Init();

						 Manual_mode.box_pickup = TRUE;
						 box_floor_test_pick = 1;
						 algo_pik1++;
					 }
					 else if (num == '2')
					 {
						 FALSE_Init();

						 Manual_mode.box_pickup = TRUE;
						 box_floor_test_pick = 2;
						 algo_pik2++;
					 }
					 else if (num == '3')
					 {
						 FALSE_Init();

						 Manual_mode.box_pickup = TRUE;
						 box_floor_test_pick = 3;
						 algo_pik3++;
					 }

					Analyzer.pik_ = FALSE;
				 }
			 }

			 else if (Analyzer.plc == TRUE)
			 {
				 tim9_flag = 0;
				 Analyzer.plc_ = TRUE;

				 Analyzer.plc = FALSE;
			 }
			 else if (Analyzer.plc_ == TRUE)
			 {
				 if (tim9_flag >= 9)   //0.1�? 마다++ >> 2.5�? ??
				 {

					 plc++;
					 //		HAL_Delay(2500);
					 if (num == '1')
					 {
						 FALSE_Init();

						 Manual_mode.Box_dropoff = TRUE;
						 box_floor_test_drop = 1;
						 algo_plc1++;
					 }
					 else if (num == '2')
					 {
						 FALSE_Init();

						 Manual_mode.Box_dropoff = TRUE;
						 box_floor_test_drop = 2;
						 algo_plc2++;
					 }
					 else if (num == '3')
					 {
						 FALSE_Init();

						 Manual_mode.Box_dropoff = TRUE;
						 box_floor_test_drop = 3;
						 algo_plc3++;
					 }
					Analyzer.plc_ = FALSE;
				}
			 }

			 //	asda++; // ok

			 //if(asda==100) HAL_Delay(1000-1);


		///////////	//?��?��리�? ?��?��

/*
			if (Analyzer.mov == TRUE)
			{


				if (rol % 2 == 0)  //짝수: x방향  ?���?
				{
					if (num == '1')
					{
						v_t_graph_DIR_BACK_X(targetS_X_BACK);  // x ?���?
					}
					else if (num == '2')
					{
						v_t_graph_DIR_BACK_X(targetS_X_BACK2);  // x ?���?2
					}
				}
				else if (rol % 2 == 1)  //???�� y방향  ?���?
				{
					if (num == '1')
					{
						v_t_graph_DIR_BACK_Y(targetS_Y_BACK);
					}

				}
				HAL_Delay(700-1);
					Analyzer.mov = FALSE;
			}
			else if (Analyzer.pov == TRUE)
			{

				if (rol % 2 == 0)  //짝수: x방향 ?���?
				{
					if (num == '1')
					{
						v_t_graph_DIR_GO_X(targetS_X_GO);  // x ?��?���?
					}
					else if (num == '2')
					{
						v_t_graph_DIR_GO_X(targetS_X_GO2);  // x ?��?���?
					}
				}
				else if (rol % 2 == 1)  //???�� y방향  ?���?
				{
					if (num == '1')
					{
						v_t_graph_DIR_GO_Y(targetS_Y_GO);
					}

				}
				HAL_Delay(700-1);
				Analyzer.pov = FALSE;
		}
		else if (Analyzer.rol == TRUE)
		{


				if (num == '0')
				{
					z_axis_DOWN(400 * 6 * 2 + 700, ball_screw_pulse_cycle); //�? ?���? x -> y

				}
				else if (num == '1')
				{
					z_axis_UP(400 * 6 * 2, ball_screw_pulse_cycle); //�? ?���? y -> x

				}
				rol++;
				HAL_Delay(700-1);

							Analyzer.rol = FALSE;
		}
		else if (Analyzer.pik == TRUE)
		{
				pik++;  //?��?�� ???��?�� : 물건?�� ?��고있?�� ?��?��?��, 짝수: �? ?��?��
				//			HAL_Delay(2500);
				if (num == '1')
				{
					FALSE_Init();

					Manual_mode.box_pickup = TRUE;
					box_floor_test_pick = 1;
				}
				else if (num == '2')
				{
					FALSE_Init();

					Manual_mode.box_pickup = TRUE;
					box_floor_test_pick = 2;
				}
				else if (num == '3')
				{
					FALSE_Init();

					Manual_mode.box_pickup = TRUE;
					box_floor_test_pick = 3;
				}

				HAL_Delay(4000-1);
							Analyzer.pik = FALSE;
		}
		else if (Analyzer.plc == TRUE)
		{

				plc++;
				//		HAL_Delay(2500);
				if (num == '1')
				{
					FALSE_Init();

					Manual_mode.Box_dropoff = TRUE;
					box_floor_test_drop = 1;
				}
				else if (num == '2')
				{
					FALSE_Init();

					Manual_mode.Box_dropoff = TRUE;
					box_floor_test_drop = 2;
				}
				else if (num == '3')
				{
					FALSE_Init();

					Manual_mode.Box_dropoff = TRUE;
					box_floor_test_drop = 3;
				}
				HAL_Delay(700-1);

					Analyzer.plc = FALSE;
	}
*/



















//////////////////////////
			tim12_flag = FALSE;

		}

		/*
		 if (DC_Y_go_sensor_flag == 1 && DC_Y_now_posiotion == 1) {
		 i_v_t_count = 0;
		 HAL_Delay(1500);
		 Finish_flag.y_go = TRUE; // ?���?
		 paser_flag = 1;

		 y_go++;
		 DC_Y_go_sensor_flag = 0;
		 } else if (DC_Y_back_sensor_flag == 1 && DC_Y_now_posiotion == 0) {
		 i_v_t_count = 0;
		 HAL_Delay(1500);
		 Finish_flag.y_back = TRUE; // ?���?
		 paser_flag = 1;

		 y_back++;

		 DC_Y_back_sensor_flag = 0;
		 }
		 */
		////////////////////

		if (tim6_flag == 1)  //1ms
				{
			//		distance_sensor();  //거리 ?��?��
			//		MPU6050_Read_Accel();  //MPU?��문에 거리?��?�� ?��균이 계산?�� ?��?���?
			//		MPU6050_Read_Gyro();






			tim6_flag = 0;
		}
		if (key_value == '#')
			Manual_Mode_Y1(1, 2, box_floor_test_pick, box_floor_test_drop);

		if (key_value == '!')
			Manual_Mode_X1(2, 1, box_floor_test_pick, box_floor_test_drop);

		if (key_value == '@')
			Manual_Mode_X2(3, 1, box_floor_test_pick, box_floor_test_drop);

		if (key_value == '$')
			Manual_Mode_X1Y1(2, 2, box_floor_test_pick, box_floor_test_drop);

		if (key_value == '%')

			Manual_Mode_X2Y1(3, 2, box_floor_test_pick, box_floor_test_drop);


		if ((Manual_mode.box_pickup == TRUE) && (RPM_X == 0) && (RPM_Y == 0)) {
		i_v_t_count = 0;

					Box_pickup(box_floor_test_pick);
				}

				if ((Manual_mode.Box_dropoff == TRUE) && (RPM_X == 0) && (RPM_Y == 0)) {
					i_v_t_count = 0;
					Box_dropoff(box_floor_test_drop);

				}

//		if (down_max_flag == 1) {
//			HAL_Delay(140);
//			down_part_DOWN(130, shaft_pulse_cycle);
//			down_max_flag = 0;
//		}
		/*  //?���? ???�� ?��?���? ?��?��
		 if(PID_Y_finish==1)
		 {
		 HAL_Delay(800);
		 RPM_Y_go=0;
		 RPM_Y_back=0;

		 save_Y_IN=0;
		 PID_Y_finish=0;
		 }

		 if(PID_X_finish==1)
		 {
		 HAL_Delay(400);
		 RPM_X_go=0;
		 RPM_X_back=0;

		 PID_X_finish=0;
		 }
		 */
		if (photo_X_go == TRUE) {
			if (RPM_X == 0) {
				HAL_Delay(200);
				RPM_X_go = 0;
				RPM_X_back = 0;

				save_X_IN = 0;
				//	PID_X_finish=0;
				photo_X_go = FALSE;
			}

		} else if (photo_X_back == TRUE) {
			if (RPM_X == 0) {
				HAL_Delay(200);
				RPM_X_go = 0;
				RPM_X_back = 0;

				save_X_IN = 0;
				//	PID_X_finish=0;
				photo_X_back = FALSE;

			}
		}
		/*		if(photo_Y_go == TRUE)
		 {
		 if(RPM_Y ==0)
		 {
		 HAL_Delay(400);
		 RPM_Y_go=0;
		 RPM_Y_back=0;

		 save_Y_IN=0;
		 //	PID_Y_finish=0;
		 photo_Y_go=FALSE;
		 }
		 }
		 else if(photo_Y_back == TRUE)
		 {
		 if(RPM_Y ==0)
		 {
		 HAL_Delay(400);
		 RPM_Y_go=0;
		 RPM_Y_back=0;

		 save_Y_IN=0;
		 //	PID_Y_finish=0;
		 photo_Y_back=FALSE;
		 }
		 }
		 */

		//?��고리�? ?��?��
		if (Transfer_finish) {

			fillQueue(Rx_data);
			//	int i = 0;
			pased_flag = '0';
			//		for (i = 0; i < countCmd(Rx_data); i++) {
			//			paser(Rx_data);
			//			//count____++;
			//		}
			paser(Rx_data);

			Transfer_finish = 0;
		}

		if (tim5_flag == 1) { //0.01s
			if (paser_flag == 1) {
				paser(Rx_data);
				//		HAL_Delay(1500);

				paser_flag = 0;
				count____++;
			}
			tim5_flag = 0;
		}

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = ENABLE;
  hadc3.Init.NbrOfDiscConversion = 3;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
	//0.001s = 1ms
	//PID ?��?���? 주기
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 84-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
	//ADC 측정 ???���? , ?��중엔 ?��?�� x
  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 84-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 300-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 16800-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 16800-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 168-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 800-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */
	// 초음?�� ?��?��
  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 8400-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */
// ?��?��?�� ?��
  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 84-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 800-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 130;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim13, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM13_Init 2 */
	//HAL_TIM_MspPostInit(&htim13);
  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */
//RPM 측정
  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 84-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */
//	TIM14->DIER |= (1 << 0);   // CC1IE: Enable the Tim14 UG interrupt
//	NVIC->ISER[1] |= (1 << (45 - 32)); // TIM14_CC
//	TIM14->CR1 |= (1 << 0);	// CEN: Counter TIM14 enable
  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 230400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_EVEN;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, linear_DIR_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(linear_goback_GPIO_Port, linear_goback_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|DC_M_GO_Y_Pin|DC_M_DIR_Y_Pin|LD3_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DC_M_GO_X_Pin|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_M_DIR_X_GPIO_Port, DC_M_DIR_X_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER_Btn_Pin PC6 */
  GPIO_InitStruct.Pin = USER_Btn_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : linear_DIR_Pin PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = linear_DIR_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : linear_goback_Pin */
  GPIO_InitStruct.Pin = linear_goback_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(linear_goback_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DOWN_LIMIT_SW_Pin */
  GPIO_InitStruct.Pin = DOWN_LIMIT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOWN_LIMIT_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin DC_M_GO_Y_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|DC_M_GO_Y_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_M_GO_X_Pin PG2 PG3 */
  GPIO_InitStruct.Pin = DC_M_GO_X_Pin|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : DC_M_DIR_Y_Pin */
  GPIO_InitStruct.Pin = DC_M_DIR_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DC_M_DIR_Y_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BALL_HIGH_Pin PD2 PD6 */
  GPIO_InitStruct.Pin = BALL_HIGH_Pin|GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : BALL_MID_Pin */
  GPIO_InitStruct.Pin = BALL_MID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BALL_MID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_X_Pin sensor_Y_Pin */
  GPIO_InitStruct.Pin = sensor_X_Pin|sensor_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DC_M_DIR_X_Pin */
  GPIO_InitStruct.Pin = DC_M_DIR_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DC_M_DIR_X_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

float input_encodor_X_GO = 34816 * 2;
float input_encodor_X_BACK = -34816 * 2;

float input_encodor_Y_GO = 10880 * 2;
float input_encodor_Y_BACK = -10880 * 2;

PUTCHAR_PROTOTYPE  //?��?��??
{
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void SerialSendChar_PC(uint8_t Ch1) // 1문자 보내�? ?��?��
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty?
	// TX buffer Empty?���? ?��?���? 계속 ??�?(?��?�� �??��?�� ?��?��까�? ??�?)
	while ((USART3->SR & 1 << 7) == RESET)
		;
	USART3->DR = (Ch1 & 0x01FF);	// ?��?�� (최�? 9bit ?���?�? 0x01FF�? masking)
}

void SerialSendChar_ESP(uint8_t Ch2) // 1문자 보내�? ?��?��
{
	while ((UART4->SR & 1 << 7) == RESET)
		;
	UART4->DR = (Ch2 & 0x01FF);
}

extern uint16_t ADC3_value[3];

extern float battery_V;
uint16_t RPM_uint16;
uint16_t battery_uint16;

uint8_t buffer[50];

void STM32_to_ESP(void)                    // ESP�? ?��?��?�� 보냄
{

#define test 1
#if test ==1
//	GPIOB->ODR ^= 1 << 7;
	RPM_uint16 = RPM_X * 10;
	battery_uint16 = battery_V * 100;

	SerialSendChar_ESP('?');

	//?��?��?��

	if (ADC3_value[0] >= 1000 && ADC3_value[0] <= 4095) {

		sprintf((char*) buffer, "%d\n\r", ADC3_value[0]);
		//		HAL_UART_Transmit(&huart3, buffer, strlen((char*) buffer), 100);
		sprintf((char*) buffer, "%d\n\r", ADC3_value[0]);
		//		HAL_UART_Transmit(&huart3, buffer, strlen((char*) buffer), 100);
		sprintf((char*) buffer, "%d\n\r", ADC3_value[0]);
		//	HAL_UART_Transmit(&huart3, buffer, strlen((char*) buffer), 100);

		//배터�?, ?��?�� ?��?��
		sprintf((char*) buffer, "%d", ADC3_value[0]);
		HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		sprintf((char*) buffer, "%d", ADC3_value[0]);
		HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		sprintf((char*) buffer, "%d", ADC3_value[0]);
		HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
	}

#elif test == 2
		 GPIOB->ODR ^= 1 << 7;
		 RPM_uint16 = RPM_X * 10;
		 battery_uint16 = battery_V * 100;

		 SerialSendChar_ESP('?');
		 sprintf((char*) buffer, "%d", battery_uint16);
		 HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		 sprintf((char*) buffer, "%d", RPM_uint16);
		 HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		 sprintf((char*) buffer, "%d", battery_uint16);
		 HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		 #endif

}

extern uint16_t shaft_pulse_cycle;
extern uint16_t ball_screw_pulse_cycle;

uint16_t ball_step = 100;
uint32_t downpart_step = 25;
extern uint32_t step_pulse_count_tim13;
extern uint16_t step_pulse_count_tim11;

extern real_T rtb_Clock;
extern int downpart_accel_step_max;

void UART3_RX_PC_to_STM32(void) // UART3: STM32 <-> PC
{
#define com_test 1

#if com_test==0
	static uint16_t rxIndex = 0;
	static int start_found = 0;  // ?��?�� 문자 발견 ?���?

	if (Rx_buffer[0] == '<') {  // ?��?�� 문자 발견
		start_found = 1;
		memset(Rx_data, 0, DATA_SIZE);  // Rx_data 버퍼 초기?��
		rxIndex = 0;  // 카운?�� 초기?��
	} else if (Rx_buffer[0] == '>' && start_found) {  // ?�� 문자 발견
		Rx_buffer[rxIndex] = '\0';
		Transfer_finish = 1;  // ?��?��?�� 메시�? ?��?�� ?���?
		start_found = 0;  // 메시�? 종료

	} else if (start_found) {
		Rx_data[rxIndex++] = Rx_buffer[0];  // ?��?�� 문자 ?��?�� ?��?��?�� ???��

	}

	//for(int i=rxIndex ; i<100 ; i++)  rxData[i] = 0;

	//	rxIndex = 0; // ?��?��?�� 초기?��

	HAL_UART_Receive_IT(&huart3, (uint8_t*) Rx_buffer, 1);  // ?��?�� 문자 ?��?�� ??�?

	// ?��?�� 바이?�� ?��?�� �?�?
	//     HAL_UART_Receive_IT(&huart3, &rxByte, 1);

#elif com_test ==1

	HAL_UART_Receive_IT(&huart3, &key_value, 1);

	printf("%d,^^ %c \r\n", key_value, key_value);

	switch (key_value) {
	case 'q':
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);  //방향 go

		GPIOB->ODR |= 1 << 0;  //LD1

		break;

	case 'w':

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);  //방향 back
		GPIOB->ODR &= ~1 << 0;  //LD1

		break;

	case '1':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //stop

		break;

	case '2':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);  //start
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		break;

	case '3':
		TIM3->CCR1 = 400;

		break;

	case '4':
		TIM3->CCR1 = 1700;

		break;

	case '5':
		TIM3->CCR1 = 6000;

		break;

		/////////////////////////////////////////////////////////////////y�? ?��?��

	case 'd':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);  //y�? 방향 go

		GPIOB->ODR |= 1 << 0;  //LD1

		break;

	case 'f':

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);  //y�? 방향 back
		GPIOB->ODR &= ~1 << 0;  //LD1

		break;

	case 'a':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);  //stop

		break;

	case 's':
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);  //start

		break;

	case 'e':
		TIM3->CCR2 = 400;

		break;

	case 'r':
		TIM3->CCR2 = 1700;

		break;
	case 't':
		TIM3->CCR2 = 6000;

		break;
		///////////////////////////////////////////////////////////////// x�? 바�?? PID

	case 'i':

		dc_motor_pid_X(34816);

		break;

	case 'o':
		dc_motor_pid_X(-34816);

		break;

//	case 'p':
//		dc_motor_pid_X(input_encodor_X_GO);

		//	break;

//	case '[':
//		dc_motor_pid_X(input_encodor_X_BACK);

//		break;
		////////////////////////////////////////////////////////////////x�? vt
	case 'I':  //30cm

		v_t_graph_DIR_GO_X(30);  //?��?���? 30cm

		break;
	case 'O':

		v_t_graph_DIR_BACK_X(30); //?���? 30cm

		break;
	case 'P':

		v_t_graph_DIR_GO_X(targetS_X_GO);  //?��?���?

		break;
	case '{':

		v_t_graph_DIR_BACK_X(targetS_X_BACK); //?���?

		break;
	case '}':

		v_t_graph_DIR_GO_X(targetS_X_GO2);  //?��?���?

		break;

	case '|':

		v_t_graph_DIR_BACK_X(targetS_X_BACK2); //?���?

		break;

		///////////////////////////////////////////////////////// y�?  바�??  PID

	case '8':

		dc_motor_pid_Y(10880);
		break;

	case '9':
		dc_motor_pid_Y(-10880);
		break;

//	case '0':
//		dc_motor_pid_Y(input_encodor_Y_GO);
//		break;
//	case '-':
//		dc_motor_pid_Y(input_encodor_Y_BACK);
//		break;
//////////////////////////////////////////////////////////      Y  VT
	case '*':

		v_t_graph_DIR_GO_Y(30);   //Y�? vt ?��
		break;

	case '(':
		v_t_graph_DIR_BACK_Y(30);   //?��
		break;

	case ')':
		v_t_graph_DIR_GO_Y(targetS_Y_GO);
		break;

	case '_':
		v_t_graph_DIR_BACK_Y(targetS_Y_BACK);
		break;

		//////////////////////////////////////////////////////z�? 볼스?���? ?��?��모터,  0625?��?��: ?��?��?��?���? enable
	case 'j':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //?���?
		step_pulse_count_tim11 = 0;
		break;

	case 'k':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 0);  //출발

		break;

	case 'l':   //cw 미소 ?��?��             ?��계방?��  ?���?
		z_axis_UP(ball_step, ball_screw_pulse_cycle);

		break;
	case ';':    //ccw 미소 ?��?��           반시계방?��  ?��?���?
		z_axis_DOWN(ball_step, ball_screw_pulse_cycle);

		break;

	case 'L':   //cw              ?��계방?��  ?���?
		z_axis_UP(400 * 6, ball_screw_pulse_cycle);

		break;
	case ':':    //ccw            반시계방?��  ?��?���?

		z_axis_DOWN(400 * 6 + 300, ball_screw_pulse_cycle);

		break;

		/////////////////////////////////////////////////////////////////////?��?��?��?�� �? ?��?��모터
	case 'm':
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);  //?���? ?��?��?��?���?
		step_pulse_count_tim13 = 0;
		break;

	case ',':
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);  //출발  ?��?��?���?

		break;

	case 'M':  //�??��
		FALSE_Init();
		down_part_UP_accel(downpart_step);

		break;
	case '<': //�??��
		FALSE_Init();
		down_part_DOWN_accel(downpart_step);

		break;
	case '.':
		FALSE_Init();
		down_part_UP(downpart_step, shaft_pulse_cycle);

		break;

	case '/':
		FALSE_Init();
		down_part_DOWN(downpart_step, shaft_pulse_cycle);

		break;
	case '>':
		FALSE_Init();
		down_part_UP(400, shaft_pulse_cycle);

		break;

	case '?':
		FALSE_Init();
		down_part_DOWN(400, shaft_pulse_cycle);

		break;
//////////////////////////////////////////////////////////////////////////////////리니?�� 모터
//	case 'z':
//
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //?���?
//
//		break;
//
//	case 'x':
//
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);  //출발
//
//		break;
//	case 'c':
//
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);   //방향 ?���?
//
//		break;
//
//	case 'v':
//
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);  //방향  ?���?
//
//		break;
//
//	case 'b': //방향  ?���?
//
//		TIM3->CCR4 = 1500;
//
//		break;
//	case 'n': //방향  ?���?
//
//		TIM3->CCR4 = 4200;
//
//		break;
//	case 'h': //방향  ?���?
//
//		TIM3->CCR4 = 8000;
//
//		break;

	case 'z': //방향 �? ?���?

		linear_motor_GO(go_time);

		break;
	case 'x': //방향 �? ?���?

		linear_motor_BACK(back_time);

		break;

		/////////////////////////////////////////////////////////?��?�� 조작
	case '!':  // (2,1)
		FALSE_Init();
		Manual_mode.target_2_1 = TRUE;

		break;
	case '@': //  (3,1)
		FALSE_Init();
		Manual_mode.target_3_1 = TRUE;

		break;

	case '#':  //(1,2)
		FALSE_Init();
		Manual_mode.target_1_2 = TRUE;

		break;

	case '$': // (2,2)
		FALSE_Init();
		Manual_mode.target_2_2 = TRUE;
		break;
	case '%':
		FALSE_Init();
		Manual_mode.target_3_2 = TRUE;
		break;

	case 'A':
		FALSE_Init();
		Manual_mode.box_pickup = TRUE;
		break;

	case 'S':
		FALSE_Init();
		Manual_mode.Box_dropoff = TRUE;
		break;

	case 'c':
		HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
		break;
	case 'v':
			HAL_TIM_OC_Stop_IT(&htim13, TIM_CHANNEL_1);
			break;
	}
#endif
}

extern uint8_t linear_go_flag;

void UART4_RX_ESP_to_STM32(void) // UART4: esp <-> stm32
{

	static uint16_t rxIndex = 0;
	static int start_found = 0;  // ?��?�� 문자 발견 ?���?

	if (Rx_buffer[0] == '<') {  // ?��?�� 문자 발견
		start_found = 1;
		memset(Rx_data, 0, DATA_SIZE);  // Rx_data 버퍼 초기?��
		rxIndex = 0;  // 카운?�� 초기?��
	} else if (Rx_buffer[0] == '>' && start_found) {  // ?�� 문자 발견
		Rx_buffer[rxIndex] = '\0';
		Transfer_finish = 1;  // ?��?��?�� 메시�? ?��?�� ?���?
		start_found = 0;  // 메시�? 종료

	} else if (start_found) {
		Rx_data[rxIndex++] = Rx_buffer[0];  // ?��?�� 문자 ?��?�� ?��?��?�� ???��

	}



	HAL_UART_Receive_IT(&huart4, (uint8_t*) Rx_buffer, 1);  // ?��?�� 문자 ?��?�� ??�?


	if (Rx_buffer[0] == '[') {  // ?��?�� 문자 발견
				start_found = 1;
				memset(Rx_data, 0, DATA_SIZE);  // Rx_data 버퍼 초기?��
				rxIndex = 0;  // 카운?�� 초기?��
			} else if (Rx_buffer[0] == ']' && start_found) {  // ?�� 문자 발견
				Rx_buffer[rxIndex] = '\0';
				//Transfer_finish = 1;  // ?��?��?�� 메시�? ?��?�� ?���?
				start_found = 0;  // 메시�? 종료

			} else if (start_found) {
				RxRX = Rx_buffer[0];  // ?��?�� 문자 ?��?�� ?��?��?�� ???��

			}


	HAL_UART_Receive_IT(&huart4, (uint8_t*) &RxRX, 1);

//HAL_UART_Receive_IT(&huart3, &key_value, 1);
//	 rxData[rxIndex++] = rxByte;
//
//		        // 종료 문자 !�? ?��?���? 받으�? 문자?�� ?��?��
//		        if (rxByte == '!') {
//		            rxData[rxIndex] = '\0'; // 문자?�� 종료 문자 추�?
//		         //   HAL_UART_Transmit(&huart3, (uint8_t*)rxData, rxIndex, HAL_MAX_DELAY);
//
//		            sprintf((char*) buffer, "%s\n\r", rxData);
//		           		 HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
//
//
//
//		            for(int i=rxIndex ; i<100 ; i++)  rxData[i] = 0;
//
//		            	rxIndex = 0; // ?��?��?�� 초기?��
//		        }
//
//		        // ?��?�� 바이?�� ?��?�� �?�?
//		        HAL_UART_Receive_IT(&huart4, &rxByte, 1);
//
//
//


	// HAL_UART_Receive_IT(&huart4, &data, 1);

//	 printf("%d -- %c \r\n", data, data);
	 switch (RxRX) {
	 case '0':

	 break;

	 case 'z':
		 linear_motor_GO(go_time);
	 	 break;
	 case 'x':
			linear_go_flag=1;
		 linear_motor_BACK(back_time);
	 	 break;
	 case 'Q':
		 z_axis_UP(400 * 6, ball_screw_pulse_cycle);
	 	 break;
	 case 'E':
		 z_axis_DOWN(400 * 6 + 100, ball_screw_pulse_cycle);
	 	 break;
	 case 'q':
		 z_axis_UP(ball_step, ball_screw_pulse_cycle);
	 	 break;
	 case 'e':
		 z_axis_DOWN(ball_step, ball_screw_pulse_cycle);
	 	 break;



	 case 'R':
		 FALSE_Init();
		 down_part_UP(3000, shaft_pulse_cycle);
	 	 break;
	 case 'T':
		 FALSE_Init();
		 down_part_DOWN(3000, shaft_pulse_cycle);
	 	 break;
	 case 'r':
		 FALSE_Init();
		 down_part_UP(downpart_step, shaft_pulse_cycle);
	 	 break;
	 case 't':
		 FALSE_Init();
		 		down_part_DOWN(downpart_step, shaft_pulse_cycle);
	 	 	 break;



	 }


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
