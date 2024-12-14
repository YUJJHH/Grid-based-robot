#include "sensor.h"
#include "main.h"
//#include "pid.h"
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOD
#define ECHO_PIN GPIO_PIN_6
#define ECHO_PORT GPIOD

float Distance = 0;  // cm
float Distance_Sum = 0;
float Distance_Avg;
uint16_t j = 0;

extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim5;

extern void dc_motor_pid_X_with_photosensor1(uint32_t input_encoder);  //x축 pid
extern void dc_motor_pid_Y_with_photosensor2(uint32_t input_encoder);  //y축 pid

extern struct flag Finish_flag;

float distance_save;

extern bool paser_flag;

extern uint16_t tim9_flag;
extern uint16_t tim10_flag;
extern bool tim12_flag;
/*
 void distance_sensor(void) {
 static uint32_t pMillis;
 static uint32_t Value1 = 0;
 static uint32_t Value2 = 0;

 HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); // pull the TRIG pin HIGH
 __HAL_TIM_SET_COUNTER(&htim12, 0);
 while (__HAL_TIM_GET_COUNTER (&htim12) < 10)
 ;  // wait for 10 us
 HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

 pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
 // wait for the echo pin to go high
 while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
 && pMillis + 10 > HAL_GetTick())
 ;
 Value1 = __HAL_TIM_GET_COUNTER(&htim12);

 pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
 // wait for the echo pin to go low
 while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
 && pMillis + 50 > HAL_GetTick())
 ;
 Value2 = __HAL_TIM_GET_COUNTER(&htim12);

 Distance = (Value2 - Value1) * 0.034 / 2 / 2;

 Distance_Sum = Distance + Distance_Sum;
 j++;
 if (j == 100) {
 Distance_Avg = Distance_Sum / (100);

 j = 0;
 Distance_Sum = 0;
 }

 //	if(Distance_Avg == 10)
 //	{
 //		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);  //정지
 //		distance_save=Distance_Avg;
 //	}

 }
 */

uint32_t save_X_GO_IN = 0;
uint32_t save_X_BACK_IN = 0;
uint32_t save_Y_GO_IN = 0;
uint32_t save_Y_BACK_IN = 0;

uint32_t save_X_IN = 0;




uint32_t save_X_OUT = 0;


uint32_t save_Y_IN = 0;
uint32_t save_Y_OUT = 0;

uint32_t save_X_avg;

extern uint8_t vt_start;  //전
extern uint8_t vt_start_Y;  //전

extern uint8_t position_start;
extern uint8_t position_start_Y;
extern uint32_t encoder_count_x;

extern float speed_sensor_X;

int photo_sensor_flag_x = 0; //그래프 용도
int photo_sensor_flag_y = 0; //그래프 용도

extern float speed_sensor_X_vt;
extern float speed_sensor_X_pid;
extern float speed_input_X_vt;

extern float maxV;

extern uint16_t i_v_t_count;

extern float maxV;
extern float fallingN;
extern float fallingN_;
extern float raisingN;
extern float staticN;
extern float orderN;

extern uint8_t v_t_dir_back_X_flag;  //0: 전진 , 1: 후진
extern uint8_t v_t_dir_back_Y_flag;  //0: 전진 , 1: 후진

extern float RPM_X, RPM_Y;

extern int x_go;
extern int x_back;

extern int y_go;
extern int y_back;

extern int RPM_X_go;
extern int RPM_X_back;

extern int RPM_Y_go;
extern int RPM_Y_back;
extern int downpart_run;

extern bool photo_X_go;
extern bool photo_X_back;
extern bool photo_Y_go;
extern bool photo_Y_back;

void photo_sensor_1(void) {

	static int sensor_status = 0;
	static int edge_status = 0;

	sensor_status++;

	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)) {
		edge_status = 0;
//		if (sensor_status == 2) {

			if (i_v_t_count > raisingN + staticN) {

		//		if (RPM_X > 0 && RPM_X_back != 1) {
					if (RPM_X > 0 && v_t_dir_back_X_flag ==0) {
						if (sensor_status == 2) {
					RPM_X_go = 1;

					photo_sensor_flag_x = 1; //그래프 용도

					save_X_GO_IN = encoder_count_x;

					dc_motor_pid_X_with_photosensor1(save_X_GO_IN);
				//	i_v_t_count = 0;

		//			if (v_t_dir_back_X_flag == 0) {
						Finish_flag.x_go = TRUE; // 전진

						if (tim10_flag >= 20)  // paser 2초 딜레이용도
						{
							paser_flag = TRUE;
						}
						tim10_flag = 0;

						printf("x-go \n\r");
			//		}
					//			else if (v_t_dir_back_X_flag == 1) //별 의미 x
					//			{
					//				Finish_flag.x_back = TRUE; // 후진
					//		paser_flag=1;
					//	printf("x-back \n\r");
					//			}
					TIM3->CCR2 = 0;

					x_go++;

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);

				//	photo_X_go = TRUE;
				}
			}

		}

	} else {
		sensor_status = 0;
		if (sensor_status <= 1) {
			edge_status++;
		}
	}
//	if (edge_status == 2) {

		if (i_v_t_count > raisingN + staticN) {

		//	if (RPM_X < 0 && RPM_X_go != 1) {
				if (RPM_X < 0 && v_t_dir_back_X_flag == 1) {
					if (edge_status == 2) {
				RPM_X_back = 1;

				GPIOB->ODR &= ~1 << 14;
				photo_sensor_flag_x = 1;

				save_X_BACK_IN = encoder_count_x;
				GPIOB->ODR |= 1 << 0;
				dc_motor_pid_X_with_photosensor1(save_X_BACK_IN);
		//		i_v_t_count = 0;


				//			if (v_t_dir_back_X_flag == 0)  //별 의미 x
				//				{
				//				Finish_flag.x_go = TRUE; // 전진
				//	paser_flag=1;

				//		printf("x-go \n\r");
				//				}

				//			else
		//		if (v_t_dir_back_X_flag == 1) {
					Finish_flag.x_back = TRUE; // 후진

					if (tim10_flag >= 20)  // paser 2초 딜레이용도
							{
						paser_flag = TRUE;
					}
					tim10_flag = 0;
				printf("x-back \n\r");
		//		}
				TIM3->CCR2 = 0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);

				x_back++;

			//	photo_X_back = TRUE;
			}
		}

	}

}
int photo2 = 0;

/*
 void photo_sensor_2(void) {

 static int sensor_status = 0;
 static int edge_status = 0;
 extern uint32_t encoder_count_y;

 sensor_status++;

 if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)) {
 edge_status = 0;


 if (sensor_status == 2)                         // 이상하긴 한데 동작은 됨
 {
 photo2++;
 printf("y-go \n\r");

 }

 } else {
 sensor_status = 0;
 if (sensor_status <= 1) {
 edge_status++;
 }
 }
 if (edge_status == 2) {

 }
 //	}
 //	printf("%d ,. %d  ,   %d\n\r",save_X,save_Y,sensor_status);
 }
 */

/*
 void photo_sensor_2(void) {

 static int sensor_status = 0;
 static int edge_status = 0;
 extern uint32_t encoder_count_y;

 sensor_status++;

 if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)) {
 edge_status = 0;

 if (i_v_t_count >= raisingN + staticN)      // 여기 순서 변경
 {
 if (sensor_status == 2)                         // 이상하긴 한데 동작은 됨
 {
 if (RPM_Y > 0 && RPM_Y_back !=1 && RPM_Y_go !=1) {
 RPM_Y_go=1;
 GPIOB->ODR &= ~1 << 14;

 photo_sensor_flag_y = 1;

 save_Y_IN = encoder_count_y;
 GPIOB->ODR |= 1 << 0;
 dc_motor_pid_Y_with_photosensor2(save_Y_IN);

 if (v_t_dir_back_Y_flag == 0)
 {	Finish_flag.y_go = TRUE; // 전진
 paser_flag=1;
 printf("y-go \n\r");
 }
 else if (v_t_dir_back_Y_flag == 1)
 {	Finish_flag.y_back = TRUE; // 후진
 //	paser_flag=1;
 //		printf("y-back \n\r");
 }
 TIM3->CCR1 = 0;

 y_go++;

 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
 }
 }

 }

 } else {
 sensor_status = 0;
 if (sensor_status <= 1) {
 edge_status++;
 }
 }
 if (i_v_t_count >= raisingN + staticN) {
 if (edge_status == 2) {
 if (RPM_Y < 0 && RPM_Y_go!=1  && RPM_Y_back !=1) {
 RPM_Y_back=1;
 GPIOB->ODR &= ~1 << 14;

 photo_sensor_flag_y = 1;

 save_Y_IN = encoder_count_y;
 GPIOB->ODR |= 1 << 0;
 dc_motor_pid_Y_with_photosensor2(save_Y_IN);

 if (v_t_dir_back_Y_flag == 0)
 {
 Finish_flag.y_go = TRUE; // 전진
 //		paser_flag=1;
 //		printf("y-go \n\r");
 }
 else if (v_t_dir_back_Y_flag == 1)
 {
 Finish_flag.y_back = TRUE; // 후진
 paser_flag=1;

 printf("y-back \n\r");
 }
 TIM3->CCR1 = 0;

 y_back++;

 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
 }
 }
 }
 //	printf("%d ,. %d  ,   %d\n\r",save_X,save_Y,sensor_status);
 }
 */
extern int DC_Y_go_sensor_flag;
extern int DC_Y_back_sensor_flag;
void photo_sensor_2(void) {

	static int sensor_status = 0;
	static int edge_status = 0;
	extern uint32_t encoder_count_y;

	sensor_status++;

	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)) {
		edge_status = 0;

		if (i_v_t_count > raisingN + staticN)      // 여기 순서 변경
				{
			if (sensor_status == 2 && RPM_Y != 0)               // 이상하긴 한데 동작은 됨
					{
				//	if (RPM_Y_back !=1 && RPM_Y_go !=1) {
				//		RPM_Y_go=1;
//				GPIOB->ODR &= ~1 << 14;
//
//				photo_sensor_flag_y = 1;
//
//				save_Y_GO_IN = encoder_count_y;
//				GPIOB->ODR |= 1 << 0;
//				dc_motor_pid_Y_with_photosensor2(save_Y_GO_IN);

				if (RPM_Y > 0 && v_t_dir_back_Y_flag == 0)  //전진일떄
					{
					//					RPM_Y_go=1;
//						Finish_flag.y_go = TRUE; // 전진
//						paser_flag=1;
					printf("y-go \n\r");
//
//						y_go++;
	//				i_v_t_count = 0;

					Finish_flag.y_go = TRUE; // 전진

					y_go++;


					GPIOB->ODR &= ~1 << 14;

									photo_sensor_flag_y = 1;

									save_Y_GO_IN = encoder_count_y;
									GPIOB->ODR |= 1 << 0;
									dc_motor_pid_Y_with_photosensor2(save_Y_GO_IN);




					if (tim10_flag >= 20)  // paser 2초 딜레이용도
							{
						paser_flag = TRUE;
					}
					tim10_flag = 0;

					//					DC_Y_go_sensor_flag =1 ;
					//					photo_Y_go =TRUE;
				} else if (RPM_Y < 0 && v_t_dir_back_Y_flag == 1) {
					//					RPM_Y_back=1;

//						Finish_flag.y_back = TRUE; // 후진
//						paser_flag=1;
					printf("y-back \n\r");
//						y_back++;

					//				DC_Y_back_sensor_flag =1 ;
					//				photo_Y_back =TRUE;

		//			i_v_t_count = 0;

					Finish_flag.y_back = TRUE; // 후진

					y_back++;



					GPIOB->ODR &= ~1 << 14;

									photo_sensor_flag_y = 1;

									save_Y_BACK_IN = encoder_count_y;
									GPIOB->ODR |= 1 << 0;
									dc_motor_pid_Y_with_photosensor2(save_Y_BACK_IN);





					if (tim10_flag >= 20)  // paser 2초 딜레이용도
					{
						paser_flag = TRUE;
					}
					tim10_flag = 0;

				}
				TIM3->CCR1 = 0;

				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
				//	}
			}

		}

	} else {
		sensor_status = 0;
		if (sensor_status <= 1) {
			edge_status++;
		}
	}
	/*	if (i_v_t_count >= raisingN + staticN) {
	 if (edge_status == 2) {
	 if (RPM_Y < 0 && RPM_Y_go!=1  && RPM_Y_back !=1) {
	 RPM_Y_back=1;
	 GPIOB->ODR &= ~1 << 14;

	 photo_sensor_flag_y = 1;

	 save_Y_IN = encoder_count_y;
	 GPIOB->ODR |= 1 << 0;
	 dc_motor_pid_Y_with_photosensor2(save_Y_IN);

	 if (v_t_dir_back_Y_flag == 0)
	 {
	 Finish_flag.y_go = TRUE; // 전진
	 //		paser_flag=1;
	 //		printf("y-go \n\r");
	 }
	 else if (v_t_dir_back_Y_flag == 1)
	 {
	 Finish_flag.y_back = TRUE; // 후진
	 paser_flag=1;

	 printf("y-back \n\r");
	 }
	 TIM3->CCR1 = 0;

	 y_back++;

	 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
	 }
	 }
	 }*/
	//	printf("%d ,. %d  ,   %d\n\r",save_X,save_Y,sensor_status);
}

/*
int max_test = 0;
void ball_limit_sw_max(void) {

	static int sensor_status = 0;
	static int edge_status = 0;
	extern uint32_t encoder_count_y;

	sensor_status++;

	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 0);  //방향 위

	if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == 0)) {
		edge_status = 0;

		//	if ((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0) == 1)
		//			&& (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1) && (RPM_X == 0)
		//			&& (RPM_Y == 0) && (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4) == 1)) {

		if (sensor_status == 2)                        //라이징
				{

			HAL_TIM_OC_Stop_IT(&htim11, TIM_CHANNEL_1);  // 볼스크류 스텝모터 타이머 정지
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //정지
			Finish_flag.ball_high = TRUE;
			//		paser_flag=1;
			max_test++;

		}
		//}
	} else {
		sensor_status = 0;
		if (sensor_status <= 1) {
			edge_status++;
		}
	}

	if (edge_status == 2) {
		Finish_flag.ball_high = FALSE;
	}

	//	printf("%d ,. %d  ,   %d\n\r",save_X,save_Y,sensor_status);
}

*/

extern int ball_min_test;

extern int ballscrew_up_run;
extern int ballscrew_down_run;

void ball_limit_sw_min(void) {

	static int sensor_status = 0;
	static int edge_status = 0;
	extern uint32_t encoder_count_y;

	sensor_status++;

	if ((HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2) == 0)) {
		edge_status = 0;

		//	if ((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0) == 1)
		//			&& (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1) && (RPM_X == 0)
		//			&& (RPM_Y == 0) && (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4) == 1)) {

		//아래방향일떄만
		//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);  //방향 아래
		if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3) == 1) {
			if (sensor_status == 2)                        //라이징
					{

				HAL_TIM_OC_Stop_IT(&htim11, TIM_CHANNEL_1);  // 볼스크류 스텝모터 타이머 정지
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //정지
				Finish_flag.ball_low = TRUE;

				if(tim10_flag >= 20)  // paser 2초 딜레이용도
				{
					paser_flag=TRUE;
				}
				tim10_flag=0;

				//printf("ball_low \n\r");
				ball_min_test++;

				ballscrew_down_run = 0;
			}
		}

	} else {
		sensor_status = 0;
		if (sensor_status <= 1) {
			edge_status++;
		}
	}

	if (edge_status == 2) {
		Finish_flag.ball_low = FALSE;
	}
}

extern int down_high_test;

int down_max_flag = 0;
void down_sw_max(void) {

	static int sensor_status = 0;
	static int edge_status = 0;
	extern uint32_t encoder_count_y;

	sensor_status++;

	if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)) {
		edge_status = 0;

		//	if (
		//	(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0) == 1)
		//				&& (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1)
		//			&& (RPM_X ==0) && (RPM_Y ==0)
		//		)

		//	(RPM_X == 0) && (RPM_Y == 0)) {

		if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14) == 1) {  //위로 올라갈 떄

			if (sensor_status == 2)                        //라이징
					{
				HAL_TIM_OC_Stop_IT(&htim13, TIM_CHANNEL_1); // 볼스크류 스텝모터 타이머 정지
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //정지
				Finish_flag.downpart_high = TRUE;

				if(tim10_flag >= 2)  // paser 2초 딜레이용도
				{
					paser_flag=TRUE;
				}
				tim10_flag=0;

				down_high_test++;

				down_max_flag = 1;

				//printf("downpart_high \n\r");
				//	downpart_run=0;
			}
		}
		//}
	} else {
		sensor_status = 0;
		if (sensor_status <= 1) {
			edge_status++;
		}
	}

	if (edge_status == 2) {
		//Finish_flag.downpart_high = FALSE;
	}
}

extern uint32_t step_pulse_count_tim13;

//void downpart_distance(void) {
//
//	static int sensor_status = 0;
//	static int edge_status = 0;
//	extern uint32_t encoder_count_y;
//
//	sensor_status++;
//
//	if (Distance_Avg < 10) {
//		edge_status = 0;
//		if (sensor_status == 2) {
//			HAL_TIM_OC_Stop_IT(&htim13, TIM_CHANNEL_1);  // 타이머 정지
//			distance_save = Distance_Avg;
//			step_pulse_count_tim13 = 0;
//		}
//
//	} else {
//		sensor_status = 0;
//		if (sensor_status <= 1) {
//			edge_status++;
//		}
//	}
//	if (edge_status == 2) {
////
//	}
//
//}

//else //Finish_flag.downpart_low=FALSE;
//extern uint32_t downpart_step;
//void downpart_distance_step_dintance(float STEP, float DISTENSE) {
//
//	static int sensor_status = 0;
//	static int edge_status = 0;
//
//	sensor_status++;
//
//	if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14) == 0) {  // 하강일떄만
//
//		if (step_pulse_count_tim13 >= STEP * 2) {
//			edge_status = 0;
//
//			if (sensor_status == 2)                        //라이징
//					{
//				HAL_TIM_OC_Stop_IT(&htim13, TIM_CHANNEL_1);  // 타이머 정지
//				Finish_flag.downpart_low = TRUE;
//			}
//			//}
//		} else {
//			sensor_status = 0;
//			if (sensor_status <= 1) {
//				edge_status++;
//			}
//		}
//	}
//
//	if (edge_status == 2) {
//
//	}
//
//}
