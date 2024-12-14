#include "pid.h"
#include "main.h"



//#include "stm32f4xx_hal.h"
extern uint32_t tim4_encoder_overflow;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern uint8_t position_start;
extern uint8_t position_start_Y;

extern uint32_t encoder_count_x;
extern uint16_t encoder_count_y;

////////////////////////////////////////////////////////////Y
float input_position_Y;

float err_Y;

float p_kp_Y = 800; //300 > 800
float p_ki_Y;
float p_kd_Y = 15;

float s_kp_Y = 1800;  //v_t:  1800
float s_ki_Y = 0;
float s_kd_Y;

//float c_kp = 1;
//float c_ki = 0;
//float c_kd;

float P_KP_Y;
float P_KI_Y;
double P_KD_Y;
double p_PID_Y;
double err_d_Y;
float previous_err_Y;

double err_d_s_Y;
float previous_err_s_Y;

float err_s_Y;
float err_sum_s_Y;
float err_sum_Y;
float S_KP_Y;
float S_KI_Y;
double S_KD_Y;
double s_PID_Y;


float speed_sensor_Y;
float speed_input_Y;

float errorgap_p_Y;
float errorgap_s_Y;
////////////////////////////////////////////////////// X

float input_position_X;

float input_current;

float err_X;

float p_kp_X = 300;
float p_ki_X;
float p_kd_X = 15;

float s_kp_X = 1800;
float s_ki_X = 0;
float s_kd_X;

float c_kp_X = 1;
float c_ki_X = 0;
float c_kd_X;

float P_KP_X;
float P_KI_X;
double P_KD_X;
double p_PID_X;
double err_d_X;
float previous_err_X;

double err_d_s_X;
float previous_err_s_X;

float err_s_X;
float err_sum_s_X;
float err_sum_X;
float S_KP_X;
float S_KI_X;
double S_KD_X;
double s_PID_X;

float err_c_X;
float err_sum_c_X;

float C_KP_X;
float C_KI_X;
double C_KD_X;
double c_PID_X;

double err_d_c_X;
float previous_err_c_X;


extern float current_A;

float speed_sensor_X;
float speed_input_X;

float current_sensor_X;
float current_input_X;

extern float current_A_floor_X;

float errorgap_p_X;
float errorgap_s_X;
float errorgap_c_X;




//float errorgap_c;



////////////////////////////////////////////v_t 그래프

//float targetS = 60;

uint16_t i_v_t_count = 0;
float orderArr[1500];
float orderN;
float raisingN;
float fallingN;
float fallingN_;
float staticN;
float maxV=23, maxV_ = 0,localV =0, maxA=50, maxA_=45, maxA__=5;
float radio =11;//최대속도 27
int v_t_static_flag = 0;

extern uint8_t sensor_flag;

void v_t_graph(float targetS) {



	float samplingHz, samplingT;
	float tAccR, tAccF;
	float tStatic;
	float tTotal;

	float vTmp;

	//maxV = 17; //cm/s
	//maxA = 30; //cm/s^2
	samplingHz = 100; //100Hz
	samplingT = 1 / samplingHz; //0.01s

//targetS = 30; //30cm

	tAccR = maxV / maxA;
	tAccF = maxV / (maxA_+maxA__*radio);

	if(targetS/(tAccR+tAccF) > maxV){
		maxV_ = maxA__*radio*tAccF;

		tStatic = targetS/maxV - tAccR/2 - tAccF/2 - maxV_/(2*maxV)*tAccF*(1+radio);
		v_t_static_flag =1;
	}else{
		float K = pow(maxA_+maxA__*radio,2)/(2*maxA)+(maxA_+maxA__*radio)/2+maxA__*radio*radio/2+maxA__*radio/2;
		tAccF = sqrt(targetS/K);
		tAccR = tAccF*(maxA_+maxA__*radio)/maxA;

		tStatic = 0;
		v_t_static_flag =0;
	}

	tTotal = tAccR+tAccF*(1+radio) + tStatic;

	orderN = (int) (tTotal / samplingT);
	raisingN = (int) (tAccR / samplingT);
	fallingN = (int) (tAccF / samplingT);
	fallingN_ = (int) (tAccF*radio / samplingT);
	staticN = orderN - raisingN - fallingN - fallingN_;

	//orderArr = (float*) malloc(sizeof(float) * orderN);
	vTmp = 0;


		for (i_v_t_count = 0; i_v_t_count < orderN; i_v_t_count++) {

		orderArr[i_v_t_count] = vTmp;

		if (i_v_t_count < raisingN) {
			vTmp = vTmp + maxA * samplingT;
		}

		else if (raisingN+staticN+fallingN >i_v_t_count && i_v_t_count >= raisingN + staticN) {
			vTmp = vTmp - maxA_ * samplingT;
		}else if(i_v_t_count >= raisingN+staticN+fallingN){
			vTmp = vTmp - maxA__ * samplingT;
		}



	}



	i_v_t_count = 0;
}


///////////////////////////////////////////////////////  V T 시작 함수

uint8_t v_t_dir_back_X_flag = 0;  //0: 전진 , 1: 후진
uint8_t v_t_dir_back_Y_flag = 0;  //0: 전진 , 1: 후진

extern int RPM_X_go;
extern int RPM_X_back;

extern int RPM_Y_go;
extern int RPM_Y_back;


extern uint8_t vt_start;  //전
extern uint8_t vt_start_Y;  //전

//extern int DC_X_now_posiotion;
extern int DC_Y_now_posiotion;

extern float targetS_X_GO;
extern float targetS_X_BACK  ;

extern float targetS_Y_GO ;
extern float targetS_Y_BACK  ;

extern float targetS_X_GO2  ;
extern float targetS_X_BACK2  ;

int algo_vt_y_go=0;
int algo_vt_y_back=0;



extern uint32_t save_X_GO_IN;
extern uint32_t save_X_BACK_IN;
extern uint32_t save_Y_GO_IN;
extern uint32_t save_Y_BACK_IN;


void v_t_graph_DIR_GO_X(float targetS)               // X축 v_t 전진
{
//	save_X_GO_IN=0;

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);


	//if(DC_X_now_posiotion >= 0 && DC_X_now_posiotion <=1 )
//	{
		RPM_X_go=0;


		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);  //start
		TIM2->CNT = 1000000;
		vt_start = 1;
		v_t_dir_back_X_flag = 0;  //0: 전진 , 1: 후진

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);  //방향 go

	//	if(targetS == targetS_X_GO2  &&  DC_X_now_posiotion==1) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //stop

		v_t_graph(targetS);

	//	if(targetS == targetS_X_GO) DC_X_now_posiotion++;
	//	else if(targetS == targetS_X_GO2 && DC_X_now_posiotion == 0)
	//	{
	//		DC_X_now_posiotion+=2;
	//		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);  //start
	//	}


	//}

}
void v_t_graph_DIR_BACK_X(float targetS)         //  X축 v_t 후진
{
//	save_X_BACK_IN=0;

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

//	if(DC_X_now_posiotion >= 1 && DC_X_now_posiotion <=2)
//	{


		RPM_X_back=0;

		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);  //start
		TIM2->CNT = 1000000;
		vt_start = 1;
		v_t_dir_back_X_flag = 1;  //0: 전진 , 1: 후진

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);  //방향 back

	//	if(targetS == targetS_X_BACK2 &&  DC_X_now_posiotion==1 ) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //stop
		v_t_graph(targetS);

	//	if(targetS == targetS_X_BACK) DC_X_now_posiotion--;
	//	else if(targetS == targetS_X_BACK2 && DC_X_now_posiotion == 2) DC_X_now_posiotion-=2;
	//}
}

void v_t_graph_DIR_GO_Y(float targetS_Y)         // Y축 v_t 전진
{
	algo_vt_y_go++;
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	if (DC_Y_now_posiotion==0)
//	{
		RPM_Y_go=0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);  //start
		TIM4->CNT = 30000;
		//	tim4_encoder_overflow = 1000000 - TIM4->CNT;


		vt_start_Y = 1;
		v_t_dir_back_Y_flag = 0;  //0: 전진 , 1: 후진

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);  //y축 방향 go
		v_t_graph(targetS_Y);

//		DC_Y_now_posiotion++;
//	}
}
void v_t_graph_DIR_BACK_Y(float targetS_Y)          // Y축 v_t 후진
{
	algo_vt_y_back++;

	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//if (DC_Y_now_posiotion==1)
//	{
		RPM_Y_back=0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);  //start
		TIM4->CNT = 30000;
		//	tim4_encoder_overflow = 1000000 - TIM4->CNT;


		vt_start_Y = 1;
		v_t_dir_back_Y_flag = 1;  //0: 전진 , 1: 후진

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);  //y축 방향 back
		v_t_graph(targetS_Y);

	//	DC_Y_now_posiotion--;
//	}

}








///////////////////////////  DC PID  엔코더 입력 함수
void dc_motor_pid_X(float input_encoder)  //X축 pid
{
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);  //start
	position_start = 1;
	TIM2->CNT = 1000000;

	input_position_X = 1000000 + input_encoder;
	err_sum_X = 0;
	err_sum_s_X = 0;

}



void dc_motor_pid_Y(float input_encoder)  //Y축 pid
{
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);  //start
	position_start_Y = 1;
	TIM4->CNT = 30000;
//	tim4_encoder_overflow = 1000000 - TIM4->CNT;

	input_position_Y = 30000 + input_encoder;
	err_sum_Y = 0;
	err_sum_s_Y = 0;

}


void dc_motor_pid_X_with_photosensor1(uint32_t input_encoder)  //X축 pid
{
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);  //start
	position_start = 1;
//	TIM2->CNT = 1000000;


		input_position_X = input_encoder;



	err_sum_X = 0;
	err_sum_s_X = 0;

}
void dc_motor_pid_Y_with_photosensor2(uint32_t input_encoder)  //y축 pid
{
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);  //start
	position_start_Y = 1;


		input_position_Y = input_encoder;



	err_sum_Y = 0;
	err_sum_s_Y = 0;

}








/////////////////////////////////////////// DC 모터  속도측정

float RPM_X, RPS_X, Omega_X;
float RPM_Y, RPS_Y, Omega_Y;
extern float current_A_X;

void dc_motor_RPM(void)  //RPM측정 x,y 둘 다
{
	encoder_count_x = TIM4->CNT ;

	static uint32_t ENCODER_NEW_X = 0, ENCODER_OLD_X;
	static uint32_t ENCODER_NEW_Y = 0, ENCODER_OLD_Y;

	//////////////////////////////////////////////DC motor1 X
	ENCODER_OLD_X = ENCODER_NEW_X;
	ENCODER_NEW_X = TIM2->CNT;

	RPM_X = (abs(ENCODER_NEW_X - ENCODER_OLD_X) * 60) / 0.01 / 68 / 512;
	RPS_X = RPM_X / 60;
	Omega_X = RPS_X * 2 * M_PI;

	if (( GPIOE->ODR & 1 << 0) == 0)	// if  앞으로
			{
		RPM_X = RPM_X;
		RPS_X = RPS_X;
		Omega_X = Omega_X;

		current_A_X = current_A_X;
	} else {
		RPM_X = -RPM_X;
		RPS_X = -RPS_X;
		Omega_X = -Omega_X;

		current_A_X = -current_A_X;
	}
	//////////////////////////////////////////////////////////DC motor2 Y
//	encoder_count_y = TIM4->CNT + tim4_encoder_overflow;
	ENCODER_OLD_Y = ENCODER_NEW_Y;
	ENCODER_NEW_Y = TIM4->CNT;

	RPM_Y = (abs(ENCODER_NEW_Y - ENCODER_OLD_Y) * 60) / 0.01 / 21.25 / 512;
	RPS_Y = RPM_Y / 60;
	Omega_Y = RPS_Y * 2 * M_PI;

	if (( GPIOB->ODR & 1 << 10) == 0)	// if  PB10==0  : go
			{
		RPM_Y = RPM_Y;
		RPS_Y = RPS_Y;
		Omega_Y = Omega_Y;

		//current_A = current_A;
	} else {
		RPM_Y = -RPM_Y;
		RPS_Y = -RPS_Y;
		Omega_Y = -Omega_Y;

		//current_A = -current_A;
	}

}

/*                 VT control                    */
uint8_t vt_finish_flag=0;



extern int photo_sensor_flag_x;
extern int photo_sensor_flag_y;


float s_kp_X_vt = 7000;
float s_ki_X_vt = 0;
float s_kd_X_vt;

double err_d_s_X_vt;
float previous_err_s_X_vt;

float err_s_X_vt;
float err_sum_s_X_vt;
float err_sum_X_vt;
float S_KP_X_vt;
float S_KI_X_vt;
double S_KD_X_vt;
double s_PID_X_vt;

float speed_sensor_X_vt;
float speed_input_X_vt;
float errorgap_s_X_vt;



float s_kp_Y_vt = 5000;
float s_ki_Y_vt = 0;
float s_kd_Y_vt;

double err_d_s_Y_vt;
float previous_err_s_Y_vt;

float err_s_Y_vt;
float err_sum_s_Y_vt;
float err_sum_Y_vt;
float S_KP_Y_vt;
float S_KI_Y_vt;
double S_KD_Y_vt;
double s_PID_Y_vt;

float speed_sensor_Y_vt;
float speed_input_Y_vt;
float errorgap_s_Y_vt;


void VT_control_X(void) {


	if (v_t_dir_back_X_flag == 1)  //후진이면
	{
		speed_sensor_X_vt = ((-RPM_X / 60) * 3.141592 * 9.5);  //  cm/s
	} else {
		speed_sensor_X_vt = (RPM_X / 60) * 3.141592 * 9.5;  //  cm/s
	}

	if (i_v_t_count <= orderN) {
		speed_input_X_vt = orderArr[i_v_t_count++];

	}

	errorgap_s_X_vt = speed_input_X_vt - speed_sensor_X_vt - err_s_X_vt;
	err_s_X_vt = speed_input_X_vt - speed_sensor_X_vt;
	err_sum_s_X_vt += err_s_X_vt * 0.01;

	if ((i_v_t_count >= raisingN) && (i_v_t_count <= (staticN+ raisingN))) {
		s_ki_X_vt = 50;
	} else
		s_ki_X_vt = 0;

	if (i_v_t_count >= orderN) {
		for (int x = orderN; x <= 3000; x++) {
			orderArr[x] = 0;
		}

	}

	if (s_PID_X_vt == 0) {
		err_sum_s_X_vt = 0;
	}
	S_KP_X_vt = err_s_X_vt * s_kp_X_vt;
	S_KI_X_vt = err_sum_s_X_vt * s_ki_X_vt;

	S_KD_X_vt = s_kd_X_vt * errorgap_s_X_vt / 0.01;

	s_PID_X_vt = S_KP_X_vt + S_KI_X_vt + S_KD_X_vt;

	if (s_PID_X_vt >= 8500) {
		s_PID_X_vt = 8500;
	} else if (s_PID_X_vt <= -8500) {
		s_PID_X_vt = -8500;
	}

	if(photo_sensor_flag_x !=1)
	{
		if (s_PID_X_vt > 0)
		{
			//	GPIOE->ODR &= ~1 << 0; // go
			TIM3->CCR1 = s_PID_X_vt;

		}
		else if (s_PID_X_vt < 0)
		{
			//	GPIOE->ODR |= 1 << 0; // back
			s_PID_X_vt = -s_PID_X_vt;
			TIM3->CCR1 = -s_PID_X_vt;

		}
		else
		{
			s_PID_X_vt = 0;
			TIM3->CCR1 = s_PID_X_vt;

			if (i_v_t_count >= orderN)
			{
				vt_start = 0;

			}

		}
	}

}
void VT_control_Y(void) {


	if (v_t_dir_back_Y_flag == 1)  //후진이면
	{
		speed_sensor_Y_vt = ((-RPM_Y / 60) * 3.141592 * 9.5);  //  cm/s
	//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);  //y축 방향 back
	}
	else
	{
		speed_sensor_Y_vt = (RPM_Y / 60) * 3.141592 * 9.5;  //  cm/s
	}

	if (i_v_t_count <= orderN)
	{
		speed_input_Y_vt = orderArr[i_v_t_count++];
	}
	errorgap_s_Y_vt = speed_input_Y_vt - speed_sensor_Y_vt - err_s_Y_vt;
	err_s_Y_vt = speed_input_Y_vt - speed_sensor_Y_vt;
	err_sum_s_Y_vt += err_s_Y_vt * 0.01;

	if ((i_v_t_count >= raisingN) && (i_v_t_count <= (staticN+ raisingN))) {
		s_ki_Y_vt = 50;
	} else
		s_ki_Y_vt = 0;

	if (i_v_t_count >= orderN) {
		for (int x = orderN; x <= 3000; x++) {
			orderArr[x] = 0;
		}

	}

	if (s_PID_Y_vt == 0) {
		err_sum_s_Y_vt = 0;
	}
	S_KP_Y_vt = err_s_Y_vt * s_kp_Y_vt;
	S_KI_Y_vt = err_sum_s_Y_vt * s_ki_Y_vt;

	S_KD_Y_vt = s_kd_Y_vt * errorgap_s_Y_vt / 0.01;

	s_PID_Y_vt = S_KP_Y_vt + S_KI_Y_vt + S_KD_Y_vt;

	if (s_PID_Y_vt >= 8500)
	{
		s_PID_Y_vt = 8500;
	}
	else if (s_PID_Y_vt <= -8500)
	{
		s_PID_Y_vt = -8500;
	}


if(photo_sensor_flag_y !=1 )
	{
		if (s_PID_Y_vt > 0) {
			//	GPIOE->ODR &= ~1 << 0; // go
			TIM3->CCR2 = s_PID_Y_vt;

		}
		else if (s_PID_Y_vt < 0)
		{
			//	GPIOE->ODR |= 1 << 0; // back
			s_PID_Y_vt = -s_PID_Y_vt;
			TIM3->CCR2 = -s_PID_Y_vt;

		} else
		{
			s_PID_Y_vt = 0;
			TIM3->CCR2 = s_PID_Y_vt;

			if (i_v_t_count >= orderN) {
				vt_start_Y = 0;
			}

		}
	}
}

extern int PID_Y_finish;
extern int PID_X_finish;

#define SOURCE 10 //  1 2 3,8 9 폐기
//4 위치 다른방식  ok
//5: 위치(4)+속도
//6: 위치4+속도5+전류

//7: 속도만(v_t) : tim6 샘플링 0.001에서 0.01로 변경

//10 : 5번 개선   > 5번 보다 좋아짐  이거 사용
//11 : 10+전휴 test
void position_pid_x(void) {
#if SOURCE == 1

#elif SOURCE == 4
	//////////////////////////////////위치

	TIM6->ARR = 1000 - 1;  //ARR  : 샘플링 주기 1ms에서

		errorgap_p_X = input_position_X - TIM2->CNT - (int)err_X;

			err_X = input_position_X - TIM2->CNT;


		err_sum_X += (int)err_X * 0.001;

		if (p_PID_X == 0) {
			err_sum_X = 0;
		}

		P_KP_X = (int)err_X * p_kp_X;
		P_KI_X = err_sum_X * p_ki_X;

		//	err_d_X = (err_X - previous_err_X) / 0.0001;   //1ms
		//	previous_err_X = err_X;

		P_KD_X = p_kd_X * errorgap_p_X / 0.001;

		p_PID_X = P_KP_X + P_KI_X + P_KD_X;

		if (p_PID_X >= 11000) {
			p_PID_X = 11000;
		} else if (p_PID_X <= -11000) {
			p_PID_X = -11000;
		}

		if (p_PID_X > 0) {
			GPIOE->ODR &= ~1 << 0; // go
			TIM3->CCR1 = p_PID_X;
		} else if (p_PID_X < 0) {
			GPIOE->ODR |= 1 << 0; // back
			p_PID_X = -p_PID_X;
			TIM3->CCR1 = p_PID_X;
		} else {
			p_PID_X = 0;
			TIM3->CCR1 = p_PID_X;

		}




#elif SOURCE == 5
		//////////////////////////////////위치

			//		p_kp = 250;
			//	p_ki = 0;
			//	p_kd = 15;
			//		s_kp = 2;
			//	s_ki = 0;
			//	s_kd = 0;

		//참고	  errorGap = target - current - realError;
		//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
		//	    accError += realError;

			errorgap_p_X = input_position_X - TIM2->CNT - err_X;
			err_X = input_position_X - TIM2->CNT;
			err_sum_X += err_X * 0.001;

			if (p_PID_X == 0) {
				err_sum_X = 0;
			}

			P_KP_X = err_X * p_kp_X;
			P_KI_X = err_sum_X * p_ki_X;

			//	err_d_X = (err_X - previous_err_X) / 0.0001;   //1ms
			//	previous_err_X = err_X;

			P_KD_X = p_kd_X * errorgap_p_X / 0.001;

			p_PID_X = P_KP_X + P_KI_X + P_KD_X;

			if (p_PID_X >= 11000) {
				p_PID_X = 11000;
			} else if (p_PID_X <= -11000) {
				p_PID_X = -11000;
			}
		/////////////////////////////////////속도

			//참고	  errorGap = target - current - realError;
			//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
			//	    accError += realError;

			speed_sensor_X = (RPM_X *34 * 512)/60;
			speed_input_X = p_PID_X;
			errorgap_s_X = speed_input_X - speed_sensor_X - err_s_X;
			err_s_X = speed_input_X - speed_sensor_X;
			err_sum_s_X += err_s_X * 0.001;

			if (s_PID_X == 0) {
				err_sum_s_X = 0;
			}
			S_KP_X = err_s_X * s_kp_X;
			S_KI_X = err_sum_s_X * s_ki_X;

			//	err_d = (err - previous_err) / 0.0001;   //1ms
			//	previous_err = err;

			S_KD_X = s_kd_X * errorgap_s_X / 0.001;

			s_PID_X = S_KP_X + S_KI_X + S_KD_X;

			if (s_PID_X >= 6000) {
				s_PID_X = 6000;
			} else if (s_PID_X <= -6000) {
				s_PID_X = -6000;
			}

			if (s_PID_X > 0) {
				GPIOE->ODR &= ~1 << 0; // go
				TIM3->CCR1 = s_PID_X;
			} else if (s_PID_X < 0) {
				GPIOE->ODR |= 1 << 0; // back
				s_PID_X = -s_PID_X;
				TIM3->CCR1 = s_PID_X;
			} else {
				s_PID_X = 0;
				TIM3->CCR1 = s_PID_X;
			}




#elif SOURCE == 6
	//////////////////////////////////위치

	//		p_kp = 10;
	//	p_ki = 1;
	//	p_kd = 0.02;

//참고	  errorGap = target - current - realError;
//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
//	    accError += realError;

	errorgap_p = input_position_X - TIM2->CNT - err;
	err = input_position_X - TIM2->CNT;
	err_sum += err * 0.001;

	if (p_PID == 0) {
		err_sum = 0;
	}

	P_KP = err * p_kp;
	P_KI = err_sum * p_ki;

	//	err_d = (err - previous_err) / 0.0001;   //1ms
	//	previous_err = err;

	P_KD = p_kd * errorgap_p / 0.001;

	p_PID = P_KP + P_KI + P_KD;

	if (p_PID >= 12000) {
		p_PID = 12000;
	} else if (p_PID <= -12000) {
		p_PID = -12000;
	}
/////////////////////////////////////속도

	//		s_kp = 10;
	//	s_ki = 1;
	//	s_kd = 0.02;

	//참고	  errorGap = target - current - realError;
	//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
	//	    accError += realError;

	speed_sensor = (RPM_X / 60 * 34 * 512);
	speed_input = p_PID;
	errorgap_s = speed_input - speed_sensor - err_s;
	err_s = speed_input - speed_sensor;
	err_sum_s += err_s * 0.001;

	if (s_PID == 0) {
		err_sum_s = 0;
	}
	S_KP = err_s * s_kp;
	S_KI = err_sum_s * s_ki;

	//	err_d = (err - previous_err) / 0.0001;   //1ms
	//	previous_err = err;

	S_KD = s_kd * errorgap_s / 0.001;

	s_PID = S_KP + S_KI + S_KD;

	 if (s_PID >= 3000) {
	 s_PID = 3000;
	 } else if (s_PID <= -3000) {
	 s_PID = -3000;
	 }



////////////////////////////    전류

	//		c_kp = ;
	//	c_ki = ;
	//	c_kd = ;

	//참고	  errorGap = target - current - realError;
	//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
	//	    accError += realError;
	current_sensor = (current_A_floor/(3.14) * 34 * 512);
	current_input = s_PID;
	errorgap_c = current_input - current_sensor - err_c;
	err_c = current_input - current_sensor;
	err_sum_c += err_c * 0.001;

	if (c_PID == 0) {
		err_sum_c = 0;
	}
	C_KP = err_c * c_kp;
	C_KI = err_sum_c * c_ki;

	//	err_d = (err - previous_err) / 0.0001;   //1ms
	//	previous_err = err;

	C_KD = c_kd * errorgap_c / 0.001;

	c_PID = C_KP + C_KI + C_KD;



	 if (c_PID >= 8000) {
	 c_PID = 8000;
	 } else if (c_PID <= -8000) {
	 c_PID = -8000;
	 }



	if (c_PID > 40) {
		GPIOE->ODR &= ~1 << 0; // go
		TIM3->CCR1 = c_PID;
	} else if (c_PID < 40) {
		GPIOE->ODR |= 1 << 0; // back
		c_PID = -c_PID;
		TIM3->CCR1 = c_PID;
	} else {
		c_PID = 0;
		TIM3->CCR1 = c_PID;
	}




#elif SOURCE == 7

	/////////////////////////////////////속도

	TIM6->ARR = 10000 - 1;  //ARR  : 샘플링 주기 1ms에서 10ms=0.01초으로 변경


	if(v_t_dir_back_flag==1)  //후진이면
	{
		speed_sensor_X = ((-RPM_X / 60) * 3.141592 * 9.5);  //  cm/s
	}
	else
	{
		speed_sensor_X = (RPM_X / 60) * 3.141592 * 9.5;  //  cm/s
	}

	if(i_v_t_count <= orderN)
	{
		speed_input_X = orderArr[i_v_t_count++];
	}
	errorgap_s_X = speed_input_X - speed_sensor_X - err_s_X;
	err_s_X = speed_input_X - speed_sensor_X;
	err_sum_s_X += err_s_X * 0.01;

	if ((i_v_t_count >= raisingN) && (i_v_t_count <= (orderN - raisingN))) {
		s_ki_X = 50;
	} else
		s_ki_X = 0;


	if (i_v_t_count >= orderN) {
			for (int x = orderN; x <= 3000; x++) {
				orderArr[x] = 0;
			}

	}



	if (s_PID_X == 0) {
		err_sum_s_X = 0;
	}
	S_KP_X = err_s_X * s_kp_X;
	S_KI_X = err_sum_s_X * s_ki_X;

	S_KD_X = s_kd_X * errorgap_s_X / 0.01;

	s_PID_X = S_KP_X + S_KI_X + S_KD_X;

	if (s_PID_X >= 8000) {
		s_PID_X = 8000;
	} else if (s_PID_X <= -8000) {
		s_PID_X = -8000;
	}

	if (s_PID_X > 0) {
		//	GPIOE->ODR &= ~1 << 0; // go
		TIM3->CCR1 = s_PID_X;

	} else if (s_PID_X < 0) {
		//	GPIOE->ODR |= 1 << 0; // back
		s_PID_X = -s_PID_X;
		TIM3->CCR1 = -s_PID_X;

	} else {
		s_PID_X = 0;
		TIM3->CCR1 = s_PID_X;

		if(i_v_t_count >=orderN)
			{position_start = 0;
			}
		//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //stop
	}




#elif SOURCE == 10
	//////////////////////////////////위치

	//TIM6->ARR = 1000 - 1;  //ARR  : 샘플링 주기 1ms에서

	errorgap_p_X = input_position_X - TIM2->CNT - err_X;
	err_X = input_position_X - TIM2->CNT;
	err_sum_X += err_X * 0.001;

	if (p_PID_X == 0) {
		err_sum_X = 0;
	}

	P_KP_X = err_X * p_kp_X;
	P_KI_X = err_sum_X * p_ki_X;

	//	err_d_X = (err_X - previous_err_X) / 0.0001;   //1ms
	//	previous_err_X = err_X;

	P_KD_X = p_kd_X * errorgap_p_X / 0.001;

	p_PID_X = P_KP_X + P_KI_X + P_KD_X;

	if (p_PID_X >= 11000) {
		p_PID_X = 11000;
	} else if (p_PID_X <= -11000) {
		p_PID_X = -11000;
	}
	/////////////////////////////////////속도

	//참고	  errorGap = target - current - realError;
	//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
	//	    accError += realError;

	speed_sensor_X = (RPM_X * 512) / 60;
	//	speed_sensor_X = Omega_X*180/(3.141592);
	speed_input_X = p_PID_X;
	errorgap_s_X = speed_input_X - speed_sensor_X - err_s_X;
	err_s_X = speed_input_X - speed_sensor_X;
	err_sum_s_X += err_s_X * 0.001;

	if (s_PID_X == 0) {
		err_sum_s_X = 0;
	}

	                   //s_kp_X=1;
	S_KP_X = err_s_X * 1;
//	S_KP_X = err_s_X * s_kp_X;

	S_KI_X = err_sum_s_X * s_ki_X;

	//	err_d = (err - previous_err) / 0.0001;   //1ms
	//	previous_err = err;

	S_KD_X = s_kd_X * errorgap_s_X / 0.001;

	s_PID_X = S_KP_X + S_KI_X + S_KD_X;

	if (s_PID_X >= 6000) {
		s_PID_X = 6000;
	} else if (s_PID_X <= -6000) {
		s_PID_X = -6000;
	}

	if (s_PID_X > 0) {
		GPIOE->ODR &= ~1 << 0; // go
		TIM3->CCR1 = s_PID_X;
	} else if (s_PID_X < 0) {
		GPIOE->ODR |= 1 << 0; // back
		s_PID_X = -s_PID_X;
		TIM3->CCR1 = s_PID_X;
	} else {
		s_PID_X = 0;
		TIM3->CCR1 = s_PID_X;
		position_start = 0;
		photo_sensor_flag_x=0;
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //stop

		PID_X_finish = 1;

//		RPM_X_go=0;
//		RPM_X_back=0;
	}

#elif SOURCE == 11
		//////////////////////////////////위치

			//		p_kp = 250;
			//	p_ki = 0;
			//	p_kd = 15;
			//		s_kp = 2;
			//	s_ki = 0;
			//	s_kd = 0;

		//참고	  errorGap = target - current - realError;
		//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
		//	    accError += realError;

			errorgap_p_X = input_position_X - TIM2->CNT - err_X;
			err_X = input_position_X - TIM2->CNT;
			err_sum_X += err_X * 0.001;

			if (p_PID_X == 0) {
				err_sum_X = 0;
			}

			P_KP_X = err_X * p_kp_X;
			P_KI_X = err_sum_X * p_ki_X;

			//	err_d_X = (err_X - previous_err_X) / 0.0001;   //1ms
			//	previous_err_X = err_X;

			P_KD_X = p_kd_X * errorgap_p_X / 0.001;

			p_PID_X = P_KP_X + P_KI_X + P_KD_X;

			if (p_PID_X >= 11000) {
				p_PID_X = 11000;
			} else if (p_PID_X <= -11000) {
				p_PID_X = -11000;
			}
		/////////////////////////////////////속도

			//참고	  errorGap = target - current - realError;
			//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
			//	    accError += realError;

			speed_sensor_X = (RPM_X * 512)/60;
		//	speed_sensor_X = Omega_X*180/(3.141592);
			speed_input_X = p_PID_X;
			errorgap_s_X = speed_input_X - speed_sensor_X - err_s_X;
			err_s_X = speed_input_X - speed_sensor_X;
			err_sum_s_X += err_s_X * 0.001;

			if (s_PID_X == 0) {
				err_sum_s_X = 0;
			}
			S_KP_X = err_s_X * s_kp_X;
			S_KI_X = err_sum_s_X * s_ki_X;

			//	err_d = (err - previous_err) / 0.0001;   //1ms
			//	previous_err = err;

			S_KD_X = s_kd_X * errorgap_s_X / 0.001;

			s_PID_X = S_KP_X + S_KI_X + S_KD_X;

			if (s_PID_X >= 6000) {
				s_PID_X = 6000;
			} else if (s_PID_X <= -6000) {
				s_PID_X = -6000;
			}
/*
			if (s_PID_X > 0) {
				GPIOE->ODR &= ~1 << 0; // go
				TIM3->CCR1 = s_PID_X;
			} else if (s_PID_X < 0) {
				GPIOE->ODR |= 1 << 0; // back
				s_PID_X = -s_PID_X;
				TIM3->CCR1 = s_PID_X;
			} else {
				s_PID_X = 0;
				TIM3->CCR1 = s_PID_X;
			}*/
		////////////////////////////    전류

				//		c_kp = ;
				//	c_ki = ;
				//	c_kd = ;

				//참고	  errorGap = target - current - realError;
				//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
				//	    accError += realError;
				current_sensor_X = (current_A_floor_X/(3.14*2) * 512);
				current_input_X = s_PID_X;
				errorgap_c_X = current_input_X - current_sensor_X - err_c_X;
				err_c_X = current_input_X - current_sensor_X;
				err_sum_c_X += err_c_X * 0.001;

				if (c_PID_X == 0) {
					err_sum_c_X = 0;
				}
				C_KP_X = err_c_X * c_kp_X;
				C_KI_X = err_sum_c_X * c_ki_X;

				//	err_d = (err - previous_err) / 0.0001;   //1ms
				//	previous_err = err;

				C_KD_X = c_kd_X * errorgap_c_X / 0.001;

				c_PID_X = C_KP_X + C_KI_X + C_KD_X;



				 if (c_PID_X >= 8000) {
				 c_PID_X = 8000;
				 } else if (c_PID_X <= -8000) {
				 c_PID_X = -8000;
				 }



				if (c_PID_X > 40) {
					GPIOE->ODR &= ~1 << 0; // go
					TIM3->CCR1 = c_PID_X;
				} else if (c_PID_X < 40) {
					GPIOE->ODR |= 1 << 0; // back
					c_PID_X = -c_PID_X;
					TIM3->CCR1 = c_PID_X;
				} else {
					c_PID_X = 0;
					TIM3->CCR1 = c_PID_X;
					HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //stop
				}



#endif
}


#define SOURCE2 10
//7 :  vt
//10 : y축 위치 속도제어기
void position_pid_y(void) {

#if SOURCE2 == 10


	errorgap_p_Y = input_position_Y - (TIM4->CNT )- err_Y;
	err_Y = input_position_Y - (TIM4->CNT );
	err_sum_Y += err_Y * 0.001;

	if (p_PID_Y == 0) {
		err_sum_Y = 0;
	}

	P_KP_Y = err_Y * p_kp_Y;
	P_KI_Y = err_sum_Y * p_ki_Y;

	//	err_d_X = (err_X - previous_err_X) / 0.0001;   //1ms
	//	previous_err_X = err_X;

	P_KD_Y = p_kd_Y * errorgap_p_Y / 0.001;

	p_PID_Y = P_KP_Y + P_KI_Y + P_KD_Y;

	if (p_PID_Y >= 11000) {
		p_PID_Y = 11000;
	} else if (p_PID_Y <= -11000) {
		p_PID_Y = -11000;
	}
	/////////////////////////////////////속도

	//참고	  errorGap = target - current - realError;
	//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
	//	    accError += realError;

	speed_sensor_Y = (RPM_Y * 512) / 60;
	speed_input_Y = p_PID_Y;
	errorgap_s_Y = speed_input_Y - speed_sensor_Y - err_s_Y;
	err_s_Y = speed_input_Y - speed_sensor_Y;
	err_sum_s_Y += err_s_Y * 0.001;

	if (s_PID_Y == 0) {
		err_sum_s_Y = 0;
	}

	                 //s_kp_Y=1;
	//S_KP_Y = err_s_Y * s_kp_Y;
	S_KP_Y = err_s_Y * 1;

	S_KI_Y = err_sum_s_Y * s_ki_Y;

	//	err_d = (err - previous_err) / 0.0001;   //1ms
	//	previous_err = err;

	S_KD_Y = s_kd_Y * errorgap_s_Y / 0.001;

	s_PID_Y = S_KP_Y + S_KI_Y + S_KD_Y;

	if (s_PID_Y >= 6000) {
		s_PID_Y = 6000;
	} else if (s_PID_Y <= -6000) {
		s_PID_Y = -6000;
	}

	if (s_PID_Y > 0)
	{
		GPIOB->ODR &= ~1 << 10;  // go
		TIM3->CCR2 = s_PID_Y;
	}
	else if (s_PID_Y < 0)
	{
		GPIOB->ODR |= 1 << 10;  // back
		s_PID_Y = -s_PID_Y;
		TIM3->CCR2 = s_PID_Y;
	}
	else
	{
		s_PID_Y = 0;
		TIM3->CCR2 = s_PID_Y;
		position_start_Y = 0;
		photo_sensor_flag_y=0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);  //stop

		PID_Y_finish=1;
//		RPM_Y_go=0;
//		RPM_Y_back=0;
	}

#elif SOURCE2 == 7


	TIM6->ARR = 10000 - 1;  //ARR  : 샘플링 주기 1ms에서 10ms=0.01초으로 변경


		if(v_t_dir_back_flag==1)  //후진이면
		{
			speed_sensor_Y = ((-RPM_Y / 60) * 3.141592 * 9.5);  //  cm/s
		}
		else
		{
			speed_sensor_Y = (RPM_Y / 60) * 3.141592 * 9.5;  //  cm/s
		}

		if(i_v_t_count <= orderN)
		{
			speed_input_Y = orderArr[i_v_t_count++];
		}
		errorgap_s_Y = speed_input_Y - speed_sensor_Y - err_s_Y;
		err_s_Y = speed_input_Y - speed_sensor_Y;
		err_sum_s_Y += err_s_Y * 0.01;

		if ((i_v_t_count >= raisingN) && (i_v_t_count <= (orderN - raisingN))) {
			s_ki_Y = 50;
		} else
			s_ki_Y = 0;


		if (i_v_t_count >= orderN) {
				for (int x = orderN; x <= 3000; x++) {
					orderArr[x] = 0;
				}

		}



		if (s_PID_Y == 0) {
			err_sum_s_Y = 0;
		}
		S_KP_Y = err_s_Y * s_kp_Y;
		S_KI_Y = err_sum_s_Y * s_ki_Y;

		S_KD_Y = s_kd_Y * errorgap_s_Y / 0.01;

		s_PID_Y = S_KP_Y + S_KI_Y + S_KD_Y;

		if (s_PID_Y >= 8000) {
			s_PID_Y = 8000;
		} else if (s_PID_Y <= -8000) {
			s_PID_Y = -8000;
		}

		if (s_PID_Y > 0) {
			//	GPIOE->ODR &= ~1 << 0; // go
			TIM3->CCR2 = s_PID_Y;

		} else if (s_PID_Y < 0) {
			//	GPIOE->ODR |= 1 << 0; // back
			s_PID_Y = -s_PID_Y;
			TIM3->CCR2 = -s_PID_Y;

		} else {
			s_PID_X = 0;
			TIM3->CCR2 = s_PID_X;

			if(i_v_t_count >=orderN)
				{position_start_Y = 0;
				}

		}





#endif
}

