#include "step.h"
#include "main.h"

extern struct flag Finish_flag ;




extern uint32_t step_pulse_count_tim13;
extern uint16_t step_pulse_count_tim11;

extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim11;

uint16_t shaft_step = 400*1;//400스텝 : 1바퀴   200*24
uint16_t shaft_pulse_cycle = 375; //속도   420

uint16_t ball_screw_step=400*1;
uint16_t ball_screw_pulse_cycle=250;


extern uint8_t step_count_for_flag__;
extern uint8_t shaft_accel_flag; //0:등속 ,1 가속

uint8_t ball_dir_flag=0; // 0:위, 1: 아래
uint8_t downpart_dir_flag=0; // 0:위, 1: 아래


int step_count__flag=0;



int step_count__=500;       //미스미 스텝: max=400~500,  min=261;
int step_count__max=450;
int step_count__min=305;      //전류 줄이면 190까지 가능 : ofo   , 정격은 240~250  ,260
int count_arr[1000];


extern bool accel_flag;

extern bool paser_flag;
//extern int downpart_run;
extern int ballscrew_up_run;
extern int ballscrew_down_run;

int down_speed;

extern int pik;
void shaft_step_motor(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)  // 다운 파트 스텝모터
{
	if(shaft_accel_flag==0)  //0: 등속
	{
		TIM13->ARR = cycle_time1;
	}

	if(step_pulse_count_tim13 >= step1*2)  //
	{
		step_count_for_flag__=0;
		HAL_TIM_OC_Stop_IT(&htim13,TIM_CHANNEL_1);  // 타이머 정지

	//	if( pik % 2 == 0)   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);   //짝수일떄 disadble


		step_count__=step_count__max;

		accel_flag=FALSE;
		if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14)==0)	{
			Finish_flag.downpart_low =TRUE ;   //내려가면
		//	paser_flag=1;
		}
	}
}

extern int ball_high;

int ball_low;
extern uint16_t tim10_flag;

void z_axis_step_motor(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)      // 볼스크류 스텝모터
{

	TIM11->ARR = cycle_time2;

	if(step_pulse_count_tim11 >= step2*2)  //
	{

		HAL_TIM_OC_Stop_IT(&htim11,TIM_CHANNEL_1);  // 타이머 정지
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //정지


		if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3)==0)
		{
			Finish_flag.ball_high = TRUE;     //올라가면

			if(tim10_flag >= 2)  // paser 2초 딜레이용도
			{
				paser_flag=TRUE;
			}
			tim10_flag=0;

			printf("ball_high \n\r");

			ball_high++;
			ballscrew_up_run=0;
		}
		else
		{
			Finish_flag.ball_low = TRUE;   //내려가면
			ballscrew_down_run=0;
			//paser_flag=1;

			//ball_low++;
		}
	}
}



void z_axis_UP(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)  //UP = 정방향
{
	ballscrew_up_run=1;
	ballscrew_down_run=0;

	Finish_flag.ball_low = FALSE;  //인터록
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 0);  //출발

	ball_dir_flag=0;

	ball_screw_step=step2;
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 0);  //방향

	step_pulse_count_tim11 = 0;  //펄스 기준값 =0
	HAL_TIM_OC_Start_IT(&htim11, TIM_CHANNEL_1);

}

void z_axis_DOWN(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)  //DOWN = 역방향
{
	ballscrew_down_run=1;
	ballscrew_up_run=0;

	Finish_flag.ball_high = FALSE;
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 0);  //출발

	ball_dir_flag=1;

	ball_screw_step=step2;

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);  //방향

	step_pulse_count_tim11 = 0;
	HAL_TIM_OC_Start_IT(&htim11, TIM_CHANNEL_1);


}



void down_part_UP(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)
{
	Finish_flag.downpart_low = FALSE;  //인터록
	 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);   //enable

	down_speed = cycle_time1;

	shaft_accel_flag=0; //0:등속 ,1 가속
	downpart_dir_flag=0; // 0:위, 1: 아래
	shaft_step = step1;

	step_pulse_count_tim13 = 0;  //펄스 기준값 =0
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 1);  //방향  //일단 업이라 가정

	HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);

//	downpart_run=1;
}

void down_part_DOWN(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)
{
	Finish_flag.downpart_high = FALSE;  //인터록
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);   //enable
	down_speed = cycle_time1;

	shaft_accel_flag=0; //0:등속 ,1 가속
	downpart_dir_flag=1; // 0:위, 1: 아래
	shaft_step = step1;

	step_pulse_count_tim13 = 0;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 0);  //방향

	HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
	//downpart_run=1;
}


//다운스텝은 6000쯤


void down_part_UP_accel(uint16_t step1)  // (스텝,주기)
{
	Finish_flag.downpart_low = FALSE;  //인터록
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);   //enable
	shaft_accel_flag=1; //0:등속 ,1 가속

	for(step_count__=step_count__min ; step_count__<step_count__max; step_count__++) //1초당 100카운트
	{
		count_arr[step_count__]=step_count__;
	}
	step_count_for_flag__++;
	//step_count_for_flag__++;
	shaft_step = step1;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 1);  //방향
	step_pulse_count_tim13 = 0;  //펄스 기준값 =0
	HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
//	downpart_run=1;
}

void down_part_DOWN_accel(uint16_t step1)  // (스텝,주기)
{
	Finish_flag.downpart_high = FALSE;  //인터록
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);   //enable
	shaft_accel_flag=1; //0:등속 ,1 가속
//	step_count_for_flag__=1;
	for(step_count__=step_count__min ; step_count__<step_count__max; step_count__++) //1초당 100카운트
	{
		count_arr[step_count__]=step_count__;
	}
	step_count_for_flag__++;
	//step_count_for_flag__++;
	//step_count_for_flag__=1;
	//step_count_for_flag__=2;


	shaft_step = step1;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 0);  //방향
	step_pulse_count_tim13 = 0;  //펄스 기준값 =0
	HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
//	downpart_run=1;
}




