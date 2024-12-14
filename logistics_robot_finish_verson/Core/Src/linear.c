#include "linear.h"
#include "main.h"

extern struct flag Finish_flag ;


uint8_t linear_count = 0;

uint8_t linear_go_flag = 0;
uint8_t linear_back_flag = 0;



void linear_motor_GO(float time)
{
	if (linear_go_flag == 0) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);  //방향  전진
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);  //출발
	TIM3->CCR4=8200;
	linear_go_flag = 1;
		}
}

void linear_motor_BACK(float time)
{
	if (linear_go_flag == 1) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);   //방향 후진
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);  //출발
	TIM3->CCR4=8200;
	linear_go_flag = 0;
		}
}

void linear_time_count_GO(float time)
{
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 0) && (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4) == 0))
		{ //리니어      전진  && 출발

				linear_count++;
				GPIOB->ODR |= 1<<0;
				if(linear_count>=(uint16_t)100.0*time)  //time=1 이면 1초
				{
					linear_count=0;
					GPIOB->ODR &= ~1<<0;
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //정지
					TIM3->CCR4=0;

					Finish_flag.linear_go = TRUE;
				//	paser_flag=1;

				}
		}
}

void linear_time_count_BACK(float time)
{
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 1) && (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4) == 0))
		{ //리니어      후진  && 출발

				linear_count++;
				GPIOB->ODR |= 1<<0;
				if(linear_count>= (uint16_t)100.0*time )  //time=1 이면 1초
				{
					GPIOB->ODR &= ~1<<0;
					linear_count=0;
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //정지
					TIM3->CCR4=0;

					Finish_flag.linear_back = TRUE;
					//paser_flag=1;

				}
		}
}
