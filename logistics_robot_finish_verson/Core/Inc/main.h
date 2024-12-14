/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>



typedef enum {
	FALSE = 0, TRUE
} bool;



struct flag{
	bool x_go;
	bool x_back;

	bool y_go;
	bool y_back;
	bool ball_high;
	bool ball_mid;
	bool ball_low;

	bool downpart_high;
	bool downpart_low;

	bool linear_go;
	bool linear_back;
};

//Finish_flag={FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,0,0,FALSE,FALSE};

struct Manual{
	bool target_2_1;
	bool target_3_1;
	bool target_1_2;
	bool target_2_2;
	bool target_3_2;

	bool box_pickup;
	bool Box_dropoff;
} ;


typedef struct {
    bool mov;
    bool pov;
    bool rol;
    bool pik;
    bool plc;

    bool mov_;
    bool pov_;
    bool rol_;
    bool pik_;
    bool plc_;

} AnalyzerType;






#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


void SerialSendChar_PC(uint8_t Ch1);
void SerialSendChar_ESP(uint8_t Ch2);

void UART3_RX_PC_to_STM32(void);
void UART4_RX_ESP_to_STM32(void);
void STM32_to_ESP(void);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define linear_DIR_Pin GPIO_PIN_4
#define linear_DIR_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define linear_goback_Pin GPIO_PIN_2
#define linear_goback_GPIO_Port GPIOC
#define DOWN_LIMIT_SW_Pin GPIO_PIN_3
#define DOWN_LIMIT_SW_GPIO_Port GPIOA
#define DOWN_LIMIT_SW_EXTI_IRQn EXTI3_IRQn
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define DC_M_GO_X_Pin GPIO_PIN_0
#define DC_M_GO_X_GPIO_Port GPIOG
#define DC_M_GO_Y_Pin GPIO_PIN_10
#define DC_M_GO_Y_GPIO_Port GPIOB
#define DC_M_DIR_Y_Pin GPIO_PIN_11
#define DC_M_DIR_Y_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BALL_HIGH_Pin GPIO_PIN_0
#define BALL_HIGH_GPIO_Port GPIOD
#define BALL_MID_Pin GPIO_PIN_1
#define BALL_MID_GPIO_Port GPIOD
#define BALL_MID_EXTI_IRQn EXTI1_IRQn
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define sensor_X_Pin GPIO_PIN_8
#define sensor_X_GPIO_Port GPIOB
#define sensor_Y_Pin GPIO_PIN_9
#define sensor_Y_GPIO_Port GPIOB
#define DC_M_DIR_X_Pin GPIO_PIN_0
#define DC_M_DIR_X_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
