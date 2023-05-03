/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define OB_LED_Pin GPIO_PIN_13
#define OB_LED_GPIO_Port GPIOC
#define GYRO_VREF_Pin GPIO_PIN_0
#define GYRO_VREF_GPIO_Port GPIOA
#define GYRO_OUTZ_Pin GPIO_PIN_1
#define GYRO_OUTZ_GPIO_Port GPIOA
#define ML_SPD_Pin GPIO_PIN_2
#define ML_SPD_GPIO_Port GPIOA
#define MR_SPD_Pin GPIO_PIN_3
#define MR_SPD_GPIO_Port GPIOA
#define RECIV_FR_Pin GPIO_PIN_4
#define RECIV_FR_GPIO_Port GPIOA
#define RECIV_R_Pin GPIO_PIN_5
#define RECIV_R_GPIO_Port GPIOA
#define ML_ENC_A_Pin GPIO_PIN_6
#define ML_ENC_A_GPIO_Port GPIOA
#define ML_ENC_B_Pin GPIO_PIN_7
#define ML_ENC_B_GPIO_Port GPIOA
#define RECIV_L_Pin GPIO_PIN_0
#define RECIV_L_GPIO_Port GPIOB
#define RECIV_FL_Pin GPIO_PIN_1
#define RECIV_FL_GPIO_Port GPIOB
#define EMIT_R_Pin GPIO_PIN_10
#define EMIT_R_GPIO_Port GPIOB
#define EMIT_L_Pin GPIO_PIN_11
#define EMIT_L_GPIO_Port GPIOB
#define EMIT_FL_Pin GPIO_PIN_12
#define EMIT_FL_GPIO_Port GPIOB
#define MR_FWD_Pin GPIO_PIN_13
#define MR_FWD_GPIO_Port GPIOB
#define ML_BACK_Pin GPIO_PIN_14
#define ML_BACK_GPIO_Port GPIOB
#define MR_BACK_Pin GPIO_PIN_15
#define MR_BACK_GPIO_Port GPIOB
#define ML_FWD_Pin GPIO_PIN_8
#define ML_FWD_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_12
#define LED_BLUE_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_15
#define LED_GREEN_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_4
#define BUTTON_GPIO_Port GPIOB
#define EMIT_FR_Pin GPIO_PIN_5
#define EMIT_FR_GPIO_Port GPIOB
#define MR_ENC_B_Pin GPIO_PIN_6
#define MR_ENC_B_GPIO_Port GPIOB
#define MR_ENC_A_Pin GPIO_PIN_7
#define MR_ENC_A_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
