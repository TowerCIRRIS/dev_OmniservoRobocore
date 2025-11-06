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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_DIR_Pin GPIO_PIN_14
#define ENCODER_DIR_GPIO_Port GPIOC
#define MTR_nFAULT_Pin GPIO_PIN_1
#define MTR_nFAULT_GPIO_Port GPIOA
#define TEMP_SENSOR_Pin GPIO_PIN_7
#define TEMP_SENSOR_GPIO_Port GPIOA
#define MTR_PMODE_Pin GPIO_PIN_0
#define MTR_PMODE_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_13
#define BTN1_GPIO_Port GPIOB
#define BTN1_EXTI_IRQn EXTI15_10_IRQn
#define OUT_RS485_TXEN_Pin GPIO_PIN_14
#define OUT_RS485_TXEN_GPIO_Port GPIOB
#define CURRENT_SENSE_Pin GPIO_PIN_15
#define CURRENT_SENSE_GPIO_Port GPIOB
#define MTR_nSLEEP_Pin GPIO_PIN_8
#define MTR_nSLEEP_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOA
#define MTR_EN_IN1_Pin GPIO_PIN_10
#define MTR_EN_IN1_GPIO_Port GPIOA
#define MTR_PH_IN2_Pin GPIO_PIN_9
#define MTR_PH_IN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
