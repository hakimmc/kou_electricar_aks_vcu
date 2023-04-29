/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN1_Pin GPIO_PIN_15
#define IN1_GPIO_Port GPIOC
#define IN1_EXTI_IRQn EXTI15_10_IRQn
#define IN2_Pin GPIO_PIN_0
#define IN2_GPIO_Port GPIOA
#define IN2_EXTI_IRQn EXTI0_IRQn
#define IN3_Pin GPIO_PIN_1
#define IN3_GPIO_Port GPIOA
#define IN3_EXTI_IRQn EXTI1_IRQn
#define IN4_Pin GPIO_PIN_2
#define IN4_GPIO_Port GPIOA
#define IN4_EXTI_IRQn EXTI2_IRQn
#define IN5_Pin GPIO_PIN_3
#define IN5_GPIO_Port GPIOA
#define IN5_EXTI_IRQn EXTI3_IRQn
#define IN6_Pin GPIO_PIN_4
#define IN6_GPIO_Port GPIOA
#define IN6_EXTI_IRQn EXTI4_IRQn
#define IN7_Pin GPIO_PIN_5
#define IN7_GPIO_Port GPIOA
#define IN7_EXTI_IRQn EXTI9_5_IRQn
#define IN8_Pin GPIO_PIN_6
#define IN8_GPIO_Port GPIOA
#define IN8_EXTI_IRQn EXTI9_5_IRQn
#define OUT8_Pin GPIO_PIN_15
#define OUT8_GPIO_Port GPIOA
#define OUT7_Pin GPIO_PIN_3
#define OUT7_GPIO_Port GPIOB
#define OUT6_Pin GPIO_PIN_4
#define OUT6_GPIO_Port GPIOB
#define OUT5_Pin GPIO_PIN_5
#define OUT5_GPIO_Port GPIOB
#define OUT4_Pin GPIO_PIN_6
#define OUT4_GPIO_Port GPIOB
#define OUT3_Pin GPIO_PIN_7
#define OUT3_GPIO_Port GPIOB
#define OUT2_Pin GPIO_PIN_8
#define OUT2_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_9
#define OUT1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
