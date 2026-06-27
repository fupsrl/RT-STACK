/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEL12_Pin GPIO_PIN_2
#define SEL12_GPIO_Port GPIOE
#define SEL34_Pin GPIO_PIN_3
#define SEL34_GPIO_Port GPIOE
#define SEL56_Pin GPIO_PIN_4
#define SEL56_GPIO_Port GPIOE
#define SEL78_Pin GPIO_PIN_5
#define SEL78_GPIO_Port GPIOE
#define SEL1112_Pin GPIO_PIN_6
#define SEL1112_GPIO_Port GPIOE
#define SEL910_Pin GPIO_PIN_13
#define SEL910_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOF
#define VBUS_Pin GPIO_PIN_3
#define VBUS_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOF
#define BOOST910_Pin GPIO_PIN_4
#define BOOST910_GPIO_Port GPIOA
#define BOOST1112_Pin GPIO_PIN_5
#define BOOST1112_GPIO_Port GPIOA
#define BOOST78_Pin GPIO_PIN_6
#define BOOST78_GPIO_Port GPIOA
#define BOOST56_Pin GPIO_PIN_7
#define BOOST56_GPIO_Port GPIOA
#define BOOST34_Pin GPIO_PIN_4
#define BOOST34_GPIO_Port GPIOC
#define BOOST12_Pin GPIO_PIN_5
#define BOOST12_GPIO_Port GPIOC
#define VBATT_Pin GPIO_PIN_2
#define VBATT_GPIO_Port GPIOB
#define IGN2_Pin GPIO_PIN_8
#define IGN2_GPIO_Port GPIOE
#define IGN1_Pin GPIO_PIN_9
#define IGN1_GPIO_Port GPIOE
#define IGN4_Pin GPIO_PIN_10
#define IGN4_GPIO_Port GPIOE
#define IGN3_Pin GPIO_PIN_11
#define IGN3_GPIO_Port GPIOE
#define IGN6_Pin GPIO_PIN_12
#define IGN6_GPIO_Port GPIOE
#define IGN5_Pin GPIO_PIN_14
#define IGN5_GPIO_Port GPIOE
#define IGN8_Pin GPIO_PIN_15
#define IGN8_GPIO_Port GPIOE
#define IGN7_Pin GPIO_PIN_10
#define IGN7_GPIO_Port GPIOB
#define IGN10_Pin GPIO_PIN_12
#define IGN10_GPIO_Port GPIOB
#define IGN9_Pin GPIO_PIN_15
#define IGN9_GPIO_Port GPIOB
#define IGN12_Pin GPIO_PIN_8
#define IGN12_GPIO_Port GPIOD
#define IGN11_Pin GPIO_PIN_9
#define IGN11_GPIO_Port GPIOD
#define TMG_OUT1_Pin GPIO_PIN_10
#define TMG_OUT1_GPIO_Port GPIOD
#define TMG_OUT6_Pin GPIO_PIN_11
#define TMG_OUT6_GPIO_Port GPIOD
#define TMG_OUT2_Pin GPIO_PIN_12
#define TMG_OUT2_GPIO_Port GPIOD
#define TMG_OUT7_Pin GPIO_PIN_13
#define TMG_OUT7_GPIO_Port GPIOD
#define TMG_OUT4_Pin GPIO_PIN_14
#define TMG_OUT4_GPIO_Port GPIOD
#define TMG_OUT8_Pin GPIO_PIN_15
#define TMG_OUT8_GPIO_Port GPIOD
#define TMG_OUT3_Pin GPIO_PIN_9
#define TMG_OUT3_GPIO_Port GPIOC
#define TMG_OUT9_Pin GPIO_PIN_8
#define TMG_OUT9_GPIO_Port GPIOA
#define TMG_OUT5_Pin GPIO_PIN_10
#define TMG_OUT5_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
