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
#define JOY_RIGHT_Pin GPIO_PIN_2
#define JOY_RIGHT_GPIO_Port GPIOA
#define BTN_LEFT_DOWN_Pin GPIO_PIN_14
#define BTN_LEFT_DOWN_GPIO_Port GPIOB
#define BTN_LEFT_UP_Pin GPIO_PIN_15
#define BTN_LEFT_UP_GPIO_Port GPIOB
#define DEBUG_LED_Pin GPIO_PIN_8
#define DEBUG_LED_GPIO_Port GPIOA
#define DISP_RES_Pin GPIO_PIN_9
#define DISP_RES_GPIO_Port GPIOA
#define DISP_SA0_Pin GPIO_PIN_10
#define DISP_SA0_GPIO_Port GPIOA
#define JOY_LEFT_Pin GPIO_PIN_3
#define JOY_LEFT_GPIO_Port GPIOB
#define JOY_DOWN_Pin GPIO_PIN_4
#define JOY_DOWN_GPIO_Port GPIOB
#define JOY_UP_Pin GPIO_PIN_5
#define JOY_UP_GPIO_Port GPIOB
#define JOY_PRESS_Pin GPIO_PIN_6
#define JOY_PRESS_GPIO_Port GPIOB
#define INTERRUPT_LINE_Pin GPIO_PIN_9
#define INTERRUPT_LINE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
