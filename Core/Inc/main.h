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
#include "stm32f3xx_hal.h"

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
#define SD_SIGN_Pin GPIO_PIN_4
#define SD_SIGN_GPIO_Port GPIOA
#define CURRENT_SIGN_Pin GPIO_PIN_5
#define CURRENT_SIGN_GPIO_Port GPIOA
#define OUTPUT_current_reset_Pin GPIO_PIN_6
#define OUTPUT_current_reset_GPIO_Port GPIOA
#define OUTPUT_LED_B_Pin GPIO_PIN_0
#define OUTPUT_LED_B_GPIO_Port GPIOB
#define OUTPUT_LED_R_Pin GPIO_PIN_1
#define OUTPUT_LED_R_GPIO_Port GPIOB
#define OUTPUT_POWER_SD_Pin GPIO_PIN_8
#define OUTPUT_POWER_SD_GPIO_Port GPIOA
#define OUTPUT_LED_G_Pin GPIO_PIN_15
#define OUTPUT_LED_G_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
