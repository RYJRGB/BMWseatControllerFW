/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BR1B_Pin GPIO_PIN_0
#define BR1B_GPIO_Port GPIOA
#define BR1A_Pin GPIO_PIN_1
#define BR1A_GPIO_Port GPIOA
#define BR2B_Pin GPIO_PIN_4
#define BR2B_GPIO_Port GPIOA
#define BR2A_Pin GPIO_PIN_5
#define BR2A_GPIO_Port GPIOA
#define BR3B_Pin GPIO_PIN_6
#define BR3B_GPIO_Port GPIOA
#define BR3A_Pin GPIO_PIN_7
#define BR3A_GPIO_Port GPIOA
#define BR4B_Pin GPIO_PIN_0
#define BR4B_GPIO_Port GPIOB
#define BR4A_Pin GPIO_PIN_1
#define BR4A_GPIO_Port GPIOB
#define BR5B_Pin GPIO_PIN_2
#define BR5B_GPIO_Port GPIOB
#define BR5A_Pin GPIO_PIN_10
#define BR5A_GPIO_Port GPIOB
#define LS3_EN_Pin GPIO_PIN_12
#define LS3_EN_GPIO_Port GPIOB
#define LS2_EN_Pin GPIO_PIN_13
#define LS2_EN_GPIO_Port GPIOB
#define LS1_EN_Pin GPIO_PIN_14
#define LS1_EN_GPIO_Port GPIOB
#define INA_ALERT_Pin GPIO_PIN_15
#define INA_ALERT_GPIO_Port GPIOB
#define LIN_CS_Pin GPIO_PIN_7
#define LIN_CS_GPIO_Port GPIOB
#define RS485DE_Pin GPIO_PIN_8
#define RS485DE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
