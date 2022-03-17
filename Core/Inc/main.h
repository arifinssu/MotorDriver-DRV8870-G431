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
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define M1_LIMIT_Pin GPIO_PIN_1
#define M1_LIMIT_GPIO_Port GPIOB
#define ADDR_SEL0_Pin GPIO_PIN_11
#define ADDR_SEL0_GPIO_Port GPIOB
#define ADDR_SEL1_Pin GPIO_PIN_12
#define ADDR_SEL1_GPIO_Port GPIOB
#define ADDR_SEL2_Pin GPIO_PIN_13
#define ADDR_SEL2_GPIO_Port GPIOB
#define ADDR_SEL3_Pin GPIO_PIN_14
#define ADDR_SEL3_GPIO_Port GPIOB
#define ADDR_SEL4_Pin GPIO_PIN_15
#define ADDR_SEL4_GPIO_Port GPIOB
#define COMM_SEL_Pin GPIO_PIN_10
#define COMM_SEL_GPIO_Port GPIOA
#define LED_ST2_Pin GPIO_PIN_3
#define LED_ST2_GPIO_Port GPIOB
#define LED_ST1_Pin GPIO_PIN_4
#define LED_ST1_GPIO_Port GPIOB
#define M1_EN1_Pin GPIO_PIN_6
#define M1_EN1_GPIO_Port GPIOB
#define M1_EN2_Pin GPIO_PIN_7
#define M1_EN2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
