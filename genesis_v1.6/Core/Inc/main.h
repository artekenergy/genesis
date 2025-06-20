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
#include "stm32g0xx_hal.h"

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
#define SCK_Pin GPIO_PIN_1
#define SCK_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_2
#define MOSI_GPIO_Port GPIOA
#define SEL_Pin GPIO_PIN_4
#define SEL_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define CurrentSense_Pin GPIO_PIN_1
#define CurrentSense_GPIO_Port GPIOB
#define DIP_1_Pin GPIO_PIN_13
#define DIP_1_GPIO_Port GPIOB
#define DIP_0_Pin GPIO_PIN_14
#define DIP_0_GPIO_Port GPIOB
#define CAN_STBY_Pin GPIO_PIN_15
#define CAN_STBY_GPIO_Port GPIOA
#define CAN_Rx_Pin GPIO_PIN_0
#define CAN_Rx_GPIO_Port GPIOD
#define CAN_Tx_Pin GPIO_PIN_1
#define CAN_Tx_GPIO_Port GPIOD
#define LOAD3_Pin GPIO_PIN_3
#define LOAD3_GPIO_Port GPIOB
#define LOAD2_Pin GPIO_PIN_4
#define LOAD2_GPIO_Port GPIOB
#define LOAD1_Pin GPIO_PIN_5
#define LOAD1_GPIO_Port GPIOB
#define LOAD0_Pin GPIO_PIN_6
#define LOAD0_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_8
#define LED_RED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
