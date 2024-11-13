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
#define IGNITER_LIGHT_Pin GPIO_PIN_0
#define IGNITER_LIGHT_GPIO_Port GPIOC
#define HEALTH_LIGHT_Pin GPIO_PIN_2
#define HEALTH_LIGHT_GPIO_Port GPIOC
#define OX_FILL_CLOSED_LIGHT_Pin GPIO_PIN_3
#define OX_FILL_CLOSED_LIGHT_GPIO_Port GPIOC
#define FLASH_HOLD_Pin GPIO_PIN_5
#define FLASH_HOLD_GPIO_Port GPIOA
#define FLASH_RESET_Pin GPIO_PIN_6
#define FLASH_RESET_GPIO_Port GPIOA
#define FLASH_WP_Pin GPIO_PIN_7
#define FLASH_WP_GPIO_Port GPIOA
#define OX_FILL_OPEN_LIGHT_Pin GPIO_PIN_4
#define OX_FILL_OPEN_LIGHT_GPIO_Port GPIOC
#define OX_RUN_CLOSED_LIGHT_Pin GPIO_PIN_5
#define OX_RUN_CLOSED_LIGHT_GPIO_Port GPIOC
#define COLUMN_1_Pin GPIO_PIN_0
#define COLUMN_1_GPIO_Port GPIOB
#define COLUMN_2_Pin GPIO_PIN_1
#define COLUMN_2_GPIO_Port GPIOB
#define COLUMN_3_Pin GPIO_PIN_2
#define COLUMN_3_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_12
#define FLASH_CS_GPIO_Port GPIOB
#define OX_RUN_OPEN_LIGHT_Pin GPIO_PIN_6
#define OX_RUN_OPEN_LIGHT_GPIO_Port GPIOC
#define FUEL_RUN_CLOSED_LIGHT_Pin GPIO_PIN_7
#define FUEL_RUN_CLOSED_LIGHT_GPIO_Port GPIOC
#define FUEL_RUN_OPEN_LIGHT_Pin GPIO_PIN_8
#define FUEL_RUN_OPEN_LIGHT_GPIO_Port GPIOC
#define FUEL_PRESS_CLOSED_LIGHT_Pin GPIO_PIN_9
#define FUEL_PRESS_CLOSED_LIGHT_GPIO_Port GPIOC
#define ARM_LIGHT_Pin GPIO_PIN_8
#define ARM_LIGHT_GPIO_Port GPIOA
#define FUEL_PRESS_OPEN_LIGHT_Pin GPIO_PIN_10
#define FUEL_PRESS_OPEN_LIGHT_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOD
#define COLUMN_4_Pin GPIO_PIN_5
#define COLUMN_4_GPIO_Port GPIOB
#define ROW_1_Pin GPIO_PIN_6
#define ROW_1_GPIO_Port GPIOB
#define ROW_2_Pin GPIO_PIN_7
#define ROW_2_GPIO_Port GPIOB
#define ROW_3_Pin GPIO_PIN_8
#define ROW_3_GPIO_Port GPIOB
#define ROW_4_Pin GPIO_PIN_9
#define ROW_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
