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
#define PYRO_CONT0_Pin GPIO_PIN_1
#define PYRO_CONT0_GPIO_Port GPIOA
#define PYRO_CONT1_Pin GPIO_PIN_2
#define PYRO_CONT1_GPIO_Port GPIOA
#define PYRO_CONT2_Pin GPIO_PIN_3
#define PYRO_CONT2_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define GPIO2_Pin GPIO_PIN_10
#define GPIO2_GPIO_Port GPIOB
#define GPIO3_Pin GPIO_PIN_11
#define GPIO3_GPIO_Port GPIOB
#define PYRO_SW0_Pin GPIO_PIN_12
#define PYRO_SW0_GPIO_Port GPIOB
#define PYRO_SW1_Pin GPIO_PIN_13
#define PYRO_SW1_GPIO_Port GPIOB
#define PYRO_SW2_Pin GPIO_PIN_14
#define PYRO_SW2_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define GPIO0_Pin GPIO_PIN_8
#define GPIO0_GPIO_Port GPIOB
#define GPIO1_Pin GPIO_PIN_9
#define GPIO1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
