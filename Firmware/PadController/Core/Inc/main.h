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
typedef enum {
    PC_STARTUP = 0,   // initial / waiting for handshake
    PC_AUTO_OFF,      // waiting for AUTO_ON command
    PC_AUTO_ON,       // AUTO switch is on (and breakwire continuity must be present)
    PC_IGNITED        // igniter fired, waiting for breakwire open
} PAD_CONTROLLER_STATE;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern PAD_CONTROLLER_STATE pcState;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STATUS_IND_Pin GPIO_PIN_13
#define STATUS_IND_GPIO_Port GPIOC



//break wire continuity
#define CONT_PIN0 GPIO_PIN_6
#define CONT_PIN0_GPIO_Port GPIOB

#define CONT_PIN1 GPIO_PIN_7
#define CONT_PIN1_GPIO_Port GPIOB

//pyro emmanuel's stuff

#define PYRO_SW0_Pin GPIO_PIN_12
#define PYRO_SW0_GPIO_Port GPIOB
#define PYRO_SW1_Pin GPIO_PIN_13
#define PYRO_SW1_GPIO_Port GPIOB
#define PYRO_SW2_Pin GPIO_PIN_14
#define PYRO_SW2_GPIO_Port GPIOB



// led for cont check:

#define LED_WAGO_PIN2 GPIO_PIN_10
#define LED_WAGO_PIN2_GPIO_Port GPIOB

#define LED_WAGO_PIN3 GPIO_PIN_11
#define LED_WAGO_PIN3_GPIO_Port GPIOB








/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
