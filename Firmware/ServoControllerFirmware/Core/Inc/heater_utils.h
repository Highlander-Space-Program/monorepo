/*
 * heater_utils.h
 *
 *  Created on: Nov 20, 2024
 *      Author: zande
 */

#ifndef INC_HEATER_UTILS_H_
#define INC_HEATER_UTILS_H_

#include "stm32f0xx_hal.h"
#include "main.h"

void Heater_On() {
	HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_SET);
	STATUS_IND_On();
}

void Heater_Off() {
	HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_RESET);
	STATUS_IND_Off();
}

#endif /* INC_HEATER_UTILS_H_ */
