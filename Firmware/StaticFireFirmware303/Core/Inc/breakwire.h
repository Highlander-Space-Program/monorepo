/*
 * breakwire.h
 *
 *  Created on: Feb 18, 2025
 *      Author: zande
 */
#include "stdbool.h"
#include "config.h"

#ifndef INC_BREAKWIRE_H_
#define INC_BREAKWIRE_H_

GPIO_PinState Check_Breakwire() {
	return HAL_GPIO_ReadPin(BRK_CONT_GPIO_Port, BRK_CONT_Pin);
}

void Breakwire_LED_On() {
	HAL_GPIO_WritePin(BRK_CONT_LED_GPIO_Port, BRK_CONT_LED_Pin, GPIO_PIN_SET);
}

void Breakwire_LED_Off() {
	HAL_GPIO_WritePin(BRK_CONT_LED_GPIO_Port, BRK_CONT_LED_Pin, GPIO_PIN_RESET);
}

void Tick_Breakwire_LED() {
	if (Check_Breakwire() == GPIO_PIN_RESET) {
		// no continuity
		Breakwire_LED_Off();
	}
	else if (Check_Breakwire() == GPIO_PIN_SET) {
		// continuity
		if (isAutoArmed) {
			// armed so turn LED on solid
			Breakwire_LED_On();
		}
		else {
			// not armed so turn LED on flash
			uint64_t curr_time = HAL_GetTick() % LED_FLASH_TIME_MS;
			if (curr_time > LED_FLASH_TIME_MS / 2) {
				Breakwire_LED_On();
			}
			else {
				Breakwire_LED_Off();
			}
		}
	}
}

#endif /* INC_BREAKWIRE_H_ */
