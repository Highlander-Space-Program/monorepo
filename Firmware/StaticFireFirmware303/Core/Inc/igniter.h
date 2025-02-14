/*
 * igniter.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_IGNITER_H_
#define INC_IGNITER_H_

#include "config.h"

//behavior for igniter
void Tick_Igniter(uint8_t cmd) {
	//transitions
	switch(igniterState) {
		case IGNITER_INIT:
		igniterState = IGNITER_DEACTIVATED;
		break;

		case IGNITER_DEACTIVATED:
		if (cmd == ACTIVATE_IGNITER && (!isCloseAll && !isAborted)) {
			igniterState = IGNITER_ACTIVATED;
		}
		else {
			igniterState = IGNITER_DEACTIVATED;
		}
		break;

		case IGNITER_ACTIVATED:
		if (((cmd == DEACTIVATE_IGNITER || cmd == CLOSE_ALL || isCloseAll)  && !isStarted) || cmd == ABORT) {
			igniterState = IGNITER_DEACTIVATED;
		}
		else {
			igniterState = IGNITER_ACTIVATED;
		}
		break;
	}

	//actions
	switch(igniterState) {
		case IGNITER_INIT:
		HAL_GPIO_WritePin(IGNITER_GPIO_Port, IGNITER_Pin, GPIO_PIN_RESET);
		break;

		case IGNITER_DEACTIVATED:
		HAL_GPIO_WritePin(IGNITER_GPIO_Port, IGNITER_Pin, GPIO_PIN_RESET);
		break;

		case IGNITER_ACTIVATED:
		HAL_GPIO_WritePin(IGNITER_GPIO_Port, IGNITER_Pin, GPIO_PIN_SET);
		break;
	}
}

#endif /* INC_IGNITER_H_ */
