/*
 * no3.h
 *
 *  Created on: Jan 8, 2025
 *      Author: zande
 */

#ifndef INC_NO3_H_
#define INC_NO3_H_

#include "config.h"
#include "deg_to_ccr.h"
#include "create_ack.h"
#include "main.h"

void Tick_NO3 (uint8_t cmd, struct Servo *servo) {
	uint64_t curr_ticks = HAL_GetTick();
	uint64_t on_ticks = curr_ticks - no3_on_time;
	//transitions
	switch(no3State) {
		case SERVO_INIT:
		no3State = SERVO_CLOSED_OFF;
		break;

		case SERVO_CLOSED_OFF:
		if (cmd == CLOSE_NO3) {
			no3_on_time = HAL_GetTick();
			no3State = SERVO_CLOSED_ON;
		}
		else if (cmd == OPEN_NO3) {
			no3_on_time = HAL_GetTick();
			no3State = SERVO_OPENED_ON;
		}
		break;

		case SERVO_CLOSED_ON:
		if (cmd == OPEN_NO3) {
			no3_on_time = HAL_GetTick();
			no3State = SERVO_OPENED_ON;
		}
		else if (on_ticks > SERVO_ON_TIME_MS) {
			no3State = SERVO_CLOSED_OFF;
		}
		break;

		case SERVO_OPENED_OFF:
		if (cmd == CLOSE_NO3) {
			no3_on_time = HAL_GetTick();
			no3State = SERVO_CLOSED_ON;
		}
		else if (cmd == OPEN_NO3) {
			no3_on_time = HAL_GetTick();
			no3State = SERVO_OPENED_ON;
		}
		break;

		case SERVO_OPENED_ON:
		if (cmd == CLOSE_NO3) {
			no3_on_time = HAL_GetTick();
			no3State = SERVO_CLOSED_ON;
		}
		else if (on_ticks > SERVO_ON_TIME_MS) {
			no3State = SERVO_OPENED_OFF;
		}
		break;
	}

	//actions
	switch(no3State) {
		case SERVO_INIT:
		break;

		case SERVO_CLOSED_OFF:
		HAL_GPIO_WritePin(NO3_EN_GPIO_Port, NO3_EN_Pin, GPIO_PIN_RESET);
		break;

		case SERVO_CLOSED_ON:
		HAL_GPIO_WritePin(NO3_EN_GPIO_Port, NO3_EN_Pin, GPIO_PIN_SET);
		*servo->ccr = Deg_To_CCR(NO3_CLOSED_DEG, servo, HSP_NO3_SERVO_MAX_DEG);
		break;

		case SERVO_OPENED_OFF:
		HAL_GPIO_WritePin(NO3_EN_GPIO_Port, NO3_EN_Pin, GPIO_PIN_RESET);
		break;

		case SERVO_OPENED_ON:
		HAL_GPIO_WritePin(NO3_EN_GPIO_Port, NO3_EN_Pin, GPIO_PIN_SET);
		*servo->ccr = Deg_To_CCR(NO3_OPENED_DEG, servo, HSP_NO3_SERVO_MAX_DEG);
		break;

	}
}

#endif /* INC_NO3_H_ */
