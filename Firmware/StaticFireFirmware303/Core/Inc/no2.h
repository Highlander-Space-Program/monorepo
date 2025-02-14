/*
 * no2.h
 *
 *  Created on: Jan 8, 2025
 *      Author: zande
 */

#ifndef INC_NO2_H_
#define INC_NO2_H_

#include "config.h"
#include "deg_to_ccr.h"
#include "create_ack.h"
#include "main.h"

void Tick_NO2 (uint8_t cmd, struct Servo *servo) {
	uint64_t curr_ticks = HAL_GetTick();
	uint64_t on_ticks = curr_ticks - no2_on_time;
	//transitions
	switch(no2State) {
		case SERVO_INIT:
		no2State = SERVO_CLOSED_OFF;
		break;

		case SERVO_CLOSED_OFF:
		if (cmd == CLOSE_NO2) {
			no2_on_time = HAL_GetTick();
			no2State = SERVO_CLOSED_ON;
		}
		else if (cmd == OPEN_NO2) {
			no2_on_time = HAL_GetTick();
			no2State = SERVO_OPENED_ON;
		}
		break;

		case SERVO_CLOSED_ON:
		if (cmd == OPEN_NO2) {
			no2_on_time = HAL_GetTick();
			no2State = SERVO_OPENED_ON;
		}
		else if (on_ticks > SERVO_ON_TIME_MS) {
			no2State = SERVO_CLOSED_OFF;
		}
		break;

		case SERVO_OPENED_OFF:
		if (cmd == CLOSE_NO2) {
			no2_on_time = HAL_GetTick();
			no2State = SERVO_CLOSED_ON;
		}
		else if (cmd == OPEN_NO2) {
			no2_on_time = HAL_GetTick();
			no2State = SERVO_OPENED_ON;
		}
		break;

		case SERVO_OPENED_ON:
		if (cmd == CLOSE_NO2) {
			no2_on_time = HAL_GetTick();
			no2State = SERVO_CLOSED_ON;
		}
		else if (on_ticks > SERVO_ON_TIME_MS) {
			no2State = SERVO_OPENED_OFF;
		}
		break;
	}

	//actions
	switch(no2State) {
		case SERVO_INIT:
		break;

		case SERVO_CLOSED_OFF:
		HAL_GPIO_WritePin(NO2_EN_GPIO_Port, NO2_EN_Pin, GPIO_PIN_RESET);
		break;

		case SERVO_CLOSED_ON:
		HAL_GPIO_WritePin(NO2_EN_GPIO_Port, NO2_EN_Pin, GPIO_PIN_SET);
		*servo->ccr = Deg_To_CCR(NO2_CLOSED_DEG, servo, HSP_SERVO_MAX_DEG);
		break;

		case SERVO_OPENED_OFF:
		HAL_GPIO_WritePin(NO2_EN_GPIO_Port, NO2_EN_Pin, GPIO_PIN_RESET);
		break;

		case SERVO_OPENED_ON:
		HAL_GPIO_WritePin(NO2_EN_GPIO_Port, NO2_EN_Pin, GPIO_PIN_SET);
		*servo->ccr = Deg_To_CCR(NO2_OPENED_DEG, servo, HSP_SERVO_MAX_DEG);
		break;

	}
}

#endif /* INC_NO2_H_ */
