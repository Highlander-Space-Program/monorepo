/*
 * no6.h
 *
 *  Created on: Jan 8, 2025
 *      Author: zande
 */

#ifndef INC_NO6_H_
#define INC_NO6_H_

#include "config.h"
#include "deg_to_ccr.h"
#include "create_ack.h"
#include "main.h"
#include "breakwire.h"

void Tick_NO6 (uint8_t cmd, struct Servo *servo) {
	uint64_t curr_ticks = HAL_GetTick();
	uint64_t on_ticks = curr_ticks - no6_on_time;
	//transitions
	switch(no6State) {
		case SERVO_INIT:
		no6State = SERVO_CLOSED_OFF;
		break;

		case SERVO_CLOSED_OFF:
		if (isAutoArmed && Check_Breakwire() == GPIO_PIN_RESET) {
			no6_on_time = HAL_GetTick();
			no6State = SERVO_OPENED_ON;
		}
		else if (cmd == CLOSE_NO6) {
			no6_on_time = HAL_GetTick();
			no6State = SERVO_CLOSED_ON;
		}
		else if (cmd == OPEN_NO6 || cmd == START_1) {
			no6_on_time = HAL_GetTick();
			no6State = SERVO_OPENED_ON;
		}
		break;

		case SERVO_CLOSED_ON:
		if (isAutoArmed && Check_Breakwire() == GPIO_PIN_RESET) {
			no6_on_time = HAL_GetTick();
			no6State = SERVO_OPENED_ON;
		}
		if (cmd == OPEN_NO6 || cmd == START_1) {
			no6_on_time = HAL_GetTick();
			no6State = SERVO_OPENED_ON;
		}
		else if (on_ticks > SERVO_ON_TIME_MS) {
			no6State = SERVO_CLOSED_OFF;
		}
		break;

		case SERVO_OPENED_OFF:
		if (cmd == CLOSE_NO6) {
			no6_on_time = HAL_GetTick();
			no6State = SERVO_CLOSED_ON;
		}
		else if (cmd == OPEN_NO6 || cmd == START_1) {
			no6_on_time = HAL_GetTick();
			no6State = SERVO_OPENED_ON;
		}
		break;

		case SERVO_OPENED_ON:
		if (cmd == CLOSE_NO6) {
			no6_on_time = HAL_GetTick();
			no6State = SERVO_CLOSED_ON;
		}
		else if (on_ticks > SERVO_ON_TIME_MS) {
			no6State = SERVO_OPENED_OFF;
		}
		break;
	}

	//actions
	switch(no6State) {
		case SERVO_INIT:
		break;

		case SERVO_CLOSED_OFF:
		HAL_GPIO_WritePin(NO6_EN_GPIO_Port, NO6_EN_Pin, GPIO_PIN_RESET);
		break;

		case SERVO_CLOSED_ON:
		HAL_GPIO_WritePin(NO6_EN_GPIO_Port, NO6_EN_Pin, GPIO_PIN_SET);
		*servo->ccr = Deg_To_CCR(NO6_CLOSED_DEG, servo, HSP_SERVO_MAX_DEG);
		break;

		case SERVO_OPENED_OFF:
		HAL_GPIO_WritePin(NO6_EN_GPIO_Port, NO6_EN_Pin, GPIO_PIN_RESET);
		break;

		case SERVO_OPENED_ON:
		HAL_GPIO_WritePin(NO6_EN_GPIO_Port, NO6_EN_Pin, GPIO_PIN_SET);
		*servo->ccr = Deg_To_CCR(NO6_OPENED_DEG, servo, HSP_SERVO_MAX_DEG);
		break;

	}
}

#endif /* INC_NO6_H_ */
