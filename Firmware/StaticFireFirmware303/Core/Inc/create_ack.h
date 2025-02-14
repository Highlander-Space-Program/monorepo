/*
 * create_ack.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_CREATE_ACK_H_
#define INC_CREATE_ACK_H_

#include "config.h"

uint8_t Create_Ack() {
	if (no2State == SERVO_OPENED_ON || no2State == SERVO_OPENED_OFF) {
		++ack;
	}
	ack = ack << 1;
	if (no3State == SERVO_OPENED_ON || no3State == SERVO_OPENED_OFF) {
		++ack;
	}
	ack = ack << 1;
	if (no4State == SERVO_OPENED_ON || no4State == SERVO_OPENED_OFF) {
		++ack;
	}
	ack = ack << 1;
	if (no6State == SERVO_OPENED_ON || no6State == SERVO_OPENED_OFF) {
		++ack;
	}
	ack = ack << 1;
	if (eo1State == SERVO_OPENED_ON || eo1State == SERVO_OPENED_OFF) {
		++ack;
	}
	ack = ack << 1;
	if (igniterState == IGNITER_ACTIVATED) {
		++ack;
	}
	return ack;
}

#endif /* INC_CREATE_ACK_H_ */
