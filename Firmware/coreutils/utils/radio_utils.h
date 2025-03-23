/*
 * radio_utils.h
 *
 *  Created on: Mar 22, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_COREUTILS_UTILS_RADIO_UTILS_H_
#define INC_COREUTILS_UTILS_RADIO_UTILS_H_

#include "stdbool.h"

static uint8_t rx_buff[1];
static uint8_t tx_buff[1];
static uint8_t ack = 0x00;

// 0,		0,		0,   	0, 		0,      0  		=> 000000
// no2,		no3,  	no4,	no6, 	eo1, 	ignitor => all off
void Update_Ack(uint8_t* ack, uint8_t shift_amount, bool is_off) {
    if (is_off) {
        *ack &= ~(1 << shift_amount);  // Clear the bit
    } else {
        *ack |= (1 << shift_amount);   // Set the bit
    }
}

uint8_t Create_Ack() {
//	if (no2State == SERVO_OPENED_ON || no2State == SERVO_OPENED_OFF) {
//		++ack;
//	}
//	ack = ack << 1;
//	if (no3State == SERVO_OPENED_ON || no3State == SERVO_OPENED_OFF) {
//		++ack;
//	}
//	ack = ack << 1;
//	if (no4State == SERVO_OPENED_ON || no4State == SERVO_OPENED_OFF) {
//		++ack;
//	}
//	ack = ack << 1;
//	if (no6State == SERVO_OPENED_ON || no6State == SERVO_OPENED_OFF) {
//		++ack;
//	}
//	ack = ack << 1;
//	if (eo1State == SERVO_OPENED_ON || eo1State == SERVO_OPENED_OFF) {
//		++ack;
//	}
//	ack = ack << 1;
//	if (igniterState == IGNITER_ACTIVATED) {
//		++ack;
//	}
//	return ack;
	return 0x00;
}

#endif /* INC_COREUTILS_UTILS_RADIO_UTILS_H_ */
