/*
 * servo_config.h
 *
 *  Created on: Mar 11, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_SERVO_CONFIG_H_
#define INC_COREUTILS_CONFIG_SERVO_CONFIG_H_

struct Servo;

typedef struct {
	uint32_t board_uid[3];
	uint32_t can_id;
	char* pnid;
	int closed_deg;
	int open_deg;
	uint16_t frequency;
} ServoConfig;

#endif /* INC_COREUTILS_CONFIG_SERVO_CONFIG_H_ */
