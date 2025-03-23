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

typedef enum {
	ON_CLOSED = 0,
	OFF_CLOSED = 1,
	ON_OPEN = 2,
	OFF_OPEN = 3
} SERVO_STATE;

typedef enum {
	OPEN_SERVO = 0,
	CLOSE_SERVO = 1,
} SERVO_CMD;

#endif /* INC_COREUTILS_CONFIG_SERVO_CONFIG_H_ */
