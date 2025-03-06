/*
 * servo_config.h
 *
 *  Created on: Mar 5, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_SERVO_CONFIG_H_
#define INC_COREUTILS_CONFIG_SERVO_CONFIG_H_

#include <string.h>

struct Servo;
typedef struct {
	uint32_t uid[3];
	char* pnid;
	int closed_deg;
	int open_deg;
	uint16_t frequency;
} ServoConfig;



#endif /* INC_COREUTILS_CONFIG_SERVO_CONFIG_H_ */
