/*
 * heater_config.h
 *
 *  Created on: Mar 5, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_HEATER_CONFIG_H_
#define INC_COREUTILS_CONFIG_HEATER_CONFIG_H_

#include <string.h>

struct Heater;

typedef struct {
	uint32_t uid[3];
	char* pnid;
	int off_temp;
	int on_temp;
	uint16_t frequency;
} HeaterConfig;

#endif /* INC_COREUTILS_CONFIG_HEATER_CONFIG_H_ */
