/*
 * heater_config.h
 *
 *  Created on: Mar 11, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_HEATER_CONFIG_H_
#define INC_COREUTILS_CONFIG_HEATER_CONFIG_H_

struct Heater;

typedef struct {
	uint32_t board_uid[3];
	uint32_t can_id;
	char* pnid;
	int off_temp;
	int on_temp;
	uint16_t frequency;
} HeaterConfig;

#endif /* INC_COREUTILS_CONFIG_HEATER_CONFIG_H_ */
