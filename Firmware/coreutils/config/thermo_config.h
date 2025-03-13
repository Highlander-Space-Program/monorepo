/*
 * thermo_config.h
 *
 *  Created on: Mar 11, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_THERMO_CONFIG_H_
#define INC_COREUTILS_CONFIG_THERMO_CONFIG_H_

struct Thermocouple;

typedef struct {
    uint32_t board_uid[3];
	uint32_t can_id;
    char* pnid;
    uint16_t frequency;
} ThermoConfig;

#endif /* INC_COREUTILS_CONFIG_THERMO_CONFIG_H_ */
