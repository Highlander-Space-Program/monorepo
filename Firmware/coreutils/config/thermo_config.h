/*
 * thermo_config.h
 *
 *  Created on: Mar 5, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_THERMO_CONFIG_H_
#define INC_COREUTILS_CONFIG_THERMO_CONFIG_H_

#include <string.h>

struct Thermocouple;

typedef struct {
    uint32_t uid[3];
    char* pnid;
    uint16_t frequency;
} ThermoConfig;

#endif /* INC_COREUTILS_CONFIG_THERMO_CONFIG_H_ */
