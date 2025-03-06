/*
 * components.h
 *
 *  Created on: Mar 5, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_COMPONENTS_H_
#define INC_COREUTILS_CONFIG_COMPONENTS_H_


#include <string.h>

#include "servo_config.h"
#include "thermo_config.h"
#include "heater_config.h"

// UID, NAME, OPEN_ANGLE, CLOSED_ANGLE, UPDATE_FREQUENCY
ServoConfig servo_lookup_table[] = {
	{{0x0039002c, 0x48585314, 0x20373733}, "FV-N02", 0, 45, -1},
	{{0x0032002D, 0x48585314, 0x20373733}, "FV-N03", 0, 135, -1}
};

// UID, NAME, UPDATE_FREQUENCY
ThermoConfig thermo_lookup_table[] = {
    {{0x39002c, 0x48585314, 0x20373733}, "TC-02", 100},
    {{0x0032002D, 0x48585314, 0x20373733}, "TC-03", 100}
};

// UID, NAME, HIGHEST_TEMP, LOWEST_TEMP, UPDATE_FREQUENCY
HeaterConfig heater_lookup_table[] = {
	{{0x39002c, 0x48585314, 0x20373733}, "H-02", 30, 25, 100},
	{{0x0032002D, 0x48585314, 0x20373733}, "H-03", 29, 27, 100}
};

ServoConfig* GET_SERVO_CONFIG(uint32_t* uid) {
    for (int i = 0; i < sizeof(servo_lookup_table) / sizeof(servo_lookup_table[0]); i++) {
        if (memcmp(uid, servo_lookup_table[i].uid, sizeof(servo_lookup_table[i].uid)) == 0) {
            return &servo_lookup_table[i]; // Return pointer to matching ServoConfig
        }
    }
    return NULL; // Return NULL if no match is found
}

ServoConfig* GET_SERVO_CONFIGS() {
	return servo_lookup_table;
}

ThermoConfig* GET_THERMO_CONFIGS() {
	return thermo_lookup_table;
}

ThermoConfig* GET_THERMO_CONFIG(uint32_t* uid) {
    for (int i = 0; i < sizeof(thermo_lookup_table) / sizeof(thermo_lookup_table[0]); i++) {
        if (memcmp(uid, thermo_lookup_table[i].uid, sizeof(thermo_lookup_table[i].uid)) == 0) {
            return &thermo_lookup_table[i]; // Return pointer to matching ServoConfig
        }
    }
    return NULL; // Return NULL if no match is found
}

HeaterConfig* GET_HEATER_CONFIGS() {
	return heater_lookup_table;
}

HeaterConfig* GET_HEATER_CONFIG(uint32_t* uid) {
    for (int i = 0; i < sizeof(heater_lookup_table) / sizeof(heater_lookup_table[0]); i++) {
        if (memcmp(uid, heater_lookup_table[i].uid, sizeof(heater_lookup_table[i].uid)) == 0) {
            return &heater_lookup_table[i]; // Return pointer to matching ServoConfig
        }
    }
    return NULL; // Return NULL if no match is found
}


#endif /* INC_COREUTILS_CONFIG_COMPONENTS_H_ */
