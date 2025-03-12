/*
 * components.h
 *
 *  Created on: Mar 11, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_COMPONENTS_H_
#define INC_COREUTILS_CONFIG_COMPONENTS_H_

#include <string.h>
#include "thermo_config.h"
#include "heater_config.h"
#include "servo_config.h"

// first 11 bits are always 0 for UID
// BoardID, CAN_ID, NAME, OPEN_ANGLE, CLOSED_ANGLE, UPDATE_FREQUENCY
ServoConfig servo_lookup_table[] = {
//	{{0x0039002C, 0x48585314, 0x20373733}, 0x00010108, "FV-N02", 0, 45, -1},
	{{0x0032002D, 0x48585314, 0x20373733}, 0x00010108, "FV-N03", 0, 135, -1}
};

// BoardID, CAN_ID (29 bits), NAME, OPEN_ANGLE, CLOSED_ANGLE, UPDATE_FREQUENCY
ThermoConfig thermo_lookup_table[] = {
//    {{0x39002C, 0x48585314, 0x20373733}, 0x00010208, "TC-02", 100},
    {{0x0032002D, 0x48585314, 0x20373733}, 0x00010208, "TC-03", 100}
};

// BoardID, CAN_ID (29 bits), NAME, OPEN_ANGLE, CLOSED_ANGLE, UPDATE_FREQUENCY
HeaterConfig heater_lookup_table[] = {
//	{{0x39002C, 0x48585314, 0x20373733}, 0x00010408, "H-02", 30, 25, 100},
	{{0x0032002D, 0x48585314, 0x20373733}, 0x00010408, "H-03", 29, 27, 100}
};

ServoConfig* GET_SERVO_CONFIGS() {
	return servo_lookup_table;
}

ThermoConfig* GET_THERMO_CONFIGS() {
	return thermo_lookup_table;
}

HeaterConfig* GET_HEATER_CONFIGS() {
	return heater_lookup_table;
}


size_t GET_NUM_SERVO_CONFIGS() {
    return sizeof(servo_lookup_table) / sizeof(servo_lookup_table[0]);
}

size_t GET_NUM_THERMO_CONFIGS() {
    return sizeof(thermo_lookup_table) / sizeof(thermo_lookup_table[0]);
}

size_t GET_NUM_HEATER_CONFIGS() {
    return sizeof(heater_lookup_table) / sizeof(heater_lookup_table[0]);
}


ServoConfig* GET_SERVO_CONFIG(const uint32_t can_id) {
    for (int i = 0; i < sizeof(servo_lookup_table) / sizeof(servo_lookup_table[0]); i++) {
        if (can_id == servo_lookup_table[i].can_id) {
            return &servo_lookup_table[i]; // Return pointer to matching ServoConfig
        }
    }
    return NULL; // Return NULL if no match is found
}

ThermoConfig* GET_THERMO_CONFIG(const uint32_t can_id) {
    for (int i = 0; i < sizeof(thermo_lookup_table) / sizeof(thermo_lookup_table[0]); i++) {
    	if (can_id == thermo_lookup_table[i].can_id) {
            return &thermo_lookup_table[i]; // Return pointer to matching ServoConfig
        }
    }
    return NULL; // Return NULL if no match is found
}

HeaterConfig* GET_HEATER_CONFIG(const uint32_t can_id) {
    for (int i = 0; i < sizeof(heater_lookup_table) / sizeof(heater_lookup_table[0]); i++) {
    	if (can_id == heater_lookup_table[i].can_id) {
            return &heater_lookup_table[i]; // Return pointer to matching ServoConfig
        }
    }
    return NULL; // Return NULL if no match is found
}


// instance-- is after because zero indexed
uint32_t GET_SERVO_CAN_ID(const uint32_t* board_uid, uint8_t instance) {
	for (int i = 0; i < sizeof(servo_lookup_table) / sizeof(servo_lookup_table[0]); i++) {
		if (memcmp(board_uid, servo_lookup_table[i].board_uid, sizeof(servo_lookup_table[i].board_uid)) == 0) {
			if (instance == 0) {
				return servo_lookup_table[i].can_id;
			}
			instance--;
		}
	}
	return -1;
}

uint32_t GET_THERMO_CAN_ID(const uint32_t* board_uid, uint8_t instance) {
	for (int i = 0; i < sizeof(thermo_lookup_table) / sizeof(thermo_lookup_table[0]); i++) {
		if (memcmp(board_uid, thermo_lookup_table[i].board_uid, sizeof(thermo_lookup_table[i].board_uid)) == 0) {
			if (instance == 0) {
				return thermo_lookup_table[i].can_id;
			}
			instance--;
		}
	}
	return -1;
}

uint32_t GET_HEATER_CAN_ID(const uint32_t* board_uid, uint8_t instance) {
	for (int i = 0; i < sizeof(heater_lookup_table) / sizeof(heater_lookup_table[0]); i++) {
		if (memcmp(board_uid, heater_lookup_table[i].board_uid, sizeof(heater_lookup_table[i].board_uid)) == 0) {
			if (instance == 0) {
				return heater_lookup_table[i].can_id;
			}
			instance--;
		}
	}
	return -1;
}

#endif /* INC_COREUTILS_CONFIG_COMPONENTS_H_ */
