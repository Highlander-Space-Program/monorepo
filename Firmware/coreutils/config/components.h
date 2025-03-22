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

// BoardID is acquired from the F042Diagnostic. Use debugger to see values inside uid variable after board id is acquired.

// CAN_ID is acquired based on a scheme. The first 11 bits are 0. the
// Sender field (8 bits - first part of ID, highest priority impact):
// - 0: Break wire system (highest priority)
// - 1: Pad controller
// - 2: Servo board
// - 3: Sensor board
// - 4: Tester board (this code and controller)
// - 254: Tester board (hardware)
// - 255: PC/Terminal (lowest priority)
// - 5-253: Reserved for future devices
//
// Board ID field (8 bits - identifies specific physical boards):
// - 0-255: Unique identifier for each physical board in the system
// - This allows up to 256 individual boards on the network
//
// Component type field (8 bits - defines what type of component is being addressed):
// - 0: System/board control
// - 1: Servo
// - 2: Thermocouple
// - 3: Pressure transducer
// - 4: Heater
// - 5: LED
// - 6: Flash Status
// - 8-255: Reserved for future component types
//
// Instance field (5 bits - identifies specific component instance):
// - 0-31: Allows up to 32 instances of each component type per board
// - For example, a board could have up to 32 servos, 32 thermocouples, etc.
//
// Example of a full extended ID:
// 0x01050401 (hex) = 00000001 00000101 00000100 00001 000 (binary)
//                     |        |        |        |     |
//                     |        |        |        |		+-- Padding
//					   |		|		 |		  +-------- Instance 1
//                     |        |        +----------------- Component Type 4 (Heater)
//                     |        +-------------------------- Board ID 5
//                     +----------------------------------- Sender 1 (Pad Controller)
//
// This example ID represents: "Pad Controller (1) sending a message to Board 5,
// addressing Heater (4), instance 1"

// BoardID, CAN_ID, NAME, OPEN_ANGLE, CLOSED_ANGLE, UPDATE_FREQUENCY
ServoConfig servo_lookup_table[] = {
	{{0x0032002D, 0x48585314, 0x20373733}, 0x00010108, "FV-N02", 0, 135, -1},
	{{0x00390043, 0x48585311, 0x20373733}, 0x00020108, "FV-N03", 0, 45, -1},
	{{0x00310043, 0x48585311, 0x20373733}, 0x00030108, "FV-N04", 0, 90, -1}
};

// BoardID, CAN_ID (29 bits), NAME, OPEN_ANGLE, CLOSED_ANGLE, UPDATE_FREQUENCY
ThermoConfig thermo_lookup_table[] = {
    {{0x0032002D, 0x48585314, 0x20373733}, 0x00010208, "TC-02", 100},
    {{0x00390043, 0x48585311, 0x20373733}, 0x00020208, "TC-03", 100},
    {{0x00310043, 0x48585311, 0x20373733}, 0x00030208, "TC-04", 100}
};

// BoardID, CAN_ID (29 bits), NAME, OPEN_ANGLE, CLOSED_ANGLE, UPDATE_FREQUENCY
HeaterConfig heater_lookup_table[] = {
	{{0x0032002D, 0x48585314, 0x20373733}, 0x00010408, "H-02", 29, 27, 100},
	{{0x00390043, 0x48585311, 0x20373733}, 0x00020408, "H-03", 30, 25, 100},
	{{0x00310043, 0x48585311, 0x20373733}, 0x00030408, "H-04", 30, 25, 100}
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
