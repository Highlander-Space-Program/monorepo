/*
 * servo_config.h
 *
 *  Created on: Jan 30, 2025
 *      Author: brandonmarcus
 */

/*
 * The ServoInfo provides information
 */

#ifndef INC_COREUTILS_CONFIG_SERVO_BOARD_CONFIG_H_
#define INC_COREUTILS_CONFIG_SERVO_BOARD_CONFIG_H_

#include <string.h>

struct Servo;
struct Thermocouple;
struct Heater;

typedef struct {
	uint32_t uid[3];
	char* pnid;
	int closed_deg;
	int open_deg;
	uint16_t frequency;
} ServoConfig;

typedef struct {
    uint32_t uid[3];
    char* pnid;
    uint16_t frequency;
} ThermoConfig;

typedef struct {
	uint32_t uid[3];
	char* pnid;
	int off_temp;
	int on_temp;
	uint16_t frequency;
} HeaterConfig;

// defaults to as fast as possible
ServoConfig servo_lookup_table[] = {
	{{0x0039002c, 0x48585314, 0x20373733}, "FV-N02", 0, 45, -1},
	{{0x0032002D, 0x48585314, 0x20373733}, "FV-N03", 0, 135, -1}
};

// defaults to 100hz
ThermoConfig thermo_lookup_table[] = {
    {{0x39002c, 0x48585314, 0x20373733}, "TC-02", 100},
    {{0x0032002D, 0x48585314, 0x20373733}, "TC-03", 100}
};

// defaults to 100hz
HeaterConfig heater_lookup_table[] = {
	{{0x39002c, 0x48585314, 0x20373733}, "H-02", 30, 25, 100},
	{{0x0032002D, 0x48585314, 0x20373733}, "H-03", 30, 25, 100}
};


ServoConfig* GET_SERVO_CONFIGS() {
	return servo_lookup_table;
}

ServoConfig* GET_SERVO_CONFIG(uint32_t* uid) {
    for (int i = 0; i < sizeof(servo_lookup_table) / sizeof(servo_lookup_table[0]); i++) {
        if (memcmp(uid, servo_lookup_table[i].uid, sizeof(servo_lookup_table[i].uid)) == 0) {
            return &servo_lookup_table[i]; // Return pointer to matching ServoConfig
        }
    }
    return NULL; // Return NULL if no match is found
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


#endif /* INC_COREUTILS_CONFIG_SERVO_BOARD_CONFIG_H_ */
