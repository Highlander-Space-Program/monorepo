/*
 * config.h
 *
 *  Created on: Jan 29, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "sensor_board_config.h"
#include "components.h"
#include "pad_config.h"
#include "flight_config.h"
#include <stdbool.h>


extern bool isAutoArmed;

enum MESSAGE_TYPE {
     MSG_TYPE_SYSTEM = 0,
     MSG_TYPE_SERVO = 1,
     MSG_TYPE_THERMOCOUPLE = 2,
     MSG_TYPE_PRESSURE = 3,
     MSG_TYPE_HEATER = 4,
     MSG_TYPE_LED = 5,
     MSG_TYPE_FLASH_SIGNAL = 6
};

enum BOARD_CAN_ID_MAPPING {
	SENDER_PAD_CONTROLLER = 1,
	SENDER_SERVO_BOARD = 2,
	SENDER_SENSOR_BOARD = 3,
	SENDER_TESTER_BOARD = 4,
	SENDER_HW_TESTER = 254,
	SENDER_PC = 255
};

enum COMMANDS {
  SIGNAL_ALL = 0,  // previously OPEN_EO1 = 0,
  REPORT_ALL = 1,  // previously CLOSE_EO1 = 1,
//  OPEN_NO6 = 2,
//  CLOSE_NO6 = 3,
  OPEN_NO4 = 4,
  CLOSE_NO4 = 5,
  OPEN_NO3 = 6,
  CLOSE_NO3 = 7,
  START_1 = 8,
  OPEN_NO2 = 9,
  CLOSE_NO2 = 10,
  AUTO_ON = 12,
  AUTO_OFF = 13,
  ACTIVATE_IGNITER = 14,
  DEACTIVATE_IGNITER = 15,
  ABORT = 16,
  ACTIVATE_SERVOS = 17,
  DEACTIVATE_SERVOS = 18,
  DEABORT = 19,
  CHECK_STATE = 20,
  DESTART = 21,

};

void GET_BOARD_UID (uint32_t* board_uid) {
  board_uid[0] = HAL_GetUIDw0();
  board_uid[1] = HAL_GetUIDw1();
  board_uid[2] = HAL_GetUIDw2();
}

/**
 * Gets board ID from CAN ID by searching through configuration tables
 *
 * @param board_uid Pointer to board's unique identifier
 * @return uint16_t CAN ID corresponding to the board UID, or 0 if not found
 */
uint32_t GET_CAN_ID_FROM_BOARD_UID(uint32_t* board_uid) {
    ServoConfig* servo_lookup_table = GET_SERVO_CONFIGS();
    ThermoConfig* thermo_lookup_table = GET_THERMO_CONFIGS();
    HeaterConfig* heater_lookup_table = GET_HEATER_CONFIGS();
    PtConfig* pt_lookup_table = GET_PT_CONFIGS();

    // Check servo configurations
    for (int i = 0; i < GET_NUM_SERVO_CONFIGS(); i++) {
        if (memcmp(board_uid, servo_lookup_table[i].board_uid, sizeof(servo_lookup_table[i].board_uid)) == 0) {
            return servo_lookup_table[i].can_id;
        }
    }

    // Check thermo configurations
    for (int i = 0; i < GET_NUM_THERMO_CONFIGS(); i++) {
        if (memcmp(board_uid, thermo_lookup_table[i].board_uid, sizeof(thermo_lookup_table[i].board_uid)) == 0) {
            return thermo_lookup_table[i].can_id;
        }
    }

    // Check heater configurations
    for (int i = 0; i < GET_NUM_HEATER_CONFIGS(); i++) {
        if (memcmp(board_uid, heater_lookup_table[i].board_uid, sizeof(heater_lookup_table[i].board_uid)) == 0) {
            return heater_lookup_table[i].can_id;
        }
    }

    // Check pt configurations
    for (int i = 0; i < GET_NUM_PT_CONFIGS(); i++) {
    	if (memcmp(board_uid, pt_lookup_table[i].board_uid, sizeof(pt_lookup_table[i].board_uid)) == 0) {
    		return pt_lookup_table[i].can_id;
    	}
    }

    // Return 0 if not found
    return -1;
}

uint32_t* GET_BOARD_ID_FROM_PNID(char* pnid) {
    ServoConfig* servo_lookup_table = GET_SERVO_CONFIGS();
    ThermoConfig* thermo_lookup_table = GET_THERMO_CONFIGS();
    HeaterConfig* heater_lookup_table = GET_HEATER_CONFIGS();
    PtConfig* pt_lookup_table = GET_PT_CONFIGS();

    // Check servo configurations
    for (int i = 0; i < GET_NUM_SERVO_CONFIGS(); i++) {
        if (strcmp(pnid, servo_lookup_table[i].pnid) == 0) {
            return servo_lookup_table[i].board_uid;
        }
    }

    // Check thermo configurations
    for (int i = 0; i < GET_NUM_THERMO_CONFIGS(); i++) {
    	if (strcmp(pnid, thermo_lookup_table[i].pnid) == 0) {
            return thermo_lookup_table[i].board_uid;
        }
    }

    // Check heater configurations
    for (int i = 0; i < GET_NUM_HEATER_CONFIGS(); i++) {
    	if (strcmp(pnid, heater_lookup_table[i].pnid) == 0) {
            return heater_lookup_table[i].board_uid;
        }
    }


    for (int i = 0; i < GET_NUM_PT_CONFIGS(); i++) {
    	if (strcmp(pnid, pt_lookup_table[i].pnid) == 0) {
    		return pt_lookup_table[i].board_uid;
    	}
    }

    // Return NULL if not found
    return NULL;
}

uint8_t GET_SHORT_BOARD_ID (uint32_t* board_uid) {
	BoardConfig* board_lookup_table = GET_BOARD_CONFIGS();

	for (int i = 0; i < GET_NUM_BOARD_CONFIGS(); ++i) {
		if (memcmp(board_uid, board_lookup_table[i].uid, sizeof(board_lookup_table[i].uid)) == 0) {
			return board_lookup_table[i].short_id;
		}
	}
	return -1;
}

/*
 * Gets short board ID from can ID
 *
 * @param can_id CAN ID to look up
 * @return uint8_t short board ID
 */

uint8_t GET_SHORT_BOARD_ID_FROM_CAN_ID(uint32_t can_id) {
	return (can_id >> 16) & 0xFF;
}

/**
 * Gets board UID from CAN ID by searching through configuration tables
 *
 * @param can_id CAN ID to look up
 * @return Pointer to the board UID, or NULL if not found
 */
uint32_t* GET_BOARD_UID_FROM_CAN_ID(uint32_t can_id) {
    BoardConfig* board_lookup_table = GET_BOARD_CONFIGS();
    uint8_t board_short_id = GET_SHORT_BOARD_ID_FROM_CAN_ID(can_id);

    for (int i = 0; i < GET_NUM_BOARD_CONFIGS(); ++i) {
		if (board_short_id == board_lookup_table[i].short_id) {
			return board_lookup_table[i].uid;
		}
	}

    // Return NULL if not found
    return NULL;
}

#endif /* INC_CONFIG_H_ */
