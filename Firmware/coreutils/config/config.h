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

#define COMP_TYPE_SYSTEM 0
#define COMP_TYPE_SERVO 1
#define COMP_TYPE_THERMOCOUPLE 2
#define COMP_TYPE_PRESSURE 3
#define COMP_TYPE_HEATER 4
#define COMP_TYPE_LED 5

#define SENDER_BREAK_WIRE 0
#define SENDER_PAD_CONTROLLER 1
#define SENDER_SERVO_BOARD 2
#define SENDER_SENSOR_BOARD 3
#define SENDER_TESTER_BOARD 4
#define SENDER_HW_TESTER 254
#define SENDER_PC 255


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

    // Return 0 if not found
    return -1;
}

uint8_t GET_SHORT_BOARD_ID (uint32_t* board_uid) {
	uint32_t can_id = GET_CAN_ID_FROM_BOARD_UID(board_uid);
	return (can_id >> 16) & 0xFF;
}


/**
 * Gets board UID from CAN ID by searching through configuration tables
 *
 * @param can_id CAN ID to look up
 * @return Pointer to the board UID, or NULL if not found
 */
uint32_t* GET_BOARD_UID_FROM_CAN_ID(uint32_t can_id) {
    ServoConfig* servo_lookup_table = GET_SERVO_CONFIGS();
    ThermoConfig* thermo_lookup_table = GET_THERMO_CONFIGS();
    HeaterConfig* heater_lookup_table = GET_HEATER_CONFIGS();

    // Check servo configurations
    for (int i = 0; i < GET_NUM_SERVO_CONFIGS(); i++) {
        if (servo_lookup_table[i].can_id == can_id) {
            return servo_lookup_table[i].board_uid;
        }
    }

    // Check thermo configurations
    for (int i = 0; i < GET_NUM_THERMO_CONFIGS(); i++) {
        if (thermo_lookup_table[i].can_id == can_id) {
            return thermo_lookup_table[i].board_uid;
        }
    }

    // Check heater configurations
    for (int i = 0; i < GET_NUM_HEATER_CONFIGS(); i++) {
        if (heater_lookup_table[i].can_id == can_id) {
            return heater_lookup_table[i].board_uid;
        }
    }

    // Return NULL if not found
    return NULL;
}

#endif /* INC_CONFIG_H_ */
