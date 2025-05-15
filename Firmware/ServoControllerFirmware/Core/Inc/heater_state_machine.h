/*
 * heater_state_machine.h
 *
 *  Created on: Jan 27, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_HEATER_STATE_MACHINE_H_
#define INC_HEATER_STATE_MACHINE_H_

#include "heater_utils.h"
#include "config/heater_config.h"
#include "config/config.h"

typedef struct  {
  HeaterConfig* heater_config;
  HEATER_STATE state;
  HEATER_DIRECTIVE directive;
  Thermocouple* thermo;
  uint32_t last_tick_update;
} Heater;

Heater* construct_heater (const uint32_t can_id, Thermocouple* thermo);
void Tick_HEATER (uint8_t cmd, Heater* heater);

Heater* construct_heater (const uint32_t can_id, Thermocouple* thermo) {
    HeaterConfig *hc = GET_HEATER_CONFIG(can_id);
    if (!hc) {
        return NULL;
    }

    Heater *heater = malloc(sizeof(Heater));
    if (!heater) {
        return NULL;
    }

    heater->heater_config    = hc;
    heater->state            = OFF;
    heater->directive 		 = DIR_OFF;
    heater->last_tick_update = HAL_GetTick();
    heater->thermo 			 = thermo;
    return heater;
}

void send_heater_status_can(Heater* heater, uint8_t short_board_id, CAN_HandleTypeDef *hcan) {
    uint8_t status_byte = (uint8_t)heater->directive; // HEATER_IS_ON (1) or HEATER_IS_OFF (0)
    uint8_t sender_id = short_board_id;
    uint8_t target_board_id = SENDER_PAD_CONTROLLER;
    uint8_t msg_type_val = MSG_TYPE_HEATER; // Using existing general heater type
    uint8_t instance_val = 0;

    uint32_t base_ext_id_32bit_shifted = build_can_extended_id(sender_id, target_board_id, msg_type_val, instance_val);
    uint32_t base_ext_id_29bit = base_ext_id_32bit_shifted >> 3;
    uint32_t ack_ext_id_29bit = base_ext_id_29bit | CAN_ID_ACK_FLAG_29BIT;
    uint32_t final_ext_id_32bit_shifted_for_send = ack_ext_id_29bit << 3;

    HAL_StatusTypeDef status = send_can_msg(final_ext_id_32bit_shifted_for_send, &status_byte, 1, hcan);

    if (status != HAL_OK) {
        // Handle CAN send error
    }
}
void Tick_HEATER (uint8_t cmd, Heater* heater) {

	//----------TRANSITIONS----------
	switch (cmd) {
		case H_OFF:
			heater->directive = DIR_OFF;
		break;

		case H_ON:
			heater->directive = DIR_ON;
		break;

		case H_AUTO:
			heater->directive = DIR_AUTO;
	  	break;
	}


	if ((HAL_GetTick() - heater->last_tick_update) >= 1.0 / heater->heater_config->frequency) {
		heater->last_tick_update = HAL_GetTick();

		//------ACTIONS----------
		switch (heater->directive) {
			case DIR_OFF:
				heater->state = OFF;
			break;

			case DIR_ON:
				heater->state = ON;
			break;

			case DIR_AUTO:
				if (heater->thermo->temperature > heater->heater_config->off_temp) {
					heater->state = OFF;
				} else if (heater->thermo->temperature < heater->heater_config->on_temp) {
					heater->state = ON;
				}
			break;
		}

		switch (heater->state) {
			case OFF:
				Heater_Off();
			break;

			case ON:
				Heater_On();
			break;
		}
	}
}


#endif /* INC_HEATER_STATE_MACHINE_H_ */
