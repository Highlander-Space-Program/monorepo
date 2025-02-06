/*
 * heater_state_machine.h
 *
 *  Created on: Jan 27, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_HEATER_STATE_MACHINE_H_
#define INC_HEATER_STATE_MACHINE_H_

#include "heater_utils.h"
#include "config/servo_board_config.h"


//States of the heater
typedef enum {
  DIR_OFF = 0,
  DIR_ON = 1,
  DIR_AUTO = 2
} HEATER_DIRECTIVE;

//States of the heater
typedef enum {
  OFF = 0,
  ON = 1,
} HEATER_STATE;

//commands for the heater
typedef enum  {
  H_ON = 0,
  H_OFF = 1,
  H_AUTO = 2
} HEATER_CMD;

typedef struct  {
  HeaterConfig* heater_config;
  HEATER_STATE state;
  HEATER_DIRECTIVE directive;
  Thermocouple* thermo;
  uint32_t last_tick_update;
} Heater;

Heater* construct_heater (const uint32_t board_uid[3], Thermocouple* thermo);
void Tick_HEATER (uint8_t cmd, Heater* heater);

Heater* construct_heater (const uint32_t board_uid[3], Thermocouple* thermo) {
    HeaterConfig *hc = GET_HEATER_CONFIG((uint32_t *)board_uid);
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

void Heater_Auto () {

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
