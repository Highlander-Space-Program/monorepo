/*
 * temperature_state_machine.h
 *
 *  Created on: Jan 27, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_TEMPERATURE_STATE_MACHINE_H_
#define INC_TEMPERATURE_STATE_MACHINE_H_

#include <thermocouple_utils.h>
#include <float.h>

#include "config/thermo_config.h"

typedef struct {
  ThermoConfig* thermo_config;
  THERMO_STATE state;
  ADC_HandleTypeDef* adc;
  float temperature;
  uint32_t* adc_val;
  uint32_t last_tick_update; // Use HAL_GetTick() for tracking time
} Thermocouple;

static uint8_t short_board_id;
extern CAN_HandleTypeDef hcan;

//Function Prototypes
void Tick_THERMO (uint8_t cmd, Thermocouple* thermo);
Thermocouple* construct_thermo (const uint32_t can_id, ADC_HandleTypeDef *adc, volatile uint32_t* adc_val);
void send_thermo_status_can(Thermocouple* thermo, uint8_t short_board_id, CAN_HandleTypeDef* hcan);

/**
  * @brief  The construct_thermo function is used to create a thermocouple.
  * @param  uid This universally unique identification is based on the
  * 		microcontroller on the thermo board to allow for different
  * 		positions of each thermocouple
  * @param  timer This is the timer used for the PWM as well as the 10
  * 		second counter which determines when to turn the thermo off
  *
  * @retval Thermocouple* returns a pointer to the constructed thermo.
  */

void send_thermo_status_can(Thermocouple* thermo, uint8_t short_board_id, CAN_HandleTypeDef* hcan) {
	uint8_t status_bytes[4];
	float temp = thermo->temperature;
	uint8_t temp_raw[4];

	memcpy(temp_raw, &temp, sizeof(temp));
	status_bytes[0] = temp_raw[3];
	status_bytes[1] = temp_raw[2];
	status_bytes[2] = temp_raw[1];
	status_bytes[3] = temp_raw[0];

	uint8_t sender_id = short_board_id;
    uint8_t target_board_id = SENDER_PAD_CONTROLLER;
    uint8_t msg_type_val = MSG_TYPE_THERMOCOUPLE; // Using existing general servo type
    uint8_t instance_val = 1;

    uint32_t base_ext_id_32bit_shifted = build_can_extended_id(sender_id, target_board_id, msg_type_val, instance_val);
    uint32_t base_ext_id_29bit = base_ext_id_32bit_shifted >> 3;
    uint32_t ack_ext_id_29bit = base_ext_id_29bit | CAN_ID_ACK_FLAG_29BIT;
    uint32_t final_ext_id_32bit_shifted_for_send = ack_ext_id_29bit << 3;

    HAL_StatusTypeDef status = send_can_msg(final_ext_id_32bit_shifted_for_send, status_bytes, 4, hcan);

    if (status != HAL_OK) {
        // Handle CAN send error
    }
}

Thermocouple* construct_thermo (const uint32_t can_id, ADC_HandleTypeDef *adc, volatile uint32_t* adc_val) {
    ThermoConfig *tc = GET_THERMO_CONFIG(can_id);
    if (!tc) {
        return NULL;
    }

    Thermocouple *thermo = malloc(sizeof(Thermocouple));
    if (!thermo) {
        return NULL;
    }


    thermo->thermo_config    = tc;
    thermo->state            = TEMP_WAIT;
    thermo->last_tick_update = HAL_GetTick();
    thermo->adc 			 = adc;
    thermo->adc_val 		 = adc_val;

    HAL_ADCEx_Calibration_Start(thermo->adc);
	HAL_ADC_Start_DMA(thermo->adc, thermo->adc_val,1);
	thermo->temperature = Get_Temperature(*(thermo->adc_val));

    return thermo;
}

void Tick_THERMO (uint8_t cmd, Thermocouple* thermo) {
	// Changing the states
	switch (cmd){
		case FORCE_GET_TEMP:
			thermo->state = TEMP_GET;
			break;
		case FORCE_RESET_THERMO_TIMER:
			thermo->last_tick_update = HAL_GetTick();
			break;
	}

    // Check if frequency time has passed and then get another measurement for thermo
    if ((HAL_GetTick() - thermo->last_tick_update) >= 1000.0 / thermo->thermo_config->frequency) {
    	thermo->state = TEMP_GET;
    }

	// acting based on the state
	switch (thermo->state) {
		case TEMP_WAIT:
			break;
		case TEMP_GET:
	        thermo->last_tick_update = HAL_GetTick();

	    	HAL_ADC_Start_DMA(thermo->adc, thermo->adc_val,1);
	    	thermo->temperature = Get_Temperature(*(thermo->adc_val));
	    	send_thermo_status_can (thermo, short_board_id, &hcan);
	    	thermo->state = TEMP_WAIT;
			break;
	}
}

#endif /* INC_TEMPERATURE_STATE_MACHINE_H_ */
