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

typedef enum {
	TEMP_WAIT = 0,
	TEMP_GET = 1
} THERMO_STATE;

typedef enum {
	FORCE_GET_TEMP = 0,
	FORCE_RESET_TIMER = 1
} THERMO_CMD;

typedef struct {
  ThermoConfig* thermo_config;
  THERMO_STATE state;
  ADC_HandleTypeDef* adc;
  double temperature;
  uint32_t adc_val;
  uint32_t last_tick_update; // Use HAL_GetTick() for tracking time
} Thermocouple;



//Function Prototypes
void Tick_THERMO (uint8_t cmd, Thermocouple* thermo);
struct Thermocouple* construct_thermocouple (const uint32_t board_uid[3]);

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
Thermocouple* construct_thermo (const uint32_t board_uid[3], ADC_HandleTypeDef* adc) {
    ThermoConfig *tc = GET_THERMO_CONFIG((uint32_t *)board_uid);
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

    HAL_ADCEx_Calibration_Start(thermo->adc);
	HAL_ADC_Start_DMA(thermo->adc,(uint32_t*)thermo->adc_val,1);
	thermo->temperature = DBL_MAX;

    return thermo;
}

void Tick_THERMO (uint8_t cmd, Thermocouple* thermo) {
	// Changing the states
	switch (cmd){
		case FORCE_GET_TEMP:
			thermo->state = TEMP_GET;
			break;
		case FORCE_RESET_TIMER:
			thermo->last_tick_update = HAL_GetTick();
			break;
	}

    // Check if frequency time has passed and then get another measurement for thermo
    if ((HAL_GetTick() - thermo->last_tick_update) >= 1.0 / thermo->thermo_config->frequency) {
    	thermo->state = TEMP_GET;
    }

	// acting based on the state
	switch (thermo->state) {
		case TEMP_WAIT:
			break;
		case TEMP_GET:
	        thermo->last_tick_update = HAL_GetTick();

	    	HAL_ADC_Start_DMA(thermo->adc,(uint32_t*)thermo->adc_val,1);

			thermo->temperature = Get_Temperature(thermo->adc_val);
			break;
	}
}

#endif /* INC_TEMPERATURE_STATE_MACHINE_H_ */
