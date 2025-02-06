/*
 * servo_state_machine.h
 *
 *  Created on: Jan 2, 2025
 *      Author: brandonmarcus
 */

/*
 * SERVO_STATE is the current state of the servo
 * SERVO_CMD is an enum of commands that the servo will respond to
 * Servo is an object that holds all everything relevant to a single servo
 */

#ifndef INC_SERVO_STATE_MACHINE_H_
#define INC_SERVO_STATE_MACHINE_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f0xx_hal.h"

#include "servo_utils.h"
#include "config/servo_board_config.h"

#define HSP_SERVO_MIN_PULSE_WIDTH 500
#define HSP_SERVO_MAX_PULSE_WIDTH 2500
#define HSP_SERVO_PWM_PERIOD 20000
#define HSP_SERVO_WAIT_TIME 10000 // in milliseconds

// Forward declarations
typedef enum {
	ON_CLOSED = 0,
	OFF_CLOSED = 1,
	ON_OPEN = 2,
	OFF_OPEN = 3
} SERVO_STATE;

typedef enum {
	OPEN_SERVO = 0,
	CLOSE_SERVO = 1,
	CHECK_SERVO = 2
} SERVO_CMD;

typedef struct  {
  ServoConfig* servo_config;
  SERVO_STATE state;
  uint32_t last_tick_update;
  const TIM_HandleTypeDef *timer;
  volatile uint32_t *ccr;
} Servo;

//Function Prototypes
void Tick_SERVO (uint8_t cmd, Servo* servo);
Servo* construct_servo (const uint32_t board_uid[3], const TIM_HandleTypeDef *timer);

/**
  * @brief  The construct_servo function is used to create a servo.
  * @param  uid This universally unique identification is based on the
  * 		microcontroller on the servo board to allow for different
  * 		positions of each servo
  * @param  timer This is the timer used for the PWM as well as the 10
  * 		second counter which determines when to turn the servo off
  *
  * @retval struct Servo* returns a pointer to the constructed servo.
  */
Servo* construct_servo (const uint32_t board_uid[3], const TIM_HandleTypeDef *timer) {
    ServoConfig *sc = GET_SERVO_CONFIG((uint32_t *)board_uid);
    if (!sc) {
        return NULL;
    }

    Servo *servo = malloc(sizeof(Servo));
    if (!servo) {
        return NULL;
    }

    servo->servo_config     = sc;
    servo->state            = ON_CLOSED;
    servo->last_tick_update = HAL_GetTick();
    servo->timer            = timer;
    servo->ccr              = &(timer->Instance->CCR1);

    return servo;
}

void Tick_SERVO (uint8_t cmd, Servo* servo) {
	// Changing the states
	switch (cmd){
		case OPEN_SERVO:
			if (servo->state == ON_CLOSED) {
				servo->state = ON_OPEN;
				servo->last_tick_update = HAL_GetTick();
			}
			else if (servo->state == OFF_CLOSED) {
				servo->state = ON_OPEN;
				servo->last_tick_update = HAL_GetTick();
			}
			else if (servo->state == ON_OPEN) {/*Not Used*/}
			else if (servo->state == OFF_OPEN) {/*Not Used*/}
			break;
		case CLOSE_SERVO:
			if (servo->state == ON_CLOSED) {/*Not Used*/}
			else if (servo->state == OFF_CLOSED) {/*Not Used*/}
			else if  (servo->state == ON_OPEN) {
				servo->state = ON_CLOSED;
				servo->last_tick_update = HAL_GetTick();
			}
			else if  (servo->state == OFF_OPEN) {
				servo->state = ON_CLOSED;
				servo->last_tick_update = HAL_GetTick();
			}
			break;
	}

    // Check if 10 seconds have passed and turn off servo
    if ((HAL_GetTick() - servo->last_tick_update) >= HSP_SERVO_WAIT_TIME) {
        if (servo->state == ON_CLOSED) {
            servo->state = OFF_CLOSED;
        } else if (servo->state == ON_OPEN) {
            servo->state = OFF_OPEN;
        }
        // Reset the tick update time even if the servo is already off
        servo->last_tick_update = HAL_GetTick();
    }

	// acting based on the state
	switch (servo->state) {
		case ON_CLOSED:
			Turn_On_Servo ();
			STATUS_IND_On ();
			Actuate_Servo (servo->servo_config->closed_deg, servo->timer);
			break;
		case OFF_CLOSED:
			Turn_Off_Servo ();
			STATUS_IND_Off ();
			break;
		case ON_OPEN:
			Turn_On_Servo ();
			STATUS_IND_On ();
			Actuate_Servo (servo->servo_config->open_deg, servo->timer);
			break;
		case OFF_OPEN:
			Turn_Off_Servo ();
			STATUS_IND_Off ();
			break;
	}
}

#endif /* INC_SERVO_STATE_MACHINE_H_ */
