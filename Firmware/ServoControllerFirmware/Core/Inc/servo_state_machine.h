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
#include "config/servo_config.h"
#include "config/config.h"

#define HSP_SERVO_MIN_PULSE_WIDTH 500
#define HSP_SERVO_MAX_PULSE_WIDTH 2500
#define HSP_SERVO_PWM_PERIOD 20000
#define HSP_SERVO_WAIT_TIME 3000 // in milliseconds

typedef struct  {
  ServoConfig* servo_config;
  SERVO_STATE state;
  uint32_t last_tick_update;
  const TIM_HandleTypeDef *timer;
  volatile uint32_t *ccr;
} Servo;

//Function Prototypes
void Tick_SERVO (uint8_t cmd, Servo* servo);
Servo* construct_servo (const uint32_t can_id, const TIM_HandleTypeDef *timer);
void send_servo_status_can(Servo* servo, uint8_t short_board_id, CAN_HandleTypeDef *hcan);

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

void send_servo_status_can(Servo* servo, uint8_t short_board_id, CAN_HandleTypeDef* hcan) {
    uint8_t status_byte = (uint8_t) servo->state; // 1 = armed, 0 = not armed
    uint8_t sender_id = short_board_id;
    uint8_t target_board_id = SENDER_PAD_CONTROLLER;
    uint8_t msg_type_val = MSG_TYPE_SERVO; // Using existing general servo type
    uint8_t instance_val = 1;

    uint32_t base_ext_id_32bit_shifted = build_can_extended_id(sender_id, target_board_id, msg_type_val, instance_val);
    uint32_t base_ext_id_29bit = base_ext_id_32bit_shifted >> 3;
    uint32_t ack_ext_id_29bit = base_ext_id_29bit | CAN_ID_ACK_FLAG_29BIT;
    uint32_t final_ext_id_32bit_shifted_for_send = ack_ext_id_29bit << 3;

    HAL_StatusTypeDef status = send_can_msg(final_ext_id_32bit_shifted_for_send, &status_byte, 1, hcan);

    if (status != HAL_OK) {
        // Handle CAN send error
    }
}

Servo* construct_servo (const uint32_t can_id, const TIM_HandleTypeDef *timer) {
    ServoConfig *sc = GET_SERVO_CONFIG(can_id);
    if (!sc) {
        return NULL;
    }

    Servo *servo = malloc(sizeof(Servo));
    if (!servo) {
        return NULL;
    }

    servo->servo_config     = sc;
    servo->state            = OFF_CLOSED;
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
			else if (servo->state == ON_OPEN) {
//				servo->state = ON_OPEN;
//				servo->last_tick_update = HAL_GetTick();
			}
			else if (servo->state == OFF_OPEN) {/*Not Used*/}
			break;
		case CLOSE_SERVO:
			if (servo->state == ON_CLOSED) {/*Not Used*/}
			else if (servo->state == OFF_CLOSED) {
//				servo->state = ON_CLOSED;
//				servo->last_tick_update = HAL_GetTick();
			}
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
			Actuate_Servo (servo->servo_config->closed_deg, servo->timer);
			break;
		case OFF_CLOSED:
			Turn_Off_Servo ();
			break;
		case ON_OPEN:
			Turn_On_Servo ();
			Actuate_Servo (servo->servo_config->open_deg, servo->timer);
			break;
		case OFF_OPEN:
			Turn_Off_Servo ();
			break;
	}
}

#endif /* INC_SERVO_STATE_MACHINE_H_ */
