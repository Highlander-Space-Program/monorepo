/*
 * igniter.h
 *
 *  Created on: Apr 16, 2025
 *      Author: emmanuelche
 */

#ifndef INC_IGNITER_H_
#define INC_IGNITER_H_

#include "config/igniter_config.h"
#include "stm32f4xx_hal.h"
#include "utils/radio_utils.h"

static inline void Tick_Igniter(uint8_t cmd, uint8_t* ackPtr) {

    switch (igniterState) {

      case IGNITER_INIT:

        igniterState = IGNITER_DEACTIVATED;
        break;

      case IGNITER_DEACTIVATED:

        if ((cmd == ACTIVATE_IGNITER) && (!isCloseAll && !isAborted)) {
          igniterState = IGNITER_ACTIVATED;

        } else {

          igniterState = IGNITER_DEACTIVATED;
        }
        break;

      case IGNITER_ACTIVATED:

        if (((cmd == DEACTIVATE_IGNITER) && !isStarted) || (cmd == ABORT)) {
          igniterState = IGNITER_DEACTIVATED;
        } else {
          igniterState = IGNITER_ACTIVATED;
        }
        break;

      default:
        break;
    }

    switch (igniterState) {
      case IGNITER_INIT:
        HAL_GPIO_WritePin(IGNITER_GPIO_Port, IGNITER_Pin, GPIO_PIN_RESET);
        break;

      case IGNITER_DEACTIVATED:
	    Update_Ack(ackPtr, 0, 1);
        HAL_GPIO_WritePin(IGNITER_GPIO_Port, IGNITER_Pin, GPIO_PIN_RESET);
        break;

      case IGNITER_ACTIVATED:
	    Update_Ack(ackPtr, 0, 0);
        HAL_GPIO_WritePin(IGNITER_GPIO_Port, IGNITER_Pin, GPIO_PIN_SET);
        break;

      default:
        break;
    }
}

#endif /* INC_IGNITER_H_ */
