/*
 * servo_utils.h
 *
 *  Created on: Apr 29, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_COREUTILS_UTILS_SERVO_UTILS_H_
#define INC_COREUTILS_UTILS_SERVO_UTILS_H_

#include "utils/can_utils.h"

extern CAN_HandleTypeDef hcan1;

// 0 is open 1 is close for servo_cmd. see SERVO_CMD in servo_state_machine.h
void ACTUATE_SERVO(uint32_t ext_id, uint8_t servo_cmd) {
	data [0] = servo_cmd;
	HAL_StatusTypeDef status = send_can_msg(ext_id, data, LENGTH, &hcan1);
//	if (status != HAL_OK) {
//	    HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Indicate error
//	}
}

#endif /* INC_COREUTILS_UTILS_SERVO_UTILS_H_ */
