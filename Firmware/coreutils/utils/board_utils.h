/*
 * board_utils.h
 *
 *  Created on: Jan 30, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_COREUTILS_UTILS_BOARD_UTILS_H_
#define INC_COREUTILS_UTILS_BOARD_UTILS_H_



#endif /* INC_COREUTILS_UTILS_BOARD_UTILS_H_ */
/*
 * board_utils.h
 *
 *  Created on: Nov 20, 2024
 *      Author: zande
 */

#ifndef INC_BOARD_UTILS_H_
#define INC_BOARD_UTILS_H_

#include "stm32f0xx_hal.h"
#include "main.h"

void STARTUP() {

}


void STATUS_IND_On() {
	HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_SET);
}

void STATUS_IND_Off() {
	HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_RESET);
}

void STATUS_IND_Toggle() {
	HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
}

void WARN_IND_On() {
	HAL_GPIO_WritePin(WARN_IND_GPIO_Port, WARN_IND_Pin, GPIO_PIN_SET);
}

void WARN_IND_Off() {
	HAL_GPIO_WritePin(WARN_IND_GPIO_Port, WARN_IND_Pin, GPIO_PIN_RESET);
}

void WARN_IND_Toggle() {
	HAL_GPIO_TogglePin(WARN_IND_GPIO_Port, WARN_IND_Pin);
}

// indicates a serious error, that can be ignored if you know what youre doing
//void NON_CRITICAL_ERROR_On() {
//
//}
//
//// indicates a critical error, that cannot be ignored. Board function compromised
void CRITIAL_ERROR_GENERIC_On() {
	while (1) {
		HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_SET);
		HAL_Delay(125);
		HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_SET);
		HAL_Delay(125);
	}
}
//
//void CRITIAL_ERROR_SPECIFIC_On(uint32_t delay) {
//	while (1) {
//		HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_SET);
//		HAL_Delay(delay);
//		HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_SET);
//		HAL_Delay(delay);
//	}
//}


void GET_BOARD_UUID(uint32_t* uid) {
	uid[0] = HAL_GetUIDw0();
	uid[1] = HAL_GetUIDw1();
	uid[2] = HAL_GetUIDw2();
}

bool COMPARE_UID(uint32_t* board_uid, uint32_t* compare_uid) {
	// 3 uint32_t compare
	return memcmp(board_uid, compare_uid, sizeof(uint32_t) * 3) == 0;
}

#endif /* INC_BOARD_UTILS_H_ */
