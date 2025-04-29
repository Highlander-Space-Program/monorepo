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

#if defined(STM32F0)
    #include "stm32f0xx_hal.h"
#elif defined(STM32F405) || defined(STM32F4)
    #include "stm32f4xx_hal.h"
#else
    #error "STM32 family not defined or not supported!"
#endif

#include "main.h"
#include <stdbool.h>

#define MAX_FLASH_TIME 3000

static uint32_t flash_timer = 0;
static uint32_t previous_flash = 0;

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

bool Tick_SIGNAL(bool flash_signal_cmd) {
	if (flash_timer == 0) {
		flash_timer = HAL_GetTick();
	}

	uint32_t current_time = HAL_GetTick();
	if (current_time - previous_flash > 250) {
		previous_flash = current_time;
		STATUS_IND_Toggle();
	}

	if (current_time - flash_timer > MAX_FLASH_TIME) {
		flash_timer = 0;
		flash_signal_cmd = 0;
		STATUS_IND_Off();
	}

	return flash_signal_cmd;
}

//void WARN_IND_On() {
//	HAL_GPIO_WritePin(WARN_IND_GPIO_Port, WARN_IND_Pin, GPIO_PIN_SET);
//}
//
//void WARN_IND_Off() {
//	HAL_GPIO_WritePin(WARN_IND_GPIO_Port, WARN_IND_Pin, GPIO_PIN_RESET);
//}
//
//void WARN_IND_Toggle() {
//	HAL_GPIO_TogglePin(WARN_IND_GPIO_Port, WARN_IND_Pin);
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


void GET_BOARD_UUID(uint32_t* board_uid) {
	board_uid[0] = HAL_GetUIDw0();
	board_uid[1] = HAL_GetUIDw1();
	board_uid[2] = HAL_GetUIDw2();
}

bool COMPARE_UID(uint32_t* board_uid, uint32_t* compare_uid) {
	// 3 uint32_t compare
	return memcmp(board_uid, compare_uid, sizeof(uint32_t) * 3) == 0;
}

#endif /* INC_BOARD_UTILS_H_ */
