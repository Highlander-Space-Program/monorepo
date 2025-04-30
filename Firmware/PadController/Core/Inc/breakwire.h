/*
 * breakwire.h
 *
 *  Created on: Apr 10, 2025
 *      Author: armandozepeda
 */


#ifndef INC_BREAKWIRE_H_
#define INC_BREAKWIRE_H_
//#include "config/config.h"
#include "main.h"
#include "config/breakwire_config.h" // config is first for anything in core utils(file path)
#include "PC_state.h"

extern bool isAutoArmed;

//#include "stdbool.h"
// Functions

//GPIO_PinState Check_Breakwire(void);
//void Breakwire_LED_On(void);
//void Breakwire_LED_Off(void);
//void Tick_Breakwire_LED(void);

// Read the breakwire state.
static inline GPIO_PinState Check_Breakwire(void) {
    return HAL_GPIO_ReadPin(BRK_CONT_GPIO_Port, BRK_CONT_Pin);
}

// Turn the breakwire LED on.
static inline void Breakwire_LED_On(void) {
    HAL_GPIO_WritePin(BRK_CONT_LED_GPIO_Port, BRK_CONT_LED_Pin, GPIO_PIN_SET);
}

// Turn the breakwire LED off.
static inline void Breakwire_LED_Off(void) {
    HAL_GPIO_WritePin(BRK_CONT_LED_GPIO_Port, BRK_CONT_LED_Pin, GPIO_PIN_RESET);
}

// Update the breakwire LED based on the sensor state and auto-armed status.
static inline void Tick_Breakwire_LED(void) {
    // Read the state once to improve efficiency.

    GPIO_PinState state = Check_Breakwire();
    if (state == GPIO_PIN_SET) {
        // No continuity: turn LED off.
        Breakwire_LED_Off();
    } else if (state == GPIO_PIN_RESET) {
        // Continuity detected.
        if (isAutoArmed) {
            // If auto armed, turn LED on solid.
            Breakwire_LED_On();

        } else {
            // If not armed, flash the LED.
            uint64_t curr_time = HAL_GetTick() % LED_FLASH_TIME_MS;
            if (curr_time > (LED_FLASH_TIME_MS / 2)) {
                Breakwire_LED_On();
            } else {
                Breakwire_LED_Off();
            }
        }
    }
}

#endif /* INC_BREAKWIRE_H_ */

