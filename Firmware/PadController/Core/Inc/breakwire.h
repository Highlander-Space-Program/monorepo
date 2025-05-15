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

void send_breakwire_status_can(bool is_system_armed) {
    BreakwireStatusByte_t status_val;
    GPIO_PinState physical_state = Check_Breakwire(); // From breakwire.h

    if (physical_state == GPIO_PIN_RESET) { // Continuity = Connected
        if (is_system_armed) {
            status_val = BREAKWIRE_CONNECTED_ARMED;
        } else {
            status_val = BREAKWIRE_CONNECTED_NOT_ARMED;
        }
    } else { // GPIO_PIN_SET = No Continuity = Disconnected
        if (is_system_armed) {
            status_val = BREAKWIRE_DISCONNECTED_ARMED;
        } else {
            // If the system is not armed and the wire is disconnected,
            // it's simply disconnected.
            status_val = BREAKWIRE_DISCONNECTED;
        }
    }

    uint8_t status_byte = (uint8_t)status_val;
    send_pad_controller_status_can(MSG_TYPE_BREAKWIRE_STATUS, status_byte);
}

void send_auto_mode_status_can(bool is_on) {
    uint8_t status_byte = is_on ? 1 : 0;
    send_pad_controller_status_can(MSG_TYPE_AUTO_MODE_STATUS, status_byte);
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

