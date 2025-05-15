/*
 * breakwire_config.h
 *
 *  Created on: Apr 10, 2025
 *      Author: armandozepeda
 */



#ifndef INC_BREAKWIRE_CONFIG_H_
#define INC_BREAKWIRE_CONFIG_H_

// LED flash timing (ms)
#define LED_FLASH_TIME_MS       1000

// Delay before marking the breakwire as "open", if required.
#define BREAKWIRE_OPEN_DELAY_MS 4000

typedef enum {
    BREAKWIRE_CONNECTED_NOT_ARMED = 0,
    BREAKWIRE_CONNECTED_ARMED     = 1,
    BREAKWIRE_DISCONNECTED        = 2, // Implies not armed by function, or could be DISCONNECTED_NOT_ARMED
    BREAKWIRE_DISCONNECTED_ARMED  = 3  // e.g. system armed, then wire breaks
} BreakwireStatusByte_t;

#endif /* INC_BREAKWIRE_CONFIG_H_ */
