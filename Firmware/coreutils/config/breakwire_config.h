/*
 * breakwire_config.h
 *
 *  Created on: Apr 10, 2025
 *      Author: armandozepeda
 */



#ifndef INC_BREAKWIRE_CONFIG_H_
#define INC_BREAKWIRE_CONFIG_H_

// LED flash timing (ms)
#define LED_FLASH_TIME_MS       500

// Delay before marking the breakwire as "open", if required.
#define BREAKWIRE_OPEN_DELAY_MS 4000

// Definitions for the breakwire pin and LED port/pin can also be placed here:
#define BRK_CONT_GPIO_Port      GPIOA   // Or whichever port is used
#define BRK_CONT_Pin            GPIO_PIN_2  // Replace X with the correct pin number // PyroCont1. also from pin16 to pin 2






#define BRK_CONT_LED_GPIO_Port  GPIOC   // Adjust as required      (not using at the moments)
#define BRK_CONT_LED_Pin        GPIO_PIN_13 // Replace Y with the LED pin // already have the status indicator (not using at the moment)

#endif /* INC_BREAKWIRE_CONFIG_H_ */
