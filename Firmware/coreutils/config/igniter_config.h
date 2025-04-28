/*
 * igniter_config.h
 *
 *  Created on: Apr 17, 2025
 *      Author: armandozepeda
 */

#ifndef INC_COREUTILS_CONFIG_IGNITER_CONFIG_H_
#define INC_COREUTILS_CONFIG_IGNITER_CONFIG_H_


#include <stdbool.h>
#include "main.h"
#define ACTIVATE_IGNITER   14
#define DEACTIVATE_IGNITER 15
#define ABORT              16
extern bool isCloseAll;
extern bool isAborted;
extern bool isStarted;

typedef enum {
    IGNITER_INIT,
    IGNITER_DEACTIVATED,
    IGNITER_ACTIVATED
} IgniterState_t;

extern IgniterState_t igniterState;

#define IGNITER_GPIO_Port  PYRO_SW1_GPIO_Port
#define IGNITER_Pin        PYRO_SW1_Pin
//#define IGNITER_GPIO_Port  PYRO_SW0_GPIO_Port
//#define IGNITER_Pin        PYRO_SW0_Pin
//#define IGNITER_GPIO_Port  PYRO_SW2_GPIO_Port
//#define IGNITER_Pin        PYRO_SW2_Pin
#endif /* INC_COREUTILS_CONFIG_IGNITER_CONFIG_H_ */
