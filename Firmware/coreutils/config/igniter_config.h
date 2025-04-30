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

//ACTIVATE_IGNITER   14
//DEACTIVATE_IGNITER 15
//ABORT              16

extern bool isCloseAll;
extern bool isAborted;
extern bool isStarted;

typedef enum {
    IGNITER_INIT,
    IGNITER_DEACTIVATED,
    IGNITER_ACTIVATED
} IgniterState_t;

extern IgniterState_t igniterState;

#endif /* INC_COREUTILS_CONFIG_IGNITER_CONFIG_H_ */
