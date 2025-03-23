/*
 * heater_config.h
 *
 *  Created on: Mar 11, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_HEATER_CONFIG_H_
#define INC_COREUTILS_CONFIG_HEATER_CONFIG_H_

struct Heater;

typedef struct {
	uint32_t board_uid[3];
	uint32_t can_id;
	char* pnid;
	int off_temp;
	int on_temp;
	uint16_t frequency;
} HeaterConfig;

//States of the heater
typedef enum {
  DIR_OFF = 0,
  DIR_ON = 1,
  DIR_AUTO = 2
} HEATER_DIRECTIVE;

//States of the heater
typedef enum {
  OFF = 0,
  ON = 1,
} HEATER_STATE;

//commands for the heater
typedef enum  {
  H_OFF = 0,
  H_ON = 1,
  H_AUTO = 2
} HEATER_CMD;

#endif /* INC_COREUTILS_CONFIG_HEATER_CONFIG_H_ */
