/*
 * board_config.h
 *
 *  Created on: Apr 2, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_CONFIG_BOARD_CONFIG_H_
#define INC_COREUTILS_CONFIG_BOARD_CONFIG_H_

typedef struct {
	uint32_t uid[3];
	uint8_t short_id;
	char* name;
	char* pnid;
} BoardConfig;

#endif /* INC_COREUTILS_CONFIG_BOARD_CONFIG_H_ */
