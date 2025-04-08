/*
 * pt_config.h
 *
 *  Created on: Mar 31, 2025
 *      Author: zande
 */

#ifndef INC_COREUTILS_PT_CONFIG_H_
#define INC_COREUTILS_PT_CONFIG_H_

struct Pt;

typedef struct {
    uint32_t board_uid[3];
	uint32_t can_id;
    char* pnid;
    uint16_t frequency;
    float gain;
    float offset;
    char port;
} PtConfig;

typedef enum {
	PRES_WAIT = 0,
	PRES_GET = 1
} PT_STATE;

typedef enum {
	FORCE_GET_PRES = 0,
	FORCE_RESET_PT_TIMER = 1
} PT_CMD;

#endif /* INC_COREUTILS_PT_CONFIG_H_ */
