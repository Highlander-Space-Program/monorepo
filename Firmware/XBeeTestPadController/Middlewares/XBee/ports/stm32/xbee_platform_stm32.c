/*
 * xbee_platform_stm32.c
 *
 *  Created on: May 9, 2025
 *      Author: brandonmarcus
 */

#include <stdint.h>
#include <string.h>
#include "xbee/platform.h"

/* return 16‑bit value stored MSB‑first at *p         */
uint16_t _xbee_get_unaligned16(const void FAR *p)
{
	const uint8_t FAR *b = (const uint8_t FAR *)p;
	return ((uint16_t)b[1] << 8) | b[0];
//    const uint8_t FAR *b = (const uint8_t FAR *)p;
//    return ((uint16_t)b[0] << 8) | b[1];
}

/* return 32‑bit value stored MSB‑first at *p         */
uint32_t _xbee_get_unaligned32(const void FAR *p)
{
    const uint8_t FAR *b = (const uint8_t FAR *)p;
    return ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) |
           ((uint32_t)b[1] <<  8) |  b[0];
//    const uint8_t FAR *b = (const uint8_t FAR *)p;
//    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
//           ((uint32_t)b[2] <<  8) |  b[3];
}

/* store 16‑bit value MSB‑first at *p                 */
void _xbee_set_unaligned16(void FAR *p, uint16_t v)
{
    uint8_t FAR *b = (uint8_t FAR *)p;
    b[1] = (uint8_t)(v >> 8);
    b[0] = (uint8_t)(v     );
}

/* store 32‑bit value MSB‑first at *p                 */
void _xbee_set_unaligned32(void FAR *p, uint32_t v)
{
    uint8_t FAR *b = (uint8_t FAR *)p;
    b[3] = (uint8_t)(v >> 24);
    b[2] = (uint8_t)(v >> 16);
    b[1] = (uint8_t)(v >>  8);
    b[0] = (uint8_t)(v      );
}


