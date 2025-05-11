/*
 * xbee_platform_stm32.c
 *
 * Platform-specific helper functions for the Digi XBee C library on STM32.
 * Contains functions for unaligned memory access.
 *
 * Created on: May 9, 2025
 * Author: brandonmarcus
 *
 * No changes made to this file based on current request, as user confirmed
 * unaligned access functions are correct as is.
 */

#include <stdint.h>
#include <string.h>       // Not strictly needed for this file's content but often included
#include "xbee/platform.h" // For FAR keyword (though FAR is empty in platform_config.h)

/*
 * @brief Reads a 16-bit value from an unaligned memory location (little-endian in memory).
 * The XBee library uses these functions to abstract platform differences in handling
 * data that might not be aligned on natural processor boundaries.
 * This implementation assumes the 16-bit value at address p is stored little-endian.
 * p[0] = LSB, p[1] = MSB.
 * @param p Pointer to the unaligned memory location.
 * @return The 16-bit value.
 */
uint16_t _xbee_get_unaligned16(const void FAR *p)
{
	const uint8_t FAR *b = (const uint8_t FAR *)p;
	// Little-endian: b[0] is LSB, b[1] is MSB
	return ((uint16_t)b[1] << 8) | b[0];
//  Big-endian version (if data at *p was MSB first):
//  return ((uint16_t)b[0] << 8) | b[1];
}

/*
 * @brief Reads a 32-bit value from an unaligned memory location (little-endian in memory).
 * This implementation assumes the 32-bit value at address p is stored little-endian.
 * p[0] = LSB, p[1], p[2], p[3] = MSB.
 * @param p Pointer to the unaligned memory location.
 * @return The 32-bit value.
 */
uint32_t _xbee_get_unaligned32(const void FAR *p)
{
    const uint8_t FAR *b = (const uint8_t FAR *)p;
    // Little-endian: b[0] is LSB, b[3] is MSB
    return ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) |
           ((uint32_t)b[1] <<  8) |  (uint32_t)b[0];
//  Big-endian version (if data at *p was MSB first):
//  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
//         ((uint32_t)b[2] <<  8) |  (uint32_t)b[3];
}

/*
 * @brief Writes a 16-bit value to an unaligned memory location (little-endian in memory).
 * Stores the host-endian value 'v' into memory at 'p' in little-endian format.
 * @param p Pointer to the unaligned memory location.
 * @param v The 16-bit value to store.
 */
void _xbee_set_unaligned16(void FAR *p, uint16_t v)
{
    uint8_t FAR *b = (uint8_t FAR *)p;
    // Store little-endian: LSB at b[0], MSB at b[1]
    b[0] = (uint8_t)(v      );
    b[1] = (uint8_t)(v >> 8 );
}

/*
 * @brief Writes a 32-bit value to an unaligned memory location (little-endian in memory).
 * Stores the host-endian value 'v' into memory at 'p' in little-endian format.
 * @param p Pointer to the unaligned memory location.
 * @param v The 32-bit value to store.
 */
void _xbee_set_unaligned32(void FAR *p, uint32_t v)
{
    uint8_t FAR *b = (uint8_t FAR *)p;
    // Store little-endian: LSB at b[0], ..., MSB at b[3]
    b[0] = (uint8_t)(v      );
    b[1] = (uint8_t)(v >>  8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}
