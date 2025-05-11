/*
 * platform_config.h
 *
 * Platform-specific configurations for the Digi XBee C library on STM32.
 * No changes made to this file based on current request.
 */
#ifndef __XBEE_PLATFORM_STM32 // Guard to match the original file's include guard name
#define __XBEE_PLATFORM_STM32

/* Ensure every XBEE_PACKED struct is truly packed to 1-byte alignment.
   This is crucial for correct parsing of XBee API frames. */
#if defined ( __GNUC__ )
  #define PACKED_STRUCT          struct __attribute__((packed))
#elif defined ( __ICCARM__ )
  /* IAR C/C++ Compiler */
  #define PACKED_STRUCT          __packed struct
#elif defined ( __CC_ARM )
  /* Keil ARM Compiler */
  #pragma pack(1) // Set packing to 1 byte for structs that follow
  #define PACKED_STRUCT          struct
  // Note: #pragma pack(pop) might be needed at the end of headers using this,
  // or ensure it's reset if it affects other system headers.
#else
  #error "Please define PACKED_STRUCT for your compiler to ensure 1-byte structure packing."
#endif

// Macro used by Digi XBee library to declare packed structures.
#define XBEE_PACKED(name, decl)  PACKED_STRUCT name decl

// Enable AT command mode features in the XBee library.
#define XBEE_DEVICE_ENABLE_ATMODE

#ifndef XBEE_ATCMD_ERROR
#define XBEE_ATCMD_ERROR -1
#endif

#ifndef WPAN_NET_ADDR_BROADCAST
#define WPAN_NET_ADDR_BROADCAST ((uint16_t)0xFFFF)
#endif

/* --- STM32 Specific Platform Configuration --- */
#include "stm32f4xx_hal.h" // STM32 HAL library include

/* Platform-wide helper macros required/used by the XBee library */
#define XBEE_MS_TIMER            HAL_GetTick     // Millisecond timer function
#define XBEE_PLATFORM_PRINTF     printf          // Platform's printf for debugging (ensure stdio is linked)
#define XBEE_MS_TIMER_RESOLUTION 1               // Resolution of XBEE_MS_TIMER in milliseconds (1 for HAL_GetTick)

// These are often related to Zigbee Cluster Library (ZCL) or other specific features.
// If not using ZCL, they might not be strictly necessary but are common in example configs.
#define ZCL_TIME_EPOCH_DELTA     0 // Or an appropriate delta if dealing with ZCL time. Original was 1.
                                   // Set to 0 if not using ZCL time features or if Unix epoch (1/1/1970) is the base.
                                   // Digi's typical epoch is 1/1/2000. Delta is seconds between 1/1/1970 and 1/1/2000.
                                   // (946684800 seconds)

// If using mbedTLS and not providing platform-specific entropy source via mbedtls_platform_entropy_poll.
#define MBEDTLS_NO_PLATFORM_ENTROPY

// FAR keyword, typically used for memory model specifics on older compilers/architectures.
// For ARM Cortex-M, it's usually not needed and defined as empty.
#define FAR

// Memory manipulation function macros, mapping to standard library functions.
#define _f_memset                memset
#define _f_memcpy                memcpy

/* --- XBee Library Platform API Requirements --- */

// Forward declaration for the UART ISR if it's defined in xbee_platform_uart.c
// and needs to be known by other parts of the XBee platform layer (though typically not directly).
void xbee_uart_isr(void); // Defined in xbee_platform_uart.c

// Unaligned memory access functions (prototypes).
// These are implemented in xbee_platform_stm32.c.
// The XBee library will call these via macros like xbee_get_unaligned16.
uint16_t _xbee_get_unaligned16(const void FAR *p);
uint32_t _xbee_get_unaligned32(const void FAR *p);
void     _xbee_set_unaligned16(void FAR *p, uint16_t v);
void     _xbee_set_unaligned32(void FAR *p, uint32_t v);

// Macros that the XBee library uses to call the platform-specific unaligned access functions.
#define xbee_get_unaligned16(p)    _xbee_get_unaligned16(p)
#define xbee_get_unaligned32(p)    _xbee_get_unaligned32(p)
#define xbee_set_unaligned16(p,v)  _xbee_set_unaligned16(p,v)
#define xbee_set_unaligned32(p,v)  _xbee_set_unaligned32(p,v)

typedef unsigned char bool_t;

/* Platform-specific serial port context structure.
   The XBee library uses a pointer to this (xbee_serial_t*) to manage serial operations. */
typedef struct {
    UART_HandleTypeDef *huart;    // Pointer to the STM32 HAL UART handle (e.g., &huart6)
    uint32_t            baudrate; // Current baud rate of the UART
    // Add other platform-specific serial port state if needed
} xbee_serial_t;

// Constants for XBee API frame escaping (used in xbee_platform_uart.c)
#define XBEE_API_ESCAPE_CHAR 0x7D
#define XBEE_API_ESCAPE_MASK 0x20


#endif  /* __XBEE_PLATFORM_STM32 */
