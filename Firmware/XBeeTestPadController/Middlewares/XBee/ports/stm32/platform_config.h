#ifndef __XBEE_PLATFORM_STM32
#define __XBEE_PLATFORM_STM32

/*— make sure every XBEE_PACKED struct is truly packed to 1-byte —*/
#if defined ( __GNUC__ )
  #define PACKED_STRUCT          struct __attribute__((packed))
#elif defined ( __ICCARM__ )
  /* IAR */
  #define PACKED_STRUCT          __packed struct
#elif defined ( __CC_ARM )
  /* Keil ARMCC */
  #pragma pack(1)
  #define PACKED_STRUCT          struct
#else
  #error "Please define PACKED_STRUCT for your compiler"
#endif

#define XBEE_PACKED(name, decl)  PACKED_STRUCT name decl
#define XBEE_DEVICE_ENABLE_ATMODE

/*  — the rest of your STM32 port_config: —  */
#include "stm32f4xx_hal.h"

/* platform-wide helpers */
#define XBEE_MS_TIMER            HAL_GetTick
#define XBEE_PLATFORM_PRINTF     printf
#define XBEE_MS_TIMER_RESOLUTION 1
#define ZCL_TIME_EPOCH_DELTA     1
#define MBEDTLS_NO_PLATFORM_ENTROPY

#define FAR
#define _f_memset                memset
#define _f_memcpy                memcpy

//void xbee_platform_init(void);
//#define XBEE_PLATFORM_INIT()    xbee_platform_init()

uint16_t _xbee_get_unaligned16(const void FAR *p);
uint32_t _xbee_get_unaligned32(const void FAR *p);
void     _xbee_set_unaligned16(void FAR *p, uint16_t v);
void     _xbee_set_unaligned32(void FAR *p, uint32_t v);

#define xbee_get_unaligned16(p)    _xbee_get_unaligned16(p)
#define xbee_get_unaligned32(p)    _xbee_get_unaligned32(p)
#define xbee_set_unaligned16(p,v)  _xbee_set_unaligned16(p,v)
#define xbee_set_unaligned32(p,v)  _xbee_set_unaligned32(p,v)

typedef unsigned char bool_t;

/* your serial struct */
typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t            baudrate;
} xbee_serial_t;


#endif  /* __XBEE_PLATFORM_STM32 */
