// -----------------------------------------------------------------------------
// xbee_platform_uart.c – STM32F4 (UART6) port layer for Digi ANSI‐C XBee library
// -----------------------------------------------------------------------------
//  * Provides the mandatory xbee_ser_*() API so upper layers can talk to an
//    XBee radio in API mode.
//  * Uses a simple 1024‑byte IRQ‑driven ring‑buffer for RX; TX is blocking.
//  * Captures **every** byte that leaves the UART in a circular debug buffer so
//    you can examine complete frames in the debugger after the fact.
//  * Hardware: STM32F405 + XBee on USART6 @ 3.3 V.
//
// 2025‑05‑09  Brandon Marcus  <brandonmarcus@example.com>
// -----------------------------------------------------------------------------

#include <errno.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "xbee/platform.h"        // Digi macros & typedefs
#include "xbee/serial.h"          // function prototypes we must implement
#include "xbee/device.h"

// -----------------------------------------------------------------------------
// 1.  Platform‑specific serial descriptor
// -----------------------------------------------------------------------------
extern UART_HandleTypeDef huart6;            // provided by CubeMX‐generated usart.c
static xbee_serial_t xbee_port = { .huart = &huart6, .baudrate = 9600 };

// -----------------------------------------------------------------------------
// 1a.  **TX DEBUG RING** – keeps last N bytes ever written via xbee_ser_write()
// -----------------------------------------------------------------------------
#ifndef TXDBG_SZ
#   define TXDBG_SZ  1024u          // must be power‑of‑two for easy wrapping
#endif
static uint8_t  txdbg[TXDBG_SZ];     // circular buffer
static volatile uint16_t txdbg_head = 0;  // next byte will be written here

/**
 * Return a *snapshot* pointer into the TX debug ring.  Use like this from the
 * debugger (or application) to dump the last k bytes:
 *    uint16_t hdr = xbee_txdbg_head();
 *    for (int i = 0; i < k; ++i) { byte = txdbg[(hdr - k + i) & (TXDBG_SZ-1)]; }
 */
uint16_t xbee_txdbg_head(void) { return txdbg_head; }
const uint8_t *xbee_txdbg_buffer(void) { return txdbg; }

// -----------------------------------------------------------------------------
// 2.  Lightweight RX ring buffer (power‑of‑two size) and ISR hook
// -----------------------------------------------------------------------------
#define RX_BUF_SZ  1024u
static uint8_t        rxbuf[RX_BUF_SZ];
static volatile uint16_t rx_head = 0, rx_tail = 0;   // head==tail → empty

void xbee_uart_isr(void)
{
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE)) {
        rxbuf[rx_head++] = (uint8_t)(huart6.Instance->DR & 0xFF);
        rx_head &= (RX_BUF_SZ - 1);                  // wrap
    }
}

static inline uint16_t rx_used(void)
{
    return (rx_head - rx_tail) & (RX_BUF_SZ - 1);
}

// -----------------------------------------------------------------------------
// 3.  Mandatory helper – validate pointer passed by upper layers
// -----------------------------------------------------------------------------
bool_t xbee_ser_invalid(xbee_serial_t *s)
{
    return (s == NULL) || (s->huart == NULL);
}

// -----------------------------------------------------------------------------
// 4.  Raw read / write primitives
// -----------------------------------------------------------------------------
int xbee_ser_write(xbee_serial_t *s, const void FAR *buf, int len)
{
    if (xbee_ser_invalid(s) || !buf || len < 0) {
        return -EINVAL;
    }

    // 4a.  **copy into debug ring *before* sending** --------------------------
    const uint8_t *p = (const uint8_t *)buf;
    for (int i = 0; i < len; ++i) {
        txdbg[txdbg_head++] = p[i];
        txdbg_head &= (TXDBG_SZ - 1);
    }

    // 4b.  blocking transmit via STM32 HAL -----------------------------------
    HAL_StatusTypeDef rc = HAL_UART_Transmit(s->huart, (uint8_t *)buf,
                                             (uint16_t)len, HAL_MAX_DELAY);
    return (rc == HAL_OK) ? len : -EIO;
}

int xbee_ser_read(xbee_serial_t *s, void FAR *buf, int len)
{
    if (xbee_ser_invalid(s) || !buf || len <= 0) {
        return -EINVAL;
    }

    int cnt = 0;
    while (cnt < len && rx_tail != rx_head) {
        uint8_t b = rxbuf[rx_tail++];
        rx_tail &= (RX_BUF_SZ - 1);

        // handle API mode byte‑escaping (0x7D 0xXX) ---------------------------
        if (b == 0x7D) {
            if (rx_tail == rx_head) {
                // escape without following byte – leave it for next pass
                rx_tail = (rx_tail - 1) & (RX_BUF_SZ - 1);
                break;
            }
            b = rxbuf[rx_tail++] ^ 0x20;
            rx_tail &= (RX_BUF_SZ - 1);
        }
        ((uint8_t *)buf)[cnt++] = b;
    }
    return cnt;                 // 0 ⇒ no data available
}

// Convenience wrappers --------------------------------------------------------
int xbee_ser_putchar(xbee_serial_t *s, uint8_t ch)
{ return xbee_ser_write(s, &ch, 1) == 1 ? 0 : -ENOSPC; }

int xbee_ser_getchar(xbee_serial_t *s)
{
    uint8_t ch;
    return xbee_ser_read(s, &ch, 1) == 1 ? ch : -ENODATA;
}

// -----------------------------------------------------------------------------
// 5.  Buffer status helpers
// -----------------------------------------------------------------------------
int xbee_ser_tx_flush(xbee_serial_t *s)
{
    if (xbee_ser_invalid(s))  return -EINVAL;
    while (__HAL_UART_GET_FLAG(s->huart, UART_FLAG_TC) == RESET) ;
    return 0;
}

int xbee_ser_tx_free (xbee_serial_t *s) { (void)s; return 128; }
int xbee_ser_tx_used (xbee_serial_t *s) { (void)s; return 0;   }
int xbee_ser_rx_used (xbee_serial_t *s) { (void)s; return rx_used(); }
int xbee_ser_rx_free (xbee_serial_t *s) { (void)s; return RX_BUF_SZ - 1 - rx_used(); }
int xbee_ser_rx_flush(xbee_serial_t *s) { (void)s; rx_head = rx_tail = 0; return 0; }

// -----------------------------------------------------------------------------
// 6.  Port / baud / control‑line management
// -----------------------------------------------------------------------------
const char *xbee_ser_portname(xbee_serial_t *s) { (void)s; return "USART6"; }

int xbee_ser_open(xbee_serial_t *s, uint32_t baud)
{
    if (xbee_ser_invalid(s)) return -EINVAL;
    s->baudrate = baud;

    s->huart->Init.BaudRate = baud;
    HAL_StatusTypeDef h = HAL_UART_Init(s->huart);
    if (h != HAL_OK && h != HAL_BUSY) return -EIO;   // BUSY == already init

    __HAL_UART_ENABLE_IT(s->huart, UART_IT_RXNE);    // enable RX IRQ
    return 0;
}

int xbee_ser_baudrate(xbee_serial_t *s, uint32_t baud) { return xbee_ser_open(s, baud); }
int xbee_ser_close   (xbee_serial_t *s) { return xbee_ser_invalid(s) ? -EINVAL : (HAL_UART_DeInit(s->huart)==HAL_OK?0:-EIO); }

// NOPs / not used -------------------------------------------------------------
int xbee_ser_break      (xbee_serial_t *s, bool_t en) { (void)s; (void)en; return 0; }
int xbee_ser_flowcontrol(xbee_serial_t *s, bool_t en) { (void)s; (void)en; return 0; }
int xbee_ser_set_rts    (xbee_serial_t *s, bool_t a ) { (void)s; (void)a ; return 0; }
int xbee_ser_get_cts    (xbee_serial_t *s)            { (void)s; return 1; }
static int always_awake (xbee_dev_t *xbee)            { (void)xbee; return 1; }

// -----------------------------------------------------------------------------
// 7.  Convenience init wrapper the app can call once at boot
// -----------------------------------------------------------------------------
void xbee_platform_init(void)
{ xbee_ser_open(&xbee_port, 9600); }

uint32_t xbee_millisecond_timer(void) { return HAL_GetTick(); }
uint32_t xbee_seconds_timer    (void) { return HAL_GetTick() / 1000u; }

xbee_serial_t *xbee_platform_serial(void) { return &xbee_port; }
