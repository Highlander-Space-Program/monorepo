// -----------------------------------------------------------------------------
// xbee_platform_uart.c – STM32F4 (UART6) port layer for Digi ANSI‐C XBee library
// File content provided under xbee_platform_uart.h, assumed to be implementation.
// No changes made to this file based on current request.
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
 * uint16_t hdr = xbee_txdbg_head();
 * for (int i = 0; i < k; ++i) { byte = txdbg[(hdr - k + i) & (TXDBG_SZ-1)]; }
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
    // Check if the RXNE flag is set (Receive Data Register Not Empty)
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE)) {
        // Read the data from the Data Register
        rxbuf[rx_head] = (uint8_t)(huart6.Instance->DR & 0xFF);
        rx_head = (rx_head + 1) & (RX_BUF_SZ - 1); // Increment head and wrap around

        // Optional: Check for overrun error if necessary, though HAL might handle it
        // if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_ORE)) {
        //    __HAL_UART_CLEAR_OREFLAG(&huart6); // Clear overrun flag
        //    // Handle overrun error (e.g., log it, increment a counter)
        // }
    }
}


static inline uint16_t rx_used(void)
{
    return (rx_head - rx_tail + RX_BUF_SZ) & (RX_BUF_SZ - 1);
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
        txdbg[txdbg_head] = p[i];
        txdbg_head = (txdbg_head + 1) & (TXDBG_SZ - 1); // Increment and wrap
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
    uint8_t *out_buf = (uint8_t *)buf;

    while (cnt < len && rx_tail != rx_head) { // While buffer has data and we need more bytes
        uint8_t b = rxbuf[rx_tail];
        rx_tail = (rx_tail + 1) & (RX_BUF_SZ - 1); // Consume byte, advance tail

        // handle API mode byte‑escaping (0x7D followed by byte to be XORed with 0x20)
        if (b == XBEE_API_ESCAPE_CHAR) { // XBEE_API_ESCAPE_CHAR is 0x7D
            if (rx_tail == rx_head) { // Escaped byte, but no following byte yet
                // Put the escape char back by retreating tail (it will be read again next call)
                rx_tail = (rx_tail - 1 + RX_BUF_SZ) & (RX_BUF_SZ - 1);
                break; // Exit, wait for the next byte to arrive
            }
            // Valid escape sequence, get the next byte and unescape it
            b = rxbuf[rx_tail] ^ XBEE_API_ESCAPE_MASK; // XBEE_API_ESCAPE_MASK is 0x20
            rx_tail = (rx_tail + 1) & (RX_BUF_SZ - 1); // Consume the escaped byte
        }
        out_buf[cnt++] = b;
    }
    return cnt; // Number of bytes actually read (0 means no data available or only partial escape sequence)
}

// Convenience wrappers --------------------------------------------------------
int xbee_ser_putchar(xbee_serial_t *s, uint8_t ch)
{ return xbee_ser_write(s, &ch, 1) == 1 ? 0 : -ENOSPC; } // Or -EIO from write

int xbee_ser_getchar(xbee_serial_t *s)
{
    uint8_t ch;
    return xbee_ser_read(s, &ch, 1) == 1 ? ch : -ENODATA; // No data available
}

// -----------------------------------------------------------------------------
// 5.  Buffer status helpers
// -----------------------------------------------------------------------------
int xbee_ser_tx_flush(xbee_serial_t *s)
{
    if (xbee_ser_invalid(s))  return -EINVAL;
    // Wait for the Transmit Complete (TC) flag
    while (__HAL_UART_GET_FLAG(s->huart, UART_FLAG_TC) == RESET) {
        // Potentially add a timeout here to prevent indefinite blocking
    }
    return 0;
}

// These are often estimates or fixed values if precise buffer tracking isn't implemented for TX.
// For TX, the HAL_UART_Transmit is blocking, so "free" is effectively the max HAL buffer or unlimited from this perspective.
// The XBee library uses these to manage its internal queuing before calling xbee_ser_write.
int xbee_ser_tx_free (xbee_serial_t *s) { (void)s; return 256; } // Typical XBee frame + overhead, or larger
int xbee_ser_tx_used (xbee_serial_t *s) { (void)s; return 0;   } // Since TX is blocking, it's 0 after call returns

int xbee_ser_rx_used (xbee_serial_t *s) { (void)s; return rx_used(); }
int xbee_ser_rx_free (xbee_serial_t *s) { (void)s; return RX_BUF_SZ - 1 - rx_used(); } // -1 to distinguish full from empty
int xbee_ser_rx_flush(xbee_serial_t *s) { (void)s; rx_head = rx_tail = 0; return 0; }

// -----------------------------------------------------------------------------
// 6.  Port / baud / control‑line management
// -----------------------------------------------------------------------------
const char *xbee_ser_portname(xbee_serial_t *s) { (void)s; return "USART6"; }

int xbee_ser_open(xbee_serial_t *s, uint32_t baud)
{
    if (xbee_ser_invalid(s)) return -EINVAL;

    // If UART is already initialized and baud rate is the same, can potentially skip re-init.
    // However, re-initializing is safer to ensure correct state.
    // HAL_UART_DeInit(s->huart); // Optional: DeInit before Init if issues occur

    s->baudrate = baud;
    s->huart->Init.BaudRate = baud;
    // Other UART parameters (WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling)
    // should be pre-configured in huart6 (e.g., by CubeMX) and are assumed correct.
    // If they need to be changed dynamically, set them here before HAL_UART_Init.
    // s->huart->Init.WordLength = UART_WORDLENGTH_8B;
    // s->huart->Init.StopBits = UART_STOPBITS_1;
    // s->huart->Init.Parity = UART_PARITY_NONE;
    // s->huart->Init.Mode = UART_MODE_TX_RX;
    // s->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;


    HAL_StatusTypeDef h_status = HAL_UART_Init(s->huart);
    if (h_status != HAL_OK) {
        // If HAL_BUSY, it might mean it's already initialized and running.
        // Depending on strictness, this could be an error or acceptable.
        // For now, treating HAL_BUSY as not an error for re-opening.
        // if (h_status == HAL_BUSY && s->huart->gState != HAL_UART_STATE_RESET) {
        //    // UART is busy but not in reset state, assume it's already open and configured.
        // } else {
        //    return -EIO; // Other HAL_UART_Init error
        // }
        return -EIO; // Treat any non-OK as an error for simplicity.
    }

    __HAL_UART_ENABLE_IT(s->huart, UART_IT_RXNE);    // Enable RX Not Empty interrupt
    // NVIC configuration for USART6_IRQn should be done elsewhere (e.g., HAL_MspInit or main init)
    // HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(USART6_IRQn);

    return 0; // Success
}

int xbee_ser_baudrate(xbee_serial_t *s, uint32_t baud) {
    // This function should ideally only change the baud rate if the port is open.
    // The current xbee_ser_open re-initializes, so it effectively handles baud change.
    return xbee_ser_open(s, baud);
}

int xbee_ser_close(xbee_serial_t *s) {
    if (xbee_ser_invalid(s)) return -EINVAL;
    __HAL_UART_DISABLE_IT(s->huart, UART_IT_RXNE); // Disable RX interrupt
    return (HAL_UART_DeInit(s->huart) == HAL_OK) ? 0 : -EIO;
}

// NOPs / not used by all XBee modules or in all modes (especially API mode)
int xbee_ser_break      (xbee_serial_t *s, bool_t en) { (void)s; (void)en; return 0; } // Not typically used
int xbee_ser_flowcontrol(xbee_serial_t *s, bool_t en) { (void)s; (void)en; return 0; } // Assuming no HW flow control
int xbee_ser_set_rts    (xbee_serial_t *s, bool_t a ) { (void)s; (void)a ; return 0; } // Assuming no RTS
int xbee_ser_get_cts    (xbee_serial_t *s)            { (void)s; return 1; } // Assuming CTS is always asserted (ready)

// This function is for power management (sleep/wake); returning 1 means always awake.
// static int always_awake (xbee_dev_t *xbee)            { (void)xbee; return XBEE_AWAKE_ALWAYS; } // Or a suitable macro from XBee lib

// -----------------------------------------------------------------------------
// 7.  Convenience init wrapper the app can call once at boot
// -----------------------------------------------------------------------------
void xbee_platform_init(void) {
    // huart6 should be initialized by MX_USART6_UART_Init() called from main.c
    // This function then configures it for XBee use.
    // The default baud rate (e.g., 9600) is set in xbee_port static struct.
    // If a different initial baud is needed, pass it here.
    if (xbee_ser_open(&xbee_port, xbee_port.baudrate) != 0) {
        // Handle error, e.g., blink LED, log error
        // Error_Handler(); // Placeholder for your error handling
    }
}

// Timer functions required by the XBee library
uint32_t xbee_millisecond_timer(void) {
    return HAL_GetTick(); // STM32 HAL tick provides milliseconds
}

uint32_t xbee_seconds_timer(void) {
    return HAL_GetTick() / 1000u; // Convert milliseconds to seconds
}

// Function for the XBee library to get a pointer to the serial port structure
xbee_serial_t *xbee_platform_serial(void) {
    return &xbee_port;
}
