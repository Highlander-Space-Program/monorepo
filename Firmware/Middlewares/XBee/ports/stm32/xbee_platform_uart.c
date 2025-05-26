// xbee_platform_uart.c – STM32F4 (UART6) port layer for Digi ANSI‐C XBee library
// MODIFIED FOR NON-BLOCKING (INTERRUPT-DRIVEN) TX and RX using HAL

#include <errno.h>
#include <string.h>
#include "stm32f4xx_hal.h"      // Adjust if using a different STM32 series
#include "xbee/platform.h"      // Digi macros & typedefs
#include "xbee/serial.h"        // function prototypes we must implement
#include "xbee/device.h"        // For XBEE_API_ESCAPE_CHAR, XBEE_API_ESCAPE_MASK etc.
#include "stm32/xbee_platform_uart.h"

// For disabling/enabling interrupts (used for critical sections)
#include "stm32f4xx_hal_cortex.h" // Or your specific STM32 series HAL cortex include

// Configuration Defines
#define XBEE_UART_TX_FLUSH_TIMEOUT_MS 200 // Timeout for flush operation

// -----------------------------------------------------------------------------
// 1.  Platform‐specific serial descriptor
// -----------------------------------------------------------------------------
//extern UART_HandleTypeDef huart6; // Provided by CubeMX‐generated usart.c
// Ensure api_escape is part of your xbee_serial_t definition if you use the s->api_escape check.
// Example: typedef struct xbee_serial_s { UART_HandleTypeDef *huart; uint32_t baudrate; bool_t api_escape; } xbee_serial_t;
//static xbee_serial_t xbee_port = { .huart = &huart6, .baudrate = 38400 /*, .api_escape = TRUE */ };
//static xbee_serial_t xbee_port = { .huart = &huart6, .baudrate = 115200 /*, .api_escape = TRUE */ };
static xbee_serial_t xbee_port = { .huart = NULL, .baudrate = 0 };

// -----------------------------------------------------------------------------
// 1a.  TX DEBUG RING (original, kept for debugging frame contents)
// -----------------------------------------------------------------------------
#ifndef TXDBG_SZ
#   define TXDBG_SZ  1024u // Must be power‑of‑two for easy wrapping
#endif
static uint8_t  txdbg[TXDBG_SZ];
static volatile uint16_t txdbg_head = 0;
uint16_t xbee_txdbg_head(void) { return txdbg_head; }
const uint8_t *xbee_txdbg_buffer(void) { return txdbg; }

// -----------------------------------------------------------------------------
// 2.  RX Ring Buffer (Filled by HAL RX Complete Callback)
// -----------------------------------------------------------------------------
#ifndef RX_BUF_SZ
#define RX_BUF_SZ  1024u // Must be power‑of‑two
#endif
static uint8_t        rxbuf[RX_BUF_SZ];
static volatile uint16_t rx_head = 0; // Written by HAL RX callback path
static volatile uint16_t rx_tail = 0; // Read by xbee_ser_read

// Single byte buffer for HAL_UART_Receive_IT
static inline uint16_t rx_buffer_used(void)
{
    // This calculation is safe as long as head/tail are volatile and accessed carefully.
    // For more complex scenarios or different compilers, a critical section might be considered
    // if the read operation itself isn't atomic for these 16-bit values.
    // However, for typical 32-bit MCUs, direct volatile reads are usually fine.
    uint16_t current_head = rx_head;
    uint16_t current_tail = rx_tail;
    return (current_head - current_tail + RX_BUF_SZ) & (RX_BUF_SZ - 1);
}

// -----------------------------------------------------------------------------
// 3.  TX Ring Buffer and State for Non-Blocking Transmit
// -----------------------------------------------------------------------------
#ifndef XBEE_PLATFORM_TX_BUF_SZ
#define XBEE_PLATFORM_TX_BUF_SZ 1024u // Must be power-of-two
#endif
static uint8_t  g_xbee_platform_tx_buf[XBEE_PLATFORM_TX_BUF_SZ];
static volatile uint16_t g_tx_buf_head = 0; // Written by xbee_ser_write (producer)
static volatile uint16_t g_tx_buf_tail = 0; // Read by UART TX completion (consumer)
static volatile bool_t   g_uart_tx_active = FALSE; // True if HAL_UART_Transmit_IT is ongoing

// Forward declaration for TX helper
static void platform_uart_start_transmit_if_needed(xbee_serial_t *s);
uint8_t g_hal_rx_byte_buffer;

// -----------------------------------------------------------------------------
// 4.  Mandatory helper – validate pointer
// -----------------------------------------------------------------------------
bool_t xbee_ser_invalid(xbee_serial_t *s)
{
    return (s == NULL) || (s->huart == NULL);
}

// -----------------------------------------------------------------------------
// 5.  Raw read / write primitives
// -----------------------------------------------------------------------------

// xbee_ser_write: Non-blocking, places data into TX ring buffer
int xbee_ser_write(xbee_serial_t *s, const void FAR *buf, int len)
{
    if (xbee_ser_invalid(s) || !buf || len < 0) return -EINVAL;
    if (len == 0) return 0;

    const uint8_t *data_to_write = (const uint8_t *)buf;
    int bytes_actually_written = 0;
    uint32_t primask_val;

    // 5a. Copy to TX debug ring (as before)
    for (int i = 0; i < len; ++i) {
        txdbg[txdbg_head] = data_to_write[i];
        txdbg_head = (txdbg_head + 1) & (TXDBG_SZ - 1);
    }

    // 5b. Attempt to copy data to the actual TX ring buffer
    primask_val = __get_PRIMASK(); // Save current global interrupt state
    __disable_irq();               // Enter critical section

    uint16_t current_tx_head = g_tx_buf_head; // Use local copies for manipulation
    uint16_t current_tx_tail = g_tx_buf_tail;
    uint16_t free_space_in_tx_buffer;

    if (current_tx_head >= current_tx_tail) { // No wrap
        free_space_in_tx_buffer = XBEE_PLATFORM_TX_BUF_SZ - (current_tx_head - current_tx_tail) - 1;
    } else { // Wrapped
        free_space_in_tx_buffer = (current_tx_tail - current_tx_head) - 1;
    }

    if (free_space_in_tx_buffer == 0) {
        __set_PRIMASK(primask_val); // Restore previous global interrupt state (Exit critical section)
        return 0;                   // Buffer full, indicate no bytes written (XBee lib should retry or use xbee_ser_tx_free)
    }

    int bytes_to_buffer = (len > free_space_in_tx_buffer) ? free_space_in_tx_buffer : len;

    for (int i = 0; i < bytes_to_buffer; ++i) {
        g_xbee_platform_tx_buf[current_tx_head] = data_to_write[i];
        current_tx_head = (current_tx_head + 1) & (XBEE_PLATFORM_TX_BUF_SZ - 1);
    }
    g_tx_buf_head = current_tx_head; // Update volatile head once
    bytes_actually_written = bytes_to_buffer;

    __set_PRIMASK(primask_val); // Restore previous global interrupt state (Exit critical section)

    platform_uart_start_transmit_if_needed(s); // Try to kick off UART transmission

    return bytes_actually_written;
}

// xbee_ser_read: Reads data from RX ring buffer (filled by HAL RX callback)
int xbee_ser_read(xbee_serial_t *s, void FAR *buf, int len)
{
    if (xbee_ser_invalid(s) || !buf || len <= 0) return -EINVAL;

    int cnt = 0;
    uint8_t *out_buf = (uint8_t *)buf;
    uint32_t primask_val;

    primask_val = __get_PRIMASK();
    __disable_irq(); // Protect rx_tail and rx_head from concurrent modification by ISR

    uint16_t current_rx_head = rx_head; // Use local copies
    uint16_t current_rx_tail = rx_tail;

    while (cnt < len && current_rx_tail != current_rx_head) {
        uint8_t b = rxbuf[current_rx_tail];
        current_rx_tail = (current_rx_tail + 1) & (RX_BUF_SZ - 1); // Consume byte

        // Handle API mode byte‑escaping (0x7D followed by byte to be XORed with 0x20)
        // The user's last version had this logic:
        // if (s->api_escape && b == XBEE_API_ESCAPE_CHAR) { // Check if API escaping is enabled
        // For now, assuming API escape is always active if escape char is found, as per user's last version.
        // This should ideally be controlled by `s->api_escape` if the XBee can be in transparent mode.
        if (b == XBEE_API_ESCAPE_CHAR) {
            if (current_rx_tail == current_rx_head) { // Escaped byte, but no following byte yet in buffer
                // Put the escape char back by retreating tail (it will be read again next call)
                current_rx_tail = (current_rx_tail - 1 + RX_BUF_SZ) & (RX_BUF_SZ - 1);
                break; // Exit, wait for the next byte to arrive
            }
            // Valid escape sequence, get the next byte and unescape it
            b = rxbuf[current_rx_tail] ^ XBEE_API_ESCAPE_MASK;
            current_rx_tail = (current_rx_tail + 1) & (RX_BUF_SZ - 1); // Consume the escaped byte
        }
        out_buf[cnt++] = b;
    }

    rx_tail = current_rx_tail; // Update the volatile tail pointer once
    __set_PRIMASK(primask_val); // Restore interrupt state

    return cnt; // Number of bytes actually read
}

// Convenience wrappers
int xbee_ser_putchar(xbee_serial_t *s, uint8_t ch)
{ return xbee_ser_write(s, &ch, 1) == 1 ? 0 : -ENOSPC; } // Or other error from write

int xbee_ser_getchar(xbee_serial_t *s)
{
    uint8_t ch;
    return xbee_ser_read(s, &ch, 1) == 1 ? ch : -ENODATA; // No data available
}

// -----------------------------------------------------------------------------
// 6.  Buffer status helpers
// -----------------------------------------------------------------------------
int xbee_ser_tx_flush(xbee_serial_t *s)
{
    if (xbee_ser_invalid(s)) return -EINVAL;
    uint32_t tickstart = HAL_GetTick();

    // Wait for the software TX buffer to be empty AND HAL TX to be inactive
    while (g_tx_buf_head != g_tx_buf_tail || g_uart_tx_active) {
        if ((HAL_GetTick() - tickstart) > XBEE_UART_TX_FLUSH_TIMEOUT_MS) {
            return -ETIMEDOUT;
        }
        // Give other tasks/interrupts a chance to run if in RTOS, or just spin in bare-metal
    }
    // Then, wait for the UART hardware to finish sending the last byte (Transmit Complete flag)
    tickstart = HAL_GetTick(); // Reset timeout for TC flag
    while (__HAL_UART_GET_FLAG(s->huart, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - tickstart) > XBEE_UART_TX_FLUSH_TIMEOUT_MS) {
            return -ETIMEDOUT;
        }
    }
    return 0;
}

int xbee_ser_tx_free (xbee_serial_t *s) {
    (void)s; // s could be used if buffers weren't global
    uint32_t primask_val;
    primask_val = __get_PRIMASK();
    __disable_irq();
    uint16_t current_tx_head = g_tx_buf_head;
    uint16_t current_tx_tail = g_tx_buf_tail;
    __set_PRIMASK(primask_val);

    uint16_t used_space_in_tx;
    if (current_tx_head >= current_tx_tail) {
        used_space_in_tx = current_tx_head - current_tx_tail;
    } else {
        used_space_in_tx = XBEE_PLATFORM_TX_BUF_SZ - (current_tx_tail - current_tx_head);
    }
    // -1 because head==tail means empty, so one byte is effectively unusable to distinguish full from empty.
    return (XBEE_PLATFORM_TX_BUF_SZ - 1) - used_space_in_tx;
}

int xbee_ser_tx_used (xbee_serial_t *s) {
    (void)s;
    uint32_t primask_val;
    primask_val = __get_PRIMASK();
    __disable_irq();
    uint16_t current_tx_head = g_tx_buf_head;
    uint16_t current_tx_tail = g_tx_buf_tail;
    __set_PRIMASK(primask_val);

    if (current_tx_head >= current_tx_tail) {
        return current_tx_head - current_tx_tail;
    } else {
        return XBEE_PLATFORM_TX_BUF_SZ - (current_tx_tail - current_tx_head);
    }
}

int xbee_ser_rx_used (xbee_serial_t *s) { (void)s; return rx_buffer_used(); }
int xbee_ser_rx_free (xbee_serial_t *s) { (void)s; return RX_BUF_SZ - 1 - rx_buffer_used(); } // -1 to distinguish full from empty
int xbee_ser_rx_flush(xbee_serial_t *s) {
    (void)s;
    uint32_t primask_val = __get_PRIMASK();
    __disable_irq();
    rx_head = rx_tail = 0;
    __set_PRIMASK(primask_val);
    return 0;
}

// -----------------------------------------------------------------------------
// 7.  UART Transmit Management (Interrupt-Driven using HAL)
// -----------------------------------------------------------------------------
static void platform_uart_start_transmit_if_needed(xbee_serial_t *s) {
    uint32_t primask_val;

    primask_val = __get_PRIMASK();
    __disable_irq(); // Start of critical section for checking/setting g_uart_tx_active

    if (g_uart_tx_active) { // If a HAL transmission is already ongoing, do nothing
        __set_PRIMASK(primask_val);
        return;
    }

    uint16_t current_tx_tail = g_tx_buf_tail; // Work with local copies
    uint16_t current_tx_head = g_tx_buf_head;

    if (current_tx_tail == current_tx_head) { // TX Buffer is empty
        __set_PRIMASK(primask_val);
        return;
    }

    // Buffer is not empty, and TX is not active, so start one
    g_uart_tx_active = TRUE;
    __set_PRIMASK(primask_val); // End of critical section for g_uart_tx_active set

    uint16_t count_to_send;
    if (current_tx_head > current_tx_tail) {
        // Data is in a single contiguous block
        count_to_send = current_tx_head - current_tx_tail;
    } else {
        // Data wraps around, send only the block from tail to end of buffer
        count_to_send = XBEE_PLATFORM_TX_BUF_SZ - current_tx_tail;
    }

    if (count_to_send == 0) { // Should not happen if buffer was not empty
        primask_val = __get_PRIMASK();
        __disable_irq();
        g_uart_tx_active = FALSE; // Reset as we are not starting a HAL transfer
        __set_PRIMASK(primask_val);
        return;
    }

    HAL_StatusTypeDef tx_hal_status;
    tx_hal_status = HAL_UART_Transmit_IT(s->huart, &g_xbee_platform_tx_buf[current_tx_tail], count_to_send);

    if (tx_hal_status != HAL_OK) {
        // Transmission could not be started by HAL
        primask_val = __get_PRIMASK();
        __disable_irq();
        g_uart_tx_active = FALSE; // Reset flag to allow retries
        __set_PRIMASK(primask_val);
        // Optionally, log this HAL error (e.g., s->huart->ErrorCode or tx_hal_status)
    }
}

// Called by HAL_UART_TxCpltCallback (which is in stm32f4xx_it.c)
void xbee_platform_uart_tx_cplt_callback(UART_HandleTypeDef *huart_that_completed) {
    // This function is now global (not static)
    if (huart_that_completed->Instance == xbee_port.huart->Instance) {
        uint32_t primask_val;
        uint16_t bytes_sent_in_last_xfer = huart_that_completed->TxXferSize;

        primask_val = __get_PRIMASK();
        __disable_irq(); // Start of critical section

        g_tx_buf_tail = (g_tx_buf_tail + bytes_sent_in_last_xfer) & (XBEE_PLATFORM_TX_BUF_SZ - 1);
        g_uart_tx_active = FALSE; // Mark HAL transmission as no longer active

        __set_PRIMASK(primask_val); // End of critical section

        // Attempt to send more data if any is pending in the TX ring buffer
        platform_uart_start_transmit_if_needed(&xbee_port);
    }
}

// -----------------------------------------------------------------------------
// 8.  UART Receive Management (Interrupt-Driven using HAL)
// -----------------------------------------------------------------------------

// Called by HAL_UART_RxCpltCallback (which is in stm32f4xx_it.c)
// to enqueue a received byte into the application's RX ring buffer.
void xbee_platform_hal_enqueue_rx_byte(uint8_t byte_val) {
    // This function is now global (not static)
    uint32_t primask_val;
    primask_val = __get_PRIMASK();
    __disable_irq(); // Protect rx_head and rx_tail

    uint16_t next_rx_head = (rx_head + 1) & (RX_BUF_SZ - 1);
    if (next_rx_head == rx_tail) {
        // RX Buffer full, byte is dropped.
        // Optionally, increment a counter for software buffer overruns.
        // g_rx_sw_overrun_count++;
    } else {
        rxbuf[rx_head] = byte_val;
        rx_head = next_rx_head;
    }
    __set_PRIMASK(primask_val); // Restore interrupt state
}


// -----------------------------------------------------------------------------
// 9.  Port / baud / control‑line management
// -----------------------------------------------------------------------------
const char *xbee_ser_portname(xbee_serial_t *s) { (void)s; return "USART6"; } // Or your UART

int xbee_ser_open(xbee_serial_t *s, uint32_t baud)
{
    if (xbee_ser_invalid(s)) return -EINVAL;

    if (s->huart->gState != HAL_UART_STATE_RESET) {
        // Abort any ongoing HAL IT operations before DeInit
        HAL_UART_AbortTransmit_IT(s->huart);
        HAL_UART_AbortReceive_IT(s->huart);
        HAL_UART_DeInit(s->huart);
    }

    s->baudrate = baud;
    s->huart->Init.BaudRate = baud;
    // Ensure other UART parameters (WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling)
    // are set correctly in your huart6.Init structure (usually done by CubeMX).

    if (HAL_UART_Init(s->huart) != HAL_OK) {
        return -EIO;
    }

    // Reset TX/RX buffers and states
    uint32_t primask_val = __get_PRIMASK();
    __disable_irq();
    g_tx_buf_head = g_tx_buf_tail = 0;
    g_uart_tx_active = FALSE;
    rx_head = rx_tail = 0;
    __set_PRIMASK(primask_val);

    __HAL_UART_FLUSH_DRREGISTER(s->huart); // Clear any pending data in DR

    // NVIC IRQ for USART6 should be enabled in main.c or HAL_UART_MspInit.
    // Example: HAL_NVIC_SetPriority(USART6_IRQn, 5, 0); HAL_NVIC_EnableIRQ(USART6_IRQn);

    // Start receiving data using HAL IT (1 byte at a time, re-armed in RxCpltCallback)
    if (HAL_UART_Receive_IT(s->huart, &g_hal_rx_byte_buffer, 1) != HAL_OK) {
        // Failed to start HAL receive IT, this is a critical error for RX path
        HAL_UART_DeInit(s->huart); // Clean up
        return -EIO;
    }
    // Note: HAL_UART_Receive_IT enables the RXNEIE and PEIE interrupts.
    // TXEIE/TCIE are enabled by HAL_UART_Transmit_IT when it's called.
    return 0;
}

int xbee_ser_baudrate(xbee_serial_t *s, uint32_t baud) {
    // Re-opening reinitializes with new baud and restarts HAL IT receive
    return xbee_ser_open(s, baud);
}

int xbee_ser_close(xbee_serial_t *s)
{
    if (xbee_ser_invalid(s)) return -EINVAL;

    // Disable UART interrupts at peripheral level
    // __HAL_UART_DISABLE_IT(s->huart, UART_IT_RXNE);
    // __HAL_UART_DISABLE_IT(s->huart, UART_IT_TXE);
    // __HAL_UART_DISABLE_IT(s->huart, UART_IT_TC);
    // __HAL_UART_DISABLE_IT(s->huart, UART_IT_PE);
    // __HAL_UART_DISABLE_IT(s->huart, UART_IT_ERR); // EIE for FE, NE, ORE

    // Abort any ongoing HAL IT operations
    HAL_UART_AbortTransmit_IT(s->huart);
    HAL_UART_AbortReceive_IT(s->huart); // This will also disable relevant IRQs

    // DeInitialize the UART peripheral
    if (HAL_UART_DeInit(s->huart) != HAL_OK) {
        return -EIO;
    }
    // NVIC IRQ can be left enabled or disabled here based on application needs.
    // If UART is closed permanently, consider HAL_NVIC_DisableIRQ(USART6_IRQn).
    return 0;
}

// NOPs / not used by all XBee modules or in all modes (especially API mode)
int xbee_ser_break       (xbee_serial_t *s, bool_t en) { (void)s; (void)en; return 0; }
int xbee_ser_flowcontrol (xbee_serial_t *s, bool_t en) { (void)s; (void)en; return 0; }
int xbee_ser_set_rts     (xbee_serial_t *s, bool_t a ) { (void)s; (void)a ; return 0; }
int xbee_ser_get_cts     (xbee_serial_t *s)            { (void)s; return 1; } // Assuming CTS is always asserted (ready)

void xbee_platform_config(UART_HandleTypeDef *huart, uint32_t baud)
{
    xbee_port.huart    = huart;
    xbee_port.baudrate = baud;
}

// -----------------------------------------------------------------------------
// 10.  Convenience init wrapper & Timer functions (largely unchanged)
// -----------------------------------------------------------------------------
void xbee_platform_init(void) {
    // Assuming huart6 is initialized by MX_USART6_UART_Init() from main.c before this.
    // Also, NVIC for USART6_IRQn should be enabled before xbee_ser_open is called.
	if (! xbee_port.huart) {
		Error_Handler();
	}

	if (xbee_ser_open(&xbee_port, xbee_port.baudrate) != 0) {
		// TODO: Error_Handler() or report failure
	}
}

uint32_t xbee_millisecond_timer(void) {
    return HAL_GetTick();
}

uint32_t xbee_seconds_timer(void) {
    return HAL_GetTick() / 1000;
}

xbee_serial_t *xbee_platform_serial(void) {
    return &xbee_port;
}
