// -----------------------------------------------------------------------------
// xbee_platform_uart.h – Header for STM32F4 (UART6) port layer for Digi ANSI‐C XBee library
// -----------------------------------------------------------------------------
//  * Declares public functions for the XBee UART platform interface.
//  * These functions provide the xbee_ser_*() API required by the Digi XBee library,
//    along with platform initialization, timer, and debug utilities.
//
// 2025‑05‑11  Generated based on xbee_platform_uart.c
// -----------------------------------------------------------------------------

#ifndef XBEE_PLATFORM_UART_H
#define XBEE_PLATFORM_UART_H

#include <stdint.h>
#include "stm32/platform_config.h" // Provides xbee_serial_t, bool_t, FAR, and includes stm32f4xx_hal.h

// It's assumed that xbee/serial.h (from the Digi XBee library) would define
// the function signatures that the xbee_ser_* functions below implement.
// This header declares the specific implementations provided by xbee_platform_uart.c.

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// TX Debug Ring Buffer Access (Implementation Specific)
// -----------------------------------------------------------------------------

/**
 * @brief Gets the current head index of the TX debug ring buffer.
 * @return The head index.
 */
uint16_t xbee_txdbg_head(void);

/**
 * @brief Gets a constant pointer to the TX debug ring buffer.
 * @return Pointer to the TX debug buffer.
 */
const uint8_t *xbee_txdbg_buffer(void);

// -----------------------------------------------------------------------------
// UART Interrupt Service Routine (Platform Specific)
// -----------------------------------------------------------------------------

/**
 * @brief UART Interrupt Service Routine for XBee communication.
 * This function should be called from the corresponding UART ISR handler
 * (e.g., USART6_IRQHandler for STM32F4). It reads incoming bytes into the
 * RX ring buffer.
 */
void xbee_uart_isr(void);

// -----------------------------------------------------------------------------
// XBee Serial API Implementation (Mandatory for Digi XBee Library)
// These functions implement the API defined in xbee/serial.h
// -----------------------------------------------------------------------------

/**
 * @brief Checks if the provided XBee serial port structure is invalid.
 * @param s Pointer to the xbee_serial_t structure.
 * @return TRUE if invalid, FALSE otherwise.
 */
bool_t xbee_ser_invalid(xbee_serial_t *s);

/**
 * @brief Writes a block of data to the XBee serial port.
 * @param s Pointer to the xbee_serial_t structure.
 * @param buf Pointer to the data buffer to write.
 * @param len Number of bytes to write.
 * @return Number of bytes written, or a negative error code.
 */
int xbee_ser_write(xbee_serial_t *s, const void FAR *buf, int len);

/**
 * @brief Reads a block of data from the XBee serial port.
 * Handles API mode byte-escaping.
 * @param s Pointer to the xbee_serial_t structure.
 * @param buf Pointer to the buffer to store read data.
 * @param len Maximum number of bytes to read.
 * @return Number of bytes read (can be less than len if not enough data is available),
 * or a negative error code. Returns 0 if no data is available.
 */
int xbee_ser_read(xbee_serial_t *s, void FAR *buf, int len);

/**
 * @brief Writes a single character to the XBee serial port.
 * @param s Pointer to the xbee_serial_t structure.
 * @param ch Character to write.
 * @return 0 on success, or a negative error code.
 */
int xbee_ser_putchar(xbee_serial_t *s, uint8_t ch);

/**
 * @brief Reads a single character from the XBee serial port.
 * @param s Pointer to the xbee_serial_t structure.
 * @return The character read (0-255) on success, or a negative error code (e.g., -ENODATA).
 */
int xbee_ser_getchar(xbee_serial_t *s);

/**
 * @brief Flushes the transmit buffer (waits for all data to be sent).
 * @param s Pointer to the xbee_serial_t structure.
 * @return 0 on success, or a negative error code.
 */
int xbee_ser_tx_flush(xbee_serial_t *s);

/**
 * @brief Gets the number of free bytes in the transmit buffer.
 * @param s Pointer to the xbee_serial_t structure.
 * @return Estimated number of free bytes.
 */
int xbee_ser_tx_free(xbee_serial_t *s);

/**
 * @brief Gets the number of used bytes in the transmit buffer.
 * @param s Pointer to the xbee_serial_t structure.
 * @return Estimated number of used bytes (typically 0 for blocking transmit).
 */
int xbee_ser_tx_used(xbee_serial_t *s);

/**
 * @brief Gets the number of bytes available in the receive buffer.
 * @param s Pointer to the xbee_serial_t structure.
 * @return Number of bytes available to read.
 */
int xbee_ser_rx_used(xbee_serial_t *s);

/**
 * @brief Gets the number of free bytes in the receive buffer.
 * @param s Pointer to the xbee_serial_t structure.
 * @return Number of free bytes in the RX buffer.
 */
int xbee_ser_rx_free(xbee_serial_t *s);

/**
 * @brief Flushes (clears) the receive buffer.
 * @param s Pointer to the xbee_serial_t structure.
 * @return 0 on success.
 */
int xbee_ser_rx_flush(xbee_serial_t *s);

/**
 * @brief Gets the port name string.
 * @param s Pointer to the xbee_serial_t structure.
 * @return Constant string representing the port name (e.g., "USART6").
 */
const char *xbee_ser_portname(xbee_serial_t *s);

/**
 * @brief Opens and initializes the XBee serial port.
 * @param s Pointer to the xbee_serial_t structure.
 * @param baud Baud rate to configure.
 * @return 0 on success, or a negative error code.
 */
int xbee_ser_open(xbee_serial_t *s, uint32_t baud);

/**
 * @brief Sets the baud rate of the XBee serial port.
 * @param s Pointer to the xbee_serial_t structure.
 * @param baud New baud rate.
 * @return 0 on success, or a negative error code.
 */
int xbee_ser_baudrate(xbee_serial_t *s, uint32_t baud);

/**
 * @brief Closes the XBee serial port.
 * @param s Pointer to the xbee_serial_t structure.
 * @return 0 on success, or a negative error code.
 */
int xbee_ser_close(xbee_serial_t *s);

/**
 * @brief Controls the serial break condition (Not typically used with XBee API mode).
 * @param s Pointer to the xbee_serial_t structure.
 * @param enabled TRUE to enable break, FALSE to disable.
 * @return 0 (typically a NOP for XBee).
 */
int xbee_ser_break(xbee_serial_t *s, bool_t enabled);

/**
 * @brief Enables or disables hardware flow control (Not typically used if not configured).
 * @param s Pointer to the xbee_serial_t structure.
 * @param enabled TRUE to enable flow control, FALSE to disable.
 * @return 0 (typically a NOP).
 */
int xbee_ser_flowcontrol(xbee_serial_t *s, bool_t enabled);

/**
 * @brief Sets the RTS (Request To Send) line state (Not typically used if no HW flow control).
 * @param s Pointer to the xbee_serial_t structure.
 * @param asserted TRUE to assert RTS, FALSE to de-assert.
 * @return 0 (typically a NOP).
 */
int xbee_ser_set_rts(xbee_serial_t *s, bool_t asserted);

/**
 * @brief Gets the CTS (Clear To Send) line state (Not typically used if no HW flow control).
 * @param s Pointer to the xbee_serial_t structure.
 * @return 1 if CTS is asserted (assumed ready), 0 otherwise.
 */
int xbee_ser_get_cts(xbee_serial_t *s);

// -----------------------------------------------------------------------------
// Platform Initialization and Timer Functions (Required/Useful for XBee Library)
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the XBee platform layer, particularly the serial port.
 * Should be called once at application startup.
 */
void xbee_platform_init(void);

/**
 * @brief Provides a millisecond timer value for the XBee library.
 * @return Current time in milliseconds.
 */
uint32_t xbee_millisecond_timer(void);

/**
 * @brief Provides a seconds timer value for the XBee library.
 * @return Current time in seconds.
 */
uint32_t xbee_seconds_timer(void);

/**
 * @brief Gets a pointer to the platform's default XBee serial port structure.
 * @return Pointer to the xbee_serial_t structure for the XBee UART.
 */
xbee_serial_t *xbee_platform_serial(void);


#ifdef __cplusplus
}
#endif

#endif // XBEE_PLATFORM_UART_H
