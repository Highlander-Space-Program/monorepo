/*
 * xbee_handler.h
 *
 * Application-level interface for interacting with the XBee module.
 * Manages a queue of parsed, received XBee frames and provides
 * functions for sending data and AT commands.
 *
 * Created on: May 11, 2025
 * Author: brandonmarcus
 *
 * Refactored: 2025-05-11
 * - xbee_handler_send_byte_array and xbee_handler_send_u32 now take xbee_dev_t* and target address params.
 * - Renamed send_broadcast to xbee_handler_send_broadcast_message and updated its signature.
 * - Added return types to send functions.
 * - Clarified comments and payload size definition.
 */

#ifndef __XBEE_HANDLER_H
#define __XBEE_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "xbee/device.h" // For xbee_dev_t, addr64, XBEE_MAX_RFPAYLOAD
#include "stm32/platform_config.h" // For platform-specific configs, potentially XBEE_MAX_RFPAYLOAD if defined there

// Define the maximum payload size for XBee frames processed by this handler.
// This should typically align with or be less than XBEE_MAX_RFPAYLOAD
// from the Digi XBee library, which depends on the XBee module type.
// (e.g., 100 bytes for 802.15.4, up to 255 for Zigbee, or more for others).
// If XBEE_MAX_RFPAYLOAD is not available directly from platform_config.h,
// ensure this define is appropriate for your XBee module.
#ifndef XBEE_HANDLER_PAYLOAD_MAX_SIZE
#define XBEE_HANDLER_PAYLOAD_MAX_SIZE XBEE_MAX_RFPAYLOAD
#endif

// Structure to hold a processed XBee received frame's payload and metadata
typedef struct {
    uint8_t payload[XBEE_HANDLER_PAYLOAD_MAX_SIZE];
    uint16_t length;
    addr64 source_addr_64;    // 64-bit IEEE address of the sender
    uint16_t source_addr_16;  // 16-bit network address (short address) of the sender
    uint8_t receive_options; // From XBEE_FRAME_RECEIVE or XBEE_FRAME_RECEIVE_EXPLICIT (e.g., XBEE_RX_OPT_BROADCAST)
    uint8_t rssi;            // Received Signal Strength Indicator (0 if not available/parsed)
} XBeeRxFrame_t;

// Options for sending XBee frames via xbee_handler_send_data_frame
#define XBEE_HANDLER_TX_OPT_NONE         0x00 // Standard transmit options (e.g., ACK enabled, route discovery enabled)
#define XBEE_HANDLER_TX_OPT_DISABLE_ACK  0x01 // Disable MAC ACK (maps to XBEE_TX_OPT_DISABLE_ACK)
// Add more application-level options as needed, to be mapped to library options.


/**
 * @brief Initializes the XBee received frame queue managed by this handler.
 * Must be called before using the handler's RX queue functions.
 */
void xbee_handler_init_rx_queue(void);

/**
 * @brief Checks if there are any parsed XBee frames available in this handler's received queue.
 * @return True if frames are available, false otherwise.
 */
bool xbee_handler_is_rx_frame_available(void);

/**
 * @brief Attempts to dequeue a parsed XBee frame from this handler's received queue.
 * @param frame Pointer to an XBeeRxFrame_t structure where the dequeued frame will be copied.
 * @return True if a frame was dequeued successfully, false if the queue was empty.
 */
bool xbee_handler_rx_frame_dequeue(XBeeRxFrame_t* frame);

/**
 * @brief Services the XBee library's raw packet output queue from xbee_actions.
 * This function should be called periodically (e.g., in the main application loop).
 * It dequeues raw packets from the low-level FIFO (managed by xbee_actions.c),
 * parses them into XBeeRxFrame_t format, and enqueues them into the queue
 * managed by this handler.
 */
void xbee_handler_service_rx_from_library(void);

/**
 * @brief Reads a uint32_t value from an XBee module using an AT command synchronously.
 * This function blocks until the command completes or times out.
 * @param xbee_dev Pointer to the initialized XBee device structure.
 * @param at_cmd The 2-character AT command string (e.g., "AP", "SL").
 * @param out_value Pointer to a uint32_t where the result will be stored.
 * @param timeout_ms Timeout in milliseconds to wait for the command response.
 * @return 0 on success, negative error code on failure (e.g., -ETIMEDOUT, -EIO, -EINVAL).
 */
int xbee_handler_at_cmd_read_u32_sync(xbee_dev_t *xbee_dev, const char *at_cmd, uint32_t *out_value, uint32_t timeout_ms);

/**
 * @brief Sends an XBee data frame (Transmit Request, API Frame Type 0x10).
 * This is the primary function for sending unicast or broadcast data.
 *
 * @param xbee_dev Pointer to the initialized XBee device structure.
 * @param dest_addr_64 Pointer to the 64-bit IEEE address of the destination.
 * For broadcasts, use WPAN_IEEE_ADDR_BROADCAST.
 * If NULL and dest_addr_16 is specific, the XBee might attempt address resolution
 * or use its configured 64-bit address if dest_addr_16 is 0xFFFE.
 * @param dest_addr_16 The 16-bit network address of the destination.
 * Use WPAN_NET_ADDR_BROADCAST (0xFFFF) for broadcast.
 * Use WPAN_NET_ADDR_UNDEFINED (0xFFFE) if unknown and relying on 64-bit address for unicast.
 * @param data Pointer to the payload data to send.
 * @param len Length of the payload data. Must be <= XBEE_HANDLER_PAYLOAD_MAX_SIZE.
 * @param frame_id_request The XBee frame ID to use for this transmission.
 * - Use 0 for no transmit status response from the XBee.
 * - Use 1 to request the XBee library to assign the next available frame ID and expect a transmit status.
 * - Any other non-zero value will be used as the frame ID, and a transmit status is expected.
 * @param options Transmit options (e.g., XBEE_HANDLER_TX_OPT_DISABLE_ACK).
 * @return 0 on successful queuing for transmission by the XBee library, negative error code on failure.
 */
int xbee_handler_send_data_frame(xbee_dev_t *xbee_dev,
                                 const addr64* dest_addr_64,
                                 uint16_t dest_addr_16,
                                 const uint8_t* data,
                                 uint16_t len,
                                 uint8_t frame_id_request,
                                 uint8_t options);

/**
 * @brief Sends a byte array to a specified XBee destination.
 * Wrapper around xbee_handler_send_data_frame.
 *
 * @param xbee_dev Pointer to the initialized XBee device structure.
 * @param dest_addr_64 Pointer to the 64-bit IEEE address of the destination.
 * @param dest_addr_16 The 16-bit network address of the destination.
 * @param payload Pointer to the byte array to send.
 * @param length Length of the byte array.
 * @param frame_id_request Frame ID for transmit status (0 for none, 1 for auto).
 * @param options Transmit options (e.g., XBEE_HANDLER_TX_OPT_DISABLE_ACK).
 * @return 0 on successful queuing, negative error code otherwise.
 */
int xbee_handler_send_byte_array(xbee_dev_t *xbee_dev,
                                  const addr64* dest_addr_64,
                                  uint16_t dest_addr_16,
                                  const uint8_t* payload,
                                  uint16_t length,
                                  uint8_t frame_id_request,
                                  uint8_t options);

/**
 * @brief Sends a uint32_t value to a specified XBee destination.
 * The value is sent as a 4-byte payload (implementation specific, e.g. big-endian).
 * Wrapper around xbee_handler_send_data_frame.
 *
 * @param xbee_dev Pointer to the initialized XBee device structure.
 * @param dest_addr_64 Pointer to the 64-bit IEEE address of the destination.
 * @param dest_addr_16 The 16-bit network address of the destination.
 * @param value The uint32_t value to send.
 * @param frame_id_request Frame ID for transmit status (0 for none, 1 for auto).
 * @param options Transmit options (e.g., XBEE_HANDLER_TX_OPT_DISABLE_ACK).
 * @return 0 on successful queuing, negative error code otherwise.
 */
int xbee_handler_send_u32(xbee_dev_t *xbee_dev,
                          const addr64* dest_addr_64,
                          uint16_t dest_addr_16,
                          uint32_t value,
                          uint8_t frame_id_request,
                          uint8_t options);

/**
 * @brief Sends a broadcast message with the given string payload.
 * Uses WPAN_IEEE_ADDR_BROADCAST and WPAN_NET_ADDR_BROADCAST.
 *
 * @param xbee_dev Pointer to the initialized XBee device structure.
 * @param message Pointer to the null-terminated string message to broadcast.
 * @return 0 on successful queuing, negative error code otherwise.
 */
int xbee_handler_send_broadcast_message(xbee_dev_t *xbee_dev, const char* message);


#endif /* __XBEE_HANDLER_H */
