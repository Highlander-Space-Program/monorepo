/*
 * xbee_handler.c
 *
 * Implements the application-level XBee interface.
 *
 * Refactored: 2025-05-11
 * Corrected: 2025-05-12 (Addressing specific compiler errors)
 * Corrected: 2025-05-12 (Fallback for missing XBEE_ATCMD_ERROR and WPAN_NET_ADDR_BROADCAST)
 * Corrected: 2025-05-12 (Removed potentially problematic xbee_cmd_set_context call)
 */
#include "xbee_handler.h"       // Own header, defines XBeeRxFrame_t, XBEE_HANDLER_PAYLOAD_MAX_SIZE, prototypes
#include "stm32/xbee_actions.h" // For xbee_rx_packet_available(), xbee_rx_packet_dequeue(), rx_packet_t
#include "xbee/device.h"        // Explicitly include for xbee_dev_t, XBee frame types, addr64, etc.
#include "xbee/atcmd.h"         // Explicitly include for AT command types, XBEE_ATCMD_DONE
                                // XBEE_ATCMD_ERROR is expected here. If missing, a fallback is used.
#include "xbee/byteorder.h"     // For be16toh, htobe16, htobe32, be32toh
#include "xbee/wpan.h"          // For WPAN_NET_ADDR_UNDEFINED, WPAN_IEEE_ADDR_BROADCAST
                                // WPAN_NET_ADDR_BROADCAST is expected here. If missing, 0xFFFF is used.
#include <string.h>             // For memcpy, memset, strlen
#include <stddef.h>             // For offsetof
#include <errno.h>              // For ETIMEDOUT, EIO, EINVAL, EMSGSIZE, EPROTONOSUPPORT, EFAULT
#include <stdio.h>              // For printf (if used for debugging)

// Fallback definition if XBEE_ATCMD_ERROR is not found in your xbee/atcmd.h
#ifndef XBEE_ATCMD_ERROR
#define XBEE_ATCMD_ERROR (-1) // General error for AT command callback
#endif

// Fallback definition if WPAN_NET_ADDR_BROADCAST is not found in your xbee/wpan.h
#ifndef WPAN_NET_ADDR_BROADCAST
#define WPAN_NET_ADDR_BROADCAST ((uint16_t)0xFFFF)
#endif

// --- XBee Received Frame Queue (managed by this handler for parsed frames) ---
#ifndef XBEE_HANDLER_RX_FIFO_DEPTH
#define XBEE_HANDLER_RX_FIFO_DEPTH 8
#endif
static volatile XBeeRxFrame_t xbee_handler_rx_fifo[XBEE_HANDLER_RX_FIFO_DEPTH];
static volatile uint8_t xbee_handler_rx_head = 0;
static volatile uint8_t xbee_handler_rx_tail = 0;
static volatile uint8_t xbee_handler_rx_count = 0;

// --- AT Command Synchronous Read Helper Structure (internal to this module) ---
typedef struct {
    volatile bool done;
    uint32_t      value;
    uint16_t      flags;
} internal_at_sync_context_t;


// --- Internal static functions ---

static int internal_at_cmd_sync_callback(const xbee_cmd_response_t *response) {
    if (!response || !response->context) {
        return XBEE_ATCMD_ERROR;
    }
    internal_at_sync_context_t *sync_context = (internal_at_sync_context_t *)response->context;

    uint32_t val = 0;
    if (response->value_length > 0 && response->value_bytes != NULL) {
        switch (response->value_length) {
            case 1: val = response->value_bytes[0]; break;
            case 2: val = be16toh(*(uint16_t *)response->value_bytes); break;
            case 4: val = be32toh(*(uint32_t *)response->value_bytes); break;
            default: /* Value too long or not handled */ break;
        }
    }

    sync_context->value = val;
    sync_context->flags = response->flags;
    sync_context->done  = true;

    return XBEE_ATCMD_DONE;
}

static bool internal_xbee_rx_frame_enqueue(const XBeeRxFrame_t* frame) {
    if (frame == NULL || xbee_handler_rx_count >= XBEE_HANDLER_RX_FIFO_DEPTH) {
        return false;
    }
    xbee_handler_rx_fifo[xbee_handler_rx_head] = *frame;
    xbee_handler_rx_head = (xbee_handler_rx_head + 1) % XBEE_HANDLER_RX_FIFO_DEPTH;
    xbee_handler_rx_count++;
    return true;
}

static int internal_extract_data_from_raw_xbee_frame(const rx_packet_t* raw_packet, XBeeRxFrame_t* parsed_frame) {
    if (raw_packet == NULL || parsed_frame == NULL || raw_packet->len == 0) {
        return -EINVAL;
    }

    uint8_t frame_type = raw_packet->buf[0];
    const uint8_t* p_payload_start_in_raw_frame = NULL;
    uint16_t actual_payload_length = 0;
    size_t header_size_before_payload = 0;

    memset(parsed_frame, 0, sizeof(XBeeRxFrame_t));

    if (frame_type == XBEE_FRAME_RECEIVE) { // 0x90
        if (raw_packet->len < offsetof(xbee_frame_receive_t, payload)) {
            return -EMSGSIZE;
        }
        const xbee_frame_receive_t* rx_frame = (const xbee_frame_receive_t*)raw_packet->buf;
        parsed_frame->source_addr_64 = rx_frame->ieee_address;
        parsed_frame->source_addr_16 = be16toh(rx_frame->network_address_be);
        parsed_frame->receive_options = rx_frame->options;
        header_size_before_payload = offsetof(xbee_frame_receive_t, payload);
        p_payload_start_in_raw_frame = rx_frame->payload;

    } else if (frame_type == XBEE_FRAME_RECEIVE_EXPLICIT) { // 0x91
        if (raw_packet->len < offsetof(xbee_frame_receive_explicit_t, payload)) {
            return -EMSGSIZE;
        }
        const xbee_frame_receive_explicit_t* rx_expl_frame = (const xbee_frame_receive_explicit_t*)raw_packet->buf;
        parsed_frame->source_addr_64 = rx_expl_frame->ieee_address;
        parsed_frame->source_addr_16 = be16toh(rx_expl_frame->network_address_be);
        parsed_frame->receive_options = rx_expl_frame->options;
        header_size_before_payload = offsetof(xbee_frame_receive_explicit_t, payload);
        p_payload_start_in_raw_frame = rx_expl_frame->payload;
    } else {
        return -EPROTONOSUPPORT;
    }

    if (raw_packet->len > header_size_before_payload) {
        actual_payload_length = raw_packet->len - header_size_before_payload;
    } else {
        actual_payload_length = 0;
    }

    if (p_payload_start_in_raw_frame != NULL && actual_payload_length > 0) {
        uint16_t copy_len = (actual_payload_length > XBEE_HANDLER_PAYLOAD_MAX_SIZE) ?
                              XBEE_HANDLER_PAYLOAD_MAX_SIZE : actual_payload_length;
        memcpy(parsed_frame->payload, p_payload_start_in_raw_frame, copy_len);
        parsed_frame->length = copy_len;
    } else {
        parsed_frame->length = 0;
    }
    parsed_frame->rssi = 0;
    return 0;
}

// --- Public API functions ---

void xbee_handler_init_rx_queue(void) {
    xbee_handler_rx_head = 0;
    xbee_handler_rx_tail = 0;
    xbee_handler_rx_count = 0;
}

bool xbee_handler_is_rx_frame_available(void) {
    return xbee_handler_rx_count > 0;
}

bool xbee_handler_rx_frame_dequeue(XBeeRxFrame_t* frame) {
    if (frame == NULL || xbee_handler_rx_count == 0) {
        return false;
    }
    *frame = xbee_handler_rx_fifo[xbee_handler_rx_tail];
    xbee_handler_rx_tail = (xbee_handler_rx_tail + 1) % XBEE_HANDLER_RX_FIFO_DEPTH;
    xbee_handler_rx_count--;
    return true;
}

void xbee_handler_service_rx_from_library(void) {
    rx_packet_t raw_xbee_packet;
    XBeeRxFrame_t new_parsed_frame;

    if (xbee_rx_packet_available()) {
        if (xbee_rx_packet_dequeue(&raw_xbee_packet) > 0) {
            if (internal_extract_data_from_raw_xbee_frame(&raw_xbee_packet, &new_parsed_frame) == 0) {
                if (!internal_xbee_rx_frame_enqueue(&new_parsed_frame)) {
                    // Parsed frame queue full
                }
            }
        }
    }
}

int xbee_handler_at_cmd_read_u32_sync(xbee_dev_t *xbee_dev, const char *at_cmd, uint32_t *out_value, uint32_t timeout_ms) {
    if (!xbee_dev || !at_cmd || !out_value || strlen(at_cmd) != 2) return -EINVAL;

    int16_t cmd_handle = xbee_cmd_create(xbee_dev, at_cmd);
    if (cmd_handle < 0) return cmd_handle;

    internal_at_sync_context_t sync_context = { .done = false, .value = 0, .flags = 0 };

    xbee_cmd_set_callback(cmd_handle, internal_at_cmd_sync_callback, &sync_context);

    int send_status = xbee_cmd_send(cmd_handle);
    if (send_status != 0) {
        // xbee_cmd_release(cmd_handle); // Library should auto-release on send error or XBEE_ATCMD_DONE
        return send_status;
    }

    uint32_t cmd_start_time = XBEE_MS_TIMER();
    while (!sync_context.done) {
        if ((XBEE_MS_TIMER() - cmd_start_time) > timeout_ms) {
            // Application-level timeout.
            // If the command is still pending, xbee_cmd_delete(cmd_handle) might be considered
            // if the library doesn't guarantee cleanup on timeout when callback isn't XBEE_ATCMD_DONE.
            // However, for simplicity, we rely on the library's internal timeout or the callback mechanism.
            return -ETIMEDOUT;
        }
        // Ensure xbee_dev_tick() and xbee_cmd_tick() are called in the main application loop
        // for AT command responses to be processed.
    }

    if (XBEE_AT_RESP_STATUS(sync_context.flags) != XBEE_AT_RESP_SUCCESS) {
        return -EIO; // Modem returned an error status for the AT command
    }
    *out_value = sync_context.value;
    return 0;
}

int xbee_handler_send_data_frame(xbee_dev_t *xbee_dev,
                                 const addr64* dest_addr_64,
                                 uint16_t dest_addr_16,
                                 const uint8_t* data,
                                 uint16_t len,
                                 uint8_t frame_id_request,
                                 uint8_t options_flags) {
    if (!xbee_dev || (!data && len > 0) ) return -EINVAL;
    if (len > XBEE_HANDLER_PAYLOAD_MAX_SIZE) return -EMSGSIZE;

    uint8_t actual_xbee_frame_id = (frame_id_request == 1) ? xbee_next_frame_id(xbee_dev) : frame_id_request;

    xbee_header_transmit_t tx_header = {
        .frame_type         = XBEE_FRAME_TRANSMIT,
        .frame_id           = actual_xbee_frame_id,
        .network_address_be = htobe16(dest_addr_16),
        .broadcast_radius   = 0,
        .options            = 0
    };

    if (options_flags & XBEE_HANDLER_TX_OPT_DISABLE_ACK) {
        tx_header.options |= XBEE_TX_OPT_DISABLE_ACK;
    }

    if (dest_addr_64) {
        tx_header.ieee_address = *dest_addr_64;
    } else {
        if (dest_addr_16 == WPAN_NET_ADDR_BROADCAST) { // Uses fallback if macro not defined
             tx_header.ieee_address = *WPAN_IEEE_ADDR_BROADCAST;
        } else if (dest_addr_16 == WPAN_NET_ADDR_UNDEFINED) {
             memset(&tx_header.ieee_address, 0xFF, sizeof(addr64));
             tx_header.ieee_address.b[0] = 0xFE;
             tx_header.ieee_address.b[1] = 0xFF;
        } else {
            memset(&tx_header.ieee_address, 0xFF, sizeof(addr64));
            tx_header.ieee_address.b[0] = 0xFE;
            tx_header.ieee_address.b[1] = 0xFF;
        }
    }

    return xbee_frame_write(xbee_dev, &tx_header, sizeof(tx_header), data, len, XBEE_WRITE_FLAG_NONE);
}

int xbee_handler_send_byte_array(xbee_dev_t *xbee_dev,
                                  const addr64* dest_addr_64,
                                  uint16_t dest_addr_16,
                                  const uint8_t* payload,
                                  uint16_t length,
                                  uint8_t frame_id_request,
                                  uint8_t options) {
    if (!payload && length > 0) return -EINVAL;
    if (length > XBEE_HANDLER_PAYLOAD_MAX_SIZE) return -EMSGSIZE;

    return xbee_handler_send_data_frame(xbee_dev, dest_addr_64, dest_addr_16,
                                       payload, length, frame_id_request, options);
}

int xbee_handler_send_u32(xbee_dev_t *xbee_dev,
                          const addr64* dest_addr_64,
                          uint16_t dest_addr_16,
                          uint32_t value,
                          uint8_t frame_id_request,
                          uint8_t options) {
    uint8_t payload_buf[4];
    uint32_t value_be = htobe32(value);
    memcpy(payload_buf, &value_be, sizeof(payload_buf));

    return xbee_handler_send_data_frame(xbee_dev, dest_addr_64, dest_addr_16,
                                       payload_buf, sizeof(payload_buf), frame_id_request, options);
}

int xbee_handler_send_broadcast_message(xbee_dev_t *xbee_dev, const char* message) {
    if (!xbee_dev || !message) return -EINVAL;
    uint16_t message_len = strlen(message);
    if (message_len == 0) return -EINVAL;
    if (message_len > XBEE_HANDLER_PAYLOAD_MAX_SIZE) return -EMSGSIZE;

    const addr64* ieee_bcast = WPAN_IEEE_ADDR_BROADCAST;
    uint16_t net_bcast = WPAN_NET_ADDR_BROADCAST; // Uses fallback if macro not defined

    return xbee_handler_send_data_frame(xbee_dev, ieee_bcast, net_bcast,
                                       (const uint8_t*)message, message_len,
                                       0, XBEE_HANDLER_TX_OPT_NONE);
}
