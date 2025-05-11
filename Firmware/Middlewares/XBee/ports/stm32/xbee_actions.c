/*
 * xbee_actions.c
 *
 * Implements callbacks for the Digi XBee C library and manages a
 * low-level FIFO for incoming raw XBee frames.
 *
 * 2025‑05‑10  – adds support for plain 0x90 “Receive Packet” frames and a
 * FIFO so higher‑level code can pull them out whenever it likes.
 *
 * Refactored: 2025-05-11
 * - Uses FRAME_QUEUE_SIZE from xbee_actions.h for FIFO depth.
 * - Renamed xbee_rx_available to xbee_rx_packet_available.
 * - Renamed xbee_rx_dequeue to xbee_rx_packet_dequeue.
 * - Clarified comments.
 *
 * Corrected: 2025-05-11
 * - Ensured definitions from xbee_actions.h (FRAME_QUEUE_SIZE, rx_packet_t, RX_MAX_LEN) are correctly utilized.
 */
#include "stm32/xbee_actions.h" // Defines FRAME_QUEUE_SIZE, rx_packet_t, RX_MAX_LEN, callback prototypes
#include "xbee/device.h"        // For xbee_dev_t, xbee_frame_receive_t, etc.
#include "xbee/byteorder.h"     // For endian conversion utilities
#include "xbee/delivery_status.h" // For transmit status constants
#include "xbee/atcmd.h"         // For XBEE_FRAME_HANDLE_LOCAL_AT, xbee_frame_dump_modem_status
#include "xbee/wpan.h"          // For WPAN address types and constants
#include <string.h>             // For memcpy
#include <stdio.h>              // For printf (if used for debugging)

/* --------------------------------------------------------------------------
 * 1.  Tiny circular buffer (FIFO) to keep the last FRAME_QUEUE_SIZE packets
 * FRAME_QUEUE_SIZE, rx_packet_t, and RX_MAX_LEN are defined in xbee_actions.h
 * -------------------------------------------------------------------------- */
static volatile rx_packet_t raw_packet_fifo[FRAME_QUEUE_SIZE];
static volatile uint8_t     fifo_write_idx = 0;     /* writer position           */
static volatile uint8_t     fifo_read_idx = 0;      /* reader position           */

// Helper to calculate the next index with wrap-around
static inline uint8_t next_fifo_idx(uint8_t current_idx) {
    return (uint8_t)((current_idx + 1u) % FRAME_QUEUE_SIZE);
}

/**
 * @brief Checks if there are any raw XBee packets available in the low-level FIFO.
 */
bool xbee_rx_packet_available(void) {
    return fifo_read_idx != fifo_write_idx;
}

/**
 * @brief Dequeues the next raw XBee packet from the low-level FIFO.
 * Copies the packet data into the provided dst structure.
 */
uint16_t xbee_rx_packet_dequeue(rx_packet_t *dst) {
    if (dst == NULL || !xbee_rx_packet_available()) { // Added NULL check for dst
        return 0; // FIFO is empty or invalid argument
    }

    // Atomically (or as close as possible without explicit locks in this context)
    // get the read index and update it. For single consumer, this is generally safe.
    uint8_t current_read_idx = fifo_read_idx;


    // Copy data from FIFO to destination buffer
    // Note: Accessing volatile struct members.
    dst->len = raw_packet_fifo[current_read_idx].len;
    memcpy(dst->buf,
           (const void *)raw_packet_fifo[current_read_idx].buf, // Discard volatile for memcpy source
           dst->len);

    // Advance read index after successful copy
    fifo_read_idx = next_fifo_idx(fifo_read_idx);

    return dst->len;
}

/**
 * @brief Stores a raw XBee frame into the FIFO.
 * If the FIFO is full, the oldest packet is overwritten.
 * @param frame Pointer to the raw frame data.
 * @param len Length of the frame data.
 */
static void fifo_store_raw_packet(const void *frame, uint16_t len) {
    if (frame == NULL) return;

    uint8_t next_write_idx = next_fifo_idx(fifo_write_idx);

    // If FIFO full (next write position would be current read position),
    // then advance the read index, effectively dropping the oldest packet.
    if (next_write_idx == fifo_read_idx) {
        fifo_read_idx = next_fifo_idx(fifo_read_idx);
        // Optional: Log FIFO overflow
        // printf("XBee Actions FIFO Overflow: Dropping oldest packet.\r\n");
    }

    // Ensure length does not exceed buffer capacity (RX_MAX_LEN from xbee_actions.h)
    raw_packet_fifo[fifo_write_idx].len = (len > RX_MAX_LEN) ? RX_MAX_LEN : len;
    memcpy((void*)raw_packet_fifo[fifo_write_idx].buf, // Discard volatile for memcpy destination
           frame,
           raw_packet_fifo[fifo_write_idx].len);

    fifo_write_idx = next_write_idx;
}

/* --------------------------------------------------------------------------
 * 2.  Frame callbacks for the XBee Library
 * -------------------------------------------------------------------------- */

/**
 * @brief Callback for 0x90 (Receive Packet) and 0x91 (Explicit Rx Indicator) frames.
 */
int rx_frame_cb(xbee_dev_t *xbee,
                const void FAR *raw_frame_data, uint16_t length, void FAR *context)
{
    (void)context; // Unused parameter

    if (raw_frame_data == NULL || xbee == NULL) return XBEE_ATCMD_ERROR; // Or some other error indication

    fifo_store_raw_packet(raw_frame_data, length);

    const uint8_t *frame_ptr = (const uint8_t *)raw_frame_data;
    uint8_t  frame_type = frame_ptr[0];

    addr64   sender_ieee_addr;
    uint16_t sender_network_addr_be; // Big-Endian
    uint8_t  receive_options;

    if (frame_type == XBEE_FRAME_RECEIVE) { // 0x90
        if (length < offsetof(xbee_frame_receive_t, payload)) return XBEE_ATCMD_ERROR; // Packet too short
        const xbee_frame_receive_t FAR *rx = (const xbee_frame_receive_t FAR *)raw_frame_data;
        sender_ieee_addr    = rx->ieee_address;
        sender_network_addr_be  = rx->network_address_be;
        receive_options        = rx->options;
    } else if (frame_type == XBEE_FRAME_RECEIVE_EXPLICIT) { // 0x91
        if (length < offsetof(xbee_frame_receive_explicit_t, payload)) return XBEE_ATCMD_ERROR; // Packet too short
        const xbee_frame_receive_explicit_t FAR *rx_explicit = (const xbee_frame_receive_explicit_t FAR *)raw_frame_data;
        sender_ieee_addr    = rx_explicit->ieee_address;
        sender_network_addr_be  = rx_explicit->network_address_be;
        receive_options        = rx_explicit->options;
    } else {
        return 0; // Not a frame type we're auto-acknowledging
    }

    if (receive_options & XBEE_RX_OPT_BROADCAST) {
        return 0;
    }

    static const char ack_payload[] = "ACK";
    xbee_header_transmit_t tx_request_header = {
        .frame_type         = XBEE_FRAME_TRANSMIT,
        .frame_id           = 0, // No XBee transmit status reply expected for this ACK
        .ieee_address       = sender_ieee_addr,
        .network_address_be = sender_network_addr_be,
        .broadcast_radius   = 0,
        .options            = 0
    };

    (void)xbee_frame_write(xbee,
                           &tx_request_header, sizeof(tx_request_header),
                           ack_payload, sizeof(ack_payload) - 1,
                           XBEE_WRITE_FLAG_NONE);

    return 0; // Successfully processed
}


/**
 * @brief Callback for 0x8B (Transmit Status) frames.
 */
int tx_status_cb(xbee_dev_t *xbee,
                 const void FAR *frame_data, // Points to xbee_frame_transmit_status_t
                 uint16_t length,
                 void FAR *context)
{
    (void)xbee;
    (void)length;
    (void)context;

    if (frame_data == NULL) return XBEE_ATCMD_ERROR; // Or other error

    const xbee_frame_transmit_status_t FAR *ts = (const xbee_frame_transmit_status_t FAR *)frame_data;
    // Example: Log or handle transmit status
    // printf("TX Status: FrameID %u, Delivery 0x%02X, Retries %u\r\n",
    //    ts->frame_id, ts->delivery_status, ts->retry_count);

    return 0; // Successfully processed
}

int always_awake (xbee_dev_t *xbee) {
	(void)xbee;
	return 1;
}


/* --------------------------------------------------------------------------
 * 3.  Global dispatch table required by the XBee driver
 * -------------------------------------------------------------------------- */
const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
{
    XBEE_FRAME_HANDLE_LOCAL_AT,
    { XBEE_FRAME_RECEIVE, 0, rx_frame_cb, NULL },
    { XBEE_FRAME_RECEIVE_EXPLICIT, 0, rx_frame_cb, NULL },
    { XBEE_FRAME_TRANSMIT_STATUS, 0, tx_status_cb, NULL },
    { XBEE_FRAME_MODEM_STATUS, 0, xbee_frame_dump_modem_status, NULL },
    XBEE_FRAME_TABLE_END
};
