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
 *
 * Enhanced: 2025-05-12
 * - Modified tx_status_cb to interact with global state variables for robust error handling.
 */
#include "stm32/xbee_actions.h" // Defines FRAME_QUEUE_SIZE, rx_packet_t, RX_MAX_LEN, callback prototypes
#include "xbee/device.h"        // For xbee_dev_t, xbee_frame_receive_t, etc.
#include "xbee/byteorder.h"     // For endian conversion utilities, be16toh
#include "xbee/delivery_status.h" // For transmit status constants (XBEE_TX_DELIVERY_SUCCESS, etc.)
#include "xbee/atcmd.h"         // For XBEE_FRAME_HANDLE_LOCAL_AT, xbee_frame_dump_modem_status
#include "xbee/wpan.h"          // For WPAN address types and constants
#include <string.h>             // For memcpy
#include <stdio.h>              // For printf (if used for debugging)
#include <stdbool.h>            // For bool type

// Include main.h or a shared config header if MAX_CONSECUTIVE_XBEE_TX_FAILS is defined there
// For this example, we assume these are global and extern linkage will work.
// If MAX_CONSECUTIVE_XBEE_TX_FAILS is needed for a check *within* this file,
// it must be defined or included. Here, the check happens in main.c.
// #include "main.h" // If MAX_CONSECUTIVE_XBEE_TX_FAILS is defined in main.h

/* --------------------------------------------------------------------------
 * Global state variables (defined in main.c, accessed via extern)
 * -------------------------------------------------------------------------- */
extern volatile uint32_t g_xbee_tx_fail_count;
extern volatile bool     g_xbee_target_unreachable;
extern volatile uint8_t  g_periodic_ack_frame_id; // Frame ID of the ACK we are waiting for status on

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
    if (dst == NULL || !xbee_rx_packet_available()) {
        return 0;
    }

    uint8_t current_read_idx = fifo_read_idx;

    dst->len = raw_packet_fifo[current_read_idx].len;
    memcpy(dst->buf,
           (const void *)raw_packet_fifo[current_read_idx].buf,
           dst->len);

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
    if (next_write_idx == fifo_read_idx) {
        fifo_read_idx = next_fifo_idx(fifo_read_idx);
        // printf("WARN: XBee Actions Raw Packet FIFO Overflow! Dropping oldest packet.\r\n");
    }

    raw_packet_fifo[fifo_write_idx].len = (len > RX_MAX_LEN) ? RX_MAX_LEN : len;
    memcpy((void*)raw_packet_fifo[fifo_write_idx].buf,
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
    (void)context;
    if (raw_frame_data == NULL || xbee == NULL) return XBEE_ATCMD_ERROR;

    fifo_store_raw_packet(raw_frame_data, length);
    // The original auto-ACK logic has been removed as requested by focusing on TX status.
    // If auto-ACKs are desired for specific incoming messages, that logic would go here.
    return 0;
}


/**
 * @brief Callback for 0x8B (Transmit Status) frames.
 */
int tx_status_cb(xbee_dev_t *xbee,
                 const void FAR *frame_data,
                 uint16_t length,
                 void FAR *context)
{
    (void)xbee;
    (void)length;
    (void)context;

    if (frame_data == NULL) return XBEE_ATCMD_ERROR;

    const xbee_frame_transmit_status_t FAR *ts = (const xbee_frame_transmit_status_t FAR *)frame_data;

    // Check if this status is for our periodic ACK
    if (ts->frame_id == g_periodic_ack_frame_id && g_periodic_ack_frame_id != 0) {
        if (ts->delivery == XBEE_TX_DELIVERY_SUCCESS) {
            // // printf("TX SUCCESS for periodic ACK (Frame ID %u) to 0x%04X\r\n", ts->frame_id, be16toh(ts->network_address_be));
            g_xbee_tx_fail_count = 0; // Reset on successful ACK delivery
            if (g_xbee_target_unreachable) {
                // // printf("Target was unreachable, now marked reachable due to successful ACK.\r\n");
            }
            g_xbee_target_unreachable = false; // Mark as reachable again
        } else {
            // // printf("TX FAIL for periodic ACK (Frame ID %u) to 0x%04X, DeliveryStatus 0x%02X, Discovery 0x%02X, Retries %u\r\n",
            // //       ts->frame_id, be16toh(ts->network_address_be), ts->delivery_status, ts->discovery_status, ts->retry_count);
            g_xbee_tx_fail_count++;
            // The check for MAX_CONSECUTIVE_XBEE_TX_FAILS and setting g_xbee_target_unreachable
            // is handled in main.c's main loop, as g_xbee_tx_fail_count is global.
        }
        g_periodic_ack_frame_id = 0; // This ACK status has been processed, clear the ID
                                     // so main.c knows it can send a new one.
        // g_ack_pending_timestamp in main.c will naturally stop being checked once ID is 0.
    } else if (ts->frame_id != 0) {
        // This is a TX status for some other frame (not our periodic ACK, or ACK ID was 0 when status arrived)
        // You might want to handle this if other parts of your application send frames
        // with specific IDs and need to track their status.
        // For debugging general TX status:
        // if (ts->delivery_status == XBEE_TX_DELIVERY_SUCCESS) {
        //     // printf("TX SUCCESS: Other FrameID %u, RemoteAddr16 0x%04X, Retries %u, Discovery %u\r\n",
        //     //       ts->frame_id, be16toh(ts->network_address_be), ts->retry_count, ts->discovery_status);
        // } else {
        //     // printf("TX FAIL: Other FrameID %u, RemoteAddr16 0x%04X, DeliveryStatus 0x%02X, DiscoveryStatus 0x%02X, Retries %u\r\n",
        //     //       ts->frame_id, be16toh(ts->network_address_be), ts->delivery_status, ts->discovery_status, ts->retry_count);
        // }
    }
    return 0;
}

// This function is for power management (sleep/wake); returning 1 means always awake.
// It needs to be defined as it's used in main.c: xbee_dev_init(&xbee, xbee_platform_serial(), always_awake, NULL);
int always_awake (xbee_dev_t *xbee_device_ptr) { // Changed param name to avoid conflict if xbee is global
	(void)xbee_device_ptr; // Unused parameter
	return 1; // XBEE_AWAKE_ALWAYS or equivalent. 1 generally means awake.
}


/* --------------------------------------------------------------------------
 * 3.  Global dispatch table required by the XBee driver
 * -------------------------------------------------------------------------- */
const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
{
    XBEE_FRAME_HANDLE_LOCAL_AT,
    { XBEE_FRAME_RECEIVE, 0, rx_frame_cb, NULL },
    { XBEE_FRAME_RECEIVE_EXPLICIT, 0, rx_frame_cb, NULL },
    { XBEE_FRAME_TRANSMIT_STATUS, 0, tx_status_cb, NULL }, // Our critical callback
    { XBEE_FRAME_MODEM_STATUS, 0, xbee_frame_dump_modem_status, NULL }, // Good for debugging modem state changes
    XBEE_FRAME_TABLE_END
};

