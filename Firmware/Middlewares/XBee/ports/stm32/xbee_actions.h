/*
 * xbee_actions.h
 *
 * 2025‑05‑10  – adds a tiny packet‑FIFO so the app can pull out any 0x90 or
 * 0x91 frames (regular “Transmit Request” and “Explicit Rx”) that the radio
 * hears.
 *
 * Refactored: 2025-05-11
 * - Removed unused RxFrame_t and frame_q.
 * - Clarified RX_MAX_LEN usage.
 */
#ifndef XBEE_ACTIONS_H_
#define XBEE_ACTIONS_H_

#include "xbee/device.h"    // For xbee_dev_t, addr64, XBEE_MAX_RFPAYLOAD
#include "xbee/atcmd.h"     // For xbee_dispatch_table_entry_t
#include <stdint.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * 1.  Small lock‑less ring to store the last N received RF packets
 * -------------------------------------------------------------------------- */
#ifndef FRAME_QUEUE_SIZE
#define FRAME_QUEUE_SIZE   32                      /* tune to your RAM budget   */
#endif

// RX_MAX_LEN: Defines the maximum size of a raw XBee API frame.
// An XBee API frame includes:
// - API Frame Type (1 byte)
// - Frame-specific header (variable, e.g., ~11 bytes for 0x90, ~17 for 0x91 without RSSI)
// - RF Payload (up to XBEE_MAX_RFPAYLOAD, typically 100 bytes for 802.15.4, can be larger for other XBee types)
// A generous buffer is XBEE_MAX_RFPAYLOAD + ~18 bytes for header data.
// (e.g. 0x91: Type(1) + IEEE(8) + NWK(2) + SrcEP(1) + DstEP(1) + Clust(2) + Prof(2) + Opts(1) = 18 bytes)
#ifndef RX_MAX_LEN
#define RX_MAX_LEN      (XBEE_MAX_RFPAYLOAD + 20) /* Max RF payload + generous header allowance */
#endif


typedef struct {
    uint16_t len;                               /* number of valid bytes in buf */
    uint8_t  buf[RX_MAX_LEN];                   /* raw frame data (e.g., starts at 0x90/0x91 for RX frames) */
} rx_packet_t;

/**
 * @brief Checks if there are any raw XBee packets available in the low-level FIFO.
 * @return True if packets are available, false otherwise.
 */
bool     xbee_rx_packet_available(void);

/**
 * @brief Dequeues the next raw XBee packet from the low-level FIFO.
 * @param pkt Pointer to an rx_packet_t structure to copy the packet into.
 * @return The length of the dequeued packet in bytes, or 0 if the FIFO was empty.
 */
uint16_t xbee_rx_packet_dequeue(rx_packet_t *pkt);

/* --------------------------------------------------------------------------
 * 2.  Frame‑handler prototypes (callbacks for the XBee library)
 * -------------------------------------------------------------------------- */

/**
 * @brief Callback function invoked by the XBee library for received RF data frames.
 * Handles 0x90 (Receive Packet) and 0x91 (Explicit Rx Indicator) frames.
 * Stores the raw frame into a FIFO for later processing by the application.
 *
 * @param xbee Pointer to the XBee device structure.
 * @param raw Pointer to the raw frame data.
 * @param len Length of the raw frame data.
 * @param ctx User-defined context (not used in this implementation).
 * @return 0 on success.
 */
int rx_frame_cb(xbee_dev_t *xbee,
                const void FAR *raw, uint16_t len, void FAR *ctx);

/**
 * @brief Callback function invoked by the XBee library for transmit status frames (0x8B).
 *
 * @param xbee Pointer to the XBee device structure.
 * @param raw Pointer to the raw frame data (xbee_frame_transmit_status_t).
 * @param len Length of the raw frame data.
 * @param ctx User-defined context (not used in this implementation).
 * @return 0 on success.
 */
int tx_status_cb(xbee_dev_t *xbee,
                 const void FAR *raw, uint16_t len, void FAR *ctx);

int always_awake (xbee_dev_t *xbee);

/* --------------------------------------------------------------------------
 * 3.  Global dispatch table the XBee driver looks for
 * -------------------------------------------------------------------------- */
extern const xbee_dispatch_table_entry_t xbee_frame_handlers[];

#endif /* XBEE_ACTIONS_H_ */
