/*
 * xbee_actions.h
 *
 * 2025‑05‑10  – adds a tiny packet‑FIFO so the app can pull out any 0x90 or
 * 0x91 frames (regular “Transmit Request” and “Explicit Rx”) that the radio
 * hears.
 */
#ifndef XBEE_ACTIONS_H_
#define XBEE_ACTIONS_H_

#include "xbee/device.h"
#include "xbee/atcmd.h"
#include <stdint.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * 1.  Small lock‑less ring to store the last N received RF packets
 * -------------------------------------------------------------------------- */
#define RX_FIFO_DEPTH   256                      /* tune to your RAM budget   */
#define RX_MAX_LEN      (XBEE_MAX_RFPAYLOAD+18) /* worst‑case frame length   */

typedef struct {
    uint16_t len;                               /* number of valid bytes     */
    uint8_t  buf[RX_MAX_LEN];                   /* raw frame (starts at 0x90/0x91) */
} rx_packet_t;

bool     xbee_rx_available(void);              /* FIFO not empty ?          */
uint16_t xbee_rx_dequeue(rx_packet_t *pkt);     /* copy next packet, returns length */

/* --------------------------------------------------------------------------
 * 2.  Frame‑handler prototypes
 * -------------------------------------------------------------------------- */
int rx_frame_cb(xbee_dev_t *xbee,
                const void FAR *raw, uint16_t len, void FAR *ctx);

int tx_status_cb(xbee_dev_t *xbee,
                 const void FAR *raw, uint16_t len, void FAR *ctx);

/* --------------------------------------------------------------------------
 * 3.  Global dispatch table the driver looks for
 * -------------------------------------------------------------------------- */
extern const xbee_dispatch_table_entry_t xbee_frame_handlers[];

#endif /* XBEE_ACTIONS_H_ */
