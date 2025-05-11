/*
 * xbee_actions.c
 *
 * 2025‑05‑10  – adds support for plain 0x90 “Receive Packet” frames and a
 *               FIFO so higher‑level code can pull them out whenever it likes.
 */
#include "stm32/xbee_actions.h"
#include "xbee/byteorder.h"
#include "xbee/delivery_status.h"
#include "xbee/atcmd.h"
#include "xbee/wpan.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * 1.  Tiny circular buffer to keep the last RX_FIFO_DEPTH packets
 * -------------------------------------------------------------------------- */
static volatile rx_packet_t fifo[RX_FIFO_DEPTH];
static volatile uint8_t     wr_idx = 0;         /* writer position           */
static volatile uint8_t     rd_idx = 0;         /* reader position           */

static inline uint8_t next_idx(uint8_t i) { return (uint8_t)((i + 1u) % RX_FIFO_DEPTH); }

bool xbee_rx_available(void)
{
    return rd_idx != wr_idx;
}

uint16_t xbee_rx_dequeue(rx_packet_t *dst)
{
    if (!xbee_rx_available()) return 0;

    uint8_t i = rd_idx;
    rd_idx    = next_idx(rd_idx);

    dst->len = fifo[i].len;
    memcpy(dst->buf,
           (const void *)fifo[i].buf,   /* discard volatile once, explicitly */
           dst->len);
    return dst->len;
}

static void fifo_store(const void *frame, uint16_t len)
{
    uint8_t nxt = next_idx(wr_idx);

    /* If FIFO full, drop the oldest packet (reader catches up).            */
    if (nxt == rd_idx) rd_idx = next_idx(rd_idx);

    fifo[wr_idx].len = len > RX_MAX_LEN ? RX_MAX_LEN : len;
    memcpy((void*)fifo[wr_idx].buf, frame, fifo[wr_idx].len);
    wr_idx = nxt;
}

/* --------------------------------------------------------------------------
 * 2.  Frame callbacks
 * -------------------------------------------------------------------------- */

/* Handles BOTH 0x90 (Receive Packet) and 0x91 (Receive Explicit) frames.     */
//int rx_frame_cb(xbee_dev_t *xbee,
//                const void FAR *frame,
//                uint16_t length,
//                void FAR *context)
//{
//    /* just stash a copy – parsing/decoding can be done later by the app     */
//    fifo_store(frame, length);
//
//    (void)xbee; (void)context;                 /* unused for now             */
//    return 0;
//}

/* … keep all the earlier #includes and FIFO code … */

/* --------------------------------------------------------------------------
 * 2.  Frame callback (now with auto‑ACK)
 * -------------------------------------------------------------------------- */
int rx_frame_cb(xbee_dev_t *xbee,
                const void FAR *raw, uint16_t len, void FAR *ctx)
{
    /* 1)  Stash a copy for the application -------------------------------- */
    fifo_store(raw, len);

    /* 2)  Peek at header to learn who sent it ------------------------------ */
    const uint8_t *frame = (const uint8_t *)raw;
    uint8_t  ftype = frame[0];

    /* Common fields we need for a reply */
    addr64   sender_ieee;
    uint16_t sender_net_be;
    uint8_t  rx_opts;

    if (ftype == XBEE_FRAME_RECEIVE)                         /* 0x90 */
    {
        const xbee_frame_receive_t FAR *rx = (const void FAR *)raw;
        sender_ieee    = rx->ieee_address;
        sender_net_be  = rx->network_address_be;
        rx_opts        = rx->options;
    }
    else                                                     /* 0x91 */
    {
        const xbee_frame_receive_explicit_t FAR *rx = (const void FAR *)raw;
        sender_ieee    = rx->ieee_address;
        sender_net_be  = rx->network_address_be;
        rx_opts        = rx->options;
    }

    /* Skip broadcasts ------------------------------------------------------ */
    if (rx_opts & 0x02)                 /* option bit 1 == broadcast */
        return 0;                       /* nothing else to do        */

    /* 3)  Build a one‑hop unicast “ACK” packet back to sender -------------- */
    static const char ack[] = "ACK";

    xbee_header_transmit_t tx = {
        .frame_type         = XBEE_FRAME_TRANSMIT,   /* 0x10 */
        .frame_id           = 0,                     /* 0 => no status reply   */
        .ieee_address       = sender_ieee,
        .network_address_be = sender_net_be,         /* already big‑endian     */
        .broadcast_radius   = 0,                     /* use NH parameter       */
        .options            = 0                      /* normal transmit        */
    };

    /* Fire it off (ignore return value for now) */
    (void)xbee_frame_write(xbee,
                           &tx, sizeof tx,
                           ack, sizeof ack - 1,
                           XBEE_WRITE_FLAG_NONE);

    (void)ctx;          /* unused */

    return 0;
}


/* 0x8B Transmit‑Status ------------------------------------------------------ */
int tx_status_cb(xbee_dev_t *xbee,
                 const void FAR *frame,
                 uint16_t length,
                 void FAR *context)
{
    const xbee_frame_transmit_status_t FAR *ts = frame;

    /* You might want to inspect ts->delivery and ts->retry_count here.      */
    (void)xbee; (void)length; (void)context; (void)ts;
    return 0;
}

/* --------------------------------------------------------------------------
 * 3.  Global dispatch table required by the driver
 * -------------------------------------------------------------------------- */
const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
{
    /* Local AT responses (library helper)                                   */
    XBEE_FRAME_HANDLE_LOCAL_AT,

    /* 0x90: transparent‑serial / standard Transmit Request frames           */
    { XBEE_FRAME_RECEIVE,           0, rx_frame_cb, NULL },

    /* 0x91: explicit RX frames                                              */
    { XBEE_FRAME_RECEIVE_EXPLICIT,  0, rx_frame_cb, NULL },

    /* 0x8B: transmit‑status frames                                          */
    { XBEE_FRAME_TRANSMIT_STATUS,   0, tx_status_cb, NULL },

    /* 0x8A: modem‑status – dump to console with helper                      */
    { XBEE_FRAME_MODEM_STATUS,      0, xbee_frame_dump_modem_status, NULL },

    XBEE_FRAME_TABLE_END            /* **must** be last                      */
};
