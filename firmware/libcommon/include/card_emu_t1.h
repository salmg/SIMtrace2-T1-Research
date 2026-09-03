/* ISO 7816-3 T=1 block protocol, card side
 *
 * This is the block layer only: framing, error detection, sequence numbers,
 * chaining and S-block handling. It has no hardware dependencies and no
 * knowledge of USB, so it can be unit-tested on the host.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/*! largest INF field ISO 7816-3 allows */
#define T1_MAX_IFS		254
/*! IFSC/IFSD default before any S(IFS) negotiation (ISO 7816-3 11.4.2) */
#define T1_DEFAULT_IFS		32
/*! NAD + PCB + LEN + INF + up to two EDC bytes */
#define T1_MAX_BLOCK		(3 + T1_MAX_IFS + 2)

/*! error detection code in use, selected by the ATR's TCi for T=1 */
enum t1_edc {
	T1_EDC_LRC = 0,		/*!< one byte, XOR over the block */
	T1_EDC_CRC = 1,		/*!< two bytes, CRC-16 per ISO/IEC 13239 */
};

/*! events reported by t1_rx_byte(), as a bit-mask */
#define T1_EV_RX_DATA		(1 << 0)	/*!< an I-block INF is available */
#define T1_EV_TX_READY		(1 << 1)	/*!< a block is queued for transmit */
#define T1_EV_CMD_COMPLETE	(1 << 2)	/*!< last I-block of a command chain */
#define T1_EV_NEED_NEXT_CHUNK	(1 << 3)	/*!< reader acked, send next chunk */
#define T1_EV_ERROR		(1 << 4)	/*!< a protocol error was handled */
#define T1_EV_RESYNCH		(1 << 5)	/*!< reader requested a resynch */

/*! receiver sub-state within a block */
enum t1_rx_state {
	T1_RX_NAD,
	T1_RX_PCB,
	T1_RX_LEN,
	T1_RX_INF,
	T1_RX_EDC,
};

struct t1_state {
	enum t1_edc edc;

	/*! largest INF we are willing to send (our own capability) */
	uint8_t ifsc;
	/*! largest INF the reader accepts, learned from S(IFS request) */
	uint8_t ifsd;

	/*! our send-sequence number */
	uint8_t ns;
	/*! sequence number we expect in the reader's next I-block */
	uint8_t nr;

	/* receive side */
	enum t1_rx_state rx_state;
	uint8_t rx_blk[T1_MAX_BLOCK];
	uint16_t rx_idx;		/*!< bytes of rx_blk filled so far */
	uint8_t rx_inf_len;		/*!< LEN of the block being received */
	uint8_t rx_edc_len;		/*!< 1 for LRC, 2 for CRC */

	/*! INF of the most recently accepted I-block */
	uint8_t rx_inf[T1_MAX_IFS];
	uint8_t rx_inf_avail;
	bool rx_more;			/*!< chaining bit of that I-block */

	/* transmit side */
	uint8_t tx_blk[T1_MAX_BLOCK];
	uint16_t tx_len;
	uint16_t tx_idx;

	/*! last I-block we sent, kept so we can honour a retransmit request */
	uint8_t last_i_blk[T1_MAX_BLOCK];
	uint16_t last_i_len;
	bool last_i_valid;

	struct {
		uint32_t rx_blocks;
		uint32_t tx_blocks;
		uint32_t edc_errors;
		uint32_t seq_errors;
		uint32_t retransmits;
		uint32_t resynchs;
	} stats;
};

/*! \brief Initialise the T=1 state, as after a card reset */
void t1_init(struct t1_state *st, enum t1_edc edc, uint8_t ifsc);

/*! \brief Feed one byte received from the reader
 *  \returns bit-mask of T1_EV_* describing what the caller should do */
uint32_t t1_rx_byte(struct t1_state *st, uint8_t byte);

/*! \brief Queue one chunk of response data as a single I-block
 *  \param[in] more true if further chunks follow, setting the chaining bit
 *  \returns 0 on success, negative if len exceeds what the reader accepts */
int t1_tx_inf(struct t1_state *st, const uint8_t *data, uint8_t len, bool more);

/*! \brief Queue an S(WTX request) asking the reader for more time
 *  \param[in] bwt_multiplier how many BWTs we need, 1..255 */
int t1_tx_wtx_request(struct t1_state *st, uint8_t bwt_multiplier);

/*! \brief Pull the next byte to transmit
 *  \returns 1 if *byte was filled in, 0 if nothing is pending */
int t1_get_tx_byte(struct t1_state *st, uint8_t *byte);

/*! \brief Is there a block waiting to go out? */
bool t1_tx_pending(const struct t1_state *st);

/*! \brief Compute the EDC over len bytes of buf, appending it to buf
 *  \returns number of EDC bytes appended */
uint8_t t1_append_edc(enum t1_edc edc, uint8_t *buf, uint16_t len);
