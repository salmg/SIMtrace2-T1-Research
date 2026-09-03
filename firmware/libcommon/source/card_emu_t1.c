/* ISO 7816-3 T=1 block protocol, card side
 *
 * Framing, error detection, sequence numbers, chaining and S-block handling.
 * Deliberately free of hardware and USB dependencies so it can be unit-tested
 * on the host; card_emu.c drives it byte by byte.
 *
 * (C) 2026 by the SIMtrace2 contributors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <string.h>

#include "card_emu_t1.h"

/* PCB encoding, ISO 7816-3 11.3.2. Bit numbering below is the standard's,
 * i.e. b8 is the most significant bit. */
#define PCB_IS_I(p)		(((p) & 0x80) == 0x00)
#define PCB_IS_R(p)		(((p) & 0xC0) == 0x80)

/* I-block: b8=0, b7=N(S), b6=M (more data), b5..b1=0 */
#define PCB_I_NS(p)		(((p) >> 6) & 1)
#define PCB_I_MORE(p)		(((p) >> 5) & 1)
#define PCB_I(ns, more)		((uint8_t)((((ns) & 1) << 6) | ((more) ? 0x20 : 0)))

/* R-block: b8..b7=10, b6=0, b5=N(R), b4..b1=error code */
#define PCB_R_NR(p)		(((p) >> 4) & 1)
#define PCB_R(nr, err)		((uint8_t)(0x80 | (((nr) & 1) << 4) | ((err) & 0x0f)))
#define R_ERR_NONE		0
#define R_ERR_EDC		1
#define R_ERR_OTHER		2

/* S-block: b8..b7=11, b6=0 for a request and 1 for a response, b5..b1=type */
#define PCB_S_IS_RESP(p)	(((p) & 0x20) != 0)
#define PCB_S_TYPE(p)		((p) & 0x1f)
#define PCB_S(type, resp)	((uint8_t)(0xC0 | ((resp) ? 0x20 : 0) | ((type) & 0x1f)))
#define S_TYPE_RESYNCH		0
#define S_TYPE_IFS		1
#define S_TYPE_ABORT		2
#define S_TYPE_WTX		3

/* NAD we put into the blocks we originate. 00 means no node addressing, which
 * is what essentially every card uses. */
#define T1_NAD_CARD_TO_READER	0x00

/* LEN=FF is reserved by ISO 7816-3 */
#define T1_LEN_RESERVED		0xff

uint8_t t1_append_edc(enum t1_edc edc, uint8_t *buf, uint16_t len)
{
	uint16_t i;

	if (edc == T1_EDC_CRC) {
		/* CRC-16 with polynomial x^16+x^12+x^5+1 and a seed of FFFF.
		 * Cards selecting CRC over LRC are rare; this path has not been
		 * validated against hardware. */
		uint16_t crc = 0xffff;

		for (i = 0; i < len; i++) {
			uint8_t bit;

			crc ^= (uint16_t) buf[i] << 8;
			for (bit = 0; bit < 8; bit++) {
				if (crc & 0x8000)
					crc = (crc << 1) ^ 0x1021;
				else
					crc <<= 1;
			}
		}
		buf[len] = crc >> 8;
		buf[len + 1] = crc & 0xff;
		return 2;
	}

	/* LRC, the default: the XOR over the whole block including the EDC
	 * byte must come out as zero */
	buf[len] = 0;
	for (i = 0; i < len; i++)
		buf[len] ^= buf[i];

	return 1;
}

void t1_init(struct t1_state *st, enum t1_edc edc, uint8_t ifsc)
{
	memset(st, 0, sizeof(*st));

	st->edc = edc;
	st->rx_edc_len = (edc == T1_EDC_CRC) ? 2 : 1;
	st->ifsc = ifsc ? ifsc : T1_DEFAULT_IFS;
	/* until the reader tells us otherwise it accepts only the default */
	st->ifsd = T1_DEFAULT_IFS;
	st->rx_state = T1_RX_NAD;
	/* both sides start their sequence numbering at zero */
	st->ns = 0;
	st->nr = 0;
}

/* assemble a block into the transmit buffer, appending the EDC */
static void queue_block(struct t1_state *st, uint8_t pcb, const uint8_t *inf, uint8_t len)
{
	uint16_t i = 0;

	st->tx_blk[i++] = T1_NAD_CARD_TO_READER;
	st->tx_blk[i++] = pcb;
	st->tx_blk[i++] = len;
	if (len) {
		memcpy(&st->tx_blk[i], inf, len);
		i += len;
	}
	i += t1_append_edc(st->edc, st->tx_blk, i);

	st->tx_len = i;
	st->tx_idx = 0;
	st->stats.tx_blocks++;
}

/* recompute the EDC over the received block and compare.
 * The bytes are compared by hand rather than with memcmp(): the firmware's
 * minimal libc provides memcpy/memset/strlen but no memcmp, and there are only
 * ever one or two EDC bytes to check. */
static bool edc_ok(const struct t1_state *st)
{
	uint8_t tmp[T1_MAX_BLOCK];
	uint16_t body = 3 + st->rx_inf_len;
	uint8_t n, i;

	memcpy(tmp, st->rx_blk, body);
	n = t1_append_edc(st->edc, tmp, body);

	for (i = 0; i < n; i++) {
		if (tmp[body + i] != st->rx_blk[body + i])
			return false;
	}

	return true;
}

static uint32_t handle_i_block(struct t1_state *st, uint8_t pcb,
			       const uint8_t *inf, uint8_t len)
{
	uint32_t ev;

	if (PCB_I_NS(pcb) != st->nr) {
		/* not the block we were expecting; ask for a retransmission */
		st->stats.seq_errors++;
		queue_block(st, PCB_R(st->nr, R_ERR_OTHER), NULL, 0);
		return T1_EV_ERROR | T1_EV_TX_READY;
	}

	memcpy(st->rx_inf, inf, len);
	st->rx_inf_avail = len;
	st->rx_more = PCB_I_MORE(pcb);

	/* the reader's next I-block carries the other sequence number */
	st->nr ^= 1;

	ev = T1_EV_RX_DATA;

	if (st->rx_more) {
		/* the command is chained: acknowledge so the reader continues */
		queue_block(st, PCB_R(st->nr, R_ERR_NONE), NULL, 0);
		ev |= T1_EV_TX_READY;
	} else {
		ev |= T1_EV_CMD_COMPLETE;
	}

	return ev;
}

static uint32_t handle_r_block(struct t1_state *st, uint8_t pcb)
{
	/* N(R) names the sequence number the reader wants next. Since we
	 * advance ns as soon as a block is queued, an N(R) equal to ns means
	 * our last block arrived and the reader is asking for the following
	 * one; anything else is a request to send the last one again. */
	if (PCB_R_NR(pcb) == st->ns)
		return T1_EV_NEED_NEXT_CHUNK;

	if (!st->last_i_valid)
		return T1_EV_ERROR;

	st->stats.retransmits++;
	memcpy(st->tx_blk, st->last_i_blk, st->last_i_len);
	st->tx_len = st->last_i_len;
	st->tx_idx = 0;
	st->stats.tx_blocks++;

	return T1_EV_TX_READY;
}

static uint32_t handle_s_block(struct t1_state *st, uint8_t pcb,
			       const uint8_t *inf, uint8_t len)
{
	uint8_t type = PCB_S_TYPE(pcb);

	/* the only response we ever solicit is the one to our WTX request,
	 * and it needs no action beyond letting the caller carry on */
	if (PCB_S_IS_RESP(pcb))
		return 0;

	switch (type) {
	case S_TYPE_IFS:
		/* the reader announces the largest INF it will accept */
		if (len != 1 || inf[0] == 0 || inf[0] == T1_LEN_RESERVED) {
			queue_block(st, PCB_R(st->nr, R_ERR_OTHER), NULL, 0);
			return T1_EV_ERROR | T1_EV_TX_READY;
		}
		st->ifsd = inf[0];
		/* an IFS response echoes the accepted value */
		queue_block(st, PCB_S(S_TYPE_IFS, 1), inf, 1);
		return T1_EV_TX_READY;

	case S_TYPE_ABORT:
		queue_block(st, PCB_S(S_TYPE_ABORT, 1), NULL, 0);
		return T1_EV_TX_READY;

	case S_TYPE_RESYNCH:
		/* drop everything and restart the sequence numbering */
		st->ns = 0;
		st->nr = 0;
		st->last_i_valid = false;
		st->stats.resynchs++;
		queue_block(st, PCB_S(S_TYPE_RESYNCH, 1), NULL, 0);
		return T1_EV_RESYNCH | T1_EV_TX_READY;

	default:
		queue_block(st, PCB_R(st->nr, R_ERR_OTHER), NULL, 0);
		return T1_EV_ERROR | T1_EV_TX_READY;
	}
}

static uint32_t handle_block(struct t1_state *st)
{
	uint8_t pcb = st->rx_blk[1];
	uint8_t len = st->rx_blk[2];
	const uint8_t *inf = &st->rx_blk[3];

	st->stats.rx_blocks++;

	if (!edc_ok(st)) {
		st->stats.edc_errors++;
		queue_block(st, PCB_R(st->nr, R_ERR_EDC), NULL, 0);
		return T1_EV_ERROR | T1_EV_TX_READY;
	}

	if (PCB_IS_I(pcb))
		return handle_i_block(st, pcb, inf, len);
	if (PCB_IS_R(pcb))
		return handle_r_block(st, pcb);

	return handle_s_block(st, pcb, inf, len);
}

uint32_t t1_rx_byte(struct t1_state *st, uint8_t byte)
{
	switch (st->rx_state) {
	case T1_RX_NAD:
		st->rx_idx = 0;
		st->rx_blk[st->rx_idx++] = byte;
		st->rx_state = T1_RX_PCB;
		return 0;

	case T1_RX_PCB:
		st->rx_blk[st->rx_idx++] = byte;
		st->rx_state = T1_RX_LEN;
		return 0;

	case T1_RX_LEN:
		st->rx_blk[st->rx_idx++] = byte;
		st->rx_inf_len = byte;
		if (byte == T1_LEN_RESERVED) {
			st->rx_state = T1_RX_NAD;
			queue_block(st, PCB_R(st->nr, R_ERR_OTHER), NULL, 0);
			return T1_EV_ERROR | T1_EV_TX_READY;
		}
		st->rx_state = st->rx_inf_len ? T1_RX_INF : T1_RX_EDC;
		return 0;

	case T1_RX_INF:
		st->rx_blk[st->rx_idx++] = byte;
		if (st->rx_idx >= 3u + st->rx_inf_len)
			st->rx_state = T1_RX_EDC;
		return 0;

	case T1_RX_EDC:
		st->rx_blk[st->rx_idx++] = byte;
		if (st->rx_idx < 3u + st->rx_inf_len + st->rx_edc_len)
			return 0;
		st->rx_state = T1_RX_NAD;
		return handle_block(st);
	}

	return 0;
}

int t1_tx_inf(struct t1_state *st, const uint8_t *data, uint8_t len, bool more)
{
	/* never send more in one block than the reader said it accepts */
	if (len > st->ifsd)
		return -1;

	queue_block(st, PCB_I(st->ns, more), data, len);

	/* hold on to it in case the reader asks for a retransmission */
	memcpy(st->last_i_blk, st->tx_blk, st->tx_len);
	st->last_i_len = st->tx_len;
	st->last_i_valid = true;

	/* our next I-block carries the other sequence number */
	st->ns ^= 1;

	return 0;
}

int t1_tx_wtx_request(struct t1_state *st, uint8_t bwt_multiplier)
{
	if (!bwt_multiplier)
		return -1;

	queue_block(st, PCB_S(S_TYPE_WTX, 0), &bwt_multiplier, 1);

	return 0;
}

int t1_get_tx_byte(struct t1_state *st, uint8_t *byte)
{
	if (st->tx_idx >= st->tx_len)
		return 0;

	*byte = st->tx_blk[st->tx_idx++];

	if (st->tx_idx >= st->tx_len) {
		st->tx_len = 0;
		st->tx_idx = 0;
	}

	return 1;
}

bool t1_tx_pending(const struct t1_state *st)
{
	return st->tx_idx < st->tx_len;
}
