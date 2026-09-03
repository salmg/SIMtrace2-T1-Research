/* PC/SC card reader backend supporting both T=0 and T=1
 *
 * libosmosim ships a PC/SC backend, but its card_open() bails out on anything
 * other than OSIM_PROTO_T0 and hardcodes SCARD_PROTOCOL_T0 / SCARD_PCI_T0.
 * Cards that only speak T=1 -- which includes a good share of EMV cards --
 * therefore cannot be opened through it at all. This is a self-contained
 * replacement that negotiates T=0 or T=1 and reports back which one is in use.
 *
 * It plugs into the same struct osim_reader_ops abstraction, so everything
 * downstream (osim_card_hdl, channels, rh->ops->transceive) is unchanged.
 *
 * (C) 2026 by the SIMtrace2 contributors
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/msgb.h>
#include <osmocom/sim/sim.h>

#include <wintypes.h>
#include <winscard.h>

#include <osmocom/simtrace2/reader_pcsc.h>

#define PCSC_ERROR(rv, text) \
	if (rv != SCARD_S_SUCCESS) { \
		fprintf(stderr, text ": %s (0x%lX)\n", pcsc_stringify_error(rv), (unsigned long) rv); \
		goto end; \
	}

struct st2_pcsc_state {
	SCARDCONTEXT hContext;
	SCARDHANDLE hCard;
	DWORD dwActiveProtocol;
	const SCARD_IO_REQUEST *pioSendPci;
	SCARD_IO_REQUEST pioRecvPci;
	char *name;
	bool connected;
};

/* translate a mask of enum osim_proto into the SCARD_PROTOCOL_* the PC/SC
 * layer expects for the 'preferred protocols' argument of SCardConnect() */
static DWORD proto_mask_to_scard(uint32_t proto_mask)
{
	DWORD out = 0;

	if (proto_mask & ST2_PROTO_MASK_T0)
		out |= SCARD_PROTOCOL_T0;
	if (proto_mask & ST2_PROTO_MASK_T1)
		out |= SCARD_PROTOCOL_T1;

	return out;
}

/* record which protocol the reader and card settled on, and select the
 * matching PCI header for subsequent SCardTransmit() calls */
static int apply_active_proto(struct st2_pcsc_state *st, struct osim_card_hdl *card)
{
	switch (st->dwActiveProtocol) {
	case SCARD_PROTOCOL_T0:
		st->pioSendPci = SCARD_PCI_T0;
		if (card)
			card->proto = OSIM_PROTO_T0;
		return 0;
	case SCARD_PROTOCOL_T1:
		st->pioSendPci = SCARD_PCI_T1;
		if (card)
			card->proto = OSIM_PROTO_T1;
		return 0;
	default:
		fprintf(stderr, "PC/SC negotiated an unsupported protocol (0x%lX)\n",
			(unsigned long) st->dwActiveProtocol);
		return -ENOTSUP;
	}
}

static int st2_pcsc_get_atr(struct osim_card_hdl *card)
{
	struct osim_reader_hdl *rh = card->reader;
	struct st2_pcsc_state *st = rh->priv;
	char pbReader[MAX_READERNAME];
	DWORD dwReaderLen = sizeof(pbReader);
	DWORD dwAtrLen = sizeof(card->atr);
	DWORD dwState, dwProt;
	long rc;

	rc = SCardStatus(st->hCard, pbReader, &dwReaderLen, &dwState, &dwProt,
			 card->atr, &dwAtrLen);
	PCSC_ERROR(rc, "SCardStatus");
	card->atr_len = dwAtrLen;

	return 0;
end:
	return -EIO;
}

static struct osim_reader_hdl *st2_pcsc_reader_open(int idx, const char *name, void *ctx)
{
	struct osim_reader_hdl *rh;
	struct st2_pcsc_state *st;
	LONG rc;
	LPSTR mszReaders = NULL;
	DWORD dwReaders;
	unsigned int num_readers;
	char *ptr;

	rh = talloc_zero(ctx, struct osim_reader_hdl);
	if (!rh)
		return NULL;
	st = rh->priv = talloc_zero(rh, struct st2_pcsc_state);
	if (!st)
		goto end;

	rh->proto_supported = ST2_PROTO_MASK_ANY;

	rc = SCardEstablishContext(SCARD_SCOPE_SYSTEM, NULL, NULL, &st->hContext);
	PCSC_ERROR(rc, "SCardEstablishContext");

	dwReaders = SCARD_AUTOALLOCATE;
	rc = SCardListReaders(st->hContext, NULL, (LPSTR)&mszReaders, &dwReaders);
	PCSC_ERROR(rc, "SCardListReaders");

	/* the reader list is a doubly NUL-terminated sequence of C strings */
	num_readers = 0;
	ptr = mszReaders;
	while (*ptr != '\0' && num_readers != idx) {
		ptr += strlen(ptr) + 1;
		num_readers++;
	}

	if (num_readers != (unsigned int) idx) {
		fprintf(stderr, "PC/SC reader index %d out of range (%u present)\n",
			idx, num_readers);
		SCardFreeMemory(st->hContext, mszReaders);
		goto end;
	}

	st->name = talloc_strdup(rh, ptr);
	st->dwActiveProtocol = -1;
	SCardFreeMemory(st->hContext, mszReaders);

	return rh;
end:
	talloc_free(rh);
	return NULL;
}

/* the osim_reader_ops entry point; opens with whatever single protocol the
 * caller asked for. osmo_st2_pcsc_card_open() below is the richer variant. */
static struct osim_card_hdl *st2_pcsc_card_open_op(struct osim_reader_hdl *rh,
						   enum osim_proto proto)
{
	return osmo_st2_pcsc_card_open(rh, 1 << proto);
}

static int st2_pcsc_card_reset(struct osim_card_hdl *card, bool cold_reset)
{
	struct st2_pcsc_state *st = card->reader->priv;
	LONG rc;

	/* offer the same set again, so a card may even come back on a
	 * different protocol after a cold reset */
	rc = SCardReconnect(st->hCard, SCARD_SHARE_SHARED,
			    SCARD_PROTOCOL_T0 | SCARD_PROTOCOL_T1,
			    cold_reset ? SCARD_UNPOWER_CARD : SCARD_RESET_CARD,
			    &st->dwActiveProtocol);
	PCSC_ERROR(rc, "SCardReconnect");

	if (apply_active_proto(st, card) < 0)
		return -ENOTSUP;

	/* the ATR may legitimately differ after a reset */
	st2_pcsc_get_atr(card);

	return 0;
end:
	return -EIO;
}

static int st2_pcsc_card_close(struct osim_card_hdl *card)
{
	struct st2_pcsc_state *st = card->reader->priv;
	LONG rc;

	if (!st->connected)
		return 0;

	rc = SCardDisconnect(st->hCard, SCARD_UNPOWER_CARD);
	st->connected = false;
	PCSC_ERROR(rc, "SCardDisconnect");

	return 0;
end:
	return -EIO;
}

static int st2_pcsc_transceive(struct osim_reader_hdl *rh, struct msgb *msg)
{
	struct st2_pcsc_state *st = rh->priv;
	DWORD rlen = msgb_tailroom(msg);
	LONG rc;

	rc = SCardTransmit(st->hCard, st->pioSendPci, msg->data, msgb_length(msg),
			   &st->pioRecvPci, msg->tail, &rlen);
	PCSC_ERROR(rc, "SCardTransmit");

	msgb_put(msg, rlen);
	msgb_apdu_le(msg) = rlen;

	return 0;
end:
	return -EIO;
}

static const struct osim_reader_ops st2_pcsc_reader_ops = {
	.name = "PC/SC (T=0 and T=1)",
	.reader_open = st2_pcsc_reader_open,
	.card_open = st2_pcsc_card_open_op,
	.card_reset = st2_pcsc_card_reset,
	.card_close = st2_pcsc_card_close,
	.transceive = st2_pcsc_transceive,
};

struct osim_reader_hdl *osmo_st2_pcsc_reader_open(int idx, const char *name, void *ctx)
{
	struct osim_reader_hdl *rh = st2_pcsc_reader_open(idx, name, ctx);

	if (rh)
		rh->ops = &st2_pcsc_reader_ops;

	return rh;
}

struct osim_card_hdl *osmo_st2_pcsc_card_open(struct osim_reader_hdl *rh, uint32_t proto_mask)
{
	struct st2_pcsc_state *st = rh->priv;
	struct osim_card_hdl *card;
	struct osim_chan_hdl *chan;
	DWORD dwPreferred;
	LONG rc;

	dwPreferred = proto_mask_to_scard(proto_mask);
	if (!dwPreferred) {
		fprintf(stderr, "no supported protocol in mask 0x%08x\n", proto_mask);
		return NULL;
	}

	rc = SCardConnect(st->hContext, st->name, SCARD_SHARE_SHARED,
			  dwPreferred, &st->hCard, &st->dwActiveProtocol);
	PCSC_ERROR(rc, "SCardConnect");
	st->connected = true;

	card = talloc_zero(rh, struct osim_card_hdl);
	if (!card)
		goto end;
	INIT_LLIST_HEAD(&card->channels);
	INIT_LLIST_HEAD(&card->apps);
	card->reader = rh;
	rh->card = card;

	if (apply_active_proto(st, card) < 0) {
		talloc_free(card);
		rh->card = NULL;
		return NULL;
	}

	/* create a default channel */
	chan = talloc_zero(card, struct osim_chan_hdl);
	if (!chan) {
		talloc_free(card);
		rh->card = NULL;
		return NULL;
	}
	chan->card = card;
	llist_add(&chan->list, &card->channels);

	st2_pcsc_get_atr(card);

	return card;
end:
	return NULL;
}

int osmo_st2_pcsc_active_proto(const struct osim_reader_hdl *rh)
{
	const struct st2_pcsc_state *st;

	if (!rh || !rh->priv)
		return -1;
	st = rh->priv;
	if (!st->connected)
		return -1;

	switch (st->dwActiveProtocol) {
	case SCARD_PROTOCOL_T0:
		return OSIM_PROTO_T0;
	case SCARD_PROTOCOL_T1:
		return OSIM_PROTO_T1;
	default:
		return -1;
	}
}

const char *osmo_st2_pcsc_reader_name(const struct osim_reader_hdl *rh)
{
	const struct st2_pcsc_state *st;

	if (!rh || !rh->priv)
		return NULL;
	st = rh->priv;

	return st->name;
}
