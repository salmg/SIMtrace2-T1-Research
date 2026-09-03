/* simtrace2-remsim - main program for the host PC to provide a remote SIM
 * using the SIMtrace 2 firmware in card emulation mode
 *
 * (C) 2016-2017 by Harald Welte <hwelte@hmw-consulting.de>
 * (C) 2018, sysmocom -s.f.m.c. GmbH, Author: Kevin Redon <kredon@sysmocom.de>
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#define _GNU_SOURCE
#include <getopt.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <libusb.h>

#include <osmocom/usb/libusb.h>
#include <osmocom/simtrace2/simtrace2_api.h>
#include <osmocom/simtrace2/simtrace_prot.h>
#include <osmocom/simtrace2/apdu_dispatch.h>
#include <osmocom/simtrace2/gsmtap.h>
#include <osmocom/simtrace2/reader_pcsc.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/application.h>
#include <osmocom/sim/class_tables.h>
#include <osmocom/sim/sim.h>

/* Minimal osmocore log config — no named categories beyond the built-in DLGLOBAL.
 * Initialising the framework routes LOGP() calls in apdu_dispatch.c to stderr
 * instead of silently dropping them. */
static const struct log_info_cat _log_categories[] = {};
static const struct log_info app_log_info = {
	.cat     = _log_categories,
	.num_cat = ARRAY_SIZE(_log_categories),
};

/* The simplest possible ATR: TS + T0 with neither interface nor historical
 * bytes, which implicitly offers T=0 only and needs no TCK. */
static const uint8_t synthetic_atr[] = { 0x3B, 0x00 };

/* Auto-reinit state: seconds of idle after last SW before reinit (0=disabled) */
static int g_auto_reinit_delay = 0;
static int g_skip_atr_reinit = 0;
static time_t g_last_sw_time = 0;

/* ATR pushed to the card emulation on every (re-)initialisation */
static uint8_t g_atr[OSIM_MAX_ATR_LEN];
static unsigned int g_atr_len;
/* protocols that ATR offers the reader, one bit per (1 << T) */
static uint32_t g_atr_proto_mask;
/* forward the real card's ATR byte for byte, warts and all */
static int g_raw_atr = 0;

/* last card interface status seen, so a once-a-second poll only reports changes */
static uint32_t g_last_status_flags;
static bool g_have_last_status;

/* which protocol(s) we are willing to use towards the real card */
static uint32_t g_card_proto_mask = ST2_PROTO_MASK_ANY;

/*! \brief Parse the --card-proto argument into a protocol bit-mask */
static int parse_card_proto(const char *arg, uint32_t *mask)
{
	if (!strcasecmp(arg, "auto") || !strcasecmp(arg, "any"))
		*mask = ST2_PROTO_MASK_ANY;
	else if (!strcasecmp(arg, "t0") || !strcmp(arg, "0"))
		*mask = ST2_PROTO_MASK_T0;
	else if (!strcasecmp(arg, "t1") || !strcmp(arg, "1"))
		*mask = ST2_PROTO_MASK_T1;
	else
		return -1;

	return 0;
}

/*! \brief Parse an ATR as per ISO 7816-3 section 8
 *  \param[in] atr ATR bytes, starting at TS
 *  \param[in] len number of bytes available
 *  \param[out] proto_mask one bit per offered protocol, i.e. (1 << T)
 *  \param[out] tck_ok cleared if a TCK is required but does not check out
 *  \returns number of bytes consumed, or -1 if the ATR is malformed/truncated */
static int atr_parse(const uint8_t *atr, unsigned int len, uint32_t *proto_mask, bool *tck_ok)
{
	unsigned int i = 0, j;
	bool have_tck = false;
	uint8_t y, k, csum;

	*proto_mask = 0;
	*tck_ok = true;

	if (len < 2)
		return -1;

	/* TS: only direct (3b) and inverse (3f) convention exist */
	if (atr[i] != 0x3b && atr[i] != 0x3f)
		return -1;
	i++;

	/* T0: Y1 in the high nibble, count of historical bytes in the low one */
	y = atr[i] >> 4;
	k = atr[i] & 0x0f;
	i++;

	/* interface bytes TAi/TBi/TCi/TDi, each present only if flagged in Yi */
	while (1) {
		if (y & 0x1) {				/* TAi */
			if (i >= len)
				return -1;
			i++;
		}
		if (y & 0x2) {				/* TBi */
			if (i >= len)
				return -1;
			i++;
		}
		if (y & 0x4) {				/* TCi */
			if (i >= len)
				return -1;
			i++;
		}
		if (!(y & 0x8))				/* no TDi -> last round */
			break;
		if (i >= len)
			return -1;
		*proto_mask |= 1 << (atr[i] & 0x0f);
		if ((atr[i] & 0x0f) != 0)
			have_tck = true;
		y = atr[i] >> 4;
		i++;
	}

	/* historical bytes */
	if (i + k > len)
		return -1;
	i += k;

	/* TCK is present iff some TDi announced a protocol other than T=0 */
	if (have_tck) {
		if (i >= len)
			return -1;
		/* the XOR over T0..TCK inclusive must come out as zero */
		for (csum = 0, j = 1; j < i; j++)
			csum ^= atr[j];
		if (csum != atr[i])
			*tck_ok = false;
		i++;
	}

	/* an ATR carrying no TDi at all offers T=0 only */
	if (*proto_mask == 0)
		*proto_mask = 1 << 0;

	return i;
}

/*! \brief Rebuild an ATR without any TA/TB/TC interface bytes.
 *
 *  The card emulation implements exactly one set of transmission parameters:
 *  the ISO 7816-3 defaults, Fd=372/Dd=1 with no extra guard time, changed only
 *  by a successful PPS exchange. card_emu.c walks TA1/TB1/TC1/TA2 solely to
 *  locate TC2 for the waiting integer and never acts on their values.
 *
 *  Forwarding them anyway tells the reader to do things the board will not:
 *
 *    TA1  Fi/Di, e.g. 0x13 asks for 93 clocks per ETU instead of 372
 *    TC1  extra guard time between characters
 *    TA2  specific mode, where the reader adopts TA1 right after the ATR
 *         with no PPS at all (ISO 7816-3 section 8.3)
 *
 *  Any of those leaves the two sides clocking differently and the reader sees
 *  framing errors rather than a card. So keep what identifies the card, TS,
 *  the historical bytes and the protocol indication in the TDi chain, and drop
 *  the timing bytes so both sides stay on the defaults the board implements.
 *
 *  \param[out] out rebuilt ATR, at most as long as the input
 *  \returns length of the rebuilt ATR, or -1 if the input does not parse */
static int atr_strip_interface_bytes(const uint8_t *in, unsigned int len, uint8_t *out)
{
	uint8_t td_proto[8];
	unsigned int n_td = 0, i, o = 0, t, j, group = 1;
	bool have_tck = false, offers_t1 = false;
	int prev_proto = -1;
	uint8_t y, k, csum, ifsc = 0;
	/* Interface bytes following a TDi that names T=15 are GLOBAL parameters,
	 * not electrical ones: TA carries the clock-stop indicator and the supply
	 * voltage classes the card accepts, TB the power consumption. Dropping
	 * them leaves the emulated card declaring no voltage class at all, which
	 * a reader running its interface at 1.8 V has no reason to accept. -1
	 * means the byte was absent. */
	int global_ta = -1, global_tb = -1;

	if (len < 2)
		return -1;
	if (in[0] != 0x3b && in[0] != 0x3f)
		return -1;

	y = in[1] >> 4;
	k = in[1] & 0x0f;
	i = 2;

	/* collect the protocol nibble of each TDi, stepping over TA/TB/TC */
	while (1) {
		if (y & 0x1) {
			if (i >= len)
				return -1;
			/* From i=3 on the interface bytes belong to the protocol
			 * named by TD(i-1), so this TAi is the card's IFSC. */
			if (group >= 3 && prev_proto == 1)
				ifsc = in[i];
			else if (group >= 3 && prev_proto == 15)
				global_ta = in[i];
			i++;
		}
		if (y & 0x2) {
			if (i >= len)
				return -1;
			if (group >= 3 && prev_proto == 15)
				global_tb = in[i];
			i++;
		}
		if (y & 0x4) {
			if (i >= len)
				return -1;
			i++;
		}
		if (!(y & 0x8))
			break;
		if (i >= len)
			return -1;
		/* Keep each protocol once. A card repeats a TDi naming the same
		 * protocol purely to hang more interface bytes off it; the chain
		 * emitted below is rebuilt from scratch, so a repeat here carries
		 * no information of its own. */
		{
			uint8_t p = in[i] & 0x0f;
			unsigned int seen;

			for (seen = 0; seen < n_td; seen++) {
				if (td_proto[seen] == p)
					break;
			}
			if (seen == n_td && n_td < ARRAY_SIZE(td_proto))
				td_proto[n_td++] = p;
			if (p != 0)
				have_tck = true;
			if (p == 1)
				offers_t1 = true;
		}
		prev_proto = in[i] & 0x0f;
		y = in[i] >> 4;
		i++;
		group++;
	}

	/* i now indexes the historical bytes */
	if (i + k > len)
		return -1;

	/* TA3/TB3 have to be kept when the ATR indicates T=1. They are protocol
	 * parameters rather than electrical ones: TA3 is the IFSC and TB3 the
	 * BWI/CWI pair. EMV Book 1 8.3 requires TB3 and rejects the card unless
	 * BWI <= 4 and CWI <= 5, whereas an absent TB3 means the ISO default
	 * CWI=13 -- so stripping it makes an EMV terminal reset and give up.
	 *
	 * They must follow a TDi that names T=1, and interface bytes are
	 * protocol-specific only from the third group on, so the T=1 link has
	 * to be TD2 or later. Extend the chain when it would not be. */
	if (offers_t1) {
		if (td_proto[n_td - 1] != 1 && n_td < ARRAY_SIZE(td_proto))
			td_proto[n_td++] = 1;
		if (n_td < 2)
			td_proto[n_td++] = 1;
	}

	out[o++] = in[0];					/* TS */
	out[o++] = (uint8_t) ((n_td ? 0x80 : 0x00) | k);	/* T0 */
	for (t = 0; t < n_td; t++) {
		bool last = (t + 1 == n_td);
		uint8_t next = last ? (offers_t1 ? 0x30 : 0x00) : 0x80;

		out[o++] = (uint8_t) (next | td_proto[t]);
		if (td_proto[t] == 15 && (global_ta != -1 || global_tb != -1)) {
			/* Re-emit the global bytes, and say so in the Y nibble of
			 * the TDi we just wrote: they are only found because a TDi
			 * announced them. */
			uint8_t yb = 0;

			if (global_ta != -1)
				yb |= 0x10;
			if (global_tb != -1)
				yb |= 0x20;
			out[o - 1] = (uint8_t) ((out[o - 1] & 0x0f) | yb |
						(last ? 0x00 : 0x80));
			if (global_ta != -1)
				out[o++] = (uint8_t) global_ta;
			if (global_tb != -1)
				out[o++] = (uint8_t) global_tb;
		}
		if (last && offers_t1) {
			/* IFSC: keep the card's if it is in the range ISO allows,
			 * else advertise the largest block the firmware buffers. */
			out[o++] = (ifsc >= 0x10 && ifsc <= 0xfe) ? ifsc : 0xfe;
			/* BWI=4, CWI=5: the most generous pair EMV accepts, which
			 * is what the relay wants given it adds a USB and PC/SC
			 * round trip to every exchange. */
			out[o++] = 0x45;
		}
	}
	memcpy(&out[o], &in[i], k);
	o += k;

	/* a TCK is required iff some TDi named a protocol other than T=0 */
	if (have_tck) {
		for (csum = 0, j = 1; j < o; j++)
			csum ^= out[j];
		out[o++] = csum;
	}

	return o;
}

/*! \brief Install the ATR that subsequent (re-)initialisations will push.
 *  Falls back to the synthetic T=0 ATR when the supplied one is unusable. */
static void set_atr_to_push(const uint8_t *atr, unsigned int len, const char *source)
{
	uint32_t proto_mask = 0;
	bool tck_ok = true;
	int rc;

	if (!atr || !len || len > sizeof(g_atr)) {
		if (len > sizeof(g_atr)) {
			fprintf(stderr, "%s ATR is %u bytes, exceeds the %zu byte limit "
				"of the card emulation; using synthetic ATR\n",
				source, len, sizeof(g_atr));
		}
		memcpy(g_atr, synthetic_atr, sizeof(synthetic_atr));
		g_atr_len = sizeof(synthetic_atr);
		source = "synthetic";
	} else {
		memcpy(g_atr, atr, len);
		g_atr_len = len;
	}

	printf("Using %s ATR: %s\n", source, osmo_hexdump(g_atr, g_atr_len));

	rc = atr_parse(g_atr, g_atr_len, &proto_mask, &tck_ok);
	if (rc < 0) {
		fprintf(stderr, "  warning: ATR does not parse as ISO 7816-3; "
			"forwarding it verbatim anyway\n");
		return;
	}
	if ((unsigned int) rc != g_atr_len) {
		fprintf(stderr, "  warning: ATR parses to %d bytes but %u were given\n",
			rc, g_atr_len);
	}
	if (!tck_ok)
		fprintf(stderr, "  warning: ATR checksum (TCK) is wrong\n");

	printf("  offers protocol(s):");
	for (unsigned int t = 0; t < 16; t++) {
		if (proto_mask & (1 << t))
			printf(" T=%u", t);
	}
	printf("\n");

	g_atr_proto_mask = proto_mask;

	/* The card emulation implements T=0 and T=1. Announcing anything beyond
	 * those makes the reader propose a protocol the board has to decline
	 * during PPS, costing a reset cycle at best. */
	if (proto_mask & ~(uint32_t) ((1 << 0) | (1 << 1))) {
		fprintf(stderr, "  warning: this ATR offers a protocol beyond T=0 and T=1, "
			"which the card emulation does not implement\n");
	}

	/* Drop the transmission parameters the board cannot follow. See
	 * atr_strip_interface_bytes(). */
	if (!g_raw_atr) {
		uint8_t stripped[OSIM_MAX_ATR_LEN];

		rc = atr_strip_interface_bytes(g_atr, g_atr_len, stripped);
		if (rc < 0) {
			fprintf(stderr, "  warning: could not rewrite the ATR; "
				"forwarding it unchanged\n");
		} else if ((unsigned int) rc != g_atr_len) {
			memcpy(g_atr, stripped, rc);
			g_atr_len = rc;
			printf("  dropped the clock rate and guard time interface bytes, "
			       "which the card emulation does not implement\n");
			if (proto_mask & (1 << 1)) {
				printf("  kept TA3/TB3 (IFSC, BWI/CWI): EMV terminals "
				       "reject a T=1 card whose ATR omits them\n");
			}
			printf("  ATR now: %s\n", osmo_hexdump(g_atr, g_atr_len));
			if (atr_parse(g_atr, g_atr_len, &proto_mask, &tck_ok) < 0 || !tck_ok)
				fprintf(stderr, "  warning: rewritten ATR does not check out\n");
			g_atr_proto_mask = proto_mask;
		}
	}
}

/*! \brief Push the configured ATR to the card emulation */
static void push_atr(struct osmo_st2_cardem_inst *ci)
{
	if (g_atr_len)
		osmo_st2_cardem_request_set_atr(ci, g_atr, g_atr_len);
}

/***********************************************************************
 * Incoming Messages
 ***********************************************************************/

/*! \brief Process a STATUS message from the SIMtrace2 */
static int process_do_status(struct osmo_st2_cardem_inst *ci, uint8_t *buf, int len)
{
	struct cardemu_usb_msg_status *status;
	status = (struct cardemu_usb_msg_status *) buf;

	/* The status is polled once a second, so only say something when it
	 * actually changes; otherwise the log is unreadable. */
	if (g_have_last_status && status->flags == g_last_status_flags)
		return 0;

	g_last_status_flags = status->flags;
	g_have_last_status = true;

	printf("=> STATUS: flags=0x%x [%s%s%s%s%s], fi=%u, di=%u, wi=%u wtime=%u\n",
		status->flags,
		status->flags & CEMU_STATUS_F_VCC_PRESENT  ? "VCC "     : "",
		status->flags & CEMU_STATUS_F_CLK_ACTIVE   ? "CLK "     : "",
		status->flags & CEMU_STATUS_F_RESET_ACTIVE ? "RST "     : "",
		status->flags & CEMU_STATUS_F_CARD_INSERT  ? "CARD_IN " : "",
		status->flags & CEMU_STATUS_F_RCEMU_ACTIVE ? "RCEMU "   : "",
		status->fi, status->di, status->wi,
		status->waiting_time);

	/* VCC and CLK are supplied by the reader. If they never show up the
	 * reader is not activating the card, and nothing else can happen. */
	if (!(status->flags & CEMU_STATUS_F_VCC_PRESENT))
		printf("   note: reader is not supplying VCC to the emulated card\n");
	else if (!(status->flags & CEMU_STATUS_F_CLK_ACTIVE))
		printf("   note: VCC present but no clock from the reader\n");

	return 0;
}

/*! \brief Process a PTS indication message from the SIMtrace2 */
static int process_do_pts(struct osmo_st2_cardem_inst *ci, uint8_t *buf, int len)
{
	struct cardemu_usb_msg_pts_info *pts;
	pts = (struct cardemu_usb_msg_pts_info *) buf;

	printf("=> PTS req: %s\n", osmo_hexdump(pts->req, sizeof(pts->req)));

	return 0;
}

/* Largest T=1 command APDU we will reassemble out of chained I-blocks */
#define T1_HOST_MAX_APDU	4096
/* Chunk size for responses. The card emulation refuses any block larger than
 * the IFSD the reader negotiated, and until it negotiates one that is the ISO
 * 7816-3 default of 32, so stay there rather than risk a rejected block. */
#define T1_HOST_CHUNK		32

static uint8_t g_t1_cmd[T1_HOST_MAX_APDU];
static unsigned int g_t1_cmd_len;

/* Response data the reader has not asked for yet, held until it issues GET
 * RESPONSE. See the 61xx handling in process_do_rx_da(). */
static uint8_t g_pending_rsp[256];
static unsigned int g_pending_rsp_len;

/*! \brief Process the INF of one T=1 I-block handed up by the card emulation */
static int process_rx_t1(struct osmo_st2_cardem_inst *ci,
			 const struct cardemu_usb_msg_rx_data *data)
{
	struct osim_reader_hdl *rh = ci->chan->card->reader;
	unsigned int off, resp_len;
	const uint8_t *resp;
	struct msgb *tmsg;
	int rc;

	if (g_t1_cmd_len + data->data_len > sizeof(g_t1_cmd)) {
		fprintf(stderr, "T=1 command APDU exceeds %zu bytes, dropping\n",
			sizeof(g_t1_cmd));
		g_t1_cmd_len = 0;
		return -1;
	}

	memcpy(g_t1_cmd + g_t1_cmd_len, data->data, data->data_len);
	g_t1_cmd_len += data->data_len;

	/* a chained command is only complete once its final block arrives */
	if (!(data->flags & CEMU_DATA_F_FINAL))
		return 0;

	printf("=> T=1 C-APDU (%u): %s\n", g_t1_cmd_len,
	       osmo_hexdump(g_t1_cmd, g_t1_cmd_len));

	tmsg = msgb_alloc(T1_HOST_MAX_APDU, "T1-APDU");
	if (!tmsg) {
		g_t1_cmd_len = 0;
		return -ENOMEM;
	}
	memcpy(msgb_put(tmsg, g_t1_cmd_len), g_t1_cmd, g_t1_cmd_len);
	g_t1_cmd_len = 0;

	/* whatever the card answers lands after l3h */
	tmsg->l3h = tmsg->tail;
	rc = rh->ops->transceive(rh, tmsg);
	if (rc < 0) {
		fprintf(stderr, "error during transceive: %d\n", rc);
		msgb_free(tmsg);
		return rc;
	}

	/* Unlike T=0 there are no procedure bytes to interleave: the response
	 * comes back whole, status word included, and goes straight back out. */
	resp = tmsg->l3h;
	resp_len = msgb_l3len(tmsg);
	printf("<= T=1 R-APDU (%u): %s\n", resp_len, osmo_hexdump(resp, resp_len));

	if (!resp_len) {
		fprintf(stderr, "T=1 card returned an empty response\n");
		msgb_free(tmsg);
		return -1;
	}

	for (off = 0; off < resp_len; off += T1_HOST_CHUNK) {
		unsigned int chunk = resp_len - off;

		if (chunk > T1_HOST_CHUNK)
			chunk = T1_HOST_CHUNK;

		osmo_st2_cardem_request_t1_tx(ci, resp + off, chunk,
					      off + chunk >= resp_len);
	}

	msgb_free(tmsg);

	if (g_auto_reinit_delay > 0)
		g_last_sw_time = time(NULL);

	return 0;
}

/*! \brief Process a RX-DATA indication message from the SIMtrace2 */
static int process_do_rx_da(struct osmo_st2_cardem_inst *ci, uint8_t *buf, int len)
{
	static struct osmo_apdu_context ac;
	struct cardemu_usb_msg_rx_data *data;
	int rc;

	data = (struct cardemu_usb_msg_rx_data *) buf;

	/* T=1 carries whole APDU fragments rather than TPDU pieces, so it needs
	 * none of the procedure-byte handling below */
	if (data->flags & CEMU_DATA_F_T1_BLOCK)
		return process_rx_t1(ci, data);

	printf("=> DATA: flags=%x, %s\n", data->flags,
		osmo_hexdump(data->data, data->data_len));

	rc = osmo_apdu_segment_in(&ac, data->data, data->data_len,
				  data->flags & CEMU_DATA_F_TPDU_HDR);

	if (rc & APDU_ACT_TX_CAPDU_TO_CARD) {
		struct msgb *tmsg;
		struct osim_reader_hdl *rh = ci->chan->card->reader;
		unsigned int rsp_len;
		uint8_t *cur;

		/* GET RESPONSE for data we are already holding. The card has
		 * nothing pending -- it handed us everything at once -- so
		 * answer from the buffer rather than forwarding. */
		if (ac.hdr.ins == 0xC0 && g_pending_rsp_len) {
			unsigned int n = ac.le.tot;
			uint8_t sw[2];

			if (!n || n > g_pending_rsp_len)
				n = g_pending_rsp_len;

			printf("<= GET RESPONSE: %u of %u held byte(s)\n",
			       n, g_pending_rsp_len);
			osmo_st2_cardem_request_pb_and_tx(ci, ac.hdr.ins, g_pending_rsp, n);

			g_pending_rsp_len -= n;
			memmove(g_pending_rsp, g_pending_rsp + n, g_pending_rsp_len);

			/* announce any remainder the same way a card would */
			if (g_pending_rsp_len) {
				sw[0] = 0x61;
				sw[1] = g_pending_rsp_len > 255 ? 0 : g_pending_rsp_len;
			} else {
				sw[0] = 0x90;
				sw[1] = 0x00;
			}
			osmo_st2_cardem_request_sw_tx(ci, sw);
			if (g_auto_reinit_delay > 0)
				g_last_sw_time = time(NULL);
			return 0;
		}

		/* any other command supersedes whatever was still held */
		g_pending_rsp_len = 0;

		tmsg = msgb_alloc(1024, "TPDU");

		/* Copy TPDU header */
		cur = msgb_put(tmsg, sizeof(ac.hdr));
		memcpy(cur, &ac.hdr, sizeof(ac.hdr));
		/* Copy D(c), if any */
		if (ac.lc.tot) {
			cur = msgb_put(tmsg, ac.lc.tot);
			memcpy(cur, ac.dc, ac.lc.tot);
		}
		/* send to actual card */
		tmsg->l3h = tmsg->tail;
		rc = rh->ops->transceive(rh, tmsg);
		if (rc < 0) {
			fprintf(stderr, "error during transceive: %d\n", rc);
			msgb_free(tmsg);
			return rc;
		}
		msgb_apdu_sw(tmsg) = msgb_get_u16(tmsg);
		ac.sw[0] = msgb_apdu_sw(tmsg) >> 8;
		ac.sw[1] = msgb_apdu_sw(tmsg) & 0xff;
		printf("SW=0x%04x, len_rx=%d\n", msgb_apdu_sw(tmsg), msgb_l3len(tmsg));

		rsp_len = msgb_l3len(tmsg);

		if (rsp_len && !ac.le.tot) {
			/* The reader asked for no data, so in T=0 it expects a
			 * status word only and will not accept a body here. A
			 * T=0 card would answer 61xx and wait for GET RESPONSE;
			 * a T=1 card has no procedure bytes and hands over the
			 * data straight away, which is what PC/SC gives us. Hold
			 * it and announce it the way the reader expects. */
			uint8_t sw[2];

			if (rsp_len > sizeof(g_pending_rsp))
				rsp_len = sizeof(g_pending_rsp);
			memcpy(g_pending_rsp, tmsg->l3h, rsp_len);
			g_pending_rsp_len = rsp_len;

			sw[0] = 0x61;
			sw[1] = rsp_len > 255 ? 0 : rsp_len;
			printf("<= holding %u byte(s) for GET RESPONSE, answering %02x %02x\n",
			       rsp_len, sw[0], sw[1]);
			osmo_st2_cardem_request_sw_tx(ci, sw);
		} else {
			if (rsp_len)
				osmo_st2_cardem_request_pb_and_tx(ci, ac.hdr.ins, tmsg->l3h, rsp_len);
			osmo_st2_cardem_request_sw_tx(ci, ac.sw);
		}
		if (g_auto_reinit_delay > 0)
			g_last_sw_time = time(NULL);
	} else if (ac.lc.tot > ac.lc.cur) {
		osmo_st2_cardem_request_pb_and_rx(ci, ac.hdr.ins, ac.lc.tot - ac.lc.cur);
	}
	return 0;
}

/*! \brief Process an incoming message from the SIMtrace2 */
static int process_usb_msg(struct osmo_st2_cardem_inst *ci, uint8_t *buf, int len)
{
	struct simtrace_msg_hdr *sh = (struct simtrace_msg_hdr *)buf;
	int rc;

	if (osmo_st2_verbose)
		printf("-> %s\n", osmo_hexdump(buf, len));

	buf += sizeof(*sh);

	switch (sh->msg_type) {
	case SIMTRACE_MSGT_BD_CEMU_STATUS:
		rc = process_do_status(ci, buf, len);
		break;
	case SIMTRACE_MSGT_DO_CEMU_PTS:
		rc = process_do_pts(ci, buf, len);
		break;
	case SIMTRACE_MSGT_DO_CEMU_RX_DATA:
		rc = process_do_rx_da(ci, buf, len);
		break;
	case SIMTRACE_MSGT_BD_CEMU_CONFIG: {
		struct cardemu_usb_msg_config *cfg = (struct cardemu_usb_msg_config *) buf;
		printf("=> CONFIG: features=0x%08x\n", cfg->features);
		rc = 0;
		break;
	}
	default:
		printf("unknown simtrace msg type 0x%02x\n", sh->msg_type);
		rc = -1;
		break;
	}

	return rc;
}

static void print_welcome(void)
{
	printf("simtrace2-remsim - Remote SIM card forwarding\n"
	       "(C) 2010-2017, Harald Welte <laforge@gnumonks.org>\n"
	       "(C) 2018, sysmocom -s.f.m.c. GmbH, Author: Kevin Redon <kredon@sysmocom.de>\n\n");
}

static void print_help(void)
{
	printf( "\t-r\t--remote-udp-host HOST\n"
		"\t-p\t--remote-udp-port PORT\n"
		"\t-h\t--help\n"
		"\t-i\t--gsmtap-ip\tA.B.C.D\n"
		"\t-a\t--skip-atr\n"
		"\t-y\t--synthetic-atr\t(push a minimal T=0 ATR instead of the real card's)\n"
		"\t\t--raw-atr\t(forward the real ATR verbatim, timing bytes and all)\n"
		"\t-v\t--verbose\t(hexdump every message to and from the device)\n"
		"\t-k\t--keep-running\n"
		"\t-R\t--auto-reinit\tSECONDS (reinit board after SECONDS idle post-SW; 0=off)\n"
		"\t-n\t--pcsc-reader-num\n"
		"\t-T\t--card-proto\tt0|t1|auto (protocol towards the real card, default auto)\n"
		"\t-V\t--usb-vendor\tVENDOR_ID\n"
		"\t-P\t--usb-product\tPRODUCT_ID\n"
		"\t-C\t--usb-config\tCONFIG_ID\n"
		"\t-I\t--usb-interface\tINTERFACE_ID\n"
		"\t-S\t--usb-altsetting ALTSETTING_ID\n"
		"\t-A\t--usb-address\tADDRESS\n"
		"\t-H\t--usb-path\tPATH\n"
		"\n"
		);
}

static const struct option opts[] = {
	{ "remote-udp-host", 1, 0, 'r' },
	{ "remote-udp-port", 1, 0, 'p' },
	{ "gsmtap-ip", 1, 0, 'i' },
	{ "skip-atr", 0, 0, 'a' },
	{ "synthetic-atr", 0, 0, 'y' },
	{ "raw-atr", 0, 0, 0x100 },
	{ "help", 0, 0, 'h' },
	{ "verbose", 0, 0, 'v' },
	{ "keep-running", 0, 0, 'k' },
	{ "auto-reinit", 1, 0, 'R' },
	{ "pcsc-reader-num", 1, 0, 'n' },
	{ "card-proto", 1, 0, 'T' },
	{ "usb-vendor", 1, 0, 'V' },
	{ "usb-product", 1, 0, 'P' },
	{ "usb-config", 1, 0, 'C' },
	{ "usb-interface", 1, 0, 'I' },
	{ "usb-altsetting", 1, 0, 'S' },
	{ "usb-address", 1, 0, 'A' },
	{ "usb-path", 1, 0, 'H' },
	{ NULL, 0, 0, 0 }
};

static void reinit_card_session(struct osmo_st2_cardem_inst *ci)
{
	printf("Auto-reinit: simulating card removal\n");
	osmo_st2_cardem_request_card_insert(ci, false);
	usleep(500000);

	printf("Auto-reinit: re-inserting card\n");
	osmo_st2_cardem_request_card_insert(ci, true);
	osmo_st2_modem_sim_select_remote(ci->slot);

	if (!g_skip_atr_reinit)
		push_atr(ci);

	osmo_st2_modem_reset_pulse(ci->slot, 300);
	g_last_sw_time = 0;
	printf("Auto-reinit: done\n");
}

static void run_mainloop(struct osmo_st2_cardem_inst *ci)
{
	struct osmo_st2_transport *transp = ci->slot->transp;
	unsigned int msg_count, byte_count = 0;
	uint8_t buf[16*265];
	time_t last_status_poll = 0;
	int xfer_len;
	int rc;

	printf("Entering main loop\n");

	while (1) {
		xfer_len = 0;

		/* Poll the card interface state once a second. The firmware's
		 * CEMU_FEAT_F_STATUS_IRQ reports are edge-triggered, so without
		 * this a reader that never powers the card looks exactly like a
		 * reader whose bytes are not arriving. process_do_status() only
		 * prints when something actually changed. */
		if (transp->udp_fd >= 0 || time(NULL) != last_status_poll) {
			last_status_poll = time(NULL);
			if (transp->udp_fd < 0)
				osmo_st2_cardem_request_status(ci);
		}

		/* Drain the interrupt endpoint first. Once the firmware has been
		 * asked for CEMU_FEAT_F_STATUS_IRQ it reports card interface
		 * state changes there, which is the only way to see whether the
		 * reader is powering and clocking the emulated card at all. */
		if (transp->udp_fd < 0 && transp->usb_ep.irq_in) {
			uint8_t ibuf[64];
			int ilen = 0;

			rc = libusb_interrupt_transfer(transp->usb_devh, transp->usb_ep.irq_in,
						       ibuf, sizeof(ibuf), &ilen, 10);
			if (rc == 0 && ilen > 0) {
				printf("IRQ: %s\n", osmo_hexdump(ibuf, ilen));
				process_usb_msg(ci, ibuf, ilen);
			}
		}

		/* read data from SIMtrace2 device (local or via USB) */
		if (transp->udp_fd < 0) {
			rc = libusb_bulk_transfer(transp->usb_devh, transp->usb_ep.in,
						  buf, sizeof(buf), &xfer_len, 100);
			if (rc < 0 && rc != LIBUSB_ERROR_TIMEOUT &&
				      rc != LIBUSB_ERROR_INTERRUPTED &&
				      rc != LIBUSB_ERROR_IO) {
				fprintf(stderr, "BULK IN transfer error; rc=%d\n", rc);
				return;
			}
		} else {
			rc = read(transp->udp_fd, buf, sizeof(buf));
			if (rc <= 0) {
				fprintf(stderr, "short read from UDP\n");
				return;
			}
			xfer_len = rc;
		}
		/* dispatch any incoming data */
		if (xfer_len > 0) {
			if (osmo_st2_verbose)
				printf("URB: %s\n", osmo_hexdump(buf, xfer_len));
			process_usb_msg(ci, buf, xfer_len);
			msg_count++;
			byte_count += xfer_len;
		}

		/* auto-reinit: if idle for g_auto_reinit_delay seconds after last SW */
		if (g_auto_reinit_delay > 0 && g_last_sw_time > 0) {
			if (time(NULL) - g_last_sw_time >= g_auto_reinit_delay)
				reinit_card_session(ci);
		}
	}
}

static struct osmo_st2_transport _transp;

static struct osmo_st2_slot _slot = {
	.transp = &_transp,
	.slot_nr = 0,
};

struct osmo_st2_cardem_inst _ci = {
	.slot = &_slot,
};

struct osmo_st2_cardem_inst *ci = &_ci;

static void signal_handler(int signal)
{
	switch (signal) {
	case SIGINT:
		osmo_st2_cardem_request_card_insert(ci, false);
		exit(0);
		break;
	default:
		break;
	}
}

int main(int argc, char **argv)
{
	struct osmo_st2_transport *transp = ci->slot->transp;
	char *gsmtap_host = "127.0.0.1";
	int rc;
	int c, ret = 1;
	int skip_atr = 0;
	int synthetic_atr_only = 0;
	int card_proto;
	int keep_running = 0;
	int remote_udp_port = 52342;
	int if_num = 0, vendor_id = -1, product_id = -1;
	int config_id = -1, altsetting = 0, addr = -1;
	int reader_num = 0;
	char *remote_udp_host = NULL;
	char *path = NULL;
	struct osim_reader_hdl *reader;
	struct osim_card_hdl *card;

	/* Make stdout unbuffered so every printf() reaches the reader
	 * immediately, whether stdout is a PTY, pipe, or socket. */
	setvbuf(stdout, NULL, _IONBF, 0);

	/* Route LOGP() calls (used by apdu_dispatch.c) to stderr so they
	 * are not silently dropped when no log target is configured. */
	osmo_init_logging2(NULL, &app_log_info);
	struct log_target *stderr_target = log_target_create_stderr();
	log_add_target(stderr_target);
	log_set_all_filter(stderr_target, 1);

	print_welcome();

	while (1) {
		int option_index = 0;

		c = getopt_long(argc, argv, "r:p:hi:V:P:C:I:S:A:H:T:akyvR:n:", opts, &option_index);
		if (c == -1)
			break;
		switch (c) {
		case 'r':
			remote_udp_host = optarg;
			break;
		case 'p':
			remote_udp_port = atoi(optarg);
			break;
		case 'h':
			print_help();
			exit(0);
			break;
		case 'i':
			gsmtap_host = optarg;
			break;
		case 'a':
			skip_atr = 1;
			break;
		case 'y':
			synthetic_atr_only = 1;
			break;
		case 0x100:
			g_raw_atr = 1;
			break;
		case 'v':
			osmo_st2_verbose = 1;
			break;
		case 'k':
			keep_running = 1;
			break;
		case 'R':
			g_auto_reinit_delay = atoi(optarg);
			break;
		case 'n':
			reader_num = atoi(optarg);
			break;
		case 'T':
			if (parse_card_proto(optarg, &g_card_proto_mask) < 0) {
				fprintf(stderr, "invalid --card-proto '%s', "
					"expected t0, t1 or auto\n", optarg);
				goto do_exit;
			}
			break;
		case 'V':
			vendor_id = strtol(optarg, NULL, 16);
			break;
		case 'P':
			product_id = strtol(optarg, NULL, 16);
			break;
		case 'C':
			config_id = atoi(optarg);
			break;
		case 'I':
			if_num = atoi(optarg);
			break;
		case 'S':
			altsetting = atoi(optarg);
			break;
		case 'A':
			addr = atoi(optarg);
			break;
		case 'H':
			path = optarg;
			break;
		}
	}

	if (!remote_udp_host && (vendor_id < 0 || product_id < 0)) {
		fprintf(stderr, "You have to specify the vendor and product ID\n");
		goto do_exit;
	}

	transp->udp_fd = -1;

	ci->card_prof = &osim_uicc_sim_cic_profile;

	if (!remote_udp_host) {
		rc = libusb_init(NULL);
		if (rc < 0) {
			fprintf(stderr, "libusb initialization failed\n");
			goto do_exit;
		}
	} else {
		transp->udp_fd = osmo_sock_init(AF_INET, SOCK_DGRAM, IPPROTO_UDP,
						remote_udp_host, remote_udp_port+if_num,
						OSMO_SOCK_F_CONNECT);
		if (transp->udp_fd < 0) {
			fprintf(stderr, "error binding UDP port\n");
			goto do_exit;
		}
	}

	rc = osmo_st2_gsmtap_init(gsmtap_host);
	if (rc < 0) {
		perror("unable to open GSMTAP");
		goto close_exit;
	}

	/* Use our own PC/SC backend rather than osim_reader_open(): libosmosim's
	 * refuses anything but T=0, which locks out T=1-only cards entirely. */
	reader = osmo_st2_pcsc_reader_open(reader_num, "", NULL);
	if (!reader) {
		fprintf(stderr, "unable to open PC/SC reader %d\n", reader_num);
		goto close_exit;
	}

	card = osmo_st2_pcsc_card_open(reader, g_card_proto_mask);
	if (!card) {
		fprintf(stderr, "unable to open card on reader '%s'\n",
			osmo_st2_pcsc_reader_name(reader));
		goto close_exit;
	}

	card_proto = osmo_st2_pcsc_active_proto(reader);
	printf("Real card on reader '%s', negotiated T=%d\n",
	       osmo_st2_pcsc_reader_name(reader), card_proto);

	ci->chan = llist_entry(card->channels.next, struct osim_chan_hdl, list);
	if (!ci->chan) {
		perror("SIM card has no channel?!?");
		goto close_exit;
	}

	signal(SIGINT, &signal_handler);
	g_skip_atr_reinit = skip_atr;

	/* Present the reader with the ATR of the card we are actually relaying,
	 * rather than a synthetic one that shares none of its characteristics. */
	if (synthetic_atr_only)
		set_atr_to_push(NULL, 0, "synthetic");
	else
		set_atr_to_push(card->atr, card->atr_len, "real card");

	/* The emulated card follows whatever the ATR we push offers, so with the
	 * real card's ATR forwarded both sides normally settle on the same
	 * protocol. They only diverge when the reader side is pinned to T=0, for
	 * instance with --synthetic-atr, and then case 4 differs: a T=1 card
	 * returns the response data directly where a T=0 card would answer 61xx
	 * and wait for GET RESPONSE. */
	if (card_proto == OSIM_PROTO_T1 && !(g_atr_proto_mask & (1 << 1))) {
		fprintf(stderr, "note: the real card runs T=1 but the ATR presented to the "
			"reader offers T=0 only; case 4 APDUs may need GET RESPONSE "
			"adaptation\n");
	}

	do {
		if (transp->udp_fd < 0) {
			struct usb_interface_match _ifm, *ifm = &_ifm;
			ifm->vendor = vendor_id;
			ifm->product = product_id;
			ifm->configuration = config_id;
			ifm->interface = if_num;
			ifm->altsetting = altsetting;
			ifm->addr = addr;
			if (path)
				osmo_strlcpy(ifm->path, path, sizeof(ifm->path));
			transp->usb_devh = osmo_libusb_open_claim_interface(NULL, NULL, ifm);
			if (!transp->usb_devh) {
				fprintf(stderr, "can't open USB device\n");
				goto close_exit;
			}

			rc = libusb_claim_interface(transp->usb_devh, if_num);
			if (rc < 0) {
				fprintf(stderr, "can't claim interface %d; rc=%d\n", if_num, rc);
				goto close_exit;
			}

			rc = osmo_libusb_get_ep_addrs(transp->usb_devh, if_num, &transp->usb_ep.out,
					      &transp->usb_ep.in, &transp->usb_ep.irq_in);
			if (rc < 0) {
				fprintf(stderr, "can't obtain EP addrs; rc=%d\n", rc);
				goto close_exit;
			}
		}

		/* Ask the firmware to report card interface state changes (VCC,
		 * CLK, RST) on the interrupt endpoint. Without this the board
		 * stays silent until the reader sends a whole TPDU header, so
		 * there is no way to tell a reader that never powers the card
		 * from one whose bytes are not getting through. */
		osmo_st2_cardem_request_config(ci, CEMU_FEAT_F_STATUS_IRQ);

		/* simulate card-insert to modem (owhw, not qmod) */
		osmo_st2_cardem_request_card_insert(ci, true);

		/* select remote (forwarded) SIM */
		osmo_st2_modem_sim_select_remote(ci->slot);

		if (!skip_atr)
			push_atr(ci);

		/* select remote (forwarded) SIM */
		osmo_st2_modem_reset_pulse(ci->slot, 300);

		run_mainloop(ci);
		ret = 0;

		if (transp->udp_fd < 0)
			libusb_release_interface(transp->usb_devh, 0);
close_exit:
		if (transp->usb_devh)
			libusb_close(transp->usb_devh);
		if (keep_running)
			sleep(1);
	} while (keep_running);

	if (transp->udp_fd < 0)
		libusb_exit(NULL);
do_exit:
	return ret;
}
