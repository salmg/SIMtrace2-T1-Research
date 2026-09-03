/* apdu_dispatch - State machine to determine Rx/Tx phases of APDU
 *
 * (C) 2016-2019 by Harald Welte <hwelte@hmw-consulting.de>
 *
 * Modified by Salvador Mendoza (salmg.net) to carry EMV proprietary
 * class 0x80 command APDUs, which the upstream state machine rejects.
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/logging.h>
#include <osmocom/sim/sim.h>
#include <osmocom/sim/class_tables.h>

#include <osmocom/simtrace2/apdu_dispatch.h>

/*! \brief Has the command-data phase been completed yet? */
static inline bool is_dc_complete(struct osmo_apdu_context *ac)
{
	return (ac->lc.tot == ac->lc.cur);
}

/*! \brief Has the expected-data phase been completed yet? */
static inline bool is_de_complete(struct osmo_apdu_context *ac)
{
	return (ac->le.tot == ac->le.cur);
}

static const char *stringify_apdu_hdr(const struct osim_apdu_cmd_hdr *h)
{
	static char buf[256];
	sprintf(buf, "CLA=%02x INS=%02x P1=%02x P2=%02x P3=%02x",
		h->cla, h->ins, h->p1, h->p2, h->p3);

	return buf;
}

/*! generate string representation of APDU context in specified output buffer.
 *  \param[in] buf output string buffer provided by caller
 *  \param[in] buf_len size of buf in bytes
 *  \param[in] ac APDU context to dump in buffer
 *  \returns pointer to buf on success */
const char *osmo_apdu_dump_context_buf(char *buf, unsigned int buf_len,
				       const struct osmo_apdu_context *ac)
{
	snprintf(buf, buf_len, "%s; case=%d, lc=%d(%d), le=%d(%d)\n",
		 stringify_apdu_hdr(&ac->hdr), ac->apdu_case,
		 ac->lc.tot, ac->lc.cur,
		 ac->le.tot, ac->le.cur);
	return buf;
}

/*! \brief input function for APDU segmentation
 *  \param ac APDU context across successive calls
 *  \param[in] apdu_buf APDU inpud data buffer
 *  \param[in] apdu_len Length of apdu_buf
 *  \param[in] new_apdu Is this the beginning of a new APDU?
 *
 *  The function returns APDU_ACT_TX_CAPDU_TO_CARD once there is
 *  sufficient data of the APDU received to transmit the command-APDU to
 *  the actual card.
 *
 *  The function returns APDU_ACT_RX_MORE_CAPDU_FROM_READER when there
 *  is more data to be received from the card reader (GSM Phone).
 */
int osmo_apdu_segment_in(struct osmo_apdu_context *ac, const uint8_t *apdu_buf,
			 unsigned int apdu_len, bool new_apdu)
{
	int rc = 0;

	if (new_apdu) {
		/* initialize the apdu context structure */
		memset(ac, 0, sizeof(*ac));
		/* copy APDU header over */
		memcpy(&ac->hdr, apdu_buf, sizeof(ac->hdr));
		ac->apdu_case = osim_determine_apdu_case(&osim_uicc_sim_cic_profile, apdu_buf);
		switch (ac->apdu_case) {
		case 0:
			/* Case 0 is not an ISO 7816-4 case. It is what
			 * osim_determine_apdu_case() returns when the command
			 * matches no entry in the UICC/SIM class table -- which
			 * is every EMV proprietary CLA=0x80 command.
			 *
			 * Upstream falls through to the default arm below, logs
			 * "Unknown APDU case 0" and returns -1, so the APDU is
			 * dropped and the exchange stalls. Relaying an EMV card
			 * requires classifying these ourselves.
			 *
			 * Commands seen on a live terminal, and what P3 means
			 * for each:
			 *
			 *   80 A8 00 00 02 83 00   GET PROCESSING OPTIONS, P3=Lc
			 *   80 CA 9F 36 05         GET DATA (ATC),          P3=Lc
			 *   80 AE 80 00 1D         GENERATE AC (ARQC),      P3=Lc
			 *   80 AE 90 00 2B         GENERATE AC (ARQC+CDA),  P3=Lc
			 */
			if (ac->hdr.cla == 0x80 && ac->hdr.p1 != 0x0) {
				/* GENERATE AC (INS 0xAE) encodes the requested
				 * cryptogram in P1: bits 7-6 select AAC/TC/ARQC
				 * and bit 4 adds CDA. 0x40 and 0x50 are TC and
				 * TC+CDA; 0x80 and 0x90 are ARQC and ARQC+CDA.
				 * All four carry command data, so P3 is Lc. */
				if (ac->hdr.cla == 0x80 &&
				    (ac->hdr.p1 == 0x80 || ac->hdr.p1 == 0x40 ||
				     ac->hdr.p1 == 0x90 || ac->hdr.p1 == 0x50)) {
					ac->lc.tot = ac->hdr.p3;
				} else {
					/* Any other non-zero P1 on class 0x80:
					 * no command data and no expected data.
					 */
					ac->le.tot = ac->lc.tot = 0;
				}
			} else {
				/* P1 == 0: P3 is Lc. Covers GET PROCESSING
				 * OPTIONS and GET DATA above. */
				ac->lc.tot = ac->hdr.p3;
			}
			break;
		case 1: /* P3 == 0, No Lc/Le */
			ac->le.tot = ac->lc.tot = 0;
			break;
		case 2: /* P3 == Le - 00 c0 00 00 20 */
			ac->le.tot = ac->hdr.p3;
			break;
		case 3: /* P3 = Lc - 00 a4 04 00 0e */
			ac->lc.tot = ac->hdr.p3;
			/* copy Dc */
			ac->lc.cur = apdu_len - sizeof(ac->hdr);
			memcpy(ac->dc, apdu_buf + sizeof(ac->hdr),
				ac->lc.cur);
			break;
		case 4: /* P3 = Lc; SW with Le */
			ac->lc.tot = ac->hdr.p3;
			/* copy Dc */
			ac->lc.cur = apdu_len - sizeof(ac->hdr);
			memcpy(ac->dc, apdu_buf + sizeof(ac->hdr),
				ac->lc.cur);
			break;
		default:
			LOGP(DLGLOBAL, LOGL_ERROR, "Unknown APDU case %d\n", ac->apdu_case);
			return -1;
		}
	} else {
		/* copy more data, if available */
		int cpy_len;
		switch (ac->apdu_case) {
		case 0:
			/* Accumulate command data exactly as cases 3 and 4 do:
			 * the reader sends Lc bytes after the header, possibly
			 * split across several USB messages. */
			cpy_len = ac->lc.tot - ac->lc.cur;
			if (cpy_len > apdu_len)
				cpy_len = apdu_len;
			memcpy(ac->dc + ac->lc.cur, apdu_buf, cpy_len);
			ac->lc.cur += cpy_len;
			break;
		case 1:
		case 2:
			break;
		case 3:
		case 4:
			cpy_len = ac->lc.tot - ac->lc.cur;
			if (cpy_len > apdu_len)
				cpy_len = apdu_len;
			memcpy(ac->dc+ac->lc.cur, apdu_buf, cpy_len);
			ac->lc.cur += cpy_len;
			break;
		default:
			LOGP(DLGLOBAL, LOGL_ERROR, "Unknown APDU case %d\n", ac->apdu_case);
			return -1;
		}
	}

	/* take some decisions... */
	switch (ac->apdu_case) {
	case 0:
		/* Class 0x80: P3 is Lc, and the status word carries Le. */
		if (!is_dc_complete(ac)) {
			/* send PB + read further Lc bytes from reader */
			rc |= APDU_ACT_RX_MORE_CAPDU_FROM_READER;
		} else {
			/* send C-APDU to card */
			/* receive SW from card, forward to reader */
			rc |= APDU_ACT_TX_CAPDU_TO_CARD;
		}
		break;
	case 1: /* P3 == 0, No Lc/Le */
		/* send C-APDU to card */
		/* receive SW from card, forward to reader */
		rc |= APDU_ACT_TX_CAPDU_TO_CARD;
		break;
	case 2: /* P3 == Le */
		/* send C-APDU to card */
		/* receive Le bytes + SW from card, forward to reader */
		rc |= APDU_ACT_TX_CAPDU_TO_CARD;
		break;
	case 3: /* P3 = Lc */
		if (!is_dc_complete(ac)) {
			/* send PB + read further Lc bytes from reader */
			rc |= APDU_ACT_RX_MORE_CAPDU_FROM_READER;
		} else {
			/* send C-APDU to card */
			/* receive SW from card, forward to reader */
			rc |= APDU_ACT_TX_CAPDU_TO_CARD;
		}
		break;
	case 4: /* P3 = Lc; SW with Le */
		if (!is_dc_complete(ac)) {
			/* send PB + read further Lc bytes from reader */
			rc |= APDU_ACT_RX_MORE_CAPDU_FROM_READER;
		} else {
			/* send C-APDU to card */
			/* receive SW from card, forward to reader */
			rc |= APDU_ACT_TX_CAPDU_TO_CARD;
		}
		break;

	default:
		LOGP(DLGLOBAL, LOGL_ERROR, "Unknown APDU case %d\n", ac->apdu_case);
		return -1;
	}

	return rc;
}
