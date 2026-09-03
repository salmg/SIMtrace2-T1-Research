/* Unit tests for the T=1 block layer in card_emu_t1.c
 *
 * PCB values are written out as literals rather than reusing the macros from
 * the implementation, so that the encoding itself is under test.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "card_emu_t1.h"

static unsigned int g_fail;
static const char *g_case;

#define CHECK(cond) do {						\
	if (!(cond)) {							\
		printf("  FAIL %s:%d: %s\n", __func__, __LINE__, #cond);\
		g_fail++;						\
	}								\
} while (0)

#define CHECK_EQ(got, want) do {					\
	long _g = (long)(got), _w = (long)(want);			\
	if (_g != _w) {							\
		printf("  FAIL %s:%d: %s == 0x%lx, expected 0x%lx\n",	\
		       __func__, __LINE__, #got, _g, _w);		\
		g_fail++;						\
	}								\
} while (0)

static void begin(const char *name)
{
	g_case = name;
	printf("- %s\n", name);
}

/* ------------------------------------------------------------------ */

/* build a block with a correct EDC */
static unsigned int mkblock(uint8_t *out, enum t1_edc edc, uint8_t nad, uint8_t pcb,
			    const uint8_t *inf, uint8_t len)
{
	unsigned int i = 0;

	out[i++] = nad;
	out[i++] = pcb;
	out[i++] = len;
	if (len) {
		memcpy(out + i, inf, len);
		i += len;
	}
	i += t1_append_edc(edc, out, i);

	return i;
}

/* feed a whole block in, accumulating the reported events */
static uint32_t feed(struct t1_state *st, const uint8_t *buf, unsigned int len)
{
	uint32_t ev = 0;
	unsigned int i;

	for (i = 0; i < len; i++)
		ev |= t1_rx_byte(st, buf[i]);

	return ev;
}

/* pull everything the card wants to send */
static unsigned int drain(struct t1_state *st, uint8_t *out)
{
	unsigned int n = 0;
	uint8_t b;

	while (t1_get_tx_byte(st, &b))
		out[n++] = b;

	return n;
}

/* ------------------------------------------------------------------ */

static void test_lrc(void)
{
	uint8_t buf[8];
	uint8_t n;

	begin("LRC is the XOR over the block and zeroes it out");

	buf[0] = 0x00; buf[1] = 0x00; buf[2] = 0x01; buf[3] = 0xAA;
	n = t1_append_edc(T1_EDC_LRC, buf, 4);
	CHECK_EQ(n, 1);
	CHECK_EQ(buf[4], 0x00 ^ 0x00 ^ 0x01 ^ 0xAA);
	CHECK_EQ(buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4], 0);
}

static void test_crc_len(void)
{
	uint8_t buf[16];

	begin("CRC matches the published CRC-16/CCITT-FALSE check value");

	/* "123456789" is the standard check vector; for seed FFFF, polynomial
	 * 1021, no reflection and no final xor the result is 29B1 */
	memcpy(buf, "123456789", 9);
	CHECK_EQ(t1_append_edc(T1_EDC_CRC, buf, 9), 2);
	CHECK_EQ((buf[9] << 8) | buf[10], 0x29B1);
}

static void test_simple_exchange(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], out[T1_MAX_BLOCK];
	const uint8_t capdu[] = { 0x00, 0xA4, 0x04, 0x00, 0x00 };
	const uint8_t rapdu[] = { 0x6F, 0x10, 0x90, 0x00 };
	unsigned int n;
	uint32_t ev;

	begin("plain command/response exchange");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);

	/* reader sends I(N(S)=0, no chaining) */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0x00, capdu, sizeof(capdu));
	ev = feed(&st, blk, n);

	CHECK(ev & T1_EV_RX_DATA);
	CHECK(ev & T1_EV_CMD_COMPLETE);
	CHECK(!(ev & T1_EV_TX_READY));		/* unchained: no ack needed */
	CHECK_EQ(st.rx_inf_avail, sizeof(capdu));
	CHECK(!memcmp(st.rx_inf, capdu, sizeof(capdu)));
	CHECK(!st.rx_more);

	/* card answers with a single I-block, which must carry N(S)=0 */
	CHECK_EQ(t1_tx_inf(&st, rapdu, sizeof(rapdu), false), 0);
	n = drain(&st, out);
	CHECK_EQ(n, 3 + sizeof(rapdu) + 1);
	CHECK_EQ(out[0], 0x00);			/* NAD */
	CHECK_EQ(out[1], 0x00);			/* I-block, N(S)=0, no chaining */
	CHECK_EQ(out[2], sizeof(rapdu));	/* LEN */
	CHECK(!memcmp(&out[3], rapdu, sizeof(rapdu)));

	/* a correct block XORs to zero */
	{
		uint8_t x = 0;
		unsigned int i;
		for (i = 0; i < n; i++)
			x ^= out[i];
		CHECK_EQ(x, 0);
	}

	/* the next command from the reader must use N(S)=1 */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0x40, capdu, sizeof(capdu));
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_CMD_COMPLETE);

	/* and our next I-block must use N(S)=1 */
	CHECK_EQ(t1_tx_inf(&st, rapdu, sizeof(rapdu), false), 0);
	n = drain(&st, out);
	CHECK_EQ(out[1], 0x40);
}

static void test_chained_command(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], out[T1_MAX_BLOCK];
	uint8_t part1[32], part2[8];
	unsigned int n;
	uint32_t ev;

	begin("chained command from the reader is acknowledged per block");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);
	memset(part1, 0xA1, sizeof(part1));
	memset(part2, 0xB2, sizeof(part2));

	/* first segment: I(N(S)=0, M=1) */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0x20, part1, sizeof(part1));
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_RX_DATA);
	CHECK(ev & T1_EV_TX_READY);
	CHECK(!(ev & T1_EV_CMD_COMPLETE));
	CHECK(st.rx_more);
	CHECK_EQ(st.rx_inf_avail, sizeof(part1));

	/* the card must acknowledge with R(N(R)=1) and no error */
	n = drain(&st, out);
	CHECK_EQ(n, 4);
	CHECK_EQ(out[1], 0x90);			/* R-block, N(R)=1, no error */
	CHECK_EQ(out[2], 0x00);			/* LEN */

	/* final segment: I(N(S)=1, M=0) */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0x40, part2, sizeof(part2));
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_RX_DATA);
	CHECK(ev & T1_EV_CMD_COMPLETE);
	CHECK(!st.rx_more);
	CHECK_EQ(st.rx_inf_avail, sizeof(part2));
	CHECK(!memcmp(st.rx_inf, part2, sizeof(part2)));
}

static void test_chained_response(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], out[T1_MAX_BLOCK];
	uint8_t chunk[32];
	unsigned int n;
	uint32_t ev;

	begin("chained response advances on the reader's R-block");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);
	memset(chunk, 0x5A, sizeof(chunk));

	/* first chunk with the chaining bit set */
	CHECK_EQ(t1_tx_inf(&st, chunk, sizeof(chunk), true), 0);
	n = drain(&st, out);
	CHECK_EQ(out[1], 0x20);			/* I-block, N(S)=0, M=1 */

	/* the reader acks by asking for N(R)=1, which is our new ns */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0x90, NULL, 0);
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_NEED_NEXT_CHUNK);

	/* final chunk */
	CHECK_EQ(t1_tx_inf(&st, chunk, 4, false), 0);
	n = drain(&st, out);
	CHECK_EQ(out[1], 0x40);			/* I-block, N(S)=1, M=0 */
	CHECK_EQ(out[2], 4);
}

static void test_edc_error(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], out[T1_MAX_BLOCK];
	const uint8_t capdu[] = { 0x00, 0xA4 };
	unsigned int n;
	uint32_t ev;

	begin("a corrupted block draws R(EDC error)");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);

	n = mkblock(blk, T1_EDC_LRC, 0x00, 0x00, capdu, sizeof(capdu));
	blk[n - 1] ^= 0xff;			/* wreck the LRC */
	ev = feed(&st, blk, n);

	CHECK(ev & T1_EV_ERROR);
	CHECK(ev & T1_EV_TX_READY);
	CHECK(!(ev & T1_EV_RX_DATA));
	CHECK_EQ(st.stats.edc_errors, 1);

	n = drain(&st, out);
	CHECK_EQ(n, 4);
	CHECK_EQ(out[1], 0x81);			/* R-block, N(R)=0, EDC error */

	/* the expected sequence number must not have advanced */
	CHECK_EQ(st.nr, 0);
}

static void test_sequence_error(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], out[T1_MAX_BLOCK];
	const uint8_t capdu[] = { 0x00, 0xA4 };
	unsigned int n;
	uint32_t ev;

	begin("an out-of-sequence I-block draws R(other error)");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);

	/* we expect N(S)=0, the reader sends N(S)=1 */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0x40, capdu, sizeof(capdu));
	ev = feed(&st, blk, n);

	CHECK(ev & T1_EV_ERROR);
	CHECK(!(ev & T1_EV_RX_DATA));
	CHECK_EQ(st.stats.seq_errors, 1);

	n = drain(&st, out);
	CHECK_EQ(out[1], 0x82);			/* R-block, N(R)=0, other error */
	CHECK_EQ(st.nr, 0);
}

static void test_retransmit(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], first[T1_MAX_BLOCK], again[T1_MAX_BLOCK];
	const uint8_t rapdu[] = { 0x90, 0x00 };
	unsigned int n1, n2, n;
	uint32_t ev;

	begin("R-block naming the sent sequence number triggers a retransmit");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);

	CHECK_EQ(t1_tx_inf(&st, rapdu, sizeof(rapdu), false), 0);
	n1 = drain(&st, first);
	CHECK_EQ(first[1], 0x00);		/* I-block, N(S)=0 */

	/* reader asks for N(R)=0 again, i.e. it did not get that block */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0x80, NULL, 0);
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_TX_READY);
	CHECK(!(ev & T1_EV_NEED_NEXT_CHUNK));
	CHECK_EQ(st.stats.retransmits, 1);

	n2 = drain(&st, again);
	CHECK_EQ(n2, n1);
	CHECK(!memcmp(first, again, n1));
}

static void test_ifs_negotiation(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], out[T1_MAX_BLOCK];
	uint8_t big[254];
	const uint8_t ifs = 254;
	unsigned int n;
	uint32_t ev;

	begin("S(IFS request) is answered and raises what we may send");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);
	CHECK_EQ(st.ifsd, T1_DEFAULT_IFS);

	/* before negotiation, more than the default must be refused */
	memset(big, 0xC3, sizeof(big));
	CHECK(t1_tx_inf(&st, big, sizeof(big), false) < 0);

	n = mkblock(blk, T1_EDC_LRC, 0x00, 0xC1, &ifs, 1);
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_TX_READY);
	CHECK_EQ(st.ifsd, 254);

	n = drain(&st, out);
	CHECK_EQ(n, 5);
	CHECK_EQ(out[1], 0xE1);			/* S(IFS response) */
	CHECK_EQ(out[2], 1);
	CHECK_EQ(out[3], 254);

	/* now the large block is allowed */
	CHECK_EQ(t1_tx_inf(&st, big, sizeof(big), false), 0);
	n = drain(&st, out);
	CHECK_EQ(n, 3 + 254 + 1);

	/* a malformed IFS request is rejected */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0xC1, (const uint8_t *)"\x00", 1);
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_ERROR);
	CHECK_EQ(st.ifsd, 254);			/* unchanged */
}

static void test_resynch_and_abort(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], out[T1_MAX_BLOCK];
	const uint8_t rapdu[] = { 0x90, 0x00 };
	unsigned int n;
	uint32_t ev;

	begin("S(RESYNCH) restarts sequencing, S(ABORT) is answered");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);

	/* move both sequence numbers off zero */
	CHECK_EQ(t1_tx_inf(&st, rapdu, sizeof(rapdu), false), 0);
	drain(&st, out);
	CHECK_EQ(st.ns, 1);

	n = mkblock(blk, T1_EDC_LRC, 0x00, 0xC0, NULL, 0);
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_RESYNCH);
	CHECK(ev & T1_EV_TX_READY);
	CHECK_EQ(st.ns, 0);
	CHECK_EQ(st.nr, 0);
	CHECK(!st.last_i_valid);

	n = drain(&st, out);
	CHECK_EQ(out[1], 0xE0);			/* S(RESYNCH response) */

	n = mkblock(blk, T1_EDC_LRC, 0x00, 0xC2, NULL, 0);
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_TX_READY);
	n = drain(&st, out);
	CHECK_EQ(out[1], 0xE2);			/* S(ABORT response) */
}

static void test_wtx(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], out[T1_MAX_BLOCK];
	unsigned int n;
	uint32_t ev;

	begin("S(WTX request) goes out and its response is absorbed");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);

	CHECK(t1_tx_wtx_request(&st, 0) < 0);
	CHECK_EQ(t1_tx_wtx_request(&st, 3), 0);

	n = drain(&st, out);
	CHECK_EQ(n, 5);
	CHECK_EQ(out[1], 0xC3);			/* S(WTX request) */
	CHECK_EQ(out[2], 1);
	CHECK_EQ(out[3], 3);

	/* the reader's S(WTX response) needs no reply */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0xE3, (const uint8_t *)"\x03", 1);
	ev = feed(&st, blk, n);
	CHECK_EQ(ev, 0);
	CHECK_EQ(drain(&st, out), 0);
}

static void test_reserved_len(void)
{
	struct t1_state st;
	uint8_t out[T1_MAX_BLOCK];
	uint32_t ev = 0;
	unsigned int n;

	begin("LEN=FF is reserved and rejected");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);

	ev |= t1_rx_byte(&st, 0x00);		/* NAD */
	ev |= t1_rx_byte(&st, 0x00);		/* PCB */
	ev |= t1_rx_byte(&st, 0xff);		/* LEN */

	CHECK(ev & T1_EV_ERROR);
	n = drain(&st, out);
	CHECK_EQ(n, 4);
	CHECK_EQ(out[1], 0x82);			/* R-block, other error */

	/* the receiver must be back at the start of a block */
	CHECK_EQ(st.rx_state, T1_RX_NAD);
}

static void test_crc_roundtrip(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK], out[T1_MAX_BLOCK];
	const uint8_t capdu[] = { 0x00, 0xA4, 0x04, 0x00 };
	unsigned int n;
	uint32_t ev;

	begin("a CRC-protected block round-trips");

	t1_init(&st, T1_EDC_CRC, T1_DEFAULT_IFS);
	CHECK_EQ(st.rx_edc_len, 2);

	n = mkblock(blk, T1_EDC_CRC, 0x00, 0x00, capdu, sizeof(capdu));
	CHECK_EQ(n, 3 + sizeof(capdu) + 2);
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_CMD_COMPLETE);
	CHECK(!(ev & T1_EV_ERROR));

	/* flipping a CRC byte must be caught */
	t1_init(&st, T1_EDC_CRC, T1_DEFAULT_IFS);
	n = mkblock(blk, T1_EDC_CRC, 0x00, 0x00, capdu, sizeof(capdu));
	blk[n - 1] ^= 0x01;
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_ERROR);
	CHECK_EQ(st.stats.edc_errors, 1);

	n = drain(&st, out);
	CHECK_EQ(n, 3 + 0 + 2);			/* R-block with a two byte CRC */
	CHECK_EQ(out[1], 0x81);
}

static void test_max_size_block(void)
{
	struct t1_state st;
	uint8_t blk[T1_MAX_BLOCK];
	uint8_t inf[254];
	const uint8_t ifs = 254;
	unsigned int n;
	uint32_t ev;

	begin("a 254 byte INF is accepted without overrunning the buffer");

	t1_init(&st, T1_EDC_LRC, T1_DEFAULT_IFS);
	memset(inf, 0x7E, sizeof(inf));

	/* let the reader raise IFSD first so our own send limit is lifted too */
	n = mkblock(blk, T1_EDC_LRC, 0x00, 0xC1, &ifs, 1);
	feed(&st, blk, n);
	drain(&st, blk);

	n = mkblock(blk, T1_EDC_LRC, 0x00, 0x00, inf, sizeof(inf));
	CHECK_EQ(n, T1_MAX_BLOCK - 1);		/* LRC, so one byte short of max */
	ev = feed(&st, blk, n);
	CHECK(ev & T1_EV_CMD_COMPLETE);
	CHECK_EQ(st.rx_inf_avail, 254);
	CHECK(!memcmp(st.rx_inf, inf, sizeof(inf)));
}

int main(void)
{
	printf("T=1 block layer tests\n\n");

	test_lrc();
	test_crc_len();
	test_simple_exchange();
	test_chained_command();
	test_chained_response();
	test_edc_error();
	test_sequence_error();
	test_retransmit();
	test_ifs_negotiation();
	test_resynch_and_abort();
	test_wtx();
	test_reserved_len();
	test_crc_roundtrip();
	test_max_size_block();

	printf("\n%s\n", g_fail ? "FAILURES PRESENT" : "all checks passed");

	return g_fail ? 1 : 0;
}
