/* PC/SC card reader backend supporting both T=0 and T=1
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#pragma once

#include <stdint.h>

#include <osmocom/sim/sim.h>

/* bit-mask of enum osim_proto values */
#define ST2_PROTO_MASK_T0	(1 << OSIM_PROTO_T0)
#define ST2_PROTO_MASK_T1	(1 << OSIM_PROTO_T1)
#define ST2_PROTO_MASK_ANY	(ST2_PROTO_MASK_T0 | ST2_PROTO_MASK_T1)

/*! \brief Open a PC/SC reader by index.
 *  Equivalent to osim_reader_open(OSIM_READER_DRV_PCSC, ...) but backed by an
 *  implementation that is not restricted to T=0. */
struct osim_reader_hdl *osmo_st2_pcsc_reader_open(int idx, const char *name, void *ctx);

/*! \brief Connect to the card, negotiating any of the protocols in proto_mask.
 *  Pass ST2_PROTO_MASK_ANY to let the reader and card agree on one. */
struct osim_card_hdl *osmo_st2_pcsc_card_open(struct osim_reader_hdl *rh, uint32_t proto_mask);

/*! \brief Protocol actually in use, as enum osim_proto, or -1 if not connected. */
int osmo_st2_pcsc_active_proto(const struct osim_reader_hdl *rh);

/*! \brief Name of the underlying PC/SC reader, or NULL. */
const char *osmo_st2_pcsc_reader_name(const struct osim_reader_hdl *rh);
