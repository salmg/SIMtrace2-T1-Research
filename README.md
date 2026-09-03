# SIMtrace2 with T=1 card emulation

Osmocom's [SIMtrace2](https://osmocom.org/projects/simtrace2), extended with an
**ISO 7816-3 T=1 block layer in the card emulation**. Upstream's `cardem`
speaks T=0 only; this tree speaks both, and selects between them from the ATR or
a PPS exchange.

Everything else tracks upstream `0.9.1-22-gf5eead5`. Of the 158 shared firmware
sources, 133 are byte-identical to it; the remainder have been merged.

This is a standalone tree with no dependency on any other project.

---

## Why

A card emulator that only speaks T=0 can present itself to a modem, which is
what SIMtrace2 was built for. It cannot present itself to a reader that requires
T=1 — which includes most EMV terminals, many UICC-aware devices, and any host
that runs a PPS to negotiate up.

This tree removes that limit. The protocol used toward the *emulated* side is
independent of the protocol used toward the *real* card: a T=0 SIM in a PC/SC
reader can be presented to a T=1 reader, and vice versa, because relaying
happens at APDU level.

---

## What is added

### Firmware — the T=1 block layer

| file | status |
| --- | --- |
| `firmware/libcommon/source/card_emu_t1.c` | **new** — the block layer |
| `firmware/libcommon/include/card_emu_t1.h` | **new** — its interface |
| `firmware/test/card_emu_t1_tests.c` | **new** — 14 unit tests |
| `firmware/libcommon/source/card_emu.c` | +304 lines |
| `firmware/libcommon/include/simtrace_prot.h` | +1 flag |

The block layer implements, and has a test for each:

- **I-block exchange**, plain and chained, in both directions
- **Chaining** — a chained command from the reader acknowledged per block; a
  chained response advanced on the reader's R-block
- **R-blocks** — `R(EDC error)` on a corrupted block, `R(other error)` on an
  out-of-sequence I-block, and retransmission when an R-block names the sequence
  number we sent
- **S-blocks** — `S(IFS request)` answered and the send window raised,
  `S(RESYNCH)` restarting sequencing, `S(ABORT)` answered, `S(WTX request)`
  emitted with its response absorbed
- **Error detection** — both LRC (XOR) and CRC-16/CCITT-FALSE, verified against
  the published check value
- **Bounds** — `LEN=FF` rejected as reserved; a 254-byte INF accepted without
  overrunning the buffer

`card_emu.c` gains an `ISO_S_IN_T1` state, protocol selection driven by
`atr_proto_mask`, and `enter_data_phase()` / `data_phase_state()` replacing
upstream's unconditional `ISO_S_WAIT_TPDU`. It also implements a `FIXME` that
upstream left open — *"check if proposal matches capabilities in ATR"* — so a
PPS proposing a protocol the firmware cannot speak is now rejected rather than
silently accepted.

Footprint on a SAM3S: **28,052 B ROM** (11.4% of 245 KB), 30,160 B RAM.

### Host — T=1 relaying and a dual-protocol PC/SC backend

| file | status |
| --- | --- |
| `host/lib/reader_pcsc.c`, `include/.../reader_pcsc.h` | **new** — PC/SC backend supporting T=0 and T=1 |
| `host/src/simtrace2-remsim.c` | `process_rx_t1()` and the T=1 dispatch |
| `host/lib/simtrace2_api.c` | `osmo_st2_cardem_request_t1_tx()` |
| `host/include/.../simtrace_prot.h` | `CEMU_DATA_F_T1_BLOCK` |
| `host/lib/apdu_dispatch.c` | EMV class `0x80` dispatch — see below |

Every other host file is current upstream. The tool is still named
`simtrace2-remsim`; upstream renamed theirs to `simtrace2-cardem-pcsc` and
rebuilt it around `osmo_select_main()` with asynchronous URBs, which is a
different program — reusing the name would invite reading their documentation
against this one.

Public API:

```c
/* transmit one chunk of a T=1 response; final=true on the last of a chain */
int osmo_st2_cardem_request_t1_tx(struct osmo_st2_cardem_inst *ci,
                                  const uint8_t *data, uint16_t len, bool final);

/* PC/SC reader and card, with an explicit protocol mask */
struct osim_reader_hdl *osmo_st2_pcsc_reader_open(int idx, const char *name, void *ctx);
struct osim_card_hdl   *osmo_st2_pcsc_card_open(struct osim_reader_hdl *rh, uint32_t proto_mask);
int                     osmo_st2_pcsc_active_proto(const struct osim_reader_hdl *rh);
```

`simtrace2-remsim` gains `-T / --card-proto t0|t1|auto` to select the protocol
used toward the **real** card, independently of what is presented to the reader.

### Host — EMV proprietary class support

*Salvador Mendoza ([salmg.net](https://salmg.net))*

`osim_determine_apdu_case()` classifies a command APDU against the UICC/SIM
class table. Every EMV proprietary `CLA=0x80` command matches nothing in it and
comes back as case 0 — which upstream's dispatcher treats as an error, logging
`Unknown APDU case 0` and returning `-1`. The APDU is dropped and the exchange
stalls.

`host/lib/apdu_dispatch.c` adds a case 0 arm that classifies these directly, so
an EMV card can be relayed:

| command | | P3 |
| --- | --- | --- |
| `80 A8 00 00 02 83 00` | GET PROCESSING OPTIONS | Lc |
| `80 CA 9F 36 05` | GET DATA (ATC) | Lc |
| `80 AE 80 00 1D` | GENERATE AC — ARQC | Lc |
| `80 AE 90 00 2B` | GENERATE AC — ARQC + CDA | Lc |

`GENERATE AC` encodes the requested cryptogram in P1: bits 7–6 select AAC, TC or
ARQC, and bit 4 adds CDA. The four values that carry command data — `0x40`,
`0x50`, `0x80`, `0x90` — take P3 as Lc; any other non-zero P1 on class `0x80`
has neither command nor expected data. Command data is then accumulated across
USB messages exactly as ISO cases 3 and 4 do.

**Verified against a Verifone terminal and a Mastercard test card.** All three
class `0x80` commands of a full transaction were classified correctly, each Lc
matching the data phase that followed:

| command | P1 | branch | Lc |
| --- | --- | --- | --- |
| `80 A8 00 00 02` GET PROCESSING OPTIONS | `0x00` | else | 2 |
| `80 AE 90 00 2B` GENERATE AC, ARQC + CDA | `0x90` | cryptogram | 43 |
| `80 AE 00 00 1D` GENERATE AC, AAC | `0x00` | else | 29 |

Without this arm the transaction stops at GET PROCESSING OPTIONS.

### Other host additions

- `--raw-atr` — forward the card's ATR verbatim, timing bytes and all, for
  comparison against the rewritten one
- `--synthetic-atr` — present a minimal T=0 ATR instead of the card's
- `-R / --auto-reinit SECONDS` — re-present the card after an idle period,
  avoiding a manual board reset between sessions
- ATR rewriting that preserves `TA3`/`TB3` when the ATR indicates T=1, since
  those are protocol parameters (IFSC, BWI/CWI) rather than electrical ones, and
  EMV Book 1 §8.3 rejects a T=1 card whose `TB3` is absent

---

## Status

| | |
| --- | --- |
| T=1 block layer | implemented, **14/14 unit tests pass** |
| T=1 protocol selection from ATR / PPS | implemented |
| Firmware builds and runs on hardware | yes — `0.9.2.1`, verified on a SIMtrace2 |
| **EMV relay to a payment terminal** | **working** — a complete transaction relayed to a Verifone: PSE discovery, application selection, GET PROCESSING OPTIONS, record reads, and both GENERATE AC exchanges |
| T=0 card emulation | **regression-tested on hardware**: 1054 APDUs relayed to an Android payment terminal, 0 errors, full SIM initialisation including writes |
| Host tools build and link the T=1 paths | yes |
| **T=1 exercised end-to-end against a real reader** | **not yet** |

The last row is the honest gap. The block layer is unit-tested and the host code
compiles and links, but no reader has yet negotiated T=1 against this firmware.
Presenting a T=1-only ATR to one Android terminal produced no response, which
that device also gives to every T=0 ATR — so it says nothing about T=1 either
way.

---

## Building

### Firmware

```bash
sudo apt install gcc-arm-none-eabi dfu-util
make -C firmware BOARD=simtrace APP=cardem
```

Boards: `simtrace`, `qmod`, `owhw`, `octsimtest`, `ngff_cardem`.
Applications: `cardem`, `trace`, `dfu`, `ccid`. **T=1 is in `cardem`.**

Build from a clean working tree, or `git-version-gen` stamps the image `-dirty`.

### Firmware tests

```bash
make -C firmware/test card_emu_t1_test && ./firmware/test/card_emu_t1_test
```

Fourteen checks, no hardware required, ending `all checks passed`.

*(The other target, `card_emu_test`, additionally needs host `libosmocore`
headers and `pkg-config`.)*

### Host tools

The host tree needs **five** libraries, and one of them is not packaged by any
distribution:

| | |
| --- | --- |
| `libosmocore`, `libosmosim` | in Debian/Ubuntu as `libosmocore-dev` |
| `libusb-1.0`, `libpcsclite` | packaged |
| **`libosmousb`** | **not packaged anywhere** |

`libosmousb` is built only when libosmocore is configured `--enable-libusb`,
which distributions do not do. Since all four binaries include
`osmocom/usb/libusb.h`, it is not optional — libosmocore must be built from
source.

```bash
sudo apt install -y libtool pkg-config libtalloc-dev libsctp-dev \
                    libmnl-dev libgnutls28-dev libusb-1.0-0-dev libpcsclite-dev

# Build libosmocore OUTSIDE this tree. Cloning it inside host/ leaves a 44 MB
# repository where the build system will not expect one, and it is easy to do
# by accident since the next step is also run from host/.
mkdir -p ~/src && cd ~/src
git clone --depth 30 https://gitea.osmocom.org/osmocom/libosmocore.git
cd ~/src/libosmocore
autoreconf -fi
./configure --prefix="$HOME/.local/osmo" --enable-libusb --disable-uring --disable-doxygen
make -j"$(nproc)" && make install
```

`--disable-uring` avoids a `liburing >= 0.7` requirement that recent libosmocore
adds and which is unrelated to anything here. The prefix keeps everything inside
`$HOME`; drop it to install system-wide.

Then:

```bash
export PKG_CONFIG_PATH="$HOME/.local/osmo/lib/pkgconfig"
cd /path/to/simtrace2-t1/host
autoreconf -fi && ./configure && make -j"$(nproc)"
```

Run the binaries through the `host/src/` wrappers, or set the library path:

```bash
export LD_LIBRARY_PATH="$HOME/.local/osmo/lib"
```

Produces `simtrace2-remsim`, `simtrace2-list`, `simtrace2-sniff` and
`simtrace2-remsim-usb2udp`. The files in `host/src/` are libtool wrappers that
set the library path; the ELF binaries are in `host/src/.libs/`.

```bash
./src/simtrace2-list
USB matches: 2
    1d50:60e3 Addr=21, Path=1-2.3, Cfg=1, Intf=0, Alt=0: 255/2/0 (CardEmulator Modem 1)
    1d50:60e3 Addr=21, Path=1-2.3, Cfg=2, Intf=0, Alt=0: 255/255/0 (0.9.2.1-f4e1)
```

The second interface carries the firmware version string. It lives in
configuration 2, which the board does not activate, so tools that read only the
active configuration will not see it.

---

## Relaying an EMV card

```bash
cd host && LD_LIBRARY_PATH="$HOME/.local/osmo/lib" \
  ./src/simtrace2-remsim --usb-vendor 1d50 --usb-product 60e3 \
                         --usb-path <PATH> --usb-config 1 -n 0 --synthetic-atr
```

**`--synthetic-atr` is required, not optional**, and the reason is a real
conflict rather than an oversight.

A SIM's ATR names **T=15** in its TDi chain. T=15 is not a transmission
protocol — it is the marker saying "the interface bytes after me are global
parameters", which is where the clock-stop indicator and the supply voltage
classes live. The rewriter keeps those deliberately: a modem running its SIM
interface at 1.8 V has no reason to accept a card that will not declare it
tolerates 1.8 V.

EMV Book 1 §8.3 admits **T=0 and T=1 only**. A terminal reading a TDi naming
T=15 has grounds to reject the card outright.

So the voltage classes can only be carried behind exactly the thing that gets
you rejected. There is no ATR that suits both. `--synthetic-atr` sends `3B 00`
— no interface bytes at all — which is the answer for EMV.

Measured against a Verifone terminal, in order:

| ATR presented | reached VCC+CLK, reset released | APDUs |
| --- | --- | --- |
| rewritten, `TA3` dropped | never | 0 |
| `--raw-atr` (`TA1 = 0x96` forwarded) | yes, cycling | 0 |
| rewritten, `TA3` kept | yes, consistently | 0 |
| **`--synthetic-atr`** (`3B 00`) | **yes** | **all of them** |

`--raw-atr` fails for the other reason the rewriter exists: `TA1 = 0x96` asks
for 16 clock cycles per ETU while the emulation runs a fixed 372, so every byte
is framed wrong.

A full transaction relayed this way is in
[`docs/emv-relay-verifone.md`](docs/emv-relay-verifone.md).

## Flashing

```bash
dfu-util --device 0x1d50:0x60e3 --path <PATH> --cfg 1 --alt 1 \
         --reset --download firmware/bin/simtrace-cardem-dfu.bin
```

`<PATH>` comes from `dfu-util --list`. **Re-read it every time** — flashing
causes a re-enumeration, so the path and device number change. In a virtual
machine a re-enumerating board is frequently handed to the *host* rather than the
guest, which from inside the guest is indistinguishable from a dead board.

A pre-built image is in `prebuilt/`, built from this repository at a clean
tree so its version string identifies the commit it came from. To see what is
on a board:

```bash
python3 contrib/flash.py list
```

### Do not flash the Osmocom image over this

`contrib/flash.py` downloads from `ftp.osmocom.org`. **That image has no T=1**,
so flashing it silently removes the capability and nothing in the output says so.

Two guards prevent it:

1. A board reporting an unparseable version — which is what an untagged local
   build reports — is refused.
2. A board reporting **≥ 0.9.2**, this fork's version floor, is refused
   regardless of version arithmetic. This is deliberately conservative: if
   upstream ever reaches 0.9.2, their image is refused too.

`--force-upstream` overrides both, having thought about it.

Recovery from a bad flash is the board's physical DFU button, which no firmware
change can affect.

---

## Driving the board yourself

If you write your own host rather than using `simtrace2-remsim`, the T=1
contract is:

`CEMU_DATA_F_T1_BLOCK` (`0x00000010`) marks a T=1 payload.

- **Card → host**: one received block's INF field, with `CEMU_DATA_F_FINAL` set
  on the last block of a chain.
- **Host → card**: one chunk of the response, again with `CEMU_DATA_F_FINAL` on
  the last.

The firmware owns EDC, sequence numbers, R-blocks and S-blocks. The host
supplies and consumes **INF only** — never raw blocks. NAD, PCB, LEN and the
EDC never cross USB.

Three details that are easy to get wrong:

**Chunk responses at 32 bytes.** `T1_HOST_CHUNK` is 32 because
`T1_DEFAULT_IFS` is 32, and `t1_tx_inf()` rejects anything larger — the
firmware logs the rejection and drops the chunk, so an oversized write is
silently lost. The *negotiated* IFSD is never reported back over USB, so 32 is
the only value a host can safely assume even after a successful `S(IFS)`
exchange. Push all chunks back to back; the firmware releases them one per
reader acknowledgement and there is no per-chunk ack to wait for.

**Do not strip the status word.** Under T=0 the host splits SW1SW2 off the
response. Under T=1 it must not: the status word is the tail of the response
APDU and the reader's own T=1 stack consumes it.

**A non-final block gets no reply.** When `CEMU_DATA_F_FINAL` is clear, append
to the reassembly buffer and return without sending anything — the firmware
answers a chained I-block itself with an R-block.

One consequence worth knowing: PC/SC readers commonly do *not* perform the
case-4 → `GET RESPONSE` conversion. A case-4 APDU with an explicit `Le` can
still return `61 xx` with no body. Under T=0 relaying this is invisible, because
the reader issues its own `GET RESPONSE` and you relay it. **Under T=1, whole
APDUs are exchanged and there is no such round trip**, so a host must run the
`61`/`6C` loop against the card itself or lose every response body.

---

## Layout

```
firmware/     the SAM3S firmware, T=1 block layer included
host/         C tooling: simtrace2-remsim, -list, -sniff, -usb2udp
hardware/     board design files (upstream)
contrib/      flash.py and packaging helpers
prebuilt/     a ready-to-flash cardem image
```

---

## Provenance and licence

Upstream SIMtrace2 is Osmocom's, under the licences stated in the individual
files; the T=1 additions follow the file they are in. This is not a fork of the
project's direction — it is upstream plus one capability, kept close enough that
merging from upstream again stays cheap. The last such merge brought the tree to
`0.9.1-22-gf5eead5`.
