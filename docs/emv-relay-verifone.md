# EMV relay to a Verifone terminal — 2026-09-03

A complete EMV transaction relayed through a SIMtrace2 in card emulation:
Verifone terminal → flex → SIMtrace2 → USB → `simtrace2-remsim` → PC/SC → a
Mastercard test card.

## The ATR is the whole difficulty

Three ATRs were tried against this terminal before one worked.

| presented | terminal reached `VCC CLK` with reset released? | APDUs |
| --- | --- | --- |
| rewritten, T=15 kept, `TA3` **dropped** | never | 0 |
| `--raw-atr`, `TA1 = 0x96` forwarded | yes, cycling | 0 |
| rewritten, T=15 kept, `TA3` kept | yes, consistently | 0 |
| **`--synthetic-atr` — `3B 00`** | **yes** | **all of them** |

Two independent problems, and they pull against each other:

**`TA1` must go.** `0x96` asks for Fi=512, Di=32 — 16 clock cycles per ETU —
while the emulation runs a fixed 372. Forwarding it frames every byte wrong, so
the terminal sees garbage and resets. That is what `--raw-atr` does.

**T=15 must also go, for EMV.** EMV Book 1 §8.3 admits T=0 and T=1 only. A SIM's
ATR names T=15 to carry its global bytes — clock-stop and supply voltage classes
— and an EMV terminal has grounds to reject a card announcing a protocol outside
the two it accepts.

The tension: the voltage-class byte is worth keeping, and it can **only** be
carried behind a `TDi` naming T=15. So the ATR that is right for a modem's SIM
slot is wrong for an EMV terminal. There is no single answer; `3B 00` is the one
for EMV.

## The transaction

```
SELECT 1PAY.SYS.DDF01           → 61 20, FCI
READ RECORD (PSE directory)     → six Mastercard applications
SELECT a0000000041010 01        → 61 3E, FCI
80 A8 GET PROCESSING OPTIONS    → AIP + AFL                    [CLA 0x80]
READ RECORD ×8 across the AFL   → PAN, track 2, CVM list, certificates
80 AE GENERATE AC   P1 = 0x90   → 9F27 = 80, ARQC + CDA        [CLA 0x80]
80 AE GENERATE AC   P1 = 0x00   → 9F27 = 00, AAC, declined     [CLA 0x80]
```

The card produced a genuine ARQC. The terminal, having no way to go online,
then requested an AAC and the card declined. Nothing was approved and nothing
reached an acquirer.

## What this validates

**`apdu_dispatch.c` case 0 is load-bearing.** `osim_determine_apdu_case()`
returns 0 for every `CLA 0x80` command, and upstream treats that as an error and
drops the APDU. All three of the commands above are class `0x80`. Without the
case 0 arm the transaction stops at GET PROCESSING OPTIONS.

Each classification was correct, with Lc matching the data phase that followed:
2, 43 and 29 bytes respectively. The `0x90` command took the cryptogram branch
(`P1` in `{0x40, 0x50, 0x80, 0x90}` — TC and ARQC, with and without CDA); the
other two took the else branch.

## Reproducing

```bash
cd host && LD_LIBRARY_PATH="$HOME/.local/osmo/lib" \
  ./src/simtrace2-remsim --usb-vendor 1d50 --usb-product 60e3 \
                         --usb-path <PATH> --usb-config 1 -n 0 --synthetic-atr
```

`--synthetic-atr` is not optional here. Anything derived from a SIM's ATR names
T=15 and the terminal refuses it.

Card data is deliberately not reproduced in this document.

## The same approach in Python

The SIM-side tool this firmware was developed alongside relays EMV too, and hit
the same two problems independently. Its resolution is worth recording because
it took a different shape:

- **The ATR**: identical conclusion, `--synthetic-atr` sending `3B 00`.
- **Class 0x80**: rather than a generic case 0 arm with a P1 heuristic, it keys
  on the instruction — `0xA8` GET PROCESSING OPTIONS and `0xAE` GENERATE AC,
  both of which carry command data unconditionally. That is only possible
  because it consults a SIM instruction table first, so `CLA 0x80` commands the
  SIM defines (`STATUS`, `FETCH`, `TERMINAL PROFILE`, `TERMINAL RESPONSE`) are
  classified before the EMV table is ever reached.

Both arrive at the same behaviour for the three commands measured here. The C
version classifies more broadly and needs the P1 test to do it; the Python
version classifies narrowly and does not.
