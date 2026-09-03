#!/bin/sh
# Capture the SIMtrace2 debug UART to a file while showing the interesting
# lines on screen.
#
# The firmware's own trace output goes to the debug UART, not over USB:
# 921600 8N1, TTL 3.3V. On the SIMtrace2 board use either the 2.5 mm stereo
# jack (tip = TX, ring = RX, sleeve = GND) or the DEBUG header (pin 1 = GND,
# pin 4 = TX, pin 5 = RX).
#
# Usage:  contrib/debug-uart-log.sh [device] [logfile]
#
# Build the firmware with TRACE_LEVEL=4. TRACE_LEVEL=5 additionally enables
# debug tracing inside interrupt handlers, which floods the line to the point
# of being unreadable and can stall the firmware; this tree even carries a
# "FIXME: no printfs in ISRs?" note about it.

set -e

DEV=${1:-/dev/ttyUSB0}
LOG=${2:-simtrace-debug.log}

if [ ! -e "$DEV" ]; then
	echo "no such device: $DEV" >&2
	echo "attached USB serial adapters:" >&2
	ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null >&2 || echo "  (none)" >&2
	exit 1
fi

stty -F "$DEV" 921600 raw -echo -echoe -echok -crtscts

echo "logging $DEV at 921600 8N1 to $LOG (everything is written to the file,"
echo "only card-interface lines are shown here). Ctrl-C to stop."
echo

# Everything lands in the log; the console shows just the lines that matter
# for bringing up card emulation.
cat "$DEV" | tee "$LOG" | grep --line-buffered -E \
	'etu_counter|waiting time expired|ATR|VCC|CLK|RST|card state|-E-|-W-'
