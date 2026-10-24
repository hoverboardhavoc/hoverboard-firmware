#!/usr/bin/env bash
# Power-cycle with the three-cycle seating the relay contact now needs, verify the rail actually
# opened (the target must go unreachable), then bring it back and re-attach both probes.
set -euo pipefail
PI="${PI_HOST:-pi@192.168.0.248}"
OCD="$HOME/dev/openocd-elaphurelink/openocd/src/openocd"
TCL="$HOME/dev/openocd-elaphurelink/openocd/tcl"
HERE="$(cd "$(dirname "$0")" && pwd)"
pkill -f "elaphurelink addr 192.168.0.1" 2>/dev/null || true
sleep 1
ssh -o ConnectTimeout=10 "$PI" 'for i in 1 2 3; do pinctrl set 4 op dl; sleep 4; pinctrl set 4 op dh; sleep 4; done; pinctrl set 4 op dh; sleep 14; pinctrl get 4'
if "$OCD" -s "$TCL" -f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.172' \
   -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg -c init -c shutdown >/dev/null 2>&1; then
  echo "REFUSED: the target still answers with the relay open, the contact has not released"
  exit 1
fi
echo "rail confirmed open"
ssh -o ConnectTimeout=10 "$PI" 'pinctrl set 4 op dl; sleep 8; pinctrl get 4'
"$HERE/attach.sh"
