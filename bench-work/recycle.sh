#!/usr/bin/env bash
# Power-cycle the offroad pair's rail and re-attach OpenOCD to both elaphureLink probes.
# The probes are USB-powered from the Pi and survive the cycle; the SWD sessions do not.
set -euo pipefail
PI="${PI_HOST:-pi@192.168.0.248}"
OCD="$HOME/dev/openocd-elaphurelink/openocd/src/openocd"
TCL="$HOME/dev/openocd-elaphurelink/openocd/tcl"
HERE="$(cd "$(dirname "$0")" && pwd)"

pkill -f "elaphurelink addr 192.168.0.17" 2>/dev/null || true
pkill -f "elaphurelink addr 192.168.0.19" 2>/dev/null || true
sleep 1
ssh -o ConnectTimeout=8 "$PI" 'pinctrl set 4 op dh; sleep 6; pinctrl set 4 op dl; sleep 6; pinctrl get 4'
for spec in "192.168.0.172 6666 3333 4444 m" "192.168.0.195 6667 3334 4445 s"; do
  set -- $spec
  nohup "$OCD" -s "$TCL" -f interface/elaphurelink.cfg \
    -c "cmsis-dap elaphurelink addr $1" -c 'transport select swd' -c 'adapter speed 1000' \
    -c 'set CPUTAPID 0' -f target/stm32f1x.cfg \
    -c "gdb_port $3" -c "telnet_port $4" -c "tcl_port $2" -c init > "$HERE/ocd-$5.log" 2>&1 &
  sleep 4
  tail -2 "$HERE/ocd-$5.log"
done
