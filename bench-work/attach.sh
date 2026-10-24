#!/usr/bin/env bash
# Attach OpenOCD to both offroad elaphureLink probes without touching the rail.
set -euo pipefail
OCD="$HOME/dev/openocd-elaphurelink/openocd/src/openocd"
TCL="$HOME/dev/openocd-elaphurelink/openocd/tcl"
HERE="$(cd "$(dirname "$0")" && pwd)"
for spec in "192.168.0.172 6666 3333 4444 m" "192.168.0.195 6667 3334 4445 s"; do
  set -- $spec
  nohup "$OCD" -s "$TCL" -f interface/elaphurelink.cfg \
    -c "cmsis-dap elaphurelink addr $1" -c 'transport select swd' -c 'adapter speed 1000' \
    -c 'set CPUTAPID 0' -f target/stm32f1x.cfg \
    -c "gdb_port $3" -c "telnet_port $4" -c "tcl_port $2" -c init > "$HERE/ocd-$5.log" 2>&1 < /dev/null &
  sleep 4
  tail -1 "$HERE/ocd-$5.log"
done
