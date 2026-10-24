#!/usr/bin/env bash
set -euo pipefail
OCD="$HOME/dev/openocd-elaphurelink/openocd/src/openocd"
TCL="$HOME/dev/openocd-elaphurelink/openocd/tcl"
HERE="$(cd "$(dirname "$0")" && pwd)"
nohup "$OCD" -s "$TCL" -f interface/elaphurelink.cfg \
  -c "cmsis-dap elaphurelink addr 192.168.0.195" -c 'transport select swd' -c 'adapter speed 1000' \
  -c 'set CPUTAPID 0' -f target/stm32f1x.cfg \
  -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6667" -c init > "$HERE/ocd-s.log" 2>&1 < /dev/null &
sleep 6
grep -E "DPIDR|Listening on port 6667|Error" "$HERE/ocd-s.log" || tail -3 "$HERE/ocd-s.log"
