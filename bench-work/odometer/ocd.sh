#!/usr/bin/env bash
# One single-shot OpenOCD session against an offroad board: ocd.sh <master|slave> <-c cmd> ...
set -u
B="$1"; shift
case "$B" in
  master) IP=192.168.0.172 ;;
  slave)  IP=192.168.0.195 ;;
  *) echo "board must be master|slave" >&2; exit 2 ;;
esac
OCD=$HOME/dev/openocd-elaphurelink/openocd/src/openocd
TCL=$HOME/dev/openocd-elaphurelink/openocd/tcl
exec "$OCD" -s "$TCL" -f interface/elaphurelink.cfg \
  -c "cmsis-dap elaphurelink addr $IP" -c 'transport select swd' \
  -c 'adapter speed 1000' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg \
  -c 'cortex_m reset_config sysresetreq' -c init "$@" -c shutdown 2>&1
