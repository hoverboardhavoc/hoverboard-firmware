#!/usr/bin/env bash
# One single-shot OpenOCD session against a LAN-probe board: ocd.sh <offroad-master|offroad-slave|rover-master|rover-slave> <-c cmd> ...
set -u
B="$1"; shift
case "$B" in
  offroad-master) IP=192.168.0.172 ;;
  offroad-slave)  IP=192.168.0.195 ;;
  rover-master)   IP=192.168.0.139 ;;
  rover-slave)    IP=192.168.0.147 ;;
  *) echo "board?" >&2; exit 2 ;;
esac
OCD=$HOME/dev/openocd-elaphurelink/openocd/src/openocd
TCL=$HOME/dev/openocd-elaphurelink/openocd/tcl
exec "$OCD" -s "$TCL" -f interface/elaphurelink.cfg \
  -c "cmsis-dap elaphurelink addr $IP" -c 'transport select swd' \
  -c 'adapter speed 1000' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg \
  -c 'cortex_m reset_config sysresetreq' -c init "$@" -c shutdown 2>&1
