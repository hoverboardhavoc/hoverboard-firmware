#!/usr/bin/env bash
# Long-lived OpenOCD against one LAN-probe board; TCL port per board.
set -u
B="$1"
case "$B" in
  offroad-master) IP=192.168.0.172; TCLP=6666; GDBP=3333; TELP=4444 ;;
  offroad-slave)  IP=192.168.0.195; TCLP=6667; GDBP=3334; TELP=4445 ;;
  rover-master)   IP=192.168.0.139; TCLP=6668; GDBP=3335; TELP=4446 ;;
  rover-slave)    IP=192.168.0.147; TCLP=6669; GDBP=3336; TELP=4447 ;;
  *) echo "board?" >&2; exit 2 ;;
esac
OCD=$HOME/dev/openocd-elaphurelink/openocd/src/openocd
TCL=$HOME/dev/openocd-elaphurelink/openocd/tcl
exec "$OCD" -s "$TCL" -f interface/elaphurelink.cfg \
  -c "cmsis-dap elaphurelink addr $IP" -c 'transport select swd' \
  -c 'adapter speed 1000' -c 'set CPUTAPID 0' \
  -c "tcl_port $TCLP" -c "gdb_port $GDBP" -c "telnet_port $TELP" \
  -f target/stm32f1x.cfg -c 'cortex_m reset_config sysresetreq' -c init
