#!/usr/bin/env bash
# Long-lived OpenOCD against one offroad board, TCL on 6666 (master) / 6667 (slave).
set -u
B="$1"
case "$B" in
  master) IP=192.168.0.172; TCLP=6666; GDBP=3333; TELP=4444 ;;
  slave)  IP=192.168.0.195; TCLP=6667; GDBP=3334; TELP=4445 ;;
  *) echo "board must be master|slave" >&2; exit 2 ;;
esac
OCD=$HOME/dev/openocd-elaphurelink/openocd/src/openocd
TCL=$HOME/dev/openocd-elaphurelink/openocd/tcl
exec "$OCD" -s "$TCL" -f interface/elaphurelink.cfg \
  -c "cmsis-dap elaphurelink addr $IP" -c 'transport select swd' \
  -c 'adapter speed 1000' -c 'set CPUTAPID 0' \
  -c "tcl_port $TCLP" -c "gdb_port $GDBP" -c "telnet_port $TELP" \
  -f target/stm32f1x.cfg -c 'cortex_m reset_config sysresetreq' -c init
