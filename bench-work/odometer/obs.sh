#!/usr/bin/env bash
# One observation of a board: EDGES, ODOMETER_TOTAL, boot_count, RCU_RSTSCK, motor_state, CCHP.
set -u
B="$1"
./ocd.sh "$B" \
  -c 'mdw 0x200008c0 1' \
  -c 'mdw 0x200008f8 1' \
  -c 'mdw 0x20000a9c 1' \
  -c 'mdw 0x40021024 1' \
  -c 'mdw 0x40012c44 1' 2>&1 | grep -E "^0x[0-9a-f]+:" 
