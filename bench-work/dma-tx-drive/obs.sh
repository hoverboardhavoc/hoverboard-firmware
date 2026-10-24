#!/usr/bin/env bash
# obs.sh <board> <ctrl_obs_hex>: print the CTRL_OBS words 0-2,7,20,23,30,35,36,38 + RCU_RSTSCK
B="$1"; BASE="$2"
D="$(dirname "$0")"
"$D/ocd.sh" "$B" -c "mdw $BASE 3" -c "mdw [expr {$BASE+4*7}] 1" -c "mdw [expr {$BASE+4*20}] 4" -c "mdw [expr {$BASE+4*30}] 1" -c "mdw [expr {$BASE+4*35}] 2" -c "mdw [expr {$BASE+4*38}] 1" -c 'mdw 0x40021024 1' | grep -E "^0x[0-9a-f]+:"
