#!/usr/bin/env bash
# Disarm-then-halt over SWD, and the TIMER0_HOLD freeze check that validates the other half of the
# adopted halt posture (specs/motor-integration.md, "The debug-halt posture").
#
# ================================ THE STANDING RULE ==================================
# NO TOOL IN tools/ MAY HALT THE CORE WHILE MOE READS SET, unless it has just performed the disarm
# write itself. Halting stops software, not the timer's output stage: the counter keeps running and
# the last duties keep being applied with nothing stepping the commutator. That is the recorded
# incident that cooked a FET (runtime-hal/specs/M3-hotpath-milestone.md). MOE lives in TIMER0's CCHP
# and ONLY a write clears it, so there is no hardware posture that makes a halt safe.
#
# This script is the sanctioned halt path: it clears MOE over the DAP on the RUNNING target, reads
# it back to prove it cleared, and only then halts. Every other halt-capable tool here
# (tools/flash.sh, tools/store-bench-oracle.sh) READS CCHP first and REFUSES when MOE is set,
# pointing at this script. The bench rule "never halt while energized" stands under all of it.
# =====================================================================================
#
# Why the write works on a running target: MOE is memory, reachable through the MEM-AP exactly the
# way the CTRL_OBS observation reads already work without halting. That is what makes the deliberate
# case fully coverable. It covers nothing about a halt nobody initiated (a probe attach that halts, a
# hardware breakpoint); the firmware's boot-time TIMER0_HOLD is the backstop for that half, and
# `freeze` below is what proves that backstop actually works on this silicon.
#
# Usage:
#   tools/swd-disarm-halt.sh                       # disarm, verify, halt (master)
#   tools/swd-disarm-halt.sh --board slave
#   tools/swd-disarm-halt.sh resume                # release a halted target
#   tools/swd-disarm-halt.sh freeze                # the TIMER0_HOLD freeze validation, DISARMED
#   tools/swd-disarm-halt.sh --keep-rail freeze    # ...leaving the rail up for a follow-on step
#
# Env overrides: PI_HOST (default pi@192.168.0.248), BENCH_OWNER (default $USER).
set -euo pipefail

# TIMER0 register map, both families (GD32F10x UM Rev2.6 and GD32F1x0 UM Rev3.6, the TIMERx register
# tables). Base 0x4001_2C00, confirmed identical on F103 and F130:
CCHP_ADDR=0x40012C44   # CCHP, offset 0x44. MOE ("POEN") is bit 15.
CNT_ADDR=0x40012C24    # CNT,  offset 0x24, "accessed by word (32-bit)".
MOE_BIT=$((1 << 15))
# DBG_CTL0 @ 0xE004_2004 (DBG base 0xE004_2000, offset 0x04, reset 0 POWER-RESET-ONLY on both
# families). TIMER0_HOLD is bit 10 on BOTH (F1x0 UM 9.4.2, F10x UM 10.4.2: "hold the TIMER0 counter
# for debug when core is halted").
DBG_CTL0=0xE0042004
TIMER0_HOLD=$((1 << 10))

MODE=disarm-halt
BOARD=master
KEEP_RAIL=0
RAIL_ON=0
while [ $# -gt 0 ]; do
  case "$1" in
    --board) BOARD="${2:?--board needs master|slave}"; shift 2 ;;
    --keep-rail) KEEP_RAIL=1; shift ;;
    --rail-on) RAIL_ON=1; shift ;;
    disarm-halt|resume|freeze) MODE="$1"; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "swd-disarm-halt: unknown argument $1" >&2; exit 2 ;;
  esac
done
case "$BOARD" in
  master) OC_CFG="-f interface/stlink.cfg -c 'transport select dapdirect_swd' -c 'adapter usb location 1-1.2.4' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  slave)  OC_CFG="-c 'adapter driver cmsis-dap' -c 'cmsis-dap backend usb_bulk' -c 'cmsis-dap vid_pid 0x1209 0xda42' -c 'transport select swd' -c 'adapter speed 1000' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  *) echo "swd-disarm-halt: --board must be master|slave (got '$BOARD')" >&2; exit 2 ;;
esac

PI="${PI_HOST:-pi@192.168.0.248}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOCK="$SCRIPT_DIR/bench-lock.sh"
OWNER="${BENCH_OWNER:-${USER:-hoverboardhavoc}}"
TOOK_LOCK=0
CM="$HOME/.ssh/disarm-cm-%r@%h"

cleanup() {
  ssh -O exit -o ControlPath="$CM" "$PI" >/dev/null 2>&1 || true
  ssh "$PI" 'sudo pkill -x openocd >/dev/null 2>&1; true' || true
  if [ "$RAIL_ON" = 1 ] && [ "$KEEP_RAIL" = 0 ]; then
    ssh "$PI" 'pinctrl set 4 op dh' || true
    echo "swd-disarm-halt: rail OFF"
  fi
  [ "$TOOK_LOCK" = 1 ] && "$LOCK" release "$OWNER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

SSHM() { ssh -o ControlMaster=auto -o ControlPath="$CM" -o ControlPersist=60 "$PI" "$@"; }

# One telnet round trip to the persistent openocd; returns its raw output.
#
# The strip is two-stage and both stages matter. On the Pi: telnet IAC negotiation bytes and CRs, so
# `tr -cd` keeps only tab/newline/CR and printable ASCII (a bare CR survives IFS as its own token and
# shifts every positional past it; the non-ASCII IAC bytes make a UTF-8 `sed` abort with "illegal
# byte sequence" on the Mac, which reads as a failed attach when the read actually succeeded).
# Locally: LC_ALL=C on every parse, so no byte sequence can be "illegal" in the first place.
ocd() { SSHM "printf '%s\nexit\n' \"$1\" | nc -w 3 localhost 4444 | tr -d '\r\000' | tr -cd '\11\12\40-\176'"; }

# Read one 32-bit word; echoes bare hex (no 0x). Fails loudly rather than echoing zero.
rd32() {
  local out
  out=$(ocd "mdw $1 1" 2>&1 || true)
  local v
  v=$(printf '%s\n' "$out" | LC_ALL=C sed -n 's/^0x[0-9a-fA-F]*: *\([0-9a-fA-F]*\).*/\1/p' | head -1)
  if [ -z "$v" ]; then
    echo "swd-disarm-halt: READ FAILED at $1 (probe/rail/attach), not a zero value:" >&2
    printf '%s\n' "$out" >&2
    exit 1
  fi
  printf '%s' "$v"
}

moe_set() { [ $(( 0x$(rd32 $CCHP_ADDR) & MOE_BIT )) -ne 0 ]; }

if acq="$("$LOCK" acquire "$OWNER" "swd-disarm-halt: $MODE, $BOARD")"; then
  case "$acq" in ACQUIRED*) TOOK_LOCK=1 ;; esac
else
  echo "swd-disarm-halt: bench is busy, not starting." >&2; echo "$acq" >&2; exit 1
fi

if [ "$RAIL_ON" = 1 ]; then
  echo "swd-disarm-halt: rail ON"
  ssh "$PI" 'pinctrl set 4 op dl'
  sleep 12
fi

echo "swd-disarm-halt: attaching (board=$BOARD, NO halt at attach)"
ssh "$PI" "sudo pkill -x openocd >/dev/null 2>&1; sudo nohup openocd $OC_CFG -c init >/tmp/disarm-oocd.log 2>&1 & exit 0"
sleep 3
ssh "$PI" 'pgrep -x openocd >/dev/null' || {
  echo "swd-disarm-halt: openocd did not stay up:" >&2; ssh "$PI" 'tail -5 /tmp/disarm-oocd.log' >&2; exit 1; }

# ---------------------------------------------------------------------------------------------
# The disarm: write MOE clear on the RUNNING target, then PROVE it read back clear. The halt is
# gated on that proof; a CCHP that will not clear is a refusal, never a "halt anyway".
# ---------------------------------------------------------------------------------------------
disarm_and_verify() {
  local before after
  before=$(rd32 $CCHP_ADDR)
  printf 'swd-disarm-halt: CCHP before = 0x%s  (MOE %s)\n' "$before" \
    "$([ $(( 0x$before & MOE_BIT )) -ne 0 ] && echo SET || echo clear)"
  ocd "mww $CCHP_ADDR $(printf '0x%08X' $(( 0x$before & ~MOE_BIT )))" >/dev/null
  after=$(rd32 $CCHP_ADDR)
  if [ $(( 0x$after & MOE_BIT )) -ne 0 ]; then
    echo "swd-disarm-halt: REFUSING TO HALT - CCHP = 0x$after, MOE is STILL SET after the write." >&2
    echo "  Cut the rail instead. Do not halt an energized bridge." >&2
    exit 1
  fi
  printf 'swd-disarm-halt: CCHP after  = 0x%s  (MOE clear, verified)\n' "$after"
}

case "$MODE" in
  disarm-halt)
    disarm_and_verify
    echo "swd-disarm-halt: halting"
    ocd "halt" >/dev/null
    ocd "targets" | LC_ALL=C grep -E "halted|running" || true
    echo "swd-disarm-halt: target halted, bridge disarmed. 'resume' releases it."
    ;;

  resume)
    ocd "resume" >/dev/null
    echo "swd-disarm-halt: resumed"
    ;;

  # -----------------------------------------------------------------------------------------
  # The TIMER0_HOLD freeze validation. It needs NO energy: the counter runs with MOE clear (that
  # is the whole disarmed posture every stage-1..3 gate has run in), so this measures the halt
  # BEHAVIOUR without ever arming. It disarms first anyway, on the standing rule.
  # -----------------------------------------------------------------------------------------
  freeze)
    if moe_set; then
      echo "swd-disarm-halt: MOE is SET - this check runs DISARMED only." >&2
      disarm_and_verify
    fi
    hold=$(rd32 $DBG_CTL0)
    printf 'swd-disarm-halt: DBG_CTL0 = 0x%s  ->  TIMER0_HOLD (bit 10) %s\n' "$hold" \
      "$([ $(( 0x$hold & TIMER0_HOLD )) -ne 0 ] && echo SET || echo '** CLEAR: the firmware did not set it **')"
    [ $(( 0x$hold & TIMER0_HOLD )) -ne 0 ] || { echo "swd-disarm-halt: nothing to validate." >&2; exit 1; }

    a=$(rd32 $CNT_ADDR); b=$(rd32 $CNT_ADDR)
    echo "  running   CNT: 0x$a  0x$b   $([ "$a" != "$b" ] && echo 'ADVANCING (expected)' || echo '** IDENTICAL: counter already stopped? **')"
    [ "$a" != "$b" ] || { echo "swd-disarm-halt: the counter was not running; the freeze check proves nothing." >&2; exit 1; }

    disarm_and_verify
    ocd "halt" >/dev/null
    c=$(rd32 $CNT_ADDR); sleep 1; d=$(rd32 $CNT_ADDR)
    echo "  HALTED    CNT: 0x$c  0x$d   (1 s apart)"
    if [ "$c" = "$d" ]; then
      echo "  => FROZEN: TIMER0_HOLD holds the counter across a halt. PASS"
    else
      echo "  => ** NOT FROZEN ($((0x$d - 0x$c)) counts in 1 s): TIMER0_HOLD did not take effect. FAIL **"
    fi

    ocd "resume" >/dev/null
    e=$(rd32 $CNT_ADDR); f=$(rd32 $CNT_ADDR)
    echo "  resumed   CNT: 0x$e  0x$f   $([ "$e" != "$f" ] && echo 'ADVANCING again (expected)' || echo '** STILL STOPPED **')"
    [ "$c" = "$d" ] && [ "$e" != "$f" ] && echo "swd-disarm-halt: freeze validation PASS on $BOARD"
    ;;
esac
