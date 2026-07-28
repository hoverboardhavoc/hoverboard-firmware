#!/usr/bin/env bash
# One-shot runbook for the hall-code hand-spin check (specs/hall-check.md, the one user step of
# motor-integration silicon stage 1). It stands up everything against the bench Pi and drops you into
# a live hall-code readout, then tears it ALL down on exit -- rail off, bench lock released -- even
# if a step in the middle fails. The tools/tilt-session.sh pattern, minus the tunnel: this readout is
# a plain SWD word read, so it needs no host-side tool.
#
# DISARMED: this only READS. The firmware it reads cannot arm a bridge (the arming gate is not named
# anywhere in crates/firmware; a host test enforces it), so the wheel turns only because you turn it.
# Bench rule stands: wheel free, never load-bearing.
#
# Usage:
#   tools/hall-check.sh                  # master board (the one with the motor staged)
#   tools/hall-check.sh --board slave
#   tools/hall-check.sh --keep-rail      # leave the rail ON for a follow-on session
#   tools/hall-check.sh --addr 0x20000d50  # CTRL_OBS override (default: resolved from the ELF)
#
# Env overrides: PI_HOST (default pi@192.168.0.248), BENCH_OWNER (default $USER, else hoverboardhavoc).
set -euo pipefail

BOARD=master
KEEP_RAIL=0
ADDR=""
ELF="target/thumbv7m-none-eabi/release/firmware"
while [ $# -gt 0 ]; do
  case "$1" in
    --board) BOARD="${2:?--board needs master|slave}"; shift 2 ;;
    --keep-rail) KEEP_RAIL=1; shift ;;
    --addr) ADDR="${2:?--addr needs a hex address}"; shift 2 ;;
    --elf) ELF="${2:?--elf needs a path}"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "hall-check: unknown argument $1" >&2; exit 2 ;;
  esac
done
case "$BOARD" in
  master) OC_CFG="-f interface/stlink.cfg -c 'transport select dapdirect_swd' -c 'adapter usb location 1-1.2.4' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  slave)  OC_CFG="-c 'adapter driver cmsis-dap' -c 'cmsis-dap backend usb_bulk' -c 'cmsis-dap vid_pid 0x1209 0xda42' -c 'transport select swd' -c 'adapter speed 1000' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  *) echo "hall-check: --board must be master|slave (got '$BOARD')" >&2; exit 2 ;;
esac

PI="${PI_HOST:-pi@192.168.0.248}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOCK="$SCRIPT_DIR/bench-lock.sh"
OWNER="${BENCH_OWNER:-${USER:-hoverboardhavoc}}"
TOOK_LOCK=0

cleanup() {
  echo
  echo "hall-check: tearing down"
  ssh -O exit -o ControlPath="$HOME/.ssh/hallcheck-cm-%r@%h" "$PI" >/dev/null 2>&1 || true
  ssh "$PI" 'sudo pkill -x openocd >/dev/null 2>&1; true' || true
  if [ "$KEEP_RAIL" = 0 ]; then
    ssh "$PI" 'pinctrl set 4 op dh' || true
    echo "hall-check: rail OFF"
  else
    echo "hall-check: rail left ON (--keep-rail)"
  fi
  [ "$TOOK_LOCK" = 1 ] && "$LOCK" release "$OWNER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# The CTRL_OBS address: resolved from the ELF so it cannot go stale across builds.
if [ -z "$ADDR" ]; then
  NM="$(ls ~/.rustup/toolchains/*/lib/rustlib/*/bin/llvm-nm 2>/dev/null | head -1)"
  [ -n "$NM" ] || NM=$(command -v arm-none-eabi-nm || command -v nm)
  [ -f "$ELF" ] || { echo "hall-check: no ELF at $ELF (build it, or pass --addr)" >&2; exit 1; }
  ADDR="0x$("$NM" "$ELF" | awk '$3=="CTRL_OBS"{print $1}')"
  [ "$ADDR" != "0x" ] || { echo "hall-check: CTRL_OBS not found in $ELF" >&2; exit 1; }
fi
echo "hall-check: board=$BOARD  CTRL_OBS=$ADDR"

if acq="$("$LOCK" acquire "$OWNER" "hall-check: hand-spin, $BOARD")"; then
  case "$acq" in ACQUIRED*) TOOK_LOCK=1 ;; esac
else
  echo "hall-check: bench is busy, not starting." >&2; echo "$acq" >&2; exit 1
fi

echo "hall-check: rail ON"
ssh "$PI" 'pinctrl set 4 op dl'
sleep 12   # boot + IMU settle; the motor bring-up runs late in boot (after the BLE probe window)

# Word map (specs/motor-integration.md "Observation"): w19 periods, w20 state, w22 duty2_angle,
# w23 fault. NOTE the off-by-one: `set -- $WORDS` makes $1 the FIRST word (w0, the magic), so
# CTRL_OBS word N is positional $((N+1)) -- w19 is $20, not $19. Verified against the `CtrlObs`
# struct in crates/firmware/src/main.rs (repr(C), 25 words) and specs/hall-check.md.
cat <<'EOT'

Turn the wheel SLOWLY BY HAND, one full revolution each way.
  want: the code walks 1,3,2,6,4,5 (one bit changes per step), never sticking at 0 or 7,
        with dwell 0 and no HALL fault, and the angle stepping at each code change.
Ctrl-C when done.

    periods      hall  angle   dwell  fault
EOT

LAST_P=""
# One persistent openocd on the Pi (attach ONCE); each row is then a cheap telnet query.
# The old shape paid a full openocd start + probe attach + shutdown per row (~4.5 s/row).
echo "hall-check: starting persistent openocd on the Pi"
ssh "$PI" "sudo pkill -x openocd >/dev/null 2>&1; sudo nohup openocd $OC_CFG -c init >/tmp/hallcheck-oocd.log 2>&1 & exit 0"
sleep 3
if ! ssh "$PI" 'pgrep -x openocd >/dev/null'; then
  echo "hall-check: openocd did not stay up; its log says:" >&2
  ssh "$PI" 'tail -5 /tmp/hallcheck-oocd.log' >&2 || true
  exit 1
fi
# Multiplex the per-row ssh over one authenticated connection (~100 ms/row instead of a fresh TCP+auth).
SSHM() { ssh -o ControlMaster=auto -o ControlPath="$HOME/.ssh/hallcheck-cm-%r@%h" -o ControlPersist=60 "$PI" "$@"; }

while true; do
  # stderr is KEPT (a failed attach must be loud, not parsed as zeros).
  # tr strips telnet CRs and NULs: a bare \r survives IFS as its OWN token and shifts every
  # positional past it by one per line (the misparse that showed motor_state as "angle 0x0700").
  RAW=$(SSHM "printf 'mdw $ADDR 25\nexit\n' | nc -w 2 localhost 4444 | tr -d '\r\000'" 2>&1 || true)
  OUT=$(printf '%s\n' "$RAW" | grep -A4 "^0x" || true)
  WORDS=$(printf '%s\n' "$OUT" | sed 's/^0x[0-9a-f]*: //' | tr '\n' ' ')
  # shellcheck disable=SC2086
  set -- $WORDS
  # A read is trusted only if the struct magic ("CTRL" = 0x4c525443) is word 0. An empty or
  # magic-less read means the ATTACH failed (probe busy, rail off, boot race), not that the
  # motor is unconfigured; print the real error instead of a row of zeros.
  if [ "${1:-}" != "4c525443" ]; then
    ERR=$(printf '%s\n' "$RAW" | grep -i -m1 "error\|failed\|timeout" || echo "no data returned")
    echo "  ** READ FAILED (probe/rail/attach), not motor state: ${ERR} **"
    sleep 2; continue
  fi
  # w19 motor_periods, w20 motor_state, w23 motor_fault, w22 motor_duty2_angle (each +1 positional).
  P=$((0x${20:-0})); S=$((0x${21:-0})); F=$((0x${24:-0})); DA=$((0x${23:-0}))
  HALL=$((S & 0xFF)); FLAGS=$(((S >> 24) & 0xFF)); ANGLE=$(((DA >> 16) & 0xFFFF))
  DWELL=$(((F >> 16) & 0xFFFF)); FAULT=$((F & 0xFFFF))
  LIVE="LIVE"; [ "$P" = "${LAST_P:-}" ] && LIVE="** ISR NOT ADVANCING **"
  [ $((FLAGS & 1)) = 1 ] || LIVE="** MOTOR NOT CONFIGURED (flags=$FLAGS) **"
  printf "  %10d   %2d   0x%04x   %5d   0x%04x  %s\n" "$P" "$HALL" "$ANGLE" "$DWELL" "$FAULT" "$LIVE"
  LAST_P=$P
  sleep 0.4
done
