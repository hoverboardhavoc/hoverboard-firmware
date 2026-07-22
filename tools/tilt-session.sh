#!/usr/bin/env bash
# One-shot runbook for the hand-tilt IMU sign-validation session (specs/imu-tilt-tool.md,
# specs/silicon-queue.md section 2). It stands up everything the session needs against the bench Pi
# and drops you into the live pitch readout, then tears it ALL down on exit -- rail off, probe
# server killed, tunnel closed, bench lock released -- even if a step in the middle fails.
#
# Flow: acquire the bench lock -> rail ON -> ensure the local firmware ELF -> start OpenOCD (slave
# probe cfg) on the Pi -> tunnel its TCL RPC port to localhost -> imu-tilt.py --selftest -> launch
# imu-tilt.py in the foreground (your args pass straight through).
#
# Usage:
#   tools/tilt-session.sh                         # live strip-chart readout, slave board
#   tools/tilt-session.sh --checklist             # the guided pitch sign checklist
#   tools/tilt-session.sh --checklist --record specs/bench-evidence/<date>/imu-tilt.csv
#   tools/tilt-session.sh --board master          # use the master probe cfg (master-IMU probe work)
#   tools/tilt-session.sh --keep-rail             # leave the rail ON for a follow-on session
#   tools/tilt-session.sh --dry-run --checklist   # print every command, touch nothing
#
# Any argument this script does not recognize is forwarded verbatim to tools/imu-tilt.py, so
# `--checklist`, `--record FILE`, `--no-graph`, `--addr`, etc. all just work.
#
# Env overrides: PI_HOST (default pi@192.168.0.248), TILT_PORT (default 6666),
#                BENCH_OWNER (default the invoking user, else hoverboardhavoc).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCK="$SCRIPT_DIR/bench-lock.sh"
FLASH_SH="$SCRIPT_DIR/flash.sh"
IMU_TILT="$SCRIPT_DIR/imu-tilt.py"

PI="${PI_HOST:-pi@192.168.0.248}"
PORT="${TILT_PORT:-6666}"
OWNER="${BENCH_OWNER:-${USER:-hoverboardhavoc}}"
ELF_REL="target/thumbv7m-none-eabi/release/firmware"
ELF="$REPO_DIR/$ELF_REL"

BOARD="slave"
KEEP_RAIL=0
DRY_RUN=0
PASS=()   # forwarded to imu-tilt.py

usage() { sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    --keep-rail) KEEP_RAIL=1; shift ;;
    --dry-run)   DRY_RUN=1; shift ;;
    --board)     BOARD="${2:?--board needs master|slave}"; shift 2 ;;
    --board=*)   BOARD="${1#*=}"; shift ;;
    -h|--help)   usage; exit 0 ;;
    *)           PASS+=("$1"); shift ;;
  esac
done

case "$BOARD" in
  master|slave) ;;
  *) echo "tilt-session: --board must be master|slave (got '$BOARD')" >&2; exit 2 ;;
esac

# Probe config: single source of truth is tools/flash.sh. Pull the exact OC_CFG line for this board
# out of it rather than re-declaring the probe wiring here (so a bench-probe change lands in one
# place). Fail loud if it cannot be found -- a wrong/blank probe cfg must never be guessed.
extract_oc_cfg() {
  awk -v b="$1" '
    $0 ~ "^[[:space:]]*"b"\\)" { found=1; next }
    found && /OC_CFG=/ {
      line=$0
      sub(/^[[:space:]]*OC_CFG="/, "", line)
      sub(/"[[:space:]]*;;[[:space:]]*$/, "", line)
      print line
      exit
    }
  ' "$2"
}
OC_CFG="$(extract_oc_cfg "$BOARD" "$FLASH_SH")"
if [ -z "$OC_CFG" ]; then
  echo "tilt-session: could not extract the $BOARD OC_CFG from $FLASH_SH (its format changed?)." >&2
  echo "tilt-session: flash.sh is the source of truth for the probe wiring; fix the parse or that file." >&2
  exit 2
fi

# --- state for the cleanup trap (set BEFORE anything that can fail) ---
TOOK_LOCK=0
TUNNEL_PID=""
CLEANED=0

log()  { printf '\n== %s ==\n' "$*"; }
cmd()  { printf '  + %s\n' "$*"; }              # show a command
sh_run() { cmd "$@"; [ "$DRY_RUN" = 0 ] && "$@"; return 0; }   # show + run (unless dry-run)

cleanup() {
  [ "$CLEANED" = 1 ] && return 0
  CLEANED=1
  log "cleanup"

  # 1. local tunnel
  if [ -n "$TUNNEL_PID" ]; then
    cmd "kill $TUNNEL_PID  (ssh tunnel)"
    if [ "$DRY_RUN" = 0 ]; then kill "$TUNNEL_PID" 2>/dev/null || true; fi
  fi

  # 2. remote OpenOCD (best-effort: we hold the lock, so no other openocd should be running)
  cmd "ssh $PI 'sudo pkill -f openocd; sudo rm -f /tmp/tilt-openocd.pid /tmp/tilt-openocd.log'"
  if [ "$DRY_RUN" = 0 ]; then
    ssh "$PI" 'sudo pkill -f openocd 2>/dev/null; sudo rm -f /tmp/tilt-openocd.pid /tmp/tilt-openocd.log 2>/dev/null' >/dev/null 2>&1 || true
  fi

  # 3. rail OFF (unless asked to keep it), then VERIFY it reads hi (= off)
  if [ "$KEEP_RAIL" = 1 ]; then
    cmd "(rail left ON: --keep-rail)"
  else
    sh_run ssh "$PI" "pinctrl set 4 op dh"
    if [ "$DRY_RUN" = 0 ]; then
      rv="$(ssh "$PI" 'pinctrl get 4' 2>/dev/null || true)"
      case "$rv" in
        *hi*) echo "  rail OFF confirmed ($rv)" ;;
        *)    echo "  !! WARNING: rail did NOT confirm OFF (pinctrl get 4 -> '${rv:-no reply}')." >&2
              echo "  !! CHECK THE BENCH and power the rail down by hand: ssh $PI 'pinctrl set 4 op dh'" >&2 ;;
      esac
    else
      cmd "ssh $PI 'pinctrl get 4'   (expect: hi)"
    fi
  fi

  # 4. bench lock (release only a lock this run acquired)
  if [ "$TOOK_LOCK" = 1 ]; then
    cmd "$LOCK release $OWNER"
    if [ "$DRY_RUN" = 0 ]; then "$LOCK" release "$OWNER" >/dev/null 2>&1 || true; fi
  else
    cmd "(lock not released: not acquired by this run)"
  fi
}
trap cleanup EXIT INT TERM

# --------------------------------------------------------------------------------------------------
log "bench lock"
cmd "$LOCK acquire $OWNER 'tilt session'"
if [ "$DRY_RUN" = 0 ]; then
  if acq="$("$LOCK" acquire "$OWNER" "tilt session")"; then
    case "$acq" in
      ACQUIRED*) TOOK_LOCK=1 ;;   # we took it; release on exit
      *)         TOOK_LOCK=0 ;;   # ALREADY-OURS: held for a session, leave it
    esac
    echo "  $acq"
  else
    echo "tilt-session: bench is busy, aborting." >&2
    echo "  $acq" >&2
    echo "tilt-session: wait or coordinate; see: $LOCK status" >&2
    exit 1
  fi
fi

# --------------------------------------------------------------------------------------------------
log "rail ON (pinctrl GPIO4 dl) + settle"
sh_run ssh "$PI" "pinctrl set 4 op dl"
cmd "sleep 3   (rail settle)"
[ "$DRY_RUN" = 0 ] && sleep 3

# --------------------------------------------------------------------------------------------------
log "ensure local firmware ELF"
# cargo build is incremental (a no-op when already current), so this also covers the 'stale' case.
sh_run cargo build --release -p firmware --manifest-path "$REPO_DIR/Cargo.toml"
if [ "$DRY_RUN" = 0 ] && [ ! -f "$ELF" ]; then
  echo "tilt-session: build did not produce $ELF" >&2
  exit 1
fi
echo "  NOTE: this is the LOCAL build ($ELF_REL); it cannot verify remotely that the boards carry"
echo "        this exact image. If pitch reads garbage, reflash the boards from this build first."

# --------------------------------------------------------------------------------------------------
log "start OpenOCD on the Pi ($BOARD probe cfg, from flash.sh)"
# System /usr/bin/openocd on the Pi; libusb needs sudo (bench-overview.md). No '-c shutdown', so it
# stays up as a server; its TCL RPC listens on 6666 (the default) for the tunnel. OC_CFG is expanded
# UNQUOTED exactly as flash.sh does, so the remote shell re-parses its single-quoted -c args.
OCD_REMOTE="sudo nohup openocd $OC_CFG -c init </dev/null >/tmp/tilt-openocd.log 2>&1 & echo TILT_OCD_STARTED"
sh_run ssh "$PI" "$OCD_REMOTE"

log "wait for OpenOCD to listen on the Pi (port $PORT)"
cmd "ssh $PI  poll ss/netstat for :$PORT (timeout ~15s)"
if [ "$DRY_RUN" = 0 ]; then
  if ! ssh "$PI" "for i in \$(seq 1 30); do { ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null; } | grep -q '[:.]$PORT '  && exit 0; sleep 0.5; done; exit 1"; then
    echo "tilt-session: OpenOCD did not open port $PORT on the Pi. Its log:" >&2
    ssh "$PI" "tail -n 40 /tmp/tilt-openocd.log 2>/dev/null" >&2 || true
    exit 1
  fi
  echo "  OpenOCD is listening on $PORT"
fi

# --------------------------------------------------------------------------------------------------
log "tunnel TCL RPC to localhost:$PORT"
cmd "ssh -N -o ExitOnForwardFailure=yes -L $PORT:localhost:$PORT $PI &"
if [ "$DRY_RUN" = 0 ]; then
  ssh -N -o ExitOnForwardFailure=yes -L "$PORT:localhost:$PORT" "$PI" &
  TUNNEL_PID=$!
  # wait for the local end to accept + forward
  ok=0
  for _ in $(seq 1 20); do
    if (exec 3<>"/dev/tcp/localhost/$PORT") 2>/dev/null; then ok=1; break; fi
    sleep 0.3
  done
  if [ "$ok" != 1 ]; then
    echo "tilt-session: local port $PORT never came up (tunnel failed)." >&2
    exit 1
  fi
  echo "  tunnel up (pid $TUNNEL_PID)"
fi

# --------------------------------------------------------------------------------------------------
log "imu-tilt.py self-test"
sh_run python3 "$IMU_TILT" --selftest

# --------------------------------------------------------------------------------------------------
log "live readout (imu-tilt.py) -- Ctrl-C to end the session"
cmd "python3 $IMU_TILT --elf $ELF_REL --host localhost --port $PORT ${PASS[*]-}"
if [ "$DRY_RUN" = 0 ]; then
  # Foreground; when it exits (or Ctrl-C), the EXIT trap runs full cleanup.
  python3 "$IMU_TILT" --elf "$ELF" --host localhost --port "$PORT" ${PASS[@]+"${PASS[@]}"}
fi

# cleanup runs via the EXIT trap
