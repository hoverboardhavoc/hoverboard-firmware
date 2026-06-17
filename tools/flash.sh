#!/usr/bin/env bash
# Cargo runner: flash a locally-built ELF through the ST-Link attached to the Pi.
# Usage (via cargo): cargo run --release   -> cargo invokes "flash.sh <elf>"
#
# Flashing touches the shared bench, so this runner holds the bench lock (tools/bench-lock.sh) for the
# duration. If another agent holds it, the flash is refused instead of colliding. Set BENCH_OWNER to your
# agent name (e.g. claude-main) so the lock is attributed to you; if you already hold the lock for a
# session this runner reuses it and does not release it (only releases a lock it acquired itself).
set -euo pipefail

ELF="${1:?usage: flash.sh <elf>}"
PI="${PI_HOST:-hoverboardhavoc@192.168.0.108}"
REMOTE="/tmp/hoverboard-fw.elf"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOCK="$SCRIPT_DIR/bench-lock.sh"
OWNER="${BENCH_OWNER:-flash-runner@$(hostname -s 2>/dev/null || echo host)}"
TOOK_LOCK=0

# Acquire the bench. exit 0 = ACQUIRED (we took it) or ALREADY-OURS (we already held it); exit 1 = held
# by someone else, so refuse to flash.
if acq="$("$LOCK" acquire "$OWNER" "flash.sh: $(basename "$ELF")")"; then
  case "$acq" in
    ACQUIRED*) TOOK_LOCK=1 ;;   # we took it here, so we release it on exit
    *) TOOK_LOCK=0 ;;           # ALREADY-OURS: held for a session, leave it held
  esac
else
  echo "flash: bench is busy, not flashing." >&2
  echo "$acq" >&2
  echo "flash: wait, or coordinate; see $LOCK status" >&2
  exit 1
fi
# Release only a lock this runner acquired, even if the flash fails.
trap '[ "$TOOK_LOCK" = 1 ] && "$LOCK" release "$OWNER" >/dev/null 2>&1 || true' EXIT

echo "flash: copying $(basename "$ELF") to $PI"
scp -q "$ELF" "$PI:$REMOTE"

echo "flash: programming via ST-Link"
ssh "$PI" "timeout 60 openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c 'program $REMOTE verify reset exit'"
