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
PI="${PI_HOST:-pi@192.168.0.248}"
REMOTE="/tmp/hoverboard-fw.elf"

# Which bench board to program (~/notes/bench-overview.md, Pi-1 host map): the master F103 sits on
# the dapdirect ST-Link clone at USB 1-1.2.4, the slave F130 on the dap42 CMSIS-DAP probe at
# 1-1.2.1. Both need CPUTAPID 0 on these GD32s. Default master (the plain stlink.cfg reach).
BOARD="${BOARD:-master}"
case "$BOARD" in
  master)
    OC_CFG="-f interface/stlink.cfg -c 'transport select dapdirect_swd' -c 'adapter usb location 1-1.2.4' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  slave)
    OC_CFG="-c 'adapter driver cmsis-dap' -c 'cmsis-dap backend usb_bulk' -c 'cmsis-dap vid_pid 0x1209 0xda42' -c 'transport select swd' -c 'adapter speed 1000' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  *) echo "flash: BOARD must be master|slave (got '$BOARD')" >&2; exit 2 ;;
esac

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

# GD32 SWD-lockout guard: a bare `wfi`/sleep executed with DBG_CTL0=0 (debug-hold bits unset) locks SWD
# re-attach on the GD32. The bench probes (ST-Link clone / dap42) cannot drive NRST, so it is an
# unrecoverable-looking brick. Refuse any image containing a `wfi` instruction unless ALLOW_WFI=1 asserts the
# firmware sets DBG_CTL0 |= 0b111 early in boot. See ~/notes/f130-firmware-debug-lockout-recoverable.md.
if [ "${ALLOW_WFI:-0}" != "1" ]; then
  echo "flash: scanning image for unguarded wfi/sleep (GD32 SWD-lockout guard)"
  set +e
  ssh "$PI" 'for c in arm-none-eabi-objdump llvm-objdump rust-objdump objdump; do command -v "$c" >/dev/null 2>&1 && { "$c" -d "'"$REMOTE"'" 2>/dev/null | grep -iqw wfi; exit $?; }; done; exit 2'
  WFI_RC=$?
  set -e
  case "$WFI_RC" in
    0) echo "flash: REFUSED - image contains a 'wfi' instruction." >&2
       echo "flash: a bare wfi with DBG_CTL0=0 locks GD32 SWD re-attach (unrecoverable on the bench probes)." >&2
       echo "flash: use busy-spin firmware; if this image sets DBG_CTL0 debug-hold early in boot, re-run with ALLOW_WFI=1." >&2
       exit 1 ;;
    1) echo "flash: clean - no wfi instruction in image" ;;
    2) echo "flash: WARNING - no disassembler on the Pi; could not verify wfi. Verify manually or set ALLOW_WFI=1." >&2 ;;
    *) echo "flash: WARNING - wfi scan inconclusive (rc=$WFI_RC); verify manually." >&2 ;;
  esac
fi

# LTO-gutted-image guard: a capacity/assert mismatch can make LTO delete the whole main loop while
# the image still links and flashes "fine" (round-7a: .text 55 KB -> 16.8 KB, missing symbols were
# the only tell). Before programming, refuse an image whose .text has collapsed below a floor, or
# that is missing a core symbol the live firmware must contain. Dependency-light: the same binutils
# the wfi scan relies on (size + nm). Missing tools warn-and-continue, matching the wfi guard.
echo "flash: LTO-gutted-image guard (release .text floor + required symbols)"
set +e
ssh "$PI" '
  ELF="'"$REMOTE"'"
  TEXT_FLOOR=40000
  size_tool=""; for c in arm-none-eabi-size llvm-size size; do command -v "$c" >/dev/null 2>&1 && { size_tool="$c"; break; }; done
  nm_tool="";   for c in arm-none-eabi-nm   llvm-nm   rust-nm nm; do command -v "$c" >/dev/null 2>&1 && { nm_tool="$c";   break; }; done
  if [ -z "$size_tool" ] || [ -z "$nm_tool" ]; then
    echo "flash: WARNING - no size/nm on the Pi; skipping the LTO-gutted-image guard" >&2; exit 2
  fi
  text=$("$size_tool" -A "$ELF" 2>/dev/null | awk "\$1==\".text\"{print \$2}")
  if [ -z "$text" ]; then echo "flash: WARNING - could not read .text size; skipping guard" >&2; exit 2; fi
  if [ "$text" -lt "$TEXT_FLOOR" ]; then
    echo "flash: REFUSED - release .text is ${text} B, below the ${TEXT_FLOOR} B floor (LTO-gutted image? a healthy image is ~50 KB)." >&2; exit 1
  fi
  syms=$("$nm_tool" "$ELF" 2>/dev/null)
  miss=""
  for s in " main\$" " SysTick\$" "usart1_rx_isr" "dma_rx_isr" " CTRL_OBS\$"; do
    printf "%s\n" "$syms" | grep -qE "$s" || miss="${miss} ${s}"
  done
  if [ -n "$miss" ]; then
    echo "flash: REFUSED - required symbol(s) absent from the image (LTO-gutted?):${miss}" >&2; exit 1
  fi
  echo "flash: guard OK - .text ${text} B, required symbols present"
'
GUARD_RC=$?
set -e
case "$GUARD_RC" in
  0) ;;  # healthy
  2) ;;  # tools missing: warned above, proceed
  *) echo "flash: aborting on LTO-gutted-image guard failure." >&2; exit 1 ;;
esac

echo "flash: programming $BOARD via ST-Link"
ssh "$PI" "timeout 60 sudo openocd $OC_CFG \
  -c 'program $REMOTE verify reset exit'"
