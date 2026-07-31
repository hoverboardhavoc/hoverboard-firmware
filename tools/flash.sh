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

# Which image is being flashed, so the gutted-image guard (below) applies the right .text floor and
# required-symbol set. The integrated firmware is ~54 KB .text with the control-stack symbols; the
# imu-bench validator is a legitimately small ~18 KB .text image with only IMU_BENCH_OBS. A single
# integrated-tuned guard rejects the legit imu-bench image (round 9), so the profile selects the set.
# FAIL-CLOSED: an unknown/absent value defaults to `integrated` (the shipping image), so a stray flash
# of the integrated firmware is never waved through a laxer floor.
IMAGE_PROFILE="${IMAGE_PROFILE:-integrated}"
case "$IMAGE_PROFILE" in
  integrated)
    # ~54 KB healthy; the control-stack symbols the live firmware must contain, PLUS the six detect
    # fault-path fns (matched by the length-prefixed path `..5probe<len><name>`, valid under BOTH legacy
    # and v0 mangling -- rustc 1.97 flipped the default to v0 and the old `17h` hash anchor
    # broke CI closed, 2026-07-22): these are #[inline(never)] +
    # host-untestable, so an LTO/codegen change that quietly dropped or inlined-away the bus-fault-safe
    # probe would strip the fleet's ONLY runtime chip-identity path (a class the round-7a gutted image
    # showed can pass link + flash). Keying the guard on them refuses such an image before it programs.
    # Comma-separated (survives ssh arg-splitting as one token; grep -E patterns).
    # The motor symbols (motor-integration slice 3) join the set on the same reasoning: the 16 kHz
    # period ISR is reached ONLY through a registered function pointer and its state through a
    # `static mut` the ISR alone touches, so an LTO/codegen change that dropped either would leave a
    # configured, counting timer with nothing stepping the commutator -- an image that still links,
    # still flashes, and silently has no motor. Length-prefixed path patterns (module + name,
    # never the rustc hash), valid under both legacy and v0 mangling.
    PROFILE_TEXT_FLOOR=40000
    PROFILE_REQ_SYMS='T main$,T SysTick$,usart1_rx_isr,dma_rx_isr,B CTRL_OBS$,B INJECT_UART_LINE_ERROR$,5probe3run,5probe12probe_family,5probe15probe_candidate,5probe13probe_present,5probe14measure_counts,5probe15scratch_present,5motor2hw10period_isr,5motor2hw5MOTOR,5motor7PERIODS,5motor9OBS_STATE,5motor7OBS_CAL,3arm2hw4GATE,3arm2hw5ARMED' ;;
  imu-bench)
    # ~18 KB healthy (full-LTO Mahony/CORDIC); the one SWD-readable block the validator publishes.
    # Floor well below 18 KB but far above a gutted few-KB image.
    PROFILE_TEXT_FLOOR=8000
    PROFILE_REQ_SYMS='B IMU_BENCH_OBS$' ;;
  probe)
    # The runtime-hal detect bench validators (bench-fw-detect / bench-fw-probe / bench-fw-faultpin):
    # legitimately tiny opt-s images (~1.5-4 KB .text) that exercise the fault-safe probe on silicon.
    # They publish their result to a FIXED RAM address (RESULT_ADDR), not a named symbol, so the guard
    # keys on the probe machinery every one of them links: the single-access probe read and the naked
    # BusFault entry's Rust bridge. The `.text` SECTION alone of the smallest validator (faultpin) is
    # ~536 B (the rest is .rodata); floor below that but well above a gutted stub. The required-symbol
    # check is the load-bearing guard for these tiny images (a gutted image loses the probe machinery).
    PROFILE_TEXT_FLOOR=256
    PROFILE_REQ_SYMS='probe_read32,bus_fault_trampoline' ;;
  motor-verify)
    # runtime-hal's bench-fw/motor-verify: the CONFIG-ONLY hot-path validator (advanced-timer
    # complementary-PWM bring-up + the motor-era NVIC priority ordering, then read both back; MOE is
    # never armed). ~3.1 KB .text at opt-s + full LTO, so it needs its own floor: the `integrated`
    # 40 KB floor rejects it and the `probe` 256 B floor is far too lax to notice a gutted one.
    # Symbols: its own crate-qualified cortex-m-rt main body (the whole configure/verify sequence
    # inlines INTO it, so if that body is gutted there is nothing left to run), plus the detect
    # fault-safe probe machinery it family-detects through -- family detection is what resolves the
    # per-family IRQ numbers the priority write targets, so an image that lost it would program the
    # WRONG vectors' priorities and still blink a verdict. Mangling-agnostic patterns (length-prefixed
    # paths + a crate-name anchor), valid under both legacy and v0.
    PROFILE_TEXT_FLOOR=2000
    PROFILE_REQ_SYMS='T main$,bench_fw_motor_verify.*cortex_m_rt_main,5probe3run,5probe12probe_family,5probe15bus_fault_entry' ;;
  commutation-example)
    # runtime-hal's proven spin choreography (examples/src/bin/commutation_splitboard.rs and
    # commutation_12fet.rs): the adopted reference this spec's bring-up order came from, re-run from
    # current mains at the user-present session (specs/arm-session.md). ~5.5 KB and ~6.0 KB of
    # FLASHED SPAN at opt-s + LTO, of which the code this floor measures is 4,352 B and 4,776 B
    # (measured 2026-07-31, both mains; the rest of the span is the 1 KB vector table plus
    # .rodata). Neither the 40 KB integrated floor nor the 256 B probe floor fits.
    #
    # THESE IMAGES ARM THE BRIDGE THEMSELVES: they call the arming gate directly, with no mode
    # machine in front of it, which is exactly why they are user-present-only. The profile exists so
    # they go through the SAME bench lock, wfi scan and armed-bridge guard as everything else rather
    # than being flashed by a raw openocd line.
    #
    # Symbols: the bin's own cortex-m-rt main body (the whole bring-up + spin loop inlines into it,
    # so a gutted image has nothing left) and the detect fault-safe probe machinery both bins
    # family-detect through. Mangling-agnostic patterns; the bin NAME is deliberately not anchored so
    # one profile covers both examples.
    PROFILE_TEXT_FLOOR=3000
    PROFILE_REQ_SYMS='cortex_m_rt_main,5probe3run,5probe15bus_fault_entry,5probe20bus_fault_trampoline' ;;
  *) echo "flash: IMAGE_PROFILE must be integrated|imu-bench|probe|motor-verify|commutation-example (got '$IMAGE_PROFILE')" >&2; exit 2 ;;
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
echo "flash: LTO-gutted-image guard (profile=$IMAGE_PROFILE: release code-size floor + required symbols)"
set +e
# Pass the profile's floor + symbol set as positional args so the remote guard is profile-driven
# (the integrated firmware and the small imu-bench validator need different floors/symbols).
ssh "$PI" 'bash -s' "$REMOTE" "$PROFILE_TEXT_FLOOR" "$PROFILE_REQ_SYMS" <<'REMOTE_GUARD'
  ELF="$1"; TEXT_FLOOR="$2"; REQ_SYMS="$3"
  size_tool=""; for c in arm-none-eabi-size llvm-size size; do command -v "$c" >/dev/null 2>&1 && { size_tool="$c"; break; }; done
  nm_tool="";   for c in arm-none-eabi-nm   llvm-nm   rust-nm nm; do command -v "$c" >/dev/null 2>&1 && { nm_tool="$c";   break; }; done
  if [ -z "$size_tool" ] || [ -z "$nm_tool" ]; then
    echo "flash: WARNING - no size/nm on the Pi; skipping the LTO-gutted-image guard" >&2; exit 2
  fi
  # Executable bytes, NOT one section name: the F1x0 zero-wait-flash split (specs/motor-integration.md,
  # the hot-path placement) moves the 16 kHz ISR and the 250 Hz control path into `.hotcode` in the
  # first 32 KiB, so a `.text`-only floor sees a healthy image as gutted. Sum every code section.
  text=$("$size_tool" -A "$ELF" 2>/dev/null | awk '$1==".text" || $1==".hotcode" {t+=$2} END{if(t>0) print t}')
  if [ -z "$text" ]; then echo "flash: WARNING - could not read code-section size; skipping guard" >&2; exit 2; fi
  if [ "$text" -lt "$TEXT_FLOOR" ]; then
    echo "flash: REFUSED - release code is ${text} B, below the ${TEXT_FLOOR} B floor for this profile (LTO-gutted image?)." >&2; exit 1
  fi
  syms=$("$nm_tool" "$ELF" 2>/dev/null)
  miss=""
  OLDIFS="$IFS"; IFS=','
  for s in $REQ_SYMS; do
    printf "%s\n" "$syms" | grep -qE "$s" || miss="${miss} ${s}"
  done
  IFS="$OLDIFS"
  if [ -n "$miss" ]; then
    echo "flash: REFUSED - required symbol(s) absent from the image (LTO-gutted?):${miss}" >&2; exit 1
  fi
  echo "flash: guard OK - code ${text} B >= ${TEXT_FLOOR} B, required symbols present"
REMOTE_GUARD
GUARD_RC=$?
set -e
case "$GUARD_RC" in
  0) ;;  # healthy
  2) ;;  # tools missing: warned above, proceed
  *) echo "flash: aborting on LTO-gutted-image guard failure." >&2; exit 1 ;;
esac

# ============================ THE STANDING ARMED-HALT RULE ===================================
# `program ... verify reset` HALTS the core. No tool in tools/ may halt while MOE reads set unless it
# just performed the disarm write itself (tools/swd-disarm-halt.sh is the one that does; this one
# deliberately does not, because reflashing a board whose bridge is live is the SECOND recorded
# incident, not merely a halt hazard). So: read TIMER0's CCHP first, and let the single owner of the
# policy (tools/armed-guard-verdict.sh) rule on the read. An INCONCLUSIVE read fails CLOSED there.
echo "flash: armed-bridge guard (CCHP MOE must be clear before anything halts the core)"
set +e
CCHP=$(ssh "$PI" "timeout 30 sudo openocd $OC_CFG -c init -c 'mdw 0x40012C44 1' -c shutdown 2>&1 \
  | sed -n 's/^0x40012c44: *\([0-9a-f]*\).*/\1/p' | head -1")
set -e
"$SCRIPT_DIR/armed-guard-verdict.sh" flash "$CCHP" || exit 1

echo "flash: programming $BOARD via ST-Link"
ssh "$PI" "timeout 60 sudo openocd $OC_CFG \
  -c 'program $REMOTE verify reset exit'"
