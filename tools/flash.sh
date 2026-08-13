#!/usr/bin/env bash
# Cargo runner: flash a locally-built ELF onto a bench board through its probe.
# Usage (via cargo): cargo run --release   -> cargo invokes "flash.sh <elf>"
#
# There are two RUNNERS, i.e. two hosts a probe-touching command can execute on, and which one a board
# uses is a physical fact about that board (see the runner block below the BOARD table):
#   pi    - bench master/slave, on USB ST-Link clones plugged into the Pi.
#   local - the classywalk offroad pair, on LAN-attached ESP32-C3 elaphureLink probes.
# The guards are the same text and run in the same order on both; only the host they execute on differs.
# That includes the binutils the guards read the image through: on BOTH runners they are resolved and
# exercised on the real image, on the host that will run the guard, before the bench lock is taken, and
# a guard that cannot run refuses the flash rather than warning past it.
#
# Flashing touches the shared bench, so this runner holds the bench lock (tools/bench-lock.sh) for the
# duration. If another agent holds it, the flash is refused instead of colliding. Set BENCH_OWNER to your
# agent name (e.g. claude-main) so the lock is attributed to you; if you already hold the lock for a
# session this runner reuses it and does not release it (only releases a lock it acquired itself).
# The lock lives on the Pi for BOTH runners on purpose: it is the mutex for the whole physical bench,
# so a local-runner flash must still serialize against an agent working through the Pi.
set -euo pipefail

ELF="${1:?usage: flash.sh <elf>}"
PI="${PI_HOST:-pi@192.168.0.248}"
# Per-run remote name, and it has to be: the Pi runner's copy is made BEFORE the bench lock is taken
# (so the binutils can be exercised on the real image without holding the mutex), which is outside the
# mutex that used to make one fixed /tmp name safe. A shared name copied outside the lock is how one
# run's image gets programmed by another run. Removed on exit.
REMOTE="/tmp/hoverboard-fw.$$.elf"

# The offroad pair needs the PATCHED OpenOCD (the elaphureLink CMSIS-DAP-over-TCP driver); the stock
# openocd has no such interface. Built on THIS host, with its tcl tree beside the binary.
OFFROAD_OCD_BIN="${OFFROAD_OCD_BIN:-$HOME/dev/openocd-elaphurelink/openocd/src/openocd}"
OFFROAD_OCD_TCL="${OFFROAD_OCD_TCL:-$HOME/dev/openocd-elaphurelink/openocd/tcl}"

# Which bench board to program (~/notes/bench-overview.md, Pi-1 host map): the master F103 sits on
# the dapdirect ST-Link clone at USB 1-1.2.4, the slave F130 on the dap42 CMSIS-DAP probe at
# 1-1.2.1. Both need CPUTAPID 0 on these GD32s. Default master (the plain stlink.cfg reach).
BOARD="${BOARD:-master}"
case "$BOARD" in
  master)
    RUNNER=pi
    OC_CFG="-f interface/stlink.cfg -c 'transport select dapdirect_swd' -c 'adapter usb location 1-1.2.4' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  slave)
    RUNNER=pi
    OC_CFG="-c 'adapter driver cmsis-dap' -c 'cmsis-dap backend usb_bulk' -c 'cmsis-dap vid_pid 0x1209 0xda42' -c 'transport select swd' -c 'adapter speed 1000' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  # The classywalk offroad pair (the robot boards): reached over WiFi via the battery-powered
  # ESP32-C3 elaphureLink probes (.171 master / .195 slave, TCP 3240) and the PATCHED OpenOCD.
  # Those probes are on the LAN, not on the Pi's USB, so the Pi was only ever a shell to run
  # OpenOCD from and these boards run on the LOCAL runner. Same guards, same discipline; only the
  # transport and the host executing it differ. CPUTAPID 0 as ever. NOTE: this path cannot ATTACH
  # while a motor is PWM-driving (attach first, then drive; ~/notes/12fet-board-access.md).
  offroad-master)
    RUNNER=local
    OCD_BIN="$OFFROAD_OCD_BIN -s $OFFROAD_OCD_TCL"
    OC_CFG="-f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.171' -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  offroad-slave)
    RUNNER=local
    OCD_BIN="$OFFROAD_OCD_BIN -s $OFFROAD_OCD_TCL"
    OC_CFG="-f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.195' -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  *) echo "flash: BOARD must be master|slave|offroad-master|offroad-slave (got '$BOARD')" >&2; exit 2 ;;
esac

# ================================ RUNNER: WHERE COMMANDS EXECUTE ==============================
# RUNNER is set by BOARD in the table above, in one place, and is never probed for at runtime.
#
# There is deliberately NO "the Pi is unreachable, fall back to local" auto-detection. This tool's
# failure mode is programming the wrong thing, and a transport picked by whichever host happened to
# answer means a dropped ssh session silently changes which physical board gets written. For
# master/slave a local fallback could not work at all: their probes are USB ST-Link clones plugged
# into the Pi and simply do not exist on this host, so "falling back" would attach to nothing, or
# worse to some other probe on this host, while still printing the same reassuring progress lines.
# The runner is therefore a property of the board, fixed and greppable, and a Pi that is down is an
# error for the boards that need it rather than a different code path.
#
# FLASH_RUNNER exists to ASSERT the runner, not to choose it: setting it to something the board
# cannot use is refused, loudly, instead of quietly attempting the impossible.
if [ -n "${FLASH_RUNNER:-}" ] && [ "$FLASH_RUNNER" != "$RUNNER" ]; then
  echo "flash: REFUSED - FLASH_RUNNER='$FLASH_RUNNER' but BOARD=$BOARD runs on '$RUNNER'." >&2
  case "$RUNNER" in
    pi)
      echo "flash: the bench $BOARD sits on a USB ST-Link clone physically plugged into $PI." >&2
      echo "flash: that probe is not visible from this host, so there is no local path for it." >&2 ;;
    local)
      echo "flash: the offroad probes are LAN-attached ESP32-C3s driven by the patched OpenOCD on" >&2
      echo "flash: this host. The Pi is not in their signal path. Leave FLASH_RUNNER unset." >&2 ;;
  esac
  exit 2
fi

case "$RUNNER" in
  pi)
    OCD_BIN="${OCD_BIN:-openocd}"  # the bench ST-Links use the Pi's system openocd
    SUDO="sudo "                   # which needs root for the USB probe device nodes
    IMG="$REMOTE"                  # programmed from the copy made below
    TIMEOUT="timeout"              # GNU coreutils, always on the Pi
    HOST_LABEL="the Pi"
    PROBE_LABEL="ST-Link" ;;
  local)
    SUDO=""                        # verified 2026-08-03: the patched OpenOCD reaches the ESP32
                                   # probes over TCP 3240 unprivileged. No USB device node is
                                   # involved, so there is nothing for root to open.
    IMG="$ELF"                     # no copy: OpenOCD reads the real local ELF in place
    HOST_LABEL="this host"
    PROBE_LABEL="the elaphureLink probe"
    if [ ! -x "${OCD_BIN%% *}" ]; then
      echo "flash: REFUSED - patched OpenOCD not found or not executable at '${OCD_BIN%% *}'." >&2
      echo "flash: BOARD=$BOARD needs the elaphureLink build (the stock openocd has no such" >&2
      echo "flash: interface). Build it, or point OFFROAD_OCD_BIN/OFFROAD_OCD_TCL at it." >&2
      exit 2
    fi
    if [ ! -d "$OFFROAD_OCD_TCL" ]; then
      echo "flash: REFUSED - OpenOCD tcl tree missing at '$OFFROAD_OCD_TCL'." >&2
      exit 2
    fi
    # A hung OpenOCD must not sit on the bench lock forever, so every probe-touching command runs
    # under a wall-clock cap. macOS ships no coreutils `timeout`; perl's alarm(2) survives exec and
    # is the portable stand-in (SIGALRM's default action terminates the exec'd OpenOCD). If none of
    # the three exists, refuse rather than run uncapped while claiming a cap.
    # The trailing `exit 127` is load-bearing: perl's exec RETURNS on failure (a vanished or
    # non-executable OpenOCD) and the script would otherwise end normally with status 0, so a launch
    # that never happened would read as a program step that succeeded. 127 is the shell's own
    # command-not-found status.
    if command -v timeout >/dev/null 2>&1; then TIMEOUT="timeout"
    elif command -v gtimeout >/dev/null 2>&1; then TIMEOUT="gtimeout"
    elif command -v perl >/dev/null 2>&1; then TIMEOUT="perl -e 'alarm shift; exec @ARGV; exit 127'"
    else
      echo "flash: REFUSED - no timeout, gtimeout or perl on $HOST_LABEL to cap a hung OpenOCD." >&2
      exit 2
    fi ;;
esac

# Run a shell command string on whichever host owns the probe. Both branches take exactly ONE string
# and hand it to a shell, and both pass stdin through, so every guard below is the same text either
# way and the two runners cannot drift apart.
target_sh() {
  case "$RUNNER" in
    pi)    ssh "$PI" "$1" ;;
    local) bash -c "$1" ;;
  esac
}

# Release a lock this runner acquired, and remove the Pi's copy of the image, however the script
# leaves. One handler for both, because bash keeps ONE EXIT trap and a second `trap ... EXIT` later
# would silently replace the first. TOOK_LOCK/STAGED are initialised here so the handler is safe
# under `set -u` at any exit point; LOCK/OWNER only exist by the time TOOK_LOCK can be 1.
#
# `set -e` is live in here, so BOTH cleanups end in `|| true` and the status is captured first and
# returned last. Without that, either one failing takes the handler down with it: a failed release
# would skip the removal and leave the staged image on the Pi, and a failed removal after a
# SUCCESSFUL flash would report failure to cargo for a board that was in fact programmed, which
# invites exactly the re-flash of a live board this tool exists to prevent. The cleanups are
# best-effort; the run's own verdict is not theirs to change.
TOOK_LOCK=0
STAGED=""
cleanup() {
  local rc=$?
  if [ "$TOOK_LOCK" = 1 ]; then "$LOCK" release "$OWNER" >/dev/null 2>&1 || true; fi
  if [ -n "$STAGED" ]; then ssh "$PI" "rm -f $(printf '%q' "$STAGED")" >/dev/null 2>&1 || true; fi
  return "$rc"
}
trap cleanup EXIT

# The Pi runner programs from a copy, and the guards read that copy, so it has to exist before the
# binutils are exercised on it. That is BEFORE the bench lock: a host that cannot read the image must
# cost nobody the mutex. The local runner has the ELF already and every step below reads $IMG, so
# nothing downstream refers to the remote temp path on that path.
if [ "$RUNNER" = pi ]; then
  echo "flash: copying $(basename "$ELF") to $PI"
  if ! scp -q "$ELF" "$PI:$REMOTE"; then
    echo "flash: REFUSED - could not copy '$ELF' to $PI:$REMOTE." >&2
    echo "flash: the image guards run on $HOST_LABEL, against that copy; with no copy there is" >&2
    echo "flash: nothing to guard and nothing to program." >&2
    exit 2
  fi
  STAGED="$REMOTE"
else
  echo "flash: no copy needed; OpenOCD and the guards read the local ELF at $ELF"
fi

# The image guards below (wfi scan, code floor, required symbols, hot window) are the brick-preventers
# for boards whose probes cannot drive NRST, and they are only as good as the binutils they run
# through. "The tool is on PATH" is not "the tool works": Apple's /usr/bin/size EXISTS and rejects
# `-A`, which silently no-ops the whole gutted-image guard (floor + symbols + hot window at once), and
# an image with no disassembler behind it would let a `wfi` through, which locks SWD re-attach on
# these boards for good. So each tool is resolved in the SAME order the guards resolve it and then
# actually RUN against this image, ON THE HOST THAT WILL RUN THE GUARD (through target_sh, so the Pi's
# tools are exercised on the Pi and not merely looked for here), BEFORE the bench lock is taken.
#
# This applies to both runners. The Pi's native armv6l binutils do work on these images today,
# including Thumb wfi decode, but that is a property of the current Pi and not of the path: an arm64
# Pi, a trimmed image, or any PATH change would otherwise disable every image guard on the bench
# runner while it printed warnings that read like housekeeping. The bench boards are not the easier
# case either - the ST-Link clones cannot connect-under-reset, and the bench slave is a GD32F130, so
# a wfi image strands it exactly as it would an offroad board. A guard that cannot run is not a guard
# that passed, on either runner.
resolve_binutil() {
  local what="$1" flag="$2"; shift 2
  local probe out tool rc=0
  # One string, run on the target host: walk the candidates in the guards' own order, print the first
  # one that exists, then RUN it on the image. exit 3 = found but cannot read this image (Apple `size`
  # rejecting -A); exit 2 = nothing to try; anything else = the probe itself did not complete (a dead
  # ssh, say), which is equally not a pass.
  probe=$(printf 'for c in %s; do command -v "$c" >/dev/null 2>&1 || continue; echo "$c"; "$c" %s %q >/dev/null 2>&1 || exit 3; exit 0; done; exit 2' "$*" "$flag" "$IMG")
  out=$(target_sh "$probe" 2>/dev/null) || rc=$?
  tool=$(printf '%s\n' "$out" | sed -n 1p)
  case "$rc" in
    0) echo "flash: $what: $tool on $HOST_LABEL (exercised on $(basename "$ELF"))"; return 0 ;;
    2) echo "flash: REFUSED - no usable $what on $HOST_LABEL; looked for: $*" >&2
       echo "flash: the image guards cannot run without it, and on a board whose probe cannot drive" >&2
       echo "flash: NRST a skipped guard is how an unrecoverable image gets programmed. Put the" >&2
       echo "flash: arm-none-eabi toolchain on PATH on $HOST_LABEL and re-run." >&2 ;;
    3) echo "flash: REFUSED - '$tool' cannot read $(basename "$ELF") on $HOST_LABEL (ran: $tool $flag $IMG)." >&2
       echo "flash: either this image is not a readable ELF, or '$tool' is not the cross binutil the" >&2
       echo "flash: guards need (Apple's /usr/bin/size, for one, exists but rejects -A, which would" >&2
       echo "flash: silently disable the code-floor, required-symbol and hot-window checks together)." >&2
       echo "flash: put arm-none-eabi-$what on PATH on $HOST_LABEL and re-run." >&2 ;;
    *) echo "flash: REFUSED - could not run the $what check on $HOST_LABEL (rc=$rc)." >&2
       echo "flash: the guards read the image through it, so an unanswered check is a guard that did" >&2
       echo "flash: not run, not one that passed." >&2 ;;
  esac
  return 1
}
# ALLOW_WFI=1 asserts the firmware sets DBG_CTL0 debug-hold early in boot and skips the scan, so
# the disassembler is only required when the scan will actually run. size and nm always are.
if [ "${ALLOW_WFI:-0}" != "1" ]; then
  resolve_binutil objdump -d arm-none-eabi-objdump llvm-objdump rust-objdump objdump || exit 2
fi
resolve_binutil size -A arm-none-eabi-size llvm-size size || exit 2
resolve_binutil nm '' arm-none-eabi-nm llvm-nm rust-nm nm || exit 2

# "The guard could not RUN" is not "the image passed the guard". Every binutil was resolved and
# exercised against this image, on the host that runs the guards, before the bench lock was taken, so
# a tool that is missing or unusable by the time a guard runs means the ground moved under the run:
# fail closed, on either runner.
refuse_unrunnable() {
  echo "flash: REFUSED - $1" >&2
  echo "flash: a guard that cannot run is not a guard that passed, and this image would be" >&2
  echo "flash: programmed onto a board whose probe cannot drive NRST. The binutils were resolved AND" >&2
  echo "flash: exercised on $HOST_LABEL before the bench lock was taken, so they changed under this" >&2
  echo "flash: run. Fix PATH on $HOST_LABEL (arm-none-eabi-objdump/size/nm) and re-run." >&2
  exit 1
}

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
    PROFILE_REQ_SYMS='T main$,T SysTick$,usart1_rx_isr,dma_rx_isr,B CTRL_OBS$,B INJECT_UART_LINE_ERROR$,5probe3run,5probe12probe_family,5probe15probe_candidate,5probe13probe_present,5probe14measure_counts,5probe15scratch_present,5motor2hw10period_isr,5motor2hw5MOTOR,5motor7PERIODS,5motor9OBS_STATE,5motor7OBS_CAL,3arm2hw4GATE,3arm2hw5ARMED'
    # Hot-window membership (audit D7, balance-prep round): each of these must RESOLVE and sit
    # BELOW 0x08008000 (the F1x0 zero-wait boundary). A symbol that vanishes (inlined/renamed)
    # FAILS LOUDLY so the change is conscious: a silent drop out of the window is a 8.8x fetch
    # regression with CI green (the fsm_step near-miss). fsm_step itself is deliberately absent
    # (it fully inlines into run_shell).
    #
    # `4base5fixed3div` (the wide-division slice) is here for BOTH halves of the guard, and the
    # missing half is the one that matters most:
    #   - ABOVE the boundary: it is the image's only `Fix` division body, entered 9 times per 250 Hz
    #     control tick, so left to link order it lands ~0x0800a4xx and every one of those 9 pays 2
    #     wait states with no prefetch. It is placed by `#[link_section]` at the definition, which is
    #     rename-proof, but a `memory.x` edit could still evict it.
    #   - ABSENT: `base::fixed::div` only exists as a symbol because of its `#[inline(never)]`. Drop
    #     that and LLVM re-inlines a fresh ~730 B 128-bit division at all four call sites, the image
    #     grows ~8.6 KB, and nothing else fails. The gate turns that into a refusal.
    PROFILE_HOT_SYMS='5motor2hw10period_isr,12service_loop,control_task_cb,input_task_cb,9run_shell,7adc_isr,15systick_handler,systick_tick_cb,route_emits,route_handback,4base5fixed3div' ;;
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
# The EXIT handler installed above releases it (only a lock this runner acquired) and removes the
# Pi's copy of the image, whichever way the script leaves.

# GD32 SWD-lockout guard: a bare `wfi`/sleep executed with DBG_CTL0=0 (debug-hold bits unset) locks SWD
# re-attach on the GD32. The bench probes (ST-Link clone / dap42) cannot drive NRST, so it is an
# unrecoverable-looking brick. Refuse any image containing a `wfi` instruction unless ALLOW_WFI=1 asserts the
# firmware sets DBG_CTL0 |= 0b111 early in boot. See ~/notes/f130-firmware-debug-lockout-recoverable.md.
if [ "${ALLOW_WFI:-0}" != "1" ]; then
  echo "flash: scanning image for unguarded wfi/sleep (GD32 SWD-lockout guard)"
  set +e
  # printf %q so the image path survives the runner's shell as one word, on either runner.
  # The disassembly is taken as a VALUE, not piped straight into grep: piping makes the scan's exit
  # status grep's alone, so a disassembler that errors out (a broken tool, or a truncated/non-ELF
  # file) produces no output, no match, and rc 1, i.e. "clean - no wfi instruction in image" for an
  # image nothing ever read. rc 3 says the scan did not happen, which is not the same as passing.
  WFI_CMD=$(printf 'for c in arm-none-eabi-objdump llvm-objdump rust-objdump objdump; do command -v "$c" >/dev/null 2>&1 && { out=$("$c" -d %q 2>/dev/null) || exit 3; [ -n "$out" ] || exit 3; printf "%%s" "$out" | grep -iqw wfi; exit $?; }; done; exit 2' "$IMG")
  target_sh "$WFI_CMD"
  WFI_RC=$?
  set -e
  case "$WFI_RC" in
    0) echo "flash: REFUSED - image contains a 'wfi' instruction." >&2
       echo "flash: a bare wfi with DBG_CTL0=0 locks GD32 SWD re-attach (unrecoverable on the bench probes)." >&2
       echo "flash: use busy-spin firmware; if this image sets DBG_CTL0 debug-hold early in boot, re-run with ALLOW_WFI=1." >&2
       exit 1 ;;
    1) echo "flash: clean - no wfi instruction in image" ;;
    2) refuse_unrunnable "no disassembler on $HOST_LABEL, so the wfi scan did not run." ;;
    *) refuse_unrunnable "the wfi scan did not complete (rc=$WFI_RC)." ;;
  esac
fi

# LTO-gutted-image guard: a capacity/assert mismatch can make LTO delete the whole main loop while
# the image still links and flashes "fine" (round-7a: .text 55 KB -> 16.8 KB, missing symbols were
# the only tell). Before programming, refuse an image whose .text has collapsed below a floor, or
# that is missing a core symbol the live firmware must contain. Dependency-light: the same binutils
# the wfi scan relies on (size + nm). Missing or unusable tools REFUSE on BOTH runners: they were
# resolved and exercised on the guard's own host before the lock was taken, so reaching that branch
# means the toolchain changed under the run, which is never a routine condition.
echo "flash: LTO-gutted-image guard (profile=$IMAGE_PROFILE: release code-size floor + required symbols)"
set +e
# Pass the profile's floor + symbol set as positional args so the guard is profile-driven
# (the integrated firmware and the small imu-bench validator need different floors/symbols).
# printf %q: the command string is flattened by the runner's shell (the Pi's, via ssh, or this
# host's, via bash -c), so an unescaped arg containing a space ('T main$' in REQ_SYMS)
# word-splits and shifts every later positional.
# That had made the flash-side required-symbol check VACUOUS ($3 arrived as 'T', which matches
# every nm line) since the patterns gained spaces; the hot-window gate's first run exposed it.
GUARD_CMD=$(printf 'bash -s %q %q %q %q %q' "$IMG" "$PROFILE_TEXT_FLOOR" "$PROFILE_REQ_SYMS" "${PROFILE_HOT_SYMS:-}" "$HOST_LABEL")
target_sh "$GUARD_CMD" <<'IMAGE_GUARD'
  ELF="$1"; TEXT_FLOOR="$2"; REQ_SYMS="$3"; HOT_SYMS="$4"; HOST_LABEL="$5"
  size_tool=""; for c in arm-none-eabi-size llvm-size size; do command -v "$c" >/dev/null 2>&1 && { size_tool="$c"; break; }; done
  nm_tool="";   for c in arm-none-eabi-nm   llvm-nm   rust-nm nm; do command -v "$c" >/dev/null 2>&1 && { nm_tool="$c";   break; }; done
  if [ -z "$size_tool" ] || [ -z "$nm_tool" ]; then
    echo "flash: no size/nm on $HOST_LABEL; the LTO-gutted-image guard cannot run" >&2; exit 2
  fi
  # Executable bytes, NOT one section name: the F1x0 zero-wait-flash split (specs/motor-integration.md,
  # the hot-path placement) moves the 16 kHz ISR and the 250 Hz control path into `.hotcode` in the
  # first 32 KiB, so a `.text`-only floor sees a healthy image as gutted. Sum every code section.
  text=$("$size_tool" -A "$ELF" 2>/dev/null | awk '$1==".text" || $1==".hotcode" {t+=$2} END{if(t>0) print t}')
  if [ -z "$text" ]; then echo "flash: could not read code-section size on $HOST_LABEL (does this size accept -A?)" >&2; exit 2; fi
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
  # Hot-window membership: every listed symbol must resolve below 0x08008000 (see the profile
  # table's comment). Missing = FAIL (a rename/inline must be a conscious list update).
  if [ -n "$HOT_SYMS" ]; then
    hotmiss=""; hotcold=""
    OLDIFS="$IFS"; IFS=','
    for s in $HOT_SYMS; do
      line=$(printf "%s\n" "$syms" | grep -E "$s" | head -1)
      if [ -z "$line" ]; then hotmiss="${hotmiss} ${s}"; continue; fi
      addr=$(printf "%s" "$line" | awk '{print $1}')
      if [ $((16#$addr)) -ge $((16#08008000)) ]; then hotcold="${hotcold} ${s}@0x${addr}"; fi
    done
    IFS="$OLDIFS"
    if [ -n "$hotmiss" ]; then
      echo "flash: REFUSED - hot-window symbol(s) no longer resolve (inlined/renamed? update the list CONSCIOUSLY):${hotmiss}" >&2; exit 1
    fi
    if [ -n "$hotcold" ]; then
      echo "flash: REFUSED - hot-path symbol(s) ABOVE the 0x08008000 zero-wait boundary (8.8x fetch regression):${hotcold}" >&2; exit 1
    fi
    echo "flash: hot-window membership OK"
  fi
  echo "flash: guard OK - code ${text} B >= ${TEXT_FLOOR} B, required symbols present"
IMAGE_GUARD
GUARD_RC=$?
set -e
case "$GUARD_RC" in
  0) ;;  # healthy
  2) # size/nm missing, or `size -A` unreadable: the floor, the required-symbol check and the
     # hot-window check all no-op TOGETHER, so this rc is the whole guard vanishing at once.
     refuse_unrunnable "the LTO-gutted-image guard did not run (missing or unusable size/nm; see the line above)." ;;
  *) echo "flash: aborting on LTO-gutted-image guard failure." >&2; exit 1 ;;
esac

# ============================ THE STANDING ARMED-HALT RULE ===================================
# `program ... verify reset` HALTS the core. No tool in tools/ may halt while MOE reads set unless it
# just performed the disarm write itself (tools/swd-disarm-halt.sh is the one that does; this one
# deliberately does not, because reflashing a board whose bridge is live is the SECOND recorded
# incident, not merely a halt hazard). So: read TIMER0's CCHP first, and let the single owner of the
# policy (tools/armed-guard-verdict.sh) rule on the read. An INCONCLUSIVE read fails CLOSED there.
# The read itself runs on whichever host owns the probe, but the VERDICT stays where it has always
# been: tools/armed-guard-verdict.sh, called once, here, with whatever the read produced. Adding a
# local policy branch would give the offroad path its own opinion on "armed", which is exactly the
# drift that single owner exists to prevent.
#
# The READ is deliberately stronger than one mdw. `set CPUTAPID 0` (needed on these GD32s, whose
# IDCODE does not match the stm32f1x target) disables OpenOCD's only transport sanity check, so a
# link that is attached to nothing convincing can still answer a memory read with zeros, and 0 is
# exactly the value the policy must treat as SAFE (an unclocked TIMER0 legitimately reads 0). One
# such transient zero was observed on the master and did not reproduce in twelve samples. So the
# session reads CCHP TWICE and then a canary that is architecturally never zero, and hands all three
# to the verdict: a disagreement between the two reads, or a canary that is not a live ARM CPUID,
# means the read cannot be trusted, and an untrustworthy read is the verdict's inconclusive case.
# The canary is the SCB CPUID at 0xE000_ED00, which needs no clock enable and no peripheral to be
# configured, and whose implementer byte is always 0x41 (measured 2026-08-11 on both offroad boards:
# 0x412fc231). Same session as the CCHP reads, so it certifies THIS attach, not a later one.
echo "flash: armed-bridge guard (CCHP MOE must be clear before anything halts the core)"
set +e
ARMED_READ=$(target_sh "$TIMEOUT 30 ${SUDO}$OCD_BIN $OC_CFG -c init \
  -c 'mdw 0x40012C44 1' -c 'mdw 0x40012C44 1' -c 'mdw 0xE000ED00 1' -c shutdown 2>&1")
set -e
CCHP=$(printf '%s\n'   "$ARMED_READ" | sed -n 's/^0x40012c44: *\([0-9a-f]*\).*/\1/p' | sed -n 1p)
CCHP2=$(printf '%s\n'  "$ARMED_READ" | sed -n 's/^0x40012c44: *\([0-9a-f]*\).*/\1/p' | sed -n 2p)
CANARY=$(printf '%s\n' "$ARMED_READ" | sed -n 's/^0xe000ed00: *\([0-9a-f]*\).*/\1/p' | sed -n 1p)
"$SCRIPT_DIR/armed-guard-verdict.sh" flash "$CCHP" "$CCHP2" "$CANARY" "$RUNNER" || exit 1

PROGRAM_CMD="$TIMEOUT 60 ${SUDO}$OCD_BIN $OC_CFG \
  -c 'program $(printf '%q' "$IMG") verify reset exit'"

# FLASH_DRY_RUN=1 stops here, AFTER every guard has run for real, and prints the command instead of
# executing it. It can only ever do less than a normal run, never more, which is why it is safe to
# have: there is no way to reach the write with a guard skipped.
if [ "${FLASH_DRY_RUN:-0}" = "1" ]; then
  echo "flash: DRY RUN - NOTHING WAS PROGRAMMED." >&2
  echo "flash: the command that would have run on $HOST_LABEL for BOARD=$BOARD:" >&2
  printf '%s\n' "$PROGRAM_CMD" >&2
  exit 0
fi

echo "flash: programming $BOARD via $PROBE_LABEL"
target_sh "$PROGRAM_CMD"
