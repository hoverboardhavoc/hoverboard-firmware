#!/usr/bin/env bash
# Flash a locally-built ELF onto a bench board through its probe.
# Usage: CHIP=gd32f103c8|gd32f130c8 BOARD=<board> tools/flash.sh <elf>   (a release image)
#        IMAGE_PROFILE=imu-bench BOARD=<board> tools/flash.sh <elf>        (the IMU bench validator)
#
# There are two RUNNERS, i.e. two hosts a probe-touching command can execute on, and which one a board
# uses is a physical fact about that board (see the runner block below the BOARD table):
#   pi    - bench master/slave, on USB ST-Link clones plugged into the Pi.
#   local - the classywalk offroad pair, on LAN-attached ESP32-C3 elaphureLink probes.
# The guards are the same text and run in the same order on both; only the host they execute on differs.
# That includes the binutils the guards read the image through: on BOTH runners they are resolved and
# exercised on the real image, on the host that will run the guard, before any guard runs, and
# a guard that cannot run refuses the flash rather than warning past it.
#
# This tool takes no bench lock. The operator takes it first with
# ~/dev/hoverboard-bench/bench-lock.sh (hoverboard-bench, bench-locks.md) and releases it after. The
# resource follows from BOARD: rover-master/rover-slave are `rover`, offroad-master/offroad-slave
# are `offroad`, master (the ST-Link at Pi USB 1-1.2.4, which is the MM32's probe now) is `mm32`, and
# slave (the dap42 opened by VID:PID, a probe scan) is `probes`.
set -euo pipefail

ELF="${1:?usage: flash.sh <elf>}"
PI="${PI_HOST:-pi@192.168.0.248}"
# Per-run remote name, and it has to be: two runs can stage on the Pi at once (the locks are per
# resource, so two boards can be flashed side by side), and a shared name is how one run's image gets
# programmed by another run. Removed on exit.
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
  # The classywalk offroad pair (the robot boards): reached over WiFi via the ESP32-C3
  # elaphureLink probes (.172 master / .195 slave, TCP 3240; the master's was .171 until 2026-10,
  # DHCP addresses drift, find them by MAC) and the PATCHED OpenOCD.
  # Those probes are on the LAN, not on the Pi's USB, so the Pi was only ever a shell to run
  # OpenOCD from and these boards run on the LOCAL runner. Same guards, same discipline; only the
  # transport and the host executing it differ. CPUTAPID 0 as ever. NOTE: this path cannot ATTACH
  # while a motor is PWM-driving (attach first, then drive; ~/notes/12fet-board-access.md).
  offroad-master)
    RUNNER=local
    OCD_BIN="$OFFROAD_OCD_BIN -s $OFFROAD_OCD_TCL"
    OC_CFG="-f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.172' -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  offroad-slave)
    RUNNER=local
    OCD_BIN="$OFFROAD_OCD_BIN -s $OFFROAD_OCD_TCL"
    OC_CFG="-f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.195' -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  # The ROVER pair: the boards that used to be the bench master/slave, now mounted on the rover's
  # walls, each behind an ESP32-C3 SuperMini running the same elaphureLink probe firmware
  # (.139 master F103 UID 0x3239dc8a, .147 slave F130 UID 0x3639045e, confirmed 2026-10-23).
  # A SuperMini needs a ground path to the board it probes or SWD returns corrupted DPIDRs
  # (~/notes/bench-overview.md). Addresses drift: resolve by MAC with target/bench/probe-addr.sh.
  rover-master)
    RUNNER=local
    OCD_BIN="$OFFROAD_OCD_BIN -s $OFFROAD_OCD_TCL"
    OC_CFG="-f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.139' -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  rover-slave)
    RUNNER=local
    OCD_BIN="$OFFROAD_OCD_BIN -s $OFFROAD_OCD_TCL"
    OC_CFG="-f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.147' -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  *) echo "flash: BOARD must be master|slave|offroad-master|offroad-slave|rover-master|rover-slave (got '$BOARD')" >&2; exit 2 ;;
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
    # A hung OpenOCD must not hold the probe forever, so every probe-touching command runs
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

# Remove the Pi's copy of the image, however the script leaves. bash keeps ONE EXIT trap, so any
# later cleanup goes in this handler rather than a second `trap ... EXIT`. STAGED is initialised here
# so the handler is safe under `set -u` at any exit point.
#
# `set -e` is live in here, so the cleanup ends in `|| true` and the status is captured first and
# returned last. Without that, a failed removal after a SUCCESSFUL flash would report failure to the
# caller for a board that was in fact programmed, which invites exactly the re-flash of a live board
# this tool exists to prevent. The cleanup is best-effort; the run's own verdict is not its to change.
STAGED=""
cleanup() {
  local rc=$?
  if [ -n "$STAGED" ]; then ssh "$PI" "rm -f $(printf '%q' "$STAGED")" >/dev/null 2>&1 || true; fi
  return "$rc"
}
trap cleanup EXIT

# The Pi runner programs from a copy, and the guards read that copy, so it has to exist before the
# binutils are exercised on it, before any guard runs. The local runner has the ELF already and every step below reads $IMG, so
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
# tools are exercised on the Pi and not merely looked for here), BEFORE any guard runs.
#
# This applies to both runners. The Pi's native armv6l binutils do work on these images today,
# including Thumb wfi decode, but that is a property of the current Pi and not of the path: an arm64
# Pi, a trimmed image, or any PATH change would otherwise disable every image guard on the bench
# runner while it printed warnings that read like housekeeping. The bench boards are not the easier
# case either - the ST-Link clones cannot connect-under-reset, and the bench slave is a GD32F130, so
# a wfi image strands it exactly as it would an offroad board. A guard that cannot run is not a guard
# that passed, on either runner.
#
# The disassembler is held to one more thing than the others, because "it ran" was never the property
# the wfi guard needed: it has to report arm as the architecture it decoded (see resolve_binutil).
# That closes the tool-present-but-wrong-target half of the same scenario, which an arm64 Pi with
# multi-target binutils is exactly how you would meet. What is still self-report rather than proof is
# the decode itself: a known-instruction positive control (assemble a wfi, watch this tool find it)
# would prove it, and is deliberately not paid for, for a failure that has not occurred.
resolve_binutil() {
  local what="$1" flag="$2"; shift 2
  local probe out tool rc=0 arch=""
  # The disassembler carries one extra requirement, and it is the one the wfi guard actually rests
  # on: running is not DECODING. A multi-target binutils on a non-arm host resolves clean here,
  # disassembles this image as its own ISA, emits plenty of output, and contains no `wfi` mnemonic
  # anywhere - so the scan reports "clean - no wfi instruction in image" and the guard is silently
  # absent for the exact image class it exists to stop. `objdump -f` prints the architecture it
  # selected (GNU says `architecture: armv7, flags ...`, llvm-objdump says `architecture: arm`), so
  # the tool has to SAY arm as well as run. exit 4 = it decodes this image as something else, or
  # will not say what it decodes it as, neither of which is a pass.
  if [ "$what" = objdump ]; then
    arch=$(printf ' "$c" -f %q 2>/dev/null | grep -qi "^architecture:.*arm" || exit 4;' "$IMG")
  fi
  # One string, run on the target host: walk the candidates in the guards' own order, print the first
  # one that exists, then RUN it on the image. exit 3 = found but cannot read this image (Apple `size`
  # rejecting -A); exit 2 = nothing to try; anything else = the probe itself did not complete (a dead
  # ssh, say), which is equally not a pass.
  probe=$(printf 'for c in %s; do command -v "$c" >/dev/null 2>&1 || continue; echo "$c"; "$c" %s %q >/dev/null 2>&1 || exit 3;%s exit 0; done; exit 2' "$*" "$flag" "$IMG" "$arch")
  out=$(target_sh "$probe" 2>/dev/null) || rc=$?
  tool=$(printf '%s\n' "$out" | sed -n 1p)
  case "$rc" in
    0) echo "flash: $what: $tool on $HOST_LABEL (exercised on $(basename "$ELF")${arch:+, decodes arm})"
       return 0 ;;
    2) echo "flash: REFUSED - no usable $what on $HOST_LABEL; looked for: $*" >&2
       echo "flash: the image guards cannot run without it, and on a board whose probe cannot drive" >&2
       echo "flash: NRST a skipped guard is how an unrecoverable image gets programmed. Put the" >&2
       echo "flash: arm-none-eabi toolchain on PATH on $HOST_LABEL and re-run." >&2 ;;
    3) echo "flash: REFUSED - '$tool' cannot read $(basename "$ELF") on $HOST_LABEL (ran: $tool $flag $IMG)." >&2
       echo "flash: either this image is not a readable ELF, or '$tool' is not the cross binutil the" >&2
       echo "flash: guards need (Apple's /usr/bin/size, for one, exists but rejects -A, which would" >&2
       echo "flash: silently disable the code-floor, required-symbol and hot-window checks together)." >&2
       echo "flash: put arm-none-eabi-$what on PATH on $HOST_LABEL and re-run." >&2 ;;
    4) echo "flash: REFUSED - '$tool' does not decode $(basename "$ELF") as arm on $HOST_LABEL." >&2
       echo "flash: it RAN on the image, which is all the exercise above proves; 'objdump -f' reports" >&2
       echo "flash: an architecture that is not arm, or reports none at all. Its disassembly is then" >&2
       echo "flash: the wrong instruction set, the wfi scan finds no 'wfi' mnemonic in it, and an" >&2
       echo "flash: image that locks SWD re-attach for good gets programmed reported as clean." >&2
       echo "flash: put arm-none-eabi-objdump on PATH on $HOST_LABEL and re-run." >&2 ;;
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
# exercised against this image, on the host that runs the guards, before the guards ran, so
# a tool that is missing or unusable by the time a guard runs means the ground moved under the run:
# fail closed, on either runner.
refuse_unrunnable() {
  echo "flash: REFUSED - $1" >&2
  echo "flash: a guard that cannot run is not a guard that passed, and this image would be" >&2
  echo "flash: programmed onto a board whose probe cannot drive NRST. The binutils were resolved AND" >&2
  echo "flash: exercised on $HOST_LABEL before the guards ran, so they changed under this" >&2
  echo "flash: run. Fix PATH on $HOST_LABEL (arm-none-eabi-objdump/size/nm) and re-run." >&2
  exit 1
}

# Which image is being flashed, so the gutted-image guard (below) applies the right code floor and
# required-symbol set. A release image is built per chip (specs/hal.md section 6): CHIP names the
# chip, the image is `mainboard-$CHIP`, and its guards come from tools/image-guard-syms.sh, keyed by
# the image name its Cargo.toml table gives. The imu-bench validator is not a release image and keeps
# its own profile below, selected with IMAGE_PROFILE=imu-bench. The integrated firmware is ~54 KB of
# code with the control-stack symbols; the imu-bench validator is a legitimately small ~18 KB .text
# image with only IMU_BENCH_OBS. A single integrated-tuned guard rejects the legit imu-bench image
# (round 9), so the profile selects the set.
# FAIL-CLOSED, in three ways, none of which can select a weaker guard set:
#   - neither CHIP nor IMAGE_PROFILE given is refused. There is no default chip: an F103 board and an
#     F130 board run different images, and the wrong one drives the wrong register model.
#   - an UNKNOWN chip or profile is refused outright by the `*)` arm below rather than defaulted.
#   - CHIP and IMAGE_PROFILE that name different images are refused rather than one winning.
# The CHIP given and the chip the ELF was built for are not compared by name, but each image's
# required symbols carry its chip's vector names (image-guard-syms.sh), so the other chip's ELF fails
# the required-symbol guard. Which chip the BOARD carries is not checked here (specs/hal.md section 7).
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CHIP="${CHIP:-}"
IMAGE_PROFILE="${IMAGE_PROFILE:-}"
if [ -n "$CHIP" ]; then
  if [ -n "$IMAGE_PROFILE" ] && [ "$IMAGE_PROFILE" != "mainboard-$CHIP" ]; then
    echo "flash: REFUSED - CHIP=$CHIP names the image mainboard-$CHIP, but IMAGE_PROFILE=$IMAGE_PROFILE. Give one of them." >&2
    exit 2
  fi
  IMAGE_PROFILE="mainboard-$CHIP"
elif [ -z "$IMAGE_PROFILE" ]; then
  echo "flash: REFUSED - no image named. Set CHIP=gd32f103c8|gd32f130c8 for a release image" >&2
  echo "flash: (mainboard-<chip>), or IMAGE_PROFILE=imu-bench for the IMU bench validator." >&2
  echo "flash: there is no default chip: each chip has its own image and its own guards." >&2
  exit 2
fi
. "$SCRIPT_DIR/image-guard-syms.sh"
case "$IMAGE_PROFILE" in
  imu-bench)
    # ~18 KB healthy (full-LTO Mahony/CORDIC); the one SWD-readable block the validator publishes.
    # Floor well below 18 KB but far above a gutted few-KB image.
    PROFILE_TEXT_FLOOR=8000
    PROFILE_REQ_SYMS='B IMU_BENCH_OBS$' ;;
  *)
    # A release image: its floor and both symbol lists live in tools/image-guard-syms.sh, which is
    # the ONE copy CI's build-time guard, the bundle packer and the flash-guard harness read as well,
    # and which carries the reason every entry is on them. They were literals here and in ci.yml
    # until 2026-10-24 and had already drifted apart.
    if ! image_guards "$IMAGE_PROFILE"; then
      echo "flash: REFUSED - '$IMAGE_PROFILE' is not an image named in tools/image-guard-syms.sh (CHIP=gd32f103c8|gd32f130c8) and not imu-bench." >&2; exit 2
    fi
    PROFILE_TEXT_FLOOR="$IMAGE_GUARD_FLOOR"
    PROFILE_REQ_SYMS="$IMAGE_GUARD_REQ_SYMS"
    PROFILE_HOT_SYMS="$IMAGE_GUARD_HOT_SYMS" ;;
esac

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
  #
  # The `file format` line is dropped before the match, and that is not cosmetic: objdump -d opens
  # its output with `<image-path>:     file format elf32-littlearm`, so the PATH is inside the text
  # being scanned. `grep -iqw wfi` then matched any image whose path contained the word - proven with
  # one image under two names, `wfi.elf` refused and `clean.elf` clean, same bytes, no wfi in either.
  # It failed safe (a spurious refusal, never a spurious pass), but it also meant the harness's wfi
  # cases were passing on the FILENAME, so the instruction scan itself was barely exercised. Dropping
  # the line by CONTENT rather than by position: an instruction line cannot contain "file format",
  # while a header that ever moved off line 1 would still be removed.
  WFI_CMD=$(printf 'for c in arm-none-eabi-objdump llvm-objdump rust-objdump objdump; do command -v "$c" >/dev/null 2>&1 && { out=$("$c" -d %q 2>/dev/null) || exit 3; [ -n "$out" ] || exit 3; printf "%%s" "$out" | grep -v "file format" | grep -iqw wfi; exit $?; }; done; exit 2' "$IMG")
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
# resolved and exercised on the guard's own host before the guards ran, so reaching that branch
# usually means the toolchain changed under the run, which is never a routine condition. It is not the
# only way in: an image whose code sits in neither `.text` nor `.hotcode` also lands here, with a
# working `size`. Both are refusals, which is the point of the branch.
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
  # first 32 KiB, so a `.text`-only floor sees a healthy image as gutted. Sum the two code sections
  # this project emits, BY NAME: `.text` and `.hotcode`. An image whose code lives in neither sums to
  # nothing and is refused by the emptiness check below - fail-closed, but note that the refusal then
  # reads as a broken `size`, which is not the only way to reach it.
  text=$("$size_tool" -A "$ELF" 2>/dev/null | awk '$1==".text" || $1==".hotcode" {t+=$2} END{if(t>0) print t}')
  if [ -z "$text" ]; then echo "flash: no .text/.hotcode size on $HOST_LABEL (does this size accept -A, and does this image use those section names?)" >&2; exit 2; fi
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
