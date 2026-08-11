#!/usr/bin/env bash
# Read the stack high-water instrument over SWD and print the margin in bytes.
#
# The firmware paints the free stack with a known word at boot and publishes, as CTRL_OBS word 30,
# how much of that paint is still intact (`specs/firmware.md`, "RAM/stack budget";
# `crates/firmware/src/main.rs`, `mod stack_paint`). This reads it. The number it prints is the gap
# between the deepest word the running image has ever touched and the top of `.uninit`, i.e. how
# close the board has come to overwriting CTRL_OBS itself with a stack frame.
#
# Usage:
#   tools/stack-margin.sh [--board master|slave|offroad-master|offroad-slave] [--elf <path>]
#                         [--watch <seconds>]
#   tools/stack-margin.sh --selftest      # decode self-check, no hardware, no lock
#
# READ-ONLY, and deliberately so. It runs `mdw` in an `init`/`shutdown` session and NEVER halts the
# core, so it does not come under the standing armed-halt rule the way tools/flash.sh does (that rule
# governs tools that halt; tools/armed-guard-verdict.sh is its single owner and is not consulted here
# because nothing here can halt). It is therefore safe on an armed, driving board, which matters: the
# margin under load is the only margin worth measuring, and a tool that had to disarm first could
# only ever read an idle board.
#
# It takes the bench lock like every other hardware tool here: an OpenOCD attach is a physical bench
# op whether or not it writes anything, and two attaches at once garble SWD.
set -euo pipefail

PI="${BENCH_PI:-pi@192.168.0.248}"

# The offroad pair needs the PATCHED OpenOCD (the elaphureLink CMSIS-DAP-over-TCP driver); the stock
# openocd has no such interface. Built on THIS host, with its tcl tree beside the binary.
OFFROAD_OCD_BIN="${OFFROAD_OCD_BIN:-$HOME/dev/openocd-elaphurelink/openocd/src/openocd}"
OFFROAD_OCD_TCL="${OFFROAD_OCD_TCL:-$HOME/dev/openocd-elaphurelink/openocd/tcl}"

CTRL_OBS_MAGIC=4c525443              # "CTRL" little-endian
STACK_WORD=30                        # CTRL_OBS word 30, byte offset 0x78 (the offset-preserving append)
NEED=$(( (STACK_WORD + 1) * 4 ))     # 124 B: the block must be at least this long to carry it
# Floor policy: >= 250 B = a 32 B exception frame + the deepest ISR chain + headroom
# (specs/firmware.md, "RAM/stack budget"). Below it, one more handler frame lands in CTRL_OBS.
MARGIN_FLOOR=250

BOARD="${BOARD:-master}"
ELF="${ELF:-target/thumbv7m-none-eabi/release/firmware}"
WATCH=0
SELFTEST=0
while [ $# -gt 0 ]; do
  case "$1" in
    --board) BOARD="${2:?--board needs master|slave|offroad-master|offroad-slave}"; shift 2 ;;
    --elf)   ELF="${2:?--elf needs a path}"; shift 2 ;;
    --watch) WATCH="${2:?--watch needs seconds}"; shift 2 ;;
    --selftest) SELFTEST=1; shift ;;
    -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
    *) echo "stack-margin: unknown argument '$1'" >&2; exit 2 ;;
  esac
done

# Run a shell command string on whichever host owns the probe. One string, one shell, both branches,
# so the read is the same text either way and the two runners cannot drift apart (tools/flash.sh).
target_sh() {
  case "$RUNNER" in
    pi)    ssh "$PI" "$1" ;;
    local) bash -c "$1" ;;
  esac
}

# One attach, one read, no halt. `mdw <addr> <n>` prints four words per line after an address
# prefix; take hex only AFTER the "0x...:" prefix so the address itself is never counted as data.
read_obs() {
  local raw
  raw=$(target_sh "$TIMEOUT 30 ${SUDO}$OCD_BIN $OC_CFG -c init -c 'mdw 0x$OBS_ADDR $READ_WORDS' -c shutdown 2>&1") || return 1
  printf '%s\n' "$raw" | sed -n 's/^0x[0-9a-f]*: *//p' | tr ' ' '\n' | grep -E '^[0-9a-f]{8}$' || true
}

# Decode one read and print the verdict. Exit 0 = read and interpreted, 1 = the read did not happen
# or is not trustworthy, 3 = a MEASURED margin below the floor.
report_once() {
  local magic w30 free painted boot ticks line
  # Not `mapfile`: this host runs bash 3.2 (macOS), which does not have it, and the LOCAL runner
  # executes here. A bash-4 builtin would make the offroad path fail at the read with an obscure
  # "command not found" after the bench lock was already taken.
  local words
  words=()
  while IFS= read -r line; do words+=("$line"); done < <(read_obs)
  if [ "${#words[@]}" -lt "$((STACK_WORD + 1))" ]; then
    echo "stack-margin: FAILED - read ${#words[@]} words, needed $((STACK_WORD + 1))." >&2
    echo "stack-margin: the probe did not answer, or answered short. Nothing is concluded." >&2
    return 1
  fi
  magic="${words[0]}"
  if [ "$magic" != "$CTRL_OBS_MAGIC" ]; then
    echo "stack-margin: FAILED - CTRL_OBS magic is 0x$magic, expected 0x$CTRL_OBS_MAGIC." >&2
    echo "stack-margin: the block is not live: either the board has not reached its first publish," >&2
    echo "stack-margin: or this ELF does not match the image on the board. Nothing is concluded." >&2
    return 1
  fi
  boot=$((16#${words[1]}))
  ticks=$((16#${words[2]}))
  w30=$((16#${words[$STACK_WORD]}))
  free=$(( w30 & 0xFFFF ))
  painted=$(( w30 >> 16 ))

  # boot_count is printed with the margin, not as a footnote: the paint is laid down ONCE per boot,
  # so a board that reset during the soak has thrown away the deep chain the soak was accumulating
  # and the margin below it describes only the time since that reset.
  printf 'stack-margin: boot_count=%d  tick_count=%d (~%d s at 250 Hz)\n' "$boot" "$ticks" "$((ticks / 250))"
  printf 'stack-margin: MARGIN = %d B free of %d B painted\n' "$free" "$painted"

  # The readings that are not measurements, called out rather than left to be misread as one.
  if [ "$painted" -eq 0 ]; then
    echo "stack-margin: INCONCLUSIVE - nothing was painted (word 30 is zero)."
    echo "stack-margin: either no sweep has completed yet, or the statics have grown until there is"
    echo "stack-margin: no free stack left to paint. Re-read; if it stays zero, that is the answer."
    return 0
  fi
  if [ "$free" -eq "$painted" ]; then
    echo "stack-margin: the paint was NEVER REACHED, so this is a LOWER BOUND, not a measurement:"
    echo "stack-margin: the margin is at least ${painted} B. Load the board (flood the link, connect"
    echo "stack-margin: a phone, configure the IMU) and re-read."
    return 0
  fi
  if [ "$free" -lt "$MARGIN_FLOOR" ]; then
    echo "stack-margin: BELOW THE >= ${MARGIN_FLOOR} B FLOOR. One more handler frame lands in .uninit/CTRL_OBS."
    return 3
  fi
  echo "stack-margin: above the >= ${MARGIN_FLOOR} B floor."
  return 0
}

# ================================ OFFLINE SELF-TEST ==========================================
# Exercises the decode and every verdict branch against canned `mdw` output, with no probe, no ELF
# and no bench lock. It runs the REAL read_obs parser (only the transport is stubbed), so the thing
# under test is the same sed/tr/grep pipeline and the same arithmetic the hardware path uses.
if [ "$SELFTEST" = 1 ]; then
  FAILS=0
  # A canned OpenOCD `mdw` transcript: 31 words, four per line, with the address prefix the real
  # tool emits (and the leading Info: chatter the parser must ignore).
  canned_mdw() {
    local w30="$1" magic="${2:-$CTRL_OBS_MAGIC}" boot="${3:-00000001}" ticks="${4:-0000fa00}"
    local i out=""
    local ws=("$magic" "$boot" "$ticks")
    for i in $(seq 3 29); do ws+=("000000$(printf '%02x' "$i")"); done
    ws+=("$w30")
    echo "Info : SWD DPIDR 0x1ba01477"
    echo "Info : [stm32f1x.cpu] Cortex-M3 r2p1 processor detected"
    local addr=0x20000e08
    for i in $(seq 0 4 30); do
      printf '%s: %s %s %s %s \n' "$addr" "${ws[$i]}" "${ws[$((i+1))]:-00000000}" "${ws[$((i+2))]:-00000000}" "${ws[$((i+3))]:-00000000}"
    done
  }
  expect() { # expect <label> <wanted-rc> <grep-pattern> ; reads report_once output on stdin
    local label="$1" want="$2" pat="$3" got_rc out
    out=$(report_once 2>&1) && got_rc=0 || got_rc=$?
    if [ "$got_rc" != "$want" ]; then
      echo "SELFTEST FAIL [$label]: rc $got_rc, wanted $want"; echo "$out" | sed 's/^/    /'; FAILS=$((FAILS+1)); return
    fi
    if ! printf '%s\n' "$out" | grep -qE "$pat"; then
      echo "SELFTEST FAIL [$label]: output did not match /$pat/"; echo "$out" | sed 's/^/    /'; FAILS=$((FAILS+1)); return
    fi
    echo "selftest ok: $label"
  }
  READ_WORDS=31; OBS_ADDR=20000e08

  # A measured margin well above the floor: free 0x0198 = 408 B, painted 0x0A54 = 2644 B.
  read_obs() { canned_mdw "0a540198" | sed -n 's/^0x[0-9a-f]*: *//p' | tr ' ' '\n' | grep -E '^[0-9a-f]{8}$'; }
  expect "measured margin above the floor" 0 "MARGIN = 408 B free of 2644 B painted"
  expect "above-floor verdict"             0 "above the >= 250 B floor"

  # A measured margin BELOW the floor: free 0x0018 = 24 B (the round-9 number), painted 2644.
  read_obs() { canned_mdw "0a540018" | sed -n 's/^0x[0-9a-f]*: *//p' | tr ' ' '\n' | grep -E '^[0-9a-f]{8}$'; }
  expect "below-floor margin is rc 3" 3 "BELOW THE >= 250 B FLOOR"

  # free == painted: the paint was never reached, a LOWER BOUND and not a measurement.
  read_obs() { canned_mdw "0a540a54" | sed -n 's/^0x[0-9a-f]*: *//p' | tr ' ' '\n' | grep -E '^[0-9a-f]{8}$'; }
  expect "unreached paint is a lower bound" 0 "LOWER BOUND"

  # Nothing painted / no sweep yet.
  read_obs() { canned_mdw "00000000" | sed -n 's/^0x[0-9a-f]*: *//p' | tr ' ' '\n' | grep -E '^[0-9a-f]{8}$'; }
  expect "zero word is inconclusive" 0 "INCONCLUSIVE"

  # A wrong magic must conclude NOTHING, whatever word 30 happens to hold. This is the branch the
  # real offroad read exercised on silicon (the running image predates the field, so the address
  # resolved from the new ELF is live-but-unrelated RAM).
  read_obs() { canned_mdw "0a540198" "00000000" | sed -n 's/^0x[0-9a-f]*: *//p' | tr ' ' '\n' | grep -E '^[0-9a-f]{8}$'; }
  expect "wrong magic concludes nothing" 1 "magic is 0x00000000"

  # A short read must fail rather than index off the end of the array.
  read_obs() { printf '%s\n' 4c525443 00000001 00000002; }
  expect "short read concludes nothing" 1 "read 3 words, needed 31"

  # A read that produced no words at all (a dead probe).
  read_obs() { printf '%s\n' ""; }
  expect "empty read concludes nothing" 1 "needed 31"

  # boot_count is surfaced, because a reset mid-soak discards the paint the soak was accumulating.
  read_obs() { canned_mdw "0a540198" "$CTRL_OBS_MAGIC" "00000007" "0007a120" | sed -n 's/^0x[0-9a-f]*: *//p' | tr ' ' '\n' | grep -E '^[0-9a-f]{8}$'; }
  expect "boot_count and uptime are reported" 0 "boot_count=7  tick_count=500000 \(~2000 s"

  if [ "$FAILS" -gt 0 ]; then echo "stack-margin: SELFTEST FAILED ($FAILS)"; exit 1; fi
  echo "stack-margin: selftest passed"
  exit 0
fi

# ============================ BOARD -> TRANSPORT AND RUNNER =================================
# Lifted in shape from tools/flash.sh: RUNNER is a physical fact about the board, set here in one
# place, never probed for at runtime. The bench master/slave sit on USB ST-Link clones plugged into
# the Pi and are simply not visible from this host; the offroad pair sits on LAN-attached ESP32-C3
# elaphureLink probes that the Pi is not in the signal path for. There is deliberately no "Pi
# unreachable, fall back to local": for master/slave a local fallback would attach to nothing, or to
# some other probe on this host, while printing the same reassuring output.
case "$BOARD" in
  master)
    RUNNER=pi
    OC_CFG="-f interface/stlink.cfg -c 'transport select dapdirect_swd' -c 'adapter usb location 1-1.2.4' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  slave)
    RUNNER=pi
    OC_CFG="-c 'adapter driver cmsis-dap' -c 'cmsis-dap backend usb_bulk' -c 'cmsis-dap vid_pid 0x1209 0xda42' -c 'transport select swd' -c 'adapter speed 1000' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  offroad-master)
    RUNNER=local
    OCD_BIN="$OFFROAD_OCD_BIN -s $OFFROAD_OCD_TCL"
    OC_CFG="-f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.171' -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  offroad-slave)
    RUNNER=local
    OCD_BIN="$OFFROAD_OCD_BIN -s $OFFROAD_OCD_TCL"
    OC_CFG="-f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.195' -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg" ;;
  *) echo "stack-margin: --board must be master|slave|offroad-master|offroad-slave (got '$BOARD')" >&2; exit 2 ;;
esac

# STACK_RUNNER asserts the runner, it does not choose it: setting it to something the board cannot
# use is refused loudly rather than quietly attempted (tools/flash.sh's FLASH_RUNNER contract).
if [ -n "${STACK_RUNNER:-}" ] && [ "$STACK_RUNNER" != "$RUNNER" ]; then
  echo "stack-margin: REFUSED - STACK_RUNNER='$STACK_RUNNER' but BOARD=$BOARD runs on '$RUNNER'." >&2
  case "$RUNNER" in
    pi)    echo "stack-margin: the bench $BOARD sits on a USB ST-Link clone plugged into $PI; that" >&2
           echo "stack-margin: probe is not visible from this host, so there is no local path." >&2 ;;
    local) echo "stack-margin: the offroad probes are LAN-attached ESP32-C3s driven by the patched" >&2
           echo "stack-margin: OpenOCD on this host. The Pi is not in their signal path." >&2 ;;
  esac
  exit 2
fi

case "$RUNNER" in
  pi)
    OCD_BIN="${OCD_BIN:-openocd}"   # the bench ST-Links use the Pi's system openocd
    SUDO="sudo "                    # which needs root for the USB probe device nodes
    TIMEOUT="timeout"               # GNU coreutils, always on the Pi
    HOST_LABEL="the Pi"
    PROBE_LABEL="ST-Link" ;;
  local)
    SUDO=""                         # the patched OpenOCD reaches the ESP32 probes over TCP 3240
                                    # unprivileged: no USB device node for root to open.
    HOST_LABEL="this host"
    PROBE_LABEL="the elaphureLink probe"
    if [ ! -x "${OCD_BIN%% *}" ]; then
      echo "stack-margin: REFUSED - patched OpenOCD not found or not executable at '${OCD_BIN%% *}'." >&2
      echo "stack-margin: BOARD=$BOARD needs the elaphureLink build (the stock openocd has no such" >&2
      echo "stack-margin: interface). Build it, or point OFFROAD_OCD_BIN/OFFROAD_OCD_TCL at it." >&2
      exit 2
    fi
    if [ ! -d "$OFFROAD_OCD_TCL" ]; then
      echo "stack-margin: REFUSED - OpenOCD tcl tree missing at '$OFFROAD_OCD_TCL'." >&2
      exit 2
    fi
    # A hung OpenOCD must not sit on the bench lock forever. macOS ships no coreutils `timeout`;
    # perl's alarm(2) survives exec and is the portable stand-in. The trailing `exit 127` is
    # load-bearing: perl's exec RETURNS on failure, and without it a launch that never happened
    # would exit 0 and read as a read that succeeded.
    if command -v timeout >/dev/null 2>&1; then TIMEOUT="timeout"
    elif command -v gtimeout >/dev/null 2>&1; then TIMEOUT="gtimeout"
    elif command -v perl >/dev/null 2>&1; then TIMEOUT="perl -e 'alarm shift; exec @ARGV; exit 127'"
    else
      echo "stack-margin: REFUSED - no timeout, gtimeout or perl on $HOST_LABEL to cap a hung OpenOCD." >&2
      exit 2
    fi ;;
esac

# ============================== RESOLVE CTRL_OBS FROM THE ELF ================================
# The address is resolved from the ELF every run, never hardcoded: it moves whenever the statics
# move, which is exactly what this instrument exists to track. The SIZE is checked too, because
# `mdw` reads MEMORY rather than the struct, so an ELF that predates this field returns plausible
# garbage for word 30 rather than an error (the tools/imu-tilt.py precedent).
if [ ! -r "$ELF" ]; then
  echo "stack-margin: REFUSED - cannot read ELF '$ELF'." >&2
  echo "stack-margin: build it with \`cargo image\` or pass --elf <path>." >&2
  exit 2
fi
nm_tool=""
for c in arm-none-eabi-nm llvm-nm rust-nm nm; do
  command -v "$c" >/dev/null 2>&1 && { nm_tool="$c"; break; }
done
if [ -z "$nm_tool" ]; then
  echo "stack-margin: REFUSED - no nm on this host; cannot resolve CTRL_OBS from the ELF." >&2
  exit 2
fi
# `nm -S` prints "<value> <size> <type> <name>". Fail closed on a symbol that does not resolve
# rather than reading address 0 and reporting a confident number about nothing.
SYM_LINE=$("$nm_tool" -S "$ELF" 2>/dev/null | awk '$4=="CTRL_OBS"{print; exit}')
if [ -z "$SYM_LINE" ]; then
  echo "stack-margin: REFUSED - no CTRL_OBS symbol with a size in '$ELF'." >&2
  echo "stack-margin: is this the firmware image? (\`$nm_tool -S \"$ELF\" | grep CTRL_OBS\`)" >&2
  exit 2
fi
OBS_ADDR=$(printf '%s' "$SYM_LINE" | awk '{print $1}')
OBS_SIZE=$((16#$(printf '%s' "$SYM_LINE" | awk '{print $2}')))
if [ "$OBS_SIZE" -lt "$NEED" ]; then
  echo "stack-margin: REFUSED - CTRL_OBS in '$ELF' is ${OBS_SIZE} B, short of the ${NEED} B that" >&2
  echo "stack-margin: carries word ${STACK_WORD}. This ELF predates the stack instrument, so word" >&2
  echo "stack-margin: ${STACK_WORD} would be whatever happens to sit past the end of the block." >&2
  exit 2
fi
READ_WORDS=$(( OBS_SIZE / 4 ))

# ================================== THE BENCH LOCK ==========================================
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOCK="$SCRIPT_DIR/bench-lock.sh"
OWNER="${BENCH_OWNER:-stack-margin@$(hostname -s 2>/dev/null || echo host)}"
TOOK_LOCK=0
if acq="$("$LOCK" acquire "$OWNER" "stack-margin: read $BOARD")"; then
  case "$acq" in
    ACQUIRED*) TOOK_LOCK=1 ;;   # we took it here, so we release it on exit
    *) TOOK_LOCK=0 ;;           # ALREADY-OURS: held for a session, leave it held
  esac
else
  echo "stack-margin: bench is busy, not reading." >&2
  echo "$acq" >&2
  echo "stack-margin: wait, or coordinate; see $LOCK status" >&2
  exit 1
fi
trap '[ "$TOOK_LOCK" = 1 ] && "$LOCK" release "$OWNER" >/dev/null 2>&1 || true' EXIT

echo "stack-margin: board=$BOARD runner=$RUNNER via $PROBE_LABEL on $HOST_LABEL"
echo "stack-margin: CTRL_OBS=0x$OBS_ADDR size=${OBS_SIZE} B (${READ_WORDS} words) from $(basename "$ELF")"

if [ "$WATCH" -gt 0 ]; then
  # Repeat until the margin stops moving, the way the round-9/11b soaks were read: a mark that is
  # still deepening has not found the deep chain yet, and the deep chain is the whole point.
  end=$(( $(date +%s) + WATCH ))
  while [ "$(date +%s)" -lt "$end" ]; do
    report_once || true
    "$LOCK" refresh "$OWNER" >/dev/null 2>&1 || true
    sleep 5
  done
  echo "stack-margin: watch window done; the LAST line above is the reading that counts."
else
  report_once
fi
