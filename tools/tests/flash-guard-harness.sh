#!/usr/bin/env bash
# Adversarial proof for tools/flash.sh's image guards, on the LOCAL runner.
#
# Every guard in flash.sh is a refusal, and a refusal is only real if it is driven with the input it
# exists to refuse. This harness fabricates that input (real ARM ELFs: a wfi image, an LTO-gutted
# image, an image missing required symbols, an image whose hot-path symbol was evicted past the
# zero-wait boundary) and drives the REAL flash.sh over it, asserting both the exit code AND that
# nothing reached a probe.
#
# Nothing here can touch hardware: bench-lock.sh, ssh, scp, sudo, timeout and openocd are all stubbed
# in a temp dir, the tools tree is COPIED there with the lock replaced, and every run is
# FLASH_DRY_RUN=1. The stub logs are the evidence: a case that must refuse asserts zero openocd
# invocations, and the pre-lock cases assert zero lock acquisitions, which is what makes "the refusal
# fires before the bench lock is taken" a checked fact rather than a reading of the source.
#
# The armed-bridge verdict itself is proved separately and exhaustively by
# tools/tests/armed-guard-verdict-test.sh; the fabricated sessions here prove the wiring, i.e. that
# flash.sh's strengthened read (two CCHP reads plus the CPUID canary, one session) reaches the
# verdict intact and that a bogus all-zero link cannot present as the "MOE clear for free" case.
#
# Usage: tools/tests/flash-guard-harness.sh   (exit 0 = every case as expected)
set -u

FLASH="$(cd "$(dirname "$0")/.." && pwd)"
H="$(mktemp -d "${TMPDIR:-/tmp}/flash-guard-harness.XXXXXX")"
trap 'rm -rf "$H"' EXIT
PASS=0; FAIL=0

command -v arm-none-eabi-as >/dev/null 2>&1 || { echo "SKIP: no arm-none-eabi-as; the fabricated images need it"; exit 0; }
ARM_BIN="$(dirname "$(command -v arm-none-eabi-as)")"

# ---------------------------------------------------------------- fabricated images
REQ="main SysTick usart1_rx_isr dma_rx_isr x5probe3run x5probe12probe_family x5probe15probe_candidate \
x5probe13probe_present x5probe14measure_counts x5probe15scratch_present x5motor2hw10period_isr \
x5motor2hw5MOTOR x5motor7PERIODS x5motor9OBS_STATE x5motor7OBS_CAL x3arm2hw4GATE x3arm2hw5ARMED"
HOT="x12service_loop control_task_cb input_task_cb x9run_shell x7adc_isr x15systick_handler \
systick_tick_cb route_emits route_handback"

mkimg() {  # mkimg <name> <div-instruction> <div-placement: hot|cold|none> <padding>
  local n="$1" ins="$2" place="$3" pad="$4" f="$H/$1.s"
  { echo '  .syntax unified'; echo '  .cpu cortex-m3'; echo '  .thumb'; echo '  .section .text'; } > "$f"
  if [ "$n" != gutted ] && [ "$n" != missing ]; then
    for s in $REQ $HOT; do printf '  .global %s\n  .thumb_func\n%s:\n  nop\n' "$s" "$s" >> "$f"; done
  else
    printf '  .global main\n  .thumb_func\nmain:\n  nop\n' >> "$f"
  fi
  [ "$place" = hot ] && printf '  .global x4base5fixed3div\n  .thumb_func\nx4base5fixed3div:\n  %s\n' "$ins" >> "$f"
  printf '  .space %s\n' "$pad" >> "$f"
  [ "$place" = cold ] && printf '  .global x4base5fixed3div\n  .thumb_func\nx4base5fixed3div:\n  nop\n' >> "$f"
  if [ "$n" != gutted ] && [ "$n" != missing ]; then
    echo '  .section .bss' >> "$f"
    for s in CTRL_OBS INJECT_UART_LINE_ERROR; do printf '  .global %s\n%s:\n  .space 4\n' "$s" "$s" >> "$f"; done
  fi
  arm-none-eabi-as -mcpu=cortex-m3 -mthumb "$f" -o "$H/$n.o" || return 1
  arm-none-eabi-ld -Ttext=0x08000000 --section-start=.bss=0x20000000 -e main "$H/$n.o" -o "$H/$n.elf"
}
mkimg good    nop hot  41000 || { echo "FAIL: could not build the fabricated images"; exit 1; }
mkimg wfi     wfi hot  41000
mkimg cold    nop cold 41000   # the hot-path symbol evicted past 0x08008000
mkimg gutted  nop none 120     # LTO ate it
mkimg missing nop none 41000   # over the floor, no required symbols
head -c 4096 /dev/urandom > "$H/corrupt.elf"   # not an ELF at all

# ---------------------------------------------------------------- stubs
mkdir -p "$H/stub" "$H/log" "$H/tcl" "$H/minbin"
cat > "$H/stub/bench-lock.sh" <<'EOF'
#!/usr/bin/env bash
printf 'LOCK %s\n' "$*" >> "$HARNESS_LOG/lock.log"
case "$1" in acquire) echo "ACQUIRED: $2 (stub)";; esac
EOF
cat > "$H/stub/ssh" <<'EOF'
#!/usr/bin/env bash
printf 'SSH %s\n' "$*" >> "$HARNESS_LOG/ssh.log"
shift
exec bash -c "$*"
EOF
cat > "$H/stub/scp" <<'EOF'
#!/usr/bin/env bash
printf 'SCP %s\n' "$*" >> "$HARNESS_LOG/scp.log"
src=""; dst=""
for a in "$@"; do case "$a" in -*) ;; *:*) dst="${a#*:}";; *) src="$a";; esac; done
cp "$src" "$dst"
EOF
cat > "$H/stub/sudo" <<'EOF'
#!/usr/bin/env bash
exec "$@"
EOF
cat > "$H/stub/timeout" <<'EOF'
#!/usr/bin/env bash
shift
exec "$@"
EOF
# The probe. HARNESS_CCHP / HARNESS_CCHP2 / HARNESS_CPUID fabricate a session; unset means a healthy
# disarmed board answering the values measured on the offroad pair 2026-08-11.
cat > "$H/stub/openocd" <<'EOF'
#!/usr/bin/env bash
printf 'OCD %s\n' "$*" >> "$HARNESS_LOG/openocd.log"
n=0
for a in "$@"; do
  case "$a" in
    "mdw 0x40012C44 1")
      n=$((n+1))
      if [ "$n" = 1 ]; then v="${HARNESS_CCHP-00000c1c}"; else v="${HARNESS_CCHP2-${HARNESS_CCHP-00000c1c}}"; fi
      [ -n "$v" ] && echo "0x40012c44: $v " ;;
    "mdw 0xE000ED00 1") v="${HARNESS_CPUID-412fc231}"; [ -n "$v" ] && echo "0xe000ed00: $v " ;;
    program*) printf 'PROGRAM %s\n' "$a" >> "$HARNESS_LOG/program.log"; echo "** Verified OK **" ;;
  esac
done
EOF
chmod +x "$H/stub"/*
# A PATH with the ordinary utilities and NO binutils of any kind.
for u in bash sh perl sed grep awk head cat cp env tr hostname basename dirname printf mkdir rm wc; do
  p=$(command -v "$u" 2>/dev/null) && ln -sf "$p" "$H/minbin/$u"
done
# Shims that pass the pre-lock check ONCE and fail afterwards: the toolchain changing under the run,
# which is the only way to reach the in-guard fail-closed branches now that the pre-lock check exists.
mkdir -p "$H/vanish"
for t in size objdump; do
  cat > "$H/vanish/arm-none-eabi-$t" <<EOF
#!/usr/bin/env bash
c="$H/vanish/.$t.n"; n=\$(cat "\$c" 2>/dev/null || echo 0); echo \$((n+1)) > "\$c"
[ "\$n" = 0 ] && exec "$ARM_BIN/arm-none-eabi-$t" "\$@"
echo "arm-none-eabi-$t: command not found" >&2; exit 127
EOF
done
chmod +x "$H/vanish"/*

# The tools tree under test, with the bench lock replaced by the stub.
mkdir -p "$H/tools"
cp "$FLASH"/*.sh "$H/tools/" 2>/dev/null
cp "$H/stub/bench-lock.sh" "$H/tools/bench-lock.sh"
chmod +x "$H/tools"/*.sh

FULL_PATH="$ARM_BIN:$H/stub:/usr/bin:/bin:/usr/sbin:/sbin"
APPLE_PATH="$H/stub:/usr/bin:/bin:/usr/sbin:/sbin"       # Apple size exists and rejects -A
NONE_PATH="$H/stub:$H/minbin"                            # no binutils at all
VANISH_PATH="$H/vanish:$ARM_BIN:$H/stub:/usr/bin:/bin"

# ---------------------------------------------------------------- the case runner
# case_is <label> <want-rc> <want-locks> <want-openocd> <path> <board> <img> [env=val...]
case_is() {
  local label="$1" wrc="$2" wlock="$3" wocd="$4" path="$5" board="$6" img="$7"; shift 7
  rm -rf "$H/log"; mkdir -p "$H/log"
  local out rc
  out=$( export PATH="$path" HARNESS_LOG="$H/log"
         env "$@" BOARD="$board" FLASH_DRY_RUN=1 BENCH_OWNER=harness \
             OFFROAD_OCD_BIN="$H/stub/openocd" OFFROAD_OCD_TCL="$H/tcl" \
             bash "$H/tools/flash.sh" "$img" 2>&1 )
  rc=$?
  local lock ocd prog
  lock=$(grep -c acquire "$H/log/lock.log" 2>/dev/null || echo 0)
  ocd=$(grep -c . "$H/log/openocd.log" 2>/dev/null || echo 0)
  prog=$(grep -c . "$H/log/program.log" 2>/dev/null || echo 0)
  local why=""
  [ "$rc"   = "$wrc" ]   || why="$why rc=$rc(want $wrc)"
  [ "$lock" = "$wlock" ] || why="$why locks=$lock(want $wlock)"
  [ "$ocd"  = "$wocd" ]  || why="$why openocd=$ocd(want $wocd)"
  [ "$prog" = 0 ]        || why="$why PROGRAMMED=$prog"
  if [ -z "$why" ]; then
    printf 'PASS  %-56s rc=%s locks=%s ocd=%s\n' "$label" "$rc" "$lock" "$ocd"; PASS=$((PASS+1))
  else
    printf 'FAIL  %-56s%s\n' "$label" "$why"; printf '%s\n' "$out" | sed 's/^/        | /'; FAIL=$((FAIL+1))
  fi
}
expect_out() {  # expect_out <label> <regex> <path> <board> <img> [env=val...]
  local label="$1" re="$2" path="$3" board="$4" img="$5"; shift 5
  rm -rf "$H/log"; mkdir -p "$H/log"
  local out
  out=$( export PATH="$path" HARNESS_LOG="$H/log"
         env "$@" BOARD="$board" FLASH_DRY_RUN=1 BENCH_OWNER=harness \
             OFFROAD_OCD_BIN="$H/stub/openocd" OFFROAD_OCD_TCL="$H/tcl" \
             bash "$H/tools/flash.sh" "$img" 2>&1 )
  if printf '%s' "$out" | grep -qE "$re"; then
    printf 'PASS  %-56s matched /%s/\n' "$label" "$re"; PASS=$((PASS+1))
  else
    printf 'FAIL  %-56s no match for /%s/\n' "$label" "$re"; printf '%s\n' "$out" | sed 's/^/        | /'; FAIL=$((FAIL+1))
  fi
}

echo "== flash.sh image guards: adversarial input, local runner =="
echo "-- with the cross toolchain present (locks=1: the guards run under the bench lock)"
case_is "wfi image refused"                    1 1 0 "$FULL_PATH" offroad-master "$H/wfi.elf"
case_is "gutted image refused (below floor)"   1 1 0 "$FULL_PATH" offroad-master "$H/gutted.elf"
case_is "missing required symbols refused"     1 1 0 "$FULL_PATH" offroad-master "$H/missing.elf"
case_is "hot symbol above 0x08008000 refused"  1 1 0 "$FULL_PATH" offroad-master "$H/cold.elf"
case_is "healthy image passes to the read"     0 1 1 "$FULL_PATH" offroad-master "$H/good.elf"

echo "-- with the cross toolchain ABSENT: refuse, and BEFORE the lock (locks=0)"
for i in wfi gutted missing cold good; do
  case_is "no cross toolchain: $i refused pre-lock" 2 0 0 "$APPLE_PATH" offroad-master "$H/$i.elf"
done
case_is "no binutils at all: refused pre-lock"   2 0 0 "$NONE_PATH"  offroad-master "$H/good.elf"
case_is "unreadable non-ELF refused pre-lock"    2 0 0 "$FULL_PATH"  offroad-master "$H/corrupt.elf"
expect_out "the pre-lock refusal names the tool" "REFUSED - (no usable|'[^']*' cannot read)" \
  "$APPLE_PATH" offroad-master "$H/good.elf"

echo "-- a guard that cannot run must fail closed, not warn (toolchain changes under the run)"
rm -f "$H/vanish/.size.n" "$H/vanish/.objdump.n"
case_is "size vanishes mid-run: guard fails closed"     1 1 0 "$VANISH_PATH" offroad-master "$H/gutted.elf"
rm -f "$H/vanish/.size.n" "$H/vanish/.objdump.n"
expect_out "objdump broken: wfi scan does NOT read clean" "REFUSED - the wfi scan did not complete" \
  "$VANISH_PATH" offroad-master "$H/wfi.elf"

echo "-- the Pi runner keeps its audited warn-and-continue (its toolchain may genuinely be absent)"
case_is "pi: no cross toolchain warns and proceeds" 0 1 1 "$APPLE_PATH" master "$H/gutted.elf"
expect_out "pi: and says so"  "WARNING - could not read code-section size" "$APPLE_PATH" master "$H/gutted.elf"

echo "-- the strengthened armed-bridge read reaches the verdict intact"
case_is "armed bridge refused"                 1 1 1 "$FULL_PATH" offroad-master "$H/good.elf" HARNESS_CCHP=00008c1c
case_is "armed bridge, FORCE does not override" 1 1 1 "$FULL_PATH" offroad-master "$H/good.elf" HARNESS_CCHP=00008c1c FORCE_ARMED_GUARD=1
case_is "all-zero session refused (bogus link)" 1 1 1 "$FULL_PATH" offroad-master "$H/good.elf" HARNESS_CCHP=00000000 HARNESS_CPUID=00000000
case_is "transient zero then armed: refused"   1 1 1 "$FULL_PATH" offroad-master "$H/good.elf" HARNESS_CCHP=00000000 HARNESS_CCHP2=00008c1c
case_is "reads disagree: refused"              1 1 1 "$FULL_PATH" offroad-master "$H/good.elf" HARNESS_CCHP=00000c1c HARNESS_CCHP2=00000c1e
case_is "no read at all: refused"              1 1 1 "$FULL_PATH" offroad-master "$H/good.elf" HARNESS_CCHP= HARNESS_CCHP2=
case_is "genuine zero board with live canary"  0 1 1 "$FULL_PATH" offroad-master "$H/good.elf" HARNESS_CCHP=00000000
expect_out "the inconclusive hint is local, not the Pi" "pkill -x openocd .*THIS host" \
  "$FULL_PATH" offroad-master "$H/good.elf" HARNESS_CCHP=
expect_out "the Pi keeps the Pi hint"          "ssh pi@192\.168\.0\.248" \
  "$APPLE_PATH" master "$H/good.elf" HARNESS_CCHP=

echo "-- the wall-clock cap: a launch that never happened must not read as success"
t=$(bash -c "perl -e 'alarm shift; exec @ARGV; exit 127' 5 $H/no-such-openocd" 2>/dev/null; echo $?)
if [ "$t" != 0 ]; then printf 'PASS  %-56s rc=%s\n' "failed exec exits non-zero" "$t"; PASS=$((PASS+1))
else printf 'FAIL  %-56s rc=0\n' "failed exec exits non-zero"; FAIL=$((FAIL+1)); fi
t=$(bash -c "perl -e 'alarm shift; exec @ARGV; exit 127' 1 sleep 30" 2>/dev/null; echo $?)
if [ "$t" = 142 ]; then printf 'PASS  %-56s rc=%s\n' "the alarm still fires (SIGALRM)" "$t"; PASS=$((PASS+1))
else printf 'FAIL  %-56s rc=%s, wanted 142\n' "the alarm still fires (SIGALRM)" "$t"; FAIL=$((FAIL+1)); fi

echo "-- $PASS passed, $FAIL failed"
[ "$FAIL" = 0 ]
