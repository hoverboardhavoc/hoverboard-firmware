#!/usr/bin/env bash
# Adversarial proof for tools/flash.sh's image guards, on BOTH runners.
#
# Every guard in flash.sh is a refusal, and a refusal is only real if it is driven with the input it
# exists to refuse. This harness fabricates that input (real ARM ELFs: a wfi image, an LTO-gutted
# image, an image missing required symbols, an image whose hot-path symbol was evicted past the
# zero-wait boundary) and drives the REAL flash.sh over it, asserting both the exit code AND that
# nothing reached a probe.
#
# Nothing here can touch hardware: bench-lock.sh, ssh, scp, sudo, timeout and openocd are all stubbed
# in a temp dir, the tools tree is COPIED there with the lock replaced, and every run is
# FLASH_DRY_RUN=1. The stub logs are the evidence: an IMAGE-guard case that must refuse asserts zero
# openocd invocations, and the pre-lock cases assert zero lock acquisitions, which is what makes "the
# refusal fires before the bench lock is taken" a checked fact rather than a reading of the source.
# The armed-bridge refusals near the end are the exception, and necessarily so: the SWD read that
# triggers them IS an openocd invocation, so those assert one.
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
mkimg sleeper wfi hot  41000   # CONTAINS a wfi instruction; named so no case can pass on the path
mkimg wfi     nop hot  41000   # NO wfi instruction, but the path says wfi: the scan must read it clean
mkimg cold    nop cold 41000   # the hot-path symbol evicted past 0x08008000
mkimg gutted  nop none 120     # LTO ate it
mkimg missing nop none 41000   # over the floor, no required symbols
head -c 4096 /dev/urandom > "$H/corrupt.elf"   # not an ELF at all

# ---------------------------------------------------------------- stubs
mkdir -p "$H/stub" "$H/log" "$H/tcl" "$H/minbin"
# HARNESS_LOCK_RELEASE_FAIL fabricates a release that fails (a Pi that went away mid-run). The EXIT
# handler must survive it: still remove the staged image, still report the run's own status.
cat > "$H/stub/bench-lock.sh" <<'EOF'
#!/usr/bin/env bash
printf 'LOCK %s\n' "$*" >> "$HARNESS_LOG/lock.log"
case "$1" in
  acquire) echo "ACQUIRED: $2 (stub)";;
  release) [ -n "${HARNESS_LOCK_RELEASE_FAIL:-}" ] && exit 1;;
esac
exit 0
EOF
# The Pi. HARNESS_REMOTE_PATH is what makes "the tools run THERE, not here" a checked fact rather
# than a reading of the source: it is the PATH the remote command sees, independent of this host's.
cat > "$H/stub/ssh" <<'EOF'
#!/usr/bin/env bash
printf 'SSH %s\n' "$*" >> "$HARNESS_LOG/ssh.log"
shift
# HARNESS_SSH_RM_FAIL: the staged-image removal fails (dead ssh). A run that already programmed the
# board must not turn into a failure because its housekeeping did.
case "$*" in "rm -f"*) [ -n "${HARNESS_SSH_RM_FAIL:-}" ] && exit 255;; esac
[ -n "${HARNESS_REMOTE_PATH:-}" ] && export PATH="$HARNESS_REMOTE_PATH"
exec bash -c "$*"
EOF
# HARNESS_SCP_FAIL fabricates an unreachable Pi: the copy the guards read must fail closed.
cat > "$H/stub/scp" <<'EOF'
#!/usr/bin/env bash
printf 'SCP %s\n' "$*" >> "$HARNESS_LOG/scp.log"
[ -n "${HARNESS_SCP_FAIL:-}" ] && exit 1
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
# A PATH with the ordinary utilities and NO binutils of any kind. (`printf` resolves to the bash
# builtin rather than a path, so its link here is dangling and unused; the builtin is what runs.)
for u in bash sh perl sed grep awk head cat cp env tr hostname basename dirname printf mkdir rm wc; do
  p=$(command -v "$u" 2>/dev/null) && ln -sf "$p" "$H/minbin/$u"
done
# Shims that pass the pre-lock check ONCE and fail afterwards: the toolchain changing under the run,
# which is the only way to reach the in-guard fail-closed branches now that the pre-lock check exists.
#
# What is counted is any invocation carrying the FLAG the guard uses ($cflag: `size -A`,
# `objdump -d`), which is the pre-lock exercise's `-d` as well as the guard's own - two per run, not
# one. Counting the `-f` architecture probe too would spend this shim's single pass there: the run
# would refuse pre-lock and these cases would stop testing the in-guard branch they exist for. Not
# silently, as it happens - dropping the filter turns five cases red, loudly - but they would be
# testing a different thing than their names say. The claim each case makes is unchanged either way: the
# tool works during the pre-lock exercise and is broken by the time the guard reads the image.
mkdir -p "$H/vanish"
for t in size objdump; do
  case $t in size) cflag="-A" ;; objdump) cflag="-d" ;; esac
  cat > "$H/vanish/arm-none-eabi-$t" <<EOF
#!/usr/bin/env bash
case " \$* " in *" $cflag "*) ;; *) exec "$ARM_BIN/arm-none-eabi-$t" "\$@" ;; esac
c="$H/vanish/.$t.n"; n=\$(cat "\$c" 2>/dev/null || echo 0); echo \$((n+1)) > "\$c"
[ "\$n" = 0 ] && exec "$ARM_BIN/arm-none-eabi-$t" "\$@"
echo "arm-none-eabi-$t: command not found" >&2; exit 127
EOF
done
chmod +x "$H/vanish"/*
# One tool at a time, so a case pins the branch it claims to: with BOTH shimmed, objdump fails first
# at the wfi scan and the size branch is never reached. Each dir keeps its own counter.
for t in size objdump; do
  case $t in size) cflag="-A" ;; objdump) cflag="-d" ;; esac
  mkdir -p "$H/vanish-$t"
  cat > "$H/vanish-$t/arm-none-eabi-$t" <<EOF
#!/usr/bin/env bash
case " \$* " in *" $cflag "*) ;; *) exec "$ARM_BIN/arm-none-eabi-$t" "\$@" ;; esac
c="$H/vanish-$t/.n"; n=\$(cat "\$c" 2>/dev/null || echo 0); echo \$((n+1)) > "\$c"
[ "\$n" = 0 ] && exec "$ARM_BIN/arm-none-eabi-$t" "\$@"
echo "arm-none-eabi-$t: command not found" >&2; exit 127
EOF
  chmod +x "$H/vanish-$t/arm-none-eabi-$t"
done

# A disassembler that RUNS on the image and decodes it as the wrong instruction set: multi-target
# binutils on a non-arm host, the most realistic way the wfi guard could be present, resolve clean,
# and still not see a `wfi`. MODELLED, not observed: every objdump available here (GNU
# arm-none-eabi and Apple's llvm) reads the architecture from the ELF header and decodes Thumb
# correctly, so this stub is a hypothesis about a tool nobody here has produced. The other half of
# the guard - a disassembler that reports no architecture line at all - is not hypothetical, and is
# what makes the assertion worth its two lines either way. `-d` emits plausible output with no `wfi` mnemonic anywhere
# (so a scan that reaches it reports the image clean) and `-f` reports the architecture it really
# decoded. HARNESS_WRONG_ARCH is what `-f` says; unset means it says nothing at all, which is the
# tool that will not answer the question and must be refused just the same.
#
# The FILE FORMAT line says elf32-littlearm while the architecture line says something else, and that
# combination is the realistic one rather than a contrived one: the format is read out of the ELF
# header, which is genuinely an arm ELF, while the architecture is the ISA the tool actually selected
# to decode it with. It is also what makes these cases pin the `^architecture:` ANCHOR rather than
# merely the word: an unanchored `grep -qi arm` matches "elf32-littlearm" (and would match a build
# directory called /tmp/arm-build/, since -f echoes the path too), accepts this tool, and the guard
# is silently absent again. Drop the anchor from flash.sh and these cases go red.
#
# It is installed as plain `objdump` with no arm-none-eabi- sibling on the PATH, because that IS the
# host being modelled: the candidate order (arm-none-eabi-objdump, llvm-objdump, rust-objdump,
# objdump) has to fall through to it for this to be the tool the guards would use.
mkdir -p "$H/wrongisa"
cat > "$H/wrongisa/objdump" <<'EOF'
#!/usr/bin/env bash
img=""; mode=""
for a in "$@"; do case "$a" in -d) mode=d;; -f) mode=f;; -*) ;; *) img="$a";; esac; done
[ -r "$img" ] || exit 1
case "$mode" in
  d) printf '%s:     file format elf32-littlearm\n\nDisassembly of section .text:\n' "$img"
     printf '08000000 <main>:\n 8000000:\t90                   \tnop\n 8000001:\tc3                   \tret\n' ;;
  f) printf '\n%s:     file format elf32-littlearm\n' "$img"
     [ -n "${HARNESS_WRONG_ARCH:-}" ] && printf 'architecture: %s, flags 0x00000112:\n' "$HARNESS_WRONG_ARCH"
     printf 'start address 0x08000000\n' ;;
  *) exit 1 ;;
esac
EOF
# The same shape, but honest about being an arm decoder and able to see the wfi. Proves the new
# assertion accepts an architecture line it should (`armv7e-m`, not the bare `arm` llvm prints) and
# does not become the thing that refuses instead of the wfi scan.
cat > "$H/wrongisa/arm-objdump-stub" <<'EOF'
#!/usr/bin/env bash
img=""; mode=""
for a in "$@"; do case "$a" in -d) mode=d;; -f) mode=f;; -*) ;; *) img="$a";; esac; done
[ -r "$img" ] || exit 1
case "$mode" in
  d) printf '%s:     file format elf32-littlearm\n\n 8000000:\tbf30      \twfi\n' "$img" ;;
  f) printf '\n%s:     file format elf32-littlearm\narchitecture: armv7e-m, flags 0x00000112:\n' "$img" ;;
  *) exit 1 ;;
esac
EOF
chmod +x "$H/wrongisa"/*
mkdir -p "$H/armstub" && cp "$H/wrongisa/arm-objdump-stub" "$H/armstub/objdump" && chmod +x "$H/armstub/objdump"
# size and nm still have to resolve, or the run refuses on those instead and the case would pass for
# the wrong reason. Real cross tools, reached through these dirs, with no objdump sibling.
for t in size nm; do
  ln -sf "$ARM_BIN/arm-none-eabi-$t" "$H/wrongisa/arm-none-eabi-$t"
  ln -sf "$ARM_BIN/arm-none-eabi-$t" "$H/armstub/arm-none-eabi-$t"
done

# The tools tree under test, with the bench lock replaced by the stub.
mkdir -p "$H/tools"
cp "$FLASH"/*.sh "$H/tools/" 2>/dev/null
cp "$H/stub/bench-lock.sh" "$H/tools/bench-lock.sh"
chmod +x "$H/tools"/*.sh

FULL_PATH="$ARM_BIN:$H/stub:/usr/bin:/bin:/usr/sbin:/sbin"
APPLE_PATH="$H/stub:/usr/bin:/bin:/usr/sbin:/sbin"       # Apple size exists and rejects -A
NONE_PATH="$H/stub:$H/minbin"                            # no binutils at all
VANISH_PATH="$H/vanish:$ARM_BIN:$H/stub:/usr/bin:/bin"
VANISH_SIZE_PATH="$H/vanish-size:$ARM_BIN:$H/stub:/usr/bin:/bin"
VANISH_OD_PATH="$H/vanish-objdump:$ARM_BIN:$H/stub:/usr/bin:/bin"
REMOTE_FULL="$ARM_BIN:$H/stub:/usr/bin:/bin"             # the Pi's PATH, cross toolchain present
REMOTE_NONE="$H/stub:$H/minbin"                          # the Pi's PATH, no binutils at all
# The only disassembler is a multi-target one decoding the wrong ISA; size and nm are real.
WRONG_ISA_PATH="$H/wrongisa:$H/stub:$H/minbin"
# The same shape, but an arm decoder that reports `armv7e-m` and does see the wfi.
ARM_STUB_PATH="$H/armstub:$H/stub:$H/minbin"

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

# Some claims are about the stub LOGS or the filesystem, not the exit code: which remote path the
# copy went to, whether it was removed again, whether the handler's own failures leaked into the
# run's status. pi_run leaves $H/log standing for the caller and reports the run in OUT/OUT_RC.
pi_run() {  # pi_run <path> <img> [env=val...]
  local path="$1" img="$2"; shift 2
  rm -rf "$H/log"; mkdir -p "$H/log"
  OUT=$( export PATH="$path" HARNESS_LOG="$H/log"
         env "$@" BOARD=master FLASH_DRY_RUN=1 BENCH_OWNER=harness \
             bash "$H/tools/flash.sh" "$img" 2>&1 )
  OUT_RC=$?
}
staged_dst() { sed -n 's/.*:\(\/tmp\/[^ ]*\)$/\1/p' "$H/log/scp.log" 2>/dev/null | head -1; }
pass() { printf 'PASS  %-56s %s\n' "$1" "${2-}"; PASS=$((PASS+1)); }
fail() { printf 'FAIL  %-56s %s\n' "$1" "${2-}"; FAIL=$((FAIL+1)); }

echo "== flash.sh image guards: adversarial input, local runner =="
echo "-- with the cross toolchain present (locks=1: the guards run under the bench lock)"
case_is "wfi image refused"                    1 1 0 "$FULL_PATH" offroad-master "$H/sleeper.elf"
# The scan must key on the INSTRUCTION, not on the path. `objdump -d` opens with
# `<image-path>:  file format elf32-littlearm`, so an image at a path containing the word used to be
# refused with no wfi in it, and every case above was passing on its filename rather than on its
# bytes. This image has the word in its path and nops in its text: it must flash.
case_is "clean image at a wfi-named path flashes" 0 1 1 "$FULL_PATH" offroad-master "$H/wfi.elf"
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
# Named for what it actually pins: with BOTH tools shimmed, the wfi scan runs first and refuses
# there, so this is the objdump branch failing closed, not the size branch. It was labelled "size
# vanishes mid-run" while the comment 180 lines above already explained why that cannot be what it
# tests. The genuine size case is `size vanishes mid-run: healthy image refused`, on VANISH_SIZE_PATH.
case_is "both tools vanish mid-run: the FIRST guard fails closed" 1 1 0 "$VANISH_PATH" offroad-master "$H/gutted.elf"
rm -f "$H/vanish/.size.n" "$H/vanish/.objdump.n"
expect_out "objdump broken: wfi scan does NOT read clean" "REFUSED - the wfi scan did not complete" \
  "$VANISH_PATH" offroad-master "$H/sleeper.elf"

echo "== the PI runner: the same refusals, with the same toolchain discipline =="
echo "-- with the cross toolchain present (locks=1: the guards run under the bench lock)"
case_is "pi: wfi image refused"                   1 1 0 "$FULL_PATH" master "$H/sleeper.elf"
case_is "pi: gutted image refused (below floor)"  1 1 0 "$FULL_PATH" master "$H/gutted.elf"
case_is "pi: missing required symbols refused"    1 1 0 "$FULL_PATH" master "$H/missing.elf"
case_is "pi: hot symbol above 0x08008000 refused" 1 1 0 "$FULL_PATH" master "$H/cold.elf"
case_is "pi: healthy image still flashes"         0 1 1 "$FULL_PATH" master "$H/good.elf"

echo "-- with the cross toolchain ABSENT: refuse, and BEFORE the lock (this used to warn and program)"
for i in wfi gutted missing cold good; do
  case_is "pi: no cross toolchain: $i refused pre-lock" 2 0 0 "$APPLE_PATH" master "$H/$i.elf"
done
case_is "pi: no binutils at all: refused pre-lock" 2 0 0 "$NONE_PATH" master "$H/good.elf"
case_is "pi: unreadable non-ELF refused pre-lock"  2 0 0 "$FULL_PATH" master "$H/corrupt.elf"
expect_out "pi: the refusal names the Pi and the tool" \
  "REFUSED - (no usable [a-z]+ on the Pi|'[^']*' cannot read .* on the Pi)" \
  "$APPLE_PATH" master "$H/good.elf"
expect_out "pi: no binutils, healthy image, still refuses" "REFUSED - no usable (objdump|size|nm) on the Pi" \
  "$NONE_PATH" master "$H/good.elf"

echo "-- the Pi's tools are exercised ON THE PI, not looked for on this host"
case_is "pi: toolchain only on the Pi: flashes"    0 1 1 "$NONE_PATH" master "$H/good.elf" \
  HARNESS_REMOTE_PATH="$REMOTE_FULL"
case_is "pi: toolchain only HERE: refused pre-lock" 2 0 0 "$FULL_PATH" master "$H/good.elf" \
  HARNESS_REMOTE_PATH="$REMOTE_NONE"
case_is "pi: toolchain only on the Pi: wfi refused" 1 1 0 "$NONE_PATH" master "$H/sleeper.elf" \
  HARNESS_REMOTE_PATH="$REMOTE_FULL"
case_is "pi: toolchain only HERE: wfi refused pre-lock" 2 0 0 "$FULL_PATH" master "$H/sleeper.elf" \
  HARNESS_REMOTE_PATH="$REMOTE_NONE"

echo "-- the guards read the Pi's copy, so a copy that did not happen is not a guard that passed"
case_is "pi: image copy fails: refused pre-lock"   2 0 0 "$FULL_PATH" master "$H/good.elf" HARNESS_SCP_FAIL=1
expect_out "pi: and says the copy failed"          "REFUSED - could not copy" \
  "$FULL_PATH" master "$H/good.elf" HARNESS_SCP_FAIL=1

echo "== the Pi's staging copy: per-run, matched end to end, and cleaned up =="
pi_run "$FULL_PATH" "$H/good.elf"
d1=$(staged_dst)
if printf '%s' "$d1" | grep -qE '^/tmp/hoverboard-fw\.[0-9]+\.elf$'; then
  pass "pi: the copy goes to a per-run name" "$d1"
else
  fail "pi: the copy goes to a per-run name" "scp destination was '$d1'"
fi
if printf '%s' "$OUT" | grep -qF "program $d1 verify"; then
  pass "pi: the program command names that same copy"
else
  fail "pi: the program command names that same copy" "no 'program $d1' in the dry run"
fi
if grep -qE 'SSH .* rm -f /tmp/hoverboard-fw\.[0-9]+\.elf' "$H/log/ssh.log"; then
  pass "pi: the removal is issued over ssh"
else
  fail "pi: the removal is issued over ssh" "no rm in ssh.log"
fi
if [ ! -e "$d1" ]; then pass "pi: the staged copy is gone afterwards"
else fail "pi: the staged copy is gone afterwards" "$d1 still exists"; rm -f "$d1"; fi
if grep -q release "$H/log/lock.log"; then pass "pi: and the lock was released by the same handler"
else fail "pi: and the lock was released by the same handler" "no release in lock.log"; fi
pi_run "$FULL_PATH" "$H/good.elf"
d2=$(staged_dst); rm -f "$d2"
if [ -n "$d2" ] && [ "$d1" != "$d2" ]; then pass "pi: two runs stage two different names" "$d1 vs $d2"
else fail "pi: two runs stage two different names" "both were '$d2'"; fi

echo "-- the EXIT handler is best-effort: neither cleanup may change the run's verdict"
pi_run "$FULL_PATH" "$H/good.elf" HARNESS_LOCK_RELEASE_FAIL=1
d3=$(staged_dst)
if [ "$OUT_RC" = 0 ]; then pass "pi: a failed lock release keeps the run's status" "rc=0"
else fail "pi: a failed lock release keeps the run's status" "rc=$OUT_RC"; fi
if [ ! -e "$d3" ]; then pass "pi: a failed lock release still removes the staged copy"
else fail "pi: a failed lock release still removes the staged copy" "$d3 orphaned"; rm -f "$d3"; fi
pi_run "$FULL_PATH" "$H/good.elf" HARNESS_SSH_RM_FAIL=1
rm -f "$(staged_dst)"
if [ "$OUT_RC" = 0 ]; then pass "pi: a failed removal keeps a clean run clean" "rc=0"
else fail "pi: a failed removal keeps a clean run clean" \
  "rc=$OUT_RC: a programmed board reporting failure invites a re-flash"; fi
pi_run "$APPLE_PATH" "$H/good.elf" HARNESS_SSH_RM_FAIL=1
rm -f "$(staged_dst)"
if [ "$OUT_RC" = 2 ]; then pass "pi: a refusal keeps its own exit 2 through cleanup" "rc=2"
else fail "pi: a refusal keeps its own exit 2 through cleanup" "rc=$OUT_RC"; fi

echo "-- a copy that did not happen stops the run before it probes the Pi"
pi_run "$FULL_PATH" "$H/good.elf" HARNESS_SCP_FAIL=1
if [ ! -s "$H/log/ssh.log" ]; then pass "pi: failed copy: no remote command at all"
else fail "pi: failed copy: no remote command at all" "ssh.log has $(grep -c . "$H/log/ssh.log") line(s)"; fi

echo "-- ALLOW_WFI=1 skips the wfi scan and nothing else"
case_is "pi: ALLOW_WFI=1 flashes a wfi image"          0 1 1 "$FULL_PATH"  master "$H/sleeper.elf"  ALLOW_WFI=1
case_is "local: ALLOW_WFI=1 flashes a wfi image"       0 1 1 "$FULL_PATH"  offroad-master "$H/sleeper.elf" ALLOW_WFI=1
case_is "pi: ALLOW_WFI=1 still needs a usable size"    2 0 0 "$APPLE_PATH" master "$H/good.elf" ALLOW_WFI=1
case_is "pi: ALLOW_WFI=1 still needs size/nm present"  2 0 0 "$NONE_PATH"  master "$H/good.elf" ALLOW_WFI=1
case_is "pi: ALLOW_WFI=1 does not excuse a gutted image" 1 1 0 "$FULL_PATH" master "$H/gutted.elf" ALLOW_WFI=1
pi_run "$FULL_PATH" "$H/sleeper.elf" ALLOW_WFI=1
rm -f "$(staged_dst)"
if printf '%s' "$OUT" | grep -q "flash: objdump:"; then
  fail "pi: ALLOW_WFI=1 does not demand a disassembler" "objdump was resolved anyway"
else
  pass "pi: ALLOW_WFI=1 does not demand a disassembler"
fi

echo "== a guard that cannot run fails closed on BOTH runners =="
echo "-- healthy image, one tool vanishing after the pre-lock exercise: refuse, do not warn"
for b in offroad-master master; do
  rm -f "$H/vanish-size/.n"
  case_is "$b: size vanishes mid-run: healthy image refused"    1 1 0 "$VANISH_SIZE_PATH" "$b" "$H/good.elf"
  rm -f "$H/vanish-objdump/.n"
  case_is "$b: objdump vanishes mid-run: healthy image refused" 1 1 0 "$VANISH_OD_PATH"   "$b" "$H/good.elf"
done
rm -f "$H/vanish-objdump/.n"
expect_out "pi: the mid-run refusal says the guard did not run" "REFUSED - the wfi scan did not complete" \
  "$VANISH_OD_PATH" master "$H/good.elf"
rm -f "$H/vanish-size/.n"
expect_out "pi: the gutted-image guard refuses when it cannot run" \
  "REFUSED - the LTO-gutted-image guard did not run" "$VANISH_SIZE_PATH" master "$H/good.elf"

echo "== the wfi guard rests on Thumb decode, so the disassembler must decode arm =="
# The hazard, driven with the input it exists to refuse: a disassembler that RUNS on the image and
# decodes it as something else. Every one of these ran clean before the architecture assertion
# existed: objdump resolved, the scan disassembled a wfi image as i386, found no `wfi` mnemonic,
# printed "clean - no wfi instruction in image", took the lock and programmed the board.
#
# Re-checked by removing the assertion from a copy of flash.sh: 72 pass, these 9 fail, and the two
# wfi cases come back rc=0 with one lock and one openocd - clean, and programmed. That control only
# started telling the truth once the wfi scan stopped matching the image's PATH: while it did, these
# cases refused at the scan on the filename `wfi.elf` and the sentence above was wrong about them.
for b in offroad-master master; do
  case_is "$b: wrong-ISA objdump: wfi image refused pre-lock" 2 0 0 "$WRONG_ISA_PATH" "$b" "$H/sleeper.elf" \
    HARNESS_WRONG_ARCH=i386
  case_is "$b: wrong-ISA objdump: healthy image refused too"  2 0 0 "$WRONG_ISA_PATH" "$b" "$H/good.elf" \
    HARNESS_WRONG_ARCH=i386
  # A tool that will not say what it decoded is not a tool that decoded arm: fail closed on silence
  # exactly as on a wrong answer.
  case_is "$b: objdump with no architecture line refused"     2 0 0 "$WRONG_ISA_PATH" "$b" "$H/sleeper.elf"
done
expect_out "the refusal says it does not decode as arm" "REFUSED - .* does not decode .* as arm" \
  "$WRONG_ISA_PATH" offroad-master "$H/sleeper.elf" HARNESS_WRONG_ARCH=i386
expect_out "pi: the same refusal names the Pi" "REFUSED - .* does not decode .* as arm on the Pi" \
  "$WRONG_ISA_PATH" master "$H/sleeper.elf" HARNESS_WRONG_ARCH=i386
# ...and the assertion must not become the thing that refuses. An arm decoder still resolves (on an
# `armv7e-m` line, not the bare `arm` llvm-objdump prints), and it is the WFI SCAN that then refuses
# the wfi image, under the lock, which is a different exit code and a different message.
case_is "an arm-reporting objdump resolves, wfi scan refuses" 1 1 0 "$ARM_STUB_PATH" offroad-master "$H/sleeper.elf"
expect_out "and it refuses for the wfi, not the architecture" "REFUSED - image contains a 'wfi'" \
  "$ARM_STUB_PATH" offroad-master "$H/sleeper.elf"
expect_out "the resolve line records that it decodes arm" "flash: objdump: .*decodes arm" \
  "$FULL_PATH" offroad-master "$H/good.elf"
# The real cross toolchain is the positive control that matters, and it is already driven by every
# case above this block: the healthy image still flashes and the wfi image is still caught.

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
  "$FULL_PATH" master "$H/good.elf" HARNESS_CCHP=

echo "-- the wall-clock cap: a launch that never happened must not read as success"
t=$(bash -c "perl -e 'alarm shift; exec @ARGV; exit 127' 5 $H/no-such-openocd" 2>/dev/null; echo $?)
if [ "$t" != 0 ]; then printf 'PASS  %-56s rc=%s\n' "failed exec exits non-zero" "$t"; PASS=$((PASS+1))
else printf 'FAIL  %-56s rc=0\n' "failed exec exits non-zero"; FAIL=$((FAIL+1)); fi
t=$(bash -c "perl -e 'alarm shift; exec @ARGV; exit 127' 1 sleep 30" 2>/dev/null; echo $?)
if [ "$t" = 142 ]; then printf 'PASS  %-56s rc=%s\n' "the alarm still fires (SIGALRM)" "$t"; PASS=$((PASS+1))
else printf 'FAIL  %-56s rc=%s, wanted 142\n' "the alarm still fires (SIGALRM)" "$t"; FAIL=$((FAIL+1)); fi

echo "-- $PASS passed, $FAIL failed"
[ "$FAIL" = 0 ]
