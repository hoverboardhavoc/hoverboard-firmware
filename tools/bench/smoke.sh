#!/usr/bin/env bash
# The bench smoke test on the offroad pair (specs/bench-ci.md), run by the bench workflow
# (.github/workflows/bench.yml) on the bench runner, or by hand with the same inputs.
#
#   smoke.sh flash <bundle>                   section 4: relay on, flash slave then master (bridge,
#                                             then board, each over its own bridge), boot and
#                                             settings checks
#   smoke.sh drive <elf> <app.apk> <test.apk> sections 5 to 7: the app drives while the CLI records
#                                             and the guard holds the speed cap; then the judge
#   smoke.sh disarm                           stop the recorders and the app, disarm both boards
#   smoke.sh cleanup                          disarm, then switch the relay off
#
# It takes no bench lock: the caller does (the workflow, or a person with bench-lock.sh).
#
# Environment:
#   CLI     hoverboard-cli for the machine this runs on (default: hoverboard-cli on PATH)
#   OUT     the run's directory: every recording, the verdict and smoke.log (default: ./bench-out)
#   RIG     the rig file: the bridges' and the adb host's addresses, the phone's serial, the boards'
#           uids (default: $BENCH_RIG, which the runner's .env sets). It is kept on the runner, not
#           in the repository; tools/bench/offroad-rig.example.json shows its shape.
#   PI      the ssh host of the bench Pi's gate (default: bench-pi, an alias in the runner's
#           ~/.ssh/config). The gate switches the PSU relay, takes the bench lock and records the
#           master's telemetry over its bridge's USB console, which is wired to the Pi.
#   LIMITS  the gentle-driving values and the pass criteria (default: tools/bench/limits.json)
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
CLI="${CLI:-hoverboard-cli}"
OUT="${OUT:-$PWD/bench-out}"
RIG="${RIG:-${BENCH_RIG:?set RIG or BENCH_RIG to the rig file}}"
PI="${PI:-bench-pi}"
LIMITS="${LIMITS:-$HERE/limits.json}"
PY="python3 -u $HERE/bench.py"
mkdir -p "$OUT"

log() { printf '%s %s\n' "$(date -u +%H:%M:%S)" "$*" | tee -a "$OUT/smoke.log" >&2; }
die() { log "FAIL: $*"; exit 1; }
# A field of a JSON file, with python3 (no jq needed).
field() { python3 -c 'import json,sys; v=json.load(open(sys.argv[1])); [v:=v[int(k)] if k.isdigit() else v[k] for k in sys.argv[2].split(".")]; print(v)' "$1" "$2"; }
role_field() { python3 -c 'import json,sys; print(next(b for b in json.load(open(sys.argv[1]))["boards"] if b["role"]==sys.argv[2]).get(sys.argv[3],""))' "$RIG" "$1" "$2"; }
# A command with a time limit; macOS has no timeout(1), and perl's alarm survives the exec.
to() { local secs="$1"; shift; perl -e 'alarm shift; exec @ARGV or die "exec $ARGV[0]: $!"' "$secs" "$@"; }
cli() { local tcp="$1"; shift; to "${CLI_TIMEOUT:-120}" "$CLI" --tcp "$tcp" "$@"; }
adb() { $PY adb --host "$(field "$RIG" adb_host)" --serial "$(field "$RIG" phone)" "$@"; }
relay() { ssh "$PI" relay "$1"; }   # on | off
ROLES="slave master"   # flash order: slave first (section 4)
APP=com.hoverboardhavoc.remote
MASTER_TCP="$(role_field master tcp)"

wait_bridges() {
  local r tcp br t
  for r in $ROLES; do
    tcp="$(role_field "$r" tcp)"; br="$(role_field "$r" bridge)"
    for t in $(seq 1 30); do
      if CLI_TIMEOUT=15 cli "$tcp" bridge info "$br" >/dev/null 2>&1; then log "$r bridge $tcp answers"; break; fi
      [ "$t" = 30 ] && die "$r bridge $tcp does not answer"
      sleep 4
    done
  done
}

do_flash() {
  local bundle="$1" r tcp br uid
  [ -f "$bundle" ] || die "no bundle at $bundle"
  log "relay on"
  relay on
  sleep 5
  wait_bridges
  for r in $ROLES; do
    tcp="$(role_field "$r" tcp)"; br="$(role_field "$r" bridge)"; uid="$(role_field "$r" uid)"
    log "$r: bridge update"
    CLI_TIMEOUT=600 cli "$tcp" bridge update "$br" "$bundle" --yes 2>&1 | tee -a "$OUT/flash-$r.txt" || die "$r bridge update"
    log "$r: board update"
    CLI_TIMEOUT=600 cli "$tcp" board update "$uid" "$bundle" --yes 2>&1 | tee -a "$OUT/flash-$r.txt" || die "$r board update"
  done
  sleep 3
  wait_bridges
  log "boot and settings checks"
  local t ok=0
  for t in $(seq 1 10); do
    if cli "$MASTER_TCP" board status 2>&1 | $PY stamp > "$OUT/status-before.txt"; then
      if [ "$(grep -c ': ok$' "$OUT/status-before.txt")" -ge 2 ]; then ok=1; break; fi
    fi
    sleep 3
  done
  cat "$OUT/status-before.txt" >&2
  [ "$ok" = 1 ] || die "board status is not ok on both boards"
  local rc=0
  cli "$MASTER_TCP" machine diff "$REPO/$(field "$RIG" machine_file)" > "$OUT/machine-diff.txt" 2>&1 || rc=$?
  cat "$OUT/machine-diff.txt" >&2
  [ "$rc" = 0 ] || die "machine diff against $(field "$RIG" machine_file) reports changes or an error (exit $rc)"
  log "flash and check passed"
}

start_recorder() { # <name> <command...>: run it with its output stamped into $OUT/<name>.txt
  local name="$1"; shift
  ( "$@" 2> "$OUT/$name.err" | $PY stamp > "$OUT/$name.txt" ) &
  echo "$!" >> "$OUT/pids"
}

stop_recorders() {
  [ -f "$OUT/pids" ] || return 0
  touch "$OUT/guard.stop"
  local p
  for p in $(cat "$OUT/pids"); do pkill -TERM -P "$p" 2>/dev/null || true; kill -TERM "$p" 2>/dev/null || true; done
  sleep 1
  pkill -KILL -f "$CLI .*board read" 2>/dev/null || true
  pkill -KILL -f "ssh $PI watch" 2>/dev/null || true
  rm -f "$OUT/pids"
}

do_disarm() {
  stop_recorders
  adb shell "am force-stop $APP" >/dev/null 2>&1 || true
  local r
  for r in $ROLES; do
    CLI_TIMEOUT=30 cli "$(role_field "$r" tcp)" board disarm "$(role_field "$r" uid)" 2>&1 | tee -a "$OUT/disarm.txt" >&2 || log "$r: disarm failed"
  done
}

do_drive() {
  local elf="$1" app="$2" test="$3" r tcp uid
  $PY span "$elf" "$OUT/span.json" > "$OUT/span.txt"
  local addr len hz secs
  addr="$(field "$OUT/span.json" addr)"; len="$(field "$OUT/span.json" len)"
  hz="$(field "$LIMITS" read_hz)"
  cli "$MASTER_TCP" machine nodes 2>&1 | $PY stamp > "$OUT/nodes.txt"
  for r in $ROLES; do
    cli "$(role_field "$r" tcp)" board get "$(role_field "$r" uid)" motors.0.current.cal 2>&1 | $PY stamp > "$OUT/cal-$r.txt"
  done
  log "install the app and its test on the phone"
  adb shell "am force-stop $APP" >/dev/null || true
  adb install "$app" "$test" | tee -a "$OUT/smoke.log" >&2
  local p
  for p in ACCESS_FINE_LOCATION BLUETOOTH_SCAN BLUETOOTH_CONNECT; do
    # A permission the phone's Android does not have (BLUETOOTH_SCAN before 12) is refused; logged.
    adb shell "pm grant $APP android.permission.$p" >> "$OUT/grant.txt" 2>&1 || true
  done
  adb shell "input keyevent KEYCODE_WAKEUP" >/dev/null

  # The recorders: telemetry over the master bridge's USB console, through the Pi's gate (a bridge's
  # TCP console takes one client, and the master's carries its hall reads), the hall words of each
  # board over its own bridge.
  rm -f "$OUT/pids" "$OUT/guard.stop" "$OUT/guard.txt"
  secs=$(( 120 + ($(field "$LIMITS" ramp_ms) * 4 + $(field "$LIMITS" steady_ms) * 2 + $(field "$LIMITS" coast_ms)) / 1000 ))
  start_recorder watch ssh "$PI" watch "$secs"
  for r in $ROLES; do
    start_recorder "reads-$r" "$CLI" --tcp "$(role_field "$r" tcp)" board read "$(role_field "$r" uid)" "$addr" "$len" --hz "$hz"
  done
  local pp
  pp="$(python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); print(min(mo.get("pole_pairs") or 15 for b in m["boards"].values() for mo in b.get("motors",[{}])))' "$REPO/$(field "$RIG" machine_file)")"
  ( BENCH_SPAN="$OUT/span.json" BENCH_POLE_PAIRS="$pp" BENCH_GUARD_STOP="$OUT/guard.stop" \
      $PY guard "$(field "$LIMITS" speed_cap_rpm)" "OUT='$OUT' CLI='$CLI' RIG='$RIG' PI='$PI' '$HERE/smoke.sh' disarm" \
      "$OUT/watch.txt" "$OUT/reads-master.txt" "$OUT/reads-slave.txt" >> "$OUT/guard.txt" 2>> "$OUT/guard.err"
    [ -s "$OUT/guard.txt" ] || rm -f "$OUT/guard.txt" ) &
  echo "$!" >> "$OUT/pids"
  sleep 4

  log "the app drives"
  adb --stamp shell "am instrument -r -w \
      -e device $(field "$RIG" device) -e boards 2 \
      -e throttle $(field "$LIMITS" throttle) -e ramp_ms $(field "$LIMITS" ramp_ms) \
      -e steady_ms $(field "$LIMITS" steady_ms) -e coast_ms $(field "$LIMITS" coast_ms) \
      -e class $APP.bench.BenchDriveTest $APP.test/androidx.test.runner.AndroidJUnitRunner" \
      > "$OUT/instrument.txt" || true
  grep -E 'bench_phase|OK \(|FAILURES|AssertionError' "$OUT/instrument.txt" >&2 || true
  sleep 2
  stop_recorders
  wait 2>/dev/null || true
  cli "$MASTER_TCP" board status 2>&1 | $PY stamp > "$OUT/status-after.txt" || true
  log "judge"
  $PY judge "$OUT" "$LIMITS" "$RIG" "$REPO/$(field "$RIG" machine_file)" | tee "$OUT/verdict.txt" >&2
}

case "${1:-}" in
  flash) [ "$#" = 2 ] || exit 2; do_flash "$2" ;;
  drive) [ "$#" = 4 ] || exit 2; do_drive "$2" "$3" "$4" ;;
  disarm) do_disarm ;;
  cleanup) do_disarm; log "relay off"; relay off ;;
  *) sed -n '2,20p' "$0" >&2; exit 2 ;;
esac
