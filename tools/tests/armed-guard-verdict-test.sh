#!/usr/bin/env bash
# Synthetic proof for tools/armed-guard-verdict.sh: drives the REAL script (not a copy of its logic)
# with every class of CCHP read a halt-capable tool can get, and checks the exit code.
#
# Why synthetic: the refusal branches need an ARMED bridge to reach on silicon, and arming is a
# user-present act. The verdict is a pure function of the read string, so every case is reachable
# here, including the two that matter most: MOE set (refuse, no override) and an inconclusive read
# (refuse, overridable). The 2026-07-31 arm session found the inconclusive case WARNING and
# proceeding, with the probe busy on the mailbox session as its cause.
#
# Usage: tools/tests/armed-guard-verdict-test.sh   (exit 0 = all cases as expected)
set -u

GUARD="$(cd "$(dirname "$0")/.." && pwd)/armed-guard-verdict.sh"
PASS=0
FAIL=0

# case <label> <expected-exit> <force> [<cchp> [<cchp-confirm> [<canary> [<runner>]]]]
# Everything from the 4th argument on is handed to the guard verbatim, INCLUDING the argument count:
# supplying a confirm read is what opts a caller into the strengthened check, so "2 args" and "4 args
# where two are empty" are different cases and both are exercised below.
case_is() {
  local label="$1" want="$2" force="$3"
  shift 3
  local got
  FORCE_ARMED_GUARD="$force" "$GUARD" test "$@" >/dev/null 2>&1
  got=$?
  if [ "$got" = "$want" ]; then
    printf 'PASS  %-52s exit %s\n' "$label" "$got"
    PASS=$((PASS + 1))
  else
    printf 'FAIL  %-52s exit %s, wanted %s\n' "$label" "$got" "$want"
    FAIL=$((FAIL + 1))
  fi
}

echo "== armed-guard-verdict.sh: synthetic CCHP verdicts =="

# Good reads, MOE clear (bit 15 = 0): the pre-motor and disarmed cases. Allowed to halt.
case_is "0000 (never-configured TIMER0)"          0 0 "0000"
case_is "00000000 (8-digit clear read)"           0 0 "00000000"
case_is "7fff (every bit but MOE set)"            0 0 "7fff"
case_is "0 (single digit)"                        0 0 "0"

# Good reads, MOE SET (bit 15 = 1): the armed bridge. Refused, and NOT overridable.
case_is "8000 (MOE only)"                         1 0 "8000"
case_is "8001 (armed, one output enabled)"        1 0 "8001"
case_is "FFFF (uppercase hex, armed)"             1 0 "FFFF"
case_is "8000 with FORCE_ARMED_GUARD=1"           1 1 "8000"
case_is "00008000 (8-digit armed read)"           1 1 "00008000"

# Inconclusive reads: the probe was busy, the rail was off, the capture was an error line. The
# armed state is UNKNOWN, so it fails closed - unless the operator asserts otherwise.
case_is "empty (no argument at all)"              1 0
case_is "empty string (probe busy: sed caught 0)" 1 0 ""
case_is "an openocd error line"                   1 0 "Error: init mode failed"
case_is "non-hex junk"                            1 0 "zzzz"
case_is "9 hex digits (over-long capture)"        1 0 "000080000"
case_is "hex with trailing space"                 1 0 "8000 "
case_is "empty with FORCE_ARMED_GUARD=1"          0 1 ""
case_is "junk with FORCE_ARMED_GUARD=1"           0 1 "Error: init mode failed"

# ---- the strengthened read: same-session double CCHP + CPUID canary ------------------------------
# A read of 0 is legitimately safe (unclocked TIMER0), which is what makes a transport that answers
# zeros indistinguishable from a healthy pre-motor board. The confirm read and the canary are what
# separate them. 0x412fc231 is the real CPUID measured on both offroad boards; 0x410cc200 is the
# Cortex-M0 value the F130s would give.
CLEAN=412fc231
case_is "double read agrees, canary good (clear)"   0 0 "0c1c" "0c1c"     "$CLEAN" local
case_is "double read agrees, canary good (zero)"    0 0 "00000000" "00000000" "$CLEAN" local
case_is "same value, different width"               0 0 "0" "00000000"    "$CLEAN" local
case_is "M0 canary (0x410cc200)"                    0 0 "0c1c" "0c1c"     "410cc200" local
case_is "all-zero session (canary 0 too)"           1 0 "00000000" "00000000" "00000000" local
case_is "all-zero session, FORCE=1 overrides"       0 1 "00000000" "00000000" "00000000" local
case_is "canary empty (read did not land)"          1 0 "0c1c" "0c1c"     "" local
case_is "canary not an ARM implementer"             1 0 "0c1c" "0c1c"     "deadbeef" local
case_is "transient zero: 0000 then 8c1c"            1 0 "0000" "8c1c"     "$CLEAN" local
case_is "transient zero the other way: 8c1c, 0000"  1 0 "8c1c" "0000"     "$CLEAN" local
case_is "reads disagree, both MOE clear"            1 0 "0c1c" "0c1e"     "$CLEAN" local
case_is "confirm read empty"                        1 0 "0c1c" ""         "$CLEAN" local
case_is "confirm read junk"                         1 0 "0c1c" "Error:"   "$CLEAN" local
# The armed test must run BEFORE the agreement/canary tests, so that a bridge seen energized once can
# never be demoted into the overridable inconclusive branch by a bad second read or a bad canary.
case_is "armed 1st + junk 2nd, FORCE=1"             1 1 "8c1c" "Error:"   "$CLEAN" local
case_is "armed 2nd + zero 1st, FORCE=1"             1 1 "0000" "8c1c"     "$CLEAN" local
case_is "armed both + zero canary, FORCE=1"         1 1 "8c1c" "8c1c"     "00000000" local
# Callers that pass only the single read keep the old behaviour exactly: no confirm, no canary check.
case_is "single-read caller, clear"                 0 0 "0c1c"
case_is "single-read caller, zero"                  0 0 "00000000"
case_is "single-read caller, armed"                 1 0 "8c1c"
# The runner argument is a hint for the message only: it must not move any verdict.
for r in "" pi local bogus; do
  case_is "runner hint '$r' does not change a clear verdict"  0 0 "0c1c" "0c1c" "$CLEAN" "$r"
  case_is "runner hint '$r' does not change an armed verdict" 1 0 "8c1c" "8c1c" "$CLEAN" "$r"
done

echo "-- $PASS passed, $FAIL failed"
[ "$FAIL" = 0 ]
