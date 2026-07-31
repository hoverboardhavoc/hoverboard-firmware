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

# case <label> <expected-exit> <force> <cchp>
case_is() {
  local label="$1" want="$2" force="$3" cchp="${4-}"
  local got
  FORCE_ARMED_GUARD="$force" "$GUARD" test "$cchp" >/dev/null 2>&1
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

echo "-- $PASS passed, $FAIL failed"
[ "$FAIL" = 0 ]
