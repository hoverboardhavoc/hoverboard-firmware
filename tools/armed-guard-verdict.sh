#!/usr/bin/env bash
# The armed-bridge guard's VERDICT: the single owner of the "may this tool halt the core?" policy.
#
# Every halt-capable tool in tools/ (flash.sh's `program ... verify reset`, store-bench-oracle.sh's
# per-phase `reset halt`) must read TIMER0's CCHP first and refuse while MOE is set. Halting stops
# software, not the timer's output stage: the last duties stay live with nothing stepping them, which
# is the recorded FET failure. Only tools/swd-disarm-halt.sh may halt an armed board, because it
# performs the disarm write itself and proves the read-back.
#
# The policy lives here, once, so the two callers cannot drift and so the refusal branches are
# exercisable on the host (tools/tests/armed-guard-verdict-test.sh drives every case synthetically;
# proving them against real silicon needs an armed bridge, which is a user-present session).
#
# Usage:
#   armed-guard-verdict.sh <tool-label> <cchp-hex-or-empty>
#
# <cchp-hex-or-empty> is whatever the caller's CCHP read produced: 1-8 hex digits on a good read,
# the empty string (or anything non-hex) when the read did not land.
#
# Exit codes:
#   0  MOE clear, verified from a good read: safe to halt.
#   1  REFUSED: MOE is SET, or the read was inconclusive (see FORCE_ARMED_GUARD).
#
# FORCE_ARMED_GUARD=1 downgrades the INCONCLUSIVE case to a warning and exits 0. It never applies to
# a good read that says MOE is set: a positive armed read is a refusal with no override.
#
# TIMER0 CCHP = 0x4001_2C44 (base 0x4001_2C00 + offset 0x44, identical on F103 and F130); MOE is
# bit 15. An unclocked or never-configured TIMER0 reads 0, i.e. clear, so a board with no motor
# brought up passes this check for free.
set -u

TOOL="${1:?usage: armed-guard-verdict.sh <tool-label> <cchp-hex-or-empty>}"
CCHP="${2-}"

# A good read is 1-8 hex digits and nothing else. Anything else (empty, an openocd error line, a
# truncated capture) is INCONCLUSIVE: we do not know the bridge's state, so we do not get to halt.
if ! printf '%s' "$CCHP" | grep -qE '^[0-9a-fA-F]{1,8}$'; then
  if [ "${FORCE_ARMED_GUARD:-0}" = "1" ]; then
    echo "$TOOL: WARNING - CCHP read inconclusive (got '${CCHP}'); FORCE_ARMED_GUARD=1, proceeding." >&2
    echo "$TOOL: you are asserting the bridge is disarmed. Verify the rail is off or MOE is clear." >&2
    exit 0
  fi
  echo "$TOOL: REFUSED - could not read CCHP (got '${CCHP}'), so the armed state is UNKNOWN." >&2
  echo "$TOOL: an unknown bridge state fails closed: this tool halts the core, and halting an" >&2
  echo "$TOOL: energized bridge leaves the last duties live with nothing stepping them." >&2
  echo "$TOOL: the common cause is the probe being BUSY - a persistent openocd (the mailbox/tunnel" >&2
  echo "$TOOL: session) already owns it, so this read never attached. Kill it first:" >&2
  echo "$TOOL:   ssh pi@192.168.0.248 'sudo pkill -x openocd'" >&2
  echo "$TOOL: other causes: the rail is off, or the probe is unplugged." >&2
  echo "$TOOL: if you have verified the bridge is disarmed by other means, re-run with" >&2
  echo "$TOOL: FORCE_ARMED_GUARD=1." >&2
  exit 1
fi

if [ $(( 0x$CCHP & 0x8000 )) -ne 0 ]; then
  echo "$TOOL: REFUSED - CCHP = 0x$CCHP, MOE is SET: the bridge is ARMED." >&2
  echo "$TOOL: halting an energized bridge leaves the last duties live with nothing stepping them" >&2
  echo "$TOOL: (the recorded FET failure). FORCE_ARMED_GUARD does NOT override this." >&2
  echo "$TOOL: run tools/swd-disarm-halt.sh first, or cut the rail." >&2
  exit 1
fi

echo "$TOOL: CCHP = 0x$CCHP, MOE clear - safe to halt"
exit 0
