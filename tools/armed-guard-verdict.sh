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
#   armed-guard-verdict.sh <tool-label> <cchp-hex-or-empty> [<cchp-confirm> [<canary> [<runner>]]]
#
# <cchp-hex-or-empty> is whatever the caller's CCHP read produced: 1-8 hex digits on a good read,
# the empty string (or anything non-hex) when the read did not land.
#
# <cchp-confirm> and <canary> are the strengthened read (see "WHY A SINGLE READ IS NOT ENOUGH"). A
# caller that supplies them opts into the stricter check; once supplied they must hold up, so an
# empty or junk value in either is a REFUSAL, not a skip. A caller that supplies neither gets the
# single-read behaviour, unchanged.
#
# <runner> ("pi" or "local") only selects which "probe is busy" hint the inconclusive branch prints.
# It is a display hint and nothing else: no verdict anywhere depends on it, so the two runners cannot
# get different policies out of this script.
#
# Exit codes:
#   0  MOE clear, verified from a good read: safe to halt.
#   1  REFUSED: MOE is SET, or the read was inconclusive (see FORCE_ARMED_GUARD).
#
# FORCE_ARMED_GUARD=1 downgrades the INCONCLUSIVE case to a warning and exits 0. It never applies to
# a good read that says MOE is set: a positive armed read is a refusal with no override, and that
# holds if EITHER read says so, which is why the armed test runs before the agreement test below.
#
# TIMER0 CCHP = 0x4001_2C44 (base 0x4001_2C00 + offset 0x44, identical on F103 and F130); MOE is
# bit 15. An unclocked or never-configured TIMER0 reads 0, i.e. clear, so a board with no motor
# brought up passes this check for free.
#
# WHY A SINGLE READ IS NOT ENOUGH. That free pass for 0 is also this guard's one fail-open: a
# transport-level misread that returns zeros on a genuinely armed board is indistinguishable from a
# board that never configured TIMER0, and gets ruled safe. One transient 0x00000000 was observed on
# the bench master (2026-08-11) and did not reproduce in twelve samples. The GD32s need
# `set CPUTAPID 0`, which disables OpenOCD's expected-IDCODE check, so init itself proves nothing
# about the link. Two defences, both from the caller's SAME probe session:
#   - the CCHP read repeated: a value that does not repeat is noise, not a bridge state;
#   - a canary read of the SCB CPUID (0xE000_ED00), which is core-internal (no clock enable, no
#     peripheral configured, valid with the core running) and always carries ARM's implementer byte
#     0x41. Zeros from a bogus link fail it; measured 0x412fc231 on both offroad boards.
# Neither can prove a link is sound, but together they mean the specific failure this guard exists to
# catch cannot present as the "safe for free" reading.
set -u

TOOL="${1:?usage: armed-guard-verdict.sh <tool-label> <cchp-hex-or-empty> [<cchp-confirm> [<canary> [<runner>]]]}"
CCHP="${2-}"
CCHP_CONFIRM="${3-}"
CANARY="${4-}"
RUNNER_HINT="${5-}"
HAVE_CONFIRM=0; [ "$#" -ge 3 ] && HAVE_CONFIRM=1
HAVE_CANARY=0;  [ "$#" -ge 4 ] && HAVE_CANARY=1

good_read() { printf '%s' "$1" | grep -qE '^[0-9a-fA-F]{1,8}$'; }
armed()     { good_read "$1" && [ $(( 0x$1 & 0x8000 )) -ne 0 ]; }

# The ARMED test comes first, and considers EITHER read, so no amount of disagreement or canary
# trouble can demote a positive armed read into the overridable inconclusive branch.
if armed "$CCHP"; then
  ARMED_VAL="$CCHP"
elif [ "$HAVE_CONFIRM" = 1 ] && armed "$CCHP_CONFIRM"; then
  ARMED_VAL="$CCHP_CONFIRM"
  echo "$TOOL: NOTE - the two CCHP reads disagreed ('$CCHP' then '$CCHP_CONFIRM') and one is ARMED." >&2
  echo "$TOOL: the armed reading wins: a bridge seen energized once is treated as energized." >&2
else
  ARMED_VAL=""
fi
if [ -n "$ARMED_VAL" ]; then
  echo "$TOOL: REFUSED - CCHP = 0x$ARMED_VAL, MOE is SET: the bridge is ARMED." >&2
  echo "$TOOL: halting an energized bridge leaves the last duties live with nothing stepping them" >&2
  echo "$TOOL: (the recorded FET failure). FORCE_ARMED_GUARD does NOT override this." >&2
  echo "$TOOL: run tools/swd-disarm-halt.sh first, or cut the rail." >&2
  exit 1
fi

# Everything from here reads as "not armed", so every remaining doubt about the read is a doubt about
# a SAFE-looking answer. Collect the reasons the read cannot be trusted; any one of them is the
# inconclusive case.
UNTRUSTED=""
if ! good_read "$CCHP"; then
  UNTRUSTED="could not read CCHP (got '${CCHP}')"
elif [ "$HAVE_CONFIRM" = 1 ] && ! good_read "$CCHP_CONFIRM"; then
  UNTRUSTED="the confirming CCHP read did not land (got '${CCHP_CONFIRM}')"
elif [ "$HAVE_CONFIRM" = 1 ] && [ $(( 0x$CCHP )) -ne $(( 0x$CCHP_CONFIRM )) ]; then
  UNTRUSTED="the two CCHP reads disagree ('$CCHP' then '$CCHP_CONFIRM'), so the value is noise"
elif [ "$HAVE_CANARY" = 1 ] && ! printf '%s' "$CANARY" | grep -qE '^41[0-9a-fA-F]{6}$'; then
  UNTRUSTED="the CPUID canary read '${CANARY}' is not a live ARM core ID (expected 41xxxxxx), so the link itself is unproven"
fi

if [ -n "$UNTRUSTED" ]; then
  if [ "${FORCE_ARMED_GUARD:-0}" = "1" ]; then
    echo "$TOOL: WARNING - CCHP read inconclusive ($UNTRUSTED); FORCE_ARMED_GUARD=1, proceeding." >&2
    echo "$TOOL: you are asserting the bridge is disarmed. Verify the rail is off or MOE is clear." >&2
    exit 0
  fi
  echo "$TOOL: REFUSED - $UNTRUSTED, so the armed state is UNKNOWN." >&2
  echo "$TOOL: an unknown bridge state fails closed: this tool halts the core, and halting an" >&2
  echo "$TOOL: energized bridge leaves the last duties live with nothing stepping them." >&2
  # One policy, one owner; only the "who is probably holding the probe" hint is runner-shaped,
  # because the two runners have physically different contended resources.
  echo "$TOOL: the common cause is the probe being BUSY - a persistent openocd already owns it, so" >&2
  echo "$TOOL: this read never attached. Kill it first:" >&2
  case "$RUNNER_HINT" in
    local)
      echo "$TOOL:   pkill -x openocd        # on THIS host" >&2
      echo "$TOOL: the LAN probes (ESP32-C3 elaphureLink) accept a SINGLE TCP client at a time, so a" >&2
      echo "$TOOL: local openocd left running holds the probe against every other tool here." >&2 ;;
    pi)
      echo "$TOOL:   ssh pi@192.168.0.248 'sudo pkill -x openocd'" >&2 ;;
    *)
      echo "$TOOL:   pkill -x openocd                                  # if the probe is on this host" >&2
      echo "$TOOL:   ssh pi@192.168.0.248 'sudo pkill -x openocd'      # if the probe is on the Pi" >&2 ;;
  esac
  echo "$TOOL: other causes: the rail is off, or the probe is unplugged." >&2
  echo "$TOOL: if you have verified the bridge is disarmed by other means, re-run with" >&2
  echo "$TOOL: FORCE_ARMED_GUARD=1." >&2
  exit 1
fi

HOW=""
[ "$HAVE_CONFIRM" = 1 ] && HOW=" (read twice, agreeing"
[ "$HAVE_CONFIRM" = 1 ] && [ "$HAVE_CANARY" = 1 ] && HOW="$HOW; CPUID canary 0x$CANARY"
[ "$HAVE_CONFIRM" = 1 ] && HOW="$HOW)"
echo "$TOOL: CCHP = 0x$CCHP$HOW, MOE clear - safe to halt"
exit 0
