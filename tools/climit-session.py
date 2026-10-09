#!/usr/bin/env python3
"""Guided, owner-present session for the current limit's energised bench gates.

The implement contract is specs/current-limit-session.md; the gates themselves are
specs/motor-integration.md, "The current limit" (the Bench list). One script: it takes the bench
lock, seats the relay and powers the rail, starts OpenOCD on the Pi and tunnels its TCL port, drives
one mailbox session owner as a child process, tells the operator what to do at each step, asks for
the meter readings only a human can see, samples CTRL_OBS plus the motor statics at 10 Hz into a
self-describing CSV, writes RECORD.md with the verdicts, and tears everything down on ANY exit.

The operator owns the PSU and the kill. The tool never asks them to type a command.

Usage:
    tools/climit-session.py [--board master] [--limit-ma 2500] [--cal-demand 3000]
                            [--trip-demand 32767] [--record DIR] [--elf PATH]
                            [--skip-gate N ...] [--dry-run] [--selftest]

    --dry-run   print every command and prompt in order against a simulated board; nothing is
                executed, no network, no lock, no files written (the RECORD is printed instead).
    --selftest  run tools/tests/test_climit_session.py (parsers, verdicts, calibration arithmetic,
                the RECORD golden, the teardown order, the dry run), no hardware.
    --brake-fallback  gates 3 and 4 brake a spinning wheel instead of locking the rotor.

    --board slave aborts at stand-up today: the bench slave carries a pre-existing
    FAULT_DEMAND_STALE latch, and motor_fault must read 0 at the disarmed read.

Every side effect goes through one seam, `Shell` (RealShell runs it, FakeShell simulates it for the
dry run and the tests). The gate logic is pure functions over decoded sample dicts.

The core is NEVER halted: every read is `read_memory` on the running target (tools/swdobs.py).
"""

import argparse
import contextlib
import io
import os
import re
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import swdobs  # noqa: E402
from swdobs import (  # noqa: E402
    CFG_READ_RE, CTRL_MAGIC, CTRL_OBS_WORDS, CURRENT_CAL_DEFAULT, CURRENT_CAL_FIELD, PI, REPO,
    W_ARM_REFUSALS, W_BATTERY, W_BOOT_COUNT, W_CONTROL_TICKS, W_DUTY01, W_DUTY2_ANGLE,
    W_ENACT_INITS, W_ENACT_SHUTDOWNS, W_EVENTS_HI, W_EVENTS_LO, W_MOTOR_CAL, W_MOTOR_CURRENT,
    W_MOTOR_FAULT, W_MOTOR_SPEED, W_MOTOR_STATE, W_PERIODS, W_SUB_FLAGS, W_TICK_COUNT,
    W_TORQUE_MODE, clamp_current_cal, decode_arm_refusals, host_target, parse_config_read,
    read_current_cal, s16, s32,
)

TOOL = "climit-session"
TOOL_VERSION = "climit-session/1"
M32 = 0xFFFFFFFF

# --------------------------------------------------------------------------------------------------
# Firmware facts, each cited to its owner.
# --------------------------------------------------------------------------------------------------
# mode_byte: crates/state/src/mode.rs, Mode (Off = 0 .. Shutdown = 4). MOE is set on the INIT pass
# and cleared on the SHUTDOWN pass (the same file's tests), so MOE with INIT or READY is legitimate.
MODE_OFF, MODE_INIT, MODE_READY, MODE_RUN, MODE_SHUTDOWN = 0, 1, 2, 3, 4
MODE_NAMES = {0: "OFF", 1: "INIT", 2: "READY", 3: "RUN", 4: "SHUTDOWN"}
# sub_state: crates/control/src/fsm.rs, SubState (Idle = 0).
SUB_IDLE = 0
SUB_ARMING = 1                # the soft start: where a trip re-enters while the arm is still held
# event_counts index of latch A: crates/orchestrator/src/events.rs, EV_LATCH_A (1 << 4).
EV_LATCH_A_INDEX = 4
# motor_state flags byte (bits 24..31): crates/firmware/src/motor.rs, OBS_CONFIGURED / OBS_CAL_ACCEPTED.
OBS_CONFIGURED = 1 << 0
OBS_CAL_ACCEPTED = 1 << 7
# motor_fault: FAULT bits in 0..15, the invalid-hall dwell in 16..31 (crates/firmware/src/main.rs).
FAULT_BITS = 0xFFFF
# The PWM period (ARR) the duty compares are against; the +-28500 clamp maps to 1956
# (specs/motor-integration.md, "The current limit").
PWM_PERIOD = 2250
# crates/firmware/src/motor.rs, limit_counts: the staged milliamp CEILING, and the converted
# limit's floor in COUNTS (the sense chain's noise is a count-domain fact, so with a per-board
# counts-per-amp a milliamp floor would mean a different current on every board). The scale itself
# is the board's: swdobs.CURRENT_CAL_FIELD (0x67), read at stand-up.
CURRENT_LIMIT_CEILING_MA = 40_000
MIN_LIMIT_COUNTS = 2_100      # crates/firmware/src/motor.rs: the HIGH-WATER of 2026-10-09's five
                              # rest-floor reads (1,444/1,863/1,910/1,941/2,097), not the lowest. At
                              # 2,000 the limiter chopped in 2 of 94 samples of an armed STILL soak.
# crates/store/src/field.rs, MOTOR_CURRENT_LIMIT (0x20, u32 mA, default 10_000).
LIMIT_FIELD = 0x20

# --------------------------------------------------------------------------------------------------
# Session parameters (specs/current-limit-session.md, "Parameters", "The flow").
# --------------------------------------------------------------------------------------------------
SESSION_LIMIT_MIN_MA = 2000   # the floor on --limit-ma, in MILLIAMPS. It cannot be stated as a
                              # count value: what a milliamp buys in counts is the board's own
                              # scale (0x67), and the firmware then floors the result at
                              # MIN_LIMIT_COUNTS. So this bounds what the session ASKS for, and the
                              # count floor is what actually holds the trip clear of the rest noise
SESSION_LIMIT_MAX_MA = 5000   # the bench PSU rule: limit plus 1 A, never above 6 A
CAL_LIMIT_MA = 15_000         # what gate 3 wants stored: high enough that no ladder step can chop
PSU_CAP_A = 6.0               # never above 6 A on this bench
PSU_MIN_A = 1.5               # a floor: below this the rail cannot even start the board cleanly
PSU_HEADROOM_X = 1.5          # the declared limit must clear the predicted link current by this
# The PSU carries the DC-LINK mean, which at a locked rotor is the phase current times the duty, and
# the chop holds the phase current at the staged limit. So the link current at the chop is
# I_lim^2 * R / V, far below I_lim: MEASURED 2026-10-09, a 2500 mA limit chopping at about 5 A in the
# winding drew 0.60 to 0.68 A at the PSU. The old rule ("the staged limit plus 1 A") treated the PSU
# limit as if it bounded the PHASE current, which it never has; it demanded 6 A where under 2 A is
# carried, and a HIGHER PSU limit is worse, since the PSU limit is what bounds a shoot-through.
BENCH_PAIR_OHMS = 0.605       # two phases in series, from two duty points, 2026-10-09 (0.605/0.609)
BENCH_LINK_V = 25.0
# The labelled limit may still be optimistic, even now the board carries its own counts per amp
# (0x67): that default is an UPPER bound, not a fit (the 2026-10-09 two-point slopes within a
# session disagreed, 81 and 306 counts per amp, which is noise domination). A TRUE scale below the
# coded one means a label buys more real current than it says, so the PSU prediction keeps a factor
# on it. 2.0 is the same factor the provisional 800 earned against a measured ~455.
CPA_PESSIMISM = 2.0
PSU_RULE_TOL_A = 0.1          # the typed PSU limit may differ from the target by a knob's width
CAL_MIN_PSU_A = 3.0           # the braking fallback's calibration wants at least 3 A on the PSU
CPA_LO, CPA_HI = 0.7, 1.4     # CONFIRMED band around the board's staged 0x67
CAL_SIGNAL_FACTOR = 2.0       # the calibration window's mean peak must clear this many rest floors
                              # before the ratio is anything but an UPPER BOUND (2026-10-09: a 29%
                              # rise in current moved `peak` by 4%)
FLOOR_FACTOR = 1.5            # "peak within 1.5x of gate 1's max"
SOAK_PCT = 90                 # the still soak's peak distribution is judged at its p90, not its
                              # median: a median admits anything flowing in fewer than half the
                              # samples (an intermittently conducting gate), and the measured quiet
                              # soaks had p90 1366 to 1440 against floors near 2097
SOAK_GROSS_FACTOR = 2.0       # the soak's per-sample ceiling: a gross fault, not a noise extreme.
                              # 2x the floor still clears the worst quiet sample measured
                              # 2026-10-09 (2112 counts against a 2097 floor, so 4194)
# The measured rest-noise peak, used only when gate 1 is skipped:
# specs/bench-evidence/2026-10-08/rover-gates/RECORD.md ("about 1,000 to 1,400 counts").
FLOOR_FALLBACK_COUNTS = 2100  # MEASURED 2026-10-09 gate 1: peak max 2097 at rest (was 1400, from
                              # an earlier session's "~1,400-count peak", which is 33% low)
G4_PEAK_LO, G4_PEAK_HI = 0.8, 1.5
G4_PSU_TOL = 0.3
# Gate 5's "chopped toward 64": at least half the window floated in the worst sample.
G5_CHOPPED_MIN = 32
# The locked-rotor demand ladder (gate 3): from --cal-demand upward in steps of 1000, 3 s each, until
# the duty-corrected phase estimate I_psu * 2250 / duty_on lands in [LADDER_EST_LO_A, LADDER_EST_HI_A)
# (below 8 A so the 10 A default limit cannot chop); reaching LADDER_MAX_DEMAND short of it is an
# abort. Gate 4 holds the ladder's final demand plus G4_DEMAND_MARGIN
# (specs/current-limit-session.md, "The flow").
LADDER_STEP, LADDER_STEP_S, LADDER_MAX_DEMAND = 1000, 3.0, 12000
LADDER_EST_LO_A, LADDER_EST_HI_A = 4.0, 8.0
G4_DEMAND_MARGIN = 2000
SETTLE_S = 1.0                # the ~0.57 s soft-start ramp settles before a ladder step or gate-4 window
ARM_EXPIRED = "the arm expired"
RELOCK_MAX = 3                # "the rotor is not locked" re-prompts before the step aborts
LOCK_COUNTDOWN_S = 3.0        # the operator's hands-on time after an Enter they press one-handed
# The one promise every lock prompt makes, so the operator can answer one-handed and then take hold.
HANDS_ON = f"nothing is measured for {LOCK_COUNTDOWN_S:g} s after you press"
LOCK_PROMPT = ("Hand on the kill. Press Enter, then take hold of the rotor (strap, or both hands on the "
               f"tyre) and keep it locked: {HANDS_ON}.")
RELOCK_PROMPT = f"Hand on the kill. The rotor moved: press Enter, then lock it harder: {HANDS_ON}."
FREE_PROMPT = "Hand on the kill. Wheel free, nothing touching the tyre. Press Enter to arm."
ENGAGE_DEMAND_MIN = 590       # the engagement gate's edge on the +-32767 frame: 590 engages (arm-session D4)

SAMPLE_HZ = 10
G1_S, G2_S, SOAK_S, CAL_WINDOW_S, G4_WINDOW_S = 10.0, 5.0, 10.0, 3.0, 3.0
ARM_WITHIN_S = 2.0
SPIN_WITHIN_S, SPIN_STEADY_S = 2.0, 1.0
TRIP_WINDOW_S, POST_TRIP_S = 5.0, 2.0
MOE_CHECK_S = 3.0
REARM_WAIT_S = 3.0
STOP_WITHIN_S = 10.0
RELAY_SEAT_ON_S, RELAY_SEAT_OFF_S = 10.0, 2.0
BOOT_SETTLE_S = 12.0          # boot + IMU settle; the motor bring-up runs late (tools/hall-check.sh)
INPUTS_HOLD_S = 600          # the session owner's --hold: one bounded, owner-present session
DEMAND_ACK_S = 0.3           # the session owner applies a stdin command within a poll + one send
DEMAND_MAX_HELD_S = 180      # one unchanged demand; the bound the drive tool's 60 s cap used to carry
DST_TIMEOUT_S = 45.0          # the walk alone may take 30 s
TCL_PORT = 6666
ENDPOINT = f"127.0.0.1:{TCL_PORT}"
OCD_LOG = "/tmp/climit-openocd.log"

BINS = ("swd-mailbox-config", "swd-mailbox-session", "swd-mailbox-drive")
DST_RE = re.compile(r"dst resolved: attached node 0x([0-9a-fA-F]{2})")


ROTOR_MOVED = "the wheel moved"   # the prefix arm() retries on rather than ending the session
ROTOR_NET_EDGES = 2           # the NET hall edges over a window that mean the rotor turned: one
                              # edge each way is a held rotor dithering, two the same way is not


class SessionAbort(Exception):
    """An abort condition: the session ends through the teardown with this reason recorded."""


class SessionEnd(Exception):
    """Not an abort: a gate failed in a way that makes the later gates meaningless."""


# --------------------------------------------------------------------------------------------------
# Pure arithmetic and parameter rules.
# --------------------------------------------------------------------------------------------------
def limit_counts(ma, staged_cpa):
    """crates/firmware/src/motor.rs, limit_counts: min(ma, 40 A) * clamp(cal, 100, 819) / 1000,
    floored at MIN_LIMIT_COUNTS. `staged_cpa` is the board's own counts per amp (0x67, read at
    stand-up), so this is the firmware's boot-seam arithmetic against the firmware's own number."""
    counts = min(ma, CURRENT_LIMIT_CEILING_MA) * clamp_current_cal(staged_cpa) // 1000
    return max(counts, MIN_LIMIT_COUNTS)


def hard_trip_counts(lc):
    """crates/firmware/src/motor.rs, hard_trip_counts: 2x, saturated at 32767."""
    return min(2 * lc, 32767)


def psu_target_a(limit_ma):
    return max(PSU_MIN_A, round(PSU_HEADROOM_X * link_at_chop_a(limit_ma), 1))


def check_limit_arg(limit_ma):
    """None when --limit-ma is usable, else the reason it is refused."""
    if limit_ma < SESSION_LIMIT_MIN_MA:
        return (f"--limit-ma {limit_ma} is below {SESSION_LIMIT_MIN_MA}: the hard trip is 2x the limit "
                f"in COUNTS, and the counts a milliamp buys are the board's own scale (0x67), so a "
                f"value this low asks for a trip inside the sense chain's noise whatever that scale "
                f"is (the firmware floors the limit at {MIN_LIMIT_COUNTS} counts, the measured "
                f"rest-noise high-water)")
    if limit_ma > SESSION_LIMIT_MAX_MA:
        return (f"--limit-ma {limit_ma} is above {SESSION_LIMIT_MAX_MA}: it needs a PSU limit of "
                f"{psu_target_a(limit_ma):g} A (clearing the {link_at_chop_a(limit_ma):.2f} A the link "
                f"carries at the chop), and this bench's PSU never goes above {PSU_CAP_A:.0f} A")
    return None


def phase_estimate(psu_a, duty_on):
    """The duty-corrected phase current, I_psu * 2250 / duty_on; None when no duty was applied."""
    return psu_a * PWM_PERIOD / duty_on if duty_on > 0 else None


def ladder_next(demand, est, backed_off):
    """The gate-3 ladder's next move from one step's phase estimate:
    ("done", demand) | ("up", next) | ("back", next) | ("abort", reason)."""
    lo, hi = LADDER_EST_LO_A, LADDER_EST_HI_A
    if est is None:
        return "abort", f"duty_on read 0 at demand {demand}: no phase estimate"
    if lo <= est < hi:
        return "done", demand
    if est >= hi:
        back = demand - LADDER_STEP // 2
        if backed_off or back < ENGAGE_DEMAND_MIN:
            return "abort", (f"the phase estimate {est:.1f} A at demand {demand} is at or above {hi:g} A "
                             f"(the 10 A default limit could chop); lower --cal-demand")
        return "back", back
    if backed_off:
        return "abort", (f"the phase estimate {est:.1f} A at demand {demand} is under {lo:g} A after backing "
                         f"off from over {hi:g} A: one step straddles the band; set --cal-demand inside it")
    if demand >= LADDER_MAX_DEMAND:
        return "abort", (f"the phase estimate is {est:.1f} A at demand {demand}: the locked rotor draws less "
                         "than expected; check the lock and the phase wiring")
    return "up", demand + LADDER_STEP


def link_at_chop_a(limit_ma):
    """The DC-link mean the PSU will carry while the limiter chops at `limit_ma`: the phase current
    held at the limit, times the duty that sustains it, over a resistive locked rotor."""
    i_phase = CPA_PESSIMISM * limit_ma / 1000.0
    return i_phase * i_phase * BENCH_PAIR_OHMS / BENCH_LINK_V


def psu_declared_ok(limit_ma, declared_a):
    target = psu_target_a(limit_ma)
    return declared_a >= target - PSU_RULE_TOL_A - 1e-9 and declared_a <= PSU_CAP_A + 1e-9


def psu_reading_abort(reading_a, declared_a):
    """Abort reason when a typed PSU reading is above the PSU limit the operator declared."""
    if reading_a > declared_a + 1e-9:
        return (f"the operator read {reading_a:g} A on the PSU, above the {declared_a:g} A limit "
                f"they declared")
    return None


# --------------------------------------------------------------------------------------------------
# Parsers: mailbox tool output and the CTRL_OBS block.
# --------------------------------------------------------------------------------------------------
def parse_dst(text):
    """The attached node from a mailbox tool's 'dst resolved: attached node 0xNN' line, or None."""
    m = DST_RE.search(text)
    return int(m.group(1), 16) if m else None


def config_write_ok(text, field):
    """True when swd-mailbox-config wrote `field` and its read-back matched."""
    for line in text.splitlines():
        m = CFG_READ_RE.search(line)
        if m and int(m.group(1), 16) == field and "(write -> read matches)" in m.group(5):
            return "PASS:" in text
    return False


def tear_ok(a, b):
    """The tear guard (specs/bench-evidence/2026-10-08/rover-gates/RECORD.md): two back-to-back
    reads of the block must agree on control_ticks - tick_count."""
    return ((a[W_CONTROL_TICKS] - a[W_TICK_COUNT]) & M32) == ((b[W_CONTROL_TICKS] - b[W_TICK_COUNT]) & M32)


def decode_sample(w, m, off, t, label):
    """One sample dict from the CTRL_OBS words `w` and the motor statics block `m`."""
    tm, sf, mc = w[W_TORQUE_MODE], w[W_SUB_FLAGS], w[W_MOTOR_CURRENT]
    d01, d2a, mf, ms = w[W_DUTY01], w[W_DUTY2_ANGLE], w[W_MOTOR_FAULT], w[W_MOTOR_STATE]
    ev = [(w[W_EVENTS_LO] >> (8 * i)) & 0xFF for i in range(4)]
    ev += [(w[W_EVENTS_HI] >> (8 * i)) & 0xFF for i in range(4)]
    refusals = decode_arm_refusals(w[W_ARM_REFUSALS])
    d0, d1, d2 = d01 & 0xFFFF, (d01 >> 16) & 0xFFFF, d2a & 0xFFFF
    return {
        "t": t, "label": label, "magic": w[0], "boot": w[W_BOOT_COUNT], "tick": w[W_TICK_COUNT],
        "ctl": w[W_CONTROL_TICKS], "inits": w[W_ENACT_INITS], "shutdowns": w[W_ENACT_SHUTDOWNS],
        "torque": s16(tm & 0xFFFF), "mode": (tm >> 16) & 0xFF, "moe": (tm >> 24) & 0xFF,
        "sub": sf & 0xFF, "cmode": (sf >> 8) & 0xFF, "flags": (sf >> 16) & 0xFF,
        "levels": (sf >> 24) & 0xFF, "periods": w[W_PERIODS], "mstate": ms,
        "mflags": (ms >> 24) & 0xFF, "d0": d0, "d1": d1, "d2": d2, "duty_on": max(d0, d1, d2),
        "fault": mf & FAULT_BITS, "dwell": (mf >> 16) & 0xFFFF, "speed": s32(w[W_MOTOR_SPEED]),
        "cal": w[W_MOTOR_CAL], "ev": ev, "latch_a": ev[EV_LATCH_A_INDEX],
        "peak": s16(mc & 0xFFFF), "chopped": (mc >> 16) & 0xFF, "trips": (mc >> 24) & 0xFF,
        "battery": w[W_BATTERY] & 0xFFFF,
        # Word 33, the arm refusals: both counts are per BOOT and nothing clears them, so they are
        # read as absolute numbers rather than deltas, and the cause text is already decoded.
        "reread_refusals": refusals[0], "confirm_refusals": refusals[1], "refusal_cause": refusals[2],
        "demand": s32(m[off["DEMAND"]]), "s_speed": s32(m[off["SPEED"]]),
        "s_fault": m[off["FAULT"]], "s_periods": m[off["PERIODS"]], "s_state": m[off["OBS_STATE"]],
        "s_duty01": m[off["OBS_DUTY01"]], "s_duty2": m[off["OBS_DUTY2_ANGLE"]],
        "words": list(w),
    }


# --------------------------------------------------------------------------------------------------
# Verdicts: pure functions over sample lists. Each returns a result dict:
#   {"verdict": str, "lines": [str], ...numbers}
# --------------------------------------------------------------------------------------------------
def _percentile(xs, pct):
    """The nearest-rank percentile: the smallest sample at or above `pct` of the distribution.
    Integer arithmetic on the rank, so there is no float-ceil edge case at small sample counts."""
    v = sorted(xs)
    if not v:
        return 0.0
    rank = -(-pct * len(v) // 100)          # ceil(pct/100 * n), 1-based
    return float(v[rank - 1])


def _mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else 0.0


def floor_max(gate1):
    """The rest-noise peak gate 2 and the still soaks are judged against."""
    return gate1["peak_max"] if gate1 else FLOOR_FALLBACK_COUNTS


def abort_reason(s, gate2=False, before_gate5=True, trips_base=0):
    """The abort conditions the tool enforces on every sample (specs/current-limit-session.md,
    "Safety posture"). None when the sample is acceptable."""
    if s["magic"] != CTRL_MAGIC:
        return f"CTRL_OBS magic reads 0x{s['magic']:08x}, not 0x{CTRL_MAGIC:08x}: the image is not the ELF's"
    if s["moe"] and s["mode"] in (MODE_OFF, MODE_SHUTDOWN):
        return f"moe_bits 0x{s['moe']:02x} set while mode_byte is {MODE_NAMES[s['mode']]}"
    if s["mode"] == MODE_OFF and not s["moe"] and s["fault"]:
        return f"motor_fault 0x{s['fault']:04x} at a disarmed read"
    if before_gate5 and s["trips"] > trips_base:
        return f"trips reads {s['trips']} before gate 5 (baseline {trips_base})"
    if gate2 and s["chopped"]:
        return f"chopped reads {s['chopped']} during gate 2 (disarmed)"
    if before_gate5 and s["latch_a"] % 2:
        return f"event_counts latch-A reads {s['latch_a']} (odd, latch held) before gate 5"
    return None


def standup_problems(s):
    """What is wrong with the disarmed state read at stand-up (empty when ready)."""
    p = []
    if not s["mflags"] & OBS_CONFIGURED:
        p.append(f"motor not configured (motor_state flags 0x{s['mflags']:02x})")
    if not s["mflags"] & OBS_CAL_ACCEPTED:
        p.append(f"calibration not accepted (motor_state flags 0x{s['mflags']:02x}, bit 7 clear)")
    if s["fault"]:
        p.append(f"motor_fault 0x{s['fault']:04x}")
    if s["moe"]:
        p.append(f"moe_bits 0x{s['moe']:02x}")
    if s["mode"] != MODE_OFF:
        p.append(f"mode_byte {MODE_NAMES.get(s['mode'], s['mode'])}, not OFF")
    if s["cmode"] != 0:
        p.append(f"control_mode {s['cmode']} (balance): the demand path here is throttle's; "
                 "restore control.mode to 0")
    return p


def gate1_verdict(samples, staged_cpa):
    peaks = [s["peak"] for s in samples]
    chopped = max(s["chopped"] for s in samples)
    trips = max(s["trips"] for s in samples)
    r = {
        "peak_max": max(peaks), "peak_mean": _mean(peaks), "chopped_max": chopped,
        "trips_max": trips, "cal": samples[-1]["cal"], "n": len(samples),
    }
    ok = chopped == 0 and trips == 0
    r["verdict"] = "INFO" if ok else "FAIL"
    r["lines"] = [
        f"motor_cal 0x{r['cal']:08x}",
        f"peak max {r['peak_max']} counts, mean {r['peak_mean']:.0f} counts "
        f"({r['peak_max'] / staged_cpa:.2f} A equivalent at this board's {staged_cpa} counts/A), "
        f"n={r['n']}",
        f"chopped max {chopped} (must be 0), trips {trips} (must be 0)",
    ]
    return r


def _checks_result(checks, extra_lines=()):
    ok = all(c[1] for c in checks)
    lines = [f"{'PASS' if c[1] else 'FAIL'}: {c[0]} ({c[2]})" for c in checks]
    return {"verdict": "PASS" if ok else "FAIL", "lines": lines + list(extra_lines), "checks": checks}


def gate2_verdict(samples, fmax):
    lim = FLOOR_FACTOR * fmax
    pk = max(s["peak"] for s in samples)
    modes = sorted({MODE_NAMES.get(s["mode"], str(s["mode"])) for s in samples})
    checks = [
        ("mode_byte stays OFF", all(s["mode"] == MODE_OFF for s in samples), "seen " + ",".join(modes)),
        ("moe_bits 0", all(s["moe"] == 0 for s in samples), f"max 0x{max(s['moe'] for s in samples):02x}"),
        (f"peak within {FLOOR_FACTOR}x of the rest floor", pk <= lim, f"max {pk} vs {lim:.0f} counts"),
        ("chopped 0", all(s["chopped"] == 0 for s in samples), f"max {max(s['chopped'] for s in samples)}"),
    ]
    info = [f"INFO: sub_state seen {sorted({s['sub'] for s in samples})} (may leave 0: the gate opens "
            "on demand, the bridge is not enabled)",
            f"INFO: DEMAND static max {max(s['demand'] for s in samples)}"]
    return _checks_result(checks, info)


def _within(samples, t0, secs):
    return [s for s in samples if s["t"] - t0 <= secs + 1e-9]


def refusal_text(s, since=None):
    """What CTRL_OBS word 33 says about the arm refusals (specs/integration.md, "The arm refusals").

    The two counts are per BOOT and nothing clears them; `since` adds what changed across an
    attempt. They are the only record a bench read gets of a REFUSED re-read, because that refusal
    clears on the first pass whose resulting mode is OFF, so a board refused for a bad axis frame
    and left in its OFF dwell reads exactly like a board that was never engaged. The confirm count
    is the other refusal, the one that stays held for the boot.

    The cause names the frame check that refused the value-row re-read, so an operator who wrote a
    bad `imu.axis_role` / `imu.axis_sign` from the app sees which field to fix rather than a board
    that will not arm for no stated reason.
    """
    text = f"arm refusals: re-read {s['reread_refusals']}, confirm {s['confirm_refusals']}"
    if since is not None:
        text += (f" (+{s['reread_refusals'] - since['reread_refusals']} re-read, "
                 f"+{s['confirm_refusals'] - since['confirm_refusals']} confirm during the attempt)")
    if s["reread_refusals"]:
        text += f", cause: {s['refusal_cause']}"
    return text


def arm_ok(samples, t0):
    """(ok, detail): mode RUN with moe_bits set within ARM_WITHIN_S of the hold starting."""
    for s in _within(samples, t0, ARM_WITHIN_S):
        if s["mode"] == MODE_RUN and s["moe"]:
            return True, f"RUN with moe 0x{s['moe']:02x} at +{s['t'] - t0:.1f} s"
    last = samples[-1] if samples else None
    detail = "no sample" if last is None else (
        f"last mode {MODE_NAMES.get(last['mode'], last['mode'])}, moe 0x{last['moe']:02x}, "
        f"motor_fault 0x{last['fault']:04x}; {refusal_text(last, samples[0])}")
    return False, f"not RUN with MOE within {ARM_WITHIN_S:.0f} s ({detail})"


def rotor_moved(samples):
    """The nonzero `motor_speed` that means the rotor TURNED, or 0 for none. Hall jitter at a held
    rotor does not count.

    `motor_speed` is a SIGNED hall-edge count per 20 ms window, so the running sum over the samples
    is the net displacement in edges, and that is what the rule is on: a window whose net reaches
    +-2 edges has turned, while a +1 followed by a -1 has not moved at all.

    A rotor held by a strap or a hand dithers across a hall edge. MEASURED 2026-10-09 in a gate-4
    arm soak: speed +1 in one sample of 94 and -1 in another 2.6 s later, everything else 0 and
    `peak` at the noise floor throughout, which ended the session two gates from the end. The
    earlier rule (more than one unit in one sample, or two CONSECUTIVE samples of the same sign)
    kept that case passing but admitted a unidirectional creep: a rotor turning slower than one
    hall edge per sample reads 1,0,0,1,0,1, which is never consecutive and is three edges one way.

    The return is the net at the sample that reached the bound, so it carries the direction."""
    net = 0
    for s in samples:
        net += s["speed"]
        if abs(net) >= ROTOR_NET_EDGES:
            return net
    return 0


def soak_abort(samples, fmax):
    """Abort reason for the armed still soak, None when the wheel sat still at the floor.

    The test is on the soak's P90 peak, not on any single sample and not on its median. What this
    check exists to catch is current flowing while the board is armed and undemanded, and `peak` is
    itself a window maximum over ADC noise, so its per-sample extremes cross any threshold near the
    floor eventually. Measured 2026-10-09: an armed, undemanded soak (demand, speed and duty 0 in
    all 94 samples) ran median 1072 and p90 1440 counts with a single sample at 2112, against a
    disarmed gate-1 maximum of 2097 in the same session. Comparing that one sample to a threshold
    aborted a soak that was indistinguishable from rest.

    The median is too weak in the other direction: it passes anything flowing in fewer than half
    the samples, which is exactly what an intermittently conducting gate looks like. p90 is the
    quantile that catches that and still clears the measured noise by better than a factor of two
    (the quiet soaks of that day ran p90 1366 to 1440 against 1.5x floors near 3,100). A per-sample
    ceiling stays for a gross fault, at 2x the floor: the worst quiet sample measured was 2112
    counts against a 2097 floor, so 2x leaves it 2,000 counts of room."""
    lim = FLOOR_FACTOR * fmax
    gross = SOAK_GROSS_FACTOR * fmax
    p90 = _percentile((s["peak"] for s in samples), SOAK_PCT)
    if samples and p90 > lim:
        return (f"p{SOAK_PCT} peak {p90:.0f} counts over the soak is above the rest floor ({lim:.0f}): "
                "current is flowing while the board is armed and undemanded")
    # Armed, undemanded, and the limiter chopping means the staged limit is inside the sense
    # chain's noise: MEASURED 2026-10-09, 2 of 94 soak samples chopped at a 2,000-count limit with
    # demand, speed and duty 0 throughout (climit-092353.csv). The floor exists to prevent exactly
    # this, so a chop here says the floor is too low, not that the board is faulty.
    chopped = max((s["chopped"] for s in samples), default=0)
    if chopped:
        return (f"the limiter chopped {chopped} period(s) during the armed STILL soak: the staged limit "
                "is inside the sense chain's noise floor")
    moved = rotor_moved(samples)
    if moved:
        return f"{ROTOR_MOVED}: motor_speed {moved} during the armed still soak"
    for s in samples:
        if s["peak"] > gross:
            return (f"peak {s['peak']} counts, over {SOAK_GROSS_FACTOR:g}x the rest floor "
                    f"({gross:.0f}), during the armed still soak")
        if s["fault"]:
            return f"motor_fault 0x{s['fault']:04x} during the armed still soak"
        if s["mode"] != MODE_RUN or not s["moe"]:
            return f"the arm dropped during the still soak (mode {MODE_NAMES.get(s['mode'])}, moe {s['moe']})"
    return None


def spin_ok(samples, t0):
    """(ok, detail): sub_state leaves 0 and motor_speed is nonzero within SPIN_WITHIN_S, and the
    speed keeps one sign for the SPIN_STEADY_S after."""
    first = None
    for s in _within(samples, t0, SPIN_WITHIN_S):
        if s["sub"] != SUB_IDLE and s["speed"]:
            first = s
            break
    if first is None:
        subs = sorted({s["sub"] for s in samples})
        return False, (f"sub_state {subs} / motor_speed not nonzero within {SPIN_WITHIN_S:.0f} s: "
                       "read the D4 phase-order table in specs/arm-session.md")
    sign = 1 if first["speed"] > 0 else -1
    tail = [s for s in samples if first["t"] <= s["t"] <= first["t"] + SPIN_STEADY_S + 1e-9]
    if any(s["speed"] * sign <= 0 for s in tail):
        return False, ("motor_speed changed sign or stopped: read the D4 phase-order table in "
                       "specs/arm-session.md")
    return True, (f"spinning at +{first['t'] - t0:.1f} s, speed {first['speed']} "
                  f"(sign {'+' if sign > 0 else '-'}), steady")


def stopped_ok(samples):
    """The machine stopped: `motor_speed` 0 with nothing commanded (`demand`, `torque` and the duty
    all zero).

    NOT `sub_state`, which this used to require. MEASURED 2026-10-09 on the master: sub_state is 0
    through the armed soak, leaves 0 on the first demand, and then STAYS at Run (3) for as long as
    the board is armed. It came back 3 in all 94 samples of a 10 s release window in which speed,
    demand, torque and every duty word read 0 and `peak` was back at its noise floor, so the gate
    aborted a release that had in fact happened. It returns to 0 on a disarm or a trip (gate 5
    checks exactly that), not on a Neutral."""
    if not samples:
        return False
    last = samples[-1]
    return (last["speed"] == 0 and last["demand"] == 0 and last["torque"] == 0
            and last["duty_on"] == 0)


def disarmed_ok(samples):
    return bool(samples) and samples[-1]["mode"] == MODE_OFF and samples[-1]["moe"] == 0


def cal_signal_weak(mean_peak, floor_counts, ladder_peaks):
    """Why the calibration window cannot carry a verdict, or None when it can
    (specs/current-limit-session.md, "The calibration verdict must say INCONCLUSIVE when the signal
    is in the noise"). Either condition makes the measured ratio an UPPER BOUND and nothing more:

    - the window's mean `peak` is under twice the gate-1 rest-floor maximum, so most of what was
      measured is noise. `peak` is a window MAXIMUM over ADC samples, so noise pushes it one way
      only, up, and the ratio with it. Measured 2026-10-09: a 29 percent rise in current moved
      `peak` by 4 percent;
    - fewer than two ladder points are separated by more than the floor, so there is no slope in
      the data, only single-point ratios. Measured the same day: the single-point ratios read 455
      and 459 where the two-point slopes within one session read 81 and 306."""
    reasons = []
    if mean_peak < CAL_SIGNAL_FACTOR * floor_counts:
        reasons.append(f"the window's mean peak {mean_peak:.0f} counts is under {CAL_SIGNAL_FACTOR:g}x the "
                       f"{floor_counts:.0f}-count rest floor")
    peaks = list(ladder_peaks)
    span = (max(peaks) - min(peaks)) if len(peaks) >= 2 else 0.0
    if span <= floor_counts:
        how = ("no ladder was run" if not peaks else
               "the ladder has one point only" if len(peaks) == 1 else
               f"the ladder's {len(peaks)} points span {span:.0f} counts, inside the "
               f"{floor_counts:.0f}-count floor")
        reasons.append(f"there are not two points whose peak separation exceeds the floor ({how})")
    return " and ".join(reasons) or None


def calibration(samples, psu_a, clamp_a, staged_cpa, quiescent_a=0.0, *, floor_counts,
                ladder_peaks=()):
    """The duty-corrected counts-per-amp estimate (specs/current-limit-session.md, "What the two
    currents are"): I_phase ~= I_link * 2250 / duty_on, and a clamp-meter reading overrides it.

    `quiescent_a` is the board's OWN draw, read armed and undemanded, and it is subtracted first.
    It is not a refinement: at the link currents this gate runs at (a few hundred mA) the board's
    logic, LEDs and BLE module are a large fraction of the PSU reading, and the duty correction then
    multiplies the error by 2250/duty_on (about 13x at demand 3000). Leaving it in read 382 counts
    per amp against a provisional 800 on 2026-10-09, with a mean peak barely above the gate-1 noise
    floor, which is the signature of an inflated reference current rather than a wrong scale.

    `staged_cpa` is what the board currently carries in 0x67, which is what the measurement is
    judged against: this gate confirms or corrects THAT board's scale, and a correction is staged
    back into 0x67 rather than edited into a source file.

    `floor_counts` (gate 1's rest-floor maximum) and `ladder_peaks` (each ladder step's mean peak)
    are what decide whether the measurement can carry a verdict at all: see `cal_signal_weak`. When
    it cannot, the verdict is INCONCLUSIVE and the ratio is reported as an upper bound, which is
    neither a confirmation nor a correction. The braking fallback runs no ladder, so it reaches
    INCONCLUSIVE by construction."""
    mean_peak = _mean(s["peak"] for s in samples)
    duty_on = _mean(s["duty_on"] for s in samples)
    chopped = max(s["chopped"] for s in samples)
    i_link = max(psu_a - quiescent_a, 0.0)
    i_est = phase_estimate(i_link, duty_on)
    if clamp_a is not None:
        i_ref, source = clamp_a, "clamp meter"
    else:
        i_ref, source = i_est, "PSU, duty-corrected"
    r = {"mean_peak": mean_peak, "duty_on": duty_on, "psu_a": psu_a, "clamp_a": clamp_a,
         "quiescent_a": quiescent_a, "i_link": i_link,
         "i_est": i_est, "i_ref": i_ref, "source": source, "chopped_max": chopped,
         "cpa": None, "proposed": None}
    lo, hi = CPA_LO * staged_cpa, CPA_HI * staged_cpa
    if chopped:
        r["verdict"] = "INVALID"
        r["recommendation"] = (f"the window was chopped (max {chopped}): the peak is the staged limit's, "
                               "not the load's. Re-run with the limit at its default.")
    elif not i_ref or i_ref <= 0:
        r["verdict"] = "INVALID"
        r["recommendation"] = "no reference current (duty_on read 0 and no clamp reading)"
    else:
        cpa = mean_peak / i_ref
        r["cpa"] = cpa
        weak = cal_signal_weak(mean_peak, floor_counts, ladder_peaks)
        if weak:
            r["verdict"] = "INCONCLUSIVE"
            r["recommendation"] = (
                f"measured {cpa:.0f} counts per amp, an UPPER BOUND and nothing more: {weak}. "
                f"`peak` is a window maximum over ADC noise, which can only push it up, so the true "
                f"scale is at or below this. Neither a confirmation of the staged {staged_cpa} nor a "
                "correction: the measurement that can carry one is the sweep in "
                'specs/motor-integration.md, "The current-sense calibration".')
        elif lo <= cpa <= hi:
            r["verdict"] = "CONFIRMED"
            r["recommendation"] = (f"the staged motor.current_cal {staged_cpa} confirmed: measured "
                                   f"{cpa:.0f}, inside {lo:.0f}..{hi:.0f}")
        else:
            r["verdict"] = "CORRECTION"
            r["proposed"] = int(round(cpa))
            r["recommendation"] = (f"measured {cpa:.0f} counts per amp, outside {lo:.0f}..{hi:.0f}: "
                                   f"propose motor.current_cal (0x67) = {r['proposed']} on THIS board "
                                   f"(swd-mailbox-config <endpoint> --dst attached "
                                   f"0x{CURRENT_CAL_FIELD:02x}={r['proposed']}, then a power-cycle). "
                                   "NOT staged: the owner stages it after reading this record.")
    est = "n/a" if i_est is None else f"{i_est:.2f} A"
    clamp = "none" if clamp_a is None else f"{clamp_a:g} A"
    r["lines"] = [
        f"mean peak {mean_peak:.0f} counts, mean duty_on {duty_on:.0f} of {PWM_PERIOD}, chopped max {chopped}",
        f"PSU {psu_a:g} A less the board's {quiescent_a:g} A = {i_link:g} A through the bridge "
        f"-> duty-corrected phase estimate {est}; clamp meter {clamp}",
        f"I_ref {('n/a' if not i_ref else f'{i_ref:.2f} A')} ({source})"
        + ("" if r["cpa"] is None else f" -> {r['cpa']:.0f} counts per amp"),
        r["recommendation"],
    ]
    return r


def gate4_verdict(samples, limit_ma, staged_cpa, psu_a, locked=True, cal=None):
    """Locked rotor: motor_speed 0 throughout. Braking fallback: the wheel still turns. `cal` (the
    gate-3 calibration, when it ran) gives the unchopped PSU the gate-3 relation predicts."""
    lc = limit_counts(limit_ma, staged_cpa)
    n = len(samples)
    chopped_n = sum(1 for s in samples if s["chopped"])
    moving_n = sum(1 for s in samples if s["speed"])
    trips = {s["trips"] for s in samples}
    mean_peak = _mean(s["peak"] for s in samples)
    duty_on = _mean(s["duty_on"] for s in samples)
    checks = [
        ("chopped nonzero in most samples", chopped_n * 2 > n, f"{chopped_n}/{n}"),
        ("trips 0", trips == {0}, f"seen {sorted(trips)}"),
        (("motor_speed 0 (rotor locked)", moving_n == 0, f"{moving_n}/{n} nonzero") if locked else
         ("motor_speed nonzero (the wheel still turns)", moving_n * 2 > n, f"{moving_n}/{n}")),
        (f"peak near the limit ({G4_PEAK_LO:.0%}..{G4_PEAK_HI:.0%} of {lc} counts)",
         G4_PEAK_LO * lc <= mean_peak <= G4_PEAK_HI * lc, f"mean {mean_peak:.0f} counts"),
    ]
    limit_a = limit_ma / 1000.0
    raw_ok = abs(psu_a - limit_a) <= G4_PSU_TOL * limit_a
    corr = phase_estimate(psu_a, duty_on)
    corr_txt = "n/a" if corr is None else f"{corr:.2f} A"
    info = [f"INFO: duty-corrected phase estimate {corr_txt} (PSU {psu_a:g} A, duty_on {duty_on:.0f}) vs the "
            f"{limit_a:g} A limit. Judged on the firmware words, not the PSU."]
    if not locked:
        info.append(f"INFO: PSU {psu_a:g} A vs limit {limit_a:g} A raw: {'within' if raw_ok else 'outside'} "
                    f"{G4_PSU_TOL:.0%}")
    if locked and cal and cal.get("duty_on") and duty_on > 0:
        # Locked rotor: phase current ~ duty, so the DC-link mean ~ duty^2 without the chop.
        pred = cal["psu_a"] * (duty_on / cal["duty_on"]) ** 2
        info.append(f"INFO: the gate-3 relation predicts {pred:.2f} A on the PSU at this duty without the chop; "
                    f"read {psu_a:g} A ({'below: the chop is holding' if psu_a < pred else 'NOT below'})")
    r = _checks_result(checks, info)
    r.update({"limit_counts": lc, "mean_peak": mean_peak, "psu_a": psu_a, "psu_corrected_a": corr})
    return r


def gate5_verdict(pre, post, base):
    """`pre`: samples from the trip demand until the trip (or 5 s); `post`: the window after the
    demand was dropped; `base`: the sample taken just before the demand (the counters' zero)."""
    allx = pre + post
    trips0 = base["trips"]
    tripped = [s for s in allx if s["trips"] != trips0]
    after = allx[allx.index(tripped[0]):] if tripped else []
    final = allx[-1]
    delta = (final["trips"] - trips0) & 0xFF
    latch_delta = final["latch_a"] - base["latch_a"]
    odd_seen = any(s["latch_a"] % 2 for s in after)
    shut_delta = (final["shutdowns"] - base["shutdowns"]) & M32
    chopped_max = max(s["chopped"] for s in allx)
    left_run = any(s["mode"] != MODE_RUN for s in after) or shut_delta >= 1
    # What a trip looks like, MEASURED 2026-10-09 (specs/current-limit-session.md): with the arm
    # still held the board enacts a shutdown and re-enters through the SOFT START, so `sub_state`
    # reaches 1 (Arming) rather than 0, MOE is restored inside one 50 ms sample (so `enact_shutdowns`
    # is the evidence it cleared, not an observed 0), and `chopped` saturates near 25 of 64 because
    # the hard trip pre-empts the chopping. The gate used to require sub 0, moe 0 and chopped >= 32
    # and failed all three against correct firmware.
    subs_after = sorted({s["sub"] for s in after})
    checks = [
        ("trips +1 exactly", delta == 1, f"{trips0} -> {final['trips']}"),
        ("sub_state leaves Run after the trip", any(s["sub"] in (SUB_IDLE, SUB_ARMING) for s in after),
         f"sub_state seen {subs_after}" if after else "no trip"),
        ("mode_byte leaves RUN or a shutdown is enacted", left_run,
         f"modes {sorted({MODE_NAMES.get(s['mode']) for s in after})}, enact_shutdowns +{shut_delta}"),
        ("event_counts latch-A rose", latch_delta >= 1,
         f"{base['latch_a']} -> {final['latch_a']}, odd seen: {'yes' if odd_seen else 'no'}"),
        ("motor_fault unchanged (the trip sets no FAULT bit)", all(s["fault"] == base["fault"] for s in allx),
         f"0x{final['fault']:04x}"),
    ]
    r = _checks_result(checks, [
        f"INFO: chopped max {chopped_max} of 64 (not a gate: the hard trip can pre-empt the chop, and "
        "25 of 64 was measured on 2026-10-09)",
        f"INFO: moe_bits after the trip {sorted({s['moe'] for s in after})} (restored inside one sample; "
        f"enact_shutdowns +{shut_delta} is the evidence it cleared)",
    ])
    r.update({"trip_at": None if not tripped else tripped[0]["t"]})
    return r


def rearm_verdict(off_samples, arm_samples, t0, still_samples):
    ok_arm, arm_detail = arm_ok(arm_samples, t0)
    checks = [
        ("OFF pass after the hold ended", disarmed_ok(off_samples),
         "last mode " + (MODE_NAMES.get(off_samples[-1]["mode"]) if off_samples else "n/a")),
        ("re-arm: RUN and MOE again", ok_arm, arm_detail),
        ("wheel still after the re-arm", all(s["speed"] == 0 for s in still_samples),
         f"max |speed| {max([abs(s['speed']) for s in still_samples] or [0])}"),
    ]
    return _checks_result(checks)


# --------------------------------------------------------------------------------------------------
# The RECORD renderer (pure).
# --------------------------------------------------------------------------------------------------
def render_record(rec):
    p = rec["params"]
    lc = limit_counts(p["limit_ma"], rec["staged_cpa"])
    out = [f"# Current-limit session, {rec['date']}", ""]
    out.append(f"Tool `tools/climit-session.py` ({TOOL_VERSION}), board {p['board']}, "
               f"attached node {rec.get('node_txt', 'not resolved')}.")
    out.append(f"ELF `{rec['elf']}`, HEAD `{rec['head']}`. Evidence CSV `{rec['csv']}`.")
    out += ["", "## Parameters", "", "| parameter | value |", "|---|---|"]
    out.append(f"| staged limit (gates 4, 5) | {p['limit_ma']} mA = {lc} counts, hard trip "
               f"{hard_trip_counts(lc)} counts |")
    out.append(f"| calibration / plateau demand | {p['cal_demand']} |")
    out.append(f"| trip demand | {p['trip_demand']} |")
    out.append(f"| rotor, gates 3 to 5 | {rec['rotor']} |")
    psu = rec.get("psu_declared")
    out.append(f"| PSU current limit declared | {'not asked' if psu is None else f'{psu:g} A'} "
               f"(rule: at least {psu_target_a(p['limit_ma']):g} A, clearing the "
               f"{link_at_chop_a(p['limit_ma']):.2f} A the link carries at the chop; never above "
               f"{PSU_CAP_A:g} A) |")
    prev = rec.get("prev_limit_ma")
    out.append(f"| staged limit before the session (0x20) | {'not read' if prev is None else f'{prev} mA'} |")
    raw = rec.get("cal_raw")
    eff = rec["staged_cpa"]
    out.append(f"| counts per amp (0x67), read off the board | "
               f"{'not read' if raw is None else f'{raw}'}"
               f"{'' if raw is None or raw == eff else f', boot-seam clamped to {eff}'} |")
    skips = ", ".join(f"gate {g}" for g in p["skip"]) or "none"
    out.append(f"| skipped | {skips} |")
    out += ["", "## Outcome", "", rec["outcome"], ""]
    if rec.get("warnings"):
        out += ["## Warnings", ""] + [f"- {w}" for w in rec["warnings"]] + [""]
    out += ["## Gates", ""]
    if not rec["gates"]:
        out += ["No gate ran.", ""]
    for g in rec["gates"]:
        out.append(f"### {g['name']}: {g['verdict']}")
        out.append("")
        out += [f"- {line}" for line in g["lines"]]
        out.append("")
    cal = rec.get("calibration")
    out += ["## Counts per amp", ""]
    if cal:
        out.append(f"**{cal['verdict']}.** " + cal["recommendation"])
        out.append("")
        out += [f"- {line}" for line in cal["lines"][:-1]]
    else:
        out.append("Not measured this session.")
    out += ["", "## Operator readings, verbatim", ""]
    if rec["typed"]:
        out += [f"- {q} -> `{a}`" for q, a in rec["typed"]]
    else:
        out.append("None.")
    out += ["", "## Teardown", ""]
    out += [f"- {line}" for line in rec["teardown"]] or ["- not run yet (record flushed before teardown)"]
    out += ["", "## Bench state at close", ""]
    out += [f"- {line}" for line in rec["final"]]
    out.append("")
    return "\n".join(out)


# --------------------------------------------------------------------------------------------------
# The evidence CSV (the imu-tilt.py v3 precedent: `#` comment lines make it self-describing).
# --------------------------------------------------------------------------------------------------
CSV_COLUMNS = ("unix_time,label,boot,ctl_ticks,mode,moe,sub,cmode,torque,peak,chopped,trips,speed,"
               "d0,d1,d2,duty_on,fault,dwell,mflags,cal,latch_a,shutdowns,battery,demand,s_speed,"
               "s_fault,s_periods,ctrl_obs")


class EvidenceCsv:
    def __init__(self, fh):
        self.fh = fh
        self.comment(f"{TOOL_VERSION} evidence; specs/current-limit-session.md")
        self.comment(f"columns: {CSV_COLUMNS}")
        self.comment("units: peak in stock current counts (16 per 12-bit ADC LSB; the board's own "
                     "counts per amp is read from 0x67 at stand-up and commented below); duties of "
                     f"{PWM_PERIOD}; ctrl_obs = the {CTRL_OBS_WORDS} CTRL_OBS words in hex, word 0 first")

    def comment(self, text):
        for line in str(text).splitlines() or [""]:
            self.fh.write(f"# {line}\n")
        self.fh.flush()

    def row(self, s):
        words = " ".join(f"{v:08x}" for v in s["words"])
        self.fh.write(
            f"{s['t']:.3f},{s['label']},{s['boot']},{s['ctl']},{s['mode']},{s['moe']},{s['sub']},"
            f"{s['cmode']},{s['torque']},{s['peak']},{s['chopped']},{s['trips']},{s['speed']},"
            f"{s['d0']},{s['d1']},{s['d2']},{s['duty_on']},0x{s['fault']:04x},{s['dwell']},"
            f"0x{s['mflags']:02x},0x{s['cal']:08x},{s['latch_a']},{s['shutdowns']},{s['battery']},"
            f"{s['demand']},{s['s_speed']},0x{s['s_fault']:x},{s['s_periods']},{words}\n"
        )
        self.fh.flush()


# --------------------------------------------------------------------------------------------------
# The side-effect seam. RealShell does it; FakeShell (below) simulates it.
# --------------------------------------------------------------------------------------------------
class Result:
    def __init__(self, rc, stdout):
        self.returncode = rc
        self.stdout = stdout


class RealChild:
    """A child process under the watchdog (`--child-watchdog`): when this tool dies by any means,
    the watchdog's stdin closes and it interrupts the child, so a hold never outlives the tool."""

    def __init__(self, argv, tag):
        self.tag = tag
        self.lines = []
        self.proc = subprocess.Popen(
            [sys.executable, os.path.abspath(__file__), "--child-watchdog", "--"] + list(argv),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            bufsize=1, start_new_session=True,
        )
        self._t = threading.Thread(target=self._pump, daemon=True)
        self._t.start()

    def _pump(self):
        for line in self.proc.stdout:
            self.lines.append(line.rstrip("\n"))

    def alive(self):
        return self.proc.poll() is None

    def send(self, line):
        """One command to the child's stdin, through the watchdog that forwards it. A broken pipe is
        swallowed: the child being gone is an ARM question, and `check_arm` is what answers it."""
        try:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
        except (OSError, ValueError):
            pass

    def end(self, timeout=8.0):
        """Interrupt the child (its own SIGINT handling: the inputs tool sends an all-clear, the
        drive tool exits and the firmware decays the demand) and wait for it."""
        if self.proc.poll() is None:
            try:
                self.proc.stdin.close()
            except OSError:
                pass
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self._t.join(timeout=1.0)
        for f in (self.proc.stdin, self.proc.stdout):
            try:
                f.close()
            except (OSError, ValueError):
                pass


class RealShell:
    dry = False

    def __init__(self, log=print):
        self.log = log

    def run(self, cmd):
        self.log(f"  + {cmd}")
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return Result(r.returncode, r.stdout + r.stderr)

    def spawn(self, argv, tag):
        self.log(f"  + {' '.join(shlex.quote(a) for a in argv)} &   ({tag})")
        return RealChild(argv, tag)

    def connect(self, host, port):
        return swdobs.Ocd(host, port)

    def port_open(self, host, port):
        try:
            socket.create_connection((host, port), timeout=1).close()
            return True
        except OSError:
            return False

    def exists(self, path):
        return os.path.exists(path)

    def sleep(self, s):
        time.sleep(s)

    def now(self):
        return time.time()

    def ask(self, prompt):
        try:
            return input(f"\n>> {prompt} ").strip()
        except EOFError:
            raise SessionAbort("stdin closed at a prompt")

    def say(self, text):
        print(text, flush=True)

    def defer_signals(self):
        return DeferredSignals()


class DeferredSignals:
    """While active, SIGINT/SIGTERM/SIGHUP are noted, not raised, so nothing interrupts the teardown.
    (Swapping the handlers rather than blocking: Python runs handlers in the main thread whichever
    thread the OS delivers to, so a per-thread mask would not hold.)"""

    SIGS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)

    def __init__(self):
        self.pending = []
        self.saved = {}

    def __enter__(self):
        for sig in self.SIGS:
            self.saved[sig] = signal.signal(sig, lambda signum, _f: self.pending.append(signum))
        return self

    def __exit__(self, *exc):
        for sig, handler in self.saved.items():
            signal.signal(sig, handler)
        return False


def child_watchdog(argv):
    """`--child-watchdog -- CMD...`: run CMD, FORWARDING our stdin to it line by line; on EOF (the
    parent ended the hold, or died), interrupt it, then kill it if it lingers.

    The forwarding is what lets the session owner take `value N` / `neutral` from this tool without
    a second process attaching the mailbox, which would flush the owner's ring and expire its arm
    (`specs/swd-mailbox.md`, "Attach + session flush")."""
    signal.signal(signal.SIGINT, lambda *_: None)  # a caught handler resets to default in the child
    proc = subprocess.Popen(argv, stdin=subprocess.PIPE, text=True, bufsize=1)

    def watch():
        try:
            for line in sys.stdin:
                if proc.poll() is not None:
                    break
                proc.stdin.write(line)
                proc.stdin.flush()
        except (OSError, ValueError):
            pass
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    threading.Thread(target=watch, daemon=True).start()
    return proc.wait()


# --------------------------------------------------------------------------------------------------
# The simulated board behind FakeShell: what the dry run and the tests talk to.
# --------------------------------------------------------------------------------------------------
SIM_SYMS = {
    "CTRL_OBS": (0x20000AC8, 4 * CTRL_OBS_WORDS),
    "DEMAND_SEQ": (0x200008E8, 4), "OBS_DUTY01": (0x200008EC, 4), "INVALID_DWELL": (0x200008F4, 4),
    "COUNTER_RUNNING": (0x200008F8, 1), "OBS_DUTY2_ANGLE": (0x200008FC, 4), "FAULT": (0x20000904, 4),
    "SPEED": (0x20000908, 4), "DEMAND": (0x2000090C, 4), "OBS_CAL": (0x20000910, 4),
    "PERIODS": (0x20000914, 4), "OBS_STATE": (0x20000918, 4),
}


def encode_ctrl_obs(f):
    """The CTRL_OBS words from a field dict (the inverse of decode_sample's CTRL_OBS half)."""
    w = [0] * CTRL_OBS_WORDS
    w[0] = f.get("magic", CTRL_MAGIC)
    w[W_BOOT_COUNT] = f.get("boot", 1)
    w[W_TICK_COUNT] = f.get("tick", 0) & M32
    w[W_CONTROL_TICKS] = (f.get("tick", 0) - 1) & M32
    w[W_ENACT_INITS] = f.get("inits", 0)
    w[W_ENACT_SHUTDOWNS] = f.get("shutdowns", 0)
    w[W_TORQUE_MODE] = (f.get("torque", 0) & 0xFFFF) | (f.get("mode", 0) << 16) | (f.get("moe", 0) << 24)
    w[W_SUB_FLAGS] = f.get("sub", 0) | (f.get("cmode", 0) << 8) | (f.get("flags", 3) << 16)
    w[W_PERIODS] = f.get("periods", 0) & M32
    w[W_MOTOR_STATE] = f.get("hall", 1) | (f.get("mflags", 0x87) << 24)
    w[W_DUTY01] = f.get("d0", 0) | (f.get("d1", 0) << 16)
    w[W_DUTY2_ANGLE] = f.get("d2", 0)
    w[W_MOTOR_FAULT] = f.get("fault", 0)
    w[W_MOTOR_SPEED] = f.get("speed", 0) & M32
    w[W_MOTOR_CAL] = f.get("cal", 0x7F107E07)
    ev = f.get("ev", [0] * 8)
    w[W_EVENTS_LO] = sum(ev[i] << (8 * i) for i in range(4))
    w[W_EVENTS_HI] = sum(ev[4 + i] << (8 * i) for i in range(4))
    w[W_MOTOR_CURRENT] = (f.get("peak", 0) & 0xFFFF) | (f.get("chopped", 0) << 16) | ((f.get("trips", 0) & 0xFF) << 24)
    w[W_BATTERY] = f.get("battery", 2497)
    w[W_ARM_REFUSALS] = ((f.get("reread_refusals", 0) & 0xFF) | ((f.get("confirm_refusals", 0) & 0xFF) << 8)
                         | ((f.get("refusal_cause", 0) & 0xFF) << 16))
    return w


class FakeChild:
    WALK_S = 2.0

    def __init__(self, sim, kind, value, hold, tag, buttons=1):
        self.sim, self.kind, self.value, self.tag = sim, kind, value, tag
        self.buttons = buttons
        self.walked = sim.t + self.WALK_S       # the walk, then the dst line and the first send
        self.demand_t = self.walked             # when the demand last CHANGED (the stall clock)
        self.until = self.walked + hold
        self.ended = False
        self._lines = [f"dst resolved: attached node 0x{sim.node:02x} (port table: 0x{sim.node:02x} "
                       "reports the host on its port 0, kind 2 = SWD mailbox)"]

    @property
    def lines(self):
        return self._lines if self.sim.t >= self.walked or self.ended else []

    def alive(self):
        return not self.ended and self.sim.t < self.until

    def end(self, timeout=8.0):
        if self.alive():
            self._lines.append("RELEASED (interrupted)")
        self.ended = True
        self.sim.child_ended(self)

    def send(self, line):
        """The owner's stdin commands, as the simulated owner answers them."""
        self._lines.append(f"ok {line}")
        if line.startswith("value "):
            self.value = int(line.split()[1])
            self.demand_t = self.sim.t
        elif line == "neutral":
            self.value = 0
            self.sim.demand_released(self)


class SimBoard:
    """A nominal board: arms on a held power request, spins on demand, chops at the staged limit,
    trips on a stall at full demand. Knobs: `stuck_moe` keeps MOE set after the hold ends (the
    teardown's moe-check-failed path); `node` is the attached address the walk resolves."""

    def __init__(self, node=0x02, stuck_moe=False):
        self.t = 1_800_000_000.0
        self.node, self.stuck_moe = node, stuck_moe
        self.rail = False
        self.boot_t = None
        self.boot = 0
        self.store_limit = 10_000
        self.boot_limit = 10_000
        # The simulated board's own 0x67, which its peaks are generated against: a real board's
        # scale is read from it, so the fake has to have one too.
        self.store_cal = CURRENT_CAL_DEFAULT
        self.ocd = False
        self.inputs = None
        self.drive = None
        self.trips = 0
        self.latch = 0
        self.shutdowns = 0
        self.tripped = False
        self.ever_armed = False
        self.locked = True           # the operator's lock (the dry run assumes the default procedure)

    # The board's time is the shell's clock.
    def advance(self, s):
        self.t += s

    def _power_on(self):
        self.rail, self.boot_t = True, self.t
        self.boot += 1
        self.boot_limit = self.store_limit
        self.trips = self.latch = self.shutdowns = 0
        self.tripped = False

    def handle(self, cmd):
        if "pinctrl set 4 op dl" in cmd:
            if not self.rail:
                self._power_on()
            return Result(0, "")
        if "pinctrl set 4 op dh" in cmd:
            self.rail = False
            return Result(0, "")
        if "pinctrl get 4" in cmd:
            return Result(0, "4: op dh pu | hi // GPIO4 = output" if not self.rail else
                          "4: op dl pu | lo // GPIO4 = output")
        if "bench-lock.sh acquire" in cmd:
            return Result(0, "ACQUIRED by claude-climit")
        if "bench-lock.sh release" in cmd:
            return Result(0, "RELEASED")
        if "pkill -x openocd" in cmd:
            self.ocd = False
            return Result(0, "")
        if "nohup sudo openocd" in cmd:
            self.ocd = True
            return Result(0, "")
        if "Listening on port" in cmd:
            return Result(0, "1" if self.ocd else "0")
        if "rev-parse" in cmd:
            return Result(0, "0123456789abcdef0123456789abcdef01234567")
        if "cargo build" in cmd:
            return Result(0, "Finished")
        if "swd-mailbox-config" in cmd:
            return self._config(cmd)
        if "swd-mailbox-drive" in cmd:
            return Result(0, f"dst resolved: attached node 0x{self.node:02x} (port table)\n"
                             "sent 10 DRIVE frames, then an explicit Neutral\nPASS: demand released")
        return Result(0, "")

    def _config(self, cmd):
        dst = f"dst resolved: attached node 0x{self.node:02x} (port table)\n"
        cal_key = f"0x{CURRENT_CAL_FIELD:02x}"
        if cal_key in cmd:
            m = re.search(rf"{cal_key}(?::\d+)?=(\d+)", cmd)
            if m:
                self.store_cal = int(m.group(1))
            return Result(0, dst + f"1 CONFIG op(s) on node 0x{self.node:02x}\n"
                          f"  CONFIG_READ {cal_key}:0 -> CFG_OK value U16({self.store_cal})\n"
                          "PASS: 1 CONFIG op(s) answered CFG_OK")
        m = re.search(r"0x20=(\d+)", cmd)
        if m:
            self.store_limit = int(m.group(1))
            return Result(0, dst + f"1 CONFIG op(s) on node 0x{self.node:02x}\n"
                          f"  CONFIG_WRITE 0x20:0 = {m.group(1)} (U32) -> CFG_OK\n"
                          f"  CONFIG_READ  0x20:0 -> CFG_OK value U32({self.store_limit})  (write -> read matches)\n"
                          "PASS: 1 CONFIG op(s) answered CFG_OK")
        return Result(0, dst + f"1 CONFIG op(s) on node 0x{self.node:02x}\n"
                      f"  CONFIG_READ 0x20:0 -> CFG_OK value U32({self.store_limit})\n"
                      "PASS: 1 CONFIG op(s) answered CFG_OK")

    def spawn(self, argv, tag):
        def arg(name, default=0):
            return int(argv[argv.index(name) + 1]) if name in argv else default
        if any("swd-mailbox-session" in a for a in argv):
            # One child is both: it holds the levels and carries the demand, as the real owner does.
            c = FakeChild(self, "session", arg("--value"), arg("--hold"), tag,
                          buttons=arg("--buttons"))
            self.inputs = self.drive = c
        elif any("swd-mailbox-inputs" in a for a in argv):
            c = FakeChild(self, "inputs", arg("--buttons"), arg("--hold"), tag,
                          buttons=arg("--buttons"))
            self.inputs = c
        elif any("swd-mailbox-drive" in a for a in argv):
            c = FakeChild(self, "drive", arg("--value"), arg("--hold", 5), tag)
            self.drive = c
        else:
            c = FakeChild(self, "other", 0, 10 ** 6, tag)
            c._lines = []
        return c

    def child_ended(self, c):
        if c.kind in ("drive", "session"):
            self.demand_released(c)

    def demand_released(self, _c):
        if self.tripped:
            self.latch += 1          # the latch clears on the OFF pass once the demand is gone
            self.tripped = False

    def _armed(self):
        c = self.inputs
        live = c is not None and c.alive() and self.t >= c.walked and bool(c.buttons)
        return self.rail and live and not self.tripped

    def _demand(self):
        d = self.drive
        return d.value if d is not None and d.alive() and self.t >= d.walked else 0

    def fields(self):
        if not (self.rail and self.ocd and self.t - self.boot_t >= 2.0):
            raise RuntimeError("target unreachable (rail off or OpenOCD down)")
        tick = int((self.t - self.boot_t) * 250)
        armed = self._armed()
        demand = self._demand()
        lc = limit_counts(self.boot_limit, self.store_cal)
        f = {"boot": self.boot, "tick": tick, "periods": tick * 64, "shutdowns": self.shutdowns,
             "peak": 1100 + (tick % 7) * 20, "trips": self.trips}
        if armed:
            f.update(mode=MODE_RUN, moe=1)
            if demand >= 20_000:                      # a stall at full demand: chop, then trip
                f.update(sub=3, chopped=64, peak=int(lc * 1.3), d0=1956)
                if self.t - getattr(self.drive, "demand_t", self.drive.walked) >= 0.6:
                    self.trips += 1
                    self.latch += 1
                    self.shutdowns += 1
                    self.tripped = True
                    f.update(mode=MODE_OFF, moe=0, sub=0, trips=self.trips, shutdowns=self.shutdowns)
            elif demand >= ENGAGE_DEMAND_MIN and self.locked:
                duty, amps, _psu = sim_locked_rotor(demand)
                f.update(sub=3, d0=duty, peak=int(amps * self.store_cal))
                if f["peak"] > lc:
                    f.update(peak=int(lc * 1.1), chopped=40)
            elif demand >= ENGAGE_DEMAND_MIN:
                f.update(sub=3, speed=90, d0=1800)
                if self.boot_limit >= 10_000:
                    f.update(peak=int(4 * self.store_cal))   # 4 A of phase current
                else:
                    f.update(peak=int(lc * 1.1), chopped=40)
        elif self.stuck_moe and self.ever_armed:
            f.update(mode=MODE_RUN, moe=1)
        self.ever_armed = self.ever_armed or armed
        if self.tripped:
            f.update(mode=MODE_OFF, moe=0, sub=0)
        f["ev"] = [0, 0, 0, 0, self.latch, 0, 0, 0]
        return f

    def read_words(self, addr, n):
        self.advance(0.01)
        if addr == SIM_SYMS["CTRL_OBS"][0]:
            return encode_ctrl_obs(self.fields())[:n]
        f = self.fields()
        base, span, off = swdobs.motor_block({k: v[0] for k, v in SIM_SYMS.items()})
        m = [0] * span
        m[off["DEMAND"]] = self._demand() & M32
        m[off["SPEED"]] = f.get("speed", 0) & M32
        m[off["PERIODS"]] = f["periods"] & M32
        return m[:n]


SIM_LINK_V, SIM_PAIR_OHMS = 25.0, 1.2
# The simulated board's own link current, so the dry run exercises the subtraction rather than
# assuming it away: the operator's readings include it and the tool has to take it back out.
SIM_QUIESCENT_A = 0.15


def sim_ladder_end():
    """The demand the nominal ladder stops at: the first `--cal-demand` + n*step whose duty-corrected
    phase estimate reaches the band. Derived so the canned answers cannot drift from the plant."""
    d = 3000
    while d <= LADDER_MAX_DEMAND:
        _duty, amps, _psu = sim_locked_rotor(d)
        if amps >= LADDER_EST_LO_A:
            return d
        d += LADDER_STEP
    return LADDER_MAX_DEMAND


def sim_locked_rotor(demand):
    """(duty_on, phase amps, PSU amps) of the simulated locked rotor: static energisation of one
    winding pair at `|demand| * ARR / 32767`, the six-step driver's own scale, capped at the 1956
    the control path's +-28500 clamp allows. The PSU sees the phase current times the duty, which is
    why it reads well under an amp while several flow in the winding."""
    # MEASURED 2026-10-09: a wire demand of 3000 reaches the motor as 2593 and applies duty 178.
    # The path is the throttle conditioning, through the EFeru +-1000 domain and back out, with a
    # truncation each way: 3000 * 1000 / 32767 = 91, 91 * 57 / 2 = 2593, 2593 * 2250 / 32767 = 178.
    # So the +-32767 wire word loses about 14 percent and quantises to 1.96 duty counts per step.
    eferu = int(demand * 1000 / 32767)
    reference = int(eferu * 57 / 2)
    duty = min(1956, int(reference * PWM_PERIOD / 32767))
    frac = duty / PWM_PERIOD
    amps = SIM_LINK_V * frac / SIM_PAIR_OHMS
    return duty, amps, amps * frac


class FakeReader:
    def __init__(self, sim):
        self.sim = sim

    def read_words(self, addr, n):
        return self.sim.read_words(addr, n)

    def close(self):
        pass


class FakeShell:
    """The dry-run and test shell: logs every command and prompt, executes nothing, and answers
    from SimBoard. `answers(prompt) -> str` supplies the operator."""

    dry = True

    def __init__(self, sim=None, answers=None, echo=True, exists=lambda path: True):
        self.sim = sim or SimBoard()
        self.answers = answers or nominal_answers
        self.echo = echo
        self._exists = exists
        self.built = False
        self.log = []

    def _p(self, text):
        if self.echo:
            print(text, flush=True)

    def run(self, cmd):
        self.log.append(("run", cmd))
        self._p(f"  + {cmd}")
        self.built = self.built or "cargo build" in cmd
        return self.sim.handle(cmd)

    def exists(self, path):
        return self.built or self._exists(path)

    def spawn(self, argv, tag):
        line = " ".join(shlex.quote(a) for a in argv)
        self.log.append(("spawn", line))
        self._p(f"  + {line} &   ({tag})")
        return self.sim.spawn(argv, tag)

    def connect(self, host, port):
        self.log.append(("connect", f"{host}:{port}"))
        self._p(f"  + (TCL connect {host}:{port})")
        return FakeReader(self.sim)

    def port_open(self, host, port):
        return True

    def sleep(self, s):
        self.sim.advance(s)

    def now(self):
        return self.sim.t

    def ask(self, prompt):
        a = self.answers(prompt)
        self.log.append(("ask", prompt))
        self._p(f"\n>> {prompt} [dry run answers: {a!r}]")
        return a

    def say(self, text):
        self.log.append(("say", text))
        self._p(text)

    def defer_signals(self):
        self.log.append(("signals", "deferred"))
        return contextlib.nullcontext()


def nominal_answers(prompt):
    """The operator a dry run assumes: confirms, sets the PSU by the rule, reads plausible meters."""
    if prompt.startswith("Set the PSU"):
        # What this bench is actually set to, not the minimum the prompt allows: the operator types
        # what they set, and a reading above the DECLARED limit is an abort.
        m = re.search(r"at ([0-9.]+) A", prompt)
        return f"{max(3.5, float(m.group(1))):g}" if m else "3.5"
    if "board's OWN draw" in prompt:
        return f"{SIM_QUIESCENT_A:.2f}"                        # armed, nothing commanded
    m = re.search(r"PSU reading \(A\) at demand (\d+)", prompt)
    if m:
        # What the PSU shows: the bridge's DC-link mean PLUS the board's own draw.
        return f"{sim_locked_rotor(int(m.group(1)))[2] + SIM_QUIESCENT_A:.2f}"
    if "PSU reading now" in prompt and "Small" in prompt:
        return f"{sim_locked_rotor(sim_ladder_end())[2] + SIM_QUIESCENT_A:.2f}"
    if "PSU reading now" in prompt and "gate 4" in prompt:
        return "2.4"
    if "PSU reading now" in prompt:
        return "3.2"
    if "Clamp-meter" in prompt:
        return ""
    if "Stay armed" in prompt:
        return "y"
    if "(y/N)" in prompt:
        return "y"
    return ""


# --------------------------------------------------------------------------------------------------
# The session.
# --------------------------------------------------------------------------------------------------
TEARDOWN_ORDER = ("drive_end", "hold_end", "moe_check", "neutral", "rail_off", "rail_verify", "ocd_kill",
                  "tunnel_close", "lock_release")


def TEARDOWN_STEPS_OF(*fns):  # noqa: N802 - pairs the canonical order with the step functions
    return list(zip(TEARDOWN_ORDER, fns))


class Session:
    def __init__(self, args, shell, csv_fh, record_dir, today):
        self.a = args
        self.sh = shell
        self.csv = EvidenceCsv(csv_fh)
        self.record_dir = record_dir
        self.owner = os.environ.get("BENCH_OWNER", "claude-climit")
        self.bin_dir = os.path.join(REPO, "crates", "swd-bridge", "target", host_target(), "release")
        self.reader = None
        self.addrs = None
        self.inputs = None            # the one session owner (arm and demand), or None
        self.owner_buttons = None     # the levels it holds, so a step needing others ends it first
        self.drive = None
        self.node = None
        self.g1 = None
        self.psu_quiescent = 0.0      # the board's own link current, read armed and undemanded
        self.restore_wanted = None    # a stored limit too low for the ladder, offered for a raise
        self.trips_base = 0           # trips the session has already explained (gate 4's own chop)
        self.phase_gate2 = False
        self.before_gate5 = True
        self.took_lock = False
        self.rail_touched = False
        self.ocd_started = False
        self.tunnel = None
        self.torn_down = False
        self.closed = False
        self.teardown_log = []
        self.children_seen = {}
        self.rec = {
            "date": today, "params": {
                "board": args.board, "limit_ma": args.limit_ma, "cal_demand": args.cal_demand,
                "trip_demand": args.trip_demand, "skip": sorted(set(args.skip_gate or []))},
            "elf": args.elf, "head": "not read", "csv": "", "outcome": "not finished",
            "gates": [], "typed": [], "teardown": [], "final": [], "calibration": None,
            "psu_declared": None, "prev_limit_ma": None,
            # The board's counts per amp (0x67), read at stand-up: `staged_cpa` is what the
            # firmware converts against (the boot-seam clamp applied), `cal_raw` what is stored.
            "staged_cpa": CURRENT_CAL_DEFAULT, "cal_raw": None,
            "rotor": ("braking fallback (--brake-fallback): the operator braked a spinning wheel"
                      if args.brake_fallback else "locked (strap, or both hands on the tyre)"),
        }
        self.locked = not args.brake_fallback
        # Until stand-up reads 0x67 off the board, the registered default is all a report can say;
        # every gate runs after that read.
        self.staged_cpa = CURRENT_CAL_DEFAULT
        self.cal_raw = None
        self.cal = None
        self.cal_final_demand = None
        self.armed_expected = False   # every sample must then show RUN + MOE with the hold alive
        self.limit_written = False
        self.rec["warnings"] = []

    # ---- helpers ------------------------------------------------------------------------------
    def say(self, text):
        self.sh.say(text)

    def heading(self, text):
        self.say(f"\n== {text} ==")
        self.csv.comment(f"step {text}")

    def ask(self, prompt):
        a = self.sh.ask(prompt)
        self.rec["typed"].append((prompt, a))
        self.csv.comment(f"typed {prompt!r} -> {a!r}")
        if a.strip().lower() == "abort":
            raise SessionAbort("the operator typed abort")
        return a

    def ask_yes(self, prompt):
        return self.ask(prompt + " (y/N)").strip().lower() in ("y", "yes")

    def ask_float(self, prompt, allow_empty=False):
        for _ in range(3):
            a = self.ask(prompt)
            if allow_empty and not a.strip():
                return None
            try:
                return float(a)
            except ValueError:
                self.say(f"   '{a}' is not a number of amps; type it again.")
        raise SessionAbort(f"no usable number typed for: {prompt}")

    def bin(self, name):
        return os.path.join(self.bin_dir, name)

    def drain_children(self):
        for c in (self.inputs, self.drive, self.tunnel):
            if c is None:
                continue
            seen = self.children_seen.get(id(c), 0)
            for line in c.lines[seen:]:
                self.csv.comment(f"child {c.tag}: {line}")
            self.children_seen[id(c)] = len(c.lines)

    def check_node(self, text, who):
        got = parse_dst(text)
        if got is None:
            raise SessionAbort(f"{who} printed no 'dst resolved' line")
        if self.node is not None and got != self.node:
            raise SessionAbort(f"{who} resolved attached node 0x{got:02x}, not the confirmed 0x{self.node:02x}")
        self.say(f"   {who}: attached node 0x{got:02x}")
        return got

    def wait_dst(self, child, who):
        t0 = self.sh.now()
        while self.sh.now() - t0 < DST_TIMEOUT_S:
            text = "\n".join(child.lines)
            if parse_dst(text) is not None:
                self.drain_children()
                try:
                    return self.check_node(text, who)
                except SessionAbort:
                    child.end()          # the inputs tool prints dst before its first send: stop it now
                    raise
            if not child.alive():
                break
            self.sh.sleep(0.2)
        self.drain_children()
        raise SessionAbort(f"{who} did not resolve the attached node: {' | '.join(child.lines[-3:])}")

    # ---- sampling -----------------------------------------------------------------------------
    def read_sample(self, label):
        ctrl = self.addrs["CTRL_OBS"]
        base, span, off = swdobs.motor_block(self.addrs)
        try:
            for _ in range(5):
                a = self.reader.read_words(ctrl, CTRL_OBS_WORDS)
                b = self.reader.read_words(ctrl, CTRL_OBS_WORDS)
                if tear_ok(a, b):
                    break
            else:
                raise SessionAbort("CTRL_OBS reads never agreed (tear guard, 5 attempts)")
            m = self.reader.read_words(base, span)
        except (RuntimeError, OSError) as e:
            raise SessionAbort(f"SWD read failed: {e}")
        return decode_sample(b, m, off, self.sh.now(), label)

    def sample(self, label, seconds, until=None, fast=False):
        """Sample at SAMPLE_HZ (or as fast as the probe answers) for `seconds`, or until
        `until(sample)`; every sample goes to the CSV and through the abort conditions, and while
        the session holds an arm, through the arm check."""
        out = []
        t0 = self.sh.now()
        period = 1.0 / SAMPLE_HZ
        while True:
            t_loop = self.sh.now()
            s = self.read_sample(label)
            out.append(s)
            self.csv.row(s)
            self.drain_children()
            reason = abort_reason(s, gate2=self.phase_gate2, before_gate5=self.before_gate5,
                                  trips_base=self.trips_base)
            if reason:
                raise SessionAbort(reason)
            self.check_arm(s)
            if until is not None and until(s):
                break
            if self.sh.now() - t0 >= seconds:
                break
            if not fast:
                slack = period - (self.sh.now() - t_loop)
                if slack > 0:
                    self.sh.sleep(slack)
        return out

    def check_arm(self, s=None):
        """While an arm is expected: the inputs hold alive, mode_byte RUN, moe_bits set."""
        if not self.armed_expected:
            return
        if self.inputs is None or not self.inputs.alive():
            raise SessionAbort(f"{ARM_EXPIRED}: the inputs hold is no longer running")
        if s is not None and (s["mode"] != MODE_RUN or not s["moe"]):
            raise SessionAbort(f"{ARM_EXPIRED}: mode_byte {MODE_NAMES.get(s['mode'], s['mode'])}, "
                               f"moe_bits 0x{s['moe']:02x}")

    def require_armed(self, label):
        """Before an energised step: one sample through the arm check."""
        s = self.read_sample(f"{label}-armcheck")
        self.csv.row(s)
        self.check_arm(s)

    def window(self, label, seconds, **kw):
        self.csv.comment(f"step {label} begin t={self.sh.now():.3f} window={seconds:.1f}s")
        s = self.sample(label, seconds, **kw)
        return s

    def end_step(self, label, verdict):
        self.csv.comment(f"step {label} end t={self.sh.now():.3f} verdict={verdict}")

    def add_gate(self, name, result):
        self.rec["gates"].append({"name": name, "verdict": result["verdict"], "lines": result["lines"]})
        self.say(f"   {name}: {result['verdict']}")
        for line in result["lines"]:
            self.say(f"     {line}")

    # ---- the mailbox tools ----------------------------------------------------------------------
    def mailbox_args(self, *rest):
        return [ENDPOINT, "--dst", "attached"] + [str(r) for r in rest]

    def run_tool(self, name, *rest):
        cmd = " ".join(shlex.quote(a) for a in [self.bin(name)] + list(rest))
        r = self.sh.run(cmd)
        self.csv.comment(f"tool {name} rc={r.returncode}")
        for line in r.stdout.splitlines():
            self.csv.comment(f"tool {name}: {line}")
        return r

    def read_cal(self):
        """The board's counts per amp (`motor.current_cal`, 0x67, motor 0), read off the board and
        not carried as a tool constant: every amp this session reports is `counts / this`
        (specs/motor-integration.md, "The current-sense calibration"). An unreadable 0x67 is an
        abort, because a session that reports amps against a guess is worse than no session."""
        try:
            raw, eff = read_current_cal(self.sh.run, ENDPOINT, config_bin=self.bin("swd-mailbox-config"))
        except RuntimeError as e:
            raise SessionAbort(str(e)) from e
        self.cal_raw, self.staged_cpa = raw, eff
        self.rec["cal_raw"], self.rec["staged_cpa"] = raw, eff
        clamped = "" if raw == eff else f" (the boot seam clamps it to {eff})"
        self.say(f"   counts per amp now (0x67): {raw}{clamped}; every amp below is counts / {eff}")
        self.csv.comment(f"motor.current_cal (0x67) = {raw} counts per amp, effective {eff}")
        return raw, eff

    def config_read_limit(self):
        r = self.run_tool("swd-mailbox-config", *self.mailbox_args(f"0x{LIMIT_FIELD:02x}"))
        node = self.check_node(r.stdout, "swd-mailbox-config")
        status, value = parse_config_read(r.stdout, LIMIT_FIELD)
        if status != "CFG_OK" or value is None:
            raise SessionAbort(f"reading 0x20 failed (status {status}): {r.stdout.strip()[-200:]}")
        return node, value

    def start_owner(self, buttons):
        """Start the ONE session owner, holding `buttons` (1 = power_request, so armed) and, once a
        demand is set, the demand too, from a SINGLE mailbox attach. A second attaching process
        would flush this one's inbound ring and expire its arm, which aborted two gates on
        2026-10-08 (`specs/swd-mailbox.md`, "Attach + session flush").

        An owner already running with different levels is ended first: gate 2 wants a demand with no
        arm, the rest want an arm, and one process can only hold one set."""
        if self.inputs is not None and self.owner_buttons != buttons:
            self.end_inputs()
        if self.inputs is None:
            self.inputs = self.sh.spawn(
                [self.bin("swd-mailbox-session")] + self.mailbox_args(
                    "--buttons", buttons, "--rider", buttons, "--hold", INPUTS_HOLD_S),
                f"session owner (buttons {buttons}, arm {'held' if buttons else 'NOT asserted'})")
            self.owner_buttons = buttons
            self.wait_dst(self.inputs, "swd-mailbox-session")
            self.drive = None
        return self.sh.now()

    def start_inputs(self):
        return self.start_owner(1)

    def end_inputs(self):
        if self.inputs is not None:
            self.say("  + (end the session: it sends Neutral, then the all-clear)")
            self.inputs.end()
            self.drain_children()
            self.inputs, self.drive, self.owner_buttons = None, None, None

    def start_drive(self, value):
        """Set the demand on the session owner that already holds the arm. No per-demand hold goes
        with it: the owner holds a demand until it is changed, and the bound is `DEMAND_MAX_HELD_S`,
        checked by `ensure_drive`, which is what replaced the drive tool's 60 s cap."""
        if self.inputs is not None and not self.inputs.alive():
            raise SessionAbort(f"{ARM_EXPIRED}: the session owner stopped")
        # No owner yet means an unarmed step (gate 2): the demand needs a producer, not an arm.
        self.start_owner(self.owner_buttons if self.inputs is not None else 0)
        self.say(f"  > demand {value} (to the session owner, arm "
                 f"{'held' if self.owner_buttons else 'NOT asserted'})")
        self.csv.comment(f"demand {value}")
        self.inputs.send(f"value {value}")
        self.drive = self.inputs
        self.sh.sleep(DEMAND_ACK_S)
        self.drive_t0 = self.sh.now()
        return self.drive_t0

    def ensure_drive(self, window_s):
        """There is nothing to renew: one owner holds the demand for as long as it holds the arm, so
        no re-attach dips the demand mid-gate. What remains is the checking the renewal used to
        carry: the arm is live, and no single demand runs longer than `DEMAND_MAX_HELD_S`."""
        if self.drive is None:
            raise SessionAbort("ensure_drive called with no demand set")
        self.check_arm()
        held = self.sh.now() - self.drive_t0
        if held + window_s > DEMAND_MAX_HELD_S:
            raise SessionAbort(f"one unchanged demand would run {held + window_s:.0f} s, over the "
                               f"{DEMAND_MAX_HELD_S} s bound: end the gate and re-apply deliberately")

    def stop_drive(self):
        if self.drive is not None:
            self.say("  > neutral (the owner releases the demand; the firmware decays it in 200 ms)")
            self.csv.comment("demand neutral")
            self.drive.send("neutral")
            self.sh.sleep(DEMAND_ACK_S)
            self.drive = None

    def neutral(self):
        """One explicit Neutral. Through the session owner while it is running, because a one-shot
        tool would ATTACH, flushing the owner's ring and expiring the arm. The standalone tool is
        for after the owner is gone (the teardown's own Neutral)."""
        if self.inputs is not None and self.inputs.alive():
            self.inputs.send("neutral")
            self.sh.sleep(DEMAND_ACK_S)
            node = self.node if self.node is not None else 0
            return Result(0, f"dst resolved: attached node 0x{node:02x} (the session owner)")
        return self.run_tool("swd-mailbox-drive", *self.mailbox_args("--value", 0, "--hold", 1))

    def release_demand(self):
        self.stop_drive()
        self.check_node(self.neutral().stdout, "swd-mailbox-drive (Neutral)")

    # ---- bench plumbing -----------------------------------------------------------------------
    def rail(self, on):
        self.rail_touched = True
        self.sh.run(f"ssh {PI} 'pinctrl set 4 op {'dl' if on else 'dh'}'")

    def start_ocd(self):
        self.ocd_started = True
        try:
            swdobs.start_remote_ocd(self.a.board, who=TOOL, log=OCD_LOG, bind_all=False,
                                    run=self.sh.run, sleep=self.sh.sleep)
        except SystemExit as e:
            raise SessionAbort(str(e))

    def connect(self):
        if self.reader is not None:
            self.reader.close()
        try:
            self.reader = self.sh.connect("127.0.0.1", TCL_PORT)
        except OSError as e:
            raise SessionAbort(f"cannot connect to the tunnelled TCL port: {e}")
        s = self.read_sample("magic")
        if s["magic"] != CTRL_MAGIC:
            raise SessionAbort(f"CTRL_OBS magic reads 0x{s['magic']:08x}, expected 0x{CTRL_MAGIC:08x}: "
                               "the flashed image is not the one at --elf")

    def disarmed_read(self, label):
        s = self.window(label, 1.0)
        problems = standup_problems(s[-1])
        if problems:
            raise SessionAbort("disarmed state not ready: " + "; ".join(problems))
        self.say(f"   disarmed: mode OFF, moe 0, motor flags 0x{s[-1]['mflags']:02x}, fault 0, "
                 f"throttle mode, boot {s[-1]['boot']}")
        # A refusal from EARLIER THIS BOOT, which the disarmed state itself cannot show: the
        # re-read refusal has already cleared on an OFF pass by now, so the counts are what is left
        # of it. Not a stand-up problem (the board is armable again); said so it is not a surprise
        # when the arm below is refused for the same stored value.
        if s[-1]["reread_refusals"] or s[-1]["confirm_refusals"]:
            self.say(f"   {refusal_text(s[-1])} (earlier this boot)")
        self.end_step(label, "OK")

    def resolve_symbols(self):
        if self.sh.dry:
            self.say(f"  + arm-none-eabi-nm -S {self.a.elf}   (dry run: canned symbol table)")
            syms = SIM_SYMS
        else:
            syms = swdobs.resolve_symbols_sized(self.a.elf, who=TOOL)
        size = syms["CTRL_OBS"][1]
        if size != 4 * CTRL_OBS_WORDS:
            raise SessionAbort(f"CTRL_OBS is {size} B in the ELF, this tool decodes {4 * CTRL_OBS_WORDS} B "
                               f"({CTRL_OBS_WORDS} words): the ELF and the tool disagree")
        self.addrs = {k: v[0] for k, v in syms.items()}
        self.say(f"   CTRL_OBS at 0x{self.addrs['CTRL_OBS']:08x}, {size} B")

    # ---- the flow -----------------------------------------------------------------------------
    def stand_up(self):
        self.heading("0. stand up")
        self.resolve_symbols()
        r = self.sh.run(f"git -C {shlex.quote(REPO)} rev-parse HEAD")
        self.rec["head"] = r.stdout.strip() or "unknown"
        if not all(self.sh.exists(self.bin(b)) for b in BINS):
            manifest = shlex.quote(os.path.join(REPO, "crates/swd-bridge/Cargo.toml"))
            self.sh.run(f"cargo build --release --manifest-path {manifest} --target {host_target()}")
            if not all(self.sh.exists(self.bin(b)) for b in BINS):
                raise SessionAbort(f"the mailbox tools are missing from {self.bin_dir} after the build")
        r = self.sh.run(f"{shlex.quote(os.path.join(REPO, 'tools', 'bench-lock.sh'))} acquire {self.owner} "
                        "'climit-session: current-limit gates'")
        if "ACQUIRED" in r.stdout:
            self.took_lock = True
        elif "ALREADY-OURS" not in r.stdout:
            raise SessionAbort(f"bench busy: {r.stdout.strip()}")

        target = psu_target_a(self.a.limit_ma)
        declared = self.ask_float(f"Set the PSU to 25 V with its current limit at {target:g} A or above "
                                  f"(up to {PSU_CAP_A:g} A; lower is safer, it bounds a fault), output on. "
                                  "Type the current limit you set (A):")
        if not psu_declared_ok(self.a.limit_ma, declared):
            raise SessionAbort(f"PSU limit {declared:g} A is outside the rule: want {target:g} A (the staged "
                               f"at least {psu_target_a(self.a.limit_ma):g} A, never above {PSU_CAP_A:g} A)")
        self.rec["psu_declared"] = declared
        self.psu_declared = declared

        self.say("   rail off, then on (once)")
        self.rail(False)
        self.sh.sleep(RELAY_SEAT_OFF_S)
        if not self.ask_yes("Relay is off. Does the PSU read about 0 A?"):
            raise SessionAbort("the relay contact has not seated (PSU current with the relay off)")
        self.rail(True)
        self.sh.sleep(BOOT_SETTLE_S)

        self.start_ocd()
        self.tunnel = self.sh.spawn(["ssh", "-N", "-o", "ExitOnForwardFailure=yes", "-L",
                                     f"{TCL_PORT}:localhost:{TCL_PORT}", PI], "tunnel")
        for _ in range(20):
            if self.sh.port_open("127.0.0.1", TCL_PORT):
                break
            self.sh.sleep(0.3)
        else:
            raise SessionAbort(f"the tunnel to the Pi's TCL port never came up on 127.0.0.1:{TCL_PORT}")
        self.connect()
        self.disarmed_read("stand-up disarmed read")

        node, prev = self.config_read_limit()
        self.rec["prev_limit_ma"] = prev
        self.say(f"   staged limit now (0x20): {prev} mA (gate 3 runs at this limit)")
        self.read_cal()
        if self.locked and 3 not in (self.a.skip_gate or []) and prev < LADDER_EST_HI_A * 1000:
            w = (f"the staged limit {prev} mA is below the {LADDER_EST_HI_A:g} A the calibration ladder may "
                 "reach: a step can chop, which ends the calibration as INVALID")
            self.rec["warnings"].append(w)
            self.say(f"   WARNING: {w}")
            self.restore_wanted = prev
        if not self.ask_yes(f"The attached node is 0x{node:02x}. Is that the board with the motor in front of you?"):
            raise SessionAbort(f"the operator did not confirm attached node 0x{node:02x}")
        self.node = node
        if self.restore_wanted is not None and self.ask_yes(
                f"0x20 holds {self.restore_wanted} mA, left by an earlier session, and it will chop the "
                f"calibration. Raise it to {CAL_LIMIT_MA} mA for gate 3 (write plus a rail power-cycle)?"):
            self.stage_limit(CAL_LIMIT_MA, name="Restore the limit for gate 3")
        self.rec["node_txt"] = f"0x{node:02x} (operator confirmed)"

    def gate1(self):
        self.heading("gate 1, rest floor (disarmed)")
        s = self.window("gate1", G1_S)
        r = gate1_verdict(s, self.staged_cpa)
        self.g1 = r
        self.end_step("gate1", r["verdict"])
        self.add_gate("Gate 1, rest floor", r)
        if r["verdict"] == "FAIL":
            raise SessionEnd("gate 1 failed: the board is not quiet at rest; nothing is armed after that")

    def gate2(self):
        self.heading("gate 2, demand without arm (disarmed)")
        self.say(f"   holding demand {self.a.cal_demand} with no power request; nothing should flow")
        self.start_drive(self.a.cal_demand)
        self.phase_gate2 = True
        try:
            s = self.window("gate2", G2_S)
        finally:
            self.phase_gate2 = False
        self.release_demand()
        r = gate2_verdict(s, floor_max(self.g1))
        self.end_step("gate2", r["verdict"])
        self.add_gate("Gate 2, demand without arm", r)
        if r["verdict"] == "FAIL":
            raise SessionEnd("gate 2 failed: demand without an arm reached the bridge; nothing is armed after that")

    def arm(self, label, prompt):
        self.ask(prompt)
        if HANDS_ON in prompt:
            self.countdown()
        t0 = self.start_inputs()
        s = self.window(f"{label}-arm", ARM_WITHIN_S, until=lambda x: x["mode"] == MODE_RUN and x["moe"])
        ok, detail = arm_ok(s, t0)
        self.say(f"   arm: {detail}")
        if not ok:
            raise SessionEnd(f"{label}: the arm was refused or slow ({detail})")
        self.armed_expected = True
        for attempt in range(RELOCK_MAX + 1):
            soak = self.window(f"{label}-soak", SOAK_S)
            reason = soak_abort(soak, floor_max(self.g1))
            if reason is None:
                break
            if not reason.startswith(ROTOR_MOVED) or attempt == RELOCK_MAX:
                raise SessionAbort(reason)
            self.say(f"   {reason}")
            self.csv.comment(f"{label}-soak relock {attempt + 1}")
            self.ask(RELOCK_PROMPT)
            self.countdown()
        self.say(f"   armed and still for {SOAK_S:.0f} s, peak max {max(x['peak'] for x in soak)}")
        self.end_step(f"{label}-soak", "OK")

    def spin(self, label):
        self.require_armed(label)
        t0 = self.start_drive(self.a.cal_demand)
        s = self.window(f"{label}-spin", SPIN_WITHIN_S + SPIN_STEADY_S)
        ok, detail = spin_ok(s, t0)
        self.say(f"   spin: {detail}")
        return ok, detail

    def release_and_confirm(self, label):
        self.say("   releasing the demand (Neutral)." + ("" if self.locked else " Ease off the tyre."))
        self.release_demand()
        s = self.window(f"{label}-release", STOP_WITHIN_S,
                        until=lambda x: x["speed"] == 0 and x["demand"] == 0 and x["duty_on"] == 0)
        if not stopped_ok(s):
            last = s[-1] if s else {}
            raise SessionAbort(f"{label}: the demand did not release within {STOP_WITHIN_S:.0f} s of the Neutral "
                               f"(motor_speed {last.get('speed')}, demand {last.get('demand')}, "
                               f"torque {last.get('torque')}, duty_on {last.get('duty_on')})")
        self.say(f"   released: motor_speed 0, demand 0, duty 0 (sub_state {s[-1]['sub']}; it returns to "
                 "0 on the disarm or a trip, not on a Neutral)")

    def disarm_and_confirm(self, label):
        self.armed_expected = False
        self.end_inputs()
        s = self.window(f"{label}-disarm", MOE_CHECK_S, until=lambda x: x["mode"] == MODE_OFF and x["moe"] == 0)
        if not disarmed_ok(s):
            raise SessionAbort(f"{label}: moe_bits/mode_byte not OFF within {MOE_CHECK_S:.0f} s of the hold ending")

    def gate3(self):
        if self.locked:
            return self.gate3_locked()
        self.heading("gate 3, calibration (owner, braking fallback)")
        self.arm("gate3", FREE_PROMPT)
        ok, detail = self.spin("gate3")
        if not ok:
            self.rec["gates"].append({"name": "Gate 3, calibration", "verdict": "FAIL", "lines": [detail]})
            self.release_and_confirm("gate3")
            self.disarm_and_confirm("gate3")
            raise SessionEnd(f"gate 3 did not spin: {detail}")
        prompt = ("Hand on the kill. Brake the tyre by hand (gloves) until the PSU reads at least 3 A and hold "
                  "it steady. Press Enter when steady.")
        psu = None
        while True:
            if self.ask(prompt).strip().lower() == "skip" and psu is not None:
                break
            self.ensure_drive(CAL_WINDOW_S)
            s = self.window("gate3-cal", CAL_WINDOW_S)
            self.say(f"   window: mean peak {_mean(x['peak'] for x in s):.0f} counts, mean duty_on "
                     f"{_mean(x['duty_on'] for x in s):.0f} of {PWM_PERIOD}")
            psu = self.ask_float("PSU reading now (A)?")
            reason = psu_reading_abort(psu, self.psu_declared)
            if reason:
                raise SessionAbort(reason)
            if psu >= CAL_MIN_PSU_A:
                break
            prompt = ("Hand on the kill. The PSU read under 3 A. Brake harder and hold it steady. "
                      "Press Enter when steady, or type skip to end the calibration.")
        clamp = self.ask_float("Clamp-meter phase reading (A), or Enter for none:", allow_empty=True)
        # No ladder in the braking fallback: one braked point cannot separate a slope from an
        # offset, so the verdict it reaches is INCONCLUSIVE (an upper bound) by construction.
        cal = calibration(s, psu, clamp, self.staged_cpa,
                          floor_counts=floor_max(self.g1), ladder_peaks=())
        if psu < CAL_MIN_PSU_A:
            cal["verdict"] = "INVALID"
            cal["recommendation"] = f"the braked PSU reading {psu:g} A stayed under {CAL_MIN_PSU_A:g} A"
            cal["lines"][-1] = cal["recommendation"]
        self.rec["calibration"] = cal
        self.cal = cal
        self.cal_final_demand = self.a.cal_demand
        self.end_step("gate3-cal", cal["verdict"])
        self.add_gate("Gate 3, calibration", {"verdict": cal["verdict"], "lines": cal["lines"]})
        self.release_and_confirm("gate3")
        self.disarm_and_confirm("gate3")

    def gate3_locked(self):
        """The locked-rotor calibration: the tool steps the demand until the duty-corrected phase
        estimate is in the 4..8 A band, and stops on the first chopped window."""
        self.heading("gate 3, calibration (owner, rotor locked)")
        self.arm("gate3", LOCK_PROMPT)
        self.read_quiescent()
        demand, ladder, relocks, backed_off = self.a.cal_demand, [], 0, False
        chopped_at = None
        while True:
            self.require_armed(f"gate3-step-{demand}")
            self.start_drive(demand)
            self.window(f"gate3-settle-{demand}", SETTLE_S)
            s = self.window(f"gate3-step-{demand}", LADDER_STEP_S)
            moving = rotor_moved(s)
            if moving:
                relocks += 1
                self.release_and_confirm("gate3")
                if relocks > RELOCK_MAX:
                    raise SessionAbort(f"motor_speed kept reading nonzero ({moving}): the rotor is not locked")
                self.ask(f"Hand on the kill. motor_speed read {moving}: the rotor is not locked. Press Enter, "
                         f"then lock it and keep it locked: {HANDS_ON}.")
                self.countdown()
                continue
            if not ladder and not any(x["sub"] != SUB_IDLE for x in s):
                detail = (f"sub_state stayed 0 at demand {demand}: the demand is not reaching the control task "
                          "(read the D4 notes in specs/arm-session.md)")
                self.rec["gates"].append({"name": "Gate 3, calibration", "verdict": "FAIL", "lines": [detail]})
                self.release_and_confirm("gate3")
                self.disarm_and_confirm("gate3")
                raise SessionEnd(f"gate 3: {detail}")
            if any(x["chopped"] for x in s):
                chopped_at = demand          # the staged limit is the peak's author now: stop the ladder
                break
            psu = self.ask_float(f"PSU reading (A) at demand {demand}? It is small at low duty "
                                 "(about 1 A); type what it shows.")
            reason = psu_reading_abort(psu, self.psu_declared)
            if reason:
                raise SessionAbort(reason)
            duty = _mean(x["duty_on"] for x in s)
            est = phase_estimate(max(psu - self.psu_quiescent, 0.0), duty)
            ladder.append((demand, psu, _mean(x["peak"] for x in s), duty, est))
            est_txt = "n/a" if est is None else f"{est:.2f} A"
            self.say(f"   demand {demand}: duty_on {duty:.0f}, PSU {psu:g} A less the board's "
                     f"{self.psu_quiescent:g} A, phase estimate {est_txt} "
                     f"(want {LADDER_EST_LO_A:g}..{LADDER_EST_HI_A:g} A)")
            move, nxt = ladder_next(demand, est, backed_off)
            if move == "done":
                break
            if move == "abort":
                raise SessionAbort(nxt)
            backed_off = backed_off or move == "back"
            self.stop_drive()
            demand = nxt
        if chopped_at is not None:
            return self.gate3_chopped(demand, ladder, s)
        self.ensure_drive(CAL_WINDOW_S)
        s = self.window("gate3-cal", CAL_WINDOW_S)
        self.say(f"   window at demand {demand}: mean peak {_mean(x['peak'] for x in s):.0f} counts, mean duty_on "
                 f"{_mean(x['duty_on'] for x in s):.0f} of {PWM_PERIOD}")
        psu = self.ask_float("PSU reading now (A)? Small, about 1 A; type what it shows.")
        reason = psu_reading_abort(psu, self.psu_declared)
        if reason:
            raise SessionAbort(reason)
        clamp = self.ask_float("Clamp-meter phase reading (A), or Enter for none:", allow_empty=True)
        cal = calibration(s, psu, clamp, self.staged_cpa, self.psu_quiescent,
                          floor_counts=floor_max(self.g1),
                          ladder_peaks=[pk for _d, _psu, pk, _duty, _est in ladder])
        if rotor_moved(s):
            cal["verdict"] = "INVALID"
            cal["recommendation"] = "motor_speed was nonzero in the calibration window: the rotor was not locked"
            cal["lines"][-1] = cal["recommendation"]
        cal["lines"] = [ladder_line(ladder)] + cal["lines"]
        self.rec["calibration"] = cal
        self.cal = cal
        self.cal_final_demand = demand
        self.end_step("gate3-cal", cal["verdict"])
        self.add_gate("Gate 3, calibration", {"verdict": cal["verdict"], "lines": cal["lines"]})
        self.release_and_confirm("gate3")
        self.disarm_and_confirm("gate3")
        self.ask("You can release the rotor. Press Enter.")

    def countdown(self, secs=None):
        """The operator's hands-on time after a prompt they answer one-handed. Pressing Enter and
        holding a rotor still are not doable at once, so every lock prompt is followed by this
        rather than by an immediate measurement."""
        secs = LOCK_COUNTDOWN_S if secs is None else secs
        for left in range(int(secs), 0, -1):
            self.say(f"   {left}...")
            self.sh.sleep(1.0)
        self.say("   measuring")

    def read_quiescent(self):
        """The board's own supply current: armed, rotor locked, nothing commanded. Subtracted from
        every later PSU reading before the duty correction multiplies it up."""
        q = self.ask_float("PSU reading now (A), armed with no demand? This is the board's OWN draw "
                           "(logic, LEDs, BLE); Enter for 0 if the display is not readable.",
                           allow_empty=True)
        self.psu_quiescent = 0.0 if q is None else q
        self.csv.comment(f"quiescent {self.psu_quiescent} A")
        self.rec["quiescent_a"] = self.psu_quiescent
        self.say(f"   board's own draw {self.psu_quiescent:g} A, subtracted from every PSU reading below")

    def gate3_chopped(self, demand, ladder, s):
        """A ladder window the staged limit chopped: the calibration is INVALID and the ladder stops."""
        rec = (f"the window at demand {demand} was chopped (max {max(x['chopped'] for x in s)}): the peak is the "
               "staged limit's, not the load's. Re-run with the limit at its default.")
        cal = {"verdict": "INVALID", "recommendation": rec,
               "lines": [ladder_line(ladder) if ladder else "demand ladder: no reading before the chop",
                         f"mean peak {_mean(x['peak'] for x in s):.0f} counts, mean duty_on "
                         f"{_mean(x['duty_on'] for x in s):.0f}, chopped in {sum(1 for x in s if x['chopped'])}"
                         f"/{len(s)} samples", rec]}
        self.rec["calibration"] = cal
        self.cal_final_demand = demand
        self.end_step("gate3-cal", "INVALID")
        self.add_gate("Gate 3, calibration", {"verdict": "INVALID", "lines": cal["lines"]})
        self.release_and_confirm("gate3")
        self.disarm_and_confirm("gate3")
        self.ask("You can release the rotor. Press Enter.")

    def stage_limit(self, ma=None, name="Stage the limit"):
        ma = self.a.limit_ma if ma is None else ma
        lc = limit_counts(ma, self.staged_cpa)
        self.heading(f"{name}: {ma} mA = {lc} counts, hard trip {hard_trip_counts(lc)} counts")
        r = self.run_tool("swd-mailbox-config", *self.mailbox_args(f"0x{LIMIT_FIELD:02x}={ma}"))
        self.check_node(r.stdout, "swd-mailbox-config")
        if not config_write_ok(r.stdout, LIMIT_FIELD):
            raise SessionAbort(f"staging 0x20={ma} failed: {r.stdout.strip()[-200:]}")
        self.limit_written = True
        self.say("   power-cycling the rail so the boot reads the new limit")
        if self.reader is not None:
            self.reader.close()
            self.reader = None
        self.sh.run(swdobs.stop_remote_ocd_cmd())
        self.rail(False)
        self.sh.sleep(3.0)
        self.rail(True)
        self.sh.sleep(BOOT_SETTLE_S)
        self.start_ocd()
        self.connect()
        self.disarmed_read("post-stage disarmed read")
        _, back = self.config_read_limit()
        if back != ma:
            raise SessionAbort(f"0x20 reads back {back} after the power cycle, not {ma}")
        self.say(f"   0x20 reads back {back} mA after the power cycle")
        self.rec["gates"].append({"name": name, "verdict": "DONE", "lines": [
            f"0x20 = {ma} mA written, read back, power-cycled, read back {back} mA",
            f"limit {lc} counts ({ma} mA * {self.staged_cpa} / 1000, floored at {MIN_LIMIT_COUNTS}), "
            f"hard trip {hard_trip_counts(lc)} counts (2x)",
        ]})

    def gate4(self, stay_for_gate5):
        if self.locked:
            return self.gate4_locked(stay_for_gate5)
        self.heading("gate 4, the plateau (owner, braking fallback)")
        self.arm("gate4", FREE_PROMPT)
        ok, detail = self.spin("gate4")
        if not ok:
            self.rec["gates"].append({"name": "Gate 4, the plateau", "verdict": "FAIL", "lines": [detail]})
            self.release_and_confirm("gate4")
            self.disarm_and_confirm("gate4")
            raise SessionEnd(f"gate 4 did not spin: {detail}")
        self.ask("Hand on the kill. Brake harder than in gate 3, past the limit, and hold. Press Enter when "
                 "the PSU reading stops rising.")
        self.ensure_drive(G4_WINDOW_S)
        s = self.window("gate4", G4_WINDOW_S)
        psu = self.ask_float("PSU reading now (A), gate 4?")
        reason = psu_reading_abort(psu, self.psu_declared)
        if reason:
            raise SessionAbort(reason)
        r = gate4_verdict(s, self.a.limit_ma, self.staged_cpa, psu, locked=False)
        self.end_step("gate4", r["verdict"])
        self.add_gate("Gate 4, the plateau", r)
        self.adopt_trips("gate4")
        self.release_and_confirm("gate4")
        if stay_for_gate5 and self.ask_yes("Stay armed for gate 5?"):
            return True
        self.disarm_and_confirm("gate4")
        return False

    def gate4_locked(self, stay_for_gate5):
        self.heading("gate 4, the plateau (owner, rotor locked)")
        if self.cal_final_demand is not None:
            base, why = self.cal_final_demand, "the gate-3 final demand"
        else:
            base, why = self.a.cal_demand, "--cal-demand (gate 3 did not run this session)"
        demand = min(base + G4_DEMAND_MARGIN, 32767)
        self.say(f"   demand {demand} = {why} {base} + {G4_DEMAND_MARGIN}")
        self.arm("gate4", LOCK_PROMPT)
        self.require_armed("gate4")
        self.start_drive(demand)
        self.window("gate4-settle", SETTLE_S)
        s = self.window("gate4", G4_WINDOW_S)
        psu = self.ask_float("PSU reading now (A), gate 4?")
        reason = psu_reading_abort(psu, self.psu_declared)
        if reason:
            raise SessionAbort(reason)
        r = gate4_verdict(s, self.a.limit_ma, self.staged_cpa, psu, locked=True, cal=self.cal)
        r["lines"].insert(0, f"held demand {demand} ({why} {base} + {G4_DEMAND_MARGIN})")
        self.end_step("gate4", r["verdict"])
        self.add_gate("Gate 4, the plateau", r)
        self.adopt_trips("gate4")
        self.release_and_confirm("gate4")
        if stay_for_gate5 and self.ask_yes("Stay armed for gate 5, rotor still locked?"):
            return True
        self.disarm_and_confirm("gate4")
        self.ask("You can release the rotor. Press Enter.")
        return False

    def adopt_trips(self, label):
        """A trip the plateau's own chopping caused: record it and carry it as gate 5's baseline
        rather than ending the session two gates from the end.

        MEASURED 2026-10-09 at a 2500 mA limit: gate 4 passed with trips 0 through its window and
        `trips` read 1 in the first sample after the Neutral. The hard trip is 2x the soft limit, the
        window held peaks of 3430 counts against a 4000-count trip, and a 10 Hz sampler sees one
        period in 1600, so an unsampled period crossing it is expected rather than surprising. It is
        the audit's thin-margin finding, which belongs in the record, not in an abort."""
        s = self.read_sample(f"{label}-trips")
        self.csv.row(s)
        if s["trips"] > self.trips_base:
            self.trips_base = s["trips"]
            note = (f"the hard trip latched during the plateau (trips {s['trips']}): at this limit the 2x hard "
                    f"trip ({hard_trip_counts(limit_counts(self.a.limit_ma, self.staged_cpa))} counts) is "
                    "within a one-period "
                    "excursion of the chop. Gate 5 baselines on it.")
            self.say(f"   NOTE: {note}")
            self.rec["warnings"].append(note)

    def gate5(self, armed):
        self.heading("gate 5, the trip (owner, rotor " + ("locked)" if self.locked else "held by hand)"))
        if not armed:
            self.arm("gate5", LOCK_PROMPT if self.locked else FREE_PROMPT)
        self.before_gate5 = False
        if self.locked:
            self.ask(f"Hand on the kill. Press Enter, then keep the rotor locked until told to release: "
                     f"{HANDS_ON}.")
            self.countdown()
        else:
            self.ask("Hand on the kill. Hold the rotor stalled with both hands on the tyre and keep it held until "
                     "told to release. Press Enter.")
        self.require_armed("gate5")
        base = self.read_sample("gate5-base")
        self.csv.row(base)
        self.start_drive(self.a.trip_demand)
        # The drive tool's walk is over and its first send is next: a hold lost during that walk is
        # an expired arm, not a missing trip. Sample-less, because a sampled check would race the trip.
        self.check_arm()
        self.armed_expected = False      # the trip is expected to drop the arm from here
        pre = self.window("gate5-trip", TRIP_WINDOW_S, until=lambda x: x["trips"] != base["trips"], fast=True)
        self.stop_drive()
        post = self.window("gate5-post", POST_TRIP_S, fast=True)
        self.check_node(self.neutral().stdout, "swd-mailbox-drive (Neutral)")
        self.ask("Release the rotor. Press Enter.")
        r = gate5_verdict(pre, post, base)
        self.end_step("gate5", r["verdict"])
        self.end_inputs()
        off = self.window("gate5-off", REARM_WAIT_S)
        self.ask("Hand on the kill. Wheel free. Press Enter to re-arm.")
        t0 = self.start_inputs()
        arm_s = self.window("gate5-rearm", ARM_WITHIN_S, until=lambda x: x["mode"] == MODE_RUN and x["moe"])
        still = self.window("gate5-still", REARM_WAIT_S)
        rr = rearm_verdict(off, arm_s, t0, still)
        r["lines"] += rr["lines"]
        if rr["verdict"] != "PASS":
            r["verdict"] = "FAIL"
        self.add_gate("Gate 5, the trip", r)
        self.disarm_and_confirm("gate5")

    def run(self):
        skip = set(self.a.skip_gate or [])
        try:
            self.stand_up()
            for g, fn in ((1, self.gate1), (2, self.gate2), (3, self.gate3)):
                if g in skip:
                    self.rec["gates"].append({"name": f"Gate {g}", "verdict": "SKIPPED", "lines": ["--skip-gate"]})
                else:
                    fn()
            armed = False
            if not (4 in skip and 5 in skip):
                self.stage_limit()
            if 4 in skip:
                self.rec["gates"].append({"name": "Gate 4", "verdict": "SKIPPED", "lines": ["--skip-gate"]})
            else:
                armed = self.gate4(stay_for_gate5=5 not in skip)
            if 5 in skip:
                self.rec["gates"].append({"name": "Gate 5", "verdict": "SKIPPED", "lines": ["--skip-gate"]})
            else:
                self.gate5(armed)
            self.rec["outcome"] = "COMPLETED: every gate that was not skipped ran."
        except SessionAbort as e:
            self.abort(str(e))
        except SessionEnd as e:
            self.rec["outcome"] = f"ENDED EARLY (not an abort): {e}"
            self.csv.comment(f"ended {e}")
        except KeyboardInterrupt:
            self.abort("operator interrupt (Ctrl-C)")
        except Exception as e:  # noqa: BLE001 - any failure still tears the bench down
            self.abort(f"tool error: {type(e).__name__}: {e}")
            raise
        finally:
            self.close()

    def abort(self, reason):
        self.rec["outcome"] = f"ABORTED: {reason}"
        self.csv.comment(f"abort {reason}")
        self.say(f"\n!! ABORT: {reason}")

    # ---- teardown ----------------------------------------------------------------------------
    def close(self):
        """Flush, tear down, flush. Signals are deferred for the whole of it, and a failed flush
        (a full disk) is recorded and does not stop the teardown."""
        if self.closed:
            return
        self.closed = True
        with self.sh.defer_signals():
            self._flush_guarded("before teardown")   # so an abort leaves evidence
            self.teardown()
            self._flush_guarded("after teardown")

    def _flush_guarded(self, when):
        try:
            self.flush_record()
        except BaseException as e:  # noqa: BLE001 - nothing may stop the teardown
            msg = f"record flush {when} FAILED: {type(e).__name__}: {e}"
            self.rec["final"].append(msg)
            try:
                self.say(f"!! {msg}")
            except BaseException:  # noqa: BLE001
                pass

    def _td(self, tag, text):
        self.teardown_log.append(tag)
        self.rec["teardown"].append(f"{tag}: {text}")
        self.csv.comment(f"teardown {tag}: {text}")
        self.say(f"   teardown {tag}: {text}")

    def teardown(self):
        """End the drive hold, end the inputs hold, moe check, explicit Neutral, rail off, verify,
        OpenOCD kill, tunnel close, lock release. Idempotent. Every step, and the logging around it,
        is guarded: a failure or a signal landing anywhere in one step never skips the next."""
        if self.torn_down:
            return
        self.torn_down = True
        self.armed_expected = False
        try:
            self.say("\n== teardown ==")
        except BaseException:  # noqa: BLE001
            pass
        mailbox_up = self.reader is not None and self.node is not None
        state = {"moe_ok": None}

        def drive_end():
            if self.drive is None:
                return "no drive hold running"
            self.stop_drive()
            return "demand released (Neutral; the firmware zeroes it in 200 ms)"

        def hold_end():
            if self.inputs is None:
                return "no hold running"
            self.end_inputs()
            return "inputs hold ended (explicit all-clear)"

        def moe():
            if self.reader is None:
                return "skipped: no SWD connection"
            t0 = self.sh.now()
            while self.sh.now() - t0 < MOE_CHECK_S:
                smp = self.read_sample("teardown")
                self.csv.row(smp)
                if smp["moe"] == 0 and smp["mode"] == MODE_OFF:
                    state["moe_ok"] = True
                    return "moe_bits 0, mode_byte OFF"
                self.sh.sleep(1.0 / SAMPLE_HZ)
            state["moe_ok"] = False
            return "FAILED (still armed after 3 s): rail OFF immediately"

        def neutral():
            # After the disarm: its fresh attach and walk can take up to 30 s and must not delay it.
            if not mailbox_up:
                return "skipped: the mailbox was never reached"
            if state["moe_ok"] is not True:
                return "skipped: the moe check did not pass, the rail goes off first"
            self.neutral()
            return "explicit Neutral sent"

        def rail_off():
            if not self.rail_touched:
                return "skipped: this run never touched the rail"
            self.sh.run(f"ssh {PI} 'pinctrl set 4 op dh'")
            return "pinctrl set 4 op dh"

        def verify():
            if not self.rail_touched:
                return "skipped"
            out = self.sh.run(f"ssh {PI} 'pinctrl get 4'").stdout.strip()
            if "hi" in out.split("//")[0]:
                self.rec["final"].append(f"rail OFF confirmed (`pinctrl get 4` -> `{out}`)")
                return f"rail OFF confirmed ({out})"
            self.rec["final"].append(f"RAIL NOT CONFIRMED OFF (`pinctrl get 4` -> `{out or 'no reply'}`): "
                                     "power the rail down by hand")
            return f"WARNING: rail did not confirm OFF ({out or 'no reply'}); cut it by hand"

        def ocd_kill():
            if self.reader is not None:
                self.reader.close()
                self.reader = None
            if not self.ocd_started:
                return "skipped: not started"
            self.sh.run(swdobs.stop_remote_ocd_cmd())
            return "OpenOCD stopped"

        def tunnel_close():
            if self.tunnel is None:
                return "skipped: not opened"
            self.say("  + (close the tunnel)")
            self.tunnel.end()
            self.tunnel = None
            return "tunnel closed"

        def lock_release():
            if not self.took_lock:
                return "skipped: not acquired by this run"
            self.sh.run(f"{shlex.quote(os.path.join(REPO, 'tools', 'bench-lock.sh'))} release {self.owner}")
            return f"released {self.owner}"

        for tag, fn in TEARDOWN_STEPS_OF(drive_end, hold_end, moe, neutral, rail_off, verify, ocd_kill,
                                         tunnel_close, lock_release):
            try:
                text = fn()
            except BaseException as e:  # noqa: BLE001 - a failure or a signal: record it, carry on
                text = f"failed: {type(e).__name__} {e}"
                if tag == "moe_check":
                    state["moe_ok"] = False
                    text += ": rail OFF immediately"
            try:
                self._td(tag, text)
            except BaseException:  # noqa: BLE001
                if not self.teardown_log or self.teardown_log[-1] != tag:
                    self.teardown_log.append(tag)
        moe_ok = state["moe_ok"]

        if moe_ok is False:
            self.rec["final"].append("the moe check FAILED at teardown: the rail was cut with the bridge "
                                     "possibly armed; read CTRL_OBS from the next boot")
        staged = self.a.limit_ma if self.limit_written else None
        prev = self.rec.get("prev_limit_ma")
        if staged is not None:
            self.rec["final"].append(
                f"limit left staged at {staged} mA (0x20); the tool does not restore it. The previous value "
                f"was {prev} mA: restore by hand with swd-mailbox-config 0x20={prev} if wanted")
        else:
            prev_txt = f"{prev} mA" if prev is not None else "not read"
            self.rec["final"].append(f"0x20 not changed by this run ({prev_txt})")

    # ---- the record -----------------------------------------------------------------------------
    def flush_record(self):
        self.drain_children()
        text = render_record(self.rec)
        if self.sh.dry:
            self.record_text = text
            return
        os.makedirs(self.record_dir, exist_ok=True)
        with open(self.record_path, "w") as fh:
            fh.write(text)


# --------------------------------------------------------------------------------------------------
def ladder_line(ladder):
    return "demand ladder: " + ", ".join(
        f"{d} -> PSU {a:g} A, duty {du:.0f}, estimate {'n/a' if e is None else f'{e:.2f} A'} (peak {pk:.0f})"
        for d, a, pk, du, e in ladder)


def build_parser():
    ap = argparse.ArgumentParser(
        description="The current limit's energised bench gates as one guided session "
                    "(specs/current-limit-session.md).")
    ap.add_argument("--board", choices=sorted(swdobs.BOARDS), default="master")
    # 2500 mA = 2000 counts. The audit wants the soft limit near 5 A so the 2x hard trip is clear of a
    # one-period excursion, and at the provisional 800 counts per amp (measured about 480) 2500 mA
    # already chops at about 5 A in the winding. --limit-ma 3000 buys a thicker margin at the cost of
    # more torque for the operator to hold.
    ap.add_argument("--limit-ma", type=int, default=2500)
    ap.add_argument("--cal-demand", type=int, default=3000)
    ap.add_argument("--trip-demand", type=int, default=32767)
    ap.add_argument("--record", default=None, help="default specs/bench-evidence/<today>/current-limit/")
    ap.add_argument("--elf", default=swdobs.DEFAULT_ELF)
    ap.add_argument("--brake-fallback", action="store_true",
                    help="gates 3 and 4 brake a spinning wheel by hand instead of locking the rotor "
                         "(only when nothing can hold the rotor)")
    ap.add_argument("--skip-gate", type=int, action="append", choices=[1, 2, 3, 4, 5], default=[])
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    return ap


def selftest():
    import unittest
    suite = unittest.defaultTestLoader.discover(os.path.join(_HERE, "tests"), pattern="test_climit_session.py")
    res = unittest.TextTestRunner(verbosity=1).run(suite)
    return 0 if res.wasSuccessful() else 1


def main(argv=None, shell=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["--child-watchdog"]:
        return child_watchdog(argv[2:] if argv[1:2] == ["--"] else argv[1:])
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest()
    err = check_limit_arg(args.limit_ma)
    if err:
        print(f"{TOOL}: refusing: {err}", file=sys.stderr)
        return 2
    for name in ("cal_demand", "trip_demand"):
        v = getattr(args, name)
        if not ENGAGE_DEMAND_MIN <= v <= 32767:
            print(f"{TOOL}: refusing: --{name.replace('_', '-')} {v} must be {ENGAGE_DEMAND_MIN}..32767 "
                  "(below the engagement gate nothing moves)", file=sys.stderr)
            return 2

    today = time.strftime("%Y-%m-%d")
    record_dir = args.record or os.path.join(REPO, "specs", "bench-evidence", today, "current-limit")
    if shell is None:
        if args.dry_run:
            sim = SimBoard()
            sim.locked = not args.brake_fallback      # the simulated operator follows the chosen procedure
            shell = FakeShell(sim=sim, exists=os.path.exists)
        else:
            shell = RealShell()
    stamp = time.strftime("%H%M%S")
    csv_name = f"climit-{stamp}.csv"
    if shell.dry:
        csv_fh = io.StringIO()
        record_path = os.path.join(record_dir, "RECORD.md")
        print(f"{TOOL}: DRY RUN against a simulated board: nothing is executed, no network, no lock, "
              f"no files (the record would go to {record_dir}).")
    else:
        os.makedirs(record_dir, exist_ok=True)
        csv_fh = open(os.path.join(record_dir, csv_name), "w")
        record_path = os.path.join(record_dir, "RECORD.md")
        if os.path.exists(record_path):
            record_path = os.path.join(record_dir, f"RECORD-{stamp}.md")
    session = Session(args, shell, csv_fh, record_dir, today)
    session.record_path = record_path
    session.rec["csv"] = csv_name

    def on_term(signum, _frame):
        raise SessionAbort(f"signal {signal.Signals(signum).name}")

    if not shell.dry:
        import atexit
        atexit.register(session.close)
        signal.signal(signal.SIGTERM, on_term)
        signal.signal(signal.SIGHUP, on_term)
    session.run()
    if shell.dry:
        print("\n== dry run: RECORD.md would read ==\n")
        print(session.record_text)
    else:
        csv_fh.close()
        print(f"\n{TOOL}: record {record_path}, CSV {os.path.join(record_dir, csv_name)}")
    print(f"{TOOL}: {session.rec['outcome']}")
    return 0 if session.rec["outcome"].startswith("COMPLETED") else 1


if __name__ == "__main__":
    sys.exit(main())
