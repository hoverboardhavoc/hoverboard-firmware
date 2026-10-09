#!/usr/bin/env python3
"""Unit tests for tools/climit-session.py (and the shared tools/swdobs.py it imports).

Run: python3 -m unittest discover -s tools/tests -p 'test_climit_session.py'
 or: tools/climit-session.py --selftest

Covers specs/current-limit-session.md, "Tests": the parsers (mailbox tool output, the CTRL_OBS
block, the probe table, nm), every gate's verdict function with passing, failing and abort inputs,
the calibration arithmetic with clamp-meter precedence, the RECORD renderer against a golden, the
teardown order against the fake shell (including the moe-check-failed path), the dry run end to end
with no side effects, and the child watchdog. stdlib only, no hardware.
"""

import contextlib
import importlib.util
import io
import os
import signal
import subprocess
import sys
import time
import unittest
from unittest import mock

_TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _TOOLS)
_spec = importlib.util.spec_from_file_location("climit_session", os.path.join(_TOOLS, "climit-session.py"))
cs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cs)
import swdobs  # noqa: E402

OFF, RUN = cs.MODE_OFF, cs.MODE_RUN
SYMS = {k: v[0] for k, v in cs.SIM_SYMS.items()}
_, SPAN, OFFS = swdobs.motor_block(SYMS)


def mk(t=0.0, label="x", statics=None, **f):
    """A decoded sample built through the real encoder and decoder."""
    w = cs.encode_ctrl_obs(f)
    m = [0] * SPAN
    for k, v in (statics or {}).items():
        m[OFFS[k]] = v & 0xFFFFFFFF
    return cs.decode_sample(w, m, OFFS, t, label)


def series(n, dt=0.1, t0=0.0, **f):
    return [mk(t=t0 + i * dt, **f) for i in range(n)]


class Parsers(unittest.TestCase):
    INPUTS_OUT = (
        "dst resolved: attached node 0x02 (port table: 0x02 reports the host on its port 0, kind 2 = SWD mailbox)\n"
        "sent INPUTS 0x05->0x02: buttons=0x01 (power_request=true) rider=0x01 (rider_present=true)\n"
    )
    READ_OUT = (
        "dst resolved: attached node 0x02 (port table)\n1 CONFIG op(s) on node 0x02\n"
        "  CONFIG_READ 0x20:0 -> CFG_OK value U32(10000)\n"
        "PASS: 1 CONFIG op(s) answered CFG_OK; a staged layout needs a REBOOT for the board's boot validator to judge it\n"
    )
    WRITE_OUT = (
        "dst resolved: attached node 0x02 (port table)\n1 CONFIG op(s) on node 0x02\n"
        "  CONFIG_WRITE 0x20:0 = 2500 (U32) -> CFG_OK\n"
        "  CONFIG_READ  0x20:0 -> CFG_OK value U32(2500)  (write -> read matches)\n"
        "PASS: 1 CONFIG op(s) answered CFG_OK; a staged layout needs a REBOOT\n"
    )

    def test_dst(self):
        self.assertEqual(cs.parse_dst(self.INPUTS_OUT), 0x02)
        self.assertEqual(cs.parse_dst("dst resolved: attached node 0x1f (first contact)"), 0x1F)
        self.assertIsNone(cs.parse_dst("dst 0x01 (explicit; this IS the attached node)"))
        self.assertIsNone(cs.parse_dst("FAIL: walk resolved no attached node"))

    def test_config_read(self):
        self.assertEqual(cs.parse_config_read(self.READ_OUT, 0x20), ("CFG_OK", 10000))
        self.assertEqual(cs.parse_config_read(self.WRITE_OUT, 0x20), ("CFG_OK", 2500))
        self.assertEqual(cs.parse_config_read("  CONFIG_READ 0x20:0 -> CFG_ARMED\n", 0x20), ("CFG_ARMED", None))
        self.assertEqual(cs.parse_config_read(self.READ_OUT, 0x21), (None, None))

    def test_config_write(self):
        self.assertTrue(cs.config_write_ok(self.WRITE_OUT, 0x20))
        self.assertFalse(cs.config_write_ok(self.WRITE_OUT.replace("(write -> read matches)", "MISMATCH"), 0x20))
        self.assertFalse(cs.config_write_ok(self.READ_OUT, 0x20))
        self.assertFalse(cs.config_write_ok("  CONFIG_WRITE 0x20:0 = 2500 (U32) -> CFG_ARMED\n", 0x20))

    def test_decode_canned_words(self):
        w = [0] * 33
        w[0] = 0x4C525443
        w[1] = 3
        w[2], w[4] = 7529, 7528
        w[11] = 0x0103FFF6          # torque -10, mode RUN, moe 1
        w[12] = 0x10030003          # sub 3, cmode 0, flags 0x03, levels 0x10
        w[20] = 0x87000301          # hall 1, enables 3, flags 0x87
        w[21] = 0x00000708          # d0 1800, d1 0
        w[22] = 0x01230000          # d2 0, angle 0x123
        w[23] = 0x00050004          # FAULT_DEMAND_STALE, dwell 5
        w[24] = 0xFFFFFFA6          # speed -90
        w[25] = 0x7F107E07
        w[27], w[28] = 0x00000000, 0x00000003  # latch-A count 3
        w[31] = 0x012804B0          # peak 1200, chopped 40, trips 1
        w[32] = 2497
        m = [0] * SPAN
        m[OFFS["DEMAND"]] = 3000
        s = cs.decode_sample(w, m, OFFS, 1.0, "lbl")
        self.assertEqual((s["torque"], s["mode"], s["moe"]), (-10, RUN, 1))
        self.assertEqual((s["sub"], s["cmode"], s["flags"], s["levels"]), (3, 0, 3, 0x10))
        self.assertEqual((s["mflags"], s["d0"], s["d2"], s["duty_on"]), (0x87, 1800, 0, 1800))
        self.assertEqual((s["fault"], s["dwell"], s["speed"]), (4, 5, -90))
        self.assertEqual((s["peak"], s["chopped"], s["trips"], s["latch_a"]), (1200, 40, 1, 3))
        self.assertEqual((s["battery"], s["demand"], s["label"]), (2497, 3000, "lbl"))

    def test_encode_decode_roundtrip(self):
        s = mk(mode=RUN, moe=1, sub=3, speed=-5, peak=-3, chopped=64, trips=2, d1=1956, ev=[1, 0, 0, 0, 5, 0, 0, 0])
        self.assertEqual((s["mode"], s["moe"], s["sub"], s["speed"], s["peak"]), (RUN, 1, 3, -5, -3))
        self.assertEqual((s["chopped"], s["trips"], s["duty_on"], s["latch_a"]), (64, 2, 1956, 5))

    def test_tear_guard(self):
        a = cs.encode_ctrl_obs({"tick": 100})
        b = cs.encode_ctrl_obs({"tick": 103})
        self.assertTrue(cs.tear_ok(a, b))
        b[cs.W_CONTROL_TICKS] += 1
        self.assertFalse(cs.tear_ok(a, b))

    def test_oc_cfg_from_flash_sh(self):
        with open(swdobs.FLASH_SH) as fh:
            text = fh.read()
        master = swdobs.extract_oc_cfg("master", text)
        self.assertTrue(master.startswith("-f interface/stlink.cfg -c 'transport select dapdirect_swd'"))
        self.assertIn("set CPUTAPID 0", master)
        self.assertIn("vid_pid 0x1209 0xda42", swdobs.extract_oc_cfg("slave", text))
        self.assertIsNone(swdobs.extract_oc_cfg("nonesuch", text))

    def test_parse_nm(self):
        out = "20000ac8 00000084 B CTRL_OBS\n" + "".join(
            f"{a:08x} 00000004 b _RNvNtCs9BfhVdskVqt_8firmware5motor{len(n)}{n}.0\n"
            for n, (a, _) in cs.SIM_SYMS.items() if n != "CTRL_OBS")
        syms = swdobs.parse_nm(out)
        self.assertEqual(syms["CTRL_OBS"], (0x20000AC8, 132))
        self.assertEqual(syms["DEMAND"][0], 0x2000090C)
        with self.assertRaises(SystemExit):
            swdobs.parse_nm("20000ac8 00000084 B CTRL_OBS\n")


class Rules(unittest.TestCase):
    def test_limit_counts(self):
        # The board's counts per amp is an argument now, not a constant: crates/store/src/field.rs
        # MOTOR_CURRENT_CAL (0x67), read off the board at stand-up. 455 is its registered default.
        cpa = cs.CURRENT_CAL_DEFAULT
        self.assertEqual(cs.limit_counts(10_000, cpa), 4550)
        # The floor is a COUNT, so a label worth less than 2000 counts is clamped up to it: the
        # session's own 2500 mA default is 1137 counts at this scale.
        self.assertEqual(cs.limit_counts(2500, cpa), cs.MIN_LIMIT_COUNTS)
        self.assertEqual(cs.limit_counts(999, cpa), cs.MIN_LIMIT_COUNTS)
        self.assertEqual(cs.limit_counts(4_700, cpa), 2138, "clear of the floor")
        # The milliamp ceiling stays a milliamp ceiling.
        self.assertEqual(cs.limit_counts(40_001, cpa), 18_200)
        # The scale's own seam, the firmware's boot-seam clamp (CURRENT_CAL_MIN/MAX): the tool
        # reports what the board will do, not what the stored word says.
        self.assertEqual(cs.limit_counts(40_000, 10_000), 32_760)
        self.assertEqual(cs.limit_counts(40_000, 0), 4_000)
        self.assertEqual(cs.limit_counts(10_000, 800), 8_000)
        self.assertEqual(cs.hard_trip_counts(2000), 4000)
        self.assertEqual(cs.hard_trip_counts(32_000), 32_767)

    def test_limit_arg(self):
        self.assertIn("below 2000", cs.check_limit_arg(1999))
        self.assertIsNone(cs.check_limit_arg(2000))
        self.assertIsNone(cs.check_limit_arg(2500))
        self.assertIsNone(cs.check_limit_arg(5000))
        self.assertIn("above 5000", cs.check_limit_arg(5001))
        self.assertIn("never goes above 6 A", cs.check_limit_arg(5500))

    def test_psu_rule(self):
        """The PSU limit bounds the DC LINK, which at a locked rotor carries the phase current times
        the duty: measured 0.60 to 0.68 A while a 2500 mA limit chopped at about 5 A in the winding.
        So the rule is a FLOOR that clears the predicted link current, plus the bench's 6 A cap, and
        anything in between is the operator's call. A lower setting is the safer one, because the PSU
        limit is what bounds a shoot-through."""
        self.assertLess(cs.link_at_chop_a(2500), 1.0)                 # under an amp at the chop
        self.assertTrue(cs.psu_declared_ok(2500, 3.5))                # what this bench is set to
        self.assertTrue(cs.psu_declared_ok(2500, 1.5))                # the floor
        self.assertTrue(cs.psu_declared_ok(2500, 6.0))                # the cap
        self.assertFalse(cs.psu_declared_ok(2500, 6.5))               # above the cap
        self.assertFalse(cs.psu_declared_ok(5000, 1.5))               # under the link current at 5 A
        self.assertTrue(cs.psu_declared_ok(5000, 4.0))
        # And the old rule's demand, the staged limit plus an amp, was never what the link carries.
        self.assertLess(cs.psu_target_a(2500), 2500 / 1000.0 + 1.0)

    def test_psu_reading_abort(self):
        self.assertIsNone(cs.psu_reading_abort(3.5, 3.5))
        self.assertIn("above the 3.5 A limit", cs.psu_reading_abort(3.6, 3.5))


class Ladder(unittest.TestCase):
    def test_phase_estimate(self):
        self.assertAlmostEqual(cs.phase_estimate(0.94, 477), 0.94 * 2250 / 477)
        self.assertIsNone(cs.phase_estimate(1.0, 0))

    def test_steps(self):
        self.assertEqual(cs.ladder_next(3000, 1.6, False), ("up", 4000))
        self.assertEqual(cs.ladder_next(8000, 4.0, False), ("done", 8000))       # 4 A is in the band
        self.assertEqual(cs.ladder_next(8000, 7.99, False), ("done", 8000))
        self.assertEqual(cs.ladder_next(9000, 8.0, False), ("back", 8500))       # 8 A is not
        self.assertEqual(cs.ladder_next(8500, 6.0, True), ("done", 8500))

    def test_aborts(self):
        move, why = cs.ladder_next(12000, 3.9, False)
        self.assertEqual(move, "abort")
        self.assertIn("draws less than expected; check the lock and the phase wiring", why)
        move, why = cs.ladder_next(8500, 9.0, True)
        self.assertEqual(move, "abort")
        self.assertIn("could chop", why)
        move, why = cs.ladder_next(8500, 3.5, True)
        self.assertEqual(move, "abort")
        self.assertIn("straddles", why)
        self.assertEqual(cs.ladder_next(1090, 9.0, False), ("back", 590))           # 590 engages
        self.assertEqual(cs.ladder_next(1080, 9.0, False)[0], "abort")             # 580 would not
        self.assertEqual(cs.ladder_next(3000, None, False)[0], "abort")


class Verdicts(unittest.TestCase):
    def test_abort_reason(self):
        ok = mk(mode=OFF)
        self.assertIsNone(cs.abort_reason(ok))
        self.assertIn("magic", cs.abort_reason(mk(magic=0xDEADBEEF)))
        self.assertIn("moe_bits", cs.abort_reason(mk(mode=OFF, moe=1)))
        self.assertIn("moe_bits", cs.abort_reason(mk(mode=cs.MODE_SHUTDOWN, moe=1)))
        self.assertIsNone(cs.abort_reason(mk(mode=cs.MODE_INIT, moe=1)))   # MOE rises on the INIT pass
        self.assertIn("motor_fault", cs.abort_reason(mk(mode=OFF, fault=4)))
        self.assertIsNone(cs.abort_reason(mk(mode=RUN, moe=1, fault=4)))   # not a disarmed read
        self.assertIn("before gate 5", cs.abort_reason(mk(trips=1)))
        self.assertIsNone(cs.abort_reason(mk(trips=1), before_gate5=False))
        self.assertIn("gate 2", cs.abort_reason(mk(chopped=3), gate2=True))
        self.assertIsNone(cs.abort_reason(mk(chopped=3)))
        self.assertIn("latch-A", cs.abort_reason(mk(ev=[0, 0, 0, 0, 1, 0, 0, 0])))
        self.assertIsNone(cs.abort_reason(mk(ev=[0, 0, 0, 0, 2, 0, 0, 0])))

    def test_standup(self):
        self.assertEqual(cs.standup_problems(mk()), [])
        self.assertTrue(any("calibration" in p for p in cs.standup_problems(mk(mflags=0x07))))
        self.assertTrue(any("balance" in p for p in cs.standup_problems(mk(cmode=1))))
        self.assertTrue(any("not OFF" in p for p in cs.standup_problems(mk(mode=RUN))))

    def test_gate1(self):
        r = cs.gate1_verdict([mk(peak=1000), mk(peak=1300)], cs.CURRENT_CAL_DEFAULT)
        self.assertEqual((r["verdict"], r["peak_max"], r["peak_mean"]), ("INFO", 1300, 1150))
        # The amps the line quotes are the BOARD's: 1300 counts at 455 counts per amp.
        self.assertIn("2.86 A equivalent at this board's 455 counts/A", r["lines"][1])
        self.assertEqual(cs.gate1_verdict([mk(peak=1000, chopped=1)], 455)["verdict"], "FAIL")
        self.assertEqual(cs.floor_max(None), cs.FLOOR_FALLBACK_COUNTS)

    def test_gate2(self):
        self.assertEqual(cs.gate2_verdict(series(5, peak=1200, sub=3), 1300)["verdict"], "PASS")
        self.assertEqual(cs.gate2_verdict(series(5, peak=2000), 1300)["verdict"], "FAIL")
        self.assertEqual(cs.gate2_verdict(series(5, mode=RUN, moe=1), 1300)["verdict"], "FAIL")
        self.assertEqual(cs.gate2_verdict(series(5, chopped=2), 1300)["verdict"], "FAIL")

    def test_arm_and_soak(self):
        s = series(3, mode=OFF) + series(3, t0=0.3, mode=RUN, moe=1)
        self.assertTrue(cs.arm_ok(s, 0.0)[0])
        self.assertFalse(cs.arm_ok(series(25, mode=OFF), 0.0)[0])
        late = series(25, mode=OFF) + [mk(t=2.6, mode=RUN, moe=1)]
        self.assertFalse(cs.arm_ok(late, 0.0)[0])
        self.assertIsNone(cs.soak_abort(series(5, mode=RUN, moe=1, peak=1200), 1300))
        self.assertIn("moved", cs.soak_abort(series(5, mode=RUN, moe=1, speed=3), 1300))
        self.assertIn("rest floor", cs.soak_abort(series(5, mode=RUN, moe=1, peak=2500), 1300))
        self.assertIn("dropped", cs.soak_abort(series(5, mode=OFF), 1300))

    def test_a_soak_is_judged_on_its_p90_not_a_noise_extreme(self):
        """The real distribution from 2026-10-09: an armed, undemanded soak at the rest floor with
        one sample far out in the tail. It must not abort, because `peak` is itself a window maximum
        over ADC noise and its extremes cross any threshold set near the floor. A soak with the whole
        distribution lifted must still abort, which is what the check is for."""
        floor = 2097                                   # that session's disarmed gate-1 maximum
        quiet = series(93, mode=RUN, moe=1, peak=1072) + [mk(t=9.4, mode=RUN, moe=1, peak=2112)]
        self.assertIsNone(cs.soak_abort(quiet, floor))
        # The p90 the quiet soaks of that day actually ran, against the same floor.
        for p90_measured in (1366, 1440):
            tail = series(10, t0=9.4, mode=RUN, moe=1, peak=p90_measured)
            self.assertIsNone(cs.soak_abort(series(84, mode=RUN, moe=1, peak=1072) + tail, floor))
        lifted = series(94, mode=RUN, moe=1, peak=int(1.6 * floor))
        self.assertIn("current is flowing", cs.soak_abort(lifted, floor))
        # And a single sample far enough out is still a gross fault.
        gross = series(93, mode=RUN, moe=1, peak=1072) + [mk(t=9.4, mode=RUN, moe=1, peak=4 * floor)]
        self.assertIn("over 2x the rest floor", cs.soak_abort(gross, floor))

    def test_current_in_a_minority_of_samples_is_not_a_passing_soak(self):
        """What the p90 catches and a median does not. 45 of 94 samples carrying real current, the
        other 49 at the rest floor, is a gate conducting intermittently: the median reads the floor
        and passes it, which is why the median rule had to go. Every sample here is under the gross
        per-sample ceiling, so the p90 is the only thing that can fail it."""
        floor = 2097
        intermittent = (series(49, mode=RUN, moe=1, peak=1072)
                        + series(45, t0=4.9, mode=RUN, moe=1, peak=4000))
        self.assertEqual(cs._percentile([s["peak"] for s in intermittent], 50), 1072)
        self.assertLess(4000, cs.SOAK_GROSS_FACTOR * floor)
        reason = cs.soak_abort(intermittent, floor)
        self.assertIsNotNone(reason, "current in 45 of 94 samples must abort the soak")
        self.assertIn("current is flowing", reason)
        # And the gross ceiling is 2x, not 3x: a sample at 2.5x the floor is a fault.
        at_2_5x = series(93, mode=RUN, moe=1, peak=1072) + [mk(t=9.4, mode=RUN, moe=1, peak=5242)]
        self.assertIn("over 2x the rest floor", cs.soak_abort(at_2_5x, floor))

    def test_hall_jitter_at_a_held_rotor_is_not_rotation(self):
        """The real samples from 2026-10-09: one +1 and one -1 in 94, isolated, at a rotor held by
        hand. That ended a session two gates from the end. Rotation is a NET of two hall edges or
        more over the window."""
        quiet = series(50, mode=RUN, moe=1)
        jitter = (series(20, mode=RUN, moe=1) + [mk(t=2.1, mode=RUN, moe=1, speed=1)]
                  + series(20, t0=2.2, mode=RUN, moe=1) + [mk(t=4.4, mode=RUN, moe=1, speed=-1)]
                  + series(20, t0=4.5, mode=RUN, moe=1))
        self.assertEqual(cs.rotor_moved(quiet), 0)
        self.assertEqual(cs.rotor_moved(jitter), 0)
        self.assertIsNone(cs.soak_abort(jitter, 2097))
        # Two edges one way, however they are spread, or anything bigger in one sample, is movement.
        self.assertEqual(cs.rotor_moved([mk(speed=1), mk(t=0.1, speed=1)]), 2)
        self.assertEqual(cs.rotor_moved([mk(speed=-4)]), -4)
        self.assertIn(cs.ROTOR_MOVED, cs.soak_abort(series(5, mode=RUN, moe=1, speed=3), 2097))

    def test_a_creep_slower_than_one_edge_per_sample_is_rotation(self):
        """The case the consecutive-samples rule admitted: a rotor turning slowly enough that the
        hall edges land in separate samples reads 1,0,0,1,0,1, which is never consecutive and never
        bigger than one unit, and is three edges one way. The net sum is what makes it movement,
        and the same rule still passes a +1 cancelled by a -1."""
        edges = (1, 0, 0, 1, 0, 1)
        self.assertEqual(cs.rotor_moved([mk(t=0.1 * i, speed=v) for i, v in enumerate(edges)]), 2)
        armed = [mk(t=0.1 * i, mode=RUN, moe=1, speed=v) for i, v in enumerate(edges)]
        reason = cs.soak_abort(armed, 2097)
        self.assertIsNotNone(reason, "a creeping rotor must re-prompt for a firmer lock")
        self.assertIn(cs.ROTOR_MOVED, reason)
        # A creep the other way, and the cancelling pair that is not a creep.
        self.assertEqual(cs.rotor_moved([mk(t=0.1 * i, speed=v)
                                         for i, v in enumerate((-1, 0, -1))]), -2)
        self.assertEqual(cs.rotor_moved([mk(speed=1), mk(t=0.1), mk(t=0.2, speed=-1)]), 0)

    def test_the_rest_floor_fallback_covers_the_measured_floor(self):
        """A skipped gate 1 leaves no measured floor, and the fallback has to be at least what the
        bench actually reads at rest, or every armed soak aborts on noise (it did, at 1400 against a
        measured 2097 on 2026-10-09)."""
        self.assertGreaterEqual(cs.FLOOR_FALLBACK_COUNTS, 2097)
        self.assertEqual(cs.floor_max(None), cs.FLOOR_FALLBACK_COUNTS)
        self.assertEqual(cs.floor_max({"peak_max": 1500}), 1500)

    def test_spin(self):
        s = series(3, sub=0) + series(27, t0=0.3, sub=3, speed=40)
        self.assertTrue(cs.spin_ok(s, 0.0)[0])
        ok, detail = cs.spin_ok(series(30, sub=0), 0.0)
        self.assertFalse(ok)
        self.assertIn("D4 phase-order", detail)
        flip = series(3, t0=0.3, sub=3, speed=40) + series(10, t0=0.6, sub=3, speed=-40)
        self.assertFalse(cs.spin_ok(flip, 0.0)[0])

    # A window whose signal CAN carry a verdict: a rest floor well under the window's peaks, and a
    # ladder spanning more than that floor. With these the verdict turns on the arithmetic rather
    # than on the noise test (`cal_signal_weak`), which has its own tests below.
    STRONG = {"floor_counts": 400, "ladder_peaks": (900, 1820)}

    def test_calibration_duty_corrected(self):
        # The measurement is judged against the scale the BOARD carries (0x67), so the CONFIRMED
        # band moves with it: 1820 counts at a 4.0 A reference is exactly the staged 455.
        r = cs.calibration(series(30, peak=1820, d0=1800), 3.2, None, cs.CURRENT_CAL_DEFAULT,
                           **self.STRONG)
        self.assertAlmostEqual(r["i_est"], 4.0)
        self.assertAlmostEqual(r["cpa"], 455.0)
        self.assertEqual((r["verdict"], r["source"]), ("CONFIRMED", "PSU, duty-corrected"))
        self.assertIn("the staged motor.current_cal 455 confirmed", r["recommendation"])
        # The same samples against a board staging 800: the same measurement is now a CORRECTION.
        self.assertEqual(cs.calibration(series(30, peak=1820, d0=1800), 3.2, None, 800,
                                        **self.STRONG)["verdict"], "CORRECTION")

    def test_calibration_clamp_precedence(self):
        r = cs.calibration(series(30, peak=1820, d0=1800), 3.2, 2.0, cs.CURRENT_CAL_DEFAULT,
                           **self.STRONG)
        self.assertEqual((r["source"], r["i_ref"]), ("clamp meter", 2.0))
        self.assertAlmostEqual(r["cpa"], 910.0)
        self.assertEqual((r["verdict"], r["proposed"]), ("CORRECTION", 910))
        self.assertIn("NOT staged", r["recommendation"])
        # A correction is staged into the field, on this board, not edited into a source file.
        self.assertIn("motor.current_cal (0x67) = 910", r["recommendation"])
        self.assertIn("0x67=910", r["recommendation"])

    def test_calibration_duty_from_largest_channel(self):
        r = cs.calibration([mk(peak=1400, d0=0, d1=1125, d2=300)], 1.0, None, cs.CURRENT_CAL_DEFAULT,
                           floor_counts=400, ladder_peaks=(900, 1400))
        self.assertAlmostEqual(r["duty_on"], 1125)
        self.assertAlmostEqual(r["i_est"], 2.0)
        self.assertAlmostEqual(r["cpa"], 700.0)
        self.assertEqual(r["verdict"], "CORRECTION")

    def test_calibration_invalid(self):
        cpa = cs.CURRENT_CAL_DEFAULT
        self.assertEqual(
            cs.calibration(series(5, peak=1820, d0=1800, chopped=4), 3.2, None, cpa,
                           **self.STRONG)["verdict"], "INVALID")
        self.assertEqual(cs.calibration(series(5, peak=1820), 3.2, None, cpa,
                                        **self.STRONG)["verdict"], "INVALID")

    def test_a_noise_dominated_window_is_an_upper_bound_not_a_confirmation(self):
        """specs/current-limit-session.md: the verdict is INCONCLUSIVE, naming the ratio as an UPPER
        BOUND, when the window's mean peak is under about twice the gate-1 rest-floor maximum. The
        numbers are 2026-10-09's: a mean peak barely above a 2097-count floor, which is the session
        that reported a verdict it could not support."""
        floor, ladder = 2097, (1700, 1900)
        r = cs.calibration(series(30, peak=1900, d0=1800), 3.2, 4.0, cs.CURRENT_CAL_DEFAULT,
                           floor_counts=floor, ladder_peaks=ladder)
        self.assertEqual(r["verdict"], "INCONCLUSIVE")
        self.assertAlmostEqual(r["cpa"], 475.0)          # the ratio is still reported
        self.assertIsNone(r["proposed"])                 # but it is not a value to stage
        self.assertIn("UPPER BOUND", r["recommendation"])
        self.assertIn("under 2x the 2097-count rest floor", r["recommendation"])
        self.assertIn("current-sense calibration", r["recommendation"])
        # The same arithmetic over a window that clears the floor, with a ladder that has a slope,
        # is the CONFIRMED it used to be: it is the signal that changed the verdict, not the ratio.
        r = cs.calibration(series(30, peak=1900, d0=1800), 3.2, 4.0, cs.CURRENT_CAL_DEFAULT,
                           floor_counts=400, ladder_peaks=(900, 1900))
        self.assertEqual((r["verdict"], round(r["cpa"])), ("CONFIRMED", 475))

    def test_a_ladder_without_two_separated_points_cannot_carry_a_verdict(self):
        """The second half of the same rule: fewer than two ladder points separated by more than the
        floor means there is no slope in the data, only single-point ratios (455 and 459 on
        2026-10-09, where the two-point slopes of the same session read 81 and 306)."""
        flat = cs.calibration(series(30, peak=1820, d0=1800), 3.2, None, cs.CURRENT_CAL_DEFAULT,
                              floor_counts=400, ladder_peaks=(1700, 1820))
        self.assertEqual(flat["verdict"], "INCONCLUSIVE")
        self.assertIn("2 points span 120 counts", flat["recommendation"])
        # The braking fallback runs no ladder at all, so it reaches the same verdict by construction.
        none = cs.calibration(series(30, peak=1820, d0=1800), 3.2, None, cs.CURRENT_CAL_DEFAULT,
                              floor_counts=400, ladder_peaks=())
        self.assertEqual(none["verdict"], "INCONCLUSIVE")
        self.assertIn("no ladder was run", none["recommendation"])
        self.assertIsNone(cs.cal_signal_weak(1820, 400, (900, 1820)))

    def test_invalid_still_wins_over_inconclusive(self):
        """A chopped window has no load measurement in it at all, which is a stronger statement than
        a weak one, so it keeps saying INVALID."""
        r = cs.calibration(series(5, peak=1820, d0=1800, chopped=4), 3.2, None, cs.CURRENT_CAL_DEFAULT,
                           floor_counts=2097, ladder_peaks=())
        self.assertEqual(r["verdict"], "INVALID")
        self.assertIn("was chopped", r["recommendation"])

    def test_gate4(self):
        good = series(30, mode=RUN, moe=1, sub=3, speed=40, peak=2200, chopped=40, d0=1800)
        cpa = cs.CURRENT_CAL_DEFAULT
        r = cs.gate4_verdict(good, 2500, cpa, 2.4, locked=False)
        self.assertEqual(r["verdict"], "PASS")
        self.assertIn("raw: within 30%", r["lines"][-1])
        self.assertEqual(cs.gate4_verdict(good, 2500, cpa, 2.4)["verdict"], "FAIL")   # locked: speed must be 0
        locked = series(30, mode=RUN, moe=1, sub=3, peak=2200, chopped=40, d0=1956)
        cal = {"psu_a": 3.2, "duty_on": 1800}
        r = cs.gate4_verdict(locked, 2500, cpa, 2.4, locked=True, cal=cal)
        self.assertEqual(r["verdict"], "PASS")
        self.assertIn("predicts 3.78 A", r["lines"][-1])
        self.assertIn("below: the chop is holding", r["lines"][-1])
        self.assertEqual(
            cs.gate4_verdict(series(30, speed=40, peak=2200), 2500, cpa, 2.4, locked=False)["verdict"], "FAIL")
        for bad in (series(30, speed=0, peak=2200, chopped=40), series(30, speed=4, peak=1200, chopped=40),
                    series(30, speed=4, peak=2200, chopped=40, trips=1)):
            self.assertEqual(cs.gate4_verdict(bad, 2500, cpa, 2.4, locked=False)["verdict"], "FAIL")
        for bad in (series(30, peak=1200, chopped=40), series(30, peak=2200, chopped=40, trips=1),
                    series(30, peak=2200)):
            self.assertEqual(cs.gate4_verdict(bad, 2500, cpa, 2.4, locked=True)["verdict"], "FAIL")

    def _gate5(self, trips_after=1, latch_after=1, fault=0):
        base = mk(mode=RUN, moe=1)
        pre = series(5, mode=RUN, moe=1, sub=3, chopped=64, peak=2600)
        pre.append(mk(t=0.5, mode=OFF, trips=trips_after, shutdowns=1, ev=[0, 0, 0, 0, latch_after, 0, 0, 0],
                      fault=fault))
        post = series(5, t0=0.6, mode=RUN, moe=1, trips=trips_after, shutdowns=1,
                      ev=[0, 0, 0, 0, latch_after + 1, 0, 0, 0], fault=fault)
        return cs.gate5_verdict(pre, post, base)

    def test_gate5(self):
        self.assertEqual(self._gate5()["verdict"], "PASS")
        r = self._gate5(trips_after=2)
        self.assertEqual(r["verdict"], "FAIL")
        self.assertTrue(any(line.startswith("FAIL: trips +1") for line in r["lines"]))
        self.assertEqual(self._gate5(fault=8)["verdict"], "FAIL")
        no_trip = cs.gate5_verdict(series(50, mode=RUN, moe=1, sub=3, chopped=64), [], mk(mode=RUN, moe=1))
        self.assertEqual(no_trip["verdict"], "FAIL")

    def test_rearm(self):
        off = series(3, mode=OFF)
        arm = series(3, t0=0.2, mode=RUN, moe=1)
        self.assertEqual(cs.rearm_verdict(off, arm, 0.0, series(3, mode=RUN, moe=1))["verdict"], "PASS")
        self.assertEqual(cs.rearm_verdict(off, series(3, mode=OFF), 0.0, [])["verdict"], "FAIL")
        self.assertEqual(cs.rearm_verdict(off, arm, 0.0, series(3, speed=5))["verdict"], "FAIL")


GOLDEN = """\
# Current-limit session, 2026-10-09

Tool `tools/climit-session.py` (climit-session/1), board master, attached node 0x02 (operator confirmed).
ELF `target/thumbv7m-none-eabi/release/firmware`, HEAD `abc123`. Evidence CSV `climit-101500.csv`.

## Parameters

| parameter | value |
|---|---|
| staged limit (gates 4, 5) | 2500 mA = 2100 counts, hard trip 4200 counts |
| calibration / plateau demand | 3000 |
| trip demand | 32767 |
| rotor, gates 3 to 5 | locked (strap, or both hands on the tyre) |
| PSU current limit declared | 3.5 A (rule: at least 1.5 A, clearing the 0.60 A the link carries at the chop; never above 6 A) |
| staged limit before the session (0x20) | 10000 mA |
| counts per amp (0x67), read off the board | 455 |
| skipped | gate 2 |

## Outcome

ABORTED: the operator read 3.8 A on the PSU, above the 3.5 A limit they declared

## Gates

### Gate 1, rest floor: INFO

- peak max 1300 counts

## Counts per amp

Not measured this session.

## Operator readings, verbatim

- PSU reading now (A)? -> `3.8`

## Teardown

- neutral: explicit Neutral sent
- lock_release: released claude-climit

## Bench state at close

- rail OFF confirmed
"""


class Record(unittest.TestCase):
    def test_golden(self):
        rec = {
            "date": "2026-10-09",
            "params": {"board": "master", "limit_ma": 2500, "cal_demand": 3000, "trip_demand": 32767,
                       "skip": [2]},
            "node_txt": "0x02 (operator confirmed)", "elf": "target/thumbv7m-none-eabi/release/firmware",
            "head": "abc123", "csv": "climit-101500.csv",
            "outcome": "ABORTED: the operator read 3.8 A on the PSU, above the 3.5 A limit they declared",
            "gates": [{"name": "Gate 1, rest floor", "verdict": "INFO", "lines": ["peak max 1300 counts"]}],
            "typed": [("PSU reading now (A)?", "3.8")],
            "teardown": ["neutral: explicit Neutral sent", "lock_release: released claude-climit"],
            "final": ["rail OFF confirmed"], "calibration": None, "psu_declared": 3.5, "prev_limit_ma": 10000,
            "staged_cpa": 455, "cal_raw": 455,
            "rotor": "locked (strap, or both hands on the tyre)",
        }
        self.assertEqual(cs.render_record(rec), GOLDEN)

    def test_no_em_dash_anywhere(self):
        for name in ("climit-session.py", "swdobs.py", os.path.join("tests", "test_climit_session.py")):
            with open(os.path.join(_TOOLS, name), encoding="utf-8") as fh:
                self.assertNotIn(chr(0x2014), fh.read(), name)


def run_session(answers=None, sim=None, argv=("--limit-ma", "2500"), shell=None):
    args = cs.build_parser().parse_args(list(argv))
    shell = shell or cs.FakeShell(sim=sim, answers=answers, echo=False)
    session = cs.Session(args, shell, io.StringIO(), "/nonexistent", "2026-10-09")
    session.record_path = "/nonexistent/RECORD.md"
    session.run()
    return session, shell


def answering(**overrides):
    """nominal_answers, except a prompt containing a key's text gets that answer."""
    def answer(prompt):
        for key, value in overrides.items():
            if key.replace("_", " ") in prompt:
                return value
        return cs.nominal_answers(prompt)
    return answer


TEARDOWN_ORDER = ["drive_end", "hold_end", "moe_check", "neutral", "rail_off", "rail_verify", "ocd_kill",
                  "tunnel_close", "lock_release"]


def cmd_index(shell, needle, start=0, kinds=("run", "spawn", "ask", "connect")):
    for i, (kind, text) in enumerate(shell.log[start:], start):
        if kind in kinds and needle in text:
            return i
    raise AssertionError(f"{needle!r} not in the shell log after {start}")


class Teardown(unittest.TestCase):
    def test_abort_while_armed(self):
        s, sh = run_session(answers=answering(at_demand="abort"))
        self.assertTrue(s.rec["outcome"].startswith("ABORTED: the operator typed abort"))
        self.assertEqual(s.teardown_log, TEARDOWN_ORDER)
        self.assertEqual(list(cs.TEARDOWN_ORDER), TEARDOWN_ORDER)
        self.assertIn("moe_check: moe_bits 0, mode_byte OFF", s.rec["teardown"])
        self.assertIn("drive_end: demand released (Neutral; the firmware zeroes it in 200 ms)", s.rec["teardown"])
        # Drive end, inputs end, then the Neutral, rail off, verify, OpenOCD kill, lock release.
        start = cmd_index(sh, "at demand")
        d = cmd_index(sh, "> neutral (the owner releases", start, kinds=("say",))
        h = cmd_index(sh, "(end the session:", d, kinds=("say",))
        i = cmd_index(sh, "--value 0 --hold 1", h)
        j = cmd_index(sh, "pinctrl set 4 op dh", i)
        k = cmd_index(sh, "pinctrl get 4", j)
        m = cmd_index(sh, "sudo pkill -x openocd", k)
        cmd_index(sh, "bench-lock.sh release", m)
        self.assertIsNone(s.inputs)
        self.assertIsNone(s.drive)

    def test_moe_check_failed(self):
        s, sh = run_session(answers=answering(at_demand="abort"), sim=cs.SimBoard(stuck_moe=True))
        self.assertEqual(s.teardown_log, TEARDOWN_ORDER)
        moe = [t for t in s.rec["teardown"] if t.startswith("moe_check")][0]
        self.assertIn("FAILED", moe)
        self.assertIn("rail OFF immediately", moe)
        # Rail off is the very next command after the moe-check reads; the Neutral is skipped.
        after = [text for kind, text in sh.log[cmd_index(sh, "at demand"):] if kind == "run"]
        self.assertIn("pinctrl set 4 op dh", after[0])
        self.assertIn("neutral: skipped: the moe check did not pass, the rail goes off first", s.rec["teardown"])
        self.assertTrue(any("moe check FAILED" in f for f in s.rec["final"]))

    def test_abort_before_the_mailbox(self):
        s, sh = run_session(answers=answering(attached_node="n"))
        self.assertIn("did not confirm attached node 0x02", s.rec["outcome"])
        self.assertEqual(s.teardown_log, TEARDOWN_ORDER)
        self.assertTrue(s.rec["teardown"][3].startswith("neutral: skipped"))
        self.assertFalse(any("swd-mailbox-drive" in t for _k, t in sh.log))

    def test_psu_above_declared_aborts(self):
        s, _ = run_session(answers=answering(at_demand_3000="3.6"))
        self.assertIn("above the 3.5 A limit they declared", s.rec["outcome"])
        self.assertEqual(s.teardown_log, TEARDOWN_ORDER)

    def test_psu_limit_off_rule_aborts_before_the_rail(self):
        # 7 A is above the bench cap; the band itself (a floor that clears the link current, up to
        # 6 A) is the operator's to choose inside.
        s, sh = run_session(answers=answering(Set_the_PSU="7"))
        self.assertIn("outside the rule", s.rec["outcome"])
        self.assertFalse(any("pinctrl set 4 op dl" in t for _k, t in sh.log))
        self.assertIn("rail_off: skipped: this run never touched the rail", s.rec["teardown"])

    def test_wrong_board_aborts(self):
        sim = cs.SimBoard()
        orig = sim.spawn

        def spawn(argv, tag):
            if any("swd-mailbox" in a for a in argv):
                sim.node = 0x01       # the first hold's walk lands on another board
            return orig(argv, tag)
        sim.spawn = spawn
        s, _ = run_session(sim=sim)
        self.assertIn("not the confirmed 0x02", s.rec["outcome"])

    def test_ladder_steps_until_the_estimate_band(self):
        s, sh = run_session()
        demands = [int(t.split("at demand ")[1].split("?")[0]) for k, t in sh.log if k == "ask" and "at demand" in t]
        self.assertEqual(demands, [3000, 4000, 5000, 6000, 7000, 8000])        # until the estimate reaches 4 A
        self.assertEqual(s.cal_final_demand, 8000)
        # 4.4 A of simulated phase current reads 2009 counts against a 1220-count rest floor, which
        # is under the 2x the spec requires before a ratio means anything: the nominal board's own
        # scale cannot be confirmed at the current this ladder reaches, only bounded above.
        self.assertEqual(s.cal["verdict"], "INCONCLUSIVE")
        self.assertIn("UPPER BOUND", s.cal["recommendation"])
        self.assertIn("demand 10000 (to the session owner", " ".join(t for _k, t in sh.log))  # gate 4: final + 2000
        prompt = [t for k, t in sh.log if k == "ask" and "at demand" in t][0]
        self.assertIn("about 1 A", prompt)
        self.assertIn("locked", s.rec["rotor"])

    def test_ladder_backs_off_from_8a(self):
        def answers(prompt):
            if "at demand 3000" in prompt:
                return "3.0"      # 3.0 * 2250 / 179 = 37.7 A: far over the band
            return cs.nominal_answers(prompt)
        s, sh = run_session(answers=answers)
        asks = [t for k, t in sh.log if k == "ask" and "at demand" in t]
        self.assertTrue(asks[1].startswith("PSU reading (A) at demand 2500?"))
        self.assertIn("one step straddles the band", s.rec["outcome"])

    def test_ladder_aborts_at_12000(self):
        def answers(prompt):
            return "0.1" if "at demand" in prompt else cs.nominal_answers(prompt)
        s, sh = run_session(answers=answers)
        asks = [t for k, t in sh.log if k == "ask" and "at demand" in t]
        self.assertTrue(asks[-1].startswith("PSU reading (A) at demand 12000?"))
        self.assertEqual(len(asks), 10)
        self.assertIn("draws less than expected; check the lock and the phase wiring", s.rec["outcome"])
        self.assertEqual(s.teardown_log, TEARDOWN_ORDER)

    def test_unlocked_rotor_is_released_and_reprompted(self):
        sim = cs.SimBoard()
        sim.locked = False

        def answers(prompt):
            if "the rotor is not locked" in prompt:
                sim.locked = True
            return cs.nominal_answers(prompt)
        s, sh = run_session(sim=sim, answers=answers)
        asks = [t for k, t in sh.log if k == "ask"]
        relock = [a for a in asks if "motor_speed read 90: the rotor is not locked" in a]
        self.assertEqual(len(relock), 1)
        self.assertTrue(s.rec["outcome"].startswith("COMPLETED"), s.rec["outcome"])

    def test_never_locked_aborts(self):
        sim = cs.SimBoard()
        sim.locked = False
        s, _ = run_session(sim=sim)
        self.assertIn("the rotor is not locked", s.rec["outcome"])
        self.assertTrue(s.rec["outcome"].startswith("ABORTED"))

    def test_brake_fallback_completes(self):
        sim = cs.SimBoard()
        sim.locked = False
        s, sh = run_session(sim=sim, argv=("--brake-fallback",))
        self.assertTrue(s.rec["outcome"].startswith("COMPLETED"), s.rec["outcome"])
        self.assertIn("braking fallback", s.rec["rotor"])
        self.assertIn("--brake-fallback", cs.render_record(s.rec))
        self.assertFalse(any("at demand" in t for _k, t in sh.log))

    def test_teardown_idempotent(self):
        s, sh = run_session()
        n = len(sh.log)
        s.close()
        s.teardown()
        self.assertEqual(len(sh.log), n)

    def test_skipped_gates_use_the_floor_fallback(self):
        s, _ = run_session(argv=("--skip-gate", "1", "--skip-gate", "2", "--skip-gate", "3"))
        self.assertTrue(s.rec["outcome"].startswith("COMPLETED"))
        names = [(g["name"], g["verdict"]) for g in s.rec["gates"]]
        self.assertEqual(names[:3], [("Gate 1", "SKIPPED"), ("Gate 2", "SKIPPED"), ("Gate 3", "SKIPPED")])
        self.assertIsNone(s.g1)


class AuditFixes(unittest.TestCase):
    """The 2026-10-08 audit round: teardown robustness, early ends, re-applied demands, arm checks."""

    def test_flush_failure_does_not_skip_the_teardown(self):
        with mock.patch.object(cs.Session, "flush_record", side_effect=OSError("No space left on device")):
            s, sh = run_session(answers=answering(at_demand="abort"))
        self.assertEqual(s.teardown_log, TEARDOWN_ORDER)
        self.assertTrue(any("record flush before teardown FAILED" in f for f in s.rec["final"]))
        self.assertIsNone(s.inputs)
        cmd_index(sh, "bench-lock.sh release")

    def test_signal_between_teardown_steps(self):
        class Interrupting(cs.FakeShell):
            fired = False

            def say(self, text):
                super().say(text)
                if text.startswith("   teardown hold_end") and not self.fired:
                    self.fired = True
                    raise KeyboardInterrupt
        sh = Interrupting(answers=answering(at_demand="abort"), echo=False)
        s, _ = run_session(shell=sh)
        self.assertTrue(sh.fired)
        self.assertEqual(s.teardown_log, TEARDOWN_ORDER)
        cmd_index(sh, "bench-lock.sh release")
        self.assertIn(("signals", "deferred"), sh.log)

    def test_signals_are_deferred_for_real(self):
        before = signal.getsignal(signal.SIGINT)
        with cs.DeferredSignals() as d:
            os.kill(os.getpid(), signal.SIGINT)
            time.sleep(0.05)
        self.assertEqual(d.pending, [signal.SIGINT])
        self.assertIs(signal.getsignal(signal.SIGINT), before)

    def _sim_wrapping_fields(self, change):
        sim = cs.SimBoard()
        orig = sim.fields

        def fields():
            f = orig()
            change(sim, f)
            return f
        sim.fields = fields
        return sim

    def test_gate1_fail_ends_before_any_arm(self):
        sim = self._sim_wrapping_fields(lambda sim, f: f.update(chopped=1) if f.get("mode", 0) == OFF else None)
        s, sh = run_session(sim=sim)
        self.assertIn("ENDED EARLY", s.rec["outcome"])
        self.assertIn("gate 1 failed", s.rec["outcome"])
        self.assertFalse(any("--buttons 1" in t for k, t in sh.log if k == "spawn"))

    def test_gate2_fail_ends_before_any_arm(self):
        def change(sim, f):
            if sim.drive is not None and sim.drive.alive() and not sim._armed():
                f.update(peak=5000)
        s, sh = run_session(sim=self._sim_wrapping_fields(change))
        self.assertIn("gate 2 failed", s.rec["outcome"])
        self.assertFalse(any("--buttons 1" in t for k, t in sh.log if k == "spawn"))

    def test_one_owner_holds_the_demand_across_a_gate_with_no_reapply(self):
        """What the 60 s drive hold used to force: a renewal mid-gate, announced to the operator
        because their hands were on the wheel. One session owner holds the demand for as long as it
        holds the arm, so there is nothing to renew and nothing to announce. The property under test
        is that no SECOND mailbox producer is ever spawned while a demand is live, which is what
        aborted the real gates on 2026-10-08."""
        s, sh = run_session()
        spawns = [text for kind, text in sh.log if kind == "spawn" and "swd-mailbox" in text]
        # One owner per arm (gate 2 unarmed, gate 3/4 armed, gate 5's re-arm), never two at once,
        # and never the standalone drive tool while an owner is running.
        self.assertTrue(all("swd-mailbox-session" in x for x in spawns), spawns)
        self.assertNotIn("The demand is going to be re-applied",
                         " ".join(text for kind, text in sh.log if kind == "ask"))
        self.assertNotIn("renewing the drive hold", " ".join(text for _k, text in sh.log))
        self.assertTrue(s.rec["outcome"].startswith("COMPLETED"), s.rec["outcome"])

    def test_inputs_node_mismatch_ends_the_hold_at_once(self):
        sim = cs.SimBoard()
        orig = sim.spawn
        seen = {}

        def spawn(argv, tag):
            if any("swd-mailbox-session" in a for a in argv):
                sim.node = 0x01
            c = orig(argv, tag)
            seen.setdefault("owner", c) if "session owner" in tag else None
            return c
        sim.spawn = spawn
        real_teardown = cs.Session.teardown
        at_teardown = {}

        def teardown(self):
            at_teardown["ended"] = seen["owner"].ended
            return real_teardown(self)
        with mock.patch.object(cs.Session, "teardown", teardown):
            s, _ = run_session(sim=sim)
        self.assertIn("swd-mailbox-session resolved attached node 0x01, not the confirmed 0x02", s.rec["outcome"])
        self.assertTrue(at_teardown["ended"])

    def test_arm_expiry_mid_ladder_aborts_as_such(self):
        sim = cs.SimBoard()

        def answers(prompt):
            if "at demand 4000" in prompt:
                sim.inputs.ended = True    # the hold dies while the operator reads the meter
            return cs.nominal_answers(prompt)
        s, _ = run_session(sim=sim, answers=answers)
        self.assertIn("ABORTED: the arm expired", s.rec["outcome"])
        self.assertEqual(s.teardown_log, TEARDOWN_ORDER)

    def test_owner_lost_before_the_trip_demand_is_an_expired_arm(self):
        """The hazard the trip gate has to tell apart: the arm going away BEFORE the trip demand is
        an expired arm, not a firmware that failed to trip. There is no drive-tool walk to lose it
        in any more, so the surviving window is between the arm check and the demand."""
        sim = cs.SimBoard()

        def answers(prompt):
            if "keep the rotor locked until told to release" in prompt:
                sim.inputs.ended = True      # the owner dies as the operator confirms
            return cs.nominal_answers(prompt)
        s, _ = run_session(sim=sim, answers=answers)
        self.assertIn("ABORTED: the arm expired", s.rec["outcome"])
        self.assertFalse(any(g["name"] == "Gate 5, the trip" for g in s.rec["gates"]))
        self.assertEqual(s.teardown_log, TEARDOWN_ORDER)

    def test_limit_written_is_recorded_when_the_read_back_fails(self):
        sim = cs.SimBoard()
        orig = sim._config

        def config(cmd):
            r = orig(cmd)
            if "=" not in cmd.split("swd-mailbox-config", 1)[1] and sim.store_limit == 2500:
                r.stdout = r.stdout.replace("U32(2500)", "U32(1234)")
            return r
        sim._config = config
        s, _ = run_session(sim=sim)
        self.assertIn("reads back 1234 after the power cycle", s.rec["outcome"])
        self.assertTrue(any(f.startswith("limit left staged at 2500 mA") for f in s.rec["final"]))
        self.assertFalse(any(g["name"] == "Stage the limit" for g in s.rec["gates"]))

    def test_chopped_ladder_window_stops_the_ladder(self):
        sim = cs.SimBoard()
        # A board whose sense chain reads 800 counts per amp, with the limit at the count floor
        # (any label under 2,625 mA floors to MIN_LIMIT_COUNTS at that scale). The chop then starts
        # at 2,100 / 800 = 2.63 A, which the ladder reaches part way up, so the window chops and the
        # calibration is INVALID. This also exercises the path the field exists for: the tool reads
        # the board's own scale rather than assuming one.
        sim.store_cal = 800
        sim.store_limit = 2500
        # Decline the offer to raise it: a chopped ladder window is the whole scenario here.
        s, sh = run_session(sim=sim, answers=answering(Raise_it_to="n"))
        asks = [t for k, t in sh.log if k == "ask" and "at demand" in t]
        self.assertEqual([int(a.split("at demand ")[1].split("?")[0]) for a in asks], [3000, 4000])
        self.assertEqual(s.rec["calibration"]["verdict"], "INVALID")
        self.assertIn("was chopped", s.rec["calibration"]["recommendation"])
        self.assertEqual(s.cal_final_demand, 5000)
        self.assertTrue(any("below the 8 A" in w for w in s.rec["warnings"]))
        self.assertIn("## Warnings", cs.render_record(s.rec))
        self.assertTrue(s.rec["outcome"].startswith("COMPLETED"), s.rec["outcome"])

    def test_each_ladder_step_settles_first(self):
        s, _ = run_session()
        csv = s.csv.fh.getvalue()
        for d in (3000, 8000):
            self.assertLess(csv.index(f"step gate3-settle-{d} begin"), csv.index(f"step gate3-step-{d} begin"))

    def test_dry_run_brake_fallback_uses_the_fallback(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = cs.main(["--dry-run", "--brake-fallback"])
        self.assertEqual(rc, 0, out.getvalue()[-1500:])
        self.assertIn("Brake the tyre by hand", out.getvalue())
        self.assertIn("braking fallback (--brake-fallback)", out.getvalue())


class DryRun(unittest.TestCase):
    def test_end_to_end_no_side_effects(self):
        boom = mock.Mock(side_effect=AssertionError("the dry run touched the system"))
        real_open = open

        def read_only_open(path, mode="r", *a, **kw):
            if any(c in mode for c in "wax+"):
                raise AssertionError(f"the dry run opened {path} for writing")
            return real_open(path, mode, *a, **kw)
        out = io.StringIO()
        shell = cs.FakeShell(echo=True, exists=lambda p: False)
        with mock.patch.object(subprocess, "run", boom), mock.patch.object(subprocess, "Popen", boom), \
                mock.patch("socket.create_connection", boom), mock.patch("builtins.open", read_only_open), \
                mock.patch("os.makedirs", boom), contextlib.redirect_stdout(out):
            rc = cs.main(["--dry-run", "--limit-ma", "2500"], shell=shell)
        self.assertEqual(rc, 0, out.getvalue()[-2000:])
        text = out.getvalue()
        self.assertIn("COMPLETED", text)
        self.assertIn("== dry run: RECORD.md would read ==", text)
        # Every command in order.
        order = ["cargo build", "bench-lock.sh acquire", "pinctrl set 4 op dl", "nohup sudo openocd",
                 "ssh -N -o ExitOnForwardFailure=yes", "swd-mailbox-config 127.0.0.1:6666 --dst attached 0x20",
                 "--buttons 0 --rider 0 --hold 600", "--buttons 1 --rider 1 --hold 600",
                 "0x20=2500", "pinctrl set 4 op dh", "--value 0 --hold 1",
                 "pinctrl get 4", "sudo pkill -x openocd", "bench-lock.sh release"]
        i = 0
        for needle in order:
            i = cmd_index(shell, needle, i)
        for g in ("Gate 1, rest floor: INFO", "Gate 2, demand without arm: PASS",
                  "Gate 3, calibration: INCONCLUSIVE",
                  "Gate 4, the plateau: PASS", "Gate 5, the trip: PASS"):
            self.assertIn(g, text)

    def test_prompts(self):
        _, sh = run_session()
        asks = [t for k, t in sh.log if k == "ask"]
        self.assertGreaterEqual(len(asks), 12)
        words = ("hold of the rotor", "keep the rotor locked", "not locked", "rotor moved", "Brake",
                 "stalled", "to arm", "to re-arm")
        energised = [a for a in asks if any(w in a for w in words)]
        self.assertTrue(any("take hold of the rotor" in a for a in energised))
        # Every lock prompt promises hands-on time, and the countdown is what delivers it.
        for a in energised:
            if "rotor" in a:
                self.assertIn(cs.HANDS_ON, a, a)
        for a in energised:
            self.assertTrue(a.startswith("Hand on the kill."), a)
        self.assertEqual(sum("You can release the rotor" in a for a in asks), 1)   # gate 4 stays armed for 5
        free = cs.SimBoard()
        free.locked = False
        _, sh = run_session(sim=free, argv=("--brake-fallback",))
        asks = [t for k, t in sh.log if k == "ask"]
        self.assertTrue(any(a.startswith("Hand on the kill. Brake the tyre") for a in asks))
        for a in asks:
            for word in ("ssh", "pinctrl", "swd-mailbox", "cargo", "openocd", "tools/"):
                self.assertNotIn(word, a)

    def test_refusals(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.assertEqual(cs.main(["--dry-run", "--limit-ma", "1999"]), 2)
            self.assertEqual(cs.main(["--dry-run", "--limit-ma", "6500"]), 2)
            self.assertEqual(cs.main(["--dry-run", "--cal-demand", "500"]), 2)
        # The refusal says why in the domain the limit actually lives in: counts, through the
        # board's own scale. It must not quote a milliamp-to-count equivalence of its own.
        self.assertIn("the counts a milliamp buys are the board's own scale (0x67)", err.getvalue())
        self.assertIn(f"floors the limit at {cs.MIN_LIMIT_COUNTS} counts", err.getvalue())


class Watchdog(unittest.TestCase):
    def test_child_ends_when_the_hold_ends(self):
        c = cs.RealChild(["sleep", "30"], "test")
        self.assertTrue(c.alive())
        t0 = time.time()
        c.end()
        self.assertFalse(c.alive())
        self.assertLess(time.time() - t0, 6.0)

    def test_child_dies_with_its_parent(self):
        parent = subprocess.Popen(
            [sys.executable, "-c",
             "import importlib.util, sys, time\n"
             f"spec = importlib.util.spec_from_file_location('c', {os.path.join(_TOOLS, 'climit-session.py')!r})\n"
             "m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)\n"
             "c = m.RealChild(['sleep', '60'], 't')\n"
             "print(c.proc.pid, flush=True)\n"
             "time.sleep(60)\n"],
            stdout=subprocess.PIPE, text=True)
        try:
            watchdog = int(parent.stdout.readline())
            grand = None
            for _ in range(50):
                out = subprocess.run(["pgrep", "-P", str(watchdog)], capture_output=True, text=True).stdout.split()
                if out:
                    grand = int(out[0])
                    break
                time.sleep(0.1)
            self.assertIsNotNone(grand, "the watchdog never started its child")
            parent.kill()                  # kill -9: the tool runs no code at all
            parent.wait()
            deadline = time.time() + 8
            while time.time() < deadline:
                try:
                    os.kill(grand, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.1)
            else:
                os.kill(grand, signal.SIGKILL)
                self.fail("the held child outlived its parent")
        finally:
            if parent.poll() is None:
                parent.kill()
            parent.stdout.close()

    def test_child_output_is_captured(self):
        c = cs.RealChild([sys.executable, "-c", "print('dst resolved: attached node 0x02 (x)')"], "test")
        c.proc.wait(timeout=10)
        c.end()
        self.assertEqual(cs.parse_dst("\n".join(c.lines)), 2)


if __name__ == "__main__":
    unittest.main()
