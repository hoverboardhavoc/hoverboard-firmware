#!/usr/bin/env python3
"""Unit tests for tools/lqr-gains.py: the numbers of specs/state-feedback.md section 7.

Run: python3 -m unittest discover -s tools/tests -p 'test_lqr_gains.py'
 or: tools/lqr-gains.py --selftest

The model, K, the poles and the integer gains to 1 % of the spec; the simulations against the spec's
figures at the tolerance stated on each (the spec's simulation is not recorded, and this one's stop
time and held-push travel differ from it, see the per-test notes); the Riccati solve against its own
equation. numpy only, nothing written.
"""

import importlib.util
import os
import unittest

import numpy as np

_TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location("lqr_gains", os.path.join(_TOOLS, "lqr-gains.py"))
lg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lg)


def model(*argv):
    return lg.Model(lg.build_parser().parse_args(list(argv)))


def close(test, got, want, rel, what):
    test.assertLessEqual(abs(got - want), rel * abs(want), f"{what}: {got} against {want} (+-{rel:.0%})")


class TheModel(unittest.TestCase):
    def test_the_section_7_coefficients(self):
        m = model()
        close(self, m.a_, 28.87, 0.01, "A_")
        close(self, m.b_, 3.413, 0.01, "B_")
        close(self, m.d_, 33.49, 0.01, "D_")
        close(self, m.det, 38.58, 0.01, "det")
        close(self, m.A[1, 0], 25.06, 0.01, "theta'' per theta")
        close(self, m.B[1, 0], -1.821, 0.01, "theta'' per tau")
        close(self, m.A[3, 0], -2.963, 0.01, "x'' per theta")
        close(self, m.B[3, 0], 0.635, 0.01, "x'' per tau")

    def test_open_loop_poles_are_0_0_and_plus_minus_5(self):
        p = sorted(np.linalg.eigvals(model().A).real)
        close(self, p[0], -5.0, 0.01, "stable tipping pole")
        close(self, p[3], 5.0, 0.01, "unstable tipping pole")
        self.assertAlmostEqual(p[1], 0.0, places=9)
        self.assertAlmostEqual(p[2], 0.0, places=9)

    def test_the_unit_chain(self):
        m = model()
        close(self, m.edge, 5.76e-3, 0.01, "edge")
        close(self, m.count, 0.288, 0.01, "count")
        close(self, m.c, 7.027e-4, 0.001, "N.m per numerator count")


class TheDesign(unittest.TestCase):
    def test_care_solves_the_riccati_equation(self):
        m = model()
        q, r = np.diag(lg.DEFAULT_Q), np.array([[lg.DEFAULT_R]])
        k = lg.care(m.A, m.B, q, r)
        # The stabilizing X again, and the residual of the equation itself.
        n = 4
        h = np.block([[m.A, -m.B @ np.linalg.inv(r) @ m.B.T], [-q, -m.A.T]])
        w, v = np.linalg.eig(h)
        s = v[:, w.real < 0]
        x = np.real(s[n:] @ np.linalg.inv(s[:n]))
        res = m.A.T @ x + x @ m.A - x @ m.B @ np.linalg.inv(r) @ m.B.T @ x + q
        self.assertLess(np.abs(res).max(), 1e-8)
        np.testing.assert_allclose(k, np.linalg.inv(r) @ m.B.T @ x)

    def test_the_start_design(self):
        k, poles = lg.design(model(), lg.DEFAULT_Q, lg.DEFAULT_R)
        for got, want in zip(np.asarray(k).ravel(), (-140.5, -28.1, -24.0, -36.2)):
            close(self, got, want, 0.01, "K")
        re = sorted(p.real for p in poles)
        close(self, re[0], -16.5, 0.01, "fast pendulum pole")
        close(self, re[1], -9.6, 0.01, "slow pendulum pole")
        close(self, re[2], -1.07, 0.01, "body pair, real")
        close(self, max(p.imag for p in poles), 0.67, 0.01, "body pair, imaginary")

    def test_the_start_gains(self):
        m = model()
        k, _ = lg.design(m, lg.DEFAULT_Q, lg.DEFAULT_R)
        for got, want, name in zip(m.int_gains(k), (3491, 4002, 197, 926), ("gp", "gr", "gx", "gv")):
            close(self, got, want, 0.01, name)

    def test_the_neighbours(self):
        m = model()
        for q, r, ks, gains in (
            ((100, 1, 4, 1), 1 / 64, (-124.5, -23.3, -16.0, -26.0), (3092, 3316, 131, 666)),
            ((100, 1, 16, 8), 1 / 100, (-190.1, -39.6, -40.0, -56.9), (4721, 5640, 328, 1457)),
        ):
            k, _ = lg.design(m, q, r)
            for got, want in zip(np.asarray(k).ravel(), ks):
                close(self, got, want, 0.01, f"K at Q {q}")
            for got, want in zip(m.int_gains(k), gains):
                close(self, got, want, 0.01, f"gains at Q {q}")

    def test_the_torque_at_the_position_clamp(self):
        m = model()
        k, _ = lg.design(m, lg.DEFAULT_Q, lg.DEFAULT_R)
        close(self, abs(np.asarray(k).ravel()[2]) * lg.POS_CLAMP * m.edge, 6.2, 0.01, "clamp torque")

    def test_a_model_flag_moves_the_design(self):
        base = model().int_gains(lg.design(model(), lg.DEFAULT_Q, lg.DEFAULT_R)[0])
        heavy = model("--body-mass", "30")
        moved = heavy.int_gains(lg.design(heavy, lg.DEFAULT_Q, lg.DEFAULT_R)[0])
        self.assertGreater(heavy.d_, model().d_ * 1.4, "the flag reaches the model")
        self.assertGreater(abs(moved[0] - base[0]), 5, "and the design")


class TheSimulations(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = model()
        cls.k, _ = lg.design(cls.m, lg.DEFAULT_Q, lg.DEFAULT_R)

    def test_a_one_count_push(self):
        p = lg.push(self.m, self.k, 0.3)
        close(self, p["peak_torque"], 10.9, 0.01, "peak torque")
        close(self, p["peak_pitch_deg"], 4.9, 0.02, "peak pitch")
        close(self, p["travel_m"], 0.14, 0.03, "travel")
        # The spec reads 0.72 s; this simulation's first zero crossing of the speed is 0.744 s.
        close(self, p["stop_s"], 0.72, 0.04, "stop")
        self.assertLess(p["stop_s"], 1.0, "the 1 s stop target")

    def test_a_two_count_push_saturates_and_still_stops(self):
        p = lg.push(self.m, self.k, 0.6)
        self.assertAlmostEqual(p["peak_torque"], self.m.tau_max)
        close(self, p["peak_pitch_deg"], 9.8, 0.02, "peak pitch")
        close(self, p["stop_s"], 0.77, 0.04, "stop")

    def test_a_held_push_returns_without_overshoot(self):
        h = lg.held_push(self.m, self.k)
        # The spec reads 0.25 m at release; a 20 N force on the axle moves this model 0.18 m in 2 s
        # (the steady offset is F r (1 + K1 / D) / K3, 0.22 m). Both are inside the 0.26 m clamp,
        # which is what the figure is quoted against.
        self.assertGreater(h["at_release_m"], 0.15)
        self.assertLess(h["at_release_m"], 45 * self.m.edge, "inside the position clamp")
        self.assertLess(h["overshoot_m"], 0.01, "no overshoot past the start")
        self.assertLess(abs(h["final_m"]), 0.01, "back within 1 cm")

    def test_a_stick_step_reaches_90_percent_inside_the_slew_plus_one_second(self):
        s = lg.stick_step(self.m, self.k)
        close(self, s["slew_s"], 2.0, 0.01, "slew time at 80 / 41")
        self.assertLess(s["t90_s"], s["slew_s"] + 1.0)


class TheCommandLine(unittest.TestCase):
    def test_q_parsing(self):
        self.assertEqual(lg.parse_q("100,1,9,4"), (100.0, 1.0, 9.0, 4.0))
        with self.assertRaises(ValueError):
            lg.parse_q("1,2,3")

    def test_the_report_names_the_gains(self):
        text = lg.report(model(), lg.DEFAULT_Q, lg.DEFAULT_R)
        self.assertIn("CONTROL_SF_GAINS [gp, gr, gx, gv] = [3491, 4002, 197, 927]", text)
        self.assertIn("closed-loop poles", text)


if __name__ == "__main__":
    unittest.main()
