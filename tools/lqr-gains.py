#!/usr/bin/env python3
"""LQR starting gains for the four-term state-feedback balance law (specs/state-feedback.md, section 7).

Usage: tools/lqr-gains.py [model flags] [--q Q1,Q2,Q3,Q4] [--r R] [--selftest]

    --selftest  run tools/tests/test_lqr_gains.py (the section-7 numbers to 1 %), nothing written.

Linearizes the rover (body pitch theta and axle position x, both forward positive, the physics
convention; the firmware's signs are section 2's) about upright, solves the continuous algebraic
Riccati equation by the Hamiltonian's stable eigenvectors (numpy only, scipy is not installed on the
Mac), and prints the A and B matrices, the open- and closed-loop poles, K in SI, the four integer gains
of CONTROL_SF_GAINS (0x79) in the firmware's units, the torque at the position clamp and at one count
of speed error, and three simulations at the control rate with the torque clamped and the sensor
delayed: a push, a held push released, and a full-stick step.

The model numbers are flags with the rover's values as defaults. It writes nothing into the tree: the
gains are staged from the app and baked into the spec when the bench confirms them.
"""

import argparse
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))

G = 9.81
# The control rate and the firmware's speed unit: one COUNT is one hall edge per 320 periods of the
# 16 kHz loop, 50 edges per second (specs/control.md (o)).
TICK_HZ = 250
EDGES_PER_S_PER_COUNT = 50.0
# The section-4 reference row and clamp the stick-step simulation runs at (section 7's start row).
SPEED_MAX = 80  # sixteenths of a count at full stick
SPEED_SLEW = 41  # 1/4096 count per tick
POS_CLAMP = 45  # edges

DEFAULT_Q = (100.0, 1.0, 9.0, 4.0)
DEFAULT_R = 1.0 / 64.0


def build_parser():
    ap = argparse.ArgumentParser(prog="lqr-gains.py", description=__doc__.split("\n\n")[0])
    ap.add_argument("--body-mass", type=float, default=20.98, help="body with pole, kg")
    ap.add_argument("--com", type=float, default=0.1627, help="body centre of mass above the axle, m")
    ap.add_argument("--inertia-axle", type=float, default=1.74, help="body inertia about the axle, kg.m2")
    ap.add_argument("--wheel-mass", type=float, default=5.54, help="wheels and axle blocks, total, kg")
    ap.add_argument("--wheel-inertia", type=float, default=0.008, help="spinning rotor and tyre, per wheel, kg.m2")
    ap.add_argument("--radius", type=float, default=0.0825, help="wheel radius, m")
    ap.add_argument("--edges-per-rev", type=float, default=90.0, help="hall edges per wheel turn")
    ap.add_argument("--torque-full-scale", type=float, default=15.2, help="total torque at the full-scale word, N.m")
    ap.add_argument("--word-full-scale", type=float, default=28500.0, help="the output clamp, word counts")
    ap.add_argument("--nominal-battery", type=float, default=2960.0, help="battery word the torque scale is quoted at, cV")
    ap.add_argument("--delay-ms", type=float, default=8.0, help="sensor delay in the simulations, ms")
    ap.add_argument("--q", default=",".join(f"{v:g}" for v in DEFAULT_Q), help="Q diagonal: theta, theta', x, x'")
    ap.add_argument("--r", type=float, default=DEFAULT_R, help="R (torque weight)")
    ap.add_argument("--selftest", action="store_true")
    return ap


class Model:
    """The linearized rover: `A_ x'' + B_ th'' = tau / r`, `B_ x'' + I_b th'' - D_ th = -tau + F h`."""

    def __init__(self, a):
        self.a = a
        self.r = a.radius
        self.ib = a.inertia_axle
        self.a_ = a.body_mass + a.wheel_mass + 2.0 * a.wheel_inertia / a.radius**2
        self.b_ = a.body_mass * a.com
        self.d_ = a.body_mass * G * a.com
        self.det = self.a_ * self.ib - self.b_**2
        self.mass = np.array([[self.a_, self.b_], [self.b_, self.ib]])
        a_tt = self.a_ * self.d_ / self.det
        b_t = -(self.a_ + self.b_ / self.r) / self.det
        a_xt = -self.b_ * self.d_ / self.det
        b_x = (self.ib / self.r + self.b_) / self.det
        # z = [theta, theta', x, x']
        self.A = np.array([[0, 1, 0, 0], [a_tt, 0, 0, 0], [0, 0, 0, 1], [a_xt, 0, 0, 0]], float)
        self.B = np.array([[0], [b_t], [0], [b_x]], float)
        # Units of the firmware's integer gains.
        self.edge = 2.0 * math.pi * a.radius / a.edges_per_rev
        self.count = EDGES_PER_S_PER_COUNT * self.edge
        self.c = a.torque_full_scale / a.word_full_scale * 3900.0 / a.nominal_battery
        self.tau_max = a.torque_full_scale

    def deriv(self, z, tau, force=0.0):
        """The nonlinear-free rigid model the gains were designed on, with a horizontal force at the axle."""
        acc = np.linalg.solve(self.mass, np.array([tau / self.r + force, self.d_ * z[0] - tau]))
        return np.array([z[1], acc[1], z[3], acc[0]])

    def int_gains(self, k):
        """K (u = -K z, SI, physics signs) as the four positive firmware gains [gp, gr, gx, gv]."""
        k = -np.asarray(k).ravel()
        return [
            k[0] * (math.pi / 18000.0) / self.c * 100.0,
            k[1] * 1e-4 / self.c * 1000.0,
            k[2] * self.edge / self.c,
            k[3] * (self.count / 4096.0) / self.c * 256.0,
        ]


def care(A, B, Q, R):
    """The stabilizing solution of `A'X + XA - XBR^-1B'X + Q = 0` from the Hamiltonian's stable eigenvectors."""
    n = A.shape[0]
    ri = np.linalg.inv(R)
    h = np.block([[A, -B @ ri @ B.T], [-Q, -A.T]])
    w, v = np.linalg.eig(h)
    s = v[:, w.real < 0]
    if s.shape[1] != n:
        raise ValueError("the Hamiltonian has no n-dimensional stable subspace")
    x = np.real(s[n:] @ np.linalg.inv(s[:n]))
    return ri @ B.T @ x


def design(m, q, r):
    k = care(m.A, m.B, np.diag(q), np.array([[r]]))
    return k, np.linalg.eigvals(m.A - m.B @ k)


def simulate(m, k, z0, seconds, force=None, reference=None, sub=20):
    """250 Hz, the torque clamped at the full scale, the measurement `delay_ms` old, RK4 inside a tick.

    `reference(tick)` returns (x_ref, v_ref) the law tracks (section 4's position error clamped at
    POS_CLAMP edges); `force(t)` a horizontal force on the axle, N. Returns rows
    (t, tau, theta, theta', x, x', v_ref).
    """
    dt = 1.0 / TICK_HZ
    delay = int(round(m.a.delay_ms * 1e-3 * TICK_HZ))
    z = np.array(z0, float)
    hist = [z.copy()] * (delay + 1)
    rows = []
    kk = np.asarray(k).ravel()
    for i in range(int(round(seconds * TICK_HZ))):
        zm = hist[-1 - delay]
        x_ref, v_ref = reference(i) if reference else (0.0, 0.0)
        ex = max(-POS_CLAMP * m.edge, min(POS_CLAMP * m.edge, x_ref - zm[2]))
        err = np.array([zm[0], zm[1], -ex, zm[3] - v_ref])
        tau = float(np.clip(-(kk @ err), -m.tau_max, m.tau_max))
        f = force(i * dt) if force else 0.0
        h = dt / sub
        for _ in range(sub):
            k1 = m.deriv(z, tau, f)
            k2 = m.deriv(z + h / 2 * k1, tau, f)
            k3 = m.deriv(z + h / 2 * k2, tau, f)
            k4 = m.deriv(z + h * k3, tau, f)
            z = z + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        hist.append(z.copy())
        rows.append((i * dt + dt, tau, z[0], z[1], z[2], z[3], v_ref))
    return np.array(rows)


def push(m, k, v0):
    """A push to `v0` m/s from rest, released: the stop (the speed's first zero crossing), the travel
    there, the peak torque and the peak pitch."""
    o = simulate(m, k, [0, 0, 0, v0], 4.0)
    i = next(i for i in range(len(o)) if o[i, 5] <= 0.0)
    return {
        "stop_s": o[i, 0],
        "travel_m": o[i, 4],
        "peak_torque": float(np.abs(o[:, 1]).max()),
        "peak_pitch_deg": math.degrees(float(np.abs(o[:, 2]).max())),
    }


def held_push(m, k, newtons=20.0, hold_s=2.0):
    """A horizontal force on the axle held `hold_s`, then released: where it is at release and how far
    past the start it comes back (positive = overshoot behind the start)."""
    o = simulate(m, k, [0, 0, 0, 0], hold_s + 4.0, force=lambda t: newtons if t < hold_s else 0.0)
    n = int(round(hold_s * TICK_HZ))
    return {
        "at_release_m": o[n - 1, 4],
        "overshoot_m": max(0.0, -float(o[n:, 4].min())),
        "final_m": o[-1, 4],
    }


def stick_step(m, k):
    """Full stick from rest through section 4's reference (speed_max 80, speed_slew 41): the time to
    90 % of speed_max, the peak torque and pitch."""
    vmax_q = 32767 * SPEED_MAX // 128
    state = {"v": 0, "x": 0.0, "tick": -1}
    to_ms = m.count / 4096.0  # one 1/4096 count in m/s

    def reference(i):
        if i != state["tick"]:
            state["tick"] = i
            state["v"] += max(-SPEED_SLEW, min(SPEED_SLEW, vmax_q - state["v"]))
            state["x"] += state["v"] * to_ms / TICK_HZ
        return state["x"], state["v"] * to_ms

    o = simulate(m, k, [0, 0, 0, 0], 5.0, reference=reference)
    target = vmax_q * to_ms
    i = next(i for i in range(len(o)) if o[i, 5] >= 0.9 * target)
    return {
        "target_ms": target,
        "slew_s": vmax_q / SPEED_SLEW / TICK_HZ,
        "t90_s": o[i, 0],
        "peak_torque": float(np.abs(o[:, 1]).max()),
        "peak_pitch_deg": math.degrees(float(np.abs(o[:, 2]).max())),
    }


def parse_q(text):
    q = tuple(float(v) for v in text.split(","))
    if len(q) != 4:
        raise ValueError("--q takes four comma-separated weights")
    return q


def report(m, q, r):
    k, poles = design(m, q, r)
    ks = np.asarray(k).ravel()
    gains = m.int_gains(k)
    out = []
    np.set_printoptions(precision=4, suppress=True)
    out.append(f"A_ {m.a_:.2f} kg, B_ {m.b_:.3f} kg.m, D_ {m.d_:.2f} N.m, det {m.det:.2f}")
    out.append(f"A =\n{m.A}")
    out.append(f"B =\n{m.B}")
    ol = np.linalg.eigvals(m.A)
    out.append("open-loop poles: " + ", ".join(f"{p.real:+.2f}" for p in sorted(ol, key=lambda p: p.real)))
    out.append("closed-loop poles: " + ", ".join(f"{p.real:+.2f}{p.imag:+.2f}j" for p in sorted(poles, key=lambda p: p.real)))
    out.append(f"Q = diag{tuple(q)}, R = {r:g}")
    out.append(f"K = [{ks[0]:.1f} N.m/rad, {ks[1]:.1f} N.m/(rad/s), {ks[2]:.1f} N.m/m, {ks[3]:.1f} N.m/(m/s)]")
    out.append(f"units: edge {m.edge * 1e3:.3f} mm, count {m.count:.4f} m/s, {m.c:.4e} N.m per numerator count")
    out.append("CONTROL_SF_GAINS [gp, gr, gx, gv] = [" + ", ".join(str(round(g)) for g in gains) + "]")
    out.append(f"torque at the {POS_CLAMP}-edge position clamp: {abs(ks[2]) * POS_CLAMP * m.edge:.2f} N.m")
    out.append(f"torque at one count of speed error: {abs(ks[3]) * m.count:.2f} N.m")
    for v0 in (0.3, 0.6):
        p = push(m, k, v0)
        out.append(
            f"push {v0} m/s: stops in {p['stop_s']:.3f} s after {p['travel_m']:.3f} m, "
            f"peak torque {p['peak_torque']:.2f} N.m, peak pitch {p['peak_pitch_deg']:.2f} deg"
        )
    h = held_push(m, k)
    out.append(
        f"20 N held 2 s: {h['at_release_m']:.3f} m at release, overshoot past the start "
        f"{h['overshoot_m'] * 1000:.1f} mm, {h['final_m'] * 1000:.1f} mm after 4 s"
    )
    s = stick_step(m, k)
    out.append(
        f"stick step to {s['target_ms']:.2f} m/s (slew {s['slew_s']:.2f} s): 90 % at {s['t90_s']:.2f} s, "
        f"peak torque {s['peak_torque']:.2f} N.m, peak pitch {s['peak_pitch_deg']:.2f} deg"
    )
    return "\n".join(out)


def selftest():
    import unittest

    suite = unittest.defaultTestLoader.discover(os.path.join(_HERE, "tests"), pattern="test_lqr_gains.py")
    return 0 if unittest.TextTestRunner(verbosity=1).run(suite).wasSuccessful() else 1


def main(argv=None):
    args = build_parser().parse_args(sys.argv[1:] if argv is None else argv)
    if args.selftest:
        return selftest()
    print(report(Model(args), parse_q(args.q), args.r))
    return 0


if __name__ == "__main__":
    sys.exit(main())
