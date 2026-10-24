#!/usr/bin/env python3
"""Read the rigid-frame row of CTRL_OBS on both offroad boards.

The slice 1 and 2 disarmed gate (`specs/rover-one-imu.md` section 8): the control-mode byte
resolves to Leader (2) on the board with the IMU and Follower (3) on the other, neither raises
the mode fault, the peer cyclic age is live on both, and the leader's torque shadow moves while
its wheel demand stays zero.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import CTRL_MAGIC, Ocd, resolve_symbols, s16  # noqa: E402

PRODUCER = {0: "Throttle", 1: "Balance", 2: "Leader", 3: "Follower"}
FLAGS = [(1, "imu_cfg"), (2, "imu_live"), (4, "comms_loss"), (8, "mode_fault"),
         (16, "imu_loss"), (32, "motor_fault")]


def names(mask):
    return ",".join(n for b, n in FLAGS if mask & b) or "-"


def row(o, addr):
    w = o.read_words(addr, 36)
    sub_flags = w[12]
    return dict(
        boot=w[1],
        tick=w[2],
        pitch=s16(w[6] & 0xFFFF) if w[6] < 0x10000 else s16(w[6] & 0xFFFF),
        pitch_raw=w[6],
        cyc_age=w[7],
        drive_age=w[8],
        torque=s16(w[11] & 0xFFFF),
        sub=sub_flags & 0xFF,
        cmode=(sub_flags >> 8) & 0xFF,
        flags=(sub_flags >> 16) & 0xFF,
        levels=(sub_flags >> 24) & 0xFF,
        roll=w[18],
        gating=s16(w[26] & 0xFFFF),
        pre_env=s16((w[26] >> 16) & 0xFFFF),
        refus=w[33],
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", required=True)
    ap.add_argument("--master", default="127.0.0.1:6666")
    ap.add_argument("--slave", default="127.0.0.1:6667")
    ap.add_argument("--secs", type=float, default=0)
    ap.add_argument("--hz", type=float, default=2.0)
    a = ap.parse_args()

    addr = resolve_symbols(a.elf, "rigidobs")["CTRL_OBS"]
    conns = {}
    for nm, ep in (("M", a.master), ("S", a.slave)):
        h, p = ep.split(":")
        o = Ocd(h, int(p))
        m = o.read_words(addr, 1)[0]
        if m != CTRL_MAGIC:
            sys.exit(f"rigidobs: {nm} CTRL_OBS magic 0x{m:08x}: wrong ELF or dead board")
        conns[nm] = o

    t0 = time.time()
    print(f"{'t':>6} b {'producer':>9} {'sub':>3} {'cyc':>4} {'torque':>7} {'pre_env':>8} "
          f"{'gating':>7} {'roll':>8} flags")
    while True:
        for nm, o in conns.items():
            d = row(o, addr)
            print(f"{time.time()-t0:6.1f} {nm} {PRODUCER.get(d['cmode'], d['cmode']):>9} "
                  f"{d['sub']:>3} {d['cyc_age']:>4} {d['torque']:>7} {d['pre_env']:>8} "
                  f"{d['gating']:>7} {d['roll']:>8} {names(d['flags'])}")
        if a.secs and time.time() - t0 >= a.secs:
            return
        if not a.secs:
            return
        time.sleep(1.0 / a.hz)


main()
