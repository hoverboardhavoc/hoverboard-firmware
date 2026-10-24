#!/usr/bin/env python3
"""Measure which chip axis is UP on a mounted board, by asking the board itself.

`CTRL_OBS` publishes the CONDITIONED up-axis accel count (word 26 low half, `gating_field`): the
component the engagement machine gates on, after the staged sign and role map. Point the UP role at
each chip axis in turn and the board reports that axis's component. At rest one of them is gravity,
about +-8192 counts per g, and the other two are near zero, so three reads give the whole vector
without a raw-accel observable existing.

The role pairs are chosen so each frame is a proper rotation with distinct roles, which
`imu::Config::staged` requires and refuses otherwise (the refusal shows as BOARD_OBS result 11 and
no IMU at all): UP=X with PITCH=Y, UP=Y with PITCH=Z, UP=Z with PITCH=X are the three even
permutations.
"""
import subprocess
import sys
import time

sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd, resolve_symbols, CTRL_OBS_WORDS, s16, s32  # noqa: E402

CFG = ("/Users/hoverboardhavoc/dev/hoverboard-firmware/crates/swd-bridge/target/"
       "aarch64-apple-darwin/release/swd-mailbox-config")
ELF = "/Users/hoverboardhavoc/dev/hoverboard-firmware/bench-work/main-merged.elf"
addr = resolve_symbols(ELF, "upaxis")["CTRL_OBS"]
AXES = {1: "X", 2: "Y", 3: "Z"}
PAIRS = [(1, 2), (2, 3), (3, 1)]   # (up, pitch_rate), each an even permutation


def cfg(port, *args):
    subprocess.run([CFG, f"127.0.0.1:{port}"] + list(args), capture_output=True, timeout=600)


def measure(port, label):
    o = Ocd("127.0.0.1", port)
    out = {}
    for up, pitch in PAIRS:
        cfg(port, f"0x68:0={up}", f"0x68:1={pitch}")
        o.cmd("reset run")
        time.sleep(6)
        w = o.read_words(addr, CTRL_OBS_WORDS)
        b = o.read_words(0x20000890, 3)
        out[up] = (s16(w[26] & 0xFFFF), b[1] & 0xFF, (w[12] >> 16) & 0xFF, s32(w[6]), s32(w[18]))
        print(f"{label}: UP={AXES[up]} PITCH={AXES[pitch]} -> accel {out[up][0]:6d} counts "
              f"({out[up][0]/8192:+.2f} g)  BOARD={out[up][1]} flags=0x{out[up][2]:02x}")
    best = max(out, key=lambda k: abs(out[k][0]))
    print(f"{label}: gravity is on chip axis {AXES[best]}, {out[best][0]/8192:+.2f} g, "
          f"so UP = {AXES[best]} with sign {'+1' if out[best][0] > 0 else '-1'}")
    return out, best


for label, port in (("rover master", 6668), ("rover slave", 6669)):
    measure(port, label)
    print()
