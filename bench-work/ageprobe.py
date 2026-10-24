#!/usr/bin/env python3
"""Poll one board's peer cyclic age (CTRL_OBS word 7) as fast as the probe allows and keep the
distribution, for the RIGID_STALE_TICKS measurement (`specs/rover-one-imu.md` section 8).

The age counts control ticks since the last peer CYCLIC_STATE, so an excursion of k ticks is
visible for k * 4 ms; the sampling interval is printed with the result so what the run could have
missed is stated rather than assumed. The core is never halted: read_memory on the running target.
"""
import argparse
import os
import sys
import time
from collections import Counter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import Ocd, resolve_symbols  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--elf", required=True)
ap.add_argument("--port", type=int, required=True)
ap.add_argument("--secs", type=float, default=600.0)
ap.add_argument("--label", default="board")
a = ap.parse_args()

addr = resolve_symbols(a.elf, "ageprobe")["CTRL_OBS"]
o = Ocd("127.0.0.1", a.port)
hist = Counter()
worst = []
t0 = time.time()
n = 0
while time.time() - t0 < a.secs:
    age = o.read_words(addr + 4 * 7, 1)[0]
    hist[age] += 1
    n += 1
    if age >= 4:
        worst.append((round(time.time() - t0, 3), age))
el = time.time() - t0
print(f"{a.label}: {n} samples in {el:.1f} s, {n/el:.0f} Hz, interval {1000*el/n:.1f} ms")
print(f"{a.label}: max age {max(hist)} ticks ({4*max(hist)} ms)")
for age in sorted(hist):
    print(f"{a.label}: age {age:3d}  {hist[age]:7d}  {100.0*hist[age]/n:6.3f} %")
if worst:
    print(f"{a.label}: {len(worst)} samples at age >= 4: {worst[:40]}")
