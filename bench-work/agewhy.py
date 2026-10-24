#!/usr/bin/env python3
"""Sample the peer cyclic age beside the things that would explain a late frame, on one board.

The question the age alone cannot answer: is a 44 ms gap the WIRE losing frames, or this board
failing to drain them? Four words, one transaction each, as fast as the probe allows:

  word  7  cyclic_age          how stale the peer's state is, in 4 ms ticks, on a clock
  word 13  link_line_errors    framing/overrun/parity scored on the inter-board port
  word 14  link_lap_overruns   the DMA ring wrapped before this board drained it
  word  2  tick_count          this board's own control ticks, to see its progress stall

A ring overrun beside an age excursion is a RECEIVER stall: the bytes arrived and were not read
in time. A line error beside one is the WIRE. Neither, and the sender was late emitting.
Noise errors are currently invisible in word 13 (the HAL never clears or counts NERR), so a run
with neither counter moving does not yet exonerate the wire.
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
ap.add_argument("--csv", default=None)
a = ap.parse_args()

addr = resolve_symbols(a.elf, "agewhy")["CTRL_OBS"]
o = Ocd("127.0.0.1", a.port)
hist = Counter()
armed_hist = Counter()
idle_hist = Counter()
events = []          # (t, age, d_line, d_lap, d_tick) for every age >= 4
bad = 0
prev = None
t0 = time.time()
n = 0
while time.time() - t0 < a.secs:
    try:
        w = o.read_words(addr, 36)
    except Exception:
        bad += 1
        time.sleep(0.01)
        continue
    t = time.time() - t0
    age, line, lap, tick = w[7], w[13], w[14], w[2]
    # Armed or not is recorded with every sample, never assumed from the start of the
    # run: the app's INPUTS mirror expires 1.5 s after the phone stops sending, so a
    # screen that sleeps turns an ENERGISED measurement into a disarmed one halfway
    # through and nothing in the numbers says so. That cost a ten-minute run on
    # 2026-10-23.
    armed = ((w[11] >> 24) & 0xFF) != 0
    hist[age] += 1
    (armed_hist if armed else idle_hist)[age] += 1
    n += 1
    if prev is not None and age >= 4:
        events.append((t, age, line - prev[0], lap - prev[1], tick - prev[2]))
    prev = (line, lap, tick)
el = time.time() - t0
print(f"{a.label}: {n} samples in {el:.1f} s, {n/el:.0f} Hz, {bad} failed reads")
print(f"{a.label}: max age {max(hist)} ticks ({4*max(hist)} ms); line_errors {prev[0]}, lap_overruns {prev[1]}")
na, ni = sum(armed_hist.values()), sum(idle_hist.values())
print(f"{a.label}: {na} samples ARMED ({100.0*na/max(n,1):.1f} %), {ni} disarmed")
for nm_, h, tot in (("armed", armed_hist, na), ("idle", idle_hist, ni)):
    if not tot:
        continue
    print(f"{a.label}: {nm_} max age {max(h)} ticks ({4*max(h)} ms)")
    for age_ in sorted(h):
        print(f"{a.label}: {nm_} age {age_:3d}  {h[age_]:7d}  {100.0*h[age_]/tot:6.3f} %")
withlap = [e for e in events if e[3] > 0]
withline = [e for e in events if e[2] > 0]
print(f"{a.label}: {len(events)} samples at age >= 4; {len(withlap)} coincide with a ring overrun, "
      f"{len(withline)} with a line error")
for e in events[:25]:
    print(f"{a.label}:   t={e[0]:7.2f} age={e[1]:3d} d_line={e[2]} d_lap={e[3]} d_tick={e[4]}")
if a.csv:
    with open(a.csv, "w") as f:
        f.write("t,age,d_line,d_lap,d_tick\n")
        for e in events:
            f.write("%.4f,%d,%d,%d,%d\n" % e)
