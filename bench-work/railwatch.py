#!/usr/bin/env python3
"""Sample the master's own battery sense fast while the wheels drive, to catch a rail collapse.

The pair has cold-booted twice under motor load (`boot_count` back to 1, no watchdog flag). The
board measures its own supply on PA4, so the shape of the fall says which fault it is: a sag over
tens of milliseconds into a current limit, or an instant cut, which is a contact opening.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import Ocd, resolve_symbols, CTRL_OBS_WORDS, s16, s32  # noqa: E402

elf, out, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
addr = resolve_symbols(elf, "railwatch")["CTRL_OBS"]
o = Ocd("127.0.0.1", 6666)
rows = []
t0 = time.time()
last_tick = None
while time.time() - t0 < secs:
    try:
        w = o.read_words(addr, CTRL_OBS_WORDS)
    except Exception:
        continue
    t = time.time() - t0
    rows.append((t, w[32], s16(w[11] & 0xFFFF), s32(w[24]), w[2], (w[11] >> 24) & 0xFF,
                 s16(w[31] & 0xFFFF)))
    if last_tick is not None and w[2] < last_tick:
        print("RESET detected at t=%.2f" % t)
        break
    last_tick = w[2]
with open(out, "w") as f:
    f.write("t,battery_cv,torque,speed,ticks,moe,peak_current\n")
    for r in rows:
        f.write("%.4f,%d,%d,%d,%d,%d,%d\n" % r)
lo = min(r[1] for r in rows if r[1] > 0) if rows else 0
print("%d samples, %.0f Hz, battery min %d cV, max %d cV" % (
    len(rows), len(rows) / (time.time() - t0), lo, max(r[1] for r in rows)))
for r in rows[-14:]:
    print("  t=%7.3f batt=%4d torque=%6d speed=%5d moe=0x%02x" % (r[0], r[1], r[2], r[3], r[5]))
