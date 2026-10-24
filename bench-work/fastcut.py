#!/usr/bin/env python3
"""One board, two CTRL_OBS words (torque/mode/MOE and the state/flag bytes) as fast as the probe
allows, for the inter-board wire-cut gate.

Two things have to be seen apart: the rigid window zeroing the wheel demand (RIGID_STALE_TICKS,
5 ticks = 20 ms) and the slower comms_loss disarm (CYCLIC_TIMEOUT_TICKS, 25 ticks = 100 ms).
A reader sampling at the tick rate cannot tell them apart, so this reads only the two words it
needs, in one transaction.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import Ocd, resolve_symbols, s16  # noqa: E402

elf, port, out, secs = sys.argv[1], int(sys.argv[2]), sys.argv[3], float(sys.argv[4])
addr = resolve_symbols(elf, "fastcut")["CTRL_OBS"]
o = Ocd("127.0.0.1", port)
rows = []
t0 = time.time()
while time.time() - t0 < secs:
    w = o.read_words(addr + 4 * 11, 2)
    rows.append((time.time() - t0, s16(w[0] & 0xFFFF), (w[0] >> 24) & 0xFF, w[1] & 0xFF,
                 (w[1] >> 24) & 0xFF))
with open(out, "w") as f:
    f.write("t,torque,moe,sub,levels\n")
    for r in rows:
        f.write("%.4f,%d,%d,%d,%d\n" % r)
print("%s: %d samples, %.1f Hz" % (out, len(rows), len(rows) / (time.time() - t0)))
