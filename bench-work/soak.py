#!/usr/bin/env python3
"""Watch the latched peak peer age on both boards for a long run, cheaply.

The peak (CTRL_OBS word 36) is latched in firmware, so this does not have to sample fast enough
to catch an excursion: it only has to read the latch now and then. It records the arm state with
every sample because the app disarms the pair when the phone's screen times out
(`specs/ble-reliability.md`), so "how much of this run was energised" is part of the result.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import Ocd, resolve_symbols, CTRL_OBS_WORDS, s16  # noqa: E402

elf, out, secs, every = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
addr = resolve_symbols(elf, "soak")["CTRL_OBS"]
cons = [(nm, Ocd("127.0.0.1", p)) for nm, p in (("M", 6666), ("S", 6667))]
f = open(out, "w")
f.write("t,board,ticks,peak,age,armed,torque,line_err,lap,drive_age\n")
t0 = time.time()
while time.time() - t0 < secs:
    for nm, o in cons:
        try:
            w = o.read_words(addr, CTRL_OBS_WORDS)
        except Exception:
            continue
        f.write("%.1f,%s,%d,%d,%d,%d,%d,%d,%d,%d\n" % (
            time.time() - t0, nm, w[2], w[36], w[7], 1 if (w[11] >> 24) & 0xFF else 0,
            s16(w[11] & 0xFFFF), w[13], w[14], w[8]))
    f.flush()
    time.sleep(every)
f.close()
