#!/usr/bin/env python3
"""Record both boards of a rigid pair while the owner arms and nudges the frame.

The follower applies the leader's `peer_demand`, so the two torque columns side by side ARE the
demand-tracking gate: equal and simultaneous is the pass, and a lag or a disagreement names the
board that did it.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import Ocd, resolve_symbols, s16, s32  # noqa: E402

elf, out, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
addr = resolve_symbols(elf, "nudge")["CTRL_OBS"]
cons = [(nm, Ocd("127.0.0.1", p)) for nm, p in (("M", 6666), ("S", 6667))]
f = open(out, "w")
f.write("t,board,producer,sub,moe,torque,pre_env,pitch_mdeg,speed,cyc_age,flags,levels\n")
t0 = time.time()
while time.time() - t0 < secs:
    for nm, o in cons:
        try:
            w = o.read_words(addr, 36)
        except Exception:
            continue
        sf = w[12]
        f.write("%.3f,%s,%d,%d,%d,%d,%d,%d,%d,%d,0x%02x,0x%02x\n" % (
            time.time() - t0, nm, (sf >> 8) & 0xFF, sf & 0xFF, (w[11] >> 24) & 0xFF,
            s16(w[11] & 0xFFFF), s16((w[26] >> 16) & 0xFFFF), s32(w[6]), s32(w[24]), w[7],
            (sf >> 16) & 0xFF, (sf >> 24) & 0xFF))
    f.flush()
f.close()
