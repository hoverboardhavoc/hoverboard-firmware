#!/usr/bin/env python3
"""High-rate dual sampler: two narrow CTRL_OBS reads per board per round.

Words 2..12 (tick, dispatch, ctrl, input, pitch, cyclic_age, drive_age, inits, shutdowns,
torque|mode|moe, sub|cmode|flags|levels) in ONE 11-word read, plus words 19..23 in a second.
"""
import argparse, csv, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import CTRL_MAGIC, DEFAULT_ELF, Ocd, resolve_symbols

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=120)
ap.add_argument("--csv", default="bench-work/fast.csv")
ap.add_argument("--elf", default=DEFAULT_ELF)
a = ap.parse_args()
addr = resolve_symbols(a.elf, "fastobs")["CTRL_OBS"]
conns = {}
for nm, p in (("M", 6668), ("S", 6667)):
    o = Ocd("127.0.0.1", p)
    assert o.read_words(addr, 1)[0] == CTRL_MAGIC, nm
    conns[nm] = o
f = open(a.csv, "w", newline="")
wr = csv.writer(f)
wr.writerow(["t", "b"] + [f"w{i}" for i in range(2, 13)] + [f"w{i}" for i in range(19, 25)])
t0 = time.time()
n = 0
try:
    while time.time() - t0 < a.secs:
        for nm, o in conns.items():
            try:
                lo = o.read_words(addr + 8, 11)
                hi = o.read_words(addr + 4 * 19, 6)
            except Exception as e:
                print(f"{time.time()-t0:.3f} {nm} FAIL {e}", flush=True)
                continue
            wr.writerow([f"{time.time()-t0:.4f}", nm] + lo + hi)
            n += 1
except KeyboardInterrupt:
    pass
finally:
    f.close()
    for o in conns.values():
        o.close()
print(f"{n} rows over {time.time()-t0:.1f}s = {n/2/(time.time()-t0):.1f} rounds/s")
