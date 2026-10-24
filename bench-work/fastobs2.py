#!/usr/bin/env python3
"""One 28-word CTRL_OBS read per board per round (words 2..29): counters, ages, mode/levels,
link + BLE lap overruns, motor block, event counts."""
import argparse, csv, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import CTRL_MAGIC, DEFAULT_ELF, Ocd, resolve_symbols

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=180)
ap.add_argument("--csv", default="bench-work/hold.csv")
ap.add_argument("--elf", default=DEFAULT_ELF)
a = ap.parse_args()
addr = resolve_symbols(a.elf, "fastobs2")["CTRL_OBS"]
conns = {}
for nm, p in (("M", 6668), ("S", 6667)):
    o = Ocd("127.0.0.1", p)
    assert o.read_words(addr, 1)[0] == CTRL_MAGIC, nm
    conns[nm] = o
f = open(a.csv, "w", newline="")
wr = csv.writer(f)
wr.writerow(["t", "wall", "b"] + [f"w{i}" for i in range(2, 30)])
t0 = time.time()
n = 0
try:
    while time.time() - t0 < a.secs:
        for nm, o in conns.items():
            try:
                w = o.read_words(addr + 8, 28)
            except Exception as e:
                print(f"{time.time()-t0:.3f} {nm} FAIL {e}", flush=True)
                continue
            now = time.time()
            wr.writerow([f"{now-t0:.4f}", f"{now:.3f}", nm] + w)
            n += 1
except KeyboardInterrupt:
    pass
finally:
    f.close()
    for o in conns.values():
        o.close()
print(f"{n} rows, {n/2/(time.time()-t0):.1f} rounds/s")
