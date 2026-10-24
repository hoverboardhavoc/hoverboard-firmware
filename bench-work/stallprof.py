#!/usr/bin/env python3
"""Fast single-board sampler: tick_count / dispatch_count / control_ticks, to profile loop stalls."""
import argparse, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import CTRL_MAGIC, DEFAULT_ELF, Ocd, resolve_symbols

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=6668)
ap.add_argument("--secs", type=float, default=30)
ap.add_argument("--elf", default=DEFAULT_ELF)
ap.add_argument("--csv", default=None)
a = ap.parse_args()
addr = resolve_symbols(a.elf, "stallprof")["CTRL_OBS"]
o = Ocd("127.0.0.1", a.port)
assert o.read_words(addr, 1)[0] == CTRL_MAGIC
rows = []
t0 = time.time()
while time.time() - t0 < a.secs:
    w = o.read_words(addr + 4 * 2, 3)   # tick_count, dispatch_count, control_ticks
    rows.append((time.time() - t0, w[0], w[1], w[2]))
o.close()
print(f"samples={len(rows)} over {rows[-1][0]:.1f}s -> {len(rows)/rows[-1][0]:.1f} reads/s")
gaps = []
for (t1, k1, d1, c1), (t2, k2, d2, c2) in zip(rows, rows[1:]):
    dt = t2 - t1
    gaps.append((t1, dt, k2 - k1, d2 - d1, c2 - c1))
tot = rows[-1][0]
print(f"rates: tick={(rows[-1][1]-rows[0][1])/tot:.1f}/s dispatch={(rows[-1][2]-rows[0][2])/tot:.1f}/s ctrl={(rows[-1][3]-rows[0][3])/tot:.1f}/s")
stalls = [g for g in gaps if g[3] == 0 and g[2] >= 2]   # ticks advanced, no loop pass at all
print(f"intervals with ZERO loop passes but >=2 systicks: {len(stalls)}")
for g in stalls[:25]:
    print(f"   t={g[0]:7.3f} dt={g[1]*1000:6.1f}ms systicks={g[2]:3d} passes={g[3]:4d} ctrl={g[4]:3d}")
# per-interval passes-per-ms, worst
worst = sorted(gaps, key=lambda g: g[3] / max(g[1], 1e-6))[:15]
print("lowest pass-rate intervals:")
for g in worst:
    print(f"   t={g[0]:7.3f} dt={g[1]*1000:6.1f}ms systicks={g[2]:3d} passes={g[3]:5d} ctrl={g[4]:3d} -> {g[3]/g[1]:7.0f} passes/s")
if a.csv:
    with open(a.csv, "w") as f:
        f.write("t,tick,dispatch,ctrl\n")
        for r in rows:
            f.write(f"{r[0]:.4f},{r[1]},{r[2]},{r[3]}\n")
