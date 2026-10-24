#!/usr/bin/env python3
"""Measure MAIN-THREAD ABSENCES on the master, the instrument `tick - ctrl` cannot see.

Live statics in one 8-word read at 0x200008d0: PERIODS, OBS_STATE, VBATT_RAW, STACK_MARK,
TICK_COUNT, BLE_RX_LOSSES, MAILBOX_DROPS, DISPATCH_COUNT. A slower full read of both boards'
CTRL_OBS carries cyclic_age / arms / laps so an absence can be lined up against an arm cycle.
"""
import argparse, csv, sys, time
sys.path.insert(0, "tools")
from swdobs import Ocd

LIVE = 0x200008D0           # 8 words: ..., TICK_COUNT(+0x10), BLE_RX(+0x14), MBOX(+0x18), DISPATCH(+0x1C)
I_PERIODS, I_TICK, I_BLERX, I_DISP = 0, 4, 5, 7
OBS = 0x20000A88

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=200)
ap.add_argument("--csv", default="bench-work/absence.csv")
ap.add_argument("--every", type=int, default=8, help="full both-board read every N rounds")
a = ap.parse_args()

m = Ocd("127.0.0.1", 6668)
s = Ocd("127.0.0.1", 6667)
f = open(a.csv, "w", newline="")
w = csv.writer(f)
w.writerow(["wall", "m_periods", "m_tick", "m_blerx", "m_disp",
            "m_cA", "m_init", "m_lap", "m_mode", "m_moe", "s_cA", "s_init", "s_lap"])
t0 = time.time()
n = 0
full = [0] * 8
try:
    while time.time() - t0 < a.secs:
        lv = m.read_words(LIVE, 8)
        if n % a.every == 0:
            mo = m.read_words(OBS + 4 * 7, 8)      # cyclic_age..sub/flags/levels
            ml = m.read_words(OBS + 4 * 14, 1)[0]  # link_lap_overruns
            so = s.read_words(OBS + 4 * 7, 8)
            sl = s.read_words(OBS + 4 * 14, 1)[0]
            full = [mo[0], mo[2], ml, (mo[4] >> 16) & 0xFF, (mo[4] >> 24) & 0xFF, so[0], so[2], sl]
        w.writerow([f"{time.time():.4f}", lv[I_PERIODS], lv[I_TICK], lv[I_BLERX] & 0xFFFF,
                    lv[I_DISP]] + full)
        n += 1
except KeyboardInterrupt:
    pass
finally:
    f.close(); m.close(); s.close()
dt = time.time() - t0
print(f"{n} rounds over {dt:.1f}s = {n/dt:.1f}/s")
