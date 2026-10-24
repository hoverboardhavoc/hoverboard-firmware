#!/usr/bin/env python3
"""Sample the motor rows on both rover boards while the owner handles the machine roughly.

Three candidates for a slipping or growling noise under fast reversals, and the published words
separate them:
  word 31  peak current | chopped periods | trip count   the current limiter chopping
  word 23  motor fault bits | invalid-hall dwell         the hall front end losing the rotor
  word 20  hall | enables | method | flags               OBS_COASTING, the demand gone stale
  word 24  wheel speed                                    to correlate with how fast it is moving
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import Ocd, resolve_symbols, s16, s32  # noqa: E402

elf, secs = sys.argv[1], float(sys.argv[2])
addr = resolve_symbols(elf, "rough")["CTRL_OBS"]
cons = [(nm, Ocd("127.0.0.1", p)) for nm, p in (("M", 6668), ("S", 6669))]
peak = {"M": 0, "S": 0}
chopped = {"M": 0, "S": 0}
trips = {"M": 0, "S": 0}
dwell = {"M": 0, "S": 0}
faults = {"M": 0, "S": 0}
coast = {"M": 0, "S": 0}
speed = {"M": 0, "S": 0}
n = {"M": 0, "S": 0}
t0 = time.time()
while time.time() - t0 < secs:
    for nm, o in cons:
        try:
            w = o.read_words(addr, 37)
        except Exception:
            continue
        n[nm] += 1
        peak[nm] = max(peak[nm], abs(s16(w[31] & 0xFFFF)))
        chopped[nm] = max(chopped[nm], (w[31] >> 16) & 0xFF)
        trips[nm] = max(trips[nm], (w[31] >> 24) & 0xFF)
        dwell[nm] = max(dwell[nm], (w[23] >> 16) & 0xFFFF)
        faults[nm] |= w[23] & 0xFFFF
        if (w[20] >> 24) & 0x08:
            coast[nm] += 1
        speed[nm] = max(speed[nm], abs(s32(w[24])))
FA = [(1, "HALL"), (2, "DUTY"), (4, "STALE"), (8, "CAL"), (16, "METHOD")]
for nm in ("M", "S"):
    f = ",".join(x for b, x in FA if faults[nm] & b) or "-"
    print(f"{nm}: {n[nm]} samples  peak_current={peak[nm]}  chopped_periods={chopped[nm]}  "
          f"current_trips={trips[nm]}  invalid_hall_dwell={dwell[nm]}  coasting_samples={coast[nm]}  "
          f"max_speed={speed[nm]}  faults=[{f}]")
