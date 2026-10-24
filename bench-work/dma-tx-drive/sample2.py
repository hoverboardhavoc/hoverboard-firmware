#!/usr/bin/env python3
"""sample2.py PORT CTRL_OBS_HEX SECONDS: arm-cycle view: tick, peer age, inits, shutdowns, torque|mode|moe, sub|cmode|flags|levels, events hi, stack."""
import sys, time, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from swdobs import Ocd
port, base, secs = int(sys.argv[1]), int(sys.argv[2], 16), float(sys.argv[3])
o = Ocd("localhost", port); t0 = time.time()
print("t,tick,peer_age,inits,shutdowns,torque_mode,sub_flags,events_hi,stack")
while time.time() - t0 < secs:
    w = o.read_words(base, 39)
    print(f"{time.time()-t0:.2f},{w[2]},{w[7]},{w[9]},{w[10]},{w[11]:08x},{w[12]:08x},{w[28]:08x},{w[30]&0xffff}", flush=True)
    time.sleep(0.15)
o.close()
