#!/usr/bin/env python3
"""sample.py PORT CTRL_OBS_HEX OUT.csv SECONDS: log the observation words over the OpenOCD TCL port."""
import sys, time, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from swdobs import Ocd
port, base, out, secs = int(sys.argv[1]), int(sys.argv[2], 16), sys.argv[3], float(sys.argv[4])
o = Ocd("localhost", port)
t0 = time.time()
with open(out, "w") as f:
    f.write("t,boot,tick,torque_mode,sub_flags,motor_state,fault,speed,stack,txto,reset_cause,rstsck\n")
    while time.time() - t0 < secs:
        try:
            w = o.read_words(base, 39)
            r = o.read_words(0x40021024, 1)[0]
            row = [f"{time.time()-t0:.2f}", w[1], w[2], f"{w[11]:08x}", f"{w[12]:08x}", f"{w[20]:08x}", f"{w[23]:08x}", w[24] if w[24] < 0x80000000 else w[24]-(1<<32), f"{w[30]:08x}", f"{w[35]:08x}", f"{w[38]:08x}", f"{r:08x}"]
            f.write(",".join(map(str, row)) + "\n"); f.flush()
        except Exception as e:
            f.write(f"{time.time()-t0:.2f},ERR {e}\n"); f.flush()
        time.sleep(0.2)
o.close()
