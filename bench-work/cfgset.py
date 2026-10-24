#!/usr/bin/env python3
"""Write store fields and CONFIRM them, retrying the walk.

The mailbox walk times out intermittently on the rover master. A write whose output is not read is
a write that may not have happened: on 2026-10-23 three axis-map trials silently ran the unchanged
map because the tool's failures were swallowed, and the readings were then interpreted as three
different frames.
"""
import subprocess
import sys

CFG = ("/Users/hoverboardhavoc/dev/hoverboard-firmware/crates/swd-bridge/target/"
       "aarch64-apple-darwin/release/swd-mailbox-config")
port, pairs = sys.argv[1], sys.argv[2:]
for attempt in range(6):
    p = subprocess.run([CFG, f"127.0.0.1:{port}"] + pairs, capture_output=True, text=True,
                       timeout=900)
    ok = [ln.strip() for ln in p.stdout.splitlines() if "CONFIG_WRITE" in ln or "CONFIG_READ" in ln]
    if p.returncode == 0 and ok:
        print("\n".join(ok))
        sys.exit(0)
    print(f"attempt {attempt + 1} failed: {p.stdout.strip().splitlines()[-1] if p.stdout else p.stderr.strip()[:80]}")
sys.exit("cfgset: six attempts failed")
