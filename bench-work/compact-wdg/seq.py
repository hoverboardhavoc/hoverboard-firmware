#!/usr/bin/env python3
"""Time the phases of one Store::compact: halt at a sequence of addresses, CYCCNT at each.

usage: seq.py --fill N addr:label addr:label ...   (the first address is the entry)
"""
import socket, subprocess, sys, time, os
CFG = os.path.expanduser(
    "~/dev/hb-compact-wdg/crates/swd-bridge/target/aarch64-apple-darwin/release/swd-mailbox-config")
CYCCNT = "0xE0001004"
W_CTL, W_PSC, W_RLD = "0x40003000", "0x40003004", "0x40003008"

class Ocd:
    def __init__(self):
        self.s = socket.create_connection(("127.0.0.1", 6666), timeout=15)
    def cmd(self, c, t=60):
        self.s.settimeout(t); self.s.sendall(c.encode() + b"\x1a"); b = b""
        while not b.endswith(b"\x1a"): b += self.s.recv(8192)
        return b[:-1].decode(errors="replace")
    def rd(self, a): return int(self.cmd("mdw %s" % a).split(":")[1].strip().split()[0], 16)
    def wr(self, a, v): self.cmd("mww %s %s" % (a, v))
    def state(self): return self.cmd("stm32f1x.cpu curstate").strip()
    def wait(self, limit=90):
        t = time.time()
        while self.state() != "halted":
            if time.time() - t > limit: return False
            time.sleep(0.02)
        return True

args = sys.argv[1:]
fill = int(args[args.index("--fill") + 1]) if "--fill" in args else 0
pts = [a for a in args if ":" in a]
o = Ocd(); o.cmd("rbp all")
for i in range(fill):
    r = subprocess.run([CFG, "127.0.0.1:6666", "0x20=15005"], capture_output=True, text=True)
    if "CFG_OK" not in r.stdout: sys.exit("fill %d failed" % i)
print("filled %d" % fill)
first = pts[0].split(":")[0]
o.cmd("bp %s 2 hw" % first)
p = subprocess.Popen([CFG, "127.0.0.1:6666", "0x20=15005"], stdout=subprocess.PIPE,
                     stderr=subprocess.STDOUT, text=True)
if not o.wait(): sys.exit("no entry halt")
o.wr(W_CTL, "0x5555"); o.wr(W_PSC, "0x6"); o.wr(W_RLD, "0xfff"); o.wr(W_CTL, "0xaaaa")
assert o.rd(W_PSC) == 6, "widen did not take"
o.cmd("rbp %s" % first)
t = [o.rd(CYCCNT)]
for spec in pts[1:]:
    addr = spec.split(":")[0]
    o.cmd("bp %s 2 hw" % addr)
    o.cmd("resume")
    if not o.wait(): sys.exit("never reached %s" % spec)
    o.cmd("rbp %s" % addr)
    t.append(o.rd(CYCCNT))
for i in range(1, len(t)):
    print("  %-28s %8.2f ms" % (pts[i].split(":")[1], (t[i] - t[i - 1]) / 72000.0))
print("  %-28s %8.2f ms" % ("TOTAL (with halt artifact)", (t[-1] - t[0]) / 72000.0))
o.cmd("rbp all")
o.wr(W_CTL, "0x5555"); o.wr(W_PSC, "0x1"); o.wr(W_RLD, "0x9c3"); o.wr(W_CTL, "0xaaaa")
print("watchdog PSC=%d RLD=%#x (restored under halt)" % (o.rd(W_PSC), o.rd(W_RLD)))
o.cmd("resume")
try: p.communicate(timeout=20)
except Exception: p.kill()
print("boot_count", o.rd("0x20000a94"))
