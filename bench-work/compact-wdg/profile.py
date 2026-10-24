#!/usr/bin/env python3
"""Per-record profile of Store::compact: CYCCNT at every pass of its outer loop.

usage: profile.py <compact_entry> <loop_top> [--fill N]
"""
import socket, subprocess, sys, time, os

CFG = os.path.expanduser(
    "~/dev/hb-compact-wdg/crates/swd-bridge/target/aarch64-apple-darwin/release/swd-mailbox-config")
CYCCNT = "0xE0001004"
FWDGT_CTL, FWDGT_PSC, FWDGT_RLD = "0x40003000", "0x40003004", "0x40003008"

class Ocd:
    def __init__(self):
        self.s = socket.create_connection(("127.0.0.1", 6666), timeout=15)
    def cmd(self, c, timeout=60):
        self.s.settimeout(timeout)
        self.s.sendall(c.encode() + b"\x1a")
        b = b""
        while not b.endswith(b"\x1a"):
            b += self.s.recv(8192)
        return b[:-1].decode(errors="replace")
    def rd(self, a): return int(self.cmd("mdw %s" % a).split(":")[1].strip().split()[0], 16)
    def wr(self, a, v): self.cmd("mww %s %s" % (a, v))
    def state(self): return self.cmd("stm32f1x.cpu curstate").strip()
    def pc(self): return int(self.cmd("reg pc").split(":")[1].strip().split()[0], 16)
    def wait_halt(self, limit=60):
        t = time.time()
        while self.state() != "halted":
            if time.time() - t > limit: return False
            time.sleep(0.02)
        return True

entry, loop_top = sys.argv[1], sys.argv[2]
args = sys.argv[3:]
fill = int(args[args.index("--fill") + 1]) if "--fill" in args else 0
o = Ocd()
o.cmd("rbp all")
for i in range(fill):
    r = subprocess.run([CFG, "127.0.0.1:6666", "0x20=15005"], capture_output=True, text=True)
    if "CFG_OK" not in r.stdout: sys.exit("fill %d failed: %s" % (i, r.stdout[-300:]))
print("filled %d" % fill)
o.cmd("bp %s 2 hw" % entry)
p = subprocess.Popen([CFG, "127.0.0.1:6666", "0x20=15005"], stdout=subprocess.PIPE,
                     stderr=subprocess.STDOUT, text=True)
if not o.wait_halt(): sys.exit("no compact entry")
o.wr(FWDGT_CTL, "0x5555"); o.wr(FWDGT_PSC, "0x6"); o.wr(FWDGT_RLD, "0xfff"); o.wr(FWDGT_CTL, "0xaaaa")
o.cmd("rbp %s" % entry)
o.cmd("bp %s 2 hw" % loop_top)
t0 = o.rd(CYCCNT)
marks = []
while True:
    o.cmd("resume")
    if not o.wait_halt(30): break
    if o.pc() != int(loop_top, 16): break
    marks.append(o.rd(CYCCNT))
    if len(marks) > 200: break
o.cmd("rbp all")
end = o.rd(CYCCNT)
o.wr(FWDGT_CTL, "0x5555"); o.wr(FWDGT_PSC, "0x1"); o.wr(FWDGT_RLD, "0x9c3"); o.wr(FWDGT_CTL, "0xaaaa")
o.cmd("resume")
prev = t0
dl = []
for i, m in enumerate(marks):
    dl.append(m - prev); prev = m
print("loop passes: %d" % len(marks))
print("per-pass us: " + " ".join("%.0f" % (d / 72.0) for d in dl))
print("entry->first pass: %.1f ms" % ((marks[0] - t0) / 72000.0))
print("loop total: %.1f ms   tail (commit+erase+return): %.1f ms   measured total: %.1f ms"
      % ((marks[-1] - t0) / 72000.0, (end - marks[-1]) / 72000.0, (end - t0) / 72000.0))
print("watchdog restored: PSC=%d RLD=%#x" % (o.rd(FWDGT_PSC), o.rd(FWDGT_RLD)))
try: p.communicate(timeout=20)
except Exception: p.kill()
