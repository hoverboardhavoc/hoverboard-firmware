#!/usr/bin/env python3
"""Measure Store::compact on silicon with DWT CYCCNT and two hardware breakpoints.

usage: measure.py <entry_hex> [--widen] [--fill N] [--field 0x20=VALUE]

  - fills the active page with N confirming writes (none trigger compaction),
  - arms a hw breakpoint at compact's entry,
  - issues one more write in the background, which triggers compaction and halts the core,
  - optionally widens the free watchdog AT THAT HALT (needed on an image whose compaction
    does not fit the shipped window), reads CYCCNT + LR,
  - breakpoints LR, resumes, and reads CYCCNT again at the return.
"""
import socket, subprocess, sys, time, os

HOST = ("127.0.0.1", 6666)
CFG = os.path.expanduser(
    "~/dev/hb-compact-wdg/crates/swd-bridge/target/aarch64-apple-darwin/release/swd-mailbox-config")

class Ocd:
    def __init__(self, hostport=HOST):
        self.s = socket.create_connection(hostport, timeout=15)
    def cmd(self, c, timeout=60):
        self.s.settimeout(timeout)
        self.s.sendall(c.encode() + b"\x1a")
        b = b""
        while not b.endswith(b"\x1a"):
            chunk = self.s.recv(8192)
            if not chunk:
                raise RuntimeError("openocd closed")
            b += chunk
        return b[:-1].decode(errors="replace")
    def rd(self, addr):
        return int(self.cmd("mdw %s" % addr).split(":")[1].strip().split()[0], 16)
    def wr(self, addr, val):
        self.cmd("mww %s %s" % (addr, val))
    def state(self):
        return self.cmd("stm32f1x.cpu curstate").strip()

CYCCNT = "0xE0001004"
FWDGT_CTL, FWDGT_PSC, FWDGT_RLD = "0x40003000", "0x40003004", "0x40003008"

def widen(o, psc=6, rld=0xFFF):
    o.wr(FWDGT_CTL, "0x5555"); o.wr(FWDGT_PSC, hex(psc)); o.wr(FWDGT_RLD, hex(rld))
    o.wr(FWDGT_CTL, "0xaaaa")
    return o.rd(FWDGT_PSC), o.rd(FWDGT_RLD)

def restore(o):
    o.wr(FWDGT_CTL, "0x5555"); o.wr(FWDGT_PSC, "0x1"); o.wr(FWDGT_RLD, "0x9c3")
    o.wr(FWDGT_CTL, "0xaaaa")
    return o.rd(FWDGT_PSC), o.rd(FWDGT_RLD)

def write_field(arg, background=False):
    cmd = [CFG, "127.0.0.1:6666", arg]
    if background:
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return subprocess.run(cmd, capture_output=True, text=True)

def main():
    entry = sys.argv[1]
    args = sys.argv[2:]
    do_widen = "--widen" in args
    fill = int(args[args.index("--fill") + 1]) if "--fill" in args else 0
    field = args[args.index("--field") + 1] if "--field" in args else "0x20=15005"

    o = Ocd()
    print("CCHP (MOE must be clear):", hex(o.rd("0x40012c44")))
    assert (o.rd("0x40012c44") >> 15) & 1 == 0, "MOE SET - board is armed, refusing to halt"
    o.cmd("rbp all")

    for i in range(fill):
        r = write_field(field)
        ok = "CFG_OK" in r.stdout
        print("  fill %d/%d: %s" % (i + 1, fill, "CFG_OK" if ok else r.stdout[-300:]))
        if not ok:
            sys.exit("fill write did not answer CFG_OK")

    print("arming breakpoint at", entry)
    print(o.cmd("bp %s 2 hw" % entry))
    p = None if "--no-write" in args else write_field(field, background=True)
    t0 = time.time()
    while o.state() != "halted":
        if time.time() - t0 > 60:
            sys.exit("no halt in 60 s: compaction was not triggered")
        time.sleep(0.05)
    print("halted at compact entry after %.1f s" % (time.time() - t0))
    lr = o.cmd("reg lr").split(":")[1].strip().split()[0]
    lr_addr = int(lr, 16) & ~1
    c_frozen_a = o.rd(CYCCNT); time.sleep(1.0); c_frozen_b = o.rd(CYCCNT)
    print("CYCCNT while halted: %d -> %d (delta %d, must be ~0)" % (c_frozen_a, c_frozen_b, c_frozen_b - c_frozen_a))
    print("LR =", hex(lr_addr))
    if do_widen:
        print("watchdog widened to PSC=%d RLD=%#x" % widen(o))
    if "--feed" in args:
        # Start the shipped window fresh at the entry, the way the main loop's feed does a few ms
        # before a CONFIG_WRITE reaches the store. PSC/RLD untouched.
        o.wr(FWDGT_CTL, "0xaaaa")
        print("watchdog FED at the entry, window unchanged: PSC=%d RLD=%#x"
              % (o.rd(FWDGT_PSC), o.rd(FWDGT_RLD)))
    c0 = o.rd(CYCCNT)
    o.cmd("rbp %s" % entry)
    print(o.cmd("bp %#x 2 hw" % lr_addr))
    o.cmd("resume")
    t0 = time.time()
    while o.state() != "halted":
        if time.time() - t0 > 120:
            sys.exit("no return halt in 120 s")
        time.sleep(0.02)
    c1 = o.rd(CYCCNT)
    cyc = (c1 - c0) & 0xFFFFFFFF
    print("=== compact: %d cycles = %.1f ms at 72 MHz ===" % (cyc, cyc / 72000.0))
    if do_widen:
        print("watchdog restored to PSC=%d RLD=%#x" % restore(o))
    o.cmd("rbp all")
    o.cmd("resume")
    try:
        out = "" if p is None else p.communicate(timeout=30)[0]
        print("config tool:", out.strip().splitlines()[-1])
    except subprocess.TimeoutExpired:
        p.kill(); print("config tool: timed out (expected on a long halt)")
    print("state:", o.state(), " boot_count:", o.rd("0x20000a94"))

main()
