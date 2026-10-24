#!/usr/bin/env python3
"""Drive a full-page compaction with NO debugger in the path: fill the active page through
CONFIG_WRITE until the next write has to compact, then watch boot_count and the page header."""
import socket, subprocess, sys, os, time
CFG = os.path.expanduser(
    "~/dev/hb-compact-wdg/crates/swd-bridge/target/aarch64-apple-darwin/release/swd-mailbox-config")
S = socket.create_connection(("127.0.0.1", 6666), timeout=15)
def cmd(c, t=60):
    S.settimeout(t); S.sendall(c.encode() + b"\x1a"); b = b""
    while not b.endswith(b"\x1a"): b += S.recv(8192)
    return b[:-1].decode(errors="replace")
def rd(a): return int(cmd("mdw %s" % a).split(":")[1].strip().split()[0], 16)
def write(arg="0x20=15005"):
    r = subprocess.run([CFG, "127.0.0.1:6666", arg], capture_output=True, text=True)
    return "CFG_OK" in r.stdout, r.stdout
cmd("rbp all")
def hdrs():
    return rd("0x0800f800"), rd("0x0800f804"), rd("0x0800fc00"), rd("0x0800fc04")
print("boot_count %d  page headers %s" % (rd("0x20000a94"), [hex(x) for x in hdrs()]))
n = int(sys.argv[1])
t0 = time.time()
for i in range(n):
    ok, out = write()
    if not ok: sys.exit("fill %d: %s" % (i, out[-300:]))
print("%d fills CFG_OK in %.0f s" % (n, time.time() - t0))
t = time.time()
ok, out = write()
dt = time.time() - t
print("the compacting write: %s in %.2f s" % ("CFG_OK" if ok else out[-400:], dt))
print("boot_count %d  page headers %s  ticks %d" % (rd("0x20000a94"), [hex(x) for x in hdrs()], rd("0x20000a98")))
