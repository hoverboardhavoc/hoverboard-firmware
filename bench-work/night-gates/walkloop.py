#!/usr/bin/env python3
"""Gate 3: the 28th-walk reset. Loop swd-mailbox-walk against the F130 slave and watch
boot_count, the reset-cause register and the mailbox refused-writes counter every walk."""
import socket, subprocess, sys, time
BIN = "/Users/hoverboardhavoc/dev/fw-night-gates/crates/swd-bridge/target/aarch64-apple-darwin/release/swd-mailbox-walk"
PORT, CTRL, RSTSCK = 6667, 0x20000A90, 0x40021024
N = int(sys.argv[1])
S = socket.create_connection(("127.0.0.1", PORT), timeout=30)
def cmd(c, t=60):
    S.settimeout(t); S.sendall(c.encode() + b"\x1a"); b = b""
    while not b.endswith(b"\x1a"): b += S.recv(65536)
    return b[:-1].decode(errors="replace")
def rdw(a, n=1):
    out = cmd("mdw 0x%x %d" % (a, n))
    return [int(x, 16) for line in out.splitlines() if ":" in line for x in line.split(":")[1].split()]
cmd("mww 0x%x 0" % RSTSCK)
b0 = rdw(CTRL + 4)[0]
print("start: boot_count=%d mailbox_drops=%d rstsck=%#010x" % (b0, rdw(CTRL + 34*4)[0], rdw(RSTSCK)[0]))
print("walk,ok,boot_count,mailbox_drops,rstsck,h2t_head,h2t_tail,t2h_head,t2h_tail,secs")
fails = 0
t00 = time.time()
for i in range(1, N + 1):
    t0 = time.time()
    r = subprocess.run([BIN, "127.0.0.1:%d" % PORT, "0x20000000", str(3100 + i)],
                       capture_output=True, text=True)
    ok = r.returncode == 0
    if not ok:
        fails += 1
        open("walk-fail-%d.txt" % i, "w").write(r.stdout + "\n--- stderr ---\n" + r.stderr)
    bc, md = rdw(CTRL + 4)[0], rdw(CTRL + 34*4)[0]
    rs = rdw(RSTSCK)[0]
    hdr = rdw(0x20000000, 16)
    print("%d,%d,%d,%d,%#010x,%d,%d,%d,%d,%.2f" % (i, ok, bc, md, rs, hdr[6], hdr[7], hdr[10], hdr[11], time.time() - t0), flush=True)
    if bc != b0:
        print("BOOT COUNTER MOVED at walk %d: %d -> %d" % (i, b0, bc), flush=True)
        b0 = bc
print("done: %d walks, %d tool failures, %.0f s" % (N, fails, time.time() - t00))
