#!/usr/bin/env python3
"""Fill the target->host mailbox ring and watch the refusal path: attach (bump epoch), flush once,
then stop draining and poll. On the pre-fix image this is where write_all panicked."""
import socket, sys, time
PORT, BASE, CTRL = 6667, 0x20000000, 0x20000A90
SECS = int(sys.argv[1]) if len(sys.argv) > 1 else 120
S = socket.create_connection(("127.0.0.1", PORT), timeout=30)
def cmd(c, t=60):
    S.settimeout(t); S.sendall(c.encode() + b"\x1a"); b = b""
    while not b.endswith(b"\x1a"): b += S.recv(65536)
    return b[:-1].decode(errors="replace")
def rd(a, n=1):
    out = cmd("mdw 0x%x %d" % (a, n))
    return [int(x, 16) for line in out.splitlines() if ":" in line for x in line.split(":")[1].split()]
h = rd(BASE, 12)
print("header before: epoch=%d epoch_ack=%d h2t head/tail=%d/%d t2h head/tail=%d/%d cap=%d"
      % (h[2], h[3], h[6], h[7], h[10], h[11], h[9]))
ep = h[2] + 1
cmd("mww 0x%x %d" % (BASE + 8, ep))
t0 = time.time()
while rd(BASE + 12)[0] != ep and time.time() - t0 < 10: time.sleep(0.1)
print("epoch -> %d, epoch_ack=%d after %.2f s" % (ep, rd(BASE + 12)[0], time.time() - t0))
cmd("mww 0x%x %d" % (BASE + 44, rd(BASE + 40)[0]))   # t2h_tail := t2h_head, the one flush
print("secs,t2h_head,t2h_tail,t2h_used,mailbox_drops,boot_count,ctrl_ticks")
t0 = time.time()
while time.time() - t0 < SECS:
    hd, tl = rd(BASE + 40)[0], rd(BASE + 44)[0]
    md, bc = rd(CTRL + 34*4)[0], rd(CTRL + 4)[0]
    tk = rd(CTRL + 8)[0]
    print("%.0f,%d,%d,%d,%d,%d,%d" % (time.time() - t0, hd, tl, hd - tl, md, bc, tk), flush=True)
    time.sleep(3)
