#!/usr/bin/env python3
"""Read CTRL_OBS word 37 (the foot-pad block) n times, spaced. padread.py <port> <n> <gap>"""
import socket, sys, time
PORT, N, GAP = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
CTRL = 0x20000A90
S = socket.create_connection(("127.0.0.1", PORT), timeout=30)
def cmd(c, t=60):
    S.settimeout(t); S.sendall(c.encode() + b"\x1a"); b = b""
    while not b.endswith(b"\x1a"): b += S.recv(65536)
    return b[:-1].decode(errors="replace")
def rd(a, n=1):
    out = cmd("mdw 0x%x %d" % (a, n))
    return [int(x, 16) for line in out.splitlines() if ":" in line for x in line.split(":")[1].split()]
for i in range(N):
    w = rd(CTRL + 37*4)[0]
    print("t=%3.0fs  word37 = 0x%08x   state=0x%02x changes=%d held=%d   boot_count=%d uptime_ticks=%d"
          % (i*GAP, w, w & 0xFF, (w >> 8) & 0xFF, (w >> 16) & 0xFF, rd(CTRL+4)[0], rd(CTRL+8)[0]), flush=True)
    if i < N - 1: time.sleep(GAP)
