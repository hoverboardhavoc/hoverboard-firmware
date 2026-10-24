#!/usr/bin/env python3
"""Scan target RAM for a 32-bit magic. scan.py <tclport> <magic-hex> [base] [len]"""
import socket, sys
PORT = int(sys.argv[1]); MAGIC = int(sys.argv[2], 16)
BASE = int(sys.argv[3], 16) if len(sys.argv) > 3 else 0x20000000
LEN  = int(sys.argv[4], 16) if len(sys.argv) > 4 else 0x2000
S = socket.create_connection(("127.0.0.1", PORT), timeout=60)
def cmd(c, t=120):
    S.settimeout(t); S.sendall(c.encode() + b"\x1a"); b = b""
    while not b.endswith(b"\x1a"): b += S.recv(65536)
    return b[:-1].decode(errors="replace")
out = cmd("mdw 0x%x %d" % (BASE, LEN // 4))
words = {}
for line in out.splitlines():
    if ":" not in line: continue
    addr, rest = line.split(":", 1)
    a = int(addr, 16)
    for i, w in enumerate(rest.split()):
        words[a + 4 * i] = int(w, 16)
hits = [a for a, w in words.items() if w == MAGIC]
for h in hits:
    print("0x%08x" % h)
