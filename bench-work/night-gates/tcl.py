#!/usr/bin/env python3
"""Minimal OpenOCD TCL-RPC client: tcl.py <port> <command> [...]"""
import socket, sys
PORT = int(sys.argv[1])
S = socket.create_connection(("127.0.0.1", PORT), timeout=30)
def cmd(c, t=120):
    S.settimeout(t); S.sendall(c.encode() + b"\x1a"); b = b""
    while not b.endswith(b"\x1a"): b += S.recv(8192)
    return b[:-1].decode(errors="replace")
for c in sys.argv[2:]:
    print(cmd(c))
