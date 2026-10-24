#!/usr/bin/env python3
"""Send commands to an OpenOCD TCL RPC port. Usage: tcl.py <port> <cmd> [cmd ...]"""
import socket, sys

def rpc(sock, cmd, timeout=120.0):
    sock.settimeout(timeout)
    sock.sendall(cmd.encode() + b"\x1a")
    buf = b""
    while not buf.endswith(b"\x1a"):
        chunk = sock.recv(8192)
        if not chunk:
            break
        buf += chunk
    return buf[:-1].decode(errors="replace")

def main():
    port = int(sys.argv[1])
    s = socket.create_connection(("127.0.0.1", port), timeout=10)
    for cmd in sys.argv[2:]:
        print(f"> {cmd}")
        print(rpc(s, cmd))
    s.close()

main()
