#!/usr/bin/env python3
"""Read words over an already-open OpenOCD TCL port: mdw.py <port> <addr> [count]"""
import socket, sys
port=int(sys.argv[1]); addr=sys.argv[2]; cnt=sys.argv[3] if len(sys.argv)>3 else "1"
s=socket.create_connection(("127.0.0.1",port),timeout=15)
def cmd(c):
    s.sendall(c.encode()+b"\x1a"); b=b""
    while not b.endswith(b"\x1a"): b+=s.recv(8192)
    return b[:-1].decode(errors="replace")
print(cmd("mdw %s %s" % (addr,cnt)).strip())
s.close()
