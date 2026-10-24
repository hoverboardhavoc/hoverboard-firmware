#!/usr/bin/env python3
"""One sample of the odometer gate's observables over an open OpenOCD TCL port."""
import socket, sys, time
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 6666
A = {
  "edges":    0x200008c0,
  "odo":      0x200008f8,
  "speed":    0x200008c8,
  "periods":  0x200008d4,
  "fault":    0x200008c4,
  "obs_cur":  0x200008a8,
  "vbatt":    0x200008dc,
  "boot":     0x20000a9c,
  "rstsck":   0x40021024,
  "cchp":     0x40012c44,
}
s = socket.create_connection(("127.0.0.1", PORT), timeout=20)
def cmd(c):
    s.sendall(c.encode() + b"\x1a"); b = b""
    while not b.endswith(b"\x1a"): b += s.recv(8192)
    return b[:-1].decode(errors="replace")
def rd(a):
    return int(cmd("mdw 0x%x" % a).split(":")[1].strip().split()[0], 16)
v = {k: rd(a) for k, a in A.items()}
s.close()
sp = v["speed"] - (1 << 32) if v["speed"] >= (1 << 31) else v["speed"]
print("t=%s edges=%-8d odo=%-8d speed=%-5d periods=%-10d fault=0x%x cur=%d vbatt=%d boot=%d rst=0x%08x moe=%d"
      % (time.strftime("%H:%M:%S"), v["edges"], v["odo"], sp, v["periods"], v["fault"],
         v["obs_cur"] & 0xFFFF, v["vbatt"], v["boot"], v["rstsck"], (v["cchp"] >> 15) & 1))
