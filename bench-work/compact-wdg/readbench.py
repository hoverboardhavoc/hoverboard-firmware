#!/usr/bin/env python3
"""Where do compaction's cycles go? Run three identical 200-iteration loops from RAM, differing
only in what they read, and read CYCCNT across each. The board is RESET afterwards."""
import socket, time
S = socket.create_connection(("127.0.0.1", 6666), timeout=15)
def cmd(c, t=60):
    S.settimeout(t); S.sendall(c.encode() + b"\x1a"); b = b""
    while not b.endswith(b"\x1a"): b += S.recv(8192)
    return b[:-1].decode(errors="replace")
def rd(a): return int(cmd("mdw %s" % a).split(":")[1].strip().split()[0], 16)
def wr(a, v): cmd("mww %s %s" % (a, v))
def state(): return cmd("stm32f1x.cpu curstate").strip()

BASE = 0x20001400
# loop:  ldrb r2,[r0,r1] ; adds r1,#1 ; cmp r1,#200 ; bne loop ; bkpt
CODE_LOAD = [0x31015C42, 0xD1FA29C8, 0xBE00BE00]
# loop:  adds r1,#1 ; cmp r1,#200 ; bne loop ; bkpt   (no memory access)
CODE_NONE = [0x29C83101, 0xBE00D1FB, 0xBE00BE00]

def run(code, r0, label, primask=0):
    cmd("halt")
    for i, w in enumerate(code):
        wr(hex(BASE + 4 * i), hex(w))
    cmd("reg r0 0x%x" % r0); cmd("reg r1 0")
    cmd("reg primask %d" % primask)
    cmd("reg pc 0x%x" % BASE)
    c0 = rd("0xE0001004")
    cmd("resume")
    t = time.time()
    while state() != "halted":
        if time.time() - t > 20: return print("%-34s TIMEOUT" % label)
    c1 = rd("0xE0001004")
    n = (c1 - c0) & 0xFFFFFFFF
    print("%-34s %8d cycles  %6.1f cycles/iteration" % (label, n, n / 200.0))

run(CODE_NONE, 0, "no memory access, IRQs on")
run(CODE_NONE, 0, "no memory access, IRQs OFF", primask=1)
run(CODE_LOAD, 0x20000400, "read RAM, IRQs OFF", primask=1)
run(CODE_LOAD, 0x08001000, "read flash <32 KiB, IRQs OFF", primask=1)
run(CODE_LOAD, 0x0800C000, "read flash >32 KiB, IRQs OFF", primask=1)
run(CODE_LOAD, 0x0800FC00, "read the erased store page, OFF", primask=1)
run(CODE_LOAD, 0x0800F800, "read the written store page, OFF", primask=1)
run(CODE_LOAD, 0x0800F800, "read the written store page, ON")
print("resetting the board")
cmd("reset run")
