#!/usr/bin/env python3
"""Program a store-region image back by hand over SWD (ascending halfwords through the FMC).

OpenOCD's `flash write_bank` corrupts the tail of this region on these parts (measured: the last
84 bytes of a 2 KiB write came back as data nobody wrote), so the restore drives the FMC itself:
unlock, PG, ascending halfword writes, BUSY/flag check, lock. Ascending order is what the F1x0
requires (a halfword is closed by its successor word's low halfword). Halfwords that are 0xFFFF in
the image are skipped: the page is erased, so they already read 0xFFFF.
"""
import socket, sys
IMG, BASE = sys.argv[1], int(sys.argv[2], 16)
S = socket.create_connection(("127.0.0.1", int(sys.argv[3])), timeout=20)
def cmd(c, t=90):
    S.settimeout(t); S.sendall(c.encode() + b"\x1a"); b = b""
    while not b.endswith(b"\x1a"): b += S.recv(8192)
    return b[:-1].decode(errors="replace")
def rd(a): return int(cmd("mdw %s" % a).split(":")[1].strip().split()[0], 16)
FMC = 0x40022000
data = open(IMG, "rb").read()
print(cmd("halt"))
print("erase:", cmd("flash erase_address unlock 0x%x 0x%x" % (BASE, len(data))).strip())
cmd("mww 0x%x 0x45670123" % (FMC + 4)); cmd("mww 0x%x 0xcdef89ab" % (FMC + 4))
cmd("mww 0x%x 0x1" % (FMC + 0x10))          # PG
batch, n = [], 0
for off in range(0, len(data), 2):
    hw = data[off] | (data[off + 1] << 8)
    if hw == 0xFFFF: continue
    batch.append("mwh 0x%x 0x%04x" % (BASE + off, hw)); n += 1
    if len(batch) == 24:
        cmd("; ".join(batch)); batch = []
if batch: cmd("; ".join(batch))
stat = rd(hex(FMC + 0xC))
cmd("mww 0x%x 0x34" % (FMC + 0xC))           # clear ENDF|WPERR|PGERR
cmd("mww 0x%x 0x0" % (FMC + 0x10))           # clear PG
cmd("mww 0x%x 0x80" % (FMC + 0x10))          # lock
print("programmed %d halfwords, FMC_STAT = %#x (PGERR b2 / WPERR b4 must be clear)" % (n, stat))
cmd("dump_image ./.verify.bin 0x%x %d" % (BASE, len(data)))
got = open("./.verify.bin", "rb").read()
print("VERIFY: byte-exact" if got == data else "VERIFY FAILED: %d bytes differ" %
      sum(1 for x, y in zip(got, data) if x != y))
