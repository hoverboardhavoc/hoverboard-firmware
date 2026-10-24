#!/usr/bin/env python3
"""Parse a store-region dump: page headers, live records, frontier, free space."""
import sys, struct
data = open(sys.argv[1], 'rb').read()
PAGE = 1024
HDR = 8
for p in range(len(data)//PAGE):
    pg = data[p*PAGE:(p+1)*PAGE]
    print(f"--- page {p} @0x{0x0800F800 + p*PAGE:08x} header {pg[:8].hex()}")
    off = HDR
    live = {}
    n = 0
    used = HDR
    while off + HDR <= PAGE:
        fid, idx, typ, rsv, ln = struct.unpack_from('<BBBBH', pg, off)
        hcrc = struct.unpack_from('<H', pg, off+6)[0]
        if fid == 0xFF and idx == 0xFF:
            break
        pad = ln + (ln & 1)
        rec = HDR + pad + 2
        val = pg[off+HDR:off+HDR+ln]
        live[(fid, idx)] = (typ, val)
        n += 1
        off += rec
        used = off
    print(f"    {n} records, frontier {used} B, free {PAGE-used} B")
    for (fid, idx), (typ, val) in sorted(live.items()):
        print(f"      0x{fid:02x}:{idx} type {typ} = {val.hex()}")
