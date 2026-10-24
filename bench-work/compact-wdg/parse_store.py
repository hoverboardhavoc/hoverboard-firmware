#!/usr/bin/env python3
"""Parse a dumped store region (two pages) and report both sides."""
import sys

def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def page(buf, off, ps, verbose):
    magic = int.from_bytes(buf[off:off+4], 'little')
    seq = int.from_bytes(buf[off+4:off+6], 'little')
    print(f"page @{off:#06x}: magic={magic:#010x} seq={seq:#06x}", end="")
    if magic != 0x53544F52 or seq == 0xFFFF:
        print("  (not a valid side)")
        return
    print()
    cur = off + 8
    end = off + ps
    n = 0
    live = {}
    while cur < end:
        h = buf[cur:cur+8]
        if len(h) < 8 or h[0] == 0xFF:
            print(f"  frontier at +{cur-off} ({end-cur} B free)")
            break
        if crc16_modbus(h[:6]) != int.from_bytes(h[6:8], 'little'):
            print(f"  TORN header at +{cur-off}")
            break
        ln = int.from_bytes(h[4:6], 'little')
        padded = ln + (ln & 1)
        total = 8 + padded + 2
        vc = int.from_bytes(buf[cur+8+padded:cur+10+padded], 'little')
        ok = crc16_modbus(bytes(h[:6]) + bytes(buf[cur+8:cur+8+ln])) == vc
        if verbose:
            print(f"  +{cur-off:4d} id={h[0]:#04x}:{h[1]} type={h[2]} len={ln} size={total} committed={ok}")
        if ok:
            live[(h[0], h[1])] = cur - off
        n += 1
        cur += total
    else:
        print(f"  page exactly full at +{cur-off}")
    print(f"  records={n} distinct live keys={len(live)}")

buf = open(sys.argv[1], 'rb').read()
ps = int(sys.argv[2]) if len(sys.argv) > 2 else 1024
verbose = len(sys.argv) > 3
for off in (0, ps):
    page(buf, off, ps, verbose)
