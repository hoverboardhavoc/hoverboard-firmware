#!/usr/bin/env python3
"""Print the live key/value set of a dumped store region."""
import sys
def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc
def page(buf, off, ps):
    if int.from_bytes(buf[off:off+4],'little') != 0x53544F52: return None, {}
    seq = int.from_bytes(buf[off+4:off+6],'little')
    if seq == 0xFFFF: return None, {}
    cur, end, live = off+8, off+ps, {}
    while cur < end:
        h = buf[cur:cur+8]
        if len(h) < 8 or h[0] == 0xFF: break
        if crc16(h[:6]) != int.from_bytes(h[6:8],'little'): break
        ln = int.from_bytes(h[4:6],'little'); padded = ln + (ln & 1)
        val = bytes(buf[cur+8:cur+8+ln])
        vc = int.from_bytes(buf[cur+8+padded:cur+10+padded],'little')
        if crc16(bytes(h[:6]) + val) == vc:
            live[(h[0], h[1])] = (h[2], val)
        cur += 8 + padded + 2
    return seq, live
buf = open(sys.argv[1],'rb').read()
ps = 1024
best = (None, {})
for off in (0, ps):
    seq, live = page(buf, off, ps)
    if seq is not None and (best[0] is None or seq > best[0]): best = (seq, live)
print("active seq", best[0], "live keys", len(best[1]))
for (fid, idx) in sorted(best[1]):
    t, v = best[1][(fid, idx)]
    iv = int.from_bytes(v, 'little') if len(v) <= 4 else None
    print("  0x%02x:%d type=%d raw=%s%s" % (fid, idx, t, v.hex(), "" if iv is None else "  = %d (0x%x)" % (iv, iv)))
