import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from parse import (crc16, parse_header, committed, rec_size, PAGE, PAGE_HEADER_LEN, MAGIC)

r = open(os.path.join(HERE, "slave-f130.bin"), "rb").read()

# --- walk page 1 as a record log starting at +8, no valid page header ---
print("PAGE 1 header bytes:", r[1024:1032].hex())
print("magic word:", hex(int.from_bytes(r[1024:1028],'little')), " want", hex(MAGIC))
print("seq/reserved:", int.from_bytes(r[1028:1030],'little'), int.from_bytes(r[1030:1032],'little'))
cur = 1024 + 8
p1 = []
while True:
    st, h = parse_header(r, cur)
    if st != "good":
        print(f"  page1 walk ends at +{cur-1024} ({st}) bytes={r[cur:cur+8].hex()}")
        break
    ok = committed(r, cur, h)
    p1.append((cur, h, ok, r[cur+8:cur+8+h['ln']]))
    print(f"  +{cur-1024:4d} id=0x{h['field_id']:02X} idx={h['index']} tag={h['tag']} "
          f"len={h['ln']} {'OK' if ok else 'TORN'} val={r[cur+8:cur+8+h['ln']].hex()}")
    cur += rec_size(h['ln'])
    if cur >= 2048: break
print("page1 records:", len(p1), "end offset +", cur-1024)

# --- simulate compact() of page 0 and compare bytes ---
def latest_pass(region, off):
    recs = []
    cur = off + PAGE_HEADER_LEN
    while True:
        st, h = parse_header(region, cur)
        if st != "good": break
        nxt = cur + rec_size(h['ln'])
        if nxt > off + PAGE: break
        recs.append((cur, h, committed(region, cur, h)))
        cur = nxt
        if cur == off + PAGE: break
    out = []
    for i, (here, h, ok) in enumerate(recs):
        if not ok: continue
        key = (h['field_id'], h['index'])
        later = any(k2 == key and ok2 for (_, hh, ok2) in recs[i+1:]
                    for k2 in [(hh['field_id'], hh['index'])])
        if later: continue
        out.append((here, h))
    return out

sim = latest_pass(r, 0)
buf = bytearray()
for here, h in sim:
    total = rec_size(h['ln'])
    buf += r[here:here+total]
print("\nsimulated compaction payload:", len(buf), "bytes ->", 8+len(buf), "frontier")
actual = r[1024+8:1024+8+len(buf)]
print("matches page1 bytes:", bytes(buf) == actual)
if bytes(buf) != actual:
    for i in range(0, len(buf), 16):
        a, b = bytes(buf[i:i+16]), actual[i:i+16]
        if a != b:
            print(f"  first diff at +{8+i}: sim={a.hex()} flash={b.hex()}")
            break
tail = r[1024+8+len(buf):2048]
print("bytes after the copy all 0xFF:", all(x == 0xFF for x in tail), "count", len(tail))
