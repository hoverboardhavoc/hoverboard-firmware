import sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = 1024
PAGE_HEADER_LEN = 8
HEADER_LEN = 8
VAL_CRC_LEN = 2
HDR_COVERED = 6
MAGIC = 0x53544F52
BLANK = 0xFF
MAX_RECORD = 272

def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def padded(n): return n + (n & 1)
def rec_size(n): return HEADER_LEN + padded(n) + VAL_CRC_LEN

def parse_header(r, off):
    if off + HEADER_LEN > len(r): return ("torn", None)
    h = r[off:off+HEADER_LEN]
    if h[0] == BLANK: return ("blank", None)
    stored = int.from_bytes(h[6:8], "little")
    if crc16(h[:HDR_COVERED]) != stored: return ("torn", None)
    return ("good", dict(field_id=h[0], index=h[1], tag=h[2], res=h[3],
                         ln=int.from_bytes(h[4:6], "little")))

def committed(r, off, h):
    ln = h["ln"]
    vo = off + HEADER_LEN + padded(ln)
    if vo + VAL_CRC_LEN > len(r): return False
    c = crc16(r[off:off+HDR_COVERED])
    c = crc16(r[off+HEADER_LEN:off+HEADER_LEN+ln], c)
    return c == int.from_bytes(r[vo:vo+VAL_CRC_LEN], "little")

def page_hdr(r, off):
    m = int.from_bytes(r[off:off+4], "little")
    if m != MAGIC: return None
    return int.from_bytes(r[off+4:off+6], "little"), int.from_bytes(r[off+6:off+8], "little")

def erased(r, off, n): return all(b == 0xFF for b in r[off:off+n])

def report(name, path):
    r = open(path, "rb").read()
    print("=" * 78)
    print(name, path, len(r), "bytes")
    for p in (0, 1):
        off = p * PAGE
        ph = page_hdr(r, off)
        nfree = sum(1 for b in r[off:off+PAGE] if b == 0xFF)
        print(f"\n-- page {p} @0x{0x0800F800+off:08X}  header={ph}  raw={r[off:off+8].hex()}  "
              f"0xFF bytes={nfree}/{PAGE}  fully_erased={erased(r,off,PAGE)}")
        if ph is None:
            # show first non-FF content if any
            nz = [i for i in range(PAGE) if r[off+i] != 0xFF]
            if nz:
                print(f"   NOT erased, {len(nz)} non-FF bytes, first at +{nz[0]}, last at +{nz[-1]}")
                print("   first 64 bytes:", r[off:off+64].hex())
            continue
        cursor = off + PAGE_HEADER_LEN
        n_good = n_torn_payload = 0
        latest = {}
        while True:
            st, h = parse_header(r, cursor)
            if st == "blank":
                print(f"   frontier (blank) at +{cursor-off} (abs 0x{0x0800F800+cursor:08X}); "
                      f"free bytes to page end = {off+PAGE-cursor}")
                break
            if st == "torn":
                print(f"   *** TORN HEADER at +{cursor-off} (abs 0x{0x0800F800+cursor:08X}): "
                      f"{r[cursor:cursor+8].hex()}")
                break
            here = cursor
            nxt = here + rec_size(h["ln"])
            if nxt > off + PAGE:
                print(f"   *** record at +{here-off} len={h['ln']} runs past page end -> TornHeader")
                break
            ok = committed(r, here, h)
            val = r[here+HEADER_LEN:here+HEADER_LEN+h["ln"]]
            if ok:
                n_good += 1
                latest[(h["field_id"], h["index"])] = (here, h, val)
            else:
                n_torn_payload += 1
            print(f"   +{here-off:4d} id=0x{h['field_id']:02X} idx={h['index']:3d} "
                  f"tag={h['tag']} len={h['ln']:3d} size={rec_size(h['ln']):3d} "
                  f"{'OK ' if ok else 'TORN-PAYLOAD'} val={val.hex()}")
            cursor = nxt
            if cursor == off + PAGE:
                print(f"   page EXACTLY full: frontier = page end (+{PAGE})")
                break
        print(f"   records: {n_good} committed, {n_torn_payload} torn-payload, "
              f"{len(latest)} distinct keys")
        live_bytes = sum(rec_size(h['ln']) for (_, h, _) in latest.values())
        print(f"   live bytes after compaction would be {PAGE_HEADER_LEN + live_bytes}")

for n, f in (("SLAVE F130 (port 6669)", "slave-f130.bin"),
             ("MASTER F103 (port 6668)", "master-f103.bin")):
    report(n, os.path.join(HERE, f))
