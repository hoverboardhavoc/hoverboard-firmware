import sys
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd
img = open("/Users/hoverboardhavoc/dev/hoverboard-firmware/bench-work/storedump/resetcause.bin","rb").read()
o = Ocd("127.0.0.1", 6669)
bad = 0; checked = 0
for off in (0, 0x1000, 0x6548, 0x8cb0, 0xe000):
    n = 64
    w = o.read_words(0x08000000+off, n)
    exp = [int.from_bytes(img[off+4*i:off+4*i+4],"little") for i in range(n)]
    checked += n
    d = sum(1 for a,b in zip(w,exp) if a!=b)
    bad += d
    print(f"0x{0x08000000+off:08x}: {n} words, {d} mismatches")
o.close()
print("total", checked, "words,", bad, "mismatches; image size", len(img))
