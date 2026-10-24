import sys, time
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd
o = Ocd("127.0.0.1", 6669)
t0 = time.time()
prev = None
n = 0
while time.time() - t0 < 100:
    w = o.read_words(0x0800FC00, 4) + o.read_words(0x0800FDB8, 2)
    n += 1
    if w != prev:
        print(f"{time.time()-t0:7.2f} " + " ".join(f"{x:08x}" for x in w), flush=True)
        prev = w
print("samples", n, "elapsed", round(time.time()-t0,1))
o.close()
