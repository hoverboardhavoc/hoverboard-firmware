import sys, time
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd
CTRL = 0x20000A90
s = Ocd("127.0.0.1", 6669)
t0 = time.time(); prev = None
while time.time() - t0 < 170:
    w = s.read_words(CTRL, 4)
    p1 = s.read_words(0x0800FC00, 2)
    p0 = s.read_words(0x0800F800, 2) + s.read_words(0x0800FBF8, 2)
    cur = (w[1], p1[0], p1[1], p0[0], p0[1], p0[2], p0[3])
    if prev is None or cur != prev or int(time.time()-t0) % 20 == 0:
        print(f"{time.time()-t0:6.1f} boot={w[1]} tick={w[2]:7d} p1={p1[0]:08x},{p1[1]:08x} "
              f"p0hdr={p0[0]:08x},{p0[1]:08x} p0tail={p0[2]:08x},{p0[3]:08x}", flush=True)
        prev = cur
    time.sleep(0.5)
s.close()
