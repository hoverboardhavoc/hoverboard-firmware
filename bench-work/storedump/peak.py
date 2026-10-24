import sys, time
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd
CTRL = 0x20000A90
m = Ocd("127.0.0.1", 6668)
def peak(): return m.read_words(CTRL + 4*36, 1)[0]
print("start peak", peak(), flush=True)
t0 = time.time()
while time.time() - t0 < 60: time.sleep(5)
print("after 60 s idle (no slave polling):", peak(), flush=True)
s = Ocd("127.0.0.1", 6669)
t0 = time.time(); n = 0
while time.time() - t0 < 60:
    s.read_words(0x0800FC00, 4); n += 1
s.close()
print(f"after 60 s of {n} slave flash reads:", peak(), flush=True)
m.close()
