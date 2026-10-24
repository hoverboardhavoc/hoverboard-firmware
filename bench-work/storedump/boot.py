import sys, time
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd
CTRL = 0x20000A90; RC = 0x200008E4
s = Ocd("127.0.0.1", 6669)
t0 = time.time()
while time.time() - t0 < 90:
    w = s.read_words(CTRL, 38); rc = s.read_words(RC, 1)[0]
    f = s.read_words(0x0800FC00, 2)
    print(f"{time.time()-t0:5.1f} boot={w[1]} tick={w[2]} dispatch={w[3]} rc=0x{rc:08x} "
          f"page1hdr={f[0]:08x} {f[1]:08x}", flush=True)
    time.sleep(5)
s.close()
