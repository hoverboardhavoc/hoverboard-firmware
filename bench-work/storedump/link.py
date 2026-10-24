import sys, time
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd
CTRL = 0x20000A90
m = Ocd("127.0.0.1", 6668); s = Ocd("127.0.0.1", 6669)
t0 = time.time()
while time.time() - t0 < 45:
    mw = m.read_words(CTRL, 38); sw = s.read_words(CTRL, 38)
    print(f"{time.time()-t0:5.1f} M age={mw[7]:6d} peak={mw[36]:6d} lap={mw[14]} tick={mw[2]} | "
          f"S age={sw[7]:4d} peak={sw[36]} lap={sw[14]} tick={sw[2]} tx_to={sw[35]:#x}", flush=True)
    time.sleep(3)
m.close(); s.close()
