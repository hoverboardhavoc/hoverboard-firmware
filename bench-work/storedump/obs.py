import sys, time
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd

CTRL = 0x20000A90
RC = 0x200008E4
for name, port in (("slave-f130", 6669), ("master-f103", 6668)):
    o = Ocd("127.0.0.1", port)
    w = o.read_words(CTRL, 38)
    rc = o.read_words(RC, 1)[0]
    print(f"{name}: magic=0x{w[0]:08x} boot={w[1]} ticks={w[2]} ctrl_ticks={w[4]} "
          f"peer_age={w[7]} age_peak={w[36]} reset_cause=0x{rc:08x}")
    t = o.read_words(CTRL+8, 1)[0]
    time.sleep(2)
    t2 = o.read_words(CTRL+8, 1)[0]
    print(f"   ticks over 2 s: {t} -> {t2} (delta {t2-t})")
    o.close()
