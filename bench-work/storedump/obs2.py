import sys
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd
CTRL = 0x20000A90
for name, port in (("slave-f130", 6669), ("master-f103", 6668)):
    o = Ocd("127.0.0.1", port); w = o.read_words(CTRL, 38); o.close()
    print(name, " ".join(f"{i}:{v:#x}" for i, v in enumerate(w)))
    print(f"   ticks={w[2]} systick_entries={w[15]} lap_overruns={w[14]} "
          f"usart1={w[16]} dma={w[17]} peak_age={w[36]} age={w[7]}")
