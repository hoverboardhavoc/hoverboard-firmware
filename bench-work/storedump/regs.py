import sys
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd

READS = [
    ("FMC WS/KEY/OBKEY/STAT/CTL/ADDR", 0x40022000, 8),
    ("FMC OBSTAT/WP", 0x4002201C, 2),
    ("option bytes 0x1FFFF800", 0x1FFFF800, 4),
    ("flash density 0x1FFFF7E0", 0x1FFFF7E0, 1),
    ("uid 0x1FFFF7AC", 0x1FFFF7AC, 3),
]
for name, port in (("slave-f130", 6669), ("master-f103", 6668)):
    print("="*70); print(name)
    o = Ocd("127.0.0.1", port)
    for label, addr, n in READS:
        try:
            w = o.read_words(addr, n)
            print(f"  {label:34s} @0x{addr:08X}: " + " ".join(f"{v:08x}" for v in w))
        except Exception as e:
            print(f"  {label:34s} @0x{addr:08X}: ERROR {e}")
    o.close()
