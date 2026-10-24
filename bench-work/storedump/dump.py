import sys, os
sys.path.insert(0, "/Users/hoverboardhavoc/dev/hoverboard-firmware/tools")
from swdobs import Ocd

BASE = 0x0800F800
N = 512
for name, port in (("slave-f130", 6669), ("master-f103", 6668)):
    o = Ocd("127.0.0.1", port)
    words = []
    for i in range(0, N, 64):
        words += o.read_words(BASE + 4*i, 64)
    o.close()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), name + ".bin")
    with open(out, "wb") as f:
        for w in words:
            f.write(w.to_bytes(4, "little"))
    print(name, "wrote", out, "words", len(words))
