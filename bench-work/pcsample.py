#!/usr/bin/env python3
"""PC-sample a RUNNING, DISARMED board: halt briefly, read PC, resume. Never used while armed."""
import subprocess, sys, collections
sys.path.insert(0,"tools")
from swdobs import Ocd
port=int(sys.argv[1]); n=int(sys.argv[2]) if len(sys.argv)>2 else 60
elf="target/thumbv7m-none-eabi/release/firmware"
syms=[]
for line in subprocess.check_output(["arm-none-eabi-nm","-S","-n",elf],text=True).splitlines():
    p=line.split()
    if len(p)==4:
        syms.append((int(p[0],16),int(p[1],16),p[3]))
def resolve(pc):
    for a,sz,nm in syms:
        if a<=pc<a+sz: return nm, pc-a
    return f"?0x{pc:08x}", 0
o=Ocd("127.0.0.1",port)
hits=collections.Counter()
for i in range(n):
    o.cmd("halt")
    pc=int(o.cmd("reg pc").split()[-1],0)
    o.cmd("resume")
    nm,off=resolve(pc)
    hits[nm]+=1
o.close()
for nm,c in hits.most_common(15):
    print(f"  {c:3d}/{n}  {nm}")
