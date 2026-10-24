#!/usr/bin/env python3
"""Decode the live (newest-per-key) record set of a store region dump."""
import sys
def crc16(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
    return c
def live(path, ps=1024):
    buf=open(path,'rb').read()
    best=None
    for off in (0,ps):
        if int.from_bytes(buf[off:off+4],'little')==0x53544F52:
            seq=int.from_bytes(buf[off+4:off+6],'little')
            if seq!=0xFFFF and (best is None or seq>best[1]): best=(off,seq)
    off,seq=best
    cur=off+8; end=off+ps; out={}
    while cur<end:
        h=buf[cur:cur+8]
        if len(h)<8 or h[0]==0xFF: break
        if crc16(h[:6])!=int.from_bytes(h[6:8],'little'): break
        ln=int.from_bytes(h[4:6],'little'); pad=ln+(ln&1)
        if crc16(bytes(h[:6])+bytes(buf[cur+8:cur+8+ln]))==int.from_bytes(buf[cur+8+pad:cur+10+pad],'little'):
            out[(h[0],h[1],h[2])]=bytes(buf[cur+8:cur+8+ln])
        cur+=8+pad+2
    return out
a=live(sys.argv[1]); b=live(sys.argv[2])
ka,kb=set(a),set(b)
print("A live keys %d, B live keys %d" % (len(a),len(b)))
for k in sorted(ka|kb):
    va,vb=a.get(k),b.get(k)
    if va!=vb: print("  DIFF id=%#04x:%d type=%d  A=%s  B=%s" % (k[0],k[1],k[2],va.hex() if va else None, vb.hex() if vb else None))
print("identical live set" if a==b else "LIVE SETS DIFFER")
