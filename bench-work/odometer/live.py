import struct
for name in ("master","slave"):
    data=open("bench/pre-store-%s.bin" % name,"rb").read()
    for p in range(2):
        pg=data[p*1024:(p+1)*1024]
        if pg[:4]!=b"ROTS":
            continue
        off=8
        live={}
        while off+8<=1024:
            fid,idx,typ,rsv,ln=struct.unpack_from("<BBBBH",pg,off)
            if fid==0xFF and idx==0xFF:
                break
            pad=ln+(ln&1)
            live[(fid,idx)]=8+pad+2
            off+=8+pad+2
        tot=sum(live.values())
        print("%s page %d: %d live keys, %d B live, frontier %d, free %d" % (name,p,len(live),tot,off,1024-off))
        print("   now: %d odometer records fit; after a compaction: %d B free -> %d per cycle" % ((1024-off)//14, 1024-8-tot, (1024-8-tot)//14))
