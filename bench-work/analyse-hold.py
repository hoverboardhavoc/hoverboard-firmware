#!/usr/bin/env python3
"""Correlate comms_loss trips with main-thread absences, catch-up bursts and ring laps."""
import csv, sys
F = sys.argv[1] if len(sys.argv) > 1 else "bench-work/hold.csv"
rows = {"M": [], "S": []}
for r in csv.DictReader(open(F)):
    rows[r["b"]].append(r)

def g(r, i): return int(r[f"w{i}"])

for b, rs in rows.items():
    print(f"\n================ board {b}  ({len(rs)} samples, {float(rs[-1]['t']):.1f}s) ================")
    # publish events: a new publish is any row whose control_ticks (w4) changed
    pubs = []
    last = None
    for r in rs:
        c = g(r, 4)
        if c != last:
            pubs.append(r)
            last = c
    print(f"publishes observed: {len(pubs)}")
    # intervals between observed publishes
    ivs = []
    for p, q in zip(pubs, pubs[1:]):
        dw = float(q["wall"]) - float(p["wall"])
        ivs.append(dict(t=float(q["t"]), wall=float(q["wall"]), dw=dw,
                        dtick=g(q, 2) - g(p, 2), dctrl=g(q, 4) - g(p, 4),
                        ddisp=g(q, 3) - g(p, 3),
                        dlap=g(q, 14) - g(p, 14), dble=(g(q, 29) & 0xFFFF) - (g(p, 29) & 0xFFFF),
                        cA=g(q, 7), dA=g(q, 8),
                        cl=(g(q, 27) & 0xFF) - (g(p, 27) & 0xFF),
                        arm=g(q, 9) - g(p, 9),
                        mode=(g(q, 11) >> 16) & 0xFF, moe=(g(q, 11) >> 24) & 0xFF,
                        lvl=(g(q, 12) >> 24) & 0xFF))
    armed = [i for i in ivs if i["arm"]]
    print(f"arms: {sum(i['arm'] for i in ivs)}   comms_loss transitions: {sum(i['cl'] for i in ivs)}"
          f"   uart laps: {sum(i['dlap'] for i in ivs)}   ble laps: {sum(i['dble'] for i in ivs)}")
    miss = sum(max(0, i["dtick"] - i["dctrl"]) for i in ivs)
    span = ivs[-1]["wall"] - ivs[0]["wall"]
    print(f"missed control slots: {miss} over {span:.1f}s = {miss/span:.1f}/s")
    # absences: an observed publish interval with real time >> one tick and few control runs
    ab = [i for i in ivs if i["dw"] > 0.035 and i["dctrl"] <= 3]
    print(f"\nabsences (>35 ms of wall time with <=3 control runs): {len(ab)}  "
          f"= {len(ab)/span:.2f}/s")
    if ab:
        ls = sorted(i["dw"] for i in ab)
        print(f"   absence length ms: min {ls[0]*1000:.0f} med {ls[len(ls)//2]*1000:.0f} "
              f"max {ls[-1]*1000:.0f}")
    # catch-up bursts: control runs far faster than 250/s
    cu = [i for i in ivs if i["dw"] > 0.004 and i["dctrl"] / i["dw"] > 400]
    print(f"catch-up bursts (control task > 400/s): {len(cu)}")
    if cu:
        ks = sorted(i["dctrl"] for i in cu)
        print(f"   runs per burst: min {ks[0]} med {ks[len(ks)//2]} max {ks[-1]}; "
              f"peak rate {max(i['dctrl']/i['dw'] for i in cu):.0f}/s")
    # every comms_loss assertion, with the 400 ms before it
    trips = [k for k, i in enumerate(ivs) if i["cl"] > 0 and i["lvl"] & 1]
    print(f"\ncomms_loss ASSERTIONS seen with the level still up: {len(trips)}")
    for k in trips[:14]:
        w0 = ivs[k]["wall"] - 0.40
        pre = [i for i in ivs if w0 <= i["wall"] <= ivs[k]["wall"]]
        print(f"  t={ivs[k]['t']:7.2f} cA={ivs[k]['cA']:3d} | prior 400 ms: "
              f"maxAbsence={max((i['dw'] for i in pre), default=0)*1000:5.0f}ms "
              f"missed={sum(max(0,i['dtick']-i['dctrl']) for i in pre):3d} "
              f"peakCtrlRate={max((i['dctrl']/i['dw'] for i in pre if i['dw']>0.004), default=0):5.0f}/s "
              f"uartLaps={sum(i['dlap'] for i in pre)} bleLaps={sum(i['dble'] for i in pre)}")
    # how cyclic_age gets to the threshold: the biggest cA values and what preceded
    big = sorted(ivs, key=lambda i: -i["cA"])[:8]
    print("\nhighest cyclic_age samples:")
    for i in big:
        print(f"  t={i['t']:7.2f} cA={i['cA']:3d} dw={i['dw']*1000:5.0f}ms dctrl={i['dctrl']:3d} "
              f"dtick={i['dtick']:3d} ddisp={i['ddisp']:5d} dlap={i['dlap']} mode={i['mode']} moe={i['moe']}")
