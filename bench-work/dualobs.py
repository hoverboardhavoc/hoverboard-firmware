#!/usr/bin/env python3
"""Sample CTRL_OBS on BOTH offroad boards at once while the owner drives from the app.

Never halts either core: read_memory on the running target, the way the SWD mailbox reads.
Attaches to two already-running OpenOCD TCL ports (the patched elaphureLink build).
"""
import argparse, csv, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from swdobs import CTRL_MAGIC, DEFAULT_ELF, Ocd, resolve_symbols, s16, s32  # noqa: E402

W = dict(magic=0, boot=1, tick=2, dispatch=3, ctrl_ticks=4, input_ticks=5, pitch=6,
         cyclic_age=7, drive_age=8, inits=9, shutdowns=10, torque_mode=11, sub_flags=12,
         link_lerr=13, link_lap=14, systick_isr=15, usart1_isr=16, dma_isr=17, roll=18,
         periods=19, motor_state=20, duty01=21, duty2_angle=22, fault=23, speed=24, cal=25,
         gating=26, ev_lo=27, ev_hi=28, ble_rx=29, stack=30, current=31, battery=32,
         arm_ref=33, mbox=34)
FAULTS = [(1 << 0, "HALL"), (1 << 1, "DUTY"), (1 << 2, "STALE"), (1 << 3, "CAL"), (1 << 4, "METH")]
FLAGS = [(1, "imu_cfg"), (2, "imu_live"), (4, "comms_loss"), (8, "mode_fault"),
         (16, "imu_loss"), (32, "motor_fault")]
EV = ["comms_loss", "stop_all", "imu_loss", "motor_fault", "latchA", "latchB", "mode_fault",
      "power_req"]


def decode(words):
    d = {k: words[i] for k, i in W.items()}
    d["torque"] = s16(d["torque_mode"] & 0xFFFF)
    d["mode"] = (d["torque_mode"] >> 16) & 0xFF
    d["moe"] = (d["torque_mode"] >> 24) & 0xFF
    d["sub"] = d["sub_flags"] & 0xFF
    d["cmode"] = (d["sub_flags"] >> 8) & 0xFF
    d["flags"] = (d["sub_flags"] >> 16) & 0xFF
    d["levels"] = (d["sub_flags"] >> 24) & 0xFF
    d["faultbits"] = d["fault"] & 0xFFFF
    d["dwell"] = (d["fault"] >> 16) & 0xFFFF
    d["hall"] = d["motor_state"] & 0xFF
    d["enables"] = (d["motor_state"] >> 8) & 0xFF
    d["method"] = (d["motor_state"] >> 16) & 0xFF
    d["mflags"] = (d["motor_state"] >> 24) & 0xFF
    d["d0"] = s16(d["duty01"] & 0xFFFF)
    d["d1"] = s16((d["duty01"] >> 16) & 0xFFFF)
    d["speed_s"] = s32(d["speed"])
    d["peak"] = s16(d["current"] & 0xFFFF)
    d["chopped"] = (d["current"] >> 16) & 0xFF
    d["trips"] = (d["current"] >> 24) & 0xFF
    d["ble_lap"] = d["ble_rx"] & 0xFFFF
    d["ble_lerr"] = (d["ble_rx"] >> 16) & 0xFFFF
    d["counts"] = [(d["ev_lo"] >> (8 * i)) & 0xFF for i in range(4)] + \
                  [(d["ev_hi"] >> (8 * i)) & 0xFF for i in range(4)]
    return d


def names(mask, table):
    return ",".join(n for b, n in table if mask & b) or "-"


def lvlnames(mask):
    return ",".join(EV[i] for i in range(8) if mask & (1 << i)) or "-"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=8.0)
    ap.add_argument("--secs", type=float, default=0)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--master", default="127.0.0.1:6668")
    ap.add_argument("--slave", default="127.0.0.1:6667")
    ap.add_argument("--quiet", action="store_true", help="print only changes/events")
    a = ap.parse_args()

    addr = resolve_symbols(a.elf, "dualobs")["CTRL_OBS"]
    conns = {}
    for nm, ep in (("M", a.master), ("S", a.slave)):
        h, p = ep.split(":")
        o = Ocd(h, int(p))
        m = o.read_words(addr, 1)[0]
        if m != CTRL_MAGIC:
            sys.exit(f"dualobs: {nm} CTRL_OBS magic 0x{m:08x} != 0x{CTRL_MAGIC:08x}: wrong ELF")
        conns[nm] = o
    print(f"CTRL_OBS @0x{addr:08x}, magic OK on both. {a.hz} Hz. Ctrl-C to stop.")

    wr = None
    if a.csv:
        fh = open(a.csv, "w", newline="")
        wr = csv.writer(fh)
        wr.writerow(["t", "b"] + [f"w{i}" for i in range(35)])

    hdr = (f"{'t':>7} b {'mode':>4} {'sub':>3} {'moe':>3} {'torq':>6} {'d0':>6} {'spd':>6} "
           f"{'dAge':>5} {'cAge':>5} {'faul':>12} {'dwl':>4} {'flags':>28} {'lvls':>22} "
           f"{'cnts':>24} {'shut':>4} {'init':>4} {'peak':>5} {'ch':>3} {'tr':>3} "
           f"{'blap':>5} {'blerr':>5} {'llerr':>5} {'llap':>5} {'batt':>5}")
    print(hdr)
    prev = {}
    t0 = time.time()
    period = 1.0 / a.hz
    try:
        while True:
            start = time.time()
            for nm, o in conns.items():
                try:
                    w = o.read_words(addr, 35)
                except Exception as e:  # keep the other board sampling
                    print(f"{time.time()-t0:7.2f} {nm} READ-FAIL {e}")
                    continue
                d = decode(w)
                if wr:
                    wr.writerow([f"{time.time()-t0:.3f}", nm] + list(w))
                key = (d["mode"], d["sub"], d["moe"], d["faultbits"], d["flags"], d["levels"],
                       tuple(d["counts"]), d["shutdowns"], d["inits"], d["trips"],
                       d["ble_lap"], d["ble_lerr"], d["link_lerr"], d["link_lap"])
                changed = prev.get(nm) != key
                prev[nm] = key
                if a.quiet and not changed:
                    continue
                print(f"{time.time()-t0:7.2f} {nm} {d['mode']:>4} {d['sub']:>3} "
                      f"0x{d['moe']:02x} {d['torque']:>6} {d['d0']:>6} {d['speed_s']:>6} "
                      f"{d['drive_age']:>5} {d['cyclic_age']:>5} {names(d['faultbits'], FAULTS):>12} "
                      f"{d['dwell']:>4} {names(d['flags'], FLAGS):>28} {lvlnames(d['levels']):>22} "
                      f"{','.join(str(c) for c in d['counts']):>24} "
                      f"{d['shutdowns']:>4} {d['inits']:>4} {d['peak']:>5} {d['chopped']:>3} "
                      f"{d['trips']:>3} {d['ble_lap']:>5} {d['ble_lerr']:>5} {d['link_lerr']:>5} "
                      f"{d['link_lap']:>5} {d['battery']:>5}{'  *' if changed else ''}")
            if a.secs and time.time() - t0 > a.secs:
                break
            dt = period - (time.time() - start)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        if wr:
            fh.close()
        for o in conns.values():
            o.close()


if __name__ == "__main__":
    main()
