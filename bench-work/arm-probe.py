#!/usr/bin/env python3
"""The run that forks the remaining question, for an ARMED hold.

Samples the master's LIVE statics at ~100/s (the publish-gap instrument; `tick - ctrl` cannot see
an absence because the scheduler runs the backlog off) plus both boards' cyclic_age / arm counts /
ring laps, and writes a CSV for the per-arm-cycle correlation.

If the master's live DISPATCH_COUNT stops for `--stall` ms it HALTS the core and dumps PC, LR,
xPSR, the fault registers and the stack, then resumes. The firmware sets DBG_CTL0.FWDGT_HOLD at
bring-up, so a halt freezes the watchdog rather than taking a reset.

  no stall  -> the loop is fine and the frames are lost or never emitted for a logical reason
  a stall   -> it is an absence, and the dump says what it was in
"""
import argparse, csv, subprocess, sys, time
sys.path.insert(0, "tools")
from swdobs import Ocd

ELF = "target/thumbv7m-none-eabi/release/firmware"

# Addresses are RESOLVED FROM THE ELF, never typed: they move with any build that adds a static
# or a CTRL_OBS word, and a stale address reads plausible rubbish rather than failing.
def _resolve():
    import subprocess as _sp
    want = {"motor7PERIODS": None, "firmware10TICK_COUNT": None, "firmware13BLE_RX_LOSSES": None,
            "firmware14DISPATCH_COUNT": None, "firmware16LINK_LINE_ERRORS": None,
            "firmware17LINK_LAP_OVERRUNS": None, "CTRL_OBS": None}
    for ln in _sp.check_output(["arm-none-eabi-nm", ELF], text=True).splitlines():
        q = ln.split()
        if len(q) != 3:
            continue
        for k in want:
            nm3 = q[2]
            if nm3 == k or nm3.endswith(k) or nm3.endswith(k + ".0"):
                if want[k] is None:
                    want[k] = int(q[0], 16)
    missing = [k for k, v in want.items() if v is None]
    if missing:
        raise SystemExit(f"arm-probe: cannot resolve {missing} from {ELF}")
    base = want["motor7PERIODS"]
    idx = {k: (v - base) // 4 for k, v in want.items() if k != "CTRL_OBS"}
    return base, want["CTRL_OBS"], idx

LIVE, OBS, _IX = _resolve()
I_PERIODS = _IX["motor7PERIODS"]
I_TICK = _IX["firmware10TICK_COUNT"]
I_BLERX = _IX["firmware13BLE_RX_LOSSES"]
I_DISP = _IX["firmware14DISPATCH_COUNT"]
I_LERR = _IX["firmware16LINK_LINE_ERRORS"]
I_LAP = _IX["firmware17LINK_LAP_OVERRUNS"]
_NLIVE = max(I_PERIODS, I_TICK, I_BLERX, I_DISP, I_LERR, I_LAP) + 1
print(f"resolved: LIVE 0x{LIVE:08x} (+{_NLIVE} words), CTRL_OBS 0x{OBS:08x}, disp idx {I_DISP}")
syms = []
for line in subprocess.check_output(["arm-none-eabi-nm", "-S", "-n", ELF], text=True).splitlines():
    p = line.split()
    if len(p) == 4:
        syms.append((int(p[0], 16), int(p[1], 16), p[3]))
def res(x):
    for b, sz, nm in syms:
        if b <= x < b + sz:
            return f"{nm}+0x{x-b:x}"
    return None

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=200)
ap.add_argument("--stall", type=float, default=30, help="ms of no main-loop progress before halting")
ap.add_argument("--csv", default="bench-work/arm-probe.csv")
ap.add_argument("--halt-if-disarmed", action="store_true", dest="halt_safe",
                help="Halt on a stall ONLY when the last published MOE was 0. During a stall the\n"
                     "firmware is not publishing, so that value IS the bridge state at the instant\n"
                     "the main thread stopped, and a stopped main thread cannot re-arm. Safe to use\n"
                     "through an armed hold: it declines to halt the cycles that caught MOE set.")
ap.add_argument("--halt", action="store_true",
                help="DISARMED ONLY: halt and dump the PC on a stall. Halting an ARMED board freezes\n"
                     "the core with MOE set and (DBG_CTL0.TIMER0_HOLD) the PWM counter stopped, so a\n"
                     "phase can sit energised with the current limiter halted too. Never pass it armed.")
ap.add_argument("--dumps", type=int, default=3)
a = ap.parse_args()

m, s = Ocd("127.0.0.1", 6668), Ocd("127.0.0.1", 6667)
f = open(a.csv, "w", newline="")
w = csv.writer(f)
w.writerow(["wall", "m_periods", "m_tick", "m_blerx", "m_disp", "m_lerr", "m_lap_live",
            "m_cA", "m_init", "m_lap", "m_mode", "m_moe", "s_cA", "s_init", "s_lap"])
t0 = time.time(); n = 0; dumps = 0
full = [0] * 8
pd = m.read_words(LIVE, _NLIVE)[I_DISP]; td = time.time()
print(f"armed probe: stall threshold {a.stall} ms, {a.dumps} dumps max", flush=True)
try:
    while time.time() - t0 < a.secs and (not (a.halt or a.halt_safe) or dumps < a.dumps):
        try:
            lv = m.read_words(LIVE, _NLIVE)
        except Exception as e:   # a WiFi probe hiccup must not end the run
            print(f"read retry: {str(e)[:60]}", flush=True)
            time.sleep(0.05)
            continue
        now = time.time()
        if n % 8 == 0:
            try:
                mo = m.read_words(OBS + 4 * 7, 8); ml = m.read_words(OBS + 4 * 14, 1)[0]
                so = s.read_words(OBS + 4 * 7, 8); sl = s.read_words(OBS + 4 * 14, 1)[0]
                full = [mo[0], mo[2], ml, (mo[4] >> 16) & 0xFF, (mo[4] >> 24) & 0xFF, so[0], so[2], sl]
            except Exception:
                pass
        w.writerow([f"{now:.4f}", lv[I_PERIODS], lv[I_TICK], lv[I_BLERX] & 0xFFFF, lv[I_DISP],
                    lv[I_LERR], lv[I_LAP]] + full)
        n += 1
        if lv[I_DISP] != pd:
            pd, td = lv[I_DISP], now
            continue
        if (now - td) * 1000 < a.stall:
            continue
        print(f"\nt={now-t0:7.2f} MAIN THREAD AWAY {(now-td)*1000:.0f} ms "
              f"(cA m/s {full[0]}/{full[5]}, laps {full[2]}/{full[7]}, blerx {lv[I_BLERX] & 0xFFFF})",
              flush=True)
        moe = full[4]
        if not a.halt and not (a.halt_safe and moe == 0):
            if a.halt_safe:
                print(f"   (declined to halt: MOE was 0x{moe:02x} when the loop stopped)", flush=True)
            pd, td = lv[I_DISP], now
            continue
        m.cmd("halt")
        rg = lambda r: int(m.cmd(f"reg {r}").split()[-1], 0)
        pc, lr, sp, xp = rg("pc"), rg("lr"), rg("sp"), rg("xpsr")
        # PC HISTOGRAM INSIDE THIS ONE PAUSE. A single PC, and the code-looking words lying in the
        # stack below it, cannot tell a live call chain from the dead frames a recent call left
        # there. Resuming for a few hundred microseconds and re-halting, while the main thread is
        # still in the same pause, samples where it is ACTUALLY spinning. The loop stops the moment
        # the thread makes progress again, so it never runs past the pause it is measuring.
        hist, base_disp, steps = {}, m.read_words(LIVE, _NLIVE)[I_DISP], 0
        for _ in range(40):
            m.cmd("resume"); time.sleep(0.004); m.cmd("halt")
            cur = m.read_words(LIVE, _NLIVE)[I_DISP]
            p2 = rg("pc")
            nm2 = res(p2) or f"?0x{p2:08x}"
            hist[nm2] = hist.get(nm2, 0) + 1
            steps += 1
            if cur != base_disp:
                break
        print(f"   in-pause PC histogram ({steps} samples, "
              f"{'pause ended' if steps < 40 else 'still paused'}):")
        for k, v in sorted(hist.items(), key=lambda x: -x[1])[:8]:
            print(f"      {v:3d}  {k}")
        print(f"   PC 0x{pc:08x} {res(pc)}")
        print(f"   LR 0x{lr:08x}   xPSR 0x{xp:08x} IPSR={xp & 0x1ff}")
        print(f"   CFSR 0x{m.read_words(0xE000ED28,1)[0]:08x} HFSR 0x{m.read_words(0xE000ED2C,1)[0]:08x}"
              f" ICSR 0x{m.read_words(0xE000ED04,1)[0]:08x}")
        print(f"   NVIC ISPR 0x{m.read_words(0xE000E200,1)[0]:08x} IABR 0x{m.read_words(0xE000E300,1)[0]:08x}")
        for i, v in enumerate(m.read_words(sp, 48)):
            r = res(v) or res(v & ~1)
            if r:
                print(f"     sp+0x{i*4:03x} 0x{v:08x}  {r}")
        m.cmd("resume")
        dumps += 1; pd, td = m.read_words(LIVE, _NLIVE)[I_DISP], time.time()
except KeyboardInterrupt:
    pass
finally:
    f.close(); m.close(); s.close()
print(f"{n} rounds over {time.time()-t0:.1f}s, {dumps} dump(s)", flush=True)
