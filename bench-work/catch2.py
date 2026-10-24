#!/usr/bin/env python3
"""Separate the two faults on the master, using LIVE statics rather than published copies.

  DISPATCH (live)  stalls -> the MAIN THREAD is wedged   -> the IWDG path
  OBS publish      stalls -> the CONTROL TASK is absent  -> the starvation path
  TICK / PERIODS           -> whether the ISRs are still being served

On a main-thread wedge it also samples both USARTs' STAT, because the only unbounded waits in the
loop are `PolledSerial::write` (TBE) and `flush` (TC) in runtime_hal::serial.
"""
import subprocess, sys, time
sys.path.insert(0, "tools")
from swdobs import Ocd

DISPATCH, TICK, PERIODS = 0x200008EC, 0x200008E0, 0x200008D0
OBS_DISP = 0x20000A88 + 12
USART0, USART1 = 0x40013800, 0x40004400   # GD32F1x0: USART_STAT at +0x00
FLAGS = [(7,"TBE"),(6,"TC"),(5,"RBNE"),(4,"IDLE"),(3,"ORERR"),(2,"NERR"),(1,"FERR"),(0,"PERR")]
ELF = "target/thumbv7m-none-eabi/release/firmware"
syms = []
for line in subprocess.check_output(["arm-none-eabi-nm","-S","-n",ELF],text=True).splitlines():
    p = line.split()
    if len(p) == 4: syms.append((int(p[0],16), int(p[1],16), p[3]))
def res(a):
    for b,s,n in syms:
        if b <= a < b+s: return f"{n}+0x{a-b:x}"
    return None
def stat(o, base):
    v = o.read_words(base, 1)[0]
    return f"0x{v:04x} [" + " ".join(n for b,n in FLAGS if v>>b & 1) + "]"

o = Ocd("127.0.0.1", 6668)
pd = o.read_words(DISPATCH,1)[0]; po = o.read_words(OBS_DISP,1)[0]
td = to = time.time(); t0 = time.time()
rep_thread = rep_task = 0
print("watching: live DISPATCH (main thread) and the OBS publish (control task)", flush=True)
while time.time() - t0 < float(sys.argv[1] if len(sys.argv)>1 else 1500):
    d = o.read_words(DISPATCH,1)[0]; ob = o.read_words(OBS_DISP,1)[0]
    now = time.time()
    if d != pd: pd, td = d, now
    if ob != po: po, to = ob, now
    if now - td > 0.12:                      # MAIN THREAD WEDGED
        tk1, pr1 = o.read_words(TICK,1)[0], o.read_words(PERIODS,1)[0]
        s0, s1 = stat(o,USART0), stat(o,USART1)
        time.sleep(0.08)
        tk2, pr2 = o.read_words(TICK,1)[0], o.read_words(PERIODS,1)[0]
        print(f"\nt={now-t0:7.1f} MAIN THREAD WEDGED {(now-td)*1000:.0f} ms", flush=True)
        print(f"   SysTick still running: {tk2-tk1} ticks in 80 ms   period ISR: {pr2-pr1} periods")
        print(f"   USART0 STAT {s0}\n   USART1 STAT {s1}")
        o.cmd("halt")
        pc = int(o.cmd("reg pc").split()[-1],0); lr = int(o.cmd("reg lr").split()[-1],0)
        sp = int(o.cmd("reg sp").split()[-1],0); xp = int(o.cmd("reg xpsr").split()[-1],0)
        print(f"   PC 0x{pc:08x} {res(pc)}\n   LR 0x{lr:08x}\n   xPSR 0x{xp:08x} IPSR={xp & 0x1ff}")
        print("   stack:")
        for i,w in enumerate(o.read_words(sp,48)):
            r = res(w) or res(w & ~1)
            if r: print(f"     sp+0x{i*4:03x} 0x{w:08x}  {r}")
        o.cmd("resume")
        rep_thread += 1; td = time.time()
        if rep_thread >= 2: break
    elif now - to > 0.30 and rep_task < 6:   # CONTROL TASK ABSENT, loop alive
        tk1 = o.read_words(TICK,1)[0]; time.sleep(0.1); tk2 = o.read_words(TICK,1)[0]
        print(f"t={now-t0:7.1f} control task absent {(now-to)*1000:.0f} ms "
              f"(loop alive: dispatch moving; SysTick {tk2-tk1}/80ms)", flush=True)
        rep_task += 1; to = time.time()
o.close()
print("done", flush=True)
