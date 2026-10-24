#!/usr/bin/env python3
"""Watch a DISARMED board's main loop; the instant it stops, halt and dump why.

The firmware sets DBG_CTL0.FWDGT_HOLD at bring-up (runtime_hal::FreeWatchdog::freeze_on_debug_halt),
so halting freezes the 500 ms watchdog and the core stays put for as long as the dump needs.
NEVER run against an armed board.
"""
import subprocess, sys, time
sys.path.insert(0, "tools")
from swdobs import Ocd

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 6668
STALL_S = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
MAX_S = float(sys.argv[3]) if len(sys.argv) > 3 else 1500
OBS = 0x20000A88
ELF = "target/thumbv7m-none-eabi/release/firmware"

syms = []
for line in subprocess.check_output(["arm-none-eabi-nm", "-S", "-n", ELF], text=True).splitlines():
    p = line.split()
    if len(p) == 4:
        syms.append((int(p[0], 16), int(p[1], 16), p[3]))

def resolve(a):
    for base, size, nm in syms:
        if base <= a < base + size:
            return f"{nm}+0x{a-base:x}"
    return None

o = Ocd("127.0.0.1", PORT)
last = o.read_words(OBS + 12, 1)[0]
last_t = time.time()
t0 = time.time()
resets = 0
print(f"watching dispatch_count on port {PORT}, stall threshold {STALL_S*1000:.0f} ms", flush=True)
while time.time() - t0 < MAX_S:
    try:
        v = o.read_words(OBS + 12, 1)[0]
    except Exception as e:
        print(f"read fail {e}", flush=True)
        time.sleep(0.2)
        continue
    now = time.time()
    if v != last:
        if v < last:
            resets += 1
            print(f"t={now-t0:7.1f} RESET (dispatch {last} -> {v})", flush=True)
        last, last_t = v, now
        continue
    if now - last_t < STALL_S:
        continue
    # ---- caught it ----
    print(f"\nt={now-t0:7.1f} STALL: dispatch_count stuck at {v} for {(now-last_t)*1000:.0f} ms. Halting.", flush=True)
    o.cmd("halt")
    reg = lambda r: int(o.cmd(f"reg {r}").split()[-1], 0)
    pc, lr, sp, xpsr = reg("pc"), reg("lr"), reg("sp"), reg("xPSR")
    print(f"  PC   0x{pc:08x}  {resolve(pc)}")
    print(f"  LR   0x{lr:08x}  {resolve(lr)}")
    print(f"  SP   0x{sp:08x}   xPSR 0x{xpsr:08x}  (IPSR exception number {xpsr & 0x1ff})")
    cfsr, hfsr, mmar, bfar = o.read_words(0xE000ED28, 1)[0], o.read_words(0xE000ED2C, 1)[0], \
        o.read_words(0xE000ED34, 1)[0], o.read_words(0xE000ED38, 1)[0]
    print(f"  CFSR 0x{cfsr:08x}  HFSR 0x{hfsr:08x}  MMFAR 0x{mmar:08x}  BFAR 0x{bfar:08x}")
    icsr = o.read_words(0xE000ED04, 1)[0]
    print(f"  ICSR 0x{icsr:08x}  (VECTACTIVE {icsr & 0x1ff}, VECTPENDING {(icsr >> 12) & 0x1ff})")
    print(f"  NVIC ISPR 0x{o.read_words(0xE000E200,1)[0]:08x}  IABR 0x{o.read_words(0xE000E300,1)[0]:08x}")
    print("  stack (code-looking words resolved):")
    for i, w in enumerate(o.read_words(sp, 40)):
        r = resolve(w) or resolve(w & ~1)
        if r:
            print(f"    sp+0x{i*4:03x}  0x{w:08x}  {r}")
    # two more PC samples a moment apart, to see if it is spinning or wedged on one instruction
    for k in range(3):
        o.cmd("resume")
        time.sleep(0.02)
        o.cmd("halt")
        p2 = reg("pc")
        print(f"  after 20 ms of running: PC 0x{p2:08x}  {resolve(p2)}")
    print("  (left HALTED; watchdog is frozen by DBG_CTL0.FWDGT_HOLD)", flush=True)
    break
o.close()
