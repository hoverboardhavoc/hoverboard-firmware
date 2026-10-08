#!/usr/bin/env python3
"""Calibrate the VBATT sense chain on a master board against a bench supply.

WHY A MULTI-POINT FIT, AND WHY IT CALIBRATES THE CHAIN RATHER THAN THE DIVIDER
-----------------------------------------------------------------------------
A single reading at one rail voltage gives one number, volts-per-count, and cannot tell a gain
error from an offset: an ADC offset, divider leakage or an amplifier bias all hide inside it. Two
or more points give a SLOPE and an INTERCEPT, so the offset becomes visible instead of being
absorbed into the gain, and the residuals say whether the chain is linear at all.

It deliberately does NOT try to derive "the divider ratio". That number only exists if you assume
VDDA is exactly 3.300 V, which nothing here measures (and the VREFINT cross-check that would bound
it is unavailable, see the ADC0 warning below). What the firmware actually needs is counts ->
centivolts, and a fit of counts against a METERED rail gives exactly that with the divider, VDDA
and the ADC's own gain folded into one honest number. The resistor ratio is a curiosity; the chain
calibration is the deliverable.

For the same reason this asks for your METER reading, not the supply's dial. A PSU display is not a
reference, and the single-point measurement of 2026-10-14 recorded "no meter in the loop" as its
main weakness. If you have no meter, type the dial value and say so in the record, but know that
the fit is then only as good as the dial.

SCOPE
-----
Master boards only. VBATT is on PA4 and only masters sense it; a slave reads the pin stuck, so a
fit from a slave is meaningless.

Reads through ADC1, which the firmware does not use. DO NOT "improve" this by using ADC0: the
firmware owns ADC0 for the motor's injected phase-current group, and its 16 kHz triggers reset any
regular conversion and set EOC themselves, so an ADC0 regular read returns noise. That was
measured (VREFINT read 3050 with an sd of 269) and it is why no reference cross-check is attempted.

The core is never halted and nothing the firmware owns is written. RCU_APB2EN is restored on exit.

DISARMED ONLY. This neither needs nor wants the bridge enabled: it is a quiet pin read.

USAGE
-----
    tools/vbatt-calibrate.py [--board master] [--points 25,20,16,12] [-n 64]

It takes the bench lock, starts OpenOCD on the Pi for that board, and releases both on exit, the
same way tools/motor-trace.py does. Use --attach HOST:PORT to drive an OpenOCD you started
yourself, in which case it takes no lock and leaves it running.

The low end of the sweep is bounded by the board, not by this script: below some rail the regulator
drops out and the board stops running. There is no established figure for that, so the script
checks that SWD still answers and that the readings are stable at every point, and tells you to
raise the voltage rather than recording nonsense. Start at the TOP of your sweep and work down, so
a brown-out ends the run instead of corrupting the middle of it.
"""
import argparse
import atexit
import os
import socket
import statistics
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PI = os.environ.get("PI_HOST", "pi@192.168.0.248")

# NOTE: this probe table is a THIRD copy of a fact that already lives in tools/flash.sh and
# tools/motor-trace.py (whose own comment says it mirrors flash.sh). Three copies of the bench
# wiring is debt, not a pattern; it wants one owner. Kept per-tool here only to match the existing
# convention rather than refactor two working tools from inside a new one.
BOARDS = {
    "master": (
        "-f interface/stlink.cfg -c 'transport select dapdirect_swd' "
        "-c 'adapter usb location 1-1.2.4'"
    ),
    "slave": (
        "-c 'adapter driver cmsis-dap' -c 'cmsis-dap backend usb_bulk' "
        "-c 'cmsis-dap vid_pid 0x1209 0xda42' -c 'adapter speed 1000'"
    ),
}


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True)


def start_remote_ocd(board):
    """Start OpenOCD on the Pi for one bench board and return its host. Mirrors motor-trace.py."""
    cfg = BOARDS[board]
    sh(f"ssh {PI} 'sudo pkill -x openocd' >/dev/null 2>&1")
    time.sleep(1)
    launch = (
        f"nohup sudo openocd {cfg} -c 'set CPUTAPID 0' -f target/stm32f1x.cfg "
        f"-c 'bindto 0.0.0.0' -c init > /tmp/ocd-vbatt.log 2>&1 &"
    )
    sh(f"ssh {PI} \"{launch}\"")
    for _ in range(20):
        time.sleep(0.5)
        r = sh(f"ssh {PI} \"grep -c 'Listening on port 6666' /tmp/ocd-vbatt.log\"")
        if r.stdout.strip() == "1":
            return PI.split("@")[-1]
    sys.exit("vbatt-calibrate: OpenOCD did not come up on the Pi (see /tmp/ocd-vbatt.log there)")

TERM = b"\x1a"

RCU_APB2EN = 0x40021018
RCU_CFG0 = 0x40021004
GPIOA_CTL0 = 0x40010800
ADC0 = 0x40012400
ADC1 = 0x40012800
STAT, CTL0, CTL1, SAMPT1, RSQ0, RSQ1, RSQ2, RDATA = (
    0x00, 0x04, 0x08, 0x10, 0x2C, 0x30, 0x34, 0x4C)
CTL1_ADCON = 1 << 0
CTL1_CLB = 1 << 2
CTL1_RSTCLB = 1 << 3
CTL1_ETSRC_SW = 0x7 << 17
CTL1_ETERC = 1 << 20
CTL1_SWRCST = 1 << 22
STAT_EOC = 1 << 1
ADC1_CLK_BIT = 1 << 10

# A point is rejected rather than recorded if the samples scatter more than this. The 2026-10-14
# single-point run saw sd 2.2 counts at 25 V, so anything past ~15 is a different phenomenon
# (brown-out, a floating pin, switching noise) and not the measurement this tool is making.
SD_REJECT = 15.0


class Ocd:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)

    def cmd(self, text):
        self.sock.sendall(text.encode() + TERM)
        buf = b""
        while not buf.endswith(TERM):
            chunk = self.sock.recv(8192)
            if not chunk:
                raise RuntimeError("openocd closed the connection")
            buf += chunk
        return buf[:-1].decode(errors="replace").strip()

    def rd(self, addr):
        return int(self.cmd(f"read_memory 0x{addr:08x} 32 1").split()[0], 0)

    def wr(self, addr, val):
        out = self.cmd(f"write_memory 0x{addr:08x} 32 {{{val}}}")
        if out:
            raise RuntimeError(f"write_memory: {out}")


def adc1_bring_up(o, saved_ctl0):
    """Clock ADC1, put PA4 in analog mode, enable and calibrate. Verified on silicon 2026-10-14."""
    apb2 = o.rd(RCU_APB2EN)
    if not apb2 & ADC1_CLK_BIT:
        o.wr(RCU_APB2EN, apb2 | ADC1_CLK_BIT)
    o.wr(GPIOA_CTL0, saved_ctl0 & ~(0xF << 16))  # PA4 [19:16] = 0 -> analog input
    o.wr(ADC1 + CTL0, 0)
    o.wr(ADC1 + RSQ0, 0)
    o.wr(ADC1 + RSQ1, 0)
    o.wr(ADC1 + RSQ2, 4)                          # regular sequence length 1, channel 4
    o.wr(ADC1 + SAMPT1, 0x7 << (3 * 4))           # channel 4, 239.5 cycles
    base = CTL1_ETSRC_SW | CTL1_ETERC | CTL1_ADCON
    o.wr(ADC1 + CTL1, base)
    time.sleep(0.01)
    for bit in (CTL1_RSTCLB, CTL1_CLB):
        o.wr(ADC1 + CTL1, base | bit)
        for _ in range(100):
            if not o.rd(ADC1 + CTL1) & bit:
                break
        else:
            raise RuntimeError(f"ADC1 calibration bit 0x{bit:x} never cleared")
    return base, apb2


def sample(o, base, n):
    counts = []
    for _ in range(n):
        o.wr(ADC1 + STAT, 0)
        o.wr(ADC1 + CTL1, base | CTL1_SWRCST)
        for _ in range(50):
            if o.rd(ADC1 + STAT) & STAT_EOC:
                break
        else:
            raise RuntimeError("EOC never set: is the board still powered?")
        counts.append(o.rd(ADC1 + RDATA) & 0xFFF)
    return counts


def fit(points):
    """Least squares of volts against counts. Returns slope V/count, intercept V, r2, residuals."""
    n = len(points)
    xs = [c for c, _ in points]
    ys = [v for _, v in points]
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        raise RuntimeError("every point read the same count; nothing to fit")
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    intercept = my - slope * mx
    resid = [y - (slope * x + intercept) for x, y in zip(xs, ys)]
    sst = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - sum(r * r for r in resid) / sst if sst > 0 else float("nan")
    return slope, intercept, r2, resid


def main():
    ap = argparse.ArgumentParser(description="Calibrate the VBATT sense chain against a bench supply.")
    ap.add_argument("--board", choices=sorted(BOARDS), default="master",
                    help="which bench board (default master; a slave reads PA4 stuck)")
    ap.add_argument("--attach", metavar="HOST:PORT",
                    help="use an already-running OpenOCD instead of starting one (no lock taken)")
    ap.add_argument("--points", default="25,20,16,12",
                    help="nominal rail voltages to ask for, high to low (default 25,20,16,12)")
    ap.add_argument("-n", type=int, default=64, help="samples per point (default 64)")
    args = ap.parse_args()

    targets = [float(x) for x in args.points.split(",")]

    if args.attach:
        host, port = args.attach.split(":")
        port = int(port)
    else:
        r = sh(f"{REPO}/tools/bench-lock.sh acquire claude-vbatt 'vbatt calibration'")
        if "ACQUIRED" not in r.stdout and "ALREADY-OURS" not in r.stdout:
            sys.exit(f"vbatt-calibrate: bench busy\n{r.stdout}{r.stderr}")
        atexit.register(lambda: sh(f"{REPO}/tools/bench-lock.sh release claude-vbatt"))
        host, port = start_remote_ocd(args.board), 6666
        atexit.register(lambda: sh(f"ssh {PI} 'sudo pkill -x openocd'"))
        if args.board != "master":
            print(f"WARNING: --board {args.board} is not a master; PA4 reads stuck on a slave.\n")

    o = Ocd(host, port)
    saved_ctl0 = o.rd(GPIOA_CTL0)
    cfg0 = o.rd(RCU_CFG0)
    print(f"GPIOA_CTL0=0x{saved_ctl0:08x} (PA4 field 0x{(saved_ctl0 >> 16) & 0xF:x})  "
          f"ADC prescaler code {(cfg0 >> 14) & 3}")
    print(f"ADC0 CTL1=0x{o.rd(ADC0 + CTL1):08x}  (the motor's, read only, never written)")
    print()
    print("Work DOWNWARD through the voltages so a brown-out ends the run rather than spoiling")
    print("the middle of it. Type the voltage your METER reads, not the supply's dial.")
    print()

    base, apb2 = adc1_bring_up(o, saved_ctl0)
    points = []
    try:
        for want in targets:
            while True:
                ans = input(f"  Set the supply to about {want:g} V, then type the metered volts "
                            f"(s=skip, q=finish): ").strip().lower()
                if ans in ("q", "quit"):
                    raise KeyboardInterrupt
                if ans in ("s", "skip", ""):
                    break
                try:
                    volts = float(ans)
                except ValueError:
                    print("    not a number")
                    continue
                try:
                    counts = sample(o, base, args.n)
                except RuntimeError as e:
                    print(f"    READ FAILED: {e}")
                    print("    raise the voltage; the board is probably below its dropout")
                    continue
                mean = statistics.mean(counts)
                sd = statistics.pstdev(counts)
                print(f"    {volts:.3f} V -> mean {mean:.1f} counts  sd {sd:.2f}  "
                      f"min {min(counts)}  max {max(counts)}")
                if sd > SD_REJECT:
                    print(f"    REJECTED: sd {sd:.1f} over the {SD_REJECT:g} limit. That is not "
                          f"measurement noise.")
                    print("    Check the rail is steady and the board is not browning out, then retry.")
                    continue
                if mean < 10:
                    print("    REJECTED: near zero counts. Is this actually a master board?")
                    continue
                points.append((mean, volts))
                break
    except KeyboardInterrupt:
        print("\n  stopped")
    finally:
        o.wr(ADC1 + CTL1, 0)
        o.wr(RCU_APB2EN, apb2)
        print(f"\nrestored: RCU_APB2EN=0x{o.rd(RCU_APB2EN):08x} ADC1 CTL1=0x{o.rd(ADC1 + CTL1):08x}")
        print("PA4 left in analog mode (its reset state is floating input; analog is quieter "
              "for a sense pin).")

    if len(points) < 2:
        print(f"\n{len(points)} usable point(s): not enough to fit. Two or more, and spread as wide "
              f"as the board tolerates, is the whole point of this tool.")
        return 1

    slope, intercept, r2, resid = fit(points)
    print(f"\n{len(points)} points, fit of volts against counts:")
    for (c, v), r in zip(points, resid):
        print(f"  {v:7.3f} V  {c:8.1f} counts   residual {r*1000:+7.1f} mV")
    print(f"\n  slope      {slope*1000:.6f} mV per count")
    print(f"  intercept  {intercept*1000:+.1f} mV   <- a real offset in the chain, not gain")
    print(f"  r2         {r2:.6f}")
    print(f"  worst residual {max(abs(r) for r in resid)*1000:.1f} mV")
    print()
    print("  For the store field, which wants counts -> CENTIVOLTS:")
    print(f"    centivolts = counts * {slope*100:.6f} + ({intercept*100:+.2f})")
    num = round(slope * 100 * 65536)
    print(f"    integer form: centivolts = (counts * {num} >> 16) + ({round(intercept*100)})")
    print()
    if abs(intercept) > 0.25:
        print(f"  NOTE: a {intercept*1000:+.0f} mV intercept is large enough that a single-point")
        print("  calibration would have folded it into the gain and been wrong at every other")
        print("  voltage. Keep the offset in the field, do not drop it.")
    else:
        print("  The intercept is small, so a gain-only field would be defensible. Record the")
        print("  figure anyway, since that is the evidence it is small.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
