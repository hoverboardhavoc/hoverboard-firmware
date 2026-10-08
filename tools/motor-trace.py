#!/usr/bin/env python3
"""Sample the motor and control observables over SWD while someone drives the board.

The core is NEVER halted: this reads RAM through the MEM-AP the same way the SWD mailbox does,
so the 16 kHz period ISR and the 250 Hz control task keep running underneath the sampling.

Usage:
    tools/motor-trace.py [--board master|slave] [--hz 10] [--csv FILE] [--elf PATH]
    tools/motor-trace.py --attach HOST:PORT     # use an OpenOCD that is already running

Ctrl-C to stop. It stops the OpenOCD it started and releases the bench lock it took.

Every address is resolved from the ELF's symbol table at startup, never hardcoded: these are
Rust-mangled statics whose addresses move with any build, and a stale address reads plausible
rubbish rather than failing. If the flashed image is not the one at --elf, the numbers are lies.
CTRL_OBS carries a magic word ("CTRL") which is checked for exactly that reason.
"""

import argparse
import atexit
import csv
import os
import signal
import sys
import time

from swdobs import (
    BOARDS, COUNTS_PER_AMP, CTRL_MAGIC, DEFAULT_ELF, MOTOR_CURRENT_OFFSET, REPO, W_BLE_RX,
    W_TORQUE_MODE, Ocd, motor_block, resolve_symbols, s16, s32, sh, start_remote_ocd,
    stop_remote_ocd,
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", choices=sorted(BOARDS), default="master")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--attach", default=None, help="HOST:PORT of a running OpenOCD TCL port")
    args = ap.parse_args()

    addrs = resolve_symbols(args.elf)
    base, span, off = motor_block(addrs)

    if args.attach:
        host, port = args.attach.split(":")
        port = int(port)
        started = False
    else:
        r = sh(f"{REPO}/tools/bench-lock.sh acquire claude-trace 'motor-trace sampling'")
        if "ACQUIRED" not in r.stdout and "ALREADY-OURS" not in r.stdout:
            sys.exit(f"motor-trace: bench busy\n{r.stdout}{r.stderr}")
        atexit.register(lambda: sh(f"{REPO}/tools/bench-lock.sh release claude-trace"))
        host, port, started = start_remote_ocd(args.board), 6666, True
        atexit.register(stop_remote_ocd)

    ocd = Ocd(host, port)
    atexit.register(ocd.close)

    magic = ocd.read_words(addrs["CTRL_OBS"], 1)[0]
    if magic != CTRL_MAGIC:
        sys.exit(
            f"motor-trace: CTRL_OBS magic is 0x{magic:08x}, expected 0x{CTRL_MAGIC:08x}.\n"
            "  The flashed image is not the one at --elf, so every address here is wrong."
        )

    writer = None
    if args.csv:
        fh = open(args.csv, "w", newline="")
        atexit.register(fh.close)
        writer = csv.writer(fh)
        writer.writerow(
            ("t demand torque speed d0 d1 d2 angle periods dperiods dwell fault "
             "state mode moe rx_ovr rx_lerr motor_current peak_a chopped trips").split()
        )

    hdr = (
        f"{'t':>6} {'demand':>7} {'torque':>7} {'speed':>7} "
        f"{'d0':>5} {'d1':>5} {'d2':>5} {'ang':>5} {'dper':>6} "
        f"{'dwell':>5} {'fault':>5} {'mode':>4} {'moe':>3} {'rxovr':>6} {'rxerr':>6} "
        f"{'peakA':>6} {'chop':>4} {'trip':>4}"
    )
    print(f"board={args.board} elf={os.path.relpath(args.elf, REPO)} at {args.hz} Hz, Ctrl-C to stop")
    print("  dper = period-ISR ticks since the previous sample (16 kHz, so ~1600 at 10 Hz).")
    print("  A wheel drawing current with dwell climbing is commutation, not supply.\n")
    print(hdr)
    print("-" * len(hdr))

    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    t0 = time.time()
    period = 1.0 / args.hz
    prev_periods = None
    row_n = 0

    while True:
        loop_start = time.time()
        m = ocd.read_words(base, span)
        c = ocd.read_words(addrs["CTRL_OBS"] + 4 * W_TORQUE_MODE, 2)  # torque|mode|moe, then substate|cmode|flags|levels
        # Word 29: the BLE port's RX losses, two saturating u16 halves (overruns | line errors).
        # This is the counter that tells a drive-demand sag caused by the BOARD dropping inbound
        # bytes apart from one caused by the phone not delivering them: overruns stepping in time
        # with a sag is the first, a flat counter during a sag is the second.
        rx = ocd.read_words(addrs["CTRL_OBS"] + 4 * W_BLE_RX, 1)[0]
        rx_ovr, rx_lerr = rx & 0xFFFF, (rx >> 16) & 0xFFFF
        # Word 31: the phase-current observation (the current-limit slice).
        cur = ocd.read_words(addrs["CTRL_OBS"] + MOTOR_CURRENT_OFFSET, 1)[0]
        peak, chopped, trips = s16(cur & 0xFFFF), (cur >> 16) & 0xFF, (cur >> 24) & 0xFF
        peak_a = peak / COUNTS_PER_AMP

        demand = s32(m[off["DEMAND"]])
        speed = s32(m[off["SPEED"]])
        periods = m[off["PERIODS"]]
        duty01 = m[off["OBS_DUTY01"]]
        duty2ang = m[off["OBS_DUTY2_ANGLE"]]
        dwell = m[off["INVALID_DWELL"]]
        fault = m[off["FAULT"]]
        state = m[off["OBS_STATE"]]
        d0, d1 = duty01 & 0xFFFF, (duty01 >> 16) & 0xFFFF
        d2, ang = duty2ang & 0xFFFF, (duty2ang >> 16) & 0xFFFF
        torque = s16(c[0] & 0xFFFF)
        mode = (c[0] >> 16) & 0xFF
        moe = (c[0] >> 24) & 0xFF

        dper = 0 if prev_periods is None else periods - prev_periods
        prev_periods = periods
        t = time.time() - t0

        if row_n and row_n % 25 == 0:
            print(hdr)
        print(
            f"{t:6.1f} {demand:7d} {torque:7d} {speed:7d} "
            f"{d0:5d} {d1:5d} {d2:5d} {ang:5d} {dper:6d} "
            f"{dwell:5d} {fault:5d} {mode:4d} 0x{moe:02x} {rx_ovr:6d} {rx_lerr:6d} "
            f"{peak_a:6.2f} {chopped:4d} {trips:4d}",
            flush=True,
        )
        if writer:
            writer.writerow([f"{t:.3f}", demand, torque, speed, d0, d1, d2, ang,
                             periods, dper, dwell, fault, state, mode, moe,
                             rx_ovr, rx_lerr, f"0x{cur:08x}", f"{peak_a:.3f}",
                             chopped, trips])
        row_n += 1

        slack = period - (time.time() - loop_start)
        if slack > 0:
            time.sleep(slack)


if __name__ == "__main__":
    main()
