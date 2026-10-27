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
    BOARDS, CTRL_MAGIC, DEFAULT_ELF, LOCAL_BOARDS, MOTOR_CURRENT_OFFSET, REPO,
    W_FOC_DQ_MEAN, W_FOC_PHASE_MEAN, W_FOC_REF_VQ, W_GATING_PRE_ENV, W_MOTOR_RATE_Q,
    W_PITCH_MILLI, W_ROLL_MILLI, W_SF_REF, W_SF_TERMS_PR, W_TORQUE_MODE, W_WHEEL_POS_OWN,
    W_WHEEL_POS_PEER, Ocd, i16_pair, motor_block, read_current_cal, resolve_symbols, s16, s32,
    sh, start_local_ocd, start_remote_ocd, stop_local_ocd, stop_remote_ocd,
)


def sample(ocd, addrs, base, span, off, cpa):
    """One read of every observable, as the row the trace prints; raises RuntimeError on a
    failed read."""
    m = ocd.read_words(base, span)
    c = ocd.read_words(addrs["CTRL_OBS"] + 4 * W_TORQUE_MODE, 2)  # torque|mode|moe, then substate|cmode|flags|levels
    # Word 26, high half: the pre-envelope torque, what the controller would command this
    # tick regardless of sub-state.
    preenv = s16((ocd.read_words(addrs["CTRL_OBS"] + 4 * W_GATING_PRE_ENV, 1)[0] >> 16) & 0xFFFF)
    # Word 6: the fused pitch in millidegrees, which is what the tilt cutout judges.
    pitch = s32(ocd.read_words(addrs["CTRL_OBS"] + 4 * W_PITCH_MILLI, 1)[0])
    roll = s32(ocd.read_words(addrs["CTRL_OBS"] + 4 * W_ROLL_MILLI, 1)[0])
    # Word 39: the period-based wheel rate in 1/4096 of a count (specs/control.md (o)), the
    # state-feedback law's speed input. Printed in counts to two decimals beside the edge count.
    rate = s32(ocd.read_words(addrs["CTRL_OBS"] + 4 * W_MOTOR_RATE_Q, 1)[0]) / 4096.0
    # Words 42-43: the two wheel positions in hall edges, forward positive, own then peer
    # (specs/state-feedback.md section 8). One read, so the pair is from one pass.
    pos = ocd.read_words(addrs["CTRL_OBS"] + 4 * W_WHEEL_POS_OWN, 2)
    xown, xpeer = s32(pos[0]), s32(pos[W_WHEEL_POS_PEER - W_WHEEL_POS_OWN])
    # Words 44-46: the state-feedback law's four terms as torque words, then its speed reference
    # (1/4096 count, printed in counts) and position error in edges (specs/state-feedback.md
    # section 8). One read, so the five are from one pass.
    sf = ocd.read_words(addrs["CTRL_OBS"] + 4 * W_SF_TERMS_PR, W_SF_REF - W_SF_TERMS_PR + 1)
    tp, tr = s16(sf[0] & 0xFFFF), s16(sf[0] >> 16)
    tx, tv = s16(sf[1] & 0xFFFF), s16(sf[1] >> 16)
    vref, ex = s16(sf[2] & 0xFFFF) / 4096.0, s16(sf[2] >> 16)
    # Word 31: the phase-current observation (the current-limit slice).
    cur = ocd.read_words(addrs["CTRL_OBS"] + MOTOR_CURRENT_OFFSET, 1)[0]
    peak, chopped, trips = s16(cur & 0xFFFF), (cur >> 16) & 0xFF, (cur >> 24) & 0xFF
    peak_a = peak / cpa
    # Words 47-49: the FOC observation, 64-period window means in current counts
    # (specs/foc-torque.md section 7): the phase pair in every method, then id / iq and iq_ref /
    # vq, which read 0 unless the method is FOC. One read, so the three are from one publish.
    foc = ocd.read_words(addrs["CTRL_OBS"] + 4 * W_FOC_PHASE_MEAN,
                         W_FOC_REF_VQ - W_FOC_PHASE_MEAN + 1)
    ia, ib = i16_pair(foc[0])
    fid, fiq = i16_pair(foc[W_FOC_DQ_MEAN - W_FOC_PHASE_MEAN])
    iqref, vq = i16_pair(foc[W_FOC_REF_VQ - W_FOC_PHASE_MEAN])

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

    return (demand, torque, preenv, pitch, roll, speed, rate, xown, xpeer, tp, tr, tx, tv, vref, ex,
            d0, d1, d2, ang, periods, dwell, fault, state, mode, moe, cur, peak_a,
            chopped, trips, ia, ib, fid, fiq, iqref, vq)


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
        if args.board in LOCAL_BOARDS:
            proc = start_local_ocd(args.board)
            atexit.register(stop_local_ocd, proc)
            host, port, started = "localhost", 6666, True
        else:
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

    # The amps column's scale is the BOARD's, read off it here rather than compiled in: 0x67,
    # `motor.current_cal` (specs/motor-integration.md, "The current-sense calibration"). Motor 0,
    # which is the motor this block observes.
    try:
        cal_raw, cpa = read_current_cal(sh, f"{host}:{port}")
    except (RuntimeError, OSError) as e:
        sys.exit(f"motor-trace: {e}\n  The peak column is counts; amps need the board's 0x67.")
    cal_note = f"motor.current_cal (0x67) = {cal_raw} counts per amp"
    if cpa != cal_raw:
        cal_note += f", which the firmware's boot seam clamps to {cpa}"

    writer = None
    if args.csv:
        fh = open(args.csv, "w", newline="")
        atexit.register(fh.close)
        writer = csv.writer(fh)
        writer.writerow(
            ("t demand torque preenv pitch roll speed rate xown xpeer tp tr tx tv vref ex d0 d1 d2 "
             "angle periods dperiods "
             "dwell fault state mode moe motor_current peak_a chopped trips "
             "ia ib id iq iq_ref vq").split()
        )

    hdr = (
        f"{'t':>6} {'demand':>7} {'torque':>7} {'preenv':>7} {'pitch':>7} {'roll':>7} {'speed':>7} "
        f"{'rate':>8} {'xown':>7} {'xpeer':>7} {'tp':>6} {'tr':>6} {'tx':>6} {'tv':>6} "
        f"{'vref':>6} {'ex':>5} {'d0':>5} {'d1':>5} {'d2':>5} {'ang':>5} {'dper':>6} "
        f"{'dwell':>5} {'fault':>5} {'mode':>4} {'moe':>3} "
        f"{'peakA':>6} {'chop':>4} {'trip':>4} {'ia':>6} {'ib':>6} {'id':>6} {'iq':>6} "
        f"{'iqref':>6} {'vq':>6}"
    )
    print(f"board={args.board} elf={os.path.relpath(args.elf, REPO)} at {args.hz} Hz, Ctrl-C to stop")
    print("  dper = period-ISR ticks since the previous sample (16 kHz, so ~1600 at 10 Hz).")
    print(f"  {cal_note}: peakA is peak / {cpa}.")
    print("  A wheel drawing current with dwell climbing is commutation, not supply.")
    print("  preenv = the pre-envelope torque (CTRL_OBS word 26), live even disarmed; torque is")
    print("  the engagement machine's output and reads 0 whenever it is disengaged.")
    print("  speed = hall edges per 20 ms window (the recovered word); rate = the period-based")
    print("  wheel rate in the same counts (CTRL_OBS word 39), the balance law's speed input.")
    print("  xown / xpeer = this wheel's and the peer's position in hall edges, forward positive")
    print("  (CTRL_OBS words 42 and 43); a straight push moves both the same way.")
    print("  tp tr tx tv = the balance law's pitch, rate, position and speed terms as torque words")
    print("  (words 44, 45), live disarmed; vref = its speed reference in counts and ex its")
    print("  position error in edges (word 46), both 0 outside RUN.")
    print("  ia ib = the mean phase currents in counts (word 47, every method); id iq iqref vq =")
    print("  the FOC loops' means and reference (words 48, 49), 0 unless the method is FOC.\n")
    print(hdr)
    print("-" * len(hdr))

    # Both signals exit through atexit so a locally started OpenOCD is not left attached to
    # the probe: a hung orphan holds the probe's one TCP link and the next tool cannot attach.
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    t0 = time.time()
    period = 1.0 / args.hz
    prev_periods = None
    row_n = 0

    failed = 0
    while True:
        loop_start = time.time()
        try:
            row = sample(ocd, addrs, base, span, off, cpa)
        except RuntimeError as e:
            # One garbled read on the wireless link is a dropped sample, not the end of the
            # trace; a run of them is the link gone.
            failed += 1
            print(f"  -- sample dropped ({e})", flush=True)
            if failed >= 5:
                sys.exit("motor-trace: five consecutive reads failed, the link is gone")
            continue
        failed = 0
        (demand, torque, preenv, pitch, roll, speed, rate, xown, xpeer, tp, tr, tx, tv, vref, ex,
         d0, d1, d2, ang, periods, dwell, fault, state, mode, moe, cur, peak_a,
         chopped, trips, ia, ib, fid, fiq, iqref, vq) = row
        dper = 0 if prev_periods is None else periods - prev_periods
        prev_periods = periods
        t = time.time() - t0

        if row_n and row_n % 25 == 0:
            print(hdr)
        print(
            f"{t:6.1f} {demand:7d} {torque:7d} {preenv:7d} {pitch:7d} {roll:7d} {speed:7d} "
            f"{rate:8.2f} {xown:7d} {xpeer:7d} {tp:6d} {tr:6d} {tx:6d} {tv:6d} {vref:6.2f} {ex:5d} "
            f"{d0:5d} {d1:5d} {d2:5d} {ang:5d} {dper:6d} "
            f"{dwell:5d} {fault:5d} {mode:4d} 0x{moe:02x} "
            f"{peak_a:6.2f} {chopped:4d} {trips:4d} {ia:6d} {ib:6d} {fid:6d} {fiq:6d} "
            f"{iqref:6d} {vq:6d}",
            flush=True,
        )
        if writer:
            writer.writerow([f"{t:.3f}", demand, torque, preenv, pitch, roll, speed, f"{rate:.3f}",
                             xown, xpeer, tp, tr, tx, tv, f"{vref:.3f}", ex,
                             d0, d1, d2, ang, periods, dper, dwell, fault, state, mode, moe,
                             f"0x{cur:08x}", f"{peak_a:.3f}",
                             chopped, trips, ia, ib, fid, fiq, iqref, vq])
        row_n += 1

        slack = period - (time.time() - loop_start)
        if slack > 0:
            time.sleep(slack)


if __name__ == "__main__":
    main()
