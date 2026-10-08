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
import re
import signal
import socket
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PI = os.environ.get("PI_HOST", "pi@192.168.0.248")
DEFAULT_ELF = os.path.join(REPO, "target/thumbv7m-none-eabi/release/firmware")
CTRL_MAGIC = 0x4C525443  # "CTRL", little-endian in memory

# The probe wiring per board, mirroring tools/flash.sh's table. Bench boards only: the offroad
# pair is reached over the network probes and wants the local patched OpenOCD instead.
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

# Motor statics, resolved by mangled-name suffix. They sit contiguously so one read covers them.
MOTOR_SYMS = [
    "DEMAND_SEQ", "OBS_DUTY01", "INVALID_DWELL", "COUNTER_RUNNING", "OBS_DUTY2_ANGLE",
    "FAULT", "SPEED", "DEMAND", "OBS_CAL", "PERIODS", "OBS_STATE",
]


def resolve_symbols(elf):
    """Address of every motor static plus CTRL_OBS, from the ELF's symbol table."""
    try:
        out = subprocess.check_output(
            ["arm-none-eabi-nm", elf], text=True, stderr=subprocess.DEVNULL
        )
    except (OSError, subprocess.CalledProcessError):
        sys.exit(f"motor-trace: cannot read symbols from {elf} (is arm-none-eabi-nm on PATH?)")

    addrs = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        addr, _kind, name = parts
        if name == "CTRL_OBS":
            addrs["CTRL_OBS"] = int(addr, 16)
            continue
        m = re.match(r"_RNvNtCs[A-Za-z0-9_]+_8firmware5motor\d+([A-Z_0-9]+)(\.\d+)?$", name)
        if m and m.group(1) in MOTOR_SYMS:
            addrs[m.group(1)] = int(addr, 16)

    missing = [s for s in MOTOR_SYMS + ["CTRL_OBS"] if s not in addrs]
    if missing:
        sys.exit(f"motor-trace: symbols absent from {elf}: {', '.join(missing)}")
    return addrs


class Ocd:
    """OpenOCD TCL RPC. read_memory returns a list, which beats parsing mdw's log output."""

    TERM = b"\x1a"

    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)

    def cmd(self, text):
        self.sock.sendall(text.encode() + self.TERM)
        buf = b""
        while not buf.endswith(self.TERM):
            chunk = self.sock.recv(8192)
            if not chunk:
                raise RuntimeError("openocd closed the connection")
            buf += chunk
        return buf[:-1].decode(errors="replace").strip()

    def read_words(self, addr, count):
        out = self.cmd(f"read_memory 0x{addr:08x} 32 {count}")
        vals = [int(v, 0) for v in out.split()]
        if len(vals) != count:
            raise RuntimeError(f"read_memory returned {len(vals)} words, wanted {count}: {out!r}")
        return vals

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True)


def start_remote_ocd(board):
    cfg = BOARDS[board]
    sh(f"ssh {PI} 'sudo pkill -x openocd' >/dev/null 2>&1")
    time.sleep(1)
    launch = (
        f"nohup sudo openocd {cfg} -c 'set CPUTAPID 0' -f target/stm32f1x.cfg "
        f"-c 'bindto 0.0.0.0' -c init > /tmp/ocd-trace.log 2>&1 &"
    )
    sh(f"ssh {PI} \"{launch}\"")
    for _ in range(20):
        time.sleep(0.5)
        r = sh(f"ssh {PI} \"grep -c 'Listening on port 6666' /tmp/ocd-trace.log\"")
        if r.stdout.strip() == "1":
            return PI.split("@")[-1]
    sys.exit("motor-trace: OpenOCD did not come up on the Pi (see /tmp/ocd-trace.log there)")


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", choices=sorted(BOARDS), default="master")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--attach", default=None, help="HOST:PORT of a running OpenOCD TCL port")
    args = ap.parse_args()

    addrs = resolve_symbols(args.elf)
    base = min(addrs[s] for s in MOTOR_SYMS)
    span = (max(addrs[s] for s in MOTOR_SYMS) - base) // 4 + 1
    off = {s: (addrs[s] - base) // 4 for s in MOTOR_SYMS}

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
        atexit.register(lambda: sh(f"ssh {PI} 'sudo pkill -x openocd'"))

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
             "state mode moe rx_ovr rx_lerr").split()
        )

    hdr = (
        f"{'t':>6} {'demand':>7} {'torque':>7} {'speed':>7} "
        f"{'d0':>5} {'d1':>5} {'d2':>5} {'ang':>5} {'dper':>6} "
        f"{'dwell':>5} {'fault':>5} {'mode':>4} {'moe':>3} {'rxovr':>6} {'rxerr':>6}"
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
        c = ocd.read_words(addrs["CTRL_OBS"] + 44, 2)  # torque|mode|moe, then substate|cmode|flags|levels
        # Word 29: the BLE port's RX losses, two saturating u16 halves (overruns | line errors).
        # This is the counter that tells a drive-demand sag caused by the BOARD dropping inbound
        # bytes apart from one caused by the phone not delivering them: overruns stepping in time
        # with a sag is the first, a flat counter during a sag is the second.
        rx = ocd.read_words(addrs["CTRL_OBS"] + 0x74, 1)[0]
        rx_ovr, rx_lerr = rx & 0xFFFF, (rx >> 16) & 0xFFFF

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
            f"{dwell:5d} {fault:5d} {mode:4d} 0x{moe:02x} {rx_ovr:6d} {rx_lerr:6d}",
            flush=True,
        )
        if writer:
            writer.writerow([f"{t:.3f}", demand, torque, speed, d0, d1, d2, ang,
                             periods, dper, dwell, fault, state, mode, moe,
                             rx_ovr, rx_lerr])
        row_n += 1

        slack = period - (time.time() - loop_start)
        if slack > 0:
            time.sleep(slack)


if __name__ == "__main__":
    main()
