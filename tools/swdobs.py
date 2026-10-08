"""Shared SWD observation plumbing for the host-side bench tools.

One owner for the things every tool that reads the running target needs: the bench probe table
(sourced from tools/flash.sh, never re-typed), the remote OpenOCD start/stop on the Pi, the
OpenOCD TCL client, the ELF symbol resolver (mangled-suffix match for the motor statics, the
CTRL_OBS address and size), and the CTRL_OBS word map. Imported by tools/motor-trace.py and
tools/climit-session.py. tools/imu-tilt.py still carries its own copy (noted in its header).

The core is NEVER halted by anything here: reads go through `read_memory` on the running target.
"""

import os
import re
import socket
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PI = os.environ.get("PI_HOST", "pi@192.168.0.248")
DEFAULT_ELF = os.path.join(REPO, "target/thumbv7m-none-eabi/release/firmware")
FLASH_SH = os.path.join(REPO, "tools", "flash.sh")

# --------------------------------------------------------------------------------------------------
# The CTRL_OBS word map: `struct CtrlObs` in crates/firmware/src/main.rs (#[repr(C)], 33 words; the
# compile-time offset pins there are the authority, these indices mirror them).
# --------------------------------------------------------------------------------------------------
CTRL_MAGIC = 0x4C525443  # "CTRL", little-endian in memory
CTRL_OBS_WORDS = 33
W_BOOT_COUNT = 1
W_TICK_COUNT = 2
W_CONTROL_TICKS = 4
W_ENACT_INITS = 9
W_ENACT_SHUTDOWNS = 10
W_TORQUE_MODE = 11      # torque i16 | mode_byte << 16 | moe_bits << 24
W_SUB_FLAGS = 12        # sub_state | control_mode << 8 | flags << 16 | event_levels << 24
W_PERIODS = 19
W_MOTOR_STATE = 20      # hall_code | enables << 8 | method << 16 | flags << 24 (motor.rs pack_obs_state)
W_DUTY01 = 21           # d0 | d1 << 16
W_DUTY2_ANGLE = 22      # d2 | angle << 16
W_MOTOR_FAULT = 23      # FAULT bits | invalid-hall dwell << 16
W_MOTOR_SPEED = 24
W_MOTOR_CAL = 25
W_EVENTS_LO = 27        # event_counts[0..4], one byte per producer
W_EVENTS_HI = 28        # event_counts[4..8]
W_BLE_RX = 29           # BLE port RX losses: overruns u16 | line errors u16 << 16
W_MOTOR_CURRENT = 31    # peak i16 | chopped << 16 | trips << 24
W_BATTERY = 32
# CTRL_OBS word 31, motor_current: peak phase-current magnitude over the last 64-period window
# (i16, stock current counts) | chopped periods << 16 | trip count low byte << 24.
MOTOR_CURRENT_OFFSET = 4 * W_MOTOR_CURRENT
# Stock current counts per amp: crates/firmware/src/motor.rs COUNTS_PER_AMP (provisional until
# the energised bench gate confirms it; keep the two in step).
COUNTS_PER_AMP = 800

# Motor statics, resolved by mangled-name suffix. They sit contiguously so one read covers them.
MOTOR_SYMS = [
    "DEMAND_SEQ", "OBS_DUTY01", "INVALID_DWELL", "COUNTER_RUNNING", "OBS_DUTY2_ANGLE",
    "FAULT", "SPEED", "DEMAND", "OBS_CAL", "PERIODS", "OBS_STATE",
]

# The bench boards reached through the Pi's USB probes. Their OpenOCD configuration is read out of
# tools/flash.sh's BOARD table (`oc_cfg`); the offroad pair is reached over the network probes and
# wants the local patched OpenOCD instead, so it is not offered here.
BOARDS = ("master", "slave")


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def s32(v):
    return v - 0x100000000 if v & 0x80000000 else v


def extract_oc_cfg(board, text):
    """The OC_CFG string for `board` from the text of tools/flash.sh (the tilt-session.sh parse):
    the first `OC_CFG="..."` line after the `<board>)` case label. None when absent."""
    found = False
    for line in text.splitlines():
        if not found:
            if re.match(rf"^\s*{re.escape(board)}\)", line):
                found = True
            continue
        m = re.match(r'^\s*OC_CFG="(.*)"\s*;;\s*$', line)
        if m:
            return m.group(1)
    return None


def oc_cfg(board, who="motor-trace"):
    """The probe configuration for a bench board, from tools/flash.sh (the single source)."""
    try:
        with open(FLASH_SH) as fh:
            cfg = extract_oc_cfg(board, fh.read())
    except OSError:
        cfg = None
    if not cfg:
        sys.exit(f"{who}: could not extract the {board} OC_CFG from {FLASH_SH} (its format changed?)")
    return cfg


def resolve_symbols_sized(elf, who="motor-trace"):
    """{name: (address, size)} for every motor static plus CTRL_OBS, from the ELF's symbol table."""
    try:
        out = subprocess.check_output(
            ["arm-none-eabi-nm", "-S", elf], text=True, stderr=subprocess.DEVNULL
        )
    except (OSError, subprocess.CalledProcessError):
        sys.exit(f"{who}: cannot read symbols from {elf} (is arm-none-eabi-nm on PATH?)")
    return parse_nm(out, elf, who)


def parse_nm(out, elf="<elf>", who="motor-trace"):
    """Parse `nm -S` output (sized lines carry four columns, unsized three)."""
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4:
            addr, size, _kind, name = parts
            size = int(size, 16)
        elif len(parts) == 3:
            addr, _kind, name = parts
            size = None
        else:
            continue
        if name == "CTRL_OBS":
            syms["CTRL_OBS"] = (int(addr, 16), size)
            continue
        m = re.match(r"_RNvNtCs[A-Za-z0-9_]+_8firmware5motor\d+([A-Z_0-9]+)(\.\d+)?$", name)
        if m and m.group(1) in MOTOR_SYMS:
            syms[m.group(1)] = (int(addr, 16), size)

    missing = [s for s in MOTOR_SYMS + ["CTRL_OBS"] if s not in syms]
    if missing:
        sys.exit(f"{who}: symbols absent from {elf}: {', '.join(missing)}")
    return syms


def resolve_symbols(elf, who="motor-trace"):
    """Address of every motor static plus CTRL_OBS, from the ELF's symbol table."""
    return {k: v[0] for k, v in resolve_symbols_sized(elf, who).items()}


def motor_block(addrs):
    """(base, span in words, {static: word offset}) for the one read that covers the motor statics."""
    base = min(addrs[s] for s in MOTOR_SYMS)
    span = (max(addrs[s] for s in MOTOR_SYMS) - base) // 4 + 1
    off = {s: (addrs[s] - base) // 4 for s in MOTOR_SYMS}
    return base, span, off


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


def stop_remote_ocd_cmd():
    return f"ssh {PI} 'sudo pkill -x openocd'"


def start_remote_ocd_cmds(board, log, bind_all, who="motor-trace"):
    """The three commands of a remote OpenOCD start: kill any old one, launch, poll its log."""
    launch = (
        f"nohup sudo openocd {oc_cfg(board, who)} "
        + ("-c 'bindto 0.0.0.0' " if bind_all else "")
        + f"-c init > {log} 2>&1 &"
    )
    return (
        f"ssh {PI} 'sudo pkill -x openocd' >/dev/null 2>&1",
        f"ssh {PI} \"{launch}\"",
        f"ssh {PI} \"grep -c 'Listening on port 6666' {log}\"",
    )


def start_remote_ocd(board, who="motor-trace", log="/tmp/ocd-trace.log", bind_all=True,
                     run=sh, sleep=time.sleep):
    """Start OpenOCD on the Pi for a bench board (attach without halting) and wait for its TCL
    port. Returns the Pi's host name. `run` and `sleep` are the caller's side-effect seam."""
    kill, launch, poll = start_remote_ocd_cmds(board, log, bind_all, who)
    run(kill)
    sleep(1)
    run(launch)
    for _ in range(20):
        sleep(0.5)
        r = run(poll)
        if r.stdout.strip() == "1":
            return PI.split("@")[-1]
    sys.exit(f"{who}: OpenOCD did not come up on the Pi (see {log} there)")


def stop_remote_ocd(run=sh):
    run(stop_remote_ocd_cmd())
