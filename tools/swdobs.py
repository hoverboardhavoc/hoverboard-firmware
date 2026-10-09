"""Shared SWD observation plumbing for the host-side bench tools.

One owner for the things every tool that reads the running target needs: the bench probe table
(sourced from tools/flash.sh, never re-typed), the remote OpenOCD start/stop on the Pi, the
OpenOCD TCL client, the ELF symbol resolver (mangled-suffix match for the motor statics, the
CTRL_OBS address and size), the CTRL_OBS word map, and the two per-motor reads the
current limit is built from (the board's own counts per amp, 0x67, and its own phase-current noise
floor, 0x6B, neither of which any tool carries a copy of). Imported by tools/motor-trace.py and
tools/climit-session.py. tools/imu-tilt.py still carries its own copy (noted in its header).

The core is NEVER halted by anything here: reads go through `read_memory` on the running target.
"""

import os
import re
import shlex
import socket
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PI = os.environ.get("PI_HOST", "pi@192.168.0.248")
DEFAULT_ELF = os.path.join(REPO, "target/thumbv7m-none-eabi/release/firmware")
FLASH_SH = os.path.join(REPO, "tools", "flash.sh")

# --------------------------------------------------------------------------------------------------
# The CTRL_OBS word map: `struct CtrlObs` in crates/firmware/src/main.rs (#[repr(C)], 34 words; the
# compile-time offset pins there are the authority, these indices mirror them).
# --------------------------------------------------------------------------------------------------
CTRL_MAGIC = 0x4C525443  # "CTRL", little-endian in memory
CTRL_OBS_WORDS = 34
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
W_ARM_REFUSALS = 33     # ReReadValues refusals | ConfirmPeriodsLive << 8 | frame cause << 16
# CTRL_OBS word 31, motor_current: peak phase-current magnitude over the last 64-period window
# (i16, stock current counts) | chopped periods << 16 | trip count low byte << 24.
MOTOR_CURRENT_OFFSET = 4 * W_MOTOR_CURRENT
# --------------------------------------------------------------------------------------------------
# CTRL_OBS word 33, arm_refusals: the two arm refusals, counted per boot (specs/integration.md, "The
# arm refusals (word 33, permanent)"). It is a COUNT and not a level because the ReReadValues
# refusal is released when the engage that met it ends (an OFF pass with the power request dropped),
# so the level is gone by the time a bench read looks: a board refused for a bad axis frame and then
# left disarmed reads like a board that was never engaged.
#
# The cause byte mirrors crates/firmware/src/main.rs `imu_frame_cause_byte`: bit 7 = a cause is
# present (the accel refusal encodes field and index both as 0, so the flag is what tells it from
# "no frame refusal"), bit 2 = the refusal names IMU_AXIS_ROLE rather than IMU_AXIS_SIGN, bits 0..1 =
# the refused triple's first IMU_AXIS_SIGN index, 0 = accel and 3 = gyro, exactly as BOARD_OBS's
# `detail` carries it.
# --------------------------------------------------------------------------------------------------
CAUSE_PRESENT = 0x80
CAUSE_NAMES_ROLE = 0x04


def decode_arm_refusals(word):
    """(re_read count, confirm count, cause text) of CTRL_OBS word 33."""
    cause = (word >> 16) & 0xFF
    if not cause & CAUSE_PRESENT:
        text = "no frame refusal"
    elif cause & CAUSE_NAMES_ROLE:
        text = "imu.axis_role (0x68): the roles are not two distinct chip axes in 1..3"
    else:
        first = cause & 0x03
        triple = "gyro" if first else "accel"
        text = f"imu.axis_sign (0x65) index {first}: the {triple} triple is a reflection"
    return word & 0xFF, (word >> 8) & 0xFF, text
# --------------------------------------------------------------------------------------------------
# The current-sense calibration: what a `peak` count means in amps.
#
# This is BOARD data, not a tool constant: crates/store/src/field.rs MOTOR_CURRENT_CAL (0x67, u16
# stock current counts per amp, per-motor via the key's index, default 455 from the 2026-10-09
# energised gate). It replaced the compiled COUNTS_PER_AMP = 800 this module used to carry, for the
# reason this module exists at all: one owner per fact, and the owner of a per-board fact is the
# board (specs/motor-integration.md, "The current-sense calibration"). So a tool that reports amps
# reads 0x67 off the attached board at stand-up through `read_current_cal` below.
# --------------------------------------------------------------------------------------------------
CURRENT_CAL_FIELD = 0x67
# What an unstaged 0x67 reads: the registry default, the value the firmware itself would use. Only
# for a report that says it could not read the board, never as a silent substitute for the read.
CURRENT_CAL_DEFAULT = 455
# The firmware's boot seam clamps the stored word before converting anything against it
# (crates/firmware/src/motor.rs CURRENT_CAL_MIN / CURRENT_CAL_MAX), so a tool reporting what the
# board will DO applies the same clamp to what it read.
CURRENT_CAL_MIN, CURRENT_CAL_MAX = 100, 819

# --------------------------------------------------------------------------------------------------
# The phase-current NOISE FLOOR: the smallest soft limit this board's sense chain can be held to.
#
# BOARD data for the same reason and in the same units as the scale above: crates/store/src/field.rs
# MOTOR_NOISE_FLOOR (0x6B, u16 stock current counts, per-motor via the key's index, default 2,100).
# It replaced a compiled firmware constant measured on ONE board in ONE session, whose five
# rest-floor reads that day spanned 1,444 to 2,097 counts; one number for every chain chops on noise
# at the noisy end and silently raises the minimum enforceable current at the quiet end
# (specs/store-field-audit.md, the 2026-10-09 sweep, Tier 1 item 1). So a tool that reports what a
# board will enforce reads 0x6B off it through `read_noise_floor` below.
# --------------------------------------------------------------------------------------------------
NOISE_FLOOR_FIELD = 0x6B
# What an unstaged 0x6B reads: the registry default, which IS the constant the field replaced. Only
# for a report that says it could not read the board, never as a silent substitute for the read.
NOISE_FLOOR_DEFAULT = 2_100
# The firmware's boot seam clamps the stored word into this plausibility band before the limit is
# floored at it (crates/firmware/src/motor.rs, noise_floor_counts: NOISE_FLOOR_MIN up to
# MAX_LIMIT_COUNTS), so a tool reporting what the board will DO applies the same clamp.
NOISE_FLOOR_MIN, NOISE_FLOOR_MAX = 1_000, 16_383

# One CONFIG_READ line of swd-mailbox-config's output: `CONFIG_READ 0x67:0 -> CFG_OK value U16(455)`,
# with the trailing text (a write's `(write -> read matches)`) kept as its own group.
CFG_READ_RE = re.compile(
    r"CONFIG_READ\s+0x([0-9a-fA-F]+):(\d+) -> (\S+)(?: value \w+\((-?\d+)\))?(.*)$"
)


def host_target():
    """The Rust host triple the mailbox tools are built for (HOST_TARGET overrides)."""
    if os.environ.get("HOST_TARGET"):
        return os.environ["HOST_TARGET"]
    m = os.uname().machine
    arch = {"arm64": "aarch64", "aarch64": "aarch64", "x86_64": "x86_64"}.get(m, m)
    if sys.platform == "darwin":
        return f"{arch}-apple-darwin"
    return f"{arch}-unknown-linux-gnu"


def mailbox_bin(name):
    """The path of one built mailbox tool (crates/swd-bridge is outside the workspace, so it has its
    own target directory)."""
    return os.path.join(REPO, "crates", "swd-bridge", "target", host_target(), "release", name)


def parse_config_read(text, field):
    """(status, value) of the LAST CONFIG_READ line for `field` (value None on a refusal);
    (None, None) when no such line was printed."""
    status = value = None
    for line in text.splitlines():
        m = CFG_READ_RE.search(line)
        if m and int(m.group(1), 16) == field:
            status = m.group(3)
            value = int(m.group(4)) if m.group(4) is not None else None
    return status, value


def clamp_current_cal(raw):
    """What the firmware's boot seam will convert against, given the stored word."""
    return min(max(raw, CURRENT_CAL_MIN), CURRENT_CAL_MAX)


def clamp_noise_floor(raw):
    """What the firmware's boot seam will FLOOR the limit at, given the stored word
    (crates/firmware/src/motor.rs, noise_floor_counts)."""
    return min(max(raw, NOISE_FLOOR_MIN), NOISE_FLOOR_MAX)


def _read_motor_field(run, endpoint, field, name, clamp, motor, dst, config_bin):
    """Read one per-motor u16 field off the attached board over the SWD mailbox, as `(raw,
    effective)`: the stored word and what the firmware's boot seam clamps it to.

    The ONE place a per-motor board fact is read, so the two facts the limit is built from (the
    scale and the floor) cannot be read two different ways. `run(cmd)` runs a shell command and
    returns the `sh` shape (`.returncode`, `.stdout`), so a caller with its own runner (a dry-run
    simulator, a logging wrapper) passes that instead of this module reaching for a subprocess of
    its own. Raises RuntimeError when the board did not answer: a tool that cannot read a board
    fact must say so, not report against a guess.
    """
    cfg = mailbox_bin("swd-mailbox-config") if config_bin is None else config_bin
    key = f"0x{field:02x}:{motor}"
    r = run(" ".join(shlex.quote(a) for a in [cfg, endpoint, "--dst", dst, key]))
    status, value = parse_config_read(r.stdout, field)
    if status != "CFG_OK" or value is None:
        raise RuntimeError(
            f"reading {key} ({name}) failed (status {status}): {r.stdout.strip()[-200:]}"
        )
    return value, clamp(value)


def read_current_cal(run, endpoint, motor=0, dst="attached", config_bin=None):
    """Read `motor.current_cal` (0x67) for `motor`: `(raw, effective)`. See `_read_motor_field`."""
    return _read_motor_field(run, endpoint, CURRENT_CAL_FIELD, "motor.current_cal",
                             clamp_current_cal, motor, dst, config_bin)


def read_noise_floor(run, endpoint, motor=0, dst="attached", config_bin=None):
    """Read `motor.noise_floor` (0x6B) for `motor`: `(raw, effective)`. See `_read_motor_field`."""
    return _read_motor_field(run, endpoint, NOISE_FLOOR_FIELD, "motor.noise_floor",
                             clamp_noise_floor, motor, dst, config_bin)

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
