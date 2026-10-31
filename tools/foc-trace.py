#!/usr/bin/env python3
"""Sample the FOC observation words over a hoverboard bridge while someone drives the board.

The bench sampler of specs/foc-torque.md sections 7 and 10. The rover and the offroad pair are
reached only through their bridges (specs/hoverboard-bridge.md; the OpenOCD tools do not reach
them), so this polls `hoverboard-cli read` of the whole CTRL_OBS block, at up to 10 Hz, and never
halts the core: the bridge reads RAM through the MEM-AP exactly as tools/motor-trace.py's OpenOCD
does.

Usage:
    tools/foc-trace.py --tcp HOST:PORT (--cal-node NODE | --cal COUNTS) [--node BRIDGE] [--hz 10]
                       [--csv FILE] [--elf PATH] [--frame DIRECTION ALIGN_OFFSET]
                       [--cli PATH]
    (--serial DEVICE or --device NAME in place of --tcp, as hoverboard-cli takes them)

Ctrl-C to stop. It takes no bench lock: it touches no probe, only the bridge, and the gate
session it samples for holds the lock (specs/foc-torque.md section 10).

What it prints, per sample (words of motorcontroller/firmware/src/main.rs `CtrlObs`, through the
tools/swdobs.py word map, which owns the indices):

- word 11, the torque word the demand produced (the throttle conditioner sits between
  `hoverboard-cli drive <value>` and it, so every gate criterion refers to this number);
- word 20, the motor's method byte (2 = FOC); word 23, the FAULT bits;
- word 22's angle, word 24's edge count and word 39's period-based rate;
- word 31, the window's peak magnitude, `chopped` and `trips`;
- words 47 to 49, the FOC observation, every one a 64-period window mean (specs/foc-torque.md
  section 7): `ia`, `ib` (every method), `id`, `iq`, `iq_ref` and `vq` (0 unless FOC), in counts
  and, for the currents, in amps at the board's own `motor.current_cal` (0x67), read off it at
  stand-up with `hoverboard-cli config` (or given with --cal, and then said so).

The CSV adds word 11's mode byte.

With --frame DIRECTION ALIGN_OFFSET (the board's staged `motor.direction` 0x62 and
`motor.align_offset` 0x63), it also runs the crate's own Clarke and forward Park on this host over
word 47 at `angle + angle_path_offset(direction, align_offset)`, bit-exact to
motorcontroller/commutation (`clarke`, `park_forward`, `lookup_sincos`, `angle_path_offset`): gate O2's
reading, in sine, with the rotor held. The angle is word 22's instantaneous one and the currents a
window mean, which agree only while the rotor does not move; that is the gate's condition.

The CTRL_OBS address and the `FOC_GAINS` address (the current loops' live gain knob,
motorcontroller/commutation/src/foc.rs) both come from the ELF's symbol table, never from a constant, and
the block's size is checked against the word map: a stale address reads plausible rubbish. The
magic word ("CTRL") is checked on the first read for the same reason.
"""

import argparse
import atexit
import csv
import math
import os
import re
import signal
import subprocess
import sys
import time

from swdobs import (
    CTRL_MAGIC, CTRL_OBS_WORDS, DEFAULT_ELF, REPO, W_DUTY2_ANGLE, W_FOC_DQ_MEAN, W_FOC_HOLD,
    W_FOC_PHASE_MEAN, W_FOC_REF_VQ, W_MOTOR_CURRENT, W_MOTOR_FAULT, W_MOTOR_RATE_Q, W_MOTOR_SPEED, W_MOTOR_STATE,
    W_TORQUE_MODE, CURRENT_CAL_FIELD, clamp_current_cal, i16_pair, resolve_symbols_sized, s16, s32,
)

TOOL = "foc-trace"
# The built CLI (tools/hoverboard-cli, a member of the tools/ workspace, built for the host with
# `cargo build --release -p hoverboard-cli` in tools/, so into tools/target), unless --cli says
# otherwise or it is on PATH.
CLI_DIR = os.path.join(REPO, "tools", "target")

# --------------------------------------------------------------------------------------------------
# The frame, bit-exact to motorcontroller/commutation/src/foc.rs and lib.rs.
# --------------------------------------------------------------------------------------------------
# Quarter-wave sine table: round(32767 * sin((i/256) * pi/2)), i = 0..255, as the crate's
# sin_table.rs literal (the test checks every entry against that file). Half-away-from-zero rounding,
# as Rust's f64::round, which Python's round() is not.
SIN_QUARTER = [int(math.floor(32767.0 * math.sin(i / 256.0 * math.pi / 2.0) + 0.5)) for i in range(256)]
CLARKE_A = 0x49E6  # Q15 1/sqrt(3)
CLARKE_B = 0x93CC  # Q15 2/sqrt(3)
# commutation::DECODE_SECTOR_LEAD: the six-step decode drives two sectors ahead of the anchors.
DECODE_SECTOR_LEAD = 2


def _i32(x):
    """Wrap to i32, as the crate's `wrapping_add` does."""
    x &= 0xFFFFFFFF
    return x - 0x100000000 if x & 0x80000000 else x


def sat16(x):
    """foc::sat16: saturate to [-32767, 32767], the -32768 sentinel to -32767."""
    if x > 32767:
        return 32767
    if x <= -32768:
        return -32767
    return x


def mac_q15(acc):
    """foc::mac_q15: the Q15 shift with the crate's bias (2^15 - 1 on a negative, so it truncates
    toward zero), then saturate."""
    bias = 0x7FFF if acc < 0 else 0
    return sat16(_i32(acc + bias) >> 15)


def lookup_sincos(theta):
    """foc::lookup_sincos: (sin, cos) of a 16-bit angle, Q15, quadrant-folded."""
    t = theta & 0xFFFF
    quad = ((t + 0x8000) >> 6) & 0x300
    i = (t & 0x3FFF) >> 6
    ti, tj = SIN_QUARTER[i], SIN_QUARTER[0xFF - i]
    return {0x000: (-ti, -tj), 0x100: (-tj, ti), 0x200: (ti, tj), 0x300: (tj, -ti)}[quad]


def clarke(ia, ib):
    """foc::clarke: (alpha, beta), alpha = ia, beta = MAC(-(ia * A + ib * B))."""
    return ia, mac_q15(-(ia * CLARKE_A + ib * CLARKE_B))


def park_forward(alpha, beta, theta):
    """foc::park_forward: (iq, id), iq along theta (the torque axis), id 90 degrees behind it."""
    s, c = lookup_sincos(theta)
    return mac_q15(alpha * c - beta * s), mac_q15(alpha * s + beta * c)


def sector_angle(sectors):
    """commutation::sector_angle: sixths of a turn, each rounded on its own."""
    return ((sectors % 6) * 65536 + 3) // 6


def angle_path_offset(direction, align_offset):
    """commutation::angle_path_offset for the staged bytes: `direction` 0x62 (0 forward, nonzero
    reverse, the field's own rule) and `align_offset` 0x63 (taken mod 6, as SixStep::new does)."""
    half = 0x8000 if direction else 0
    return (sector_angle(DECODE_SECTOR_LEAD + align_offset % 6) + half) & 0xFFFF


def host_frame(ia, ib, angle, direction, align_offset):
    """Gate O2's reading: (theta, iq, id, magnitude) of the mean phase pair at the aligned angle."""
    theta = (angle + angle_path_offset(direction, align_offset)) & 0xFFFF
    iq, i_d = park_forward(*clarke(ia, ib), theta)
    return theta, iq, i_d, math.hypot(iq, i_d)


# --------------------------------------------------------------------------------------------------
# The bridge read and the decode.
# --------------------------------------------------------------------------------------------------
HEXDUMP_RE = re.compile(r"^([0-9a-fA-F]{8}):((?: [0-9a-fA-F]{2})+)\s*$")
# `hoverboard-cli config <node> 0x67.0` prints `0x67.0 at 0x01 (name): ok 455 (0x01c7)`.
CONFIG_RE = re.compile(r"^0x([0-9a-fA-F]+)\.(\d+) at .*?: (.+?)(?: (-?\d+)(?: \(0x[0-9a-fA-F]+\))?)?\s*$")


def parse_hexdump(text, addr, length):
    """The bytes of `hoverboard-cli read <addr> <len>` (its `%08x: bb bb ...` rows, 16 to a row).
    Raises RuntimeError on a short, non-contiguous or unparsable answer."""
    data = bytearray()
    for line in text.splitlines():
        m = HEXDUMP_RE.match(line.strip())
        if not m:
            continue
        row_addr = int(m.group(1), 16)
        if row_addr != addr + len(data):
            raise RuntimeError(f"read row at 0x{row_addr:08x}, expected 0x{addr + len(data):08x}")
        data.extend(int(b, 16) for b in m.group(2).split())
    if len(data) != length:
        raise RuntimeError(f"read returned {len(data)} B, wanted {length}: {text.strip()[-200:]!r}")
    return bytes(data)


def words_le(data):
    """Little-endian u32 words of a byte string whose length is a multiple of four."""
    return [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data), 4)]


def parse_config(text, field):
    """(status, value) of `hoverboard-cli config`'s line for `field` (value None on a refusal);
    (None, None) when no such line was printed."""
    for line in text.splitlines():
        m = CONFIG_RE.match(line.strip())
        if m and int(m.group(1), 16) == field:
            return m.group(3), int(m.group(4)) if m.group(4) is not None else None
    return None, None


def decode(w, cal, frame=None):
    """The row of one CTRL_OBS read (`w`, CTRL_OBS_WORDS words), as a dict. `cal` is the counts per
    amp the currents are scaled at; `frame` is (direction, align_offset) for the host frame, or None.
    """
    torque = s16(w[W_TORQUE_MODE] & 0xFFFF)
    state = w[W_MOTOR_STATE]
    cur = w[W_MOTOR_CURRENT]
    ia, ib = i16_pair(w[W_FOC_PHASE_MEAN])
    fid, fiq = i16_pair(w[W_FOC_DQ_MEAN])
    iq_ref, vq = i16_pair(w[W_FOC_REF_VQ])
    hold = w[W_FOC_HOLD]
    r = {
        "torque": torque,
        "mode": (w[W_TORQUE_MODE] >> 16) & 0xFF,
        "method": (state >> 16) & 0xFF,
        "mflags": (state >> 24) & 0xFF,
        "angle": (w[W_DUTY2_ANGLE] >> 16) & 0xFFFF,
        "fault": w[W_MOTOR_FAULT] & 0xFFFF,
        "speed": s32(w[W_MOTOR_SPEED]),
        "rate": s32(w[W_MOTOR_RATE_Q]) / 4096.0,
        "peak": s16(cur & 0xFFFF),
        "chopped": (cur >> 16) & 0xFF,
        "trips": (cur >> 24) & 0xFF,
        "ia": ia, "ib": ib, "id": fid, "iq": fiq, "iq_ref": iq_ref, "vq": vq,
        # The hold word (word 50): held periods per sensed phase, floated periods, the cause byte.
        "held_a": hold & 0xFF, "held_b": (hold >> 8) & 0xFF, "coasted": (hold >> 16) & 0xFF,
        "cause": (hold >> 24) & 0xFF,
    }
    for k in ("ia", "ib", "id", "iq", "iq_ref"):
        r[k + "_a"] = r[k] / cal
    if frame is not None:
        theta, hiq, hid, mag = host_frame(ia, ib, r["angle"], *frame)
        r.update(theta=theta, hiq=hiq, hid=hid, hmag=mag,
                 hiq_frac=hiq / mag if mag else 0.0, hid_frac=abs(hid) / mag if mag else 0.0)
    return r


def find_cli(given):
    """The hoverboard-cli to run: --cli, else the workspace's release build, else PATH."""
    if given:
        return given
    if os.path.isdir(CLI_DIR):
        for root, _dirs, files in os.walk(CLI_DIR):
            if os.path.basename(root) == "release" and "hoverboard-cli" in files:
                return os.path.join(root, "hoverboard-cli")
    return "hoverboard-cli"


class Bridge:
    """`hoverboard-cli` over one link. `run(argv)` returns the subprocess shape (`.returncode`,
    `.stdout`, `.stderr`), so a test passes a fake instead of a bridge."""

    def __init__(self, cli, link, node=None, run=None):
        self.cli, self.link, self.node = cli, list(link), node
        self.run = run or (lambda argv: subprocess.run(argv, capture_output=True, text=True, timeout=10))

    def argv(self, *cmd, node=True):
        a = [self.cli] + self.link
        if node and self.node:
            a += ["--node", self.node]
        return a + [str(c) for c in cmd]

    def read(self, addr, length):
        r = self.run(self.argv("read", f"0x{addr:08x}", length))
        if r.returncode != 0:
            raise RuntimeError(f"hoverboard-cli read failed: {(r.stderr or r.stdout).strip()[-200:]}")
        return parse_hexdump(r.stdout, addr, length)

    def read_words(self, addr, count):
        return words_le(self.read(addr, 4 * count))

    def config_read(self, node, field, index=0):
        # `config` walks the machine and addresses the node itself, so no --node.
        r = self.run(self.argv("config", node, f"0x{field:02x}.{index}", node=False))
        return parse_config(r.stdout, field)


def read_cal(bridge, node):
    """`motor.current_cal` (0x67) of motor 0 on `node`: (raw, effective). Exits when it cannot."""
    status, value = bridge.config_read(node, CURRENT_CAL_FIELD)
    if status != "ok" or value is None:
        sys.exit(f"{TOOL}: reading 0x67.0 on {node} failed (status {status}); give --cal to scale "
                 "the amps columns by hand")
    return value, clamp_current_cal(value)


def resolve(elf):
    """(CTRL_OBS address, FOC_GAINS address or None) from the ELF, with the block size checked."""
    syms = resolve_symbols_sized(elf, who=TOOL)
    addr, size = syms["CTRL_OBS"]
    if size is not None and size != 4 * CTRL_OBS_WORDS:
        sys.exit(f"{TOOL}: CTRL_OBS is {size} B in {elf}, the word map decodes {4 * CTRL_OBS_WORDS} B "
                 f"({CTRL_OBS_WORDS} words): the ELF and the tool disagree")
    gains = syms.get("FOC_GAINS")
    return addr, gains[0] if gains else None


FOC_GAIN_FIELDS = ("kp", "kp_divisor", "ki", "ki_divisor", "out_max", "int_max")


def gains_text(addr, words):
    """The FOC_GAINS report: its address, each field's address and live value."""
    lines = [f"FOC_GAINS at 0x{addr:08x} (hoverboard-cli write <addr> <8 hex digits, little-endian>):"]
    for i, (name, v) in enumerate(zip(FOC_GAIN_FIELDS, words)):
        lines.append(f"  0x{addr + 4 * i:08x} {name:<10} {s32(v)}")
    return "\n".join(lines)


COLUMNS = ("t torque mode method fault angle speed rate peak chopped trips "
           "ia ib id iq iq_ref vq ia_a ib_a id_a iq_a iq_ref_a held_a held_b coasted cause").split()
FRAME_COLUMNS = "theta hiq hid hmag hiq_frac hid_frac".split()


def row_text(t, r):
    s = (f"{t:6.1f} {r['torque']:6d} {r['method']:2d} {r['fault']:5d} {r['angle']:5d} {r['speed']:6d} "
         f"{r['rate']:7.2f} {r['peak']:6d} {r['chopped']:4d} {r['trips']:4d}  "
         f"{r['ia']:6d} {r['ib']:6d} {r['id']:6d} {r['iq']:6d} {r['iq_ref']:6d} {r['vq']:6d}  "
         f"{r['ia_a']:6.2f} {r['ib_a']:6.2f} {r['id_a']:6.2f} {r['iq_a']:6.2f} {r['iq_ref_a']:6.2f}  "
         f"{r['held_a']:3d} {r['held_b']:3d} {r['coasted']:3d} {r['cause']:02x}")
    if "theta" in r:
        s += (f"  {r['theta']:5d} {r['hiq']:6d} {r['hid']:6d} {r['hmag']:7.1f} "
              f"{r['hiq_frac']:5.2f} {r['hid_frac']:5.2f}")
    return s


def header(frame):
    h = (f"{'t':>6} {'torque':>6} {'me':>2} {'fault':>5} {'angle':>5} {'speed':>6} {'rate':>7} "
         f"{'peak':>6} {'chop':>4} {'trip':>4}  {'ia':>6} {'ib':>6} {'id':>6} {'iq':>6} {'iqref':>6} "
         f"{'vq':>6}  {'iaA':>6} {'ibA':>6} {'idA':>6} {'iqA':>6} {'refA':>6}  "
         f"{'hdA':>3} {'hdB':>3} {'cst':>3} {'cz':>2}")
    if frame:
        h += f"  {'theta':>5} {'hiq':>6} {'hid':>6} {'hmag':>7} {'iq/m':>5} {'id/m':>5}"
    return h


def main(argv=None):
    ap = argparse.ArgumentParser(description="FOC observation sampler over a hoverboard bridge")
    link = ap.add_mutually_exclusive_group(required=True)
    link.add_argument("--tcp", help="HOST:PORT of the bridge's console (<name>.local:5333)")
    link.add_argument("--serial", help="the bridge's USB console device")
    link.add_argument("--device", help="the bridge's BLE name or address")
    ap.add_argument("--node", help="read through the bridge at this node rather than the one on the "
                    "link (hoverboard-cli --node: an ESP command, reached through the boards)")
    ap.add_argument("--hz", type=float, default=10.0, help="sample rate, at most 10")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--cli", default=None, help="the hoverboard-cli binary")
    cal = ap.add_mutually_exclusive_group()
    cal.add_argument("--cal-node", help="the sampled board's node (address or device.name), whose "
                     "0x67.0 scales the amps")
    cal.add_argument("--cal", type=int, help="counts per amp, given rather than read off the board")
    ap.add_argument("--frame", nargs=2, type=int, metavar=("DIRECTION", "ALIGN_OFFSET"),
                    help="run Clarke and Park on the host over word 47 (gate O2)")
    args = ap.parse_args(argv)
    if not 0 < args.hz <= 10:
        sys.exit(f"{TOOL}: --hz {args.hz}: the bridge read is polled at up to 10 Hz")

    ctrl, gains = resolve(args.elf)
    link_args = (["--tcp", args.tcp] if args.tcp else
                 ["--serial", args.serial] if args.serial else ["--device", args.device])
    bridge = Bridge(find_cli(args.cli), link_args, args.node)

    if args.cal is not None:
        cpa, cal_note = clamp_current_cal(args.cal), f"--cal {args.cal} counts per amp (given, not read)"
    else:
        # The board's own node, not --node: that names the bridge the read goes through.
        node = args.cal_node
        if not node:
            sys.exit(f"{TOOL}: the amps need the board's 0x67: give --cal-node (its address or "
                     "device.name, as `hoverboard-cli nodes` lists it) or --cal")
        raw, cpa = read_cal(bridge, node)
        cal_note = f"motor.current_cal (0x67) on {node} = {raw} counts per amp"
        if cpa != raw:
            cal_note += f", which the firmware's boot seam clamps to {cpa}"

    try:
        first = bridge.read_words(ctrl, CTRL_OBS_WORDS)
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as e:
        sys.exit(f"{TOOL}: {e}")
    if first[0] != CTRL_MAGIC:
        sys.exit(f"{TOOL}: CTRL_OBS magic is 0x{first[0]:08x}, expected 0x{CTRL_MAGIC:08x}.\n"
                 "  The flashed image is not the one at --elf, so every address here is wrong.")

    print(f"elf={os.path.relpath(args.elf, REPO)} CTRL_OBS at 0x{ctrl:08x}, {args.hz} Hz, Ctrl-C to stop")
    if gains is None:
        print("FOC_GAINS: absent from the ELF")
    else:
        try:
            print(gains_text(gains, bridge.read_words(gains, len(FOC_GAIN_FIELDS))))
        except (RuntimeError, OSError, subprocess.TimeoutExpired) as e:
            print(f"FOC_GAINS at 0x{gains:08x} (its values could not be read: {e})")
    print(f"  {cal_note}: the A columns are counts / {cpa}.")
    print("  torque = CTRL_OBS word 11, the word the demand produced; me = word 20's method (2 FOC).")
    print("  ia ib = word 47, the mean phase currents (every method); id iq = word 48 and iqref vq =")
    print("  word 49, the FOC loops' window means and closing reference (0 unless FOC). vq in voltage")
    print("  counts. A nonzero chop while FOC drives is a finding, not a limiter at work.")
    frame = tuple(args.frame) if args.frame else None
    if frame:
        print(f"  --frame direction {frame[0]}, align offset {frame[1]}: theta = angle + "
              f"{angle_path_offset(*frame)}; hiq hid = the crate's Clarke and Park over word 47 there,")
        print("  hmag their magnitude, iq/m and id/m the gate O2 ratios (PASS: iq/m >= 0.9, id/m <= 0.2).")
    print()
    hdr = header(frame)
    print(hdr)
    print("-" * len(hdr))

    writer = None
    if args.csv:
        fh = open(args.csv, "w", newline="")
        atexit.register(fh.close)
        writer = csv.writer(fh)
        writer.writerow(COLUMNS + (FRAME_COLUMNS if frame else []))

    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    t0 = time.time()
    period = 1.0 / args.hz
    failed = row_n = 0
    w = first
    while True:
        loop_start = time.time()
        if w is None:
            try:
                w = bridge.read_words(ctrl, CTRL_OBS_WORDS)
            except (RuntimeError, OSError, subprocess.TimeoutExpired) as e:
                # One lost read on the wireless link is a dropped sample; a run of them is the link.
                failed += 1
                print(f"  -- sample dropped ({e})", flush=True)
                if failed >= 5:
                    sys.exit(f"{TOOL}: five consecutive reads failed, the link is gone")
                continue
        failed = 0
        r = decode(w, cpa, frame)
        w = None
        t = time.time() - t0
        if row_n and row_n % 25 == 0:
            print(hdr)
        print(row_text(t, r), flush=True)
        if writer:
            vals = [f"{t:.3f}"] + [r[c] for c in COLUMNS[1:]]
            vals += [r[c] for c in FRAME_COLUMNS] if frame else []
            writer.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in vals])
        row_n += 1
        slack = period - (time.time() - loop_start)
        if slack > 0:
            time.sleep(slack)


if __name__ == "__main__":
    main()
