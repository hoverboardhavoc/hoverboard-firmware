#!/usr/bin/env python3
"""Live attitude (pitch) readout for the hand-tilt IMU validation session.

specs/silicon-queue.md section 2 (Phase A: IMU + attitude) + the 2026-07-22 USER DECISIONS block:
"Hand-tilt orientation/sign validation: next bench visit ... agent drives SWD, user tilts". This
tool is the "agent drives SWD" half: a human tilts the board and the firmware's live pitch (in
degrees) streams to the terminal so orientation/sign checks are readable in real time.

WHAT IT READS
  The integrated firmware (crates/firmware/src/main.rs) publishes a whole-struct CTRL_OBS record
  into RAM every 250 Hz control pass. This tool reads the two words it needs out of that block over
  OpenOCD's memory-read (`mdw`), non-intrusively, on the RUNNING target (no halt -- halting would
  freeze the control loop the operator is tilting against).

  CTRL_OBS is `#[repr(C)]`; the word offsets used here (derived from the struct, cross-checked
  against specs/bench-evidence/2026-07-22/round17/01-imu-loss-fault-silicon.log):
      word  0  magic          = 0x4C525443 ("CTRL")   -- liveness marker
      word  6  pitch_milli    = i32, millidegrees      -- the published pitch
      word 12  packed         = sub_state | control_mode<<8 | flags<<16 | pad<<24
               flags byte     = (word12 >> 16) & 0xFF
                   b0 imu_configured  b1 imu_live  b2 comms_loss  b3 mode_fault  b4 imu_loss
  (round17 example: word0=4c525443, word6=fffff7c2 = -2110 mdeg = -2.11 deg, word12=00030000 ->
   flags 0x03 = CONFIGURED + LIVE.)

  The CTRL_OBS ADDRESS is resolved from the ELF symbol table every run (--elf), never hardcoded:
  the block's address has moved almost every round as .bss/.uninit shifted (round17: it moved +8 B
  again). --addr 0x... overrides the ELF lookup.

ROLL IS PUBLISHED (roll-field slice).  CTRL_OBS carries roll_milli as its last word (word 18),
  conditioned off the same held Mahony output + scale as pitch. The tool reads and reports it, but
  the ZYX roll axis + sign is still an open question (attitude.md "Open questions"): the checklist
  roll poses SAMPLE the live roll and report the observed sign, with verdict RECORDED (not
  PASS/CHECK) -- the human session confirms the sign against the physical motion.

CONNECTION (OpenOCD TCL RPC)
  The bench runs OpenOCD on the Pi (pi@192.168.0.248). Its TCL RPC listens on localhost:6666. This
  tool speaks only TCP; tunnel the port from your workstation:
      ssh -L 6666:localhost:6666 pi@192.168.0.248
  then run against localhost:6666 (the default). Or run this tool on the Pi directly.

USAGE
  # live streaming readout (default):
  tools/imu-tilt.py --elf target/thumbv7m-none-eabi/release/firmware
  # append a self-describing evidence CSV (schema v3: unix_time,pitch_mdeg,roll_mdeg,flags,label):
  tools/imu-tilt.py --record specs/bench-evidence/<date>/imu-tilt.csv
  #   while recording live, type a word + Enter to drop a `# marker <text>` row in the CSV.
  # guided sign checklist (prompts orientations, samples, prints PASS/CHECK):
  tools/imu-tilt.py --checklist --record specs/bench-evidence/<date>/imu-tilt.csv
  #   labels every sample row with the pose id and writes # step/# summary comment lines so the
  #   CSV alone reconstructs the session (windows, means, spec-expected sign, verdicts, provenance).
  # parser self-test, no hardware:
  tools/imu-tilt.py --selftest

EVIDENCE CSV (schema v3)
  A new file starts with `# columns: unix_time,pitch_mdeg,roll_mdeg,flags,label`. Data rows carry a
  `roll_mdeg` column (the published roll) and a `label` column: label empty in plain live mode, the
  pose id (level/lean_forward/lean_back/roll_right/roll_left) under --checklist. Comment lines (`#`)
  carry markers, step begin/end, and a final summary+provenance block. NOTE: earlier v2
  (pitch-only) / unlabeled recordings are superseded; re-record with this schema for anything that
  has to stand as evidence.

python3 stdlib only.
"""

import argparse
import io
import math
import os
import re
import select
import shutil
import socket
import struct
import subprocess
import sys
import time
from collections import deque

TOOL_VERSION = "imu-tilt/1.1"


def tool_commit():
    """Best-effort short git commit of this tool, for CSV provenance ('unknown' if unavailable)."""
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        out = subprocess.run(
            ["git", "-C", here, "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=3,
        )
        if out.returncode == 0 and out.stdout.strip():
            return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return "unknown"

# --------------------------------------------------------------------------------------------------
# CTRL_OBS layout (word indices into `mdw <CTRL_OBS> N`). Derived from crates/firmware/src/main.rs
# `struct CtrlObs` (#[repr(C)]); cross-checked vs specs/bench-evidence/2026-07-22/round17/.
# --------------------------------------------------------------------------------------------------
CTRL_OBS_MAGIC = 0x4C525443  # "CTRL" little-endian, word 0
MAGIC_WORD = 0
PITCH_WORD = 6  # pitch_milli: i32 at byte offset 24
FLAGS_WORD = 12  # packed word at byte offset 48; flags is byte 2
FLAGS_SHIFT = 16  # flags = (word12 >> FLAGS_SHIFT) & 0xFF
READ_WORDS = 20  # words to read per sample (through word 18 = roll + margin)

# ROLL_WORD: roll_milli was appended to CTRL_OBS as the last field (offset-preserving; the roll-field
# slice). It sits at word 18 / byte offset 72, published off the same held Mahony output + scale as
# pitch (main.rs `struct CtrlObs`). decode_sample() reads it into Sample.roll_mdeg.
ROLL_WORD = 18

# flags bits (main.rs publish_obs / round17 decode)
FLAG_CONFIGURED = 1 << 0
FLAG_LIVE = 1 << 1
FLAG_COMMS_LOSS = 1 << 2
FLAG_MODE_FAULT = 1 << 3
FLAG_LOSS = 1 << 4

STALE_SECONDS = 2.0  # pitch unchanged this long while LIVE = suspicious
SAMPLE_HZ = 10.0
CHECKLIST_SAMPLE_S = 2.0
# A tilt must move pitch at least this far from the level baseline to score PASS (else CHECK).
CHECKLIST_DELTA_MDEG = 3000  # 3.00 deg


# --------------------------------------------------------------------------------------------------
# ELF symbol resolution (pure-python ELF32 parse; the repo's audits use this pattern). Reads the
# symbol table for CTRL_OBS and returns its virtual address (st_value).
# --------------------------------------------------------------------------------------------------
def elf_resolve_symbol(path, name):
    """Return the virtual address of `name` in ELF32 `path`, or raise ValueError."""
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:4] != b"\x7fELF":
        raise ValueError(f"{path}: not an ELF file")
    ei_class = data[4]  # 1 = ELF32
    ei_data = data[5]  # 1 = little-endian
    if ei_class != 1:
        raise ValueError(f"{path}: not ELF32 (thumbv7m image expected)")
    if ei_data != 1:
        raise ValueError(f"{path}: not little-endian")
    end = "<"  # little-endian
    # ELF32 header fields we need.
    (e_shoff,) = struct.unpack_from(end + "I", data, 0x20)
    (e_shentsize,) = struct.unpack_from(end + "H", data, 0x2E)
    (e_shnum,) = struct.unpack_from(end + "H", data, 0x30)
    if e_shoff == 0 or e_shnum == 0:
        raise ValueError(f"{path}: no section headers")

    # Section header (ELF32, 40 bytes): name,type,flags,addr,offset,size,link,info,addralign,entsize
    def section(i):
        base = e_shoff + i * e_shentsize
        (sh_type, _flags, _addr, sh_offset, sh_size, sh_link, _info, _align, sh_entsize) = (
            struct.unpack_from(end + "4xIIIIIIIII", data, base)
        )
        return sh_type, sh_offset, sh_size, sh_link, sh_entsize

    SHT_SYMTAB = 2
    for i in range(e_shnum):
        sh_type, sh_offset, sh_size, sh_link, sh_entsize = section(i)
        if sh_type != SHT_SYMTAB:
            continue
        # The associated string table is section sh_link.
        _st, str_off, str_size, _l, _e = section(sh_link)
        strtab = data[str_off : str_off + str_size]
        entsize = sh_entsize or 16
        target = name.encode()
        for off in range(sh_offset, sh_offset + sh_size, entsize):
            # Symbol (ELF32, 16 bytes): name(u32), value(u32), size(u32), info(u8), other(u8), shndx(u16)
            st_name, st_value = struct.unpack_from(end + "II", data, off)
            end_i = strtab.find(b"\x00", st_name)
            sym = strtab[st_name:end_i]
            if sym == target:
                return st_value
    raise ValueError(f"{path}: symbol {name!r} not found in symbol table")


# --------------------------------------------------------------------------------------------------
# mdw reply parsing. OpenOCD `mdw <addr> <count>` prints lines like:
#   0x20000c40: 4c525443 00000001 00000d02 000014f7
#   0x20000c50: 00000cf6 00000340 fffff7c2 00000001
# The word values are already host-order (mdw prints the word, so no byte-swap). Returns a list of
# ints in address order.
# --------------------------------------------------------------------------------------------------
_HEXWORD = re.compile(r"\b[0-9a-fA-F]{8}\b")


def parse_mdw(text):
    words = []
    for line in text.splitlines():
        # Only take hex after the "0x....:" address prefix, so the address itself is not counted.
        idx = line.find(":")
        if idx < 0:
            continue
        # Guard: the part before ':' should look like an address (starts 0x / hex).
        addr_part = line[:idx].strip()
        if not (addr_part.startswith("0x") or re.fullmatch(r"[0-9a-fA-F]+", addr_part)):
            continue
        for tok in _HEXWORD.findall(line[idx + 1 :]):
            words.append(int(tok, 16))
    return words


# --------------------------------------------------------------------------------------------------
# Decode
# --------------------------------------------------------------------------------------------------
def _to_signed32(u):
    return u - 0x100000000 if u & 0x80000000 else u


def flags_from_word(word12):
    return (word12 >> FLAGS_SHIFT) & 0xFF


def decode_flags(flags_byte):
    return {
        "configured": bool(flags_byte & FLAG_CONFIGURED),
        "live": bool(flags_byte & FLAG_LIVE),
        "comms_loss": bool(flags_byte & FLAG_COMMS_LOSS),
        "mode_fault": bool(flags_byte & FLAG_MODE_FAULT),
        "loss": bool(flags_byte & FLAG_LOSS),
    }


def flags_str(flags_byte):
    """The primary trio the session watches (spec'd), plus the two fault bits when set."""
    d = decode_flags(flags_byte)
    parts = []
    parts.append("CONFIGURED" if d["configured"] else "unconfigured")
    parts.append("LIVE" if d["live"] else "not-live")
    if d["loss"]:
        parts.append("LOSS")
    if d["comms_loss"]:
        parts.append("comms_loss")
    if d["mode_fault"]:
        parts.append("mode_fault")
    return " ".join(parts)


class Sample:
    __slots__ = ("magic_ok", "pitch_mdeg", "flags", "roll_mdeg")

    def __init__(self, magic_ok, pitch_mdeg, flags, roll_mdeg=None):
        self.magic_ok = magic_ok
        self.pitch_mdeg = pitch_mdeg
        self.flags = flags
        self.roll_mdeg = roll_mdeg


def decode_sample(words):
    """Turn a word list from parse_mdw into a Sample. Raises ValueError if too short."""
    need = max(MAGIC_WORD, PITCH_WORD, FLAGS_WORD, ROLL_WORD or 0) + 1
    if len(words) < need:
        raise ValueError(f"short read: got {len(words)} words, need >= {need}")
    magic_ok = words[MAGIC_WORD] == CTRL_OBS_MAGIC
    pitch_mdeg = _to_signed32(words[PITCH_WORD])
    flags = flags_from_word(words[FLAGS_WORD])
    roll_mdeg = None
    if ROLL_WORD is not None:
        roll_mdeg = _to_signed32(words[ROLL_WORD])
    return Sample(magic_ok, pitch_mdeg, flags, roll_mdeg)


# --------------------------------------------------------------------------------------------------
# Strip-chart renderer: a PURE function (samples + dimensions -> list of strings), so it unit-tests
# with no terminal. Live mode redraws it with ANSI cursor control; it does not touch I/O itself.
#
#   samples: pitch in millidegrees, oldest..newest (the ~60 s / 10 Hz ring); may be empty.
#   width:   total character width of each line (axis label + axis + plot).
#   height:  chart rows (>= 3). Auto-scaled y-axis that always includes 0 (the baseline), a zero
#            line ('┼'/'─'), and degree labels at top / zero / bottom. Newest sample at the right.
# --------------------------------------------------------------------------------------------------
CHART_LABEL_W = 6  # chars reserved for a right-justified "-12.3"-style degree label
CHART_DEFAULT_SPAN_MDEG = 1000  # empty/flat fallback half-range: +/-1.0 deg around zero


def _nice_step(raw_mdeg):
    """Smallest 1/2/5 * 10^k step >= raw (in millidegrees); >= 1."""
    if raw_mdeg <= 0:
        return 1
    mag = 10 ** math.floor(math.log10(raw_mdeg))
    for m in (1, 2, 5, 10):
        if raw_mdeg <= m * mag:
            return int(m * mag)
    return int(10 * mag)


def nice_bounds(lo_mdeg, hi_mdeg):
    """Auto-scale bounds (millidegrees) that include 0, pad ~10%, and round out to a nice step."""
    lo = min(lo_mdeg, 0)
    hi = max(hi_mdeg, 0)
    if lo == 0 and hi == 0:
        return -CHART_DEFAULT_SPAN_MDEG, CHART_DEFAULT_SPAN_MDEG
    pad = max(int((hi - lo) * 0.1), 100)
    lo -= pad
    hi += pad
    step = _nice_step(max((hi - lo) // 4, 1))
    ymin = math.floor(lo / step) * step
    ymax = math.ceil(hi / step) * step
    if ymin == ymax:  # degenerate guard
        ymin, ymax = ymin - step, ymax + step
    return int(ymin), int(ymax)


def _value_to_row(v, ymin, ymax, height):
    if ymax == ymin:
        return height // 2
    frac = (ymax - v) / (ymax - ymin)
    return max(0, min(height - 1, round(frac * (height - 1))))


def _bucket(samples, plot_w):
    """Downsample samples into plot_w column means, newest at the right; short data left-padded."""
    n = len(samples)
    if n == 0:
        return [None] * plot_w
    if n <= plot_w:
        return [None] * (plot_w - n) + [float(s) for s in samples]
    cols = []
    for x in range(plot_w):
        a = x * n // plot_w
        b = (x + 1) * n // plot_w
        chunk = samples[a:b] or samples[a : a + 1]
        cols.append(sum(chunk) / len(chunk))
    return cols


def _fmt_deg_label(mdeg):
    return f"{mdeg / 1000.0:.1f}"


def render_chart(samples, width, height):
    """Pure strip-chart renderer -> list of `height` strings. See section header for the contract."""
    height = max(3, height)
    plot_w = max(1, width - CHART_LABEL_W - 1)  # label + axis glyph + plot
    lo = min(samples) if samples else 0
    hi = max(samples) if samples else 0
    ymin, ymax = nice_bounds(lo, hi)
    zero_row = _value_to_row(0, ymin, ymax, height)

    grid = [[" "] * plot_w for _ in range(height)]
    for x in range(plot_w):
        grid[zero_row][x] = "─"  # zero / baseline line

    cols = _bucket(list(samples), plot_w)
    prev_row = None
    for x, v in enumerate(cols):
        if v is None:
            prev_row = None
            continue
        row = _value_to_row(v, ymin, ymax, height)
        if prev_row is not None:  # connect consecutive points into a trace
            a, b = sorted((prev_row, row))
            for r in range(a, b + 1):
                if grid[r][x] in (" ", "─"):
                    grid[r][x] = "│"
        grid[row][x] = "●"
        prev_row = row

    lines = []
    for r in range(height):
        if r == 0:
            label = _fmt_deg_label(ymax)
        elif r == zero_row:
            label = _fmt_deg_label(0)
        elif r == height - 1:
            label = _fmt_deg_label(ymin)
        else:
            label = ""
        axis = "┼" if r == zero_row else "│"
        lines.append(f"{label:>{CHART_LABEL_W}}{axis}{''.join(grid[r])}")
    return lines


def chart_dims():
    """Terminal-derived (width, chart_height), re-checked per frame; sane for narrow/short terms."""
    size = shutil.get_terminal_size((80, 24))
    width = max(CHART_LABEL_W + 3, size.columns)
    height = max(6, min(16, size.lines - 3))
    return width, height


def _value_line(sample, stale_seconds):
    live = bool(sample.flags & FLAG_LIVE)
    stale = f"  [STALE {stale_seconds:0.1f}s: pitch frozen while LIVE]" if (live and stale_seconds > STALE_SECONDS) else ""
    magic = "" if sample.magic_ok else "  [MAGIC BAD -- block not live / wrong addr]"
    # Roll rides the same value line (the strip chart stays pitch-only; the roll axis has no
    # confirmed sign yet, so it is a readout, not a plotted trace). None only on a pre-roll image.
    roll = "" if sample.roll_mdeg is None else f"  roll {_fmt_pitch(sample.roll_mdeg)}"
    return f"pitch {_fmt_pitch(sample.pitch_mdeg)}{roll}   [{flags_str(sample.flags)}]{stale}{magic}"


def _emit_frame(lines, first):
    out = ["\033[2J" if first else ""]
    out.append("\033[H")  # cursor home
    for ln in lines:
        out.append(ln + "\033[K\n")  # clear to end of line
    out.append("\033[J")  # clear anything below
    sys.stdout.write("".join(out))
    sys.stdout.flush()


# --------------------------------------------------------------------------------------------------
# Evidence CSV writer. The recording is meant to be self-describing ground truth, NOT a bare number
# trace: it carries a schema header, a `label` column naming the pose each row was taken in, and
# `# ...` comment lines that let the CSV alone reconstruct the session (step windows, means, the
# spec-expected sign, verdicts, a final summary + provenance). It writes to any text file object, so
# it unit-tests against an in-memory buffer.
#
# Schema (v3):  # columns: unix_time,pitch_mdeg,roll_mdeg,flags,label
#   data row:   <unix_time:.3f>,<pitch_mdeg>,<roll_mdeg>,0x<flags>,<label>  (label empty in live mode;
#               roll_mdeg empty only on a pre-roll image)
# --------------------------------------------------------------------------------------------------
CSV_COLUMNS = "unix_time,pitch_mdeg,roll_mdeg,flags,label"


class EvidenceWriter:
    def __init__(self, fh, write_header=True):
        self.fh = fh
        self.markers = 0
        if write_header:
            fh.write(f"# columns: {CSV_COLUMNS}\n")

    @classmethod
    def open(cls, path):
        """Open `path` for append; emit the schema header only when starting a fresh/empty file."""
        is_new = (not os.path.exists(path)) or os.path.getsize(path) == 0
        fh = open(path, "a", buffering=1)
        return cls(fh, write_header=is_new)

    def close(self):
        self.fh.close()

    def row(self, unix_time, pitch_mdeg, roll_mdeg, flags, label=""):
        """One data row (schema v3). `roll_mdeg` is None only on a pre-roll image -> empty column."""
        roll = "" if roll_mdeg is None else f"{roll_mdeg}"
        self.fh.write(f"{unix_time:.3f},{pitch_mdeg},{roll},0x{flags:02x},{label}\n")

    def comment(self, text):
        self.fh.write(f"# {text}\n")

    def marker(self, unix_time, text=""):
        """Insert an ad-hoc `# marker` row; returns the label used (the text, else a running count)."""
        self.markers += 1
        label = text.strip() or str(self.markers)
        self.comment(f"marker {unix_time:.3f} {label}")
        return label

    def step_begin(self, pose_id, start, seconds):
        self.comment(f"step {pose_id} begin t={start:.3f} window={seconds:.1f}s")

    def step_end(self, pose_id, mean_mdeg, expected, verdict, n):
        mean = "n/a" if mean_mdeg is None else f"{mean_mdeg / 1000:+.2f}deg"
        self.comment(f"step {pose_id} end mean={mean} n={n} expected={expected} verdict={verdict}")

    def summary(self, rows, provenance):
        """rows: iterable of (pose_id, mean_mdeg|None, expected, verdict). provenance: (key, value) pairs."""
        self.comment("summary")
        for pose_id, mean_mdeg, expected, verdict in rows:
            mean = "n/a" if mean_mdeg is None else f"{mean_mdeg / 1000:+.2f}deg"
            self.comment(f"summary pose={pose_id} mean={mean} expected={expected} verdict={verdict}")
        for key, value in provenance:
            self.comment(f"summary {key}={value}")


def step_expected(step):
    """The spec-expected sign string for a checklist step (goes in the CSV step/summary lines)."""
    t = step["type"]
    if t == "baseline":
        return "baseline~0"
    if t == "sign":
        return step["expect"]  # "negative" / "positive" (attitude.md Pitch)
    # record/roll: roll is now published (CTRL_OBS roll_milli), but the ZYX roll axis/sign is still
    # an open question (attitude.md "Open questions"), so there is no expected sign to assert yet.
    return "unconfirmed(roll-sign-open)"


# --------------------------------------------------------------------------------------------------
# OpenOCD TCL RPC client. Protocol: send <command> + 0x1a, read reply up to 0x1a.
# --------------------------------------------------------------------------------------------------
CMD_SEP = b"\x1a"


class TclClient:
    def __init__(self, host, port, timeout=3.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.close()
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.settimeout(self.timeout)
        self.sock = s

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def command(self, cmd):
        if self.sock is None:
            raise ConnectionError("not connected")
        self.sock.sendall(cmd.encode() + CMD_SEP)
        buf = bytearray()
        while True:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("connection closed by OpenOCD")
            buf.extend(chunk)
            i = buf.find(CMD_SEP)
            if i >= 0:
                return buf[:i].decode(errors="replace")

    def read_words(self, addr, count):
        text = self.command(f"mdw 0x{addr:08x} {count}")
        return parse_mdw(text)

    def read_sample(self, addr):
        return decode_sample(self.read_words(addr, READ_WORDS))


# --------------------------------------------------------------------------------------------------
# Stream mode: one updating line at ~SAMPLE_HZ, with a staleness watch and graceful reconnect.
# --------------------------------------------------------------------------------------------------
def _fmt_pitch(mdeg):
    return f"{mdeg / 1000.0:+7.2f} deg"


def _poll_stdin_line():
    """Non-blocking: return a typed line (without newline) if one is ready on stdin, else None."""
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0)
    except (OSError, ValueError):
        return None
    if r:
        line = sys.stdin.readline()
        if line == "":  # EOF
            return None
        return line.rstrip("\n")
    return None


def stream(client, addr, record_path=None, use_graph=True):
    # Graceful degrade: no graph when asked off, or when stdout is not a tty (so `--record`
    # piping keeps producing the single-line stream unchanged).
    graph = use_graph and sys.stdout.isatty()
    # In-session markers: type a word (or just Enter) + Enter to drop a `# marker` row in the CSV.
    # A newline-triggered line read (select-polled, no raw termios) -- only when recording to a file
    # and stdin is an interactive tty.
    markers_on = record_path is not None and sys.stdin.isatty()
    period = 1.0 / SAMPLE_HZ
    writer = EvidenceWriter.open(record_path) if record_path else None
    ring = deque(maxlen=int(60 * SAMPLE_HZ))  # ~60 s window at 10 Hz
    last_pitch = None
    last_change = time.time()
    last_marker = ""
    connected = False
    first_frame = True
    if markers_on:
        print("(recording: type a word + Enter to drop a marker in the CSV)")
    try:
        while True:
            t0 = time.time()
            try:
                if not connected:
                    client.connect()
                    connected = True
                    if not graph:
                        sys.stdout.write("\r" + " " * 78 + "\rconnected\n")
                s = client.read_sample(addr)
            except (OSError, ConnectionError, ValueError) as e:
                connected = False
                client.close()
                sys.stdout.write(f"\r[disconnected: {e}] reconnecting...        ")
                sys.stdout.flush()
                time.sleep(0.5)
                continue

            now = time.time()
            if last_pitch is None or s.pitch_mdeg != last_pitch:
                last_change = now
                last_pitch = s.pitch_mdeg
            stale = now - last_change
            ring.append(s.pitch_mdeg)

            if writer is not None:
                writer.row(now, s.pitch_mdeg, s.roll_mdeg, s.flags)

            if markers_on:
                typed = _poll_stdin_line()
                if typed is not None:
                    lbl = writer.marker(time.time(), typed)
                    last_marker = f"marker #{writer.markers}: {lbl}"
                    if not graph:
                        sys.stdout.write(f"\n{last_marker}\n")

            if graph:
                width, height = chart_dims()
                frame = render_chart(ring, width, height)
                frame.append("")
                frame.append(_value_line(s, stale))
                if last_marker:
                    frame.append(f"[{last_marker}]")
                _emit_frame(frame, first_frame)
                first_frame = False
            else:
                sys.stdout.write("\r" + _value_line(s, stale).ljust(78))
                sys.stdout.flush()

            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        sys.stdout.write("\n")
    finally:
        if writer is not None:
            writer.close()
        client.close()


# --------------------------------------------------------------------------------------------------
# Checklist mode: guided pitch sign validation.
#
# Sign source (specs/attitude.md, "Outputs: pitch"):
#   pitch_deg = -(asin(...) * rad2deg)   -- "negative scale: lean forward -> negative pitch".
#   Pitch sign is PRESERVED BIT-EXACT (a recovered convention "the balance PID expects"), so the
#   forward/back sign is a firm PASS/CHECK criterion, not an open question.
#
#   "Level" is recorded as the resting baseline (the bench shows a ~-2 deg mounting/trim offset,
#   e.g. round17 rested at -2.11 deg), and the tilt steps are judged as the SIGN OF THE DELTA from
#   that baseline -- honest against the mounting offset.
#
# ROLL is now PUBLISHED (CTRL_OBS roll_milli), but its ZYX axis/sign is still an OPEN QUESTION
#   (attitude.md "Open questions": confirm the ZYX roll axis + sign the control law expects). So the
#   roll steps SAMPLE the live roll (like the pitch steps) and REPORT the observed sign of the delta
#   from the level baseline, but hold the verdict at RECORDED (not PASS/CHECK): the human session
#   confirms the sign against the physical motion, resolving the spec's open question.
# --------------------------------------------------------------------------------------------------
CHECKLIST = [
    {
        "name": "level",
        "id": "level",
        "type": "baseline",
        "prompt": "Hold the board LEVEL (flat) and still.",
        "cite": "attitude.md: baseline; expect ~0 (a small mounting/trim offset is normal).",
    },
    {
        "name": "lean-forward",
        "id": "lean_forward",
        "type": "sign",
        "expect": "negative",
        "prompt": "Tilt the board's NOSE/FRONT DOWN (lean forward) and hold.",
        "cite": 'attitude.md Pitch: "lean forward -> negative pitch" (bit-exact, CONFIRMED).',
    },
    {
        "name": "lean-back",
        "id": "lean_back",
        "type": "sign",
        "expect": "positive",
        "prompt": "Tilt the board's NOSE/FRONT UP (lean back) and hold.",
        "cite": "attitude.md Pitch: negative scale -> nose-up is positive (bit-exact, CONFIRMED).",
    },
    {
        "name": "roll-right",
        "id": "roll_right",
        "type": "record",
        "prompt": "Roll the board to the RIGHT (tilt right side down) and hold.",
        "cite": (
            "attitude.md Open questions: ZYX roll axis/sign is UNCONFIRMED; roll IS published now "
            "(CTRL_OBS roll_milli) -- the tool samples it and reports the observed physical "
            "direction/sign, verdict RECORDED (the human session confirms the sign)."
        ),
    },
    {
        "name": "roll-left",
        "id": "roll_left",
        "type": "record",
        "prompt": "Roll the board to the LEFT (tilt left side down) and hold.",
        "cite": (
            "attitude.md Open questions: ZYX roll axis/sign is UNCONFIRMED; roll IS published now "
            "(CTRL_OBS roll_milli) -- the tool samples it and reports the observed direction/sign, "
            "verdict RECORDED (the human session confirms the sign)."
        ),
    },
]


def checklist_verdict(step, baseline_mdeg, mean_mdeg, threshold_mdeg=CHECKLIST_DELTA_MDEG):
    """Pure verdict logic (unit-tested). Returns (verdict, detail).

    verdict is one of: INFO (baseline), PASS/CHECK (sign steps), RECORDED (open-question/roll).
    """
    if step["type"] == "baseline":
        note = "baseline recorded"
        if abs(mean_mdeg) > 10000:
            note += f" (WARN: |{mean_mdeg / 1000:.2f} deg| large -- check mounting/trim)"
        return ("INFO", note)
    if step["type"] == "sign":
        delta = mean_mdeg - baseline_mdeg
        d = f"delta {delta / 1000:+.2f} deg from baseline"
        if step["expect"] == "negative":
            return (("PASS", d) if delta <= -threshold_mdeg else ("CHECK", d + " (expected < 0)"))
        if step["expect"] == "positive":
            return (("PASS", d) if delta >= threshold_mdeg else ("CHECK", d + " (expected > 0)"))
    if step["type"] == "record":
        # Roll IS published now (CTRL_OBS roll_milli), but the ZYX roll axis/sign is still an open
        # question (attitude.md "Open questions"), so the tool REPORTS the observed sign and holds
        # the verdict at RECORDED for the human session to confirm -- it never PASS/CHECKs it.
        if mean_mdeg is None:  # pre-roll image: no field to sample
            return ("RECORDED", "roll not published on this image -- record observed direction; verdict deferred")
        delta = mean_mdeg - baseline_mdeg
        sign = "positive" if delta > 0 else "negative" if delta < 0 else "zero"
        d = f"observed roll delta {delta / 1000:+.2f} deg from baseline (sign {sign})"
        return ("RECORDED", d + "; ZYX roll sign open -- verdict deferred to the human session")
    raise ValueError(f"unknown step type {step['type']!r}")


def _sample_mean(client, addr, seconds, ring=None, graph=False, writer=None, label=""):
    """Sample for `seconds`, return (mean_pitch_mdeg, mean_roll_mdeg, n, last_flags). Live, no halt.

    Each sample is recorded (labeled with `label`, the pose id) when `writer` is given -- so the CSV
    holds the raw per-sample ground truth (pitch AND roll), not just the step's mean. When `graph`,
    redraws the same strip chart each read so the operator SEES the pose settle. `mean_roll_mdeg` is
    None on a pre-roll image (no roll field).
    """
    end = time.time() + seconds
    total_pitch = 0
    total_roll = 0
    have_roll = False
    n = 0
    last_flags = 0
    while time.time() < end:
        try:
            s = client.read_sample(addr)
        except (OSError, ConnectionError, ValueError):
            time.sleep(0.1)
            continue
        total_pitch += s.pitch_mdeg
        if s.roll_mdeg is not None:
            total_roll += s.roll_mdeg
            have_roll = True
        last_flags = s.flags
        n += 1
        if writer is not None:
            writer.row(time.time(), s.pitch_mdeg, s.roll_mdeg, s.flags, label)
        if ring is not None:
            ring.append(s.pitch_mdeg)
        if graph and ring is not None:
            width, height = chart_dims()
            frame = render_chart(ring, width, height)
            frame.append("")
            frame.append(_value_line(s, 0.0))
            _emit_frame(frame, False)
        time.sleep(1.0 / SAMPLE_HZ)
    mean_pitch = total_pitch // n if n else 0
    mean_roll = (total_roll // n) if (n and have_roll) else None
    return mean_pitch, mean_roll, n, last_flags


def checklist(client, addr, record_path=None, use_graph=True, elf_path=None):
    graph = use_graph and sys.stdout.isatty()
    ring = deque(maxlen=int(60 * SAMPLE_HZ))
    writer = EvidenceWriter.open(record_path) if record_path else None
    try:
        client.connect()
    except OSError as e:
        print(f"connect failed: {e}", file=sys.stderr)
        if writer is not None:
            writer.close()
        return 1
    print("IMU hand-tilt attitude sign checklist (specs/silicon-queue.md section 2).")
    print("Pitch is a firm PASS/CHECK. Roll is now published (CTRL_OBS roll_milli), but its ZYX sign")
    print("is an open question (attitude.md) -- roll poses REPORT the observed sign, verdict RECORDED.\n")
    baseline = 0  # pitch baseline (level pose)
    baseline_roll = 0  # roll baseline (level pose); roll deltas judge against it
    results = []  # (pose_id, mean_mdeg|None, expected, verdict, detail)
    try:
        for step in CHECKLIST:
            expected = step_expected(step)
            record = step["type"] == "record"
            print(f"--- {step['name']} ---")
            print(f"    {step['prompt']}")
            print(f"    ({step['cite']})")
            input(f"    Hold it, then press Enter to sample ~{CHECKLIST_SAMPLE_S:.0f}s... ")
            if writer is not None:
                writer.step_begin(step["id"], time.time(), CHECKLIST_SAMPLE_S)
            mean_p, mean_r, n, flags = _sample_mean(
                client, addr, CHECKLIST_SAMPLE_S, ring, graph, writer, step["id"]
            )
            if n == 0:
                print("    ERROR: no samples (target/connection?)\n")
                if writer is not None:
                    writer.step_end(step["id"], None, expected, "ERROR", 0)
                results.append((step["id"], None, expected, "ERROR", "no samples"))
                continue
            if step["type"] == "baseline":
                baseline = mean_p
                baseline_roll = mean_r if mean_r is not None else 0
            if not (flags & FLAG_LIVE):
                print(f"    WARN: imu_live=0 during sample (flags {flags_str(flags)})")
            # Roll poses are judged on the roll axis (observed-sign, RECORDED); pitch/baseline poses
            # on pitch. The step's recorded mean is its own axis's mean.
            if record:
                verdict, detail = checklist_verdict(step, baseline_roll, mean_r)
                step_mean = mean_r
                axis_label, shown = "roll", (mean_r if mean_r is not None else 0)
            else:
                verdict, detail = checklist_verdict(step, baseline, mean_p)
                step_mean = mean_p
                axis_label, shown = "pitch", mean_p
            if writer is not None:
                writer.step_end(step["id"], step_mean, expected, verdict, n)
            print(f"    mean {axis_label} {shown / 1000:+.2f} deg  ({n} samples, [{flags_str(flags)}])")
            print(f"    {verdict}: {detail}\n")
            results.append((step["id"], step_mean, expected, verdict, detail))
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        if writer is not None:
            provenance = [
                ("elf", elf_path or "?"),
                ("ctrl_obs", f"0x{addr:08x}"),
                ("tool", TOOL_VERSION),
                ("commit", tool_commit()),
            ]
            writer.summary([(pid, m, exp, v) for pid, m, exp, v, _ in results], provenance)
            writer.close()
        client.close()
    print("=== summary ===")
    for pid, _mean, _expected, verdict, detail in results:
        print(f"  {verdict:8s} {pid:14s} {detail}")
    return 0


# --------------------------------------------------------------------------------------------------
# Self-test: exercise the parsers against canned strings (no hardware).
# --------------------------------------------------------------------------------------------------
_SELFTEST_MDW = (
    # HEALTHY sample: magic, boot=1, tick, dispatch, control, input, pitch=-2110mdeg, cyclic_age,
    # ..., word12=00030000 (flags 0x03 = CONFIGURED+LIVE), ... link/ISR counters ..., word18 =
    # roll = fffffa24 = -1500 mdeg (the appended roll_milli field).
    "0x20000c3c: 4c525443 00000001 00000d02 000014f7\n"
    "0x20000c4c: 00000cf6 00000340 fffff7c2 00000001\n"
    "0x20000c5c: 00000000 00000000 00000000 00000000\n"
    "0x20000c6c: 00030000 00000000 00000000 00000000\n"
    "0x20000c7c: 00000000 00000000 fffffa24\n"
)


def selftest():
    ok = True

    def check(label, cond):
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {label}")
        ok = ok and cond

    print("parse_mdw / decode:")
    words = parse_mdw(_SELFTEST_MDW)
    check(f"19 words parsed (got {len(words)})", len(words) == 19)
    check("word0 == magic", words[0] == CTRL_OBS_MAGIC)
    s = decode_sample(words)
    check("magic_ok", s.magic_ok)
    check(f"pitch == -2110 mdeg (got {s.pitch_mdeg})", s.pitch_mdeg == -2110)
    check(f"flags == 0x03 (got 0x{s.flags:02x})", s.flags == 0x03)
    check(f"roll == -1500 mdeg (got {s.roll_mdeg})", s.roll_mdeg == -1500)

    print("flags decode:")
    check("0x03 -> CONFIGURED+LIVE, no LOSS", flags_str(0x03) == "CONFIGURED LIVE")
    check("0x11 -> CONFIGURED+LOSS, not-live", flags_str(0x11) == "CONFIGURED not-live LOSS")
    d = decode_flags(0x11)
    check("0x11 bits", d["configured"] and d["loss"] and not d["live"])

    print("signed pitch:")
    check("0xfffff7a6 -> -2138", _to_signed32(0xFFFFF7A6) == -2138)
    check("0x00000001 -> 1", _to_signed32(1) == 1)

    print("mdw address token not miscounted as data:")
    check("single line, 4 words", len(parse_mdw("0x20000c3c: 00000001 00000002 00000003 00000004")) == 4)

    print("checklist sign logic:")
    lf = CHECKLIST[1]  # lean-forward, expect negative
    lb = CHECKLIST[2]  # lean-back, expect positive
    base = -2110
    check("lean-forward strong-neg -> PASS", checklist_verdict(lf, base, base - 8000)[0] == "PASS")
    check("lean-forward wrong-sign -> CHECK", checklist_verdict(lf, base, base + 8000)[0] == "CHECK")
    check("lean-forward tiny-move -> CHECK", checklist_verdict(lf, base, base - 1000)[0] == "CHECK")
    check("lean-back strong-pos -> PASS", checklist_verdict(lb, base, base + 8000)[0] == "PASS")
    check("lean-back wrong-sign -> CHECK", checklist_verdict(lb, base, base - 8000)[0] == "CHECK")
    check("baseline -> INFO", checklist_verdict(CHECKLIST[0], 0, base)[0] == "INFO")
    # roll poses: RECORDED with an observed-sign report off the sampled roll (mean vs baseline);
    # a pre-roll image (mean None) still returns RECORDED. Verdict is never PASS/CHECK (sign open).
    rr = CHECKLIST[3]  # roll-right, type record
    check("roll sampled -> RECORDED", checklist_verdict(rr, 0, 1500)[0] == "RECORDED")
    check("roll observed-sign reported", "positive" in checklist_verdict(rr, 0, 1500)[1])
    check("roll None (pre-roll image) -> RECORDED", checklist_verdict(rr, 0, None)[0] == "RECORDED")
    check("step_expected sign", step_expected(lf) == "negative" and step_expected(lb) == "positive")
    check("step_expected roll", step_expected(rr) == "unconfirmed(roll-sign-open)")

    print("evidence CSV writer:")
    buf = io.StringIO()
    w = EvidenceWriter(buf)
    check("header emitted first", buf.getvalue().startswith(f"# columns: {CSV_COLUMNS}\n"))
    w.row(1000.0, -2110, -1500, 0x03, "level")
    w.row(1000.1, 0, 5, 0x03)  # plain live: empty label
    w.row(1000.2, 0, None, 0x03)  # pre-roll image: empty roll column
    check("labeled row", "1000.000,-2110,-1500,0x03,level\n" in buf.getvalue())
    check("empty-label row", "1000.100,0,5,0x03,\n" in buf.getvalue())
    check("empty-roll row", "1000.200,0,,0x03,\n" in buf.getvalue())
    lbl = w.marker(1234.5, "forward")
    check("marker text", lbl == "forward" and "# marker 1234.500 forward\n" in buf.getvalue())
    lbl2 = w.marker(1235.0, "")
    check("marker empty -> count", lbl2 == "2" and "# marker 1235.000 2\n" in buf.getvalue())
    w.step_begin("lean_forward", 2000.0, 2.0)
    w.step_end("lean_forward", -10110, "negative", "PASS", 20)
    check("step begin", "# step lean_forward begin t=2000.000 window=2.0s\n" in buf.getvalue())
    check(
        "step end",
        "# step lean_forward end mean=-10.11deg n=20 expected=negative verdict=PASS\n" in buf.getvalue(),
    )
    w.summary(
        [("level", -2110, "baseline~0", "INFO"), ("roll_right", -1500, "unconfirmed(roll-sign-open)", "RECORDED")],
        [("elf", "target/.../firmware"), ("ctrl_obs", "0x20000c40")],
    )
    sv = buf.getvalue()
    check("summary header", "# summary\n" in sv)
    check("summary pose line", "# summary pose=level mean=-2.11deg expected=baseline~0 verdict=INFO\n" in sv)
    check("summary roll pose", "# summary pose=roll_right mean=-1.50deg expected=unconfirmed(roll-sign-open) verdict=RECORDED\n" in sv)
    check("summary provenance", "# summary ctrl_obs=0x20000c40\n" in sv)

    print("chart renderer:")
    empty = render_chart([], 60, 8)
    check("empty ring -> 8 rows", len(empty) == 8)
    check("empty ring has a zero line", any("┼" in r for r in empty))
    flat = render_chart([-2110] * 40, 60, 8)
    check("flat line plots a point", any("●" in r for r in flat))
    ramp = render_chart(list(range(-5000, 5001, 100)), 60, 12)
    check("ramp -> 12 rows", len(ramp) == 12)
    ymin, ymax = nice_bounds(-5000, 5000)
    check(f"auto-scale brackets data ({ymin}..{ymax} mdeg)", ymin <= -5000 and ymax >= 5000)
    check("auto-scale includes zero", ymin <= 0 <= ymax)
    narrow = render_chart([0, 1000, -1000], 10, 6)
    check("narrow width still renders 6 rows", len(narrow) == 6 and all(len(r) > 0 for r in narrow))

    print("\nsample rendered frame (ramp -5.0 -> +5.0 deg, width 48, height 12):")
    for r in render_chart(list(range(-5000, 5001, 100)), 48, 12):
        print("  " + r)

    print("\nsample labeled evidence CSV (checklist excerpt):")
    demo = io.StringIO()
    dw = EvidenceWriter(demo)
    dw.step_begin("level", 1721600000.000, 2.0)
    dw.row(1721600000.10, -2100, -1400, 0x03, "level")
    dw.row(1721600000.20, -2118, -1520, 0x03, "level")
    dw.step_end("level", -2109, "baseline~0", "INFO", 2)
    dw.step_begin("lean_forward", 1721600004.000, 2.0)
    dw.row(1721600004.10, -9800, -1490, 0x03, "lean_forward")
    dw.row(1721600004.20, -10420, -1510, 0x03, "lean_forward")
    dw.step_end("lean_forward", -10110, "negative", "PASS", 2)
    dw.summary(
        [("level", -2109, "baseline~0", "INFO"), ("lean_forward", -10110, "negative", "PASS")],
        [("elf", "target/thumbv7m-none-eabi/release/firmware"), ("ctrl_obs", "0x20000c40"),
         ("tool", TOOL_VERSION), ("commit", "abc1234")],
    )
    for line in demo.getvalue().splitlines():
        print("  " + line)

    print("\nSELFTEST", "OK" if ok else "FAILED")
    return 0 if ok else 1


# --------------------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------------------
DEFAULT_ELF = "target/thumbv7m-none-eabi/release/firmware"


def build_parser():
    p = argparse.ArgumentParser(
        description="Live firmware pitch (CTRL_OBS) readout for the hand-tilt IMU sign validation. "
        "Pitch is PASS/CHECK; roll is now published (CTRL_OBS roll_milli) but its ZYX sign is an "
        "open question, so roll poses report the observed sign (verdict RECORDED). Reads over "
        "OpenOCD TCL RPC on the RUNNING target (no halt). Tunnel the Pi: ssh -L "
        "6666:localhost:6666 pi@192.168.0.248",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--elf", default=DEFAULT_ELF, help=f"ELF to resolve CTRL_OBS from (default {DEFAULT_ELF})")
    p.add_argument("--addr", help="CTRL_OBS address override, e.g. 0x20000c40 (skips ELF lookup)")
    p.add_argument("--host", default="localhost", help="OpenOCD TCL RPC host (default localhost)")
    p.add_argument("--port", type=int, default=6666, help="OpenOCD TCL RPC port (default 6666)")
    p.add_argument("--record", metavar="FILE", help="append timestamped CSV (unix_time,pitch_mdeg,roll_mdeg,flags,label)")
    p.add_argument("--checklist", action="store_true", help="guided pitch sign checklist")
    p.add_argument("--no-graph", action="store_true", help="disable the live strip chart (single-line mode)")
    p.add_argument("--selftest", action="store_true", help="run parsers + renderer against canned data; no hardware")
    return p


def resolve_addr(args):
    if args.addr:
        return int(args.addr, 0)
    if not os.path.exists(args.elf):
        print(
            f"error: ELF {args.elf!r} not found. Build the firmware or pass --elf / --addr.",
            file=sys.stderr,
        )
        sys.exit(2)
    try:
        addr = elf_resolve_symbol(args.elf, "CTRL_OBS")
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(2)
    print(f"CTRL_OBS @ 0x{addr:08x} (from {args.elf})")
    return addr


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest()
    addr = resolve_addr(args)
    client = TclClient(args.host, args.port)
    use_graph = not args.no_graph
    elf_path = f"addr:0x{addr:08x}" if args.addr else args.elf
    if args.checklist:
        return checklist(client, addr, args.record, use_graph, elf_path)
    stream(client, addr, args.record, use_graph)
    return 0


if __name__ == "__main__":
    sys.exit(main())
