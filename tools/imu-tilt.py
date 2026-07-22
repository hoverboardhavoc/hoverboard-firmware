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

ROLL IS NOT PUBLISHED YET.  CTRL_OBS carries pitch_milli only -- there is no roll field. The roll
  half of the sign validation (attitude.md "Open questions": confirm the ZYX roll axis + sign)
  needs a follow-up firmware field, queued post-round-18. This tool is written so adding roll is a
  two-line change: set ROLL_WORD to its word index and add its column (see the ROLL_WORD marker).

CONNECTION (OpenOCD TCL RPC)
  The bench runs OpenOCD on the Pi (pi@192.168.0.248). Its TCL RPC listens on localhost:6666. This
  tool speaks only TCP; tunnel the port from your workstation:
      ssh -L 6666:localhost:6666 pi@192.168.0.248
  then run against localhost:6666 (the default). Or run this tool on the Pi directly.

USAGE
  # live streaming readout (default):
  tools/imu-tilt.py --elf target/thumbv7m-none-eabi/release/firmware
  # append evidence CSV (unix_time,pitch_mdeg,flags):
  tools/imu-tilt.py --record specs/bench-evidence/<date>/imu-tilt.csv
  # guided sign checklist (prompts orientations, samples, prints PASS/CHECK):
  tools/imu-tilt.py --checklist
  # parser self-test, no hardware:
  tools/imu-tilt.py --selftest

python3 stdlib only.
"""

import argparse
import os
import re
import socket
import struct
import sys
import time

# --------------------------------------------------------------------------------------------------
# CTRL_OBS layout (word indices into `mdw <CTRL_OBS> N`). Derived from crates/firmware/src/main.rs
# `struct CtrlObs` (#[repr(C)]); cross-checked vs specs/bench-evidence/2026-07-22/round17/.
# --------------------------------------------------------------------------------------------------
CTRL_OBS_MAGIC = 0x4C525443  # "CTRL" little-endian, word 0
MAGIC_WORD = 0
PITCH_WORD = 6  # pitch_milli: i32 at byte offset 24
FLAGS_WORD = 12  # packed word at byte offset 48; flags is byte 2
FLAGS_SHIFT = 16  # flags = (word12 >> FLAGS_SHIFT) & 0xFF
READ_WORDS = 14  # words to read per sample (through word 12 + margin; matches the round17 method)

# ROLL_WORD: roll is NOT published in CTRL_OBS yet (pitch only). When the follow-up firmware field
# lands (queued post-round-18), set this to its word index and uncomment its use in decode_sample()
# / the stream line. That plus one column is the whole change.
ROLL_WORD = None

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
    need = max(MAGIC_WORD, PITCH_WORD, FLAGS_WORD) + 1
    if len(words) < need:
        raise ValueError(f"short read: got {len(words)} words, need >= {need}")
    magic_ok = words[MAGIC_WORD] == CTRL_OBS_MAGIC
    pitch_mdeg = _to_signed32(words[PITCH_WORD])
    flags = flags_from_word(words[FLAGS_WORD])
    roll_mdeg = None
    if ROLL_WORD is not None:  # roll follow-up field: two-line enable
        roll_mdeg = _to_signed32(words[ROLL_WORD])
    return Sample(magic_ok, pitch_mdeg, flags, roll_mdeg)


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


def stream(client, addr, record_path=None):
    period = 1.0 / SAMPLE_HZ
    rec = open(record_path, "a", buffering=1) if record_path else None
    last_pitch = None
    last_change = time.time()
    connected = False
    try:
        while True:
            t0 = time.time()
            try:
                if not connected:
                    client.connect()
                    connected = True
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

            live = bool(s.flags & FLAG_LIVE)
            stale_flag = ""
            if live and stale > STALE_SECONDS:
                stale_flag = f"  [STALE {stale:0.1f}s: pitch frozen while LIVE]"
            magic_flag = "" if s.magic_ok else "  [MAGIC BAD -- block not live / wrong addr]"

            line = f"pitch {_fmt_pitch(s.pitch_mdeg)}   [{flags_str(s.flags)}]{stale_flag}{magic_flag}"
            sys.stdout.write("\r" + line.ljust(78))
            sys.stdout.flush()

            if rec is not None:
                rec.write(f"{now:.3f},{s.pitch_mdeg},0x{s.flags:02x}\n")

            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        sys.stdout.write("\n")
    finally:
        if rec is not None:
            rec.close()
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
# ROLL is an OPEN QUESTION (attitude.md "Open questions": confirm the ZYX roll axis + sign the
#   control law expects) AND is not published in CTRL_OBS. So the roll steps only ask the operator
#   to record the observed sign; they render no PASS/CHECK and sample no data (there is no field).
#   They exist so the full validation matrix is visible and the roll gap is explicit.
# --------------------------------------------------------------------------------------------------
CHECKLIST = [
    {
        "name": "level",
        "type": "baseline",
        "prompt": "Hold the board LEVEL (flat) and still.",
        "cite": "attitude.md: baseline; expect ~0 (a small mounting/trim offset is normal).",
    },
    {
        "name": "lean-forward",
        "type": "sign",
        "expect": "negative",
        "prompt": "Tilt the board's NOSE/FRONT DOWN (lean forward) and hold.",
        "cite": 'attitude.md Pitch: "lean forward -> negative pitch" (bit-exact, CONFIRMED).',
    },
    {
        "name": "lean-back",
        "type": "sign",
        "expect": "positive",
        "prompt": "Tilt the board's NOSE/FRONT UP (lean back) and hold.",
        "cite": "attitude.md Pitch: negative scale -> nose-up is positive (bit-exact, CONFIRMED).",
    },
    {
        "name": "roll-right",
        "type": "record",
        "prompt": "Roll the board to the RIGHT (tilt right side down) and hold.",
        "cite": (
            "attitude.md Open questions: ZYX roll axis/sign is UNCONFIRMED, and roll is NOT "
            "published in CTRL_OBS -- record the observed physical direction; verdict deferred to "
            "the roll follow-up field (post-round-18)."
        ),
    },
    {
        "name": "roll-left",
        "type": "record",
        "prompt": "Roll the board to the LEFT (tilt left side down) and hold.",
        "cite": (
            "attitude.md Open questions: ZYX roll axis/sign is UNCONFIRMED, and roll is NOT "
            "published in CTRL_OBS -- record the observed direction; verdict deferred."
        ),
    },
]


def checklist_verdict(step, baseline_mdeg, mean_mdeg, threshold_mdeg=CHECKLIST_DELTA_MDEG):
    """Pure verdict logic (unit-tested). Returns (verdict, detail).

    verdict is one of: INFO (baseline), PASS/CHECK (sign steps), RECORD (open-question/roll).
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
        return ("RECORD", "roll not published -- record observed direction; verdict deferred")
    raise ValueError(f"unknown step type {step['type']!r}")


def _sample_mean_pitch(client, addr, seconds):
    """Sample pitch for `seconds`, return (mean_mdeg, n, last_flags). Live reads, no halt."""
    end = time.time() + seconds
    total = 0
    n = 0
    last_flags = 0
    while time.time() < end:
        try:
            s = client.read_sample(addr)
        except (OSError, ConnectionError, ValueError):
            time.sleep(0.1)
            continue
        total += s.pitch_mdeg
        last_flags = s.flags
        n += 1
        time.sleep(1.0 / SAMPLE_HZ)
    mean = total // n if n else 0
    return mean, n, last_flags


def checklist(client, addr, record_path=None):
    rec = open(record_path, "a", buffering=1) if record_path else None
    try:
        client.connect()
    except OSError as e:
        print(f"connect failed: {e}", file=sys.stderr)
        return 1
    print("IMU hand-tilt PITCH sign checklist (specs/silicon-queue.md section 2).")
    print("Pitch only -- roll is not published in CTRL_OBS yet (follow-up field, post-round-18).\n")
    baseline = 0
    results = []
    try:
        for step in CHECKLIST:
            print(f"--- {step['name']} ---")
            print(f"    {step['prompt']}")
            print(f"    ({step['cite']})")
            if step["type"] == "record":
                input("    Position the board, then press Enter to note it (no data sampled)... ")
                verdict, detail = checklist_verdict(step, baseline, 0)
                print(f"    {verdict}: {detail}\n")
                results.append((step["name"], verdict, detail))
                continue
            input(f"    Hold it, then press Enter to sample ~{CHECKLIST_SAMPLE_S:.0f}s... ")
            mean, n, flags = _sample_mean_pitch(client, addr, CHECKLIST_SAMPLE_S)
            if n == 0:
                print("    ERROR: no samples (target/connection?)\n")
                results.append((step["name"], "ERROR", "no samples"))
                continue
            if step["type"] == "baseline":
                baseline = mean
            if not (flags & FLAG_LIVE):
                print(f"    WARN: imu_live=0 during sample (flags {flags_str(flags)})")
            verdict, detail = checklist_verdict(step, baseline, mean)
            print(f"    mean pitch {mean / 1000:+.2f} deg  ({n} samples, [{flags_str(flags)}])")
            print(f"    {verdict}: {detail}\n")
            results.append((step["name"], verdict, detail))
            if rec is not None:
                rec.write(f"{time.time():.3f},{mean},0x{flags:02x},{step['name']},{verdict}\n")
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        if rec is not None:
            rec.close()
        client.close()
    print("=== summary ===")
    for name, verdict, detail in results:
        print(f"  {verdict:6s} {name:14s} {detail}")
    return 0


# --------------------------------------------------------------------------------------------------
# Self-test: exercise the parsers against canned strings (no hardware).
# --------------------------------------------------------------------------------------------------
_SELFTEST_MDW = (
    # round17 PATH 1 (HEALTHY): magic, boot=1, tick, dispatch, control, input, pitch=-2110mdeg,
    # cyclic_age, ... , word12=00030000 (flags 0x03 = CONFIGURED+LIVE).
    "0x20000c3c: 4c525443 00000001 00000d02 000014f7\n"
    "0x20000c4c: 00000cf6 00000340 fffff7c2 00000001\n"
    "0x20000c5c: 00000000 00000000 00000000 00000000\n"
    "0x20000c6c: 00030000 00000000\n"
)


def selftest():
    ok = True

    def check(label, cond):
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {label}")
        ok = ok and cond

    print("parse_mdw / decode:")
    words = parse_mdw(_SELFTEST_MDW)
    check(f"14 words parsed (got {len(words)})", len(words) == 14)
    check("word0 == magic", words[0] == CTRL_OBS_MAGIC)
    s = decode_sample(words)
    check("magic_ok", s.magic_ok)
    check(f"pitch == -2110 mdeg (got {s.pitch_mdeg})", s.pitch_mdeg == -2110)
    check(f"flags == 0x03 (got 0x{s.flags:02x})", s.flags == 0x03)

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
    check("roll -> RECORD", checklist_verdict(CHECKLIST[3], base, 0)[0] == "RECORD")

    print("\nSELFTEST", "OK" if ok else "FAILED")
    return 0 if ok else 1


# --------------------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------------------
DEFAULT_ELF = "target/thumbv7m-none-eabi/release/firmware"


def build_parser():
    p = argparse.ArgumentParser(
        description="Live firmware pitch (CTRL_OBS) readout for the hand-tilt IMU sign validation. "
        "ROLL IS NOT PUBLISHED YET (pitch only); the roll half needs the follow-up firmware field "
        "queued post-round-18. Reads over OpenOCD TCL RPC on the RUNNING target (no halt). Tunnel "
        "the Pi: ssh -L 6666:localhost:6666 pi@192.168.0.248",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--elf", default=DEFAULT_ELF, help=f"ELF to resolve CTRL_OBS from (default {DEFAULT_ELF})")
    p.add_argument("--addr", help="CTRL_OBS address override, e.g. 0x20000c40 (skips ELF lookup)")
    p.add_argument("--host", default="localhost", help="OpenOCD TCL RPC host (default localhost)")
    p.add_argument("--port", type=int, default=6666, help="OpenOCD TCL RPC port (default 6666)")
    p.add_argument("--record", metavar="FILE", help="append timestamped CSV (unix_time,pitch_mdeg,flags)")
    p.add_argument("--checklist", action="store_true", help="guided pitch sign checklist")
    p.add_argument("--selftest", action="store_true", help="run parsers against canned data; no hardware")
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
    if args.checklist:
        return checklist(client, addr, args.record)
    stream(client, addr, args.record)
    return 0


if __name__ == "__main__":
    sys.exit(main())
