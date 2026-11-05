#!/usr/bin/env python3
"""The bench smoke test's host side (specs/bench-ci.md sections 4 to 7), run on the bench runner.

Python 3 standard library only, so it runs wherever python3 does: it talks to the adb server itself,
with no adb binary, and the job needs nothing else beyond hoverboard-cli.

Subcommands:

  stamp                       copy stdin to stdout, each line prefixed with the host's epoch time
  span <elf> <out.json>       the RAM span `board read --hz` samples: the motor words the judge
                              needs, found by symbol in the image's ELF, never by a fixed address
  adb install <apk>...        install APKs on the bench phone through the adb server on the adb
  adb uninstall <package>     host; `adb shell <cmd>` runs a shell command and copies its output,
  adb shell <cmd>             stamped with `--stamp`. Host and serial from --host and --serial.
  guard <cap_rpm> <trip-cmd> <file>...
                              watch the live recordings and run <trip-cmd> once when a wheel turns
                              faster than the cap (section 6)
  judge <dir> <limits.json> <rig.json> <machine.json>
                              judge a run's recordings against section 7; prints the verdict and
                              writes <dir>/verdict.json; exit 1 on a failed run

The recordings a run leaves in <dir> (smoke.sh writes them; every line starts with the epoch time
`stamp` gave it):

  nodes.txt           `machine nodes`, the address of each uid
  cal-<role>.txt      `board get <uid> motors.0.current.cal`
  span.json           `span`'s output
  watch.txt           `machine watch --json`, one CYCLIC_STATE frame per line
  reads-<role>.txt    `board read <uid> <addr> <len> --hz N`, one sample of the span per line
  instrument.txt      `am instrument -r`, the phases as `bench_phase=<name>` status lines
  status-after.txt    `board status` after the drive
  guard.txt           present when the guard tripped
"""

import json
import os
import re
import socket
import statistics
import struct
import subprocess
import sys
import time

# ---------------------------------------------------------------------------------------------
# Units
# ---------------------------------------------------------------------------------------------

#: `wheel_rate`'s Q12 unit: 4096 is one hall edge per 320-period window.
Q12 = 4096.0
#: 320-period windows per second at the 16 kHz PWM.
WINDOWS_PER_SECOND = 50.0
#: Hall edges per electrical revolution.
EDGES_PER_ELECTRICAL_REV = 6
#: `commutation::foc::RATE_Q_NUM`: a rate of RATE_Q_NUM / n is one edge pair n periods apart, so
#: an edge interval in periods is RATE_Q_NUM / |rate|.
RATE_Q_NUM = 1_310_720
#: The board's RUN mode byte (`orchestrator`'s mode machine; the app's BoardMode.RUN).
MODE_RUN = 3
#: The fault mask bit that is the arm request, not a fault (`orchestrator::events::EV_POWER_REQUEST`).
EV_POWER_REQUEST = 1 << 7
#: The motor-side hall level (`orchestrator::events::EV_MOTOR_HALL`).
EV_MOTOR_HALL = 1 << 8
#: `CyclicState::FLAG_REBOOT`.
FLAG_REBOOT = 1 << 4
#: `motor::FAULT_HALL`.
FAULT_HALL = 1


def load_json(path):
    with open(path) as f:
        return json.load(f)


def rpm(rate_q12, pole_pairs):
    """A `wheel_rate` (Q12 edges per window) as wheel revolutions per minute."""
    return rate_q12 / Q12 * WINDOWS_PER_SECOND / (EDGES_PER_ELECTRICAL_REV * pole_pairs) * 60.0


# ---------------------------------------------------------------------------------------------
# stamp
# ---------------------------------------------------------------------------------------------


def stamp(inp, out, clock=time.time):
    for line in inp:
        out.write(f"{clock():.3f} {line.rstrip(chr(10))}\n")
        out.flush()


def stamped(path):
    """The lines of a stamped file as (epoch, rest), skipping any that carry no stamp."""
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for line in f:
            head, _, rest = line.rstrip("\n").partition(" ")
            try:
                rows.append((float(head), rest))
            except ValueError:
                continue
    return rows


# ---------------------------------------------------------------------------------------------
# span: the motor words, by symbol
# ---------------------------------------------------------------------------------------------

#: The words of `motorcontroller/firmware/src/motor.rs` the judge reads, each a `[AtomicU32/I32;
#: COUNT]` static; motor 0's element is the symbol's address (a one-motor image splits the array
#: into a `.0` symbol).
SPAN_WORDS = ("WHEEL_RATE_Q12", "FAULT", "INVALID_DWELL", "OBS_STATE", "PERIODS", "EDGES")
#: Signed words, read as i32; the rest are u32.
SIGNED = ("WHEEL_RATE_Q12",)


def elf_symbols(path):
    """Every named symbol of a 32-bit little-endian ELF: name -> (value, size)."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        raise ValueError(f"{path}: not a 32-bit little-endian ELF")
    shoff, = struct.unpack_from("<I", data, 0x20)
    shentsize, shnum = struct.unpack_from("<HH", data, 0x2E)
    sections = [struct.unpack_from("<IIIIIIIIII", data, shoff + i * shentsize) for i in range(shnum)]
    syms = {}
    for sh in sections:
        if sh[1] != 2:  # SHT_SYMTAB
            continue
        off, size, link, entsize = sh[4], sh[5], sh[6], sh[9]
        stroff = sections[link][4]
        for i in range(size // entsize):
            name_off, value, sz, _info, _other, _shndx = struct.unpack_from("<IIIBBH", data, off + i * entsize)
            if not name_off:
                continue
            end = data.index(b"\0", stroff + name_off)
            syms[data[stroff + name_off:end].decode("ascii", "replace")] = (value, sz)
    return syms


def motor_symbol(syms, word):
    """The address of motor 0's element of `firmware::motor::<word>`, found by its mangled path."""
    pat = re.compile(r"8firmware5motor%d%s(\.0)?$" % (len(word), re.escape(word)))
    hits = sorted({v for name, (v, _s) in syms.items() if pat.search(name)})
    if len(hits) != 1:
        raise ValueError(f"firmware::motor::{word}: {len(hits)} symbols match, want one")
    return hits[0]


def span(elf):
    syms = elf_symbols(elf)
    words = {w: motor_symbol(syms, w) for w in SPAN_WORDS}
    base = min(words.values())
    length = max(words.values()) + 4 - base
    if length > 256:
        raise ValueError(f"the motor words span {length} B, more than one read should carry")
    return {"addr": base, "len": length, "offsets": {w: a - base for w, a in words.items()}}


def decode_sample(hexstr, layout):
    b = bytes.fromhex(hexstr)
    if len(b) < layout["len"]:
        return None
    out = {}
    for w, off in layout["offsets"].items():
        out[w] = struct.unpack_from("<i" if w in SIGNED else "<I", b, off)[0]
    return out


def read_samples(path, layout):
    """A `board read --hz` recording: (epoch, decoded words) per sample line `<t> <hex>`."""
    rows = []
    for epoch, rest in stamped(path):
        parts = rest.split()
        if len(parts) != 2 or not re.fullmatch(r"[0-9a-f]+", parts[1]):
            continue
        d = decode_sample(parts[1], layout)
        if d is not None:
            rows.append((epoch, d))
    return rows


# ---------------------------------------------------------------------------------------------
# adb: a client of the adb server's host protocol
# ---------------------------------------------------------------------------------------------


class Adb:
    """The adb server's smart-socket protocol (the server's `OVERVIEW.TXT`, `SERVICES.TXT`): a
    request is four hex digits of length and the text; the reply is OKAY or FAIL with a message."""

    def __init__(self, host, serial, port=5037, timeout=30):
        self.host, self.serial, self.port, self.timeout = host, serial, port, timeout

    def _request(self, s, text):
        s.sendall(b"%04x%s" % (len(text), text.encode()))
        status = self._exact(s, 4)
        if status == b"OKAY":
            return
        n = int(self._exact(s, 4), 16)
        raise RuntimeError(f"adb: {text}: {self._exact(s, n).decode(errors='replace')}")

    @staticmethod
    def _exact(s, n):
        b = b""
        while len(b) < n:
            chunk = s.recv(n - len(b))
            if not chunk:
                raise RuntimeError("adb: the server closed the connection")
            b += chunk
        return b

    def open(self, service, timeout=None):
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.settimeout(timeout)
        self._request(s, f"host:transport:{self.serial}")
        self._request(s, service)
        return s

    def stream(self, service, data=None, timeout=None):
        """Run a service, send `data` to it, and yield its output as it arrives."""
        s = self.open(service, timeout)
        try:
            if data is not None:
                s.sendall(data)
            while True:
                chunk = s.recv(4096)
                if not chunk:
                    return
                yield chunk
        finally:
            s.close()

    def run(self, service, data=None, timeout=120):
        return b"".join(self.stream(service, data, timeout)).decode(errors="replace")

    def shell(self, cmd, timeout=120):
        return self.run("shell:" + cmd, timeout=timeout)

    def _install(self, data, tries=3, timeout=90):
        """One streamed install, tried again when the phone never answers: on the bench an install
        has hung with the data sent and no reply, and the next attempt went through (2026-11-05)."""
        for attempt in range(tries):
            try:
                return self.run(f"exec:cmd package install -r -g -S {len(data)}", data, timeout=timeout)
            except (TimeoutError, socket.timeout):
                if attempt == tries - 1:
                    raise
                time.sleep(5)
        return ""

    def install(self, apk):
        """`pm install -r -g` over `exec:` with the APK on stdin. An app signed with another key is
        uninstalled first: the CI runners' debug keys differ from run to run."""
        with open(apk, "rb") as f:
            data = f.read()
        out = self._install(data)
        if "INSTALL_FAILED_UPDATE_INCOMPATIBLE" in out:
            pkg = re.search(r"Package (\S+) signatures", out)
            if pkg:
                self.shell(f"pm uninstall {pkg.group(1)}")
                out = self._install(data)
        if "Success" not in out:
            raise RuntimeError(f"adb install {apk}: {out.strip()}")
        return out.strip()


def adb_main(args):
    host = os.environ.get("BENCH_ADB_HOST")
    serial = os.environ.get("BENCH_PHONE")
    do_stamp = False
    while args and args[0].startswith("--"):
        opt = args.pop(0)
        if opt == "--host":
            host = args.pop(0)
        elif opt == "--serial":
            serial = args.pop(0)
        elif opt == "--stamp":
            do_stamp = True
        else:
            raise SystemExit(f"adb: unknown option {opt}")
    if not host or not serial or not args:
        raise SystemExit("usage: bench.py adb [--host H] [--serial S] [--stamp] install|uninstall|shell ...")
    a = Adb(host, serial)
    cmd, rest = args[0], args[1:]
    if cmd == "install":
        for apk in rest:
            print(f"{apk}: {a.install(apk)}")
        return 0
    if cmd == "uninstall":
        print(a.shell("pm uninstall " + " ".join(rest)).strip())
        return 0
    if cmd == "shell":
        pending = b""
        for chunk in a.stream("shell:" + " ".join(rest)):
            if not do_stamp:
                sys.stdout.write(chunk.decode(errors="replace"))
                sys.stdout.flush()
                continue
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                sys.stdout.write(f"{time.time():.3f} {line.decode(errors='replace').rstrip(chr(13))}\n")
                sys.stdout.flush()
        if pending:
            sys.stdout.write(f"{time.time():.3f} {pending.decode(errors='replace')}\n" if do_stamp else pending.decode())
        return 0
    raise SystemExit(f"adb: unknown command {cmd}")


# ---------------------------------------------------------------------------------------------
# guard: the live speed cap
# ---------------------------------------------------------------------------------------------


def live_rpm(line, layout, pole_pairs):
    """The wheel speeds one stamped recording line reports: [(key, rpm)]. A watch line is one
    board's frame; a read line one board's span."""
    _epoch, _, rest = line.partition(" ")
    rest = rest.strip()
    if rest.startswith("{"):
        try:
            f = json.loads(rest)
        except ValueError:
            return []
        out = [(f"watch:{f['src']}", rpm(f["wheel_rate"], pole_pairs))]
        if f.get("motor1"):
            out.append((f"watch:{f['src']}:1", rpm(f["motor1"]["wheel_rate"], pole_pairs)))
        return out
    parts = rest.split()
    if layout and len(parts) == 2 and re.fullmatch(r"[0-9a-f]+", parts[1]):
        d = decode_sample(parts[1], layout)
        if d is not None:
            return [("read", rpm(d["WHEEL_RATE_Q12"], pole_pairs))]
    return []


def guard(cap, trip_cmd, files, layout=None, pole_pairs=15, poll=0.1, stop_file=None, out=sys.stdout):
    """Follow `files` as they grow and run `trip_cmd` once, the first time any wheel reads faster
    than `cap` rpm. Returns when `stop_file` appears (or after a trip)."""
    handles = {}
    while True:
        for path in files:
            if path not in handles and os.path.exists(path):
                handles[path] = open(path)
            h = handles.get(path)
            if not h:
                continue
            while True:
                pos = h.tell()
                line = h.readline()
                if not line or not line.endswith("\n"):
                    h.seek(pos)
                    break
                for key, r in live_rpm(line, layout, pole_pairs):
                    if abs(r) > cap:
                        msg = f"{time.time():.3f} speed cap: {os.path.basename(path)} {key} at {r:.1f} rpm, cap {cap} rpm"
                        print(msg, file=out, flush=True)
                        subprocess.run(trip_cmd, shell=True)
                        return True
        if stop_file and os.path.exists(stop_file):
            return False
        time.sleep(poll)


# ---------------------------------------------------------------------------------------------
# judge
# ---------------------------------------------------------------------------------------------


def parse_nodes(path):
    """`machine nodes`: uid -> address."""
    out = {}
    for _e, rest in stamped(path):
        m = re.match(r"\s*(0x[0-9a-f]{2})\s+motor controller\s+([0-9a-f]{24})", rest)
        if m:
            out[m.group(2)] = int(m.group(1), 16)
    return out


def parse_cal(path):
    for _e, rest in stamped(path):
        m = re.search(r"current\.cal = (\d+)", rest)
        if m:
            return int(m.group(1))
    return None


def phases(path):
    """`bench_phase=<name>` status lines: name -> epoch, plus whether the test passed."""
    out, ok, failure = {}, False, None
    rows = stamped(path)
    if rows:
        out["end"] = rows[-1][0]
    for epoch, rest in rows:
        m = re.search(r"bench_phase=(\S+)", rest)
        if m:
            out.setdefault(m.group(1), epoch)
        if re.search(r"^OK \(1 test\)", rest):
            ok = True
        m = re.search(r"AssertionError: (.*)", rest)
        if m and failure is None:
            failure = m.group(1)
    return out, ok, failure


def cv(xs):
    if len(xs) < 2:
        return None
    m = statistics.fmean(xs)
    return statistics.pstdev(xs) / abs(m) if m else None


def judge(d, limits, rig, machine):
    fails, notes, figures = [], [], {}
    ph, test_ok, test_failure = phases(os.path.join(d, "instrument.txt"))
    if not test_ok:
        fails.append("instrumented test failed" + (f": {test_failure}" if test_failure else " (no OK line)"))
    guard_file = os.path.join(d, "guard.txt")
    if os.path.exists(guard_file):
        with open(guard_file) as f:
            fails.append("speed cap tripped: " + f.read().strip())
    layout_path = os.path.join(d, "span.json")
    layout = load_json(layout_path) if os.path.exists(layout_path) else None
    uid_addr = parse_nodes(os.path.join(d, "nodes.txt"))

    watch = []
    for epoch, rest in stamped(os.path.join(d, "watch.txt")):
        if rest.startswith("{"):
            try:
                watch.append((epoch, json.loads(rest)))
            except ValueError:
                pass

    armed_from = ph.get("arm")
    armed_to = ph.get("disarm", ph.get("end"))
    # A hold the test cut short (it failed inside it) ends where the test's output ends.
    windows = {
        +1: (ph.get("forward_steady"), ph.get("forward_ramp_down", ph.get("end"))),
        -1: (ph.get("reverse_steady"), ph.get("reverse_ramp_down", ph.get("end"))),
    }

    def inside(e, w):
        return w[0] is not None and w[1] is not None and w[0] <= e <= w[1]

    for b in rig["boards"]:
        role, uid = b["role"], b["uid"]
        entry = machine["boards"].get(uid, {})
        motor = (entry.get("motors") or [{}])[0]
        pp = motor.get("pole_pairs") or 15
        fold = -1 if motor.get("side") == "right" else 1
        addr = uid_addr.get(uid)
        fig = figures.setdefault(role, {})
        if addr is None:
            fails.append(f"{role}: uid {uid} missing from machine nodes")
            continue
        frames = [(e, f) for e, f in watch if f["src"] == addr]
        if not frames:
            fails.append(f"{role}: no CYCLIC_STATE recorded")
            continue
        # A reboot: the boot tag moves, or the board says it rebooted.
        tags = sorted({f["boot_tag"] for _e, f in frames})
        if len(tags) > 1:
            fails.append(f"{role}: boot tag changed during the run {tags} (a reboot, a brown-out?)")
        if any(f["flags"] & FLAG_REBOOT for _e, f in frames):
            fails.append(f"{role}: FLAG_REBOOT set")
        # Faults and uncommanded disarms while armed.
        if armed_from is not None and armed_to is not None:
            armed = [(e, f) for e, f in frames if armed_from <= e <= armed_to]
            run = [i for i, (_e, f) in enumerate(armed) if f["mode"] == MODE_RUN]
            if not run:
                fails.append(f"{role}: never in RUN while the test was armed")
            else:
                gap = [f["mode"] for _e, f in armed[run[0]:] if f["mode"] != MODE_RUN]
                if gap:
                    fails.append(f"{role}: left RUN while armed (modes {sorted(set(gap))})")
            bits = 0
            for _e, f in armed:
                bits |= f["fault"] & ~EV_POWER_REQUEST
            if bits:
                fails.append(f"{role}: fault bits {bits:#06x} while armed")
            cal = parse_cal(os.path.join(d, f"cal-{role}.txt")) or 0
            if cal <= 0:
                fails.append(f"{role}: no motors.0.current.cal")
            else:
                peak = max(f["phase_peak"] for _e, f in armed) / cal if armed else 0.0
                fig["phase_peak_a"] = round(peak, 2)
                if peak > limits["phase_current_ceiling_a"]:
                    fails.append(f"{role}: phase current {peak:.2f} A peak, ceiling {limits['phase_current_ceiling_a']} A")
        top = max(abs(rpm(f["wheel_rate"], pp)) for _e, f in frames)
        fig["max_rpm"] = round(top, 1)
        if top > limits["speed_cap_rpm"]:
            fails.append(f"{role}: {top:.1f} rpm, above the {limits['speed_cap_rpm']} rpm cap")
        # The halls, from the SWD reads.
        reads = read_samples(os.path.join(d, f"reads-{role}.txt"), layout) if layout else []
        if not reads:
            fails.append(f"{role}: no hall reads recorded")
        for _e, r in reads:
            if r["FAULT"] & FAULT_HALL:
                fails.append(f"{role}: hall fault (motor FAULT {r['FAULT']:#x})")
                break
        bad_codes = sorted({r["OBS_STATE"] & 0xFF for _e, r in reads} & {0, 7})
        if bad_codes:
            fails.append(f"{role}: invalid hall state(s) {bad_codes} read")
        if any(f["fault"] & EV_MOTOR_HALL for _e, f in frames):
            fails.append(f"{role}: EV_MOTOR_HALL in the fault mask")
        # Each direction's steady hold.
        for sign, w in windows.items():
            name = "forward" if sign > 0 else "reverse"
            if w[0] is None or w[1] is None:
                fails.append(f"{role}: no {name} steady phase recorded")
                continue
            speeds = [fold * rpm(f["wheel_rate"], pp) for e, f in frames if inside(e, w)]
            if not speeds:
                fails.append(f"{role} {name}: no telemetry in the steady hold")
                continue
            mean = statistics.fmean(speeds)
            fig[f"{name}_rpm"] = round(mean, 1)
            if max(abs(s) for s in speeds) < limits["min_rpm"]:
                fails.append(f"{role} {name}: did not reach {limits['min_rpm']} rpm (max {max(abs(s) for s in speeds):.1f})")
            if mean * sign <= 0:
                fails.append(f"{role} {name}: turned the wrong way (mean {mean:.1f} rpm)")
            c = cv(speeds)
            fig[f"{name}_speed_cv"] = None if c is None else round(c, 3)
            if c is not None and c > limits["speed_cv_max"]:
                fails.append(f"{role} {name}: steady speed CV {c:.3f} above {limits['speed_cv_max']}")
            intervals = [RATE_Q_NUM / abs(r["WHEEL_RATE_Q12"]) for e, r in reads if inside(e, w) and r["WHEEL_RATE_Q12"]]
            c = cv(intervals)
            fig[f"{name}_edge_cv"] = None if c is None else round(c, 3)
            fig[f"{name}_edge_samples"] = len(intervals)
            if c is None:
                fails.append(f"{role} {name}: fewer than two hall-edge samples in the steady hold")
            elif c > limits["edge_cv_max"]:
                fails.append(f"{role} {name}: hall-edge interval CV {c:.3f} above {limits['edge_cv_max']}")
    # The boot report after the drive.
    after = [rest for _e, rest in stamped(os.path.join(d, "status-after.txt"))]
    oks = [l for l in after if re.search(r": ok$", l)]
    if len(oks) < len(rig["boards"]):
        fails.append("board status after the drive: " + "; ".join(l for l in after if ":" in l) or "not recorded")
    return {"pass": not fails, "failures": fails, "figures": figures, "notes": notes}


def judge_main(args):
    if len(args) != 4:
        raise SystemExit("usage: bench.py judge <dir> <limits.json> <rig.json> <machine.json>")
    d, limits, rig, machine = args[0], *(load_json(p) for p in args[1:])
    v = judge(d, limits, rig, machine)
    with open(os.path.join(d, "verdict.json"), "w") as f:
        json.dump(v, f, indent=2)
    print("PASS" if v["pass"] else "FAIL")
    for line in v["failures"]:
        print("  fail: " + line)
    for role, fig in v["figures"].items():
        print(f"  {role}: " + ", ".join(f"{k} {val}" for k, val in fig.items()))
    return 0 if v["pass"] else 1


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    cmd, args = argv[0], argv[1:]
    if cmd == "stamp":
        stamp(sys.stdin, sys.stdout)
        return 0
    if cmd == "span":
        out = span(args[0])
        with open(args[1], "w") as f:
            json.dump(out, f)
        print(f"{out['addr']:#010x} {out['len']}")
        return 0
    if cmd == "adb":
        return adb_main(list(args))
    if cmd == "guard":
        cap, trip, files = float(args[0]), args[1], args[2:]
        layout = None
        if os.environ.get("BENCH_SPAN"):
            layout = load_json(os.environ["BENCH_SPAN"])
        tripped = guard(cap, trip, files, layout, int(os.environ.get("BENCH_POLE_PAIRS", "15")),
                        stop_file=os.environ.get("BENCH_GUARD_STOP"))
        return 1 if tripped else 0
    if cmd == "judge":
        return judge_main(args)
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
