#!/usr/bin/env python3
"""Host tests for the bench smoke test's tools (specs/bench-ci.md): tools/bench/bench.py (the stamp,
the symbol span, the adb client, the speed-cap guard and the judge) and
tools/bench/runner-job-started.sh (the runner's job-started hook). stdlib only, no hardware.

Run: python3 tools/tests/test_bench.py
"""

import io
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import unittest

_TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_TOOLS, "bench"))
import bench  # noqa: E402

HOOK = os.path.join(_TOOLS, "bench", "runner-job-started.sh")
LIMITS = bench.load_json(os.path.join(_TOOLS, "bench", "limits.json"))
RIG = bench.load_json(os.path.join(_TOOLS, "bench", "offroad-rig.example.json"))
MACHINE = bench.load_json(os.path.join(os.path.dirname(_TOOLS), RIG["machine_file"]))

MASTER_UID = next(b["uid"] for b in RIG["boards"] if b["role"] == "master")
SLAVE_UID = next(b["uid"] for b in RIG["boards"] if b["role"] == "slave")
ADDR = {"master": 0x03, "slave": 0x02}


def side(uid):
    return MACHINE["boards"][uid]["motors"][0]["side"]


# ---------------------------------------------------------------------------------------------
# A synthetic run
# ---------------------------------------------------------------------------------------------

LAYOUT = {"addr": 0x200003F8, "len": 56, "offsets": {
    "WHEEL_RATE_Q12": 12, "FAULT": 32, "INVALID_DWELL": 0, "OBS_STATE": 52, "PERIODS": 48, "EDGES": 28}}

#: The phases, seconds after the arm.
PHASES = {"connect": -10, "arm": 0, "forward_ramp_up": 1, "forward_steady": 3, "forward_ramp_down": 6,
          "coast": 8, "reverse_ramp_up": 12, "reverse_steady": 14, "reverse_ramp_down": 17,
          "disarm": 19, "done": 19.5}
T0 = 1_800_000_000.0
RPM = 25.0


def q12_for(rpm, pp=15):
    return int(round(rpm / 60.0 * (bench.EDGES_PER_ELECTRICAL_REV * pp) / bench.WINDOWS_PER_SECOND * bench.Q12))


def direction_at(t):
    if PHASES["forward_ramp_up"] <= t < PHASES["coast"]:
        return 1
    if PHASES["reverse_ramp_up"] <= t < PHASES["disarm"]:
        return -1
    return 0


class Run:
    """The recordings of one run, written into a directory: by default a run that passes."""

    def __init__(self):
        self.jitter = 0.02          # fraction of speed, alternating
        self.edge_jitter = 0.02
        self.mode_drop = None       # (role, t): leaves RUN at t
        self.fault = None           # (role, t, bits)
        self.boot_tag_change = None  # role
        self.flag_reboot = None     # role
        self.peak_counts = 600
        self.wrong_way = None       # role
        self.speed_scale = 1.0
        self.hall_fault = None      # role
        self.bad_code = None        # role
        self.test_ok = True
        self.status_after_ok = True
        self.guard = False
        self.cut_after = None       # the last phase the test reported

    def raw_rate(self, role, t, i):
        d = direction_at(t)
        if not d:
            return 0
        if role == self.wrong_way:
            d = -d
        r = RPM * self.speed_scale * (1 + (self.jitter if i % 2 else -self.jitter))
        fold = -1 if side(MASTER_UID if role == "master" else SLAVE_UID) == "right" else 1
        return fold * d * q12_for(r)

    def write(self, d):
        def w(name, lines):
            with open(os.path.join(d, name), "w") as f:
                f.write("".join(f"{e:.3f} {l}\n" for e, l in lines))

        names = list(PHASES)
        shown = names[:names.index(self.cut_after) + 1] if self.cut_after else names
        w("instrument.txt", [(T0 + PHASES[n], f"INSTRUMENTATION_STATUS: bench_phase={n}") for n in shown]
          + ([(T0 + 20, "OK (1 test)")] if self.test_ok else
             [(T0 + (5.5 if self.cut_after else 20), "java.lang.AssertionError: board 2 shows the lamp(s) [HALL]"),
              (T0 + (5.5 if self.cut_after else 20), "FAILURES!!!")]))
        w("nodes.txt", [(T0 - 30, "address  class             uid                       name"),
                        (T0 - 30, f"0x03     motor controller  {MASTER_UID}  Offroad"),
                        (T0 - 30, f"0x02     motor controller  {SLAVE_UID}  Hoverboard")])
        for role in ("master", "slave"):
            w(f"cal-{role}.txt", [(T0 - 20, f"x: motors[0].current.cal = 455 (default)")])
        frames = []
        for i in range(int(32 / 0.2)):
            t = -8 + i * 0.2
            for role in ("master", "slave"):
                armed = 0.5 <= t <= 19.2
                mode = bench.MODE_RUN if armed else 0
                if self.mode_drop and self.mode_drop[0] == role and t >= self.mode_drop[1] and armed:
                    mode = 2
                fault = bench.EV_POWER_REQUEST if armed else 0
                if self.fault and self.fault[0] == role and t >= self.fault[1]:
                    fault |= self.fault[2]
                frames.append((T0 + t, json.dumps({
                    "t": t + 8, "src": ADDR[role], "mode": mode, "fault": fault,
                    "flags": bench.FLAG_REBOOT if role == self.flag_reboot else 0,
                    "boot_tag": 2 if (role == self.boot_tag_change and t > 10) else 1,
                    "wheel_speed": 0, "wheel_rate": self.raw_rate(role, t, i), "wheel_pos": 0,
                    "phase_peak": self.peak_counts if armed else 300, "phase_mean": 200, "duty_on": 0,
                    "battery": 3095, "peer_demand": 0, "pitch": 0, "roll": 0, "odometer": 0,
                    "machine": 1, "motor1": None})))
        w("watch.txt", frames)
        for role in ("master", "slave"):
            rows = []
            for i in range(int(32 / 0.05)):
                t = -8 + i * 0.05
                rate = self.raw_rate(role, t, i)
                if rate:
                    rate = int(rate * (1 + (self.edge_jitter if i % 2 else -self.edge_jitter)))
                b = bytearray(LAYOUT["len"])
                struct.pack_into("<i", b, LAYOUT["offsets"]["WHEEL_RATE_Q12"], rate)
                struct.pack_into("<I", b, LAYOUT["offsets"]["FAULT"], bench.FAULT_HALL if role == self.hall_fault and t > 5 else 0)
                code = 7 if (role == self.bad_code and 4 < t < 4.1) else 3
                struct.pack_into("<I", b, LAYOUT["offsets"]["OBS_STATE"], code)
                rows.append((T0 + t, f"{i * 0.05:.3f} {b.hex()}"))
            w(f"reads-{role}.txt", rows)
        with open(os.path.join(d, "span.json"), "w") as f:
            json.dump(LAYOUT, f)
        w("status-after.txt", [(T0 + 25, "Hoverboard: ok"),
                               (T0 + 25, "Offroad: ok" if self.status_after_ok else "Offroad: self_hold pin not configured")])
        if self.guard:
            with open(os.path.join(d, "guard.txt"), "w") as f:
                f.write("speed cap: reads-master.txt read at 130.0 rpm, cap 120 rpm\n")


def judged(run):
    with tempfile.TemporaryDirectory() as d:
        run.write(d)
        return bench.judge(d, LIMITS, RIG, MACHINE)


class JudgeTest(unittest.TestCase):
    def failing(self, run, *needles):
        v = judged(run)
        self.assertFalse(v["pass"], v)
        text = "\n".join(v["failures"])
        for n in needles:
            self.assertIn(n, text)
        return v

    def test_a_clean_run_passes_with_its_figures(self):
        v = judged(Run())
        self.assertTrue(v["pass"], v["failures"])
        for role in ("master", "slave"):
            fig = v["figures"][role]
            self.assertAlmostEqual(fig["forward_rpm"], RPM, delta=1)
            self.assertAlmostEqual(fig["reverse_rpm"], -RPM, delta=1)
            self.assertLess(fig["forward_speed_cv"], 0.05)
            self.assertLess(fig["forward_edge_cv"], 0.05)
            self.assertAlmostEqual(fig["phase_peak_a"], 600 / 455, places=2)

    def test_a_wheel_turning_the_wrong_way_fails(self):
        r = Run()
        r.wrong_way = "slave"
        self.failing(r, "slave forward: turned the wrong way", "slave reverse: turned the wrong way")

    def test_a_wheel_below_ten_rpm_fails(self):
        r = Run()
        r.speed_scale = 0.3
        self.failing(r, "did not reach 10 rpm")

    def test_a_wheel_above_the_cap_fails(self):
        r = Run()
        r.speed_scale = 5.0
        self.failing(r, "above the 120 rpm cap")

    def test_unsteady_speed_fails_on_the_speed_cv(self):
        r = Run()
        r.jitter = 0.2
        self.failing(r, "steady speed CV")

    def test_jittery_halls_fail_on_the_edge_cv(self):
        r = Run()
        r.edge_jitter = 0.3
        v = self.failing(r, "hall-edge interval CV")
        self.assertNotIn("steady speed CV", "\n".join(v["failures"]))

    def test_phase_current_above_the_ceiling_fails(self):
        r = Run()
        r.peak_counts = 1000
        self.failing(r, "phase current 2.20 A peak")

    def test_a_fault_bit_while_armed_fails_and_the_arm_request_does_not(self):
        r = Run()
        r.fault = ("master", 5.0, 1 << 1)
        self.failing(r, "master: fault bits 0x0002 while armed")

    def test_a_disarm_the_test_did_not_command_fails(self):
        r = Run()
        r.mode_drop = ("slave", 10.0)
        self.failing(r, "slave: left RUN while armed")

    def test_a_reboot_fails_by_boot_tag_and_by_flag(self):
        r = Run()
        r.boot_tag_change = "master"
        self.failing(r, "master: boot tag changed")
        r = Run()
        r.flag_reboot = "slave"
        self.failing(r, "slave: FLAG_REBOOT set")

    def test_a_hall_fault_and_an_invalid_hall_state_fail(self):
        r = Run()
        r.hall_fault = "master"
        self.failing(r, "master: hall fault")
        r = Run()
        r.bad_code = "slave"
        self.failing(r, "slave: invalid hall state(s) [7]")

    def test_a_failed_instrumented_test_fails_with_its_message(self):
        r = Run()
        r.test_ok = False
        self.failing(r, "instrumented test failed: board 2 shows the lamp(s) [HALL]")

    def test_a_boot_report_entry_after_the_drive_fails(self):
        r = Run()
        r.status_after_ok = False
        self.failing(r, "board status after the drive")

    def test_a_guard_trip_fails(self):
        r = Run()
        r.guard = True
        self.failing(r, "speed cap tripped")

    def test_a_test_that_stopped_inside_a_hold_is_still_judged_up_to_where_it_stopped(self):
        r = Run()
        r.test_ok = False
        r.cut_after = "forward_steady"
        r.wrong_way = "master"
        v = self.failing(r, "instrumented test failed", "master forward: turned the wrong way")
        self.assertIn("slave: no reverse steady phase recorded", "\n".join(v["failures"]))

    def test_missing_recordings_fail_rather_than_pass(self):
        with tempfile.TemporaryDirectory() as d:
            v = bench.judge(d, LIMITS, RIG, MACHINE)
        self.assertFalse(v["pass"])
        self.assertIn("instrumented test failed", "\n".join(v["failures"]))


# ---------------------------------------------------------------------------------------------
# stamp, span, guard
# ---------------------------------------------------------------------------------------------


class StampTest(unittest.TestCase):
    def test_each_line_gets_the_clock(self):
        out = io.StringIO()
        clock = iter([1.0, 2.5])
        bench.stamp(io.StringIO("a\nb c\n"), out, clock=lambda: next(clock))
        self.assertEqual(out.getvalue(), "1.000 a\n2.500 b c\n")


def elf_with(symbols):
    """A minimal ELF32 LE file holding a symbol table with `symbols` (name -> value)."""
    strtab = b"\0"
    syms = [b"\0" * 16]
    for name, value in symbols.items():
        syms.append(struct.pack("<IIIBBH", len(strtab), value, 4, 0x11, 0, 1))
        strtab += name.encode() + b"\0"
    symtab = b"".join(syms)
    shstr = b"\0.strtab\0.symtab\0"
    body = strtab + symtab + shstr
    off_str, off_sym, off_shstr = 52, 52 + len(strtab), 52 + len(strtab) + len(symtab)
    shoff = 52 + len(body)
    sh = [
        b"\0" * 40,
        struct.pack("<IIIIIIIIII", 1, 3, 0, 0, off_str, len(strtab), 0, 0, 1, 0),
        struct.pack("<IIIIIIIIII", 9, 2, 0, 0, off_sym, len(symtab), 1, 1, 4, 16),
        struct.pack("<IIIIIIIIII", 17, 3, 0, 0, off_shstr, len(shstr), 0, 0, 1, 0),
    ]
    hdr = b"\x7fELF\x01\x01\x01" + b"\0" * 9 + struct.pack(
        "<HHIIIIIHHHHHH", 2, 40, 1, 0, 0, shoff, 0x05000200, 52, 0, 0, 40, len(sh), 3)
    return hdr + body + b"".join(sh)


class SpanTest(unittest.TestCase):
    def test_the_span_covers_the_motor_words_by_symbol(self):
        base = 0x20000400
        syms = {f"_RNvNtCsabc_8firmware5motor{len(w)}{w}.0": base + 8 * i for i, w in enumerate(bench.SPAN_WORDS)}
        syms["_RNvNtCsabc_8firmware5motor5OTHER"] = 0x20000000
        with tempfile.NamedTemporaryFile(suffix=".elf", delete=False) as f:
            f.write(elf_with(syms))
        try:
            s = bench.span(f.name)
        finally:
            os.unlink(f.name)
        self.assertEqual(s["addr"], base)
        self.assertEqual(s["len"], 8 * (len(bench.SPAN_WORDS) - 1) + 4)
        self.assertEqual(s["offsets"]["WHEEL_RATE_Q12"], 0)
        b = bytearray(s["len"])
        struct.pack_into("<i", b, 0, -4096)
        self.assertEqual(bench.decode_sample(b.hex(), s)["WHEEL_RATE_Q12"], -4096)

    def test_a_missing_word_is_refused(self):
        with tempfile.NamedTemporaryFile(suffix=".elf", delete=False) as f:
            f.write(elf_with({"_RNvNtCsabc_8firmware5motor5FAULT.0": 0x20000400}))
        try:
            with self.assertRaises(ValueError):
                bench.span(f.name)
        finally:
            os.unlink(f.name)


class GuardTest(unittest.TestCase):
    def test_it_trips_once_above_the_cap_and_runs_the_disarm(self):
        with tempfile.TemporaryDirectory() as d:
            watch = os.path.join(d, "watch.txt")
            reads = os.path.join(d, "reads-master.txt")
            fired = os.path.join(d, "fired")
            with open(watch, "w") as f:
                f.write("1.0 " + json.dumps({"src": 3, "wheel_rate": q12_for(100)}) + "\n")
            b = bytearray(LAYOUT["len"])
            struct.pack_into("<i", b, LAYOUT["offsets"]["WHEEL_RATE_Q12"], -q12_for(130))
            with open(reads, "w") as f:
                f.write(f"2.0 0.100 {b.hex()}\n")
            out = io.StringIO()
            tripped = bench.guard(120, f"touch {fired}", [watch, reads], LAYOUT, poll=0.01, out=out)
            self.assertTrue(tripped)
            self.assertTrue(os.path.exists(fired))
            self.assertIn("reads-master.txt read at -130", out.getvalue())

    def test_it_stays_quiet_below_the_cap_until_told_to_stop(self):
        with tempfile.TemporaryDirectory() as d:
            watch = os.path.join(d, "watch.txt")
            stop = os.path.join(d, "stop")
            with open(watch, "w") as f:
                f.write("1.0 " + json.dumps({"src": 3, "wheel_rate": q12_for(119)}) + "\n")
                f.write("1.1 " + json.dumps({"src": 2, "wheel_rate": -q12_for(119)}))  # no newline yet
            open(stop, "w").close()
            self.assertFalse(bench.guard(120, "false", [watch], LAYOUT, poll=0.01, stop_file=stop))


# ---------------------------------------------------------------------------------------------
# adb, against a fake server
# ---------------------------------------------------------------------------------------------


class FakeAdbServer(threading.Thread):
    """Answers the host protocol: transport, then one service per connection."""

    def __init__(self, replies):
        super().__init__(daemon=True)
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(4)
        self.port = self.sock.getsockname()[1]
        self.replies = replies
        self.services = []
        self.received = []

    def _req(self, c):
        n = int(c.recv(4), 16)
        return c.recv(n).decode()

    def run(self):
        while True:
            try:
                c, _ = self.sock.accept()
            except OSError:
                return
            with c:
                t = self._req(c)
                if not t.startswith("host:transport:"):
                    c.sendall(b"FAIL0004nope")
                    continue
                c.sendall(b"OKAY")
                service = self._req(c)
                self.services.append(service)
                c.sendall(b"OKAY")
                if "-S " in service:
                    want = int(service.rsplit("-S ", 1)[1])
                    got = b""
                    while len(got) < want:
                        got += c.recv(65536)
                    self.received.append(got)
                c.sendall(self.replies.pop(0).encode())


class AdbTest(unittest.TestCase):
    def test_install_streams_the_apk_and_reinstalls_over_a_foreign_signature(self):
        srv = FakeAdbServer([
            "Failure [INSTALL_FAILED_UPDATE_INCOMPATIBLE: Package com.x signatures do not match]\n",
            "Success\n", "Success\n"])
        srv.start()
        with tempfile.NamedTemporaryFile(delete=False) as f:
            f.write(b"apk-bytes" * 1000)
        try:
            out = bench.Adb("127.0.0.1", "SER", port=srv.port).install(f.name)
        finally:
            os.unlink(f.name)
            srv.sock.close()
        self.assertEqual(out, "Success")
        self.assertEqual(srv.services[1], "shell:pm uninstall com.x")
        self.assertEqual(srv.received, [b"apk-bytes" * 1000] * 2)

    def test_a_refused_install_raises(self):
        srv = FakeAdbServer(["Failure [INSTALL_FAILED_INSUFFICIENT_STORAGE]\n"])
        srv.start()
        with tempfile.NamedTemporaryFile(delete=False) as f:
            f.write(b"x")
        try:
            with self.assertRaises(RuntimeError):
                bench.Adb("127.0.0.1", "SER", port=srv.port).install(f.name)
        finally:
            os.unlink(f.name)
            srv.sock.close()


# ---------------------------------------------------------------------------------------------
# The runner's job-started hook
# ---------------------------------------------------------------------------------------------


class HookTest(unittest.TestCase):
    REPO = "hoverboardhavoc/hoverboard-firmware"

    def hook(self, event, actor, repo=REPO, head=None):
        env = {"PATH": os.environ["PATH"], "GITHUB_REPOSITORY": repo, "GITHUB_EVENT_NAME": event,
               "GITHUB_ACTOR": actor}
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            payload = {"pull_request": {"head": {"repo": {"full_name": head}}}} if head is not None else {}
            json.dump(payload, f)
        env["GITHUB_EVENT_PATH"] = f.name
        try:
            return subprocess.run(["bash", HOOK], env=env, capture_output=True, text=True)
        finally:
            os.unlink(f.name)

    def assertAllowed(self, r):
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("allowed", r.stdout)

    def assertRefused(self, r, why):
        self.assertNotEqual(r.returncode, 0)
        self.assertIn(why, r.stderr)

    def test_the_owner_on_push_schedule_and_dispatch_is_allowed(self):
        for ev in ("push", "schedule", "workflow_dispatch"):
            self.assertAllowed(self.hook(ev, "hoverboardhavoc"))

    def test_anyone_else_on_push_schedule_and_dispatch_is_refused(self):
        for ev in ("push", "schedule", "workflow_dispatch"):
            self.assertRefused(self.hook(ev, "someone"), "not the owner")
        self.assertRefused(self.hook("push", "dependabot[bot]"), "not the owner")

    def test_a_pull_request_from_this_repository_by_the_owner_or_dependabot_is_allowed(self):
        self.assertAllowed(self.hook("pull_request", "hoverboardhavoc", head=self.REPO))
        self.assertAllowed(self.hook("pull_request", "dependabot[bot]", head=self.REPO))

    def test_a_pull_request_from_a_fork_is_refused_whoever_opened_it(self):
        for actor in ("someone", "hoverboardhavoc", "dependabot[bot]"):
            self.assertRefused(self.hook("pull_request", actor, head="someone/hoverboard-firmware"), "a fork?")

    def test_a_pull_request_from_this_repository_by_another_actor_is_refused(self):
        self.assertRefused(self.hook("pull_request", "collaborator", head=self.REPO), "pull_request by 'collaborator'")

    def test_a_pull_request_with_no_readable_head_is_refused(self):
        self.assertRefused(self.hook("pull_request", "hoverboardhavoc"), "head repository ''")

    def test_other_events_and_other_repositories_are_refused(self):
        self.assertRefused(self.hook("pull_request_target", "hoverboardhavoc", head=self.REPO), "event 'pull_request_target'")
        self.assertRefused(self.hook("issue_comment", "hoverboardhavoc"), "event 'issue_comment'")
        self.assertRefused(self.hook("push", "hoverboardhavoc", repo="hoverboardhavoc/other"), "is not")


if __name__ == "__main__":
    unittest.main()
