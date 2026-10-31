#!/usr/bin/env python3
"""Unit tests for tools/foc-trace.py (and the FOC words of the shared tools/swdobs.py map).

Run: python3 tools/tests/test_foc_trace.py

Covers specs/foc-torque.md section 9, "tools": the decode of CTRL_OBS words 47 to 49 (and the other
words the sampler prints) out of `hoverboard-cli read`'s hexdump, the word map against the
firmware's own offset pins, the 0x67 read, and the host-side frame arithmetic of `--frame`
(Clarke, forward Park, the sine table and `angle_path_offset`) against motorcontroller/commutation's check
values. stdlib only, no hardware.
"""

import contextlib
import importlib.util
import io
import math
import os
import re
import sys
import types
import unittest
from unittest import mock

_TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(_TOOLS)
sys.path.insert(0, _TOOLS)
_spec = importlib.util.spec_from_file_location("foc_trace", os.path.join(_TOOLS, "foc-trace.py"))
ft = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ft)
import swdobs  # noqa: E402

CTRL = 0x20000920
GAINS = 0x20000564


def hexdump(addr, data):
    """`hoverboard-cli read`'s output (tools/hoverboard-cli/src/main.rs, `hexdump`)."""
    rows = []
    for i in range(0, len(data), 16):
        rows.append(f"{addr + i:08x}: " + " ".join(f"{b:02x}" for b in data[i:i + 16]))
    return "\n".join(rows) + "\n"


def to_bytes(words):
    return b"".join((w & 0xFFFFFFFF).to_bytes(4, "little") for w in words)


def pair(lo, hi):
    """motorcontroller/firmware/src/motor.rs `pack_i16_pair`."""
    return (lo & 0xFFFF) | ((hi & 0xFFFF) << 16)


def block(**f):
    """A CTRL_OBS block with the words the sampler decodes."""
    w = [0] * swdobs.CTRL_OBS_WORDS
    w[0] = swdobs.CTRL_MAGIC
    w[swdobs.W_TORQUE_MODE] = (f.get("torque", 0) & 0xFFFF) | (f.get("mode", 0) << 16)
    w[swdobs.W_MOTOR_STATE] = (f.get("method", 0) << 16) | (f.get("mflags", 0x87) << 24)
    w[swdobs.W_DUTY2_ANGLE] = f.get("angle", 0) << 16
    w[swdobs.W_MOTOR_FAULT] = f.get("fault", 0)
    w[swdobs.W_MOTOR_SPEED] = f.get("speed", 0) & 0xFFFFFFFF
    w[swdobs.W_MOTOR_RATE_Q] = f.get("rate_q", 0) & 0xFFFFFFFF
    w[swdobs.W_MOTOR_CURRENT] = ((f.get("peak", 0) & 0xFFFF) | (f.get("chopped", 0) << 16)
                                 | (f.get("trips", 0) << 24))
    w[swdobs.W_FOC_PHASE_MEAN] = pair(f.get("ia", 0), f.get("ib", 0))
    w[swdobs.W_FOC_DQ_MEAN] = pair(f.get("id", 0), f.get("iq", 0))
    w[swdobs.W_FOC_REF_VQ] = pair(f.get("iq_ref", 0), f.get("vq", 0))
    w[swdobs.W_FOC_HOLD] = f.get("hold", 0)
    return w


class FakeCli:
    """hoverboard-cli behind a fake runner: RAM by address, and 0x67 per node."""

    def __init__(self, ram, cal=None):
        self.ram, self.cal, self.calls = dict(ram), dict(cal or {}), []

    def __call__(self, argv):
        self.calls.append(argv)
        cmd = [a for a in argv[1:]]
        while cmd and cmd[0].startswith("--"):
            cmd = cmd[2:]
        if cmd[0] == "read":
            addr, n = int(cmd[1], 16), int(cmd[2])
            data = bytes(self.ram.get(addr + i, 0) for i in range(n))
            return types.SimpleNamespace(returncode=0, stdout=hexdump(addr, data), stderr="")
        if cmd[0] == "config":
            node = cmd[1]
            if node in self.cal:
                v = self.cal[node]
                out = f"0x67.0 at 0x01 ({node}): ok {v} ({v:#06x})\n"
            else:
                out = f"0x67.0 at 0x01 ({node}): armed \n"
            return types.SimpleNamespace(returncode=0, stdout=out, stderr="")
        return types.SimpleNamespace(returncode=2, stdout="", stderr="usage")


def ram_of(addr, words):
    return {addr + i: b for i, b in enumerate(to_bytes(words))}


class WordMap(unittest.TestCase):
    def test_the_foc_words_are_where_the_firmware_pins_them(self):
        """words 47 to 50 at the byte offsets motorcontroller/firmware/src/main.rs asserts, and the count
        its size pin asserts: the map and the struct are one statement."""
        self.assertEqual((swdobs.W_FOC_PHASE_MEAN, swdobs.W_FOC_DQ_MEAN, swdobs.W_FOC_REF_VQ,
                          swdobs.W_FOC_HOLD), (47, 48, 49, 50))
        with open(os.path.join(_REPO, "motorcontroller", "firmware", "src", "main.rs")) as fh:
            src = fh.read()
        for field, word in (("foc_phase_mean", swdobs.W_FOC_PHASE_MEAN),
                            ("foc_dq_mean", swdobs.W_FOC_DQ_MEAN),
                            ("foc_ref_vq", swdobs.W_FOC_REF_VQ),
                            ("foc_hold", swdobs.W_FOC_HOLD)):
            m = re.search(rf"offset_of!\(CtrlObs, {field}\) == (0x[0-9A-Fa-f]+)", src)
            self.assertIsNotNone(m, field)
            self.assertEqual(int(m.group(1), 16), 4 * word, field)
        m = re.search(r"size_of::<CtrlObs>\(\) == (\d+) \* 4", src)
        self.assertEqual(int(m.group(1)), swdobs.CTRL_OBS_WORDS)

    def test_i16_pair_is_the_inverse_of_the_pack(self):
        for lo, hi in ((0, 0), (300, -120), (-1, 0), (0, -1), (-32768, 32767), (2450, -7)):
            self.assertEqual(swdobs.i16_pair(pair(lo, hi)), (lo, hi))
        # motorcontroller/firmware/src/motor.rs i16_pair_packing_round_trips' pinned word.
        self.assertEqual(swdobs.i16_pair(0xFFF9_0992), (2450, -7))

    def test_parse_nm_keeps_foc_gains_and_does_not_require_it(self):
        motor = "\n".join(f"2000{0x700 + 4 * i:04x} 00000004 b _RNvNtCs5hLiQtQJ0AX_8firmware5motor{len(s)}{s}"
                          for i, s in enumerate(swdobs.MOTOR_SYMS))
        out = f"{CTRL:08x} 000000c8 B CTRL_OBS\n{GAINS:08x} 00000018 D FOC_GAINS\n" + motor
        syms = swdobs.parse_nm(out)
        self.assertEqual(syms["CTRL_OBS"], (CTRL, 200))
        self.assertEqual(syms["FOC_GAINS"], (GAINS, 24))
        self.assertNotIn("FOC_GAINS", swdobs.parse_nm(f"{CTRL:08x} 000000c8 B CTRL_OBS\n" + motor))


class Decode(unittest.TestCase):
    def test_hexdump_round_trip_and_little_endian_words(self):
        words = block(ia=-1200, ib=410, id=-37, iq=688, iq_ref=688, vq=-31_999)
        data = ft.parse_hexdump(hexdump(CTRL, to_bytes(words)), CTRL, 4 * len(words))
        self.assertEqual(ft.words_le(data), [w & 0xFFFFFFFF for w in words])

    def test_hexdump_refusals(self):
        good = hexdump(CTRL, bytes(range(32)))
        with self.assertRaises(RuntimeError):
            ft.parse_hexdump(good, CTRL, 48)  # short
        gap = good.replace(f"{CTRL + 16:08x}:", f"{CTRL + 32:08x}:")
        with self.assertRaises(RuntimeError):
            ft.parse_hexdump(gap, CTRL, 32)  # a row out of place
        with self.assertRaises(RuntimeError):
            ft.parse_hexdump("error: no reply\n", CTRL, 32)

    def test_decode_the_foc_words_in_counts_and_amps(self):
        w = block(torque=-8000, mode=3, method=2, angle=0x1234, fault=0x10, speed=-90, rate_q=-4096 * 3,
                  peak=1500, chopped=2, trips=1, ia=-1200, ib=410, id=-37, iq=688, iq_ref=688, vq=-31_999,
                  hold=0x2A05_1B12)
        r = ft.decode(w, 455)
        # The hold word: motorcontroller/firmware/src/motor.rs the_hold_word_packs_the_windows_counts_and_causes'
        # pinned word: phase a held 18 periods, phase b 27, 5 floated, the freshness guard and a
        # both-held period seen, two calibration block pairs this boot.
        self.assertEqual((r["held_a"], r["held_b"], r["coasted"], r["cause"]), (18, 27, 5, 0x2A))
        self.assertEqual((r["torque"], r["mode"], r["method"], r["mflags"]), (-8000, 3, 2, 0x87))
        self.assertEqual((r["angle"], r["fault"], r["speed"], r["rate"]), (0x1234, 0x10, -90, -3.0))
        self.assertEqual((r["peak"], r["chopped"], r["trips"]), (1500, 2, 1))
        self.assertEqual((r["ia"], r["ib"], r["id"], r["iq"], r["iq_ref"], r["vq"]),
                         (-1200, 410, -37, 688, 688, -31_999))
        self.assertAlmostEqual(r["iq_a"], 688 / 455)
        self.assertAlmostEqual(r["ia_a"], -1200 / 455)
        self.assertAlmostEqual(r["iq_ref_a"], 688 / 455)
        self.assertNotIn("vq_a", r, "vq is a voltage count, never amps")
        self.assertNotIn("theta", r, "no frame unless asked")

    def test_a_non_foc_board_reads_zero_dq_and_reference(self):
        r = ft.decode(block(method=1, ia=300, ib=-150), 455)
        self.assertEqual((r["ia"], r["ib"]), (300, -150), "word 47 is live in every method")
        self.assertEqual((r["id"], r["iq"], r["iq_ref"], r["vq"]), (0, 0, 0, 0))
        self.assertEqual((r["held_a"], r["held_b"], r["coasted"], r["cause"]), (0, 0, 0, 0))

    def test_bridge_argv_and_reads(self):
        words = block(iq=5, vq=-6)
        cli = FakeCli(ram_of(CTRL, words))
        b = ft.Bridge("hb", ["--tcp", "bridge.local:5333"], node="0x02", run=cli)
        self.assertEqual(b.read_words(CTRL, len(words)), [w & 0xFFFFFFFF for w in words])
        self.assertEqual(cli.calls[0], ["hb", "--tcp", "bridge.local:5333", "--node", "0x02",
                                        "read", f"0x{CTRL:08x}", str(4 * len(words))])
        failing = ft.Bridge("hb", ["--tcp", "x:1"], run=lambda a: types.SimpleNamespace(
            returncode=1, stdout="", stderr="error: no answer"))
        with self.assertRaises(RuntimeError):
            failing.read_words(CTRL, 4)

    def test_the_cal_read(self):
        cli = FakeCli({}, cal={"left": 455, "low": 40})
        b = ft.Bridge("hb", ["--tcp", "x:1"], node="0x02", run=cli)
        self.assertEqual(ft.read_cal(b, "left"), (455, 455))
        self.assertNotIn("--node", cli.calls[-1], "config addresses the node itself")
        self.assertEqual(cli.calls[-1][-3:], ["config", "left", "0x67.0"])
        self.assertEqual(ft.read_cal(b, "low"), (40, swdobs.CURRENT_CAL_MIN), "the boot seam's clamp")
        with self.assertRaises(SystemExit):
            ft.read_cal(b, "armed-board")
        self.assertEqual(ft.parse_config("0x67.0 at 0x01: ok 455 (0x01c7)\n", 0x67), ("ok", 455))
        self.assertEqual(ft.parse_config("0x67.0 at 0x01: armed \n", 0x67), ("armed", None))
        self.assertEqual(ft.parse_config("0x20.0 at 0x01: ok 10000 (0x00002710)\n", 0x67), (None, None))

    def test_gains_text(self):
        t = ft.gains_text(GAINS, [4000, 1024, 119, 1024, 32767, 32767 * 1024])
        self.assertIn(f"FOC_GAINS at 0x{GAINS:08x}", t)
        self.assertIn(f"0x{GAINS + 8:08x} ki         119", t)
        self.assertIn(f"0x{GAINS + 20:08x} int_max    {32767 * 1024}", t)


class HostFrame(unittest.TestCase):
    """The --frame arithmetic, against motorcontroller/commutation."""

    def test_the_sine_table_is_the_crates_literal(self):
        with open(os.path.join(_REPO, "motorcontroller", "commutation", "src", "sin_table.rs")) as fh:
            body = re.search(r"=\s*\[(.*?)\];", fh.read(), re.S).group(1)
        literal = [int(v) for v in re.findall(r"-?\d+", body)]
        self.assertEqual(len(literal), 256)
        self.assertEqual(ft.SIN_QUARTER, literal)
        # sine_table_bitexact's recovered check values.
        self.assertEqual([ft.SIN_QUARTER[i] for i in (0, 1, 127, 128, 255)], [0, 201, 23027, 23170, 32766])

    def test_lookup_check_values(self):
        """foc tests, lookup_check_values."""
        for theta, want in ((0x0000, (0, 32766)), (0x1000, (12539, 30195)), (0x4000, (32766, 0)),
                            (0x5FC0, (23170, -23027)), (0x8000, (0, -32766)), (0xC000, (-32766, 0)),
                            (0xE000, (-23027, 23170))):
            self.assertEqual(ft.lookup_sincos(theta), want, f"{theta:#06x}")

    def test_rounding_and_saturation(self):
        self.assertEqual((ft.sat16(40000), ft.sat16(-32768), ft.sat16(-40000), ft.sat16(-5)),
                         (32767, -32767, -32767, -5))
        # The crate's MAC biases negatives by 2^15 - 1 before the arithmetic shift, so it truncates
        # toward zero on both signs.
        self.assertEqual(ft.mac_q15(32768 + 16384), 1)
        self.assertEqual(ft.mac_q15(-(32768 + 16384)), -1)
        self.assertEqual(ft.mac_q15(-32766), 0)
        self.assertEqual(ft.mac_q15(-2 * 32766), -1, "an ia of -2 at theta 0 is an iq of -1")
        self.assertEqual(ft.mac_q15(-32768 * 32768), -32767, "the sentinel saturates to -32767")

    def test_park_and_clarke_check_values(self):
        """park_forward_check_values and clarke_constants_exact."""
        self.assertEqual(ft.park_forward(19660, 0, 0x0000), (19658, 0))
        self.assertEqual(ft.park_forward(19660, 0, 0x4000), (0, 19658))
        self.assertEqual((ft.CLARKE_A, ft.CLARKE_B), (0x49E6, 0x93CC))
        self.assertEqual(ft.clarke(5000, 1234)[0], 5000)

    def test_the_chain_matches_the_crate(self):
        """Clarke then forward Park over a spread of pairs and angles, the values motorcontroller/commutation's
        `clarke` and `park_forward` produce for the same inputs (including both saturations)."""
        for ia, ib, theta, beta, iq, i_d in (
            (1200, -700, 0x2345, 115, 685, 987),
            (-640, 410, 0x1556, -103, -501, -407),
            (32767, 32767, 0x9ABC, -32767, -32767, 6113),
            (-32767, 12000, 0xE321, 5061, -21528, 25081),
            (900, -300, 0xBFFF, -173, -172, -899),
            (0, -500, 0x7000, 577, -217, -533),
        ):
            alpha, b = ft.clarke(ia, ib)
            self.assertEqual((alpha, b), (ia, beta), f"clarke({ia}, {ib})")
            self.assertEqual(ft.park_forward(alpha, b, theta), (iq, i_d), f"park at {theta:#06x}")

    def test_angle_path_offset_is_the_crates(self):
        """commutation::angle_path_offset for every staged (direction, align offset), and the
        composition the_angle_path_offset_is_the_decodes_composition pins."""
        want = {(0, 0): 21845, (1, 0): 54613, (0, 1): 32768, (1, 1): 0, (0, 2): 43691, (1, 2): 10923,
                (0, 3): 54613, (1, 3): 21845, (0, 4): 0, (1, 4): 32768, (0, 5): 10923, (1, 5): 43691}
        for (d, off), v in want.items():
            self.assertEqual(ft.angle_path_offset(d, off), v, f"direction {d}, offset {off}")
        self.assertEqual(ft.sector_angle(3), 0x8000)
        self.assertEqual(ft.angle_path_offset(7, 8), ft.angle_path_offset(1, 2),
                         "a nonzero direction byte is reverse, and the offset is taken mod 6")

    def test_a_current_along_theta_reads_iq_and_one_behind_reads_id(self):
        """The crate's park_forward_reads_iq_along_theta_and_id_a_quarter_turn_behind, on the host:
        a physical current vector at the aligned angle reads iq = |i| and id = 0, and one 90 degrees
        behind reads id = +|i|, each to 1 %. This is the reading gate O2 judges."""
        amp = 10_000.0

        def phases(deg):
            return tuple(int(round(amp * math.cos(math.radians(deg - 120.0 * k)))) for k in (0, 1))

        for step in range(96):
            theta = (step * 683) & 0xFFFF
            deg = theta * 360.0 / 65536.0
            iq, i_d = ft.park_forward(*ft.clarke(*phases(deg)), theta)
            self.assertLess(abs(iq - amp), amp * 0.01, f"{theta:#06x}")
            self.assertLess(abs(i_d), amp * 0.01, f"{theta:#06x}")
            iq, i_d = ft.park_forward(*ft.clarke(*phases(deg - 90.0)), theta)
            self.assertLess(abs(i_d - amp), amp * 0.01, f"behind {theta:#06x}")
            self.assertLess(abs(iq), amp * 0.01, f"behind {theta:#06x}")

    def test_the_frame_runs_at_angle_plus_the_path_offset(self):
        """--frame: word 22's angle plus angle_path_offset(direction, align), over word 47."""
        aligned = 0x3000
        for d, off in ((0, 1), (1, 1), (1, 4)):
            angle = (aligned - ft.angle_path_offset(d, off)) & 0xFFFF
            ia, ib = 1200, -700
            r = ft.decode(block(angle=angle, ia=ia, ib=ib), 455, frame=(d, off))
            self.assertEqual(r["theta"], aligned)
            want = ft.park_forward(*ft.clarke(ia, ib), aligned)
            self.assertEqual((r["hiq"], r["hid"]), want)
            self.assertAlmostEqual(r["hmag"], math.hypot(*want))
        # A pair along the aligned angle passes gate O2's ratios; the same pair a half turn off is
        # the inverted-polarity reading (iq negative, id small).
        amp = 1400.0
        deg = aligned * 360.0 / 65536.0
        ia, ib = (int(round(amp * math.cos(math.radians(deg - 120.0 * k)))) for k in (0, 1))
        angle = (aligned - ft.angle_path_offset(0, 1)) & 0xFFFF
        r = ft.decode(block(angle=angle, ia=ia, ib=ib), 455, frame=(0, 1))
        self.assertGreaterEqual(r["hiq_frac"], 0.9)
        self.assertLessEqual(r["hid_frac"], 0.2)
        r = ft.decode(block(angle=angle, ia=-ia, ib=-ib), 455, frame=(0, 1))
        self.assertLess(r["hiq_frac"], -0.9)
        self.assertLessEqual(r["hid_frac"], 0.2)


class EndToEnd(unittest.TestCase):
    def test_one_sample_over_a_fake_bridge(self):
        """Stand-up and one row: the ELF's two addresses, the magic, the gains, the cal read and the
        FOC columns, with no hardware."""
        words = block(torque=-5000, method=2, ia=-300, ib=150, id=-12, iq=430, iq_ref=430, vq=420)
        ram = ram_of(CTRL, words)
        ram.update(ram_of(GAINS, [4000, 1024, 119, 1024, 32767, 32767 * 1024]))
        cli = FakeCli(ram, cal={"0x01": 455})

        class Stop(Exception):
            pass

        def stop(_secs):
            raise Stop()

        out = io.StringIO()
        with mock.patch.object(ft, "resolve", return_value=(CTRL, GAINS)), \
                mock.patch.object(ft.subprocess, "run", side_effect=lambda argv, **_: cli(argv)), \
                mock.patch.object(ft.time, "sleep", side_effect=stop), \
                mock.patch.object(ft.signal, "signal"), \
                contextlib.redirect_stdout(out):
            with self.assertRaises(Stop):
                ft.main(["--tcp", "bridge.local:5333", "--cal-node", "0x01", "--cli", "hb",
                         "--frame", "0", "1"])
        text = out.getvalue()
        self.assertIn(f"FOC_GAINS at 0x{GAINS:08x}", text)
        self.assertIn("kp_divisor 1024", text)
        self.assertIn("motor.current_cal (0x67) on 0x01 = 455", text)
        reads = [c for c in cli.calls if "read" in c]
        self.assertTrue(reads and all("--node" not in c for c in reads), "no --node, no bridge hop")
        row = text.strip().splitlines()[-1].split()
        self.assertEqual(row[1], "-5000")
        self.assertEqual(row[10:16], ["-300", "150", "-12", "430", "430", "420"])
        self.assertEqual(row[19], f"{430 / 455:.2f}")
        self.assertEqual(row[21:25], ["0", "0", "0", "00"], "the hold word's four columns")
        self.assertEqual(len(row), 25 + 6, "the six frame columns follow")

    def test_a_wrong_image_is_refused(self):
        words = block()
        words[0] = 0xDEADBEEF
        cli = FakeCli(ram_of(CTRL, words))
        with mock.patch.object(ft, "resolve", return_value=(CTRL, None)), \
                mock.patch.object(ft.subprocess, "run", side_effect=lambda argv, **_: cli(argv)), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as e:
                ft.main(["--tcp", "x:1", "--cal", "455", "--cli", "hb"])
        self.assertIn("magic", str(e.exception.code))

    def test_the_block_size_is_checked_against_the_map(self):
        with mock.patch.object(ft, "resolve_symbols_sized", return_value={"CTRL_OBS": (CTRL, 188)}):
            with self.assertRaises(SystemExit):
                ft.resolve("x.elf")
        with mock.patch.object(ft, "resolve_symbols_sized",
                               return_value={"CTRL_OBS": (CTRL, 4 * swdobs.CTRL_OBS_WORDS),
                                             "FOC_GAINS": (GAINS, 24)}):
            self.assertEqual(ft.resolve("x.elf"), (CTRL, GAINS))


if __name__ == "__main__":
    unittest.main()
