#!/usr/bin/env python3
"""Unit tests for tools/imu-tilt.py. Run: python3 -m unittest -v tools.test_imu_tilt
(or from tools/: python3 -m unittest -v test_imu_tilt).

Covers: ELF CTRL_OBS symbol resolution (against the real release ELF if present, else a
synthesized minimal ELF32 fixture), mdw reply parsing, flags decode, and the checklist sign logic.
stdlib only.
"""

import importlib.util
import io
import os
import struct
import tempfile
import unittest

# Load the hyphenated module by path.
_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("imu_tilt", os.path.join(_HERE, "imu-tilt.py"))
imu_tilt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(imu_tilt)

REAL_ELF = os.path.join(
    _HERE, "..", "target", "thumbv7m-none-eabi", "release", "firmware"
)


def make_minimal_elf(symbols):
    """Build a minimal valid ELF32 LE with a .symtab + .strtab holding `symbols` {name: value}.

    Matches what elf_resolve_symbol needs: a valid ELF header (e_shoff/e_shentsize/e_shnum), a
    SHT_SYMTAB section whose sh_link points at the string table, and the symbol/string bytes.
    """
    end = "<"
    SHENT = 40  # ELF32 section header size
    SYMENT = 16  # ELF32 symbol size

    # String table: leading NUL, then each name NUL-terminated.
    strtab = bytearray(b"\x00")
    name_off = {}
    for name in symbols:
        name_off[name] = len(strtab)
        strtab += name.encode() + b"\x00"

    # Symbol table: index 0 is the reserved null symbol.
    symtab = bytearray(SYMENT)  # null symbol
    for name, value in symbols.items():
        symtab += struct.pack(end + "IIIBBH", name_off[name], value, 0, 0, 0, 1)

    # Layout: [ehdr(52)][symtab][strtab][section headers x3]
    ehdr_size = 52
    symtab_off = ehdr_size
    strtab_off = symtab_off + len(symtab)
    shoff = strtab_off + len(strtab)

    # Section headers: 0 = null, 1 = symtab (link->2), 2 = strtab.
    def shdr(sh_type, sh_offset, sh_size, sh_link, sh_entsize):
        return struct.pack(
            end + "IIIIIIIIII",
            0,  # name
            sh_type,
            0,  # flags
            0,  # addr
            sh_offset,
            sh_size,
            sh_link,
            0,  # info
            4,  # addralign
            sh_entsize,
        )

    sh_null = shdr(0, 0, 0, 0, 0)
    sh_sym = shdr(2, symtab_off, len(symtab), 2, SYMENT)  # SHT_SYMTAB, link -> strtab (index 2)
    sh_str = shdr(3, strtab_off, len(strtab), 0, 0)  # SHT_STRTAB
    shdrs = sh_null + sh_sym + sh_str

    # ELF32 header.
    e_ident = b"\x7fELF" + bytes([1, 1, 1]) + b"\x00" * 9  # ELF32, LE, version 1
    ehdr = e_ident + struct.pack(
        end + "HHIIIIIHHHHHH",
        2,  # e_type ET_EXEC
        0x28,  # e_machine EM_ARM
        1,  # e_version
        0,  # e_entry
        0,  # e_phoff
        shoff,  # e_shoff
        0,  # e_flags
        ehdr_size,  # e_ehsize
        0,  # e_phentsize
        0,  # e_phnum
        SHENT,  # e_shentsize
        3,  # e_shnum
        0,  # e_shstrndx
    )
    assert len(ehdr) == ehdr_size, len(ehdr)
    return bytes(ehdr) + bytes(symtab) + bytes(strtab) + shdrs


class TestElfResolve(unittest.TestCase):
    def test_synthetic_fixture(self):
        blob = make_minimal_elf({"CTRL_OBS": 0x20000C40, "BOARD_OBS": 0x20000AD4})
        path = os.path.join(_HERE, ".test_fixture.elf")
        with open(path, "wb") as fh:
            fh.write(blob)
        try:
            self.assertEqual(imu_tilt.elf_resolve_symbol(path, "CTRL_OBS"), 0x20000C40)
            self.assertEqual(imu_tilt.elf_resolve_symbol(path, "BOARD_OBS"), 0x20000AD4)
            with self.assertRaises(ValueError):
                imu_tilt.elf_resolve_symbol(path, "NOPE")
        finally:
            os.remove(path)

    def test_not_elf(self):
        path = os.path.join(_HERE, ".test_notelf.bin")
        with open(path, "wb") as fh:
            fh.write(b"not an elf at all")
        try:
            with self.assertRaises(ValueError):
                imu_tilt.elf_resolve_symbol(path, "CTRL_OBS")
        finally:
            os.remove(path)

    @unittest.skipUnless(os.path.exists(REAL_ELF), "release firmware ELF not built")
    def test_real_elf(self):
        addr = imu_tilt.elf_resolve_symbol(REAL_ELF, "CTRL_OBS")
        # In RAM (0x2000_0000..) on both parts; sane range.
        self.assertGreaterEqual(addr, 0x20000000)
        self.assertLess(addr, 0x20010000)


class TestParseMdw(unittest.TestCase):
    def test_multiline(self):
        text = (
            "0x20000c3c: 4c525443 00000001 00000d02 000014f7\n"
            "0x20000c4c: 00000cf6 00000340 fffff7c2 00000001\n"
        )
        words = imu_tilt.parse_mdw(text)
        self.assertEqual(len(words), 8)
        self.assertEqual(words[0], 0x4C525443)
        self.assertEqual(words[6], 0xFFFFF7C2)

    def test_address_not_counted(self):
        # The 0x-prefixed address is 8 hex after 0x; ensure it is not parsed as a data word.
        words = imu_tilt.parse_mdw("0x20000c3c: 00000001 00000002 00000003 00000004")
        self.assertEqual(words, [1, 2, 3, 4])

    def test_ignores_noise(self):
        words = imu_tilt.parse_mdw("some log line\n0x2000: deadbeef\nblah")
        self.assertEqual(words, [0xDEADBEEF])


class TestDecode(unittest.TestCase):
    def test_decode_sample_healthy(self):
        words = imu_tilt.parse_mdw(imu_tilt._SELFTEST_MDW)
        s = imu_tilt.decode_sample(words)
        self.assertTrue(s.magic_ok)
        self.assertEqual(s.pitch_mdeg, -2110)
        self.assertEqual(s.flags, 0x03)

    def test_decode_short_read(self):
        with self.assertRaises(ValueError):
            imu_tilt.decode_sample([1, 2, 3])

    def test_signed(self):
        self.assertEqual(imu_tilt._to_signed32(0xFFFFF7A6), -2138)
        self.assertEqual(imu_tilt._to_signed32(0x7FFFFFFF), 0x7FFFFFFF)
        self.assertEqual(imu_tilt._to_signed32(0x80000000), -0x80000000)


class TestFlags(unittest.TestCase):
    def test_flags_from_word(self):
        self.assertEqual(imu_tilt.flags_from_word(0x00030000), 0x03)
        self.assertEqual(imu_tilt.flags_from_word(0x00110000), 0x11)

    def test_flags_str(self):
        self.assertEqual(imu_tilt.flags_str(0x03), "CONFIGURED LIVE")
        self.assertEqual(imu_tilt.flags_str(0x11), "CONFIGURED not-live LOSS")
        self.assertEqual(imu_tilt.flags_str(0x00), "unconfigured not-live")

    def test_decode_flags(self):
        d = imu_tilt.decode_flags(0x11)
        self.assertTrue(d["configured"])
        self.assertTrue(d["loss"])
        self.assertFalse(d["live"])
        self.assertFalse(d["comms_loss"])


class TestChecklistLogic(unittest.TestCase):
    def setUp(self):
        by_name = {s["name"]: s for s in imu_tilt.CHECKLIST}
        self.level = by_name["level"]
        self.forward = by_name["lean-forward"]
        self.back = by_name["lean-back"]
        self.roll = by_name["roll-right"]
        self.base = -2110

    def test_baseline_info(self):
        v, _ = imu_tilt.checklist_verdict(self.level, 0, self.base)
        self.assertEqual(v, "INFO")

    def test_baseline_large_warns(self):
        _, note = imu_tilt.checklist_verdict(self.level, 0, 25000)
        self.assertIn("WARN", note)

    def test_lean_forward(self):
        self.assertEqual(imu_tilt.checklist_verdict(self.forward, self.base, self.base - 8000)[0], "PASS")
        self.assertEqual(imu_tilt.checklist_verdict(self.forward, self.base, self.base + 8000)[0], "CHECK")
        # a tilt below the delta threshold does not score PASS
        self.assertEqual(imu_tilt.checklist_verdict(self.forward, self.base, self.base - 1000)[0], "CHECK")

    def test_lean_back(self):
        self.assertEqual(imu_tilt.checklist_verdict(self.back, self.base, self.base + 8000)[0], "PASS")
        self.assertEqual(imu_tilt.checklist_verdict(self.back, self.base, self.base - 8000)[0], "CHECK")

    def test_roll_is_record_only(self):
        v, note = imu_tilt.checklist_verdict(self.roll, self.base, 0)
        self.assertEqual(v, "RECORDED")
        self.assertIn("deferred", note)

    def test_step_expected(self):
        self.assertEqual(imu_tilt.step_expected(self.level), "baseline~0")
        self.assertEqual(imu_tilt.step_expected(self.forward), "negative")
        self.assertEqual(imu_tilt.step_expected(self.back), "positive")
        self.assertEqual(imu_tilt.step_expected(self.roll), "unconfirmed(roll-unpublished)")

    def test_pose_ids_present(self):
        ids = [s["id"] for s in imu_tilt.CHECKLIST]
        self.assertEqual(ids, ["level", "lean_forward", "lean_back", "roll_right", "roll_left"])


class TestEvidenceWriter(unittest.TestCase):
    def _writer(self, header=True):
        buf = io.StringIO()
        return buf, imu_tilt.EvidenceWriter(buf, write_header=header)

    def test_header_emitted(self):
        buf, _ = self._writer()
        self.assertEqual(buf.getvalue(), f"# columns: {imu_tilt.CSV_COLUMNS}\n")

    def test_header_suppressed(self):
        buf, _ = self._writer(header=False)
        self.assertEqual(buf.getvalue(), "")

    def test_labeled_and_empty_rows(self):
        buf, w = self._writer()
        w.row(1000.0, -2110, 0x03, "level")
        w.row(1000.1, 0, 0x11)  # plain live: empty label
        self.assertIn("1000.000,-2110,0x03,level\n", buf.getvalue())
        self.assertIn("1000.100,0,0x11,\n", buf.getvalue())

    def test_row_column_count(self):
        buf, w = self._writer(header=False)
        w.row(1.0, 5, 0x03, "x")
        line = buf.getvalue().strip()
        self.assertEqual(len(line.split(",")), 4)

    def test_marker_text_and_count(self):
        buf, w = self._writer(header=False)
        self.assertEqual(w.marker(1234.5, "forward"), "forward")
        self.assertEqual(w.marker(1235.0, ""), "2")  # empty -> running count
        self.assertEqual(w.marker(1236.0, "  spaces  "), "spaces")  # trimmed
        v = buf.getvalue()
        self.assertIn("# marker 1234.500 forward\n", v)
        self.assertIn("# marker 1235.000 2\n", v)

    def test_step_begin_end(self):
        buf, w = self._writer(header=False)
        w.step_begin("lean_forward", 2000.0, 2.0)
        w.step_end("lean_forward", -10110, "negative", "PASS", 20)
        v = buf.getvalue()
        self.assertIn("# step lean_forward begin t=2000.000 window=2.0s\n", v)
        self.assertIn("# step lean_forward end mean=-10.11deg n=20 expected=negative verdict=PASS\n", v)

    def test_step_end_none_mean(self):
        buf, w = self._writer(header=False)
        w.step_end("roll_right", None, "unconfirmed(roll-unpublished)", "RECORDED", 0)
        self.assertIn("mean=n/a", buf.getvalue())

    def test_summary_block(self):
        buf, w = self._writer(header=False)
        rows = [
            ("level", -2109, "baseline~0", "INFO"),
            ("roll_right", None, "unconfirmed(roll-unpublished)", "RECORDED"),
        ]
        prov = [("elf", "target/x/firmware"), ("ctrl_obs", "0x20000c40"), ("tool", "imu-tilt/1.1")]
        w.summary(rows, prov)
        v = buf.getvalue()
        self.assertIn("# summary\n", v)
        self.assertIn("# summary pose=level mean=-2.11deg expected=baseline~0 verdict=INFO\n", v)
        self.assertIn("# summary pose=roll_right mean=n/a expected=unconfirmed(roll-unpublished) verdict=RECORDED\n", v)
        self.assertIn("# summary elf=target/x/firmware\n", v)
        self.assertIn("# summary ctrl_obs=0x20000c40\n", v)

    def test_open_writes_header_on_new_file_only(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "ev.csv")
            w = imu_tilt.EvidenceWriter.open(path)
            w.row(1.0, -1, 0x03, "level")
            w.close()
            with open(path) as fh:
                first = fh.read()
            self.assertTrue(first.startswith(f"# columns: {imu_tilt.CSV_COLUMNS}\n"))
            # reopening an existing non-empty file must NOT re-emit the header
            w2 = imu_tilt.EvidenceWriter.open(path)
            w2.row(2.0, -2, 0x03, "level")
            w2.close()
            with open(path) as fh:
                self.assertEqual(fh.read().count("# columns:"), 1)


class TestChartRenderer(unittest.TestCase):
    def test_empty_ring(self):
        rows = imu_tilt.render_chart([], 60, 8)
        self.assertEqual(len(rows), 8)
        # a zero/baseline line is always drawn
        self.assertTrue(any("┼" in r for r in rows))  # ┼
        # no data points
        self.assertFalse(any("●" in r for r in rows))  # ●

    def test_height_floor(self):
        # height is clamped to >= 3 even if asked for less
        self.assertEqual(len(imu_tilt.render_chart([0], 60, 1)), 3)

    def test_flat_line_on_zero_row(self):
        rows = imu_tilt.render_chart([0] * 30, 60, 9)
        pt_rows = [i for i, r in enumerate(rows) if "●" in r]
        zero_rows = [i for i, r in enumerate(rows) if "┼" in r]
        self.assertTrue(pt_rows)
        # a flat zero series sits on the zero line
        self.assertEqual(set(pt_rows), set(zero_rows))

    def test_flat_nonzero_off_zero(self):
        rows = imu_tilt.render_chart([-2110] * 30, 60, 12)
        pt_rows = [i for i, r in enumerate(rows) if "●" in r]
        zero_rows = [i for i, r in enumerate(rows) if "┼" in r]
        self.assertTrue(pt_rows)
        # a resting negative pitch plots below the zero line (higher row index)
        self.assertTrue(min(pt_rows) > max(zero_rows))

    def test_ramp_rises(self):
        rows = imu_tilt.render_chart(list(range(-5000, 5001, 100)), 60, 14)
        self.assertEqual(len(rows), 14)
        # first plotted column is lower on screen (higher row idx) than the last
        cols = {}
        for ri, r in enumerate(rows):
            plot = r[imu_tilt.CHART_LABEL_W + 1 :]
            for ci, ch in enumerate(plot):
                if ch == "●":
                    cols.setdefault(ci, ri)
        xs = sorted(cols)
        self.assertGreater(cols[xs[0]], cols[xs[-1]])

    def test_autoscale_bounds(self):
        ymin, ymax = imu_tilt.nice_bounds(-5000, 5000)
        self.assertLessEqual(ymin, -5000)
        self.assertGreaterEqual(ymax, 5000)
        self.assertLessEqual(ymin, 0)
        self.assertGreaterEqual(ymax, 0)

    def test_autoscale_always_includes_zero(self):
        # an all-positive series still brackets zero (baseline visible)
        ymin, ymax = imu_tilt.nice_bounds(2000, 8000)
        self.assertLessEqual(ymin, 0)
        self.assertGreaterEqual(ymax, 8000)

    def test_narrow_width(self):
        rows = imu_tilt.render_chart([0, 1500, -1500], 10, 6)
        self.assertEqual(len(rows), 6)
        self.assertTrue(all(len(r) >= imu_tilt.CHART_LABEL_W + 1 for r in rows))

    def test_row_width_consistent(self):
        rows = imu_tilt.render_chart(list(range(-3000, 3000, 50)), 72, 10)
        widths = {len(r) for r in rows}
        self.assertEqual(len(widths), 1)  # every row the same width


if __name__ == "__main__":
    unittest.main()
