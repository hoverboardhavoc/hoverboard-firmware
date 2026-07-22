#!/usr/bin/env python3
"""Unit tests for tools/imu-tilt.py. Run: python3 -m unittest -v tools.test_imu_tilt
(or from tools/: python3 -m unittest -v test_imu_tilt).

Covers: ELF CTRL_OBS symbol resolution (against the real release ELF if present, else a
synthesized minimal ELF32 fixture), mdw reply parsing, flags decode, and the checklist sign logic.
stdlib only.
"""

import importlib.util
import os
import struct
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
        self.assertEqual(v, "RECORD")
        self.assertIn("deferred", note)


if __name__ == "__main__":
    unittest.main()
