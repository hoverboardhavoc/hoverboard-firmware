#!/usr/bin/env python3
"""Unit tests for tools/panic-reach.py, the panic-reachability gate (specs/panic-free.md, req 4).

Run: python3 -m unittest discover -s tools/tests -p 'test_panic_reach.py'
 or: tools/panic-reach.py --selftest

Everything here runs against committed FIXTURE disassembly (tools/tests/fixtures/panic-reach/),
text in `objdump -d --no-show-raw-insn` format, so no ARM toolchain, no ELF and no board is needed.
The fixtures cover each property the gate claims: a clean closure (with panics present but reached
only from a cold path), a one-hop panic, a two-hop panic a direct-only check would have passed, a
root that no longer resolves, an unresolved indirect call, long-branch thunks, and a panic-looking
symbol the panic set does not claim. The parser's own corners (conditional `bl`, returns that are
not indirect calls, PC-relative switch tables) are asserted here too, because each one was a real
hole: `bleq` to a panic was invisible until the image turned out to carry three conditional calls.
"""

import importlib.util
import io
import os
import re
import sys
import unittest
from contextlib import redirect_stderr, redirect_stdout

_TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FIXTURES = os.path.join(_TOOLS, "tests", "fixtures", "panic-reach")
sys.path.insert(0, _TOOLS)
_spec = importlib.util.spec_from_file_location(
    "panic_reach", os.path.join(_TOOLS, "panic-reach.py")
)
pr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pr)

ISR = r"5motor2hw10period_isr"
CTRL = r"8firmware15control_task_cb"


def fixture(name):
    with open(os.path.join(_FIXTURES, name + ".txt"), "r") as fh:
        return fh.read()


def run(name, **kw):
    """Analyze a fixture with no cuts unless the test asks for some."""
    kw.setdefault("cuts", ())
    return pr.analyze(fixture(name), **kw)


def joined(rep):
    return "\n".join(rep.failures)


class Parsing(unittest.TestCase):
    def test_functions_and_sections(self):
        prog = pr.parse_disassembly(fixture("clean"))
        names = [f.name for f in prog.funcs]
        self.assertIn("_RNvNtNtCsHASH_8firmware5motor2hw10period_isr", names)
        # Every code section is read, not just the first.
        self.assertIn("_RNvNtCsHASH_8firmware8firmware9run_shell", names)
        # The `file format` banner line is not a function.
        self.assertTrue(all("file format" not in n for n in names))

    def test_edges_are_calls_and_tail_branches(self):
        prog = pr.parse_disassembly(fixture("clean"))
        isr = next(f for f in prog.funcs if ISR in f.name)
        targets = {prog.funcs[t].name for (t, _s, _m) in isr.edges}
        self.assertEqual(
            targets,
            {
                "_RNvNtCsHASH_4base2pi9pi_output",
                "_RNvNtCsHASH_11commutation3foc5svpwm",
            },
        )
        # An intra-function branch (`beq.n` back into the same body) is not an edge.
        self.assertEqual(len(isr.edges), 2)

    def test_returns_and_switch_tables_are_not_indirect_calls(self):
        prog = pr.parse_disassembly(fixture("clean"))
        for f in prog.funcs:
            self.assertEqual(f.indirect, [], f"{f.name} read a return/switch as indirect")

    def test_conditional_bl_is_a_call(self):
        # `bleq` is `bl` + `eq`, not `ble` + `q`. The real image carries three of them.
        prog = pr.parse_disassembly(fixture("two_hop"))
        pi = next(f for f in prog.funcs if "pi_output" in f.name)
        self.assertEqual(
            [prog.funcs[t].name for (t, _s, _m) in pi.edges],
            ["_RNvNtCsHASH_4core9panicking9panic_fmt"],
        )


class Reachability(unittest.TestCase):
    def test_clean_closure_passes(self):
        rep = run("clean")
        self.assertTrue(rep.ok, joined(rep))

    def test_panic_present_but_only_on_a_cold_path_passes(self):
        # `run_shell` calls `panic_bounds_check`; it is not a root, and the gate's roots say by
        # omission that cold-path panics stay (specs/panic-free.md, non-goals).
        rep = run("clean")
        self.assertTrue(rep.ok, joined(rep))
        self.assertIn("4 panic symbols", "\n".join(rep.notes))

    def test_one_hop_panic_fails_and_names_the_site(self):
        rep = run("one_hop")
        self.assertFalse(rep.ok)
        msg = joined(rep)
        self.assertIn("PANIC REACHABLE", msg)
        self.assertIn("panic_bounds_check", msg)
        self.assertIn("0x0800122a", msg)  # the call site's address
        self.assertIn(ISR, msg)

    def test_two_hop_panic_fails(self):
        # The defect the gate exists for: the root calls `pi_output`, which calls the panic. A gate
        # that only inspected the root's own `bl` targets would pass this.
        rep = run("two_hop")
        self.assertFalse(rep.ok)
        msg = joined(rep)
        self.assertIn("pi_output", msg)
        self.assertIn("panic_fmt", msg)

    def test_every_call_site_into_a_panic_is_listed(self):
        rep = run("clean", roots=((r"8firmware9run_shell", "a cold path, as a test root"),))
        self.assertFalse(rep.ok)
        self.assertIn("1 call site(s)", joined(rep))

    def test_every_shipped_root_resolves_in_the_clean_fixture(self):
        # Pins the fixtures to ROOTS: adding a root without giving the fixtures a function by that
        # name fails here rather than making every other test in this file fail obscurely.
        prog = pr.parse_disassembly(fixture("clean"))
        for pat, _why in pr.ROOTS:
            hits = [f.name for f in prog.funcs if re.search(pat, f.name)]
            self.assertEqual(len(hits), 1, f"root {pat} resolved to {hits}")

    def test_missing_root_fails_loudly(self):
        rep = run("missing_root")
        self.assertFalse(rep.ok)
        msg = joined(rep)
        self.assertIn("no longer resolves", msg)
        self.assertIn(CTRL, msg)

    def test_ambiguous_root_fails(self):
        rep = run("clean", roots=((r"panicking", "a pattern that matches several functions"),))
        self.assertFalse(rep.ok)
        self.assertIn("ambiguous", joined(rep))

    def test_unresolved_indirect_call_in_the_closure_fails(self):
        rep = run("indirect")
        self.assertFalse(rep.ok)
        msg = joined(rep)
        self.assertIn("unresolved indirect call", msg)
        self.assertIn("blx r3", msg)
        self.assertIn("0x08001308", msg)

    def test_thunks_resolve_by_name_and_do_not_read_as_indirect(self):
        rep = run("thunk")
        self.assertFalse(rep.ok)
        msg = joined(rep)
        # The panic thunk is the finding...
        self.assertIn("panic_bounds_check", msg)
        # ...and the ordinary RAM thunk's own `bx ip` is the thunk mechanism, not a finding.
        self.assertNotIn("unresolved indirect call", msg)
        self.assertNotIn("erase_ram", msg.split("PANIC REACHABLE")[0])


class PanicSet(unittest.TestCase):
    def test_unclaimed_panic_looking_symbol_fails(self):
        rep = run("stray_panic")
        self.assertFalse(rep.ok)
        msg = joined(rep)
        self.assertIn("no panic-set pattern claims", msg)
        self.assertIn("handle_alloc_error", msg)
        # A new `core::panicking` entry IS claimed by the named pattern, so it is not reported.
        self.assertNotIn("panic_nounwind_nobacktrace", msg)

    def test_known_panic_flavours_are_claimed(self):
        for name in (
            "_RNvNtCsX_4core9panicking18panic_bounds_check",
            "_RNvNtCsX_4core6option13unwrap_failed",
            "_RNvNtCsX_4core6result13unwrap_failed",
            "_RNvNtNtCsX_4core5slice5index16slice_index_fail",
            "_RNvNvNtCsX_4core5slice20copy_from_slice_impl17len_mismatch_fail",
            "_RNvCsX_7___rustc17rust_begin_unwind",
            "__Thumbv7ABSLongThunk__RNvNtCsX_4core9panicking18panic_bounds_check",
        ):
            self.assertIsNotNone(pr._claimed_by(name), name)

    def test_ordinary_symbols_are_not_panic_looking(self):
        # Vocabulary that used to trip a looser regex: a debug-monitor handler holds "bug", and a
        # board-observation field holds "fail".
        for name in (
            "DebugMonitor",
            "_RNvMs0_NtCsX_5board8plumbingNtB5_8BoardObs7failure",
            "_RNvNtCsX_8firmware8firmware4halt",
        ):
            self.assertIsNone(pr._claimed_by(name), name)
            self.assertIsNone(pr.PANIC_LOOKING.search(name), name)


class Cuts(unittest.TestCase):
    def test_a_named_cut_excludes_exactly_its_edge(self):
        cut = ((ISR, r"panic_bounds_check", "a test cut"),)
        rep = pr.analyze(fixture("one_hop"), cuts=cut)
        self.assertTrue(rep.ok, joined(rep))
        self.assertIn("cut applied", "\n".join(rep.notes))

    def test_a_cut_does_not_excuse_another_path_to_the_same_panic(self):
        # Cutting the root's own edge leaves the two-hop path failing.
        cut = ((ISR, r"panic_fmt", "a test cut on the wrong edge"),)
        rep = pr.analyze(fixture("two_hop"), cuts=cut)
        self.assertFalse(rep.ok)
        self.assertIn("stale", joined(rep))  # the cut matched nothing, AND the panic is reachable
        self.assertIn("PANIC REACHABLE", joined(rep))

    def test_a_stale_cut_fails(self):
        cut = ((r"8firmware4halt", r"panic_fmt", "an edge this image does not have"),)
        rep = pr.analyze(fixture("clean"), cuts=cut)
        self.assertFalse(rep.ok)
        self.assertIn("stale", joined(rep))

    def test_the_shipped_cuts_are_narrow(self):
        # A cut names one caller and one callee. A bare `.*` on either side would be a subtree
        # exclusion by another name.
        for caller, callee, why in pr.CUTS:
            self.assertNotIn(".*", caller[:2])
            self.assertTrue(len(why) > 80, "a cut carries its reason")


class Driver(unittest.TestCase):
    def test_main_reports_the_shipped_cuts_as_stale_on_a_foreign_image(self):
        # End to end through the driver with the SHIPPED cuts: the clean fixture has neither cut
        # edge, so each one is stale and the run fails. That is the loud-staleness rule, and it is
        # why a cut cannot quietly outlive the call it excused.
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            rc = pr.main(["--dis", os.path.join(_FIXTURES, "clean.txt")])
        self.assertEqual(rc, 1)
        self.assertIn("panic-reach:", out.getvalue())
        self.assertIn("stale", err.getvalue())
        # No panic was reachable in that fixture: the only findings are the stale cuts.
        self.assertNotIn("PANIC REACHABLE", err.getvalue())

    def test_main_fails_on_a_one_hop_fixture(self):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            rc = pr.main(["--dis", os.path.join(_FIXTURES, "one_hop.txt")])
        self.assertEqual(rc, 1)
        self.assertIn("PANIC REACHABLE", err.getvalue())

    def test_missing_input_is_a_setup_error_not_a_pass(self):
        err = io.StringIO()
        with redirect_stderr(err):
            rc = pr.main(["--dis", os.path.join(_FIXTURES, "does-not-exist.txt")])
        self.assertEqual(rc, 2)
        err = io.StringIO()
        with redirect_stderr(err):
            rc = pr.main(["/nonexistent/firmware.elf"])
        self.assertEqual(rc, 2)
        self.assertIn("no such ELF", err.getvalue())

    def test_empty_disassembly_fails_rather_than_passing_vacuously(self):
        rep = pr.analyze("", cuts=())
        self.assertFalse(rep.ok)
        self.assertIn("no functions", joined(rep))


if __name__ == "__main__":
    unittest.main()
