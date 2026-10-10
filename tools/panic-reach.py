#!/usr/bin/env python3
"""No panic reachable from the motor hot paths: the call-graph gate (specs/panic-free.md, req 4).

`panic-halt` turns a panic into an infinite spin AT THE POINT OF FAILURE, so what a panic costs
depends entirely on where it fires. Inside the 16 kHz period ISR the spin holds the timer's compare
registers at the last duties written with MOE still set and no later code can take them down; inside
the 250 Hz control task it stops the watchdog feed, so an armed board holds its last duties for the
full 500 ms IWDG window before the reset. Those two contexts are the ones the arming model exists to
keep a controller in charge of, which makes a panic there a safety defect rather than a halt.

This gate reads the RELEASE ELF, builds the call graph from the disassembly, and fails if any panic
symbol is reachable from a named root. Six properties, each one a defect it exists to catch:

  1. TRANSITIVE, not direct. `period_isr` never calls `core::panicking` itself: it called
     `base::pi::pi_output`, which ended in `i32::clamp` over RAM-resident bounds, which asserts
     `min <= max`. A gate that inspected only the root's own `bl` targets would have passed the
     very defect it was written for, so the check is a closure, not a scan.
  2. ROOTS ARE NAMED (`ROOTS`), and a root that no longer resolves is a FAILURE, never a skip. Same
     rule as the hot-window membership gate in .github/workflows/ci.yml: a rename or an inline is a
     conscious list update. An ambiguous root (two matches) fails for the same reason.
  3. THE PANIC SET IS NAMED (`PANIC_SET`) and checked against what the image actually contains: any
     panic-LOOKING text symbol (`PANIC_LOOKING`) that no named pattern claims is a FAILURE. A new
     flavour of panic (`panic_nounwind_fmt`, `handle_alloc_error`, an `unreachable` stub) therefore
     cannot enter the image unnamed and be quietly excluded from the reachability question.
  4. INDIRECT CALLS ARE NOT SILENTLY CLEAN. A `blx` through a register cannot be resolved from the
     disassembly, so an unresolved indirect call anywhere inside a root's closure is a FAILURE: the
     property cannot be defeated by indirection without someone noticing. Today's hot paths have
     none (the HAL's control-handler seam is entered from the vector table, above these roots); the
     point is that adding one has to be deliberate. A PC-relative `tbb`/`tbh` switch table is NOT an
     indirect call: its offsets are byte-sized and relative to the table inside the same function,
     so it cannot leave it.
  5. IT RUNS IN CI on the same ELF as the hot-window gate, with a FAIL message naming the path from
     the root to the panic, function by function, with the call site's address.
  6. ITS PARSER IS HOST-TESTED against committed fixture disassembly (tools/tests/test_panic_reach.py
     plus tools/tests/fixtures/panic-reach/): a clean closure, a one-hop panic, a two-hop panic, a
     missing root, an unresolved indirect call, a long-branch thunk, a stray panic-looking symbol.
     The fixtures are text, so the tests need no ARM toolchain and no board.

Cold-path panics (boot, config, the store) are deliberately out of scope: the roots say so by
omission, and the non-goals in specs/panic-free.md say why.

Symbols are matched MANGLED, exactly as `arm-none-eabi-nm` prints them and exactly as the
hot-window list in CI spells them (`5motor2hw10period_isr`), because the demangled spelling depends
on the binutils build while the mangled one is the linker's own.

Usage:
    tools/panic-reach.py [ELF]                 (default: the release image this repo builds)
    tools/panic-reach.py --dis FILE            (a saved/fixture disassembly instead of an ELF)
    tools/panic-reach.py --objdump BIN         (a specific disassembler)
    tools/panic-reach.py --verbose             (also print each root's closure size)
    tools/panic-reach.py --selftest            (run tools/tests/test_panic_reach.py; no toolchain)

Exit: 0 = no panic reachable, 1 = a FAILURE above, 2 = the gate could not run (no ELF, no objdump).
"""

import argparse
import bisect
import os
import re
import subprocess
import sys

TOOL = "panic-reach"

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ELF = os.path.join(
    os.path.dirname(_HERE), "target", "thumbv7m-none-eabi", "release", "firmware"
)
OBJDUMP_CANDIDATES = ("arm-none-eabi-objdump", "llvm-objdump", "rust-objdump", "objdump")

# ------------------------------------------------------------------------------------------------
# The roots. Each entry: (mangled-symbol pattern, why a panic there is unsafe rather than a halt).
# Adding a root is cheap; REMOVING one is a safety decision. A pattern that matches no function, or
# more than one, fails the gate.
# ------------------------------------------------------------------------------------------------
ROOTS = (
    (
        r"5motor2hw10period_isr",
        "the 16 kHz period ISR: a panic spins with MOE set and the last duties live",
    ),
    (
        r"8firmware15control_task_cb",
        "the 250 Hz control task: a panic stops the watchdog feed, so an armed bridge holds "
        "its duties for the whole 500 ms IWDG window",
    ),
    (
        r"8firmware13input_task_cb",
        "the 16 ms input task: a panic in it stops the pass and the watchdog feed, so neither the "
        "debounced power button nor the foot-pad field (separate banks, one task) advances again, "
        "and the pad field is what steps the balance gain schedule",
    ),
)

# ------------------------------------------------------------------------------------------------
# The panic set, named explicitly. Each entry: (mangled pattern, what it is).
#
# A long-branch thunk (`__Thumbv7ABSLongThunk__<target>`) carries its target's mangled name inside
# its own, so these patterns match the thunk too and the gate stops there rather than walking a
# thunk body whose `bx r12` would otherwise read as an unresolved indirect call.
# ------------------------------------------------------------------------------------------------
PANIC_SET = (
    (r"4core9panicking", "core::panicking::* (panic, panic_fmt, panic_bounds_check, ...)"),
    (r"4core6option13unwrap_failed", "Option::unwrap's failure arm"),
    (r"4core6result13unwrap_failed", "Result::unwrap/expect's failure arm"),
    (r"4core5slice5index16slice_index_fail", "slice index out of range"),
    (r"17len_mismatch_fail", "copy_from_slice length mismatch"),
    (r"rust_begin_unwind", "the #[panic_handler] itself (panic-halt: an infinite spin)"),
)

# ------------------------------------------------------------------------------------------------
# Named cuts: edges the closure deliberately does not cross. Each is
# (caller pattern, callee pattern, panic call sites hidden, why it is safe and what retires it).
#
# A cut names ONE caller and ONE callee, but that is not the same as being small: unless the callee
# is itself a panic symbol, a cut hides the callee's WHOLE SUBTREE, and everything that subtree can
# reach. Cut 1 below spans five `Store::get` monomorphizations and the fourteen functions behind
# them. So "named" is the honest word for a cut, not "narrow", and the third field is what keeps it
# accountable: it is the number of panic CALL SITES the cut hides, recounted on every run, and a
# change in that number FAILS. A panic added behind a cut is then a conscious list update, exactly
# like a root rename, instead of a silent widening of what CI has agreed not to look at.
#
# The other tripwire: a cut that matches no edge FAILS as stale. Both failures print, and every
# applied cut prints its edge count and its hidden-site count, so the allowances are a short list
# someone reads in CI output rather than a silence.
# ------------------------------------------------------------------------------------------------
CUTS = (
    (
        r"8firmware18re_read_arm_values",
        r"5store5store.*3get",
        # Recounted on every run and printed: the five `Store::get` monomorphizations' own range
        # index, `record::parse_header`'s, and `record::is_committed`'s two. The tool lists them
        # when the number moves, so this figure is checkable rather than asserted.
        8,
        "the arm-time store re-read: ARM step 1, which runs on the disarm->arm transition only "
        "and strictly BEFORE the MOE step (crates/firmware/src/arm.rs, `run_arm`). A panic there "
        "spins with the bridge still DISARMED and the IWDG resets the board with MOE off, which is "
        "a halt, not the energized-with-no-controller posture these roots exist to exclude. The "
        "same fact places it: `re_read_arm_values` is `#[inline(never)]` with no `.hotcode` "
        "section precisely because it is not hot path. specs/panic-free.md's non-goals keep the "
        "store's own panics out of scope, and the store being REACHABLE from a hot root is a "
        "boundary the spec did not anticipate, so this is a decision recorded here rather than a "
        "convenience: the panics are real, they are just in a context where the bridge is off. It "
        "stops applying, loudly, if the store read ever moves into the per-pass control path.",
    ),
)

# Anything in the image that LOOKS like a panic. Every text symbol matching this must be claimed by
# a PANIC_SET pattern, or the gate fails: an unrecognised panic symbol is a hole in the panic set,
# not something to ignore. Deliberately avoids matching ordinary vocabulary (`DebugMonitor` holds
# "bug"; `board::plumbing::BoardObs::failure` holds "fail", hence the anchored `_fail$`).
PANIC_LOOKING = re.compile(
    r"panic|unwind|unwrap_failed|expect_failed|_fail$|unreachable|assert_fail"
    r"|handle_alloc_error|abort"
)

# ------------------------------------------------------------------------------------------------
# Disassembly parsing
# ------------------------------------------------------------------------------------------------

_FUNC_HDR = re.compile(r"^([0-9a-fA-F]+)\s+<(.+)>:$")
_INSN = re.compile(r"^\s*([0-9a-fA-F]+):\s+(.*)$")
_TARGET = re.compile(r"<([^<>]+?)(?:\+0x[0-9a-fA-F]+)?>\s*$")
_COND = r"(?:eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|al)"
# Direct call / branch mnemonics (condition suffix and .n/.w width optional). `bl`/`blx` are calls;
# the rest are branches, which ARE edges: LLVM tail-calls with `b.w`, and a conditional branch can
# leave a function the same way.
# `bl`/`blx`, with the IT-block condition suffix (`bleq` is a CALL: `bl` + `eq`, not `ble` + `q`,
# and `_COND` has no one-letter entry, so the two spellings cannot collide). The image carries
# three of them, and a conditional call to a panic is still a call.
_CALL = re.compile(r"^bl(?:x)?" + _COND + r"?(?:\.[nw])?$")
_BRANCH = re.compile(r"^(?:b|cbn?z|tbb|tbh)" + _COND + r"?(?:\.[nw])?$")
# Indirect: a register-operand `blx`/`bx`, or anything that writes PC from a register or memory.
_IND_BLX = re.compile(r"^blx" + _COND + r"?(?:\.[nw])?$")
_IND_BX = re.compile(r"^bx" + _COND + r"?(?:\.[nw])?$")
_WRITES_PC = re.compile(
    r"^(?:mov|movs|ldr|ldr\.w|add|adds|sub|subs|orr|and|eor|lsl|lsr|asr)" + _COND + r"?(?:\.[nw])?$"
)
_REG = re.compile(r"^(?:r\d+|sl|fp|ip|sp|lr|pc)$")
_THUNK = re.compile(r"^__Thumbv7ABSLongThunk_(_.+)$")


class Function:
    __slots__ = ("start", "name", "end", "edges", "indirect")

    def __init__(self, start, name):
        self.start = start
        self.name = name
        self.end = start
        self.edges = []  # (target function index, call-site address, mnemonic)
        self.indirect = []  # (call-site address, the instruction text)

    def __repr__(self):  # pragma: no cover - debugging aid
        return f"<Function 0x{self.start:08x} {self.name}>"


class Program:
    """The parsed call graph: functions, resolved edges, and unresolved indirect call sites."""

    def __init__(self):
        self.funcs = []
        self.by_name = {}
        self.unresolved_targets = []  # (function index, site address, the raw target text)

    def index_of(self, addr):
        """The function containing `addr`, or None."""
        starts = self._starts
        i = bisect.bisect_right(starts, addr) - 1
        if i < 0:
            return None
        f = self.funcs[i]
        return i if f.start <= addr <= f.end else None

    def finish(self):
        self.funcs.sort(key=lambda f: f.start)
        self._starts = [f.start for f in self.funcs]
        self.by_name = {}
        for i, f in enumerate(self.funcs):
            # A duplicate name would make a root pattern ambiguous; keep every index so the root
            # resolution can say so instead of silently picking one.
            self.by_name.setdefault(f.name, []).append(i)


def parse_disassembly(text):
    """Parse `objdump -d --no-show-raw-insn` output into a `Program`.

    Instruction lines are read inside the function their header opened; branch and call targets are
    resolved by the symbol objdump prints in `<...>` and, failing that, by address. A target that
    resolves to neither is recorded as unresolved, which is a failure inside a closure for the same
    reason an indirect call is: the gate may not guess at an edge it cannot see.
    """
    prog = Program()
    cur = None
    pending = []  # (source Function, site addr, mnemonic, target name, raw operands)
    for line in text.splitlines():
        hdr = _FUNC_HDR.match(line)
        if hdr:
            cur = Function(int(hdr.group(1), 16), hdr.group(2))
            prog.funcs.append(cur)
            continue
        m = _INSN.match(line)
        if not m or cur is None:
            continue
        addr = int(m.group(1), 16)
        cur.end = max(cur.end, addr)
        body = m.group(2)
        # Drop objdump's trailing `@ ...` comment: it carries literal-pool values and the symbol a
        # `ldr` loads, neither of which is a branch target.
        body = re.split(r"\s@\s", body, maxsplit=1)[0].strip()
        if not body or body.startswith("."):
            continue  # `.word 0x...` and friends: data printed inside a code section
        parts = body.split(None, 1)
        mnem = parts[0]
        ops = parts[1].strip() if len(parts) > 1 else ""

        tgt = _TARGET.search(ops)
        first_op = ops.split(",", 1)[0].strip()
        reg_first = bool(_REG.match(first_op.lower()))

        # Indirect forms FIRST, by operand shape: `blx r3` is a call this disassembly cannot
        # resolve, not a direct call with a missing annotation. `bx lr` and `pop {.., pc}` are
        # returns.
        if (_IND_BLX.match(mnem) or _IND_BX.match(mnem)) and reg_first:
            if not (_IND_BX.match(mnem) and first_op.lower() == "lr"):
                cur.indirect.append((addr, f"{mnem} {ops}".strip()))
            continue
        if first_op.lower() == "pc" and _WRITES_PC.match(mnem):
            cur.indirect.append((addr, f"{mnem} {ops}".strip()))
            continue
        if mnem in ("tbb", "tbh"):
            # A PC-relative switch table: `tbb [pc, rN]`. The offsets are unsigned bytes relative
            # to the table, which sits in the function's own body, so this cannot leave it. Any
            # OTHER base register would be a real indirect branch.
            if not re.match(r"^\[\s*pc\b", ops):
                cur.indirect.append((addr, f"{mnem} {ops}".strip()))
            continue
        if _CALL.match(mnem) or _BRANCH.match(mnem):
            # Held as the Function OBJECT, not an index: `finish()` sorts by address and the
            # sections need not be dumped in address order. A direct call or branch with NO
            # `<symbol>` annotation still goes on the list, with no name: resolution falls back to
            # the address, and failing that it is recorded UNRESOLVED rather than dropped.
            # Dropping it would be a hole exactly where the gate is supposed to be loud, and only
            # `arm-none-eabi-objdump` is known to annotate every target: this tool also accepts
            # llvm-objdump, rust-objdump and plain objdump.
            pending.append((cur, addr, mnem, tgt.group(1) if tgt else None, ops))
            continue

    prog.finish()

    # Long-branch thunks: resolve by the name the thunk carries, and never walk the body (its
    # `bx r12` is the thunk mechanism, not an indirect call in the program).
    for i, f in enumerate(prog.funcs):
        t = _THUNK.match(f.name)
        if not t:
            continue
        f.indirect = []
        target = prog.by_name.get(t.group(1))
        if target:
            f.edges.append((target[0], f.start, "thunk"))
        else:
            prog.unresolved_targets.append((i, f.start, t.group(1)))

    where = {id(f): i for i, f in enumerate(prog.funcs)}
    for src, addr, mnem, tname, ops in pending:
        fidx = where[id(src)]
        hit = prog.by_name.get(tname) if tname is not None else None
        if hit is not None:
            tidx = hit[0]
        else:
            # objdump names the nearest preceding symbol; if that is not a function header we have
            # (an inter-section alias, a data symbol), fall back to the address.
            #
            # The operand is read as HEX, which is what every disassembler here prints. A
            # disassembler that printed a PC-relative DECIMAL immediate instead (`bl #131072`)
            # would be misread as hex, and what makes that loud rather than quiet is only the link
            # address: this image lives at 0x08000000 (crates/firmware/memory.x), so a small
            # decimal read as hex lands far below every function, resolves to nothing, and is
            # reported as an unresolvable target. A LOW link origin would put such a misread value
            # inside a real function instead, and the gate would follow a fabricated edge in
            # silence. The origin is fixed and is not moving, so this is a recorded property rather
            # than a guard: if it ever moves, the operand has to be parsed per disassembler, by
            # radix, before anything else here is trusted.
            addr_tgt = re.match(r"^([0-9a-fA-F]+)\s", ops) or re.match(r"^#?([0-9a-fA-F]+)$", ops)
            tidx = prog.index_of(int(addr_tgt.group(1), 16)) if addr_tgt else None
        if tidx is None:
            prog.unresolved_targets.append((fidx, addr, tname or f"{mnem} {ops}".strip()))
            continue
        if tidx != fidx:
            src.edges.append((tidx, addr, mnem))
    return prog


# ------------------------------------------------------------------------------------------------
# The gate
# ------------------------------------------------------------------------------------------------


class Report:
    def __init__(self):
        self.failures = []
        self.notes = []

    @property
    def ok(self):
        return not self.failures


def _claimed_by(name):
    for pat, what in PANIC_SET:
        if re.search(pat, name):
            return what
    return None


def analyze(text, roots=ROOTS, cuts=CUTS, verbose=False):
    """Run every property over one disassembly. Returns a `Report`."""
    rep = Report()
    prog = parse_disassembly(text)
    if not prog.funcs:
        rep.failures.append("the disassembly held no functions (wrong file, or objdump printed nothing)")
        return rep

    # Property 3: the panic set is checked against what the image actually contains.
    panic_idx = {}
    unclaimed = []
    for i, f in enumerate(prog.funcs):
        what = _claimed_by(f.name)
        if what is not None:
            panic_idx[i] = what
        elif PANIC_LOOKING.search(f.name):
            unclaimed.append(f.name)
    if unclaimed:
        rep.failures.append(
            "panic-looking symbol(s) no panic-set pattern claims (a new panic flavour must be "
            "named in PANIC_SET, not ignored):\n    " + "\n    ".join(sorted(unclaimed))
        )
    for pat, what in PANIC_SET:
        if not any(re.search(pat, f.name) for f in prog.funcs):
            rep.notes.append(f"panic-set pattern '{pat}' ({what}) matches nothing in this image")

    # The named cuts, resolved to concrete edges. Two tripwires: a cut that matches nothing is
    # stale, and a cut that hides a different number of panic call sites than it records has grown
    # (or shrunk) behind CI's back. Both FAIL.
    cut_edges = set()
    for caller, callee, hidden_recorded, why in cuts:
        matched = []
        for i, f in enumerate(prog.funcs):
            if not re.search(caller, f.name):
                continue
            for tidx, site, _mnem in f.edges:
                if re.search(callee, prog.funcs[tidx].name):
                    matched.append((i, tidx, site))
        if not matched:
            rep.failures.append(
                f"cut '{caller}' -> '{callee}' matches no edge in this image, so it is stale and "
                "must be deleted from CUTS consciously (the panic it excused may be gone, or the "
                f"call may have moved). Its reason was: {why}"
            )
            continue
        cut_edges.update((i, t) for (i, t, _s) in matched)
        hidden = _hidden_panic_sites(prog, panic_idx, matched)
        rep.notes.append(
            f"cut applied: {prog.funcs[matched[0][0]].name} -> "
            f"{prog.funcs[matched[0][1]].name} ({len(matched)} edge(s), hiding "
            f"{len(hidden)} panic call site(s))"
        )
        if verbose:
            for i, site, p in sorted(hidden, key=lambda h: h[1]):
                rep.notes.append(
                    f"  hidden by that cut: 0x{site:08x} in {prog.funcs[i].name} "
                    f"-> {prog.funcs[p].name}"
                )
        if len(hidden) != hidden_recorded:
            listing = "\n".join(
                f"    0x{site:08x} in {prog.funcs[i].name} -> {prog.funcs[p].name}"
                for (i, site, p) in sorted(hidden, key=lambda h: h[1])
            )
            rep.failures.append(
                f"cut '{caller}' -> '{callee}' now hides {len(hidden)} panic call site(s), not the "
                f"{hidden_recorded} it records. What a cut spans is as much a decision as that it "
                "exists: re-read the sites below, decide whether every one of them is still "
                "excusable for the cut's stated reason, and only then update the count.\n"
                f"{listing}\n    The cut's reason: {why}"
            )

    # Property 2: every root resolves, exactly once.
    resolved = []
    for pat, why in roots:
        hits = [i for i, f in enumerate(prog.funcs) if re.search(pat, f.name)]
        if not hits:
            rep.failures.append(
                f"root '{pat}' no longer resolves (inlined/renamed? update ROOTS consciously, the "
                f"hot-window gate's rule). It is {why}."
            )
        elif len(hits) > 1:
            rep.failures.append(
                f"root '{pat}' is ambiguous, it matches {len(hits)} functions: "
                + ", ".join(prog.funcs[i].name for i in hits[:4])
            )
        else:
            resolved.append((pat, why, hits[0]))

    # Properties 1 and 4: the closure, per root.
    for pat, why, root in resolved:
        parent = {root: None}
        order = [root]
        qi = 0
        while qi < len(order):
            cur = order[qi]
            qi += 1
            if cur in panic_idx and cur != root:
                continue  # a panic node is terminal: the path to it is the finding
            for tidx, site, mnem in prog.funcs[cur].edges:
                if (cur, tidx) in cut_edges:
                    continue
                if tidx not in parent:
                    parent[tidx] = (cur, site, mnem)
                    order.append(tidx)
        # Every panic node in the closure, and for each one EVERY call site inside the closure that
        # reaches it, not just the shortest path: fixing one site must not hide the next, so one run
        # names them all.
        for p in [i for i in order if i in panic_idx and i != root]:
            sites = [
                (i, site, mnem)
                for i in order
                if i != p
                for (t, site, mnem) in prog.funcs[i].edges
                if t == p and (i, t) not in cut_edges
            ]
            blocks = [
                f"    from 0x{site:08x} ({mnem}) in {prog.funcs[i].name}\n"
                + _render_path(prog, parent, i)
                for (i, site, mnem) in sites
            ]
            rep.failures.append(
                f"PANIC REACHABLE from root {prog.funcs[root].name}\n"
                f"    ({why})\n"
                f"    {panic_idx[p]}: {prog.funcs[p].name}\n"
                f"    {len(sites)} call site(s) inside the closure:\n" + "\n".join(blocks)
            )
        ind = [
            (prog.funcs[i].name, site, insn)
            for i in order
            for (site, insn) in prog.funcs[i].indirect
        ]
        if ind:
            rep.failures.append(
                f"unresolved indirect call(s) inside the closure of {prog.funcs[root].name}: a "
                "`blx`/`bx` through a register hides whatever it reaches, so the panic-free "
                "property cannot be proven past it. Make the call direct, or (if it is genuinely "
                "safe) hoist it out of the hot path.\n"
                + "\n".join(f"    0x{site:08x} in {n}: {insn}" for n, site, insn in ind)
            )
        bad = [
            (prog.funcs[fidx].name, site, t)
            for (fidx, site, t) in prog.unresolved_targets
            if fidx in parent
        ]
        if bad:
            rep.failures.append(
                f"unresolvable branch target(s) inside the closure of {prog.funcs[root].name}:\n"
                + "\n".join(f"    0x{site:08x} in {n} -> <{t}>" for n, site, t in bad)
            )
        if verbose:
            rep.notes.append(
                f"root {prog.funcs[root].name}: {len(order)} functions in its closure"
            )
    rep.notes.append(
        f"{len(prog.funcs)} functions, {sum(len(f.edges) for f in prog.funcs)} resolved edges, "
        f"{len(panic_idx)} panic symbols"
    )
    return rep


def _hidden_panic_sites(prog, panic_idx, matched):
    """Every panic call site a cut's edges hide: `(function index, call-site address)` pairs.

    A cut edge straight into a panic symbol hides exactly that one call site. Any other cut edge
    hides its callee's whole subtree, so the walk below collects every panic call site reachable
    from the callee. Other cuts are NOT applied inside the walk: a cut's span is what it hides on
    its own, not what is left after its neighbours have hidden their share.
    """
    sites = set()
    seen, stack = set(), []
    for i, t, site in matched:
        if t in panic_idx:
            sites.add((i, site, t))
        elif t not in seen:
            seen.add(t)
            stack.append(t)
    while stack:
        cur = stack.pop()
        for tidx, site, _mnem in prog.funcs[cur].edges:
            if tidx in panic_idx:
                sites.add((cur, site, tidx))
            elif tidx not in seen:
                seen.add(tidx)
                stack.append(tidx)
    return sites


def _render_path(prog, parent, node):
    """The shortest root-to-panic path, innermost call site per hop."""
    chain = []
    cur = node
    while cur is not None:
        p = parent[cur]
        chain.append((cur, p))
        cur = p[0] if p else None
    chain.reverse()
    out = []
    for depth, (idx, p) in enumerate(chain):
        pad = "    " + "  " * depth
        if p is None:
            out.append(f"{pad}{prog.funcs[idx].name}")
        else:
            out.append(f"{pad}-> {prog.funcs[idx].name}   [{p[2]} at 0x{p[1]:08x}]")
    return "\n".join(out)


# ------------------------------------------------------------------------------------------------
# Driver
# ------------------------------------------------------------------------------------------------


def find_objdump(explicit=None):
    cands = [explicit] if explicit else list(OBJDUMP_CANDIDATES)
    for c in cands:
        if os.path.isabs(c) and os.access(c, os.X_OK):
            return c
        for d in os.environ.get("PATH", "").split(os.pathsep):
            if d and os.access(os.path.join(d, c), os.X_OK):
                return os.path.join(d, c)
    return None


def disassemble(elf, objdump):
    out = subprocess.run(
        [objdump, "-d", "--no-show-raw-insn", elf],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if out.returncode != 0 or not out.stdout:
        sys.stderr.write(
            f"{TOOL}: {objdump} -d failed on {elf}: {out.stderr.decode(errors='replace').strip()}\n"
        )
        return None
    return out.stdout.decode(errors="replace")


def selftest():
    import unittest

    suite = unittest.defaultTestLoader.discover(
        os.path.join(_HERE, "tests"), pattern="test_panic_reach.py"
    )
    return 0 if unittest.TextTestRunner(verbosity=1).run(suite).wasSuccessful() else 1


def build_parser():
    ap = argparse.ArgumentParser(prog="panic-reach.py", add_help=True)
    ap.add_argument("elf", nargs="?", default=None, help=f"the ELF to read (default {DEFAULT_ELF})")
    ap.add_argument("--dis", default=None, help="read a saved disassembly instead of an ELF")
    ap.add_argument("--objdump", default=None, help="the disassembler to use")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    return ap


def main(argv=None):
    args = build_parser().parse_args(sys.argv[1:] if argv is None else argv)
    if args.selftest:
        return selftest()
    if args.dis:
        if not os.path.exists(args.dis):
            sys.stderr.write(f"{TOOL}: no such disassembly: {args.dis}\n")
            return 2
        with open(args.dis, "r") as fh:
            text = fh.read()
        source = args.dis
    else:
        elf = args.elf or DEFAULT_ELF
        if not os.path.exists(elf):
            sys.stderr.write(
                f"{TOOL}: no such ELF: {elf}\n{TOOL}: build it first (`cargo image`).\n"
            )
            return 2
        objdump = find_objdump(args.objdump)
        if objdump is None:
            sys.stderr.write(
                f"{TOOL}: no disassembler on PATH (tried {', '.join(OBJDUMP_CANDIDATES)}). This "
                "gate cannot run without one; install binutils-arm-none-eabi.\n"
            )
            return 2
        text = disassemble(elf, objdump)
        if text is None:
            return 2
        source = elf
    rep = analyze(text, verbose=args.verbose)
    print(f"{TOOL}: {source}")
    for n in rep.notes:
        print(f"{TOOL}: {n}")
    sys.stdout.flush()
    if rep.ok:
        print(
            f"{TOOL}: OK, no panic reachable from "
            + ", ".join(p for p, _ in ROOTS)
        )
        return 0
    for f in rep.failures:
        sys.stderr.write(f"{TOOL}: FAIL: {f}\n")
    sys.stderr.write(
        f"{TOOL}: FAIL: the motor hot paths must carry no reachable panic (specs/panic-free.md). "
        "A panic in the period ISR holds the bridge energized with no controller.\n"
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
