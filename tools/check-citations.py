#!/usr/bin/env python3
"""Check that cross-file citations in the Kotlin protocol mirror and the rider app resolve.

The mirror in protocol-kotlin/ documents, next to almost every declaration, the Rust it mirrors, and
the rider app does the same for the firmware behaviour its screens describe. Those citations name a
FILE and a SYMBOL (`crates/linkctl/src/lib.rs, CyclicState::LEN`) rather than a file and a line.
This script is what makes that claim true rather than aspirational.

Why it exists at all: a line number is falsified by any insertion above it, silently, from another
file. That happened 67 times in WireDriftTest.kt, and then thirteen more times in the very commit
that wrote the rule down. A symbol is different in kind - not because it cannot go stale, but because
staleness is DETECTABLE: the named file either declares that symbol or it does not. This script is
that detection. Without it the sweep swaps one unenforced convention for another, which is the class
of defect the sweep was carried out to remove.

Three checks, over every Kotlin file under SCOPE_ROOTS:

  1. Every citation resolves: the path exists and the file DECLARES the symbol. Declaration, not
     appearance: a `const`/`fn`/`struct`/`enum`/`static`/`type`/`mod`/`impl` of that name, a struct
     field, an enum variant, or (for a spec) a heading containing it. A bare mention in a comment
     does not count, and matching is word-boundary anchored, so `HEADER_LE` does not resolve against
     `HEADER_LEN`.

  2. No line-numbered citations come back. A rule is only worth stating if a regression fails.

  3. Nothing is silently unread. Any mention of a source path in a COMMENT that yields no
     recognised citation is reported. This is the check on the checker: the first version of this script required one exact
     word order, so it quietly saw nothing in `specs/l3.md`, "Discovery and address assignment" and
     in eleven other real citations, and reported a clean run over a file it had barely read. A
     checker that cannot say what it failed to parse is indistinguishable from one that passes.

Recognised citation forms, because prose reads better than one rigid order and all of these are in
use: `<path>, <Symbol>` / `<path>: <Symbol>` / `<Symbol> in <path>` / `<path>'s <Symbol>` /
`<path> (<Sym1>, <Sym2>)` / `<path> section <N>` (a numbered anchor inside a document, which is
file-level: see PATH_THEN_SECTION for why that is not the line-number form renamed) /
`<path>` followed by a parenthesised phrase naming no symbol (a file-level citation: the module doc,
its own diagram), which checks the path and stops there. Citations may wrap across comment lines.

WHAT IS READ. Comment text, and only comment text. The scope was two prose-heavy mirrors when this
was written and reading whole files cost nothing; it costs a great deal once the scope is every
Kotlin file, because a drift test NAMES the Rust it opens (`rust("crates/net/src/pdu.rs")`) and an
app names the files it loads. A path a program opens is not a claim about that file's contents, and
a rule that demanded it be rewritten as `<path>, <Symbol>` would be a rule about code. So each line
contributes its comment span and nothing else (see comment_span), and the line-number check still
runs over the raw text, where a `:238` in code would be just as wrong.

EXEMPTION, and there is exactly one: paths under `specs/`. That directory is gitignored (.gitignore),
so it exists in the primary checkout and nowhere else - not in a worktree, not in a clean clone, not
on CI. Citations into it are skipped with a note rather than failed, because failing would mean this
check could only ever pass on one machine. A specs/ citation still has to be WRITTEN as a citation,
in one of the forms below, or check 3 reports it: the exemption is about resolving the path, not
about naming one. Nothing else is exempt: nowhere in scope is a line number permitted, including in
a citation into a frozen external source (the EFeru dump, the Declassyfied decompile). The mirror
headers used to carve that second exemption out in prose while this script rejected it, and no file
has ever cited one; an exemption with no user is a rule kept true for nobody, so the sentence is
gone rather than the check. If one is ever needed, it arrives with the code that permits it, in the
same commit.

WHICH PATHS COUNT. A citation is a path ending .rs or .md (what these files mirror and cite), or .c
or .h. A path with no directory in it (`walk.rs`) is a short form, resolved against the same file's
other citations, so the module doc spells the path once and the KDoc below it need not. The last two exist so the form the rule FORBIDS fails loudly instead of passing silently: a
`bldc/BLDC_controller.c` citation is now seen, and reported as unresolvable, where before it was
invisible to the path scanner and slipped through unchecked whatever it claimed. `.kt` is
deliberately NOT a citation extension: these files are Kotlin and name Kotlin files while talking
about themselves ("WireDriftTest.kt carried 67 of them"), which is prose, not a claim about another
file's contents. A `.kt:<line>` still fails the line-number check, which is where that form would do
harm.

Usage:  tools/check-citations.py            (exit 0 = every citation resolves)
        tools/check-citations.py --verbose  (also list what was checked)
"""

import itertools
import os
import re
import sys

# The scope is DISCOVERED, not listed. It was a hand-written list of nine files, which made the rule
# opt-in: a tenth mirror file, or the whole rider app, could carry a `file:line` citation and no gate
# would look at it, which is exactly what `model/RiderCommand.kt` did (it cited a field that had not
# existed for months, in the present tense, by line number). Discovery means a file added to either
# tree is inside the rule the moment it exists.
#
# Both trees state the rule and are checked the same way. `apps/rider` is not a mirror of the Rust,
# but it makes the same kind of claim about it (what a board does with a value, when a value takes
# effect), and a claim about another file is a citation whoever writes it.
SCOPE_ROOTS = ("protocol-kotlin/src", "apps/rider")

# Files inside those roots that the rule does not reach. Empty, and a name goes in here only with
# the reason beside it; a growing list is the hand-written SCOPE coming back through the side door.
EXCLUDED: dict[str, str] = {}


def scope():
    """Every Kotlin file under SCOPE_ROOTS, minus EXCLUDED and minus build output.

    `build/` is skipped because it holds generated Kotlin that nobody writes citations in and that
    is absent on a clean checkout, so including it would make the result depend on whether Gradle
    had run.
    """
    out = []
    for root in SCOPE_ROOTS:
        for dirpath, dirnames, names in os.walk(root):
            dirnames[:] = [d for d in dirnames if d != "build"]
            out += [os.path.join(dirpath, n) for n in names if n.endswith(".kt")]
    found = sorted(p for p in out if p not in EXCLUDED)
    if not found:
        sys.exit(f"no Kotlin files under {SCOPE_ROOTS}: run this from the repository root")
    return found

# The trailing lookahead is not decoration: without it `.h` matches inside `com.hoverboard`, and
# every package line in both files reports as an unread citation. Check 3 caught that the moment the
# extension list grew, which is the check earning its keep on the checker's own change.
PATH = r"[A-Za-z0-9_./-]+\.(?:rs|md|c|h)(?![A-Za-z0-9_-])"
SYM = r"[A-Za-z_][A-Za-z0-9_]*(?:(?:::|\.)[A-Za-z_][A-Za-z0-9_]*)?"
Q = r"[`\"]?"

# path first: `crates/net/src/pdu.rs, HEADER_LEN`
PATH_THEN_SYM = re.compile(rf"{Q}({PATH}){Q}(?:'s)?\s*[,:]\s*{Q}({SYM}){Q}")
# symbol first: `committed_lengths_pinned` in `crates/linkctl/src/lib.rs`. The symbol MUST be
# backticked here and the connector is only in/at: without that, ordinary prose ("Mirror of X",
# "the engage gate in X", "known-answer vectors from X") parses as a citation and fails against a
# symbol nobody claimed existed.
SYM_THEN_PATH = re.compile(rf"`({SYM})`\s+(?:in|at)\s+{Q}({PATH}){Q}")
# path then a parenthesised list: `crates/linkctl/src/lib.rs` (`OP_INPUTS`, `OP_DRIVE_CMD`)
PATH_THEN_PARENS = re.compile(rf"{Q}({PATH}){Q}\s*\(([^)]*)\)")
# path then its own possessive: `link.rs`'s `Transport`. The same claim as `<Symbol>` in `<path>`,
# written the way the sentence runs, and the symbol is checked the same way.
PATH_POSSESSIVE_SYM = re.compile(rf"{Q}({PATH}){Q}'s\s+{Q}({SYM}){Q}")
# path then a SECTION anchor: `specs/rider-ui.md` section 3.4, `specs/todo.md` part 5. A numbered
# anchor inside a document, which is a file-level citation here: the path is checked and the anchor
# is not, because the anchor belongs to a file this script is not allowed to read (every one in
# this repo is under specs/, which is gitignored and absent on CI).
#
# It is NOT the line-number form in another costume. A line number names a position that any
# insertion above it falsifies silently, from another file; a section number names a part of the
# document that the author moved deliberately if it moved at all, and the reader finds it by
# searching rather than by counting. The two cannot be confused by this pattern either: the number
# must be separated from the path by a space or a comma, so `pdu.rs:45` is not read here, and
# LINE_CITATION fails it wherever it appears.
PATH_THEN_SECTION = re.compile(
    rf"{Q}({PATH}){Q}(?:'s)?,?\s+(?:§\s*|(?:section|sections|part|check|gate|step)s?\s+)?\d+(?:\.\d+)*[a-z]?\b"
)
BACKTICKED = re.compile(rf"`({SYM})`")

LINE_CITATION = re.compile(r"[A-Za-z0-9_./-]+\.(?:rs|kt|md|c|h):\d+")

# The SAME rule, for a line citation written with the path left implicit: `` `:238,241` ``. The
# pattern above requires a path immediately before the colon, so this shape escaped it, and the
# rule-stating file itself carried two of them, both already rotten (`:238,241` pointed inside
# `CyclicObs` and the `CyclicState` doc rather than at `Inputs`'s and `Fault`'s bit constants).
# A citation whose path is "the file this sentence already mentioned" is still a line citation, and
# it rots the same way; the backticks are what keep this from reading a time or a ratio in prose.
# Commas AND dashes, because `:238-241` is the same claim as `:238,241` written as a range, and it
# is matched over the FLATTENED body rather than the raw text for the same reason the path citations
# are: a citation wrapped across two comment lines is still a citation.
BARE_LINE_CITATION = re.compile(r"`:\s*\d+(?:\s*[,-]\s*\d+)*`")
PATH_MENTION = re.compile(PATH)

# A backticked Rust test / function name: snake_case with three or more segments. These get cited
# WITHOUT a path of their own ("pinned on the Rust side by `opcode_allocation_pinned`"), which is
# precisely the shape of the D1 defect this script was written after: `supervision_timeouts_pinned`
# was cited for a test actually called `supervision_constants_pinned`, and the first version of this
# script sailed past it because there was no path next to the name to resolve against. They are
# checked against the paths cited in the SAME doc block. Three segments is what keeps this precise:
# the Kotlin around them is camelCase, and single-word backticks (`flags`, `value`, `steer`) are
# field names and prose, so neither is dragged in.
RUST_FN_NAME = re.compile(r"`([a-z][a-z0-9]*(?:_[a-z0-9]+){2,})`")

# Words that follow a path but are English, not symbols: `pdu.rs, the module doc` and friends. A
# path followed by one of these is read as a FILE-level citation: the path is checked, and the
# sentence is taken at its word that it names no symbol.
PROSE = {
    "the", "its", "and", "with", "a", "an", "so", "named", "in", "at", "by", "see", "this", "that",
    "which", "where", "when", "then", "rs", "md", "kt", "line", "lines", "it", "one", "every",
    "each", "both", "all", "plus", "carrying", "holding", "whose", "what", "here", "there", "from",
    "for", "to", "of", "on", "as", "is", "are", "was", "no", "not", "but", "or", "if", "they",
    # Rust primitive types, which are never a declaration in the file being cited: `walk.rs`'s
    # `usize` capacities names the file and a TYPE, not a symbol that file declares.
    "usize", "u8", "u16", "u32", "u64", "i8", "i16", "i32", "i64", "bool", "str", "char", "f32",
}

SKIPPED_ROOTS = ("specs/",)


def flatten(text: str):
    """Strip comment furniture and join lines, keeping a char-offset -> line-number map.

    Citations wrap across lines in KDoc, so matching per line misses the wrapped ones (and one real
    citation was wrapped exactly that way). Matching over flattened text finds them; the map is what
    lets a failure still name the line a human should open.
    """
    flat, index, lineno, in_block = [], [], 1, False
    for line in text.splitlines(keepends=True):
        raw = line.rstrip("\n")
        start, end, in_block = comment_span(raw, in_block)
        body = re.sub(r"^\s*(?:/\*\*|\*/|\*|//!|///|//)?\s?", "", raw[start:end])
        flat.append(body + " ")
        index.extend([lineno] * (len(body) + 1))
        lineno += 1
    return "".join(flat), index


def comment_span(line: str, in_block: bool):
    """The (start, end) of the COMMENT text on one line, and the block state after it.

    Scanning the whole line was right while the scope was two prose-heavy mirrors and wrong the
    moment it grew: `RustSourceDriftTest` names a dozen Rust files as arguments (`rust("crates/net/
    src/pdu.rs")`), and the app names its own under test. A path a program OPENS is not a claim
    about that file's contents, and demanding it be rewritten as `<path>, <Symbol>` would be a rule
    about code, not about citations. A citation is prose, so prose is what is read.

    No string-literal awareness, which costs nothing in the direction that matters: a `//` inside a
    string makes this read the rest of that line as comment text, which can only ADD a mention to
    be checked, never hide one.
    """
    if in_block:
        end = line.find("*/")
        return (0, len(line), True) if end < 0 else (0, end, False)
    line_c, block_c = line.find("//"), line.find("/*")
    if block_c >= 0 and (line_c < 0 or block_c < line_c):
        end = line.find("*/", block_c + 2)
        return (block_c, len(line), True) if end < 0 else (block_c, end, False)
    if line_c >= 0:
        return line_c, len(line), False
    return 0, 0, False


def doc_blocks(text: str):
    """Yield (first-line-number, block text) for each run of consecutive comment lines."""
    lines = text.splitlines()
    start, buf = None, []
    for i, line in enumerate(lines, 1):
        if re.match(r"\s*(?:/\*\*|\*|//)", line):
            if start is None:
                start = i
            buf.append(re.sub(r"^\s*(?:/\*\*|\*/|\*|//!|///|//)?\s?", "", line))
        elif start is not None:
            yield start, " ".join(buf)
            start, buf = None, []
    if start is not None:
        yield start, " ".join(buf)


def declares(body: str, symbol: str, is_markdown: bool) -> bool:
    """Does `body` DECLARE `symbol`, rather than merely mention it?"""
    leaf = re.escape(re.split(r"::|\.", symbol)[-1])
    if is_markdown:
        return re.search(r"^#+ .*\b" + leaf + r"\b", body, re.M) is not None
    return any(
        re.search(p, body, re.M) is not None
        for p in (
            r"\b(?:fn|const|struct|enum|static|type|mod|impl)\s+" + leaf + r"\b",  # item
            r"^\s*(?:pub\s+)?" + leaf + r"\s*:",                                   # struct field
            r"^\s*" + leaf + r"\s*=\s*[^,;]+,",                                    # enum variant
        )
    )


def main() -> int:
    verbose = "--verbose" in sys.argv
    os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    failures: list[str] = []
    checked = skipped = 0

    for path in scope():
        if not os.path.isfile(path):
            failures.append(f"{path}: in SCOPE but not present")
            continue
        text = open(path, encoding="utf-8").read()
        flat, index = flatten(text)
        at = lambda pos: index[min(pos, len(index) - 1)]  # noqa: E731

        for hit in itertools.chain(
            ((m, text[: m.start()].count(chr(10)) + 1) for m in LINE_CITATION.finditer(text)),
            ((m, at(m.start())) for m in BARE_LINE_CITATION.finditer(flat)),
        ):
            hit, line = hit
            failures.append(
                f"{path}:{line}: line-numbered citation "
                f"`{hit.group(0)}`. This file's header states citations name a file and a SYMBOL; a "
                f"line number is falsified by any insertion above it, from any file, silently."
            )

        # (span, path, symbol-or-None). None = a file-level citation: check the path, nothing more.
        cites: list[tuple[tuple[int, int], str, str | None]] = []
        for m in PATH_THEN_SYM.finditer(flat):
            cites.append((m.span(), m.group(1), m.group(2)))
        for m in SYM_THEN_PATH.finditer(flat):
            cites.append((m.span(), m.group(2), m.group(1)))
        for m in PATH_THEN_PARENS.finditer(flat):
            syms = BACKTICKED.findall(m.group(2))
            cites.extend((m.span(), m.group(1), s) for s in syms) if syms else \
                cites.append((m.span(), m.group(1), None))
        for m in PATH_POSSESSIVE_SYM.finditer(flat):
            cites.append((m.span(), m.group(1), m.group(2)))
        for m in PATH_THEN_SECTION.finditer(flat):
            cites.append((m.span(), m.group(1), None))

        # A short-form mention (`walk.rs`) is the file named in full elsewhere in the same file.
        # The module doc spells `crates/net/src/walk.rs` once and the KDoc below it says `walk.rs`,
        # which is how these files are written and is worth keeping: the alternative is a
        # repo-relative path in the middle of every sentence. It is resolved rather than waved
        # through, so the SYMBOL beside it is still checked, and the short form has to be
        # unambiguous: zero or several files of that name cited in the file fails by name.
        by_base: dict[str, set[str]] = {}
        for p in PATH_MENTION.findall(flat):
            if "/" in p:
                by_base.setdefault(os.path.basename(p), set()).add(p)

        for span, cited, symbol in cites:
            line = at(span[0])
            if "/" not in cited:
                full = by_base.get(cited, set())
                if len(full) != 1:
                    failures.append(
                        f"{path}:{line}: names `{cited}`, which is a short form, but "
                        f"{len(full) or 'no'} file(s) of that name are cited in full anywhere in "
                        f"this file. Spell the repo-relative path here, or once above it."
                    )
                    continue
                cited = next(iter(full))
            if symbol is not None and symbol.lower() in PROSE:
                # An English word after the path, so the sentence names the FILE and no symbol in
                # it: `framer.rs, the module`. That is a file-level citation and is checked as one
                # (the path has to exist), rather than being dropped before the path is looked at,
                # which is what this did while the word list was the only thing standing between a
                # prose phrase and a fabricated symbol.
                symbol = None
            if cited.startswith(SKIPPED_ROOTS):
                skipped += 1
                if verbose:
                    print(f"  skip  {path}:{line}  {cited}, {symbol}  (specs/ is gitignored)")
                continue
            if not os.path.isfile(cited):
                failures.append(f"{path}:{line}: cites `{cited}` but that file does not exist")
                continue
            checked += 1
            if symbol is None:
                if verbose:
                    print(f"  ok    {path}:{line}  {cited}  (file-level)")
                continue
            if not declares(open(cited, encoding="utf-8", errors="ignore").read(), symbol,
                            cited.endswith(".md")):
                failures.append(
                    f"{path}:{line}: cites `{cited}, {symbol}` but {cited} declares no `{symbol}`. "
                    f"Rename it in the citation too, or cite what it became."
                )
            elif verbose:
                print(f"  ok    {path}:{line}  {cited}, {symbol}")

        # Bare Rust function/test names, resolved against the paths cited in their own doc block. A
        # block is a run of consecutive comment lines: the same paragraph a reader would take the
        # context from.
        for block_start, block in doc_blocks(text):
            paths = [p for p in PATH_MENTION.findall(block) if not p.startswith(SKIPPED_ROOTS)]
            paths = [p for p in dict.fromkeys(paths) if os.path.isfile(p)]
            for name in RUST_FN_NAME.findall(block):
                if not paths:
                    failures.append(
                        f"{path}:{block_start}: cites `{name}` but names no file in the same doc "
                        f"block, so there is nothing to check it against."
                    )
                    continue
                checked += 1
                if not any(declares(open(p, encoding="utf-8", errors="ignore").read(), name, False)
                           for p in paths):
                    failures.append(
                        f"{path}:{block_start}: cites `{name}`, but none of {', '.join(paths)} "
                        f"declares it."
                    )
                elif verbose:
                    print(f"  ok    {path}:{block_start}  {name}  (block-scoped)")

        covered = [s for s, _, _ in cites]
        for m in PATH_MENTION.finditer(flat):
            if not any(s[0] <= m.start() and m.end() <= s[1] for s in covered):
                failures.append(
                    f"{path}:{at(m.start())}: names `{m.group(0)}` but no citation this script can "
                    f"read, so it would go unchecked. Cite it as `<path>, <Symbol>`, `<Symbol> in "
                    f"<path>`, or `<path> (<Symbols>)`."
                )

    print(f"citations: {checked} resolved, {skipped} skipped (specs/, gitignored), {len(failures)} bad")
    for f in failures:
        print(f"FAIL {f}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
