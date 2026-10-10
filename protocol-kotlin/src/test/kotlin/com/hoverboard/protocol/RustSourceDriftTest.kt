package com.hoverboard.protocol

import com.hoverboard.protocol.l2.FragHdr
import com.hoverboard.protocol.l2.StreamFrame
import com.hoverboard.protocol.l3.BROADCAST
import com.hoverboard.protocol.l3.GUEST_FIRST
import com.hoverboard.protocol.l3.GUEST_LAST
import com.hoverboard.protocol.l3.HEADER_LEN
import com.hoverboard.protocol.l3.MAX_PDU
import com.hoverboard.protocol.l3.NO_ADDRESS
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.linkctl.CYCLIC_TIMEOUT_TICKS
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.linkctl.DRIVE_TIMEOUT_TICKS
import com.hoverboard.protocol.linkctl.DriveCmd
import com.hoverboard.protocol.linkctl.Fault
import com.hoverboard.protocol.linkctl.INPUTS_TIMEOUT_TICKS
import com.hoverboard.protocol.linkctl.Inputs
import com.hoverboard.protocol.linkctl.OP_CYCLIC_STATE
import com.hoverboard.protocol.linkctl.OP_DRIVE_CMD
import com.hoverboard.protocol.linkctl.OP_FAULT
import com.hoverboard.protocol.linkctl.OP_INPUTS
import com.hoverboard.protocol.board.BoardErrorKind
import com.hoverboard.protocol.board.BoardField
import com.hoverboard.protocol.board.BoardFields
import com.hoverboard.protocol.board.ChipFamily
import com.hoverboard.protocol.board.DEAD_TIME_MIN_DTG
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.McuFamily
import com.hoverboard.protocol.board.NET_PORT_BLE
import com.hoverboard.protocol.board.NET_PORT_UART
import com.hoverboard.protocol.board.PIN_ABSENT
import com.hoverboard.protocol.board.Pin
import com.hoverboard.protocol.board.SWD_PINS
import com.hoverboard.protocol.board.allowlistFor
import com.hoverboard.protocol.board.reservedSet
import com.hoverboard.protocol.board.validate
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import java.io.File

/**
 * The drift gate that actually reads the Rust.
 *
 * [WireDriftTest] pins the Kotlin against expected values that were copied out of the firmware by
 * hand. That catches a careless edit to the Kotlin, but on its own it is only half a gate: if the
 * FIRMWARE changes an opcode, the Kotlin and its hand-copied expectation still agree with each
 * other and every test stays green. The drift would surface on the bench, which is exactly the
 * outcome this module exists to prevent.
 *
 * So this file parses the Rust source in the same repository and compares it to the Kotlin. A
 * firmware change that outruns this mirror fails here.
 *
 * The comparisons are deliberately EXACT SET comparisons wherever the Rust enumerates something
 * (opcodes, type tags). A firmware change that ADDS an opcode fails too, rather than passing
 * quietly and leaving the Kotlin silently incomplete.
 *
 * That property is worth exactly as much as the SELECTORS are, and no more: an exact-set comparison
 * cannot demand a declaration whose regex never matched it, so a new constant written in a shape the
 * pattern misses is absent from BOTH sides and the sets agree on it. Every selector here therefore
 * treats the gaps between tokens as `\s`, and is scoped by something durable - a name prefix, the
 * declared type, or an enclosing block - rather than by exact spacing or a fixed indent. Each is
 * verified by injecting a declaration into the Rust in each shape and checking that the suite goes
 * red; where a shape is deliberately left out, the reason is stated at the selector. This is not a
 * hypothetical: the timeout and opcode selectors both matched a literal single space, so
 * `pub const  OP_FIFTH` and a two-space fourth timeout each passed this suite unmirrored.
 *
 * If the Rust is ever restructured enough that these regexes stop matching, this test fails loudly
 * with the pattern that missed rather than degrading into a no-op ([findAll] refuses an empty
 * match set). Note what that does and does not cover: it catches a pattern that stops matching
 * EVERYTHING, not one that stops matching one declaration out of several.
 *
 * ## The rule every selector here is written to
 *
 * **A drift gate asserts agreement, and silence is not agreement.** A selector that reads less of
 * the Rust than it appears to is not a weaker gate, it is the appearance of one: the item it cannot
 * see is absent from the Rust side, the mirror that never carried it is absent from the Kotlin
 * side, the two agree, and the suite is green across a real divergence. That is how an eight-byte
 * wire change passed here. So a selector that forms a SET is never trusted on its own output; it is
 * made to answer one question, and the answer has to be "it fails": what does this do when the Rust
 * grows a shape it has never seen?
 *
 * Three mechanisms answer it, and a new selector uses whichever of them fits rather than being
 * written more cleverly:
 *
 * - [findAll] refuses an empty match set, and [findOne] refuses an ambiguous one;
 * - [literal] refuses a value this gate cannot read, instead of a value pattern narrow enough to
 *   drop the declaration carrying it;
 * - [findAllClaiming] censuses the region the selector read, so an item written in a shape the
 *   selector does not recognise fails BY NAME rather than dropping out of the comparison. Where the
 *   items share a line and a census cannot see them (an array literal), the count the Rust declares
 *   (`[T; N]`, `IndexedField<_, N>`, `LEN`) is checked against the number read instead.
 *
 * And one rule about DIRECTION, which is the same defect a level up: **a census enumerates the side
 * that cannot be short.** A test that walks the Kotlin and checks each entry it finds confirms only
 * what the mirror already carries, so a field the Rust declares and the mirror lacks is never asked
 * about. `store::CONTROL_DRIVE_LEAN` was invisible to every gate here for that reason alone. Where
 * the Rust owns a SET, the Rust is enumerated and the mirror answers to it
 * ([everyRegisteredFieldReachesTheMirror]); where a mirrored value's owner is one named Rust
 * declaration, walking the mirror is right, because a name the Rust no longer declares fails
 * [findOne]. An exemption from such a census is derived from one side or the other, never written
 * out as a list of names to skip, which is this same defect in a smaller box.
 *
 * The 2026-10-10 sweep classified every selector in this file against that question. The ones that
 * could read LESS than they should were all of one kind, an enumeration whose other side is
 * hand-written: struct fields and members, enum variants and discriminants, match-arm tables, the
 * validator's take sites, the registered-field reads, and the allowlist entries. They now go
 * through [findAllClaiming] or a declared-count check, and each fix was proven by breaking the Rust
 * in the shape the selector was blind to and watching the named test fail. The ones left alone are
 * the by-name lookups (a named constant that disappears fails [findOne]) and the verbatim body pins
 * (a changed rule fails on its own text). Two narrowings stay deliberate and say so at the
 * selector: [U8_WIRE_CONST]'s column-0 anchor, and the `pub` requirement in [TIMEOUT_CONST] and
 * [TIMEOUT_DECL_LINE].
 *
 * What the sweep does NOT reach, stated so the next reader does not mistake green for complete:
 * every selector here enumerates DECLARATIONS, so a contract carried by an implementation the
 * Kotlin reimplements (`store::value::Value::decode`'s `STR` rule, `link::reasm::Reassembler`'s
 * capacity bound) is pinned by nothing in this file and is not pinned by its absence either.
 */
class RustSourceDriftTest {

    // --- locating and reading the Rust -----------------------------------------------------------

    private val repoRoot: File by lazy {
        var dir: File? = File(System.getProperty("user.dir")).absoluteFile
        while (dir != null && !File(dir, "crates/linkctl/src/lib.rs").isFile) dir = dir.parentFile
        checkNotNull(dir) {
            "Could not find the firmware repo root (no crates/linkctl/src/lib.rs above " +
                "${System.getProperty("user.dir")}). This module is the Kotlin mirror of that " +
                "source and its drift gate only works inside the firmware repository."
        }
        dir
    }

    private fun repoText(path: String): String {
        val f = File(repoRoot, path)
        check(f.isFile) { "Expected source at $path, relative to $repoRoot" }
        return f.readText()
    }

    private fun rust(path: String): String = repoText(path)

    /**
     * Source belonging to an in-tree CONSUMER of this protocol, read the same way the Rust is.
     *
     * The rider app is not built by this module (it needs the Android SDK) and cannot be depended
     * on from here (it depends on this module, not the other way round), but one of its numbers is
     * half of a contract this protocol states: the cadence it sends `INPUTS` on, against the window
     * the firmware holds that mirror for. Reading it as text is the same trick this whole file
     * rests on, and it fails loudly when the file moves rather than skipping.
     */
    private fun app(path: String): String = repoText(path)

    /** All matches of [pattern], failing loudly rather than silently returning nothing. */
    private fun findAll(text: String, pattern: String, what: String): List<MatchResult> {
        val hits = Regex(pattern, RegexOption.MULTILINE).findAll(text).toList()
        check(hits.isNotEmpty()) {
            "Pattern for $what matched nothing: /$pattern/. The Rust was probably restructured; " +
                "update this drift test rather than deleting it."
        }
        return hits
    }

    private fun findOne(text: String, pattern: String, what: String): MatchResult =
        findAll(text, pattern, what).single()

    /**
     * [findAll] plus the check [findAll] cannot make: that the pattern read EVERY item in the
     * region, not merely one of them.
     *
     * This is the fix for the defect class the `obs: Option<CyclicObs>` miss belongs to. A selector
     * that forms a SET and then compares it to a hand-written mirror is only as strong as its
     * weakest shape: an item the pattern does not recognise is absent from the Rust side, the
     * mirror that never carried it is absent from the Kotlin side, the two agree, and the gate is
     * green across a real divergence. Neither [findAll] nor an exact-set comparison can see that,
     * because both are looking at the pattern's output rather than at what the Rust holds.
     *
     * So the region is CENSUSED independently. [itemLine] is what an item looks like at its
     * loosest, and it is deliberately dumber than [pattern]: a field is "a name, a colon and
     * something after it", a variant is "a word at the start of a line", a match arm is "a line
     * with `=>` in it". Every line the census calls an item has to be claimed by some match, and
     * one that is not fails HERE, by name, with the line that was not read.
     *
     * The census is per LINE, which is its stated limit: an item sharing a line with a claimed one
     * (an array literal's elements) is invisible to it, and what covers those is the other half of
     * this file's discipline, checking a count the Rust declares (`[T; N]`, `IndexedField<_, N>`,
     * `REGISTRY_LEN`) against the number of items read.
     *
     * Exactly three kinds of line are skipped, and the list is this short on purpose: blank, one
     * beginning `//` (a line comment, so there is no item on it), and one beginning `#` or `*` (an
     * attribute, or the continuation of a block comment). A line that OPENS a block comment is not
     * skipped, and that is the whole of the direction this errs in. It used to be skipped, and the
     * cost was precise: a registry row carrying its id behind a one-line block-comment prefix was
     * skipped by the census AND unmatched by the selector, rustfmt does not reformat a macro body
     * so nothing normalised the shape away, and a registered, client-reachable, unmirrored field
     * passed the whole suite green.
     *
     * The price is a false-red surface, and it is worth naming in full rather than by its one
     * famous case. Five censuses look for their token ANYWHERE on the line, so a block comment
     * opening with that token in it is demanded as an item: [ARM_LINE] (any `=>`),
     * [FIELD_TAKE_LINE], [MOTOR_TAKE_LINE], [STORE_READ_LINE] and [REGISTRY_ROW_LINE]'s second
     * branch. A commented-out read inside `read_fields` fails
     * [theLayoutIsTheFieldSetTheFirmwareReads] exactly this way. The other four censuses
     * ([FIELD_LINE], [VARIANT_LINE], [TIMEOUT_DECL_LINE], [OPCODE_DECL_LINE]) anchor the shape at
     * the start of the line and cannot be reached by a comment opener at all. In every case the
     * `//` form is unaffected, so the escape hatch is one character, and the red that does happen
     * names the line and says so.
     */
    private fun findAllClaiming(
        text: String,
        pattern: String,
        itemLine: String,
        what: String,
    ): List<MatchResult> {
        val hits = findAll(text, pattern, what)
        val claimed = hits.flatMap { it.range }.toHashSet()
        val item = Regex(itemLine)
        val missed = mutableListOf<String>()
        var off = 0
        for (line in text.split("\n")) {
            val span = off..off + line.length
            off += line.length + 1
            val t = line.trim()
            if (t.isEmpty() || t.startsWith("//") || t.startsWith("#") || t.startsWith("*")) continue
            if (!item.containsMatchIn(line)) continue
            if (span.none { it in claimed }) missed += t
        }
        check(missed.isEmpty()) {
            // A line that OPENS a block comment is censused on purpose (see above), so it reaches
            // here two ways, and only one of them is answered by teaching the test a shape: if the
            // line is commented-out code, the answer is to comment it with `//`, which the census
            // skips. The message says which advice applies rather than giving one for both.
            val opener = missed.any { it.startsWith("/*") }
            "The selector for $what did not read ${missed.size} item(s) the census found: " +
                missed.joinToString(" | ") + ". /$pattern/ does not recognise that shape, so the " +
                "item drops out of this gate's comparison in silence and a mirror missing the same " +
                "item agrees with it. " +
                if (opener) {
                    "One of those lines opens a block comment. If it is a real item wearing a " +
                        "comment prefix, teach this test the shape; if it is commented-out code, " +
                        "write the comment as `//` and the census will skip it, which is the one " +
                        "form it is blind to by design."
                } else {
                    "Teach this test the shape rather than narrowing the census: a drift gate " +
                        "asserts agreement, and silence is not agreement."
                }
        }
        return hits
    }

    /**
     * The body of `enum <name> {` (with or without a visibility), up to the next column-0 close
     * brace, used by [rustVariants] and [discriminants].
     *
     * One extractor, because three tests each rolled their own `indexOf` + `substring` and two of
     * them checked neither end: a renamed enum reached `substring(-1, ...)` and failed with an
     * index out of bounds rather than with the enum it could not find.
     */
    private fun enumBlock(text: String, name: String): String {
        val m = Regex("""^(?:pub(?:\([^)]*\))?\s+)?enum $name \{""", RegexOption.MULTILINE).find(text)
        checkNotNull(m) { "No `enum $name {` (with or without a visibility) found" }
        val end = text.indexOf("\n}", m.range.first)
        check(end > m.range.first) { "Unterminated `enum $name`" }
        return text.substring(m.range.first, end)
    }

    /**
     * The `(field_id, handle name)` rows of a `field_ids!` block in crates/store/src/field.rs, in
     * declaration order. [nth] selects the block: 0 is the production registry, 1 the
     * `#[cfg(feature = "test-fields")]` copy.
     *
     * That macro list is the registry's authority rather than a doc: it expands to `FIELD_IDS`,
     * which a const assertion runs the uniqueness check over at build time. The handle name lives
     * in each row's trailing comment, which is what makes a failure here nameable, and the row's id
     * is checked against the handle's own declared id by
     * [everyRegisteredFieldReachesTheMirror], since the two are written separately.
     */
    private fun registryRows(text: String, nth: Int): List<Pair<Int, String>> {
        val blocks = Regex("""^field_ids!\s*\{""", RegexOption.MULTILINE).findAll(text).toList()
        check(blocks.size == 2) {
            "crates/store/src/field.rs declares ${blocks.size} `field_ids!` blocks, not 2: this gate " +
                "reads the production registry and the `test-fields` copy, and cannot tell which is " +
                "which otherwise"
        }
        val open = blocks[nth].range.last
        val end = text.indexOf("\n}", open)
        check(end > open) { "Unterminated `field_ids!` block" }
        // Every gap here is `[ \t]` and not `\s`, which is the opposite of this file's usual rule
        // and deliberate: `\s` crosses a newline, so a row with no trailing comment followed by a
        // comment LINE read that line's first word as the handle name and reported a fabricated
        // "the handle `the` that registry id 0x75 names". A row and its name are one line.
        return findAllClaiming(
            text.substring(open + 1, end),
            """^[ \t]*(0x[0-9A-Fa-f]+)[ \t]*,[ \t]*//[ \t]*(\w+)""",
            REGISTRY_ROW_LINE,
            "registry id rows",
        ).map { literal("field_ids!", it.groupValues[1], "registry id") to it.groupValues[2] }
    }

    private fun num(s: String): Int =
        if (s.startsWith("0x") || s.startsWith("0X")) s.drop(2).toInt(16) else s.toInt()

    /**
     * [num] for a value that came out of the Rust and MUST be a plain literal, failing loudly with
     * the offending declaration when it is not.
     *
     * A pattern that quietly matches less than it should is the same defect as one that matches
     * nothing, which [findAll] already refuses. Taking only literal-valued consts in the value
     * pattern itself would let `pub const X: u8 = Y + 1;` slip past unpinned while every test
     * stayed green, so the patterns take ANY value and the ones that cannot be read land here.
     */
    private fun literal(name: String, value: String, what: String): Int {
        val v = value.trim()
        check(Regex("""^(0[xX][0-9A-Fa-f]+|\d+)$""").matches(v)) {
            "$what `$name` is not a literal in the Rust (`= $v;`), so this gate cannot pin it. " +
                "Teach this test to evaluate it (as the flag-bit and frame checks do for their " +
                "expressions) rather than narrowing the pattern to skip it."
        }
        return num(v)
    }

    /**
     * A Rust value that may NAME another constant rather than spell a number, reduced to the
     * number: `= PIN_ABSENT;` resolves through `pub const PIN_ABSENT: u8 = 0xFF;` in the same file,
     * and a plain literal passes through with its digit separators dropped.
     *
     * This is the resolution [literal]'s own failure message asks for, rather than the narrower
     * pattern it warns against: the gate reads the Rust's declaration of the name, so a change to
     * EITHER the field's default or the constant behind it still fails here. A name the file does
     * not declare fails [findOne] loudly.
     */
    private fun constOrLiteral(text: String, owner: String, raw: String): String {
        val v = raw.trim()
        if (!Regex("""^[A-Z][A-Z0-9_]*$""").matches(v)) return v.replace("_", "")
        val m = findOne(text, """^pub\s+const\s+$v\s*:\s*\w+\s*=\s*([^;]+);""", "`$owner`'s default `$v`")
        return m.groupValues[1].trim().replace("_", "")
    }

    /** The body of `impl <name> {` up to the next column-0 close brace. */
    private fun implBlock(text: String, name: String): String {
        val start = text.indexOf("impl $name {")
        check(start >= 0) { "No `impl $name {` block found" }
        val end = text.indexOf("\n}", start)
        check(end > start) { "Unterminated `impl $name` block" }
        return text.substring(start, end)
    }

    /** The body of `pub struct <name> {` up to the next column-0 close brace. */
    private fun structBlock(text: String, name: String): String {
        val start = text.indexOf("pub struct $name {")
        check(start >= 0) { "No `pub struct $name {` found" }
        val end = text.indexOf("\n}", start)
        check(end > start) { "Unterminated struct $name" }
        return text.substring(start, end)
    }

    /**
     * Declared field order of a struct, as (name, rustType) pairs, doc comments skipped.
     *
     * The TYPE pattern is everything up to the comma, not `\w+`, and that is the difference
     * between a gate and the appearance of one: `\w+` does not match `Option<CyclicObs>`, so when
     * `CyclicState` grew exactly that field (it was an `Option` then) the regex SKIPPED it, the
     * remaining seven fields still matched the mirror's seven, their widths still summed to the
     * committed 11, and the suite stayed green over an eight-byte wire change. Taking any type
     * means an unrecognised one reaches the widths map in
     * [payloadFieldOrderAgreesWithTheRustSource] and fails there by name, which is this file's own
     * rule (see [literal]): a pattern that quietly matches less than it should is the same defect
     * as one that matches nothing.
     */
    private fun fields(text: String, name: String): List<Pair<String, String>> =
        findAllClaiming(
            structBlock(text, name),
            """^\s+pub\s+(\w+)\s*:\s*([^,]+),$""",
            FIELD_LINE,
            "$name fields",
        ).map { it.groupValues[1] to it.groupValues[2].trim() }

    /** [fields] with the names as the Kotlin mirrors spell them. */
    private fun members(text: String, name: String): List<Pair<String, String>> =
        fields(text, name).map { (f, t) -> snakeToCamel(f) to t }

    private fun snakeToCamel(s: String): String =
        s.split('_').mapIndexed { i, part -> if (i == 0) part else part.replaceFirstChar(Char::uppercase) }
            .joinToString("")

    private val linkctl by lazy { rust("crates/linkctl/src/lib.rs") }

    /**
     * Every supervision timeout the firmware declares, by name, read out of crates/linkctl.
     *
     * ONE selector, used by both tests that care (the exact-set mirror check and the keepalive
     * relation), so the two cannot come to disagree about which declarations count.
     *
     * The pattern is deliberately looser than the declarations it matches today: any type, including
     * a qualified one; any indentation, for a constant that moved inside a `mod`; and `pub`,
     * `pub(crate)`, `pub(super)` and `pub(in <path>)` alike. Every gap between tokens is `\s`-based
     * (`\s+` where a separator is required, `\s*` around `:` and `=`), and `\s` matches a newline and
     * a tab, so extra spacing or a wrap at any of those points is absorbed rather than escaped. The
     * escape that matters is a NEW timeout added in a shape this did not anticipate, because that one
     * falls outside the pattern, is never required of the mirror, and leaves the gate green while a
     * fourth supervision constant goes unmirrored. A shape change to one of the three ALREADY-mirrored
     * constants cannot hide either way: dropping out of the match set leaves the Kotlin carrying a key
     * the Rust does not, which the exact-set comparison fails on.
     *
     * That "escape" is now a FAILURE for every shape inside the naming scope, not a green: the
     * selector is censused against [TIMEOUT_DECL_LINE], so a public declaration naming a
     * `*_TIMEOUT_TICKS` constant that this pattern cannot read fails by name instead of going
     * unmirrored in silence. What is left outside is what that census also leaves out, and it is the
     * narrowing stated below: a declaration with no `pub`.
     *
     * That `\s` claim is load-bearing and was false when first written: the separator before the NAME
     * was a literal space, so `pub const  FOURTH_TIMEOUT_TICKS` (two spaces) and a name wrapped to the
     * next line both escaped, and an unmirrored fourth timeout in the firmware left this suite GREEN.
     * The shapes below are therefore checked against the GATE - inject the declaration into
     * crates/linkctl and run the suite - rather than against a reading of the regex.
     *
     * What escapes by DESIGN: a constant with no `pub` at all, or `pub(self)`, which is the same
     * visibility. Not because the mirror could not carry it - the mirror never links against the Rust,
     * it is a hand-copied `const val` whose value this gate re-derives by parsing text, so Rust
     * visibility constrains nothing and widening the alternation would capture a private declaration
     * happily. The reason is that a module-private constant is an internal detail of the crate, and
     * requiring the mirror to track one would fail this gate on a purely internal refactor that
     * changes nothing any consumer can observe.
     *
     * The pattern is NOT comment-aware, and the direction it errs in is the safe one. A declaration
     * commented out with `//`, or with a `/* ... */` that stays on one line, escapes, because the
     * comment opener sits between the line start and `pub`. One commented out with a `/* ... */`
     * spanning lines still MATCHES, and the gate then demands a mirror for a constant that does not
     * exist: a false red, loud and immediately explicable, which is the failure worth having.
     */
    private fun rustSupervisionTimeouts(): Map<String, Int> =
        findAllClaiming(linkctl, TIMEOUT_CONST, TIMEOUT_DECL_LINE, "supervision timeouts").associate {
            it.groupValues[1] to literal(it.groupValues[1], it.groupValues[2], "supervision timeout")
        }

    /**
     * The guard on [literal] itself, in the spirit of [findAll]'s: a gate that quietly reads LESS
     * of the Rust than it appears to is no gate. The exact-set patterns select constants by their
     * Rust TYPE and accept whatever value follows, so a value this test cannot read has to stop it
     * rather than fall outside a literal-only pattern and vanish from the comparison.
     */
    @Test
    fun aRustValueThisGateCannotReadFailsItRatherThanBeingSkipped() {
        assertEquals(0x2A, literal("SOME_CONST", " 0x2A ", "walk wire constant"))
        assertEquals(42, literal("SOME_CONST", "42", "walk wire constant"))

        val skipped = assertThrows(IllegalStateException::class.java) {
            literal("GUEST_LAST", "GUEST_FIRST + 0x7E", "L3 address constant")
        }
        assertTrue(
            skipped.message!!.contains("GUEST_LAST") && skipped.message!!.contains("not a literal"),
            "the failure must name the constant it could not read, got: ${skipped.message}",
        )
    }

    // --- opcodes ---------------------------------------------------------------------------------

    /**
     * Exact-set comparison against the `OP_*` consts in crates/linkctl/src/lib.rs. Adding a fifth
     * control family in the firmware fails this test until the Kotlin mirrors it.
     *
     * That sentence is the whole point of the test, so the selector has to earn it: the `OP_` name
     * prefix is what scopes the set, and everything around it is `\s`-based, so spacing, a wrap and
     * indentation inside a `mod` are absorbed rather than dropping the new constant out of the
     * comparison silently. It matched a single literal space before, and a fifth opcode written
     * `pub const  OP_FIFTH` passed this gate unmirrored.
     */
    @Test
    fun linkctlOpcodesAgreeWithTheRustSource() {
        val fromRust = findAllClaiming(linkctl, OPCODE_CONST, OPCODE_DECL_LINE, "linkctl opcodes")
            .associate { it.groupValues[1] to literal(it.groupValues[1], it.groupValues[2], "linkctl opcode") }

        val fromKotlin = mapOf(
            "CYCLIC_STATE" to OP_CYCLIC_STATE,
            "DRIVE_CMD" to OP_DRIVE_CMD,
            "INPUTS" to OP_INPUTS,
            "FAULT" to OP_FAULT,
        )
        assertEquals(fromRust, fromKotlin, "linkctl opcode allocation drifted from the Rust")
    }

    /** Exact-set comparison against the `Opcode` enum in crates/net/src/pdu.rs. */
    @Test
    fun l3OpcodesAgreeWithTheRustSource() {
        val fromRust = discriminants(rust("crates/net/src/pdu.rs"), "Opcode")
        val fromKotlin = Opcode.entries.associate { it.name to it.value }
        assertEquals(fromRust, fromKotlin, "L3 opcode table drifted from the Rust")
    }

    /**
     * Exact-set comparison against the `u8` address constants in crates/net/src/pdu.rs.
     *
     * The guest range lives there, not with the walk constants: the address space is L3's own model,
     * and `is_controller` is a predicate over the SAME range the grant allocator hands out from, so
     * the two cannot be allowed to live in different files and disagree. This mirror follows that
     * ownership, and the comparison is exact in both directions, so a new address constant on
     * either side fails until both carry it.
     */
    @Test
    fun l3AddressConstantsAgreeWithTheRustSource() {
        val fromRust = findAll(
            rust("crates/net/src/pdu.rs"),
            U8_WIRE_CONST,
            "L3 address constants",
        ).associate {
            it.groupValues[1] to literal(it.groupValues[1], it.groupValues[2], "L3 address constant")
        }

        val fromKotlin = mapOf(
            "BROADCAST" to BROADCAST,
            "NO_ADDRESS" to NO_ADDRESS,
            "GUEST_FIRST" to GUEST_FIRST,
            "GUEST_LAST" to GUEST_LAST,
        )
        assertEquals(fromRust, fromKotlin, "the L3 address constants drifted from the Rust")
    }

    /**
     * Exact-set comparison against the `u8` wire constants in crates/net/src/walk.rs.
     *
     * Every constant in the Kotlin [Walk] object is hand-copied from that file: the `NODE_HELLO`
     * kinds, the `PORTS` neighbour states and port media, `EGRESS_SELF`, the `ASSIGN_ACK` and
     * `CONFIG_RESP` statuses, and `PROTO_VER`. They were unpinned until now, which is how the R4
     * refusal status `CFG_ARMED` reached the firmware without ever reaching this mirror.
     *
     * Reading the Kotlin side by reflection makes the comparison exact in BOTH directions: a
     * constant added to the Rust fails until Kotlin mirrors it, and one added to Kotlin alone (or
     * left behind after the Rust drops it) fails too. `walk.rs`'s `usize` capacities (MAX_PORTS,
     * MAX_PDU, MAX_EMIT, MAX_NODES, MAX_TASKS) are firmware buffer sizing, not wire values, and are
     * not part of this set (MAX_PDU, the one a client must respect, is pinned on its own in
     * [maxPduAgreesWithTheRustSource]), so the pattern selects on the `u8` TYPE and takes whatever value
     * follows: an expression-valued one would otherwise fall outside a literal-only pattern and go
     * unpinned in silence. [literal] fails it loudly instead.
     *
     * [U8_WIRE_CONST] is that pattern, shared with the L3 address constants, and it states the one
     * shape this does not reach: a wire constant declared inside a nested module rather than at
     * module level.
     */
    @Test
    fun walkWireConstantsAgreeWithTheRustSource() {
        val fromRust = findAll(
            rust("crates/net/src/walk.rs"),
            U8_WIRE_CONST,
            "walk wire constants",
        ).associate {
            it.groupValues[1] to literal(it.groupValues[1], it.groupValues[2], "walk wire constant")
        }

        val fromKotlin = Walk::class.java.declaredFields
            .filter { it.type == Int::class.javaPrimitiveType }
            .associate { it.name to it.getInt(null) }
        check(fromKotlin.isNotEmpty()) { "No constants read out of the Kotlin Walk object" }

        assertEquals(fromRust, fromKotlin, "the L3 walk wire constants drifted from the Rust")
    }

    /**
     * `MAX_PDU`, the one `walk.rs` capacity a client must respect: it bounds what a board takes and
     * sends, so the app sizes a config value against it (`CONFIG_VALUE_MAX`). The other capacities
     * stay firmware-internal and unmirrored, as the test above says.
     */
    @Test
    fun maxPduAgreesWithTheRustSource() {
        val m = findOne(
            rust("crates/net/src/walk.rs"),
            """^\s*pub\s+const\s+MAX_PDU\s*:\s*usize\s*=\s*([^;]+);""",
            "walk.rs MAX_PDU",
        )
        assertEquals(literal("MAX_PDU", m.groupValues[1], "walk.rs capacity"), MAX_PDU, "MAX_PDU drifted")
    }

    /**
     * Exact-set comparison against `Type::tag` in crates/store/src/key.rs.
     *
     * Scoped to `tag()`'s own body: key.rs matches on `Type` in several places (`from_tag`, the
     * fixed-width table), and a value pattern loose enough to catch a non-literal tag would
     * otherwise drag those arms in too.
     */
    @Test
    fun storeTypeTagsAgreeWithTheRustSource() {
        val key = rust("crates/store/src/key.rs")
        val tagFn = key.substring(
            key.indexOf("pub const fn tag(self) -> u8 {").also {
                check(it >= 0) { "No `pub const fn tag(self) -> u8` in crates/store/src/key.rs" }
            },
        ).substringBefore("\n    }")

        val fromRust = findAllClaiming(
            tagFn,
            """^\s+Type::(\w+)\s*=>\s*([^,]+),""",
            ARM_LINE,
            "store type tags",
        ).associate { it.groupValues[1] to literal(it.groupValues[1], it.groupValues[2], "store type tag") }

        val fromKotlin = Type.entries.associate { it.name to it.tag }
        assertEquals(fromRust, fromKotlin, "store type tags drifted from the Rust")
    }

    // --- payload lengths and field order ---------------------------------------------------------

    /** `pub const LEN: usize = N;` inside each payload family's impl block. */
    @Test
    fun committedLengthsAgreeWithTheRustSource() {
        fun len(name: String): Int = num(
            findOne(implBlock(linkctl, name), """pub const LEN: usize = (\d+);""", "$name::LEN")
                .groupValues[1],
        )

        assertEquals(len("CyclicState"), CyclicState.LEN, "CyclicState::LEN drifted")
        assertEquals(len("DriveCmd"), DriveCmd.LEN, "DriveCmd::LEN drifted")
        assertEquals(len("Inputs"), Inputs.LEN, "Inputs::LEN drifted")
        assertEquals(len("Fault"), Fault.LEN, "Fault::LEN drifted")

        // The appended block's own length, and the payload's one on-wire length: the number above
        // is where the block starts, this one is what the wire carries.
        assertEquals(len("CyclicObs"), CyclicObs.LEN, "CyclicObs::LEN drifted")
        val encoded = num(
            findOne(
                implBlock(linkctl, "CyclicState"),
                """pub const ENCODED_LEN: usize = (\d+);""",
                "CyclicState::ENCODED_LEN",
            ).groupValues[1],
        )
        assertEquals(encoded, CyclicState.ENCODED_LEN, "CyclicState::ENCODED_LEN drifted")
        assertEquals(CyclicState.LEN + CyclicObs.LEN, CyclicState.ENCODED_LEN)
    }

    /**
     * The chip tags, as an exact set read out of the Rust enum: the wire vocabulary a client uses
     * to name the part it is talking to, so a tag added or renumbered in the firmware has to
     * reach this mirror before the app can mean anything by the byte.
     */
    @Test
    fun chipTagsAgreeWithTheRustSource() {
        assertEquals(
            discriminants(linkctl, "ChipTag"),
            ChipTag.entries.associate { it.name to it.value },
            "ChipTag drifted",
        )
    }

    /**
     * Field ORDER and width, read out of the Rust struct declarations.
     *
     * This is the pin that catches the exact break the retired rider protocol had: a `battery` u16
     * inserted mid-struct in CyclicState. Every field keeps its name and every length stays the
     * same under a reorder, so only order-aware comparison sees it.
     *
     * The widths also have to add up to the payload's own length, which is checked here so a
     * field that changes type cannot slip through, and an unrecognised Rust type fails by name
     * rather than being skipped (see [fields] for the eight-byte change that was skipped).
     */
    @Test
    fun payloadFieldOrderAgreesWithTheRustSource() {
        val widths = mapOf(
            "i16" to 2,
            "u16" to 2,
            "u8" to 1,
            "DriveKind" to 1,
            "ChipTag" to 1,
            // The appended block, as a field of the payload that carries it: it contributes its
            // own whole length, which the "CyclicObs" row below checks against its fields. NOT an
            // `Option` any more, and the type string is the gate: were the Rust to make it one
            // again, this map would not know the type and the comparison fails by name.
            "CyclicObs" to CyclicObs.LEN,
        )

        val expected = mapOf(
            "CyclicState" to listOf(
                "pitch" to "i16", "roll" to "i16", "wheelSpeed" to "i16", "battery" to "u16",
                "mode" to "u8", "fault" to "u8", "flags" to "u8", "obs" to "CyclicObs",
            ),
            "CyclicObs" to listOf(
                "phasePeak" to "i16", "phaseMean" to "i16", "dutyOn" to "u16",
                "bootTag" to "u8", "chip" to "ChipTag",
            ),
            "DriveCmd" to listOf("kind" to "DriveKind", "value" to "i16", "steer" to "i16"),
            "Inputs" to listOf("buttons" to "u8", "rider" to "u8"),
            "Fault" to listOf("code" to "u8", "action" to "u8"),
        )
        // The ENCODED length for the payload that has an appended block, because that is what its
        // fields add up to; the block's offset is checked by
        // [committedLengthsAgreeWithTheRustSource].
        val lens = mapOf(
            "CyclicState" to CyclicState.ENCODED_LEN,
            "CyclicObs" to CyclicObs.LEN,
            "DriveCmd" to DriveCmd.LEN,
            "Inputs" to Inputs.LEN,
            "Fault" to Fault.LEN,
        )

        for ((name, want) in expected) {
            val got = fields(linkctl, name).map { (f, t) -> snakeToCamel(f) to t }
            assertEquals(want, got, "$name field order/type drifted from the Rust struct")

            val total = got.sumOf { (_, t) -> widths[t] ?: error("unmapped Rust type $t in $name") }
            assertEquals(lens[name], total, "$name fields do not add up to its committed LEN")
        }
    }

    // --- flag bits and timeouts -------------------------------------------------------------------

    @Test
    fun flagAndActionConstantsAgreeWithTheRustSource() {
        fun bitConst(impl: String, name: String): Int {
            val body = implBlock(linkctl, impl)
            val m = Regex("""pub const $name: u8 = ([^;]+);""").find(body)
                ?: error("no `$name` in impl $impl")
            val expr = m.groupValues[1].trim()
            val shift = Regex("""1 << (\d+)""").find(expr)
            return if (shift != null) 1 shl shift.groupValues[1].toInt() else num(expr)
        }

        assertEquals(bitConst("CyclicState", "FLAG_RIDER"), CyclicState.FLAG_RIDER)
        assertEquals(bitConst("CyclicState", "FLAG_LOCKDOWN"), CyclicState.FLAG_LOCKDOWN)
        assertEquals(bitConst("Inputs", "BUTTON_POWER"), Inputs.BUTTON_POWER)
        assertEquals(bitConst("Inputs", "RIDER_PRESENT"), Inputs.RIDER_PRESENT)
        assertEquals(bitConst("Fault", "ACTION_NOTIFY"), Fault.ACTION_NOTIFY)
        assertEquals(bitConst("Fault", "ACTION_STOP_ALL"), Fault.ACTION_STOP_ALL)
    }

    /**
     * Exact-set comparison against every `*_TIMEOUT_TICKS` const in crates/linkctl/src/lib.rs.
     *
     * It used to look the timeouts up BY NAME, one assertion each, which is how
     * `INPUTS_TIMEOUT_TICKS` existed in the firmware for weeks with no mirror here and every test
     * green: a gate that only checks the constants it already knows about cannot notice a new one.
     * Selecting them by their NAME PATTERN and comparing as a set makes a fourth supervision
     * timeout fail this until the Kotlin carries it too.
     */
    @Test
    fun supervisionTimeoutsAgreeWithTheRustSource() {
        val fromRust = rustSupervisionTimeouts()

        val fromKotlin = mapOf(
            "CYCLIC_TIMEOUT_TICKS" to CYCLIC_TIMEOUT_TICKS,
            "DRIVE_TIMEOUT_TICKS" to DRIVE_TIMEOUT_TICKS,
            "INPUTS_TIMEOUT_TICKS" to INPUTS_TIMEOUT_TICKS,
        )
        assertEquals(fromRust, fromKotlin, "the supervision timeouts drifted from the Rust")
    }

    /**
     * The relation the mirrored VALUE alone cannot pin: the firmware's arm-mirror window has to
     * outlast the app's `INPUTS` keepalive period, with margin.
     *
     * Pinning [INPUTS_TIMEOUT_TICKS] against the Rust says nothing about the hazard, which is the
     * OTHER side moving: an app that halved its keepalive rate would leave every value pin green
     * and the arm dropping mid-ride, on a real machine with a rider on it. So this reads all three
     * numbers out of the sources that own them, none of them out of this mirror, and checks the
     * relation between them:
     *
     * - the window, from `crates/linkctl/src/lib.rs` (`INPUTS_TIMEOUT_TICKS`), in control ticks,
     *   through the same selector the exact-set mirror check uses. Reading the Rust rather than the
     *   mirrored Kotlin constant is deliberate: it keeps this test about the two artifacts that
     *   actually meet on the wire, the firmware and the app, rather than making it depend on
     *   another test having already proved the mirror faithful;
     * - the tick itself, from `crates/scheduler/src/lib.rs` (`TICK_HZ`), which is what turns ticks
     *   into milliseconds (`TICK_MS` is `1000 / TICK_HZ` there, an expression, so the rate is what
     *   gets read and the division is done here);
     * - the keepalive, from the rider app's `LinkConfig` (`SEND_INTERVAL_MS` x
     *   `INPUTS_KEEPALIVE_TICKS`), which is the app's send cadence for `INPUTS`.
     *
     * [KEEPALIVE_MARGIN] is the stated margin: the window must span at least that many keepalive
     * periods, so a lost keepalive (or two) is survivable and only a real silence disarms. At the
     * numbers this was written against (1,500 ms against 500 ms) the ratio is exactly 3.
     *
     * A failure here is not a test to relax. It means one of the two halves moved without the
     * other, and the fix is in the source that moved.
     */
    @Test
    fun theArmMirrorWindowOutlastsTheAppsKeepalive() {
        val tickHz = literal(
            "TICK_HZ",
            findOne(rust("crates/scheduler/src/lib.rs"), """^pub const TICK_HZ: u32 = ([^;]+);""", "TICK_HZ")
                .groupValues[1],
            "scheduler tick rate",
        )
        val windowTicks = rustSupervisionTimeouts()["INPUTS_TIMEOUT_TICKS"]
            ?: error(
                "no INPUTS_TIMEOUT_TICKS in crates/linkctl/src/lib.rs: the firmware's arm-mirror " +
                    "window is what this test measures the app's cadence against, so it cannot " +
                    "check anything without it.",
            )
        val windowMs = windowTicks * 1000 / tickHz

        val linkConfig = app("apps/rider/app/src/main/java/com/hoverboard/remote/ble/LinkConfig.kt")
        fun appConst(name: String, suffix: String) = literal(
            name,
            findOne(linkConfig, """^\s*const val $name: \w+ = (\d+)$suffix$""", "rider app $name")
                .groupValues[1],
            "rider app link config",
        )
        val keepaliveMs = appConst("SEND_INTERVAL_MS", "L") * appConst("INPUTS_KEEPALIVE_TICKS", "")

        assertTrue(
            windowMs >= keepaliveMs * KEEPALIVE_MARGIN,
            "the firmware holds a controller's arm mirror for ${windowMs}ms but the rider app only " +
                "re-sends INPUTS every ${keepaliveMs}ms; the window must span at least " +
                "$KEEPALIVE_MARGIN keepalive periods, so this build can disarm a board mid-ride on " +
                "ordinary frame loss. Move the timeout (crates/linkctl) or the cadence " +
                "(LinkConfig.SEND_INTERVAL_MS / INPUTS_KEEPALIVE_TICKS), not this margin.",
        )
    }

    /**
     * [DriveCmd.FULL_SCALE] is the one number in this mirror whose owner is NOT `linkctl`: the
     * demand word's scale is established by the frame-in adapter in the control crate, so that is
     * where this reads it from. Pinned here because a sender that gets it wrong commands a
     * thirty-third of what it meant to and the mistake is silent on the wire.
     */
    @Test
    fun theDriveDemandScaleAgreesWithTheRustSource() {
        val config = rust("crates/control/src/config.rs")
        // These live inside per-area modules rather than at the top level of the file, so unlike
        // the linkctl patterns above they cannot anchor the `pub` to column 0.
        fun c(name: String, ty: String) = literal(
            name,
            findOne(config, """^\s*pub const $name: $ty = ([^;]+);""", name).groupValues[1],
            "control config",
        )

        assertEquals(c("FRAME_IN_MAX", "i32"), DriveCmd.FULL_SCALE, "DriveCmd.FULL_SCALE drifted")

        // The two derived facts the Kotlin doc states about that scale, re-derived here so the
        // doc cannot rot: the frame-in truncation floor, and the smallest value that engages.
        val cmdLimit = c("CMD_LIMIT", "i16")
        val gate = c("GATING_THRESHOLD", "i16")
        val refNum = c("REF_SCALE_NUM", "i32")
        val refDen = c("REF_SCALE_DEN", "i32")

        // `|value| < FULL_SCALE / CMD_LIMIT` truncates to a zero command.
        assertEquals(33, DriveCmd.FULL_SCALE / cmdLimit + 1, "frame-in truncation floor drifted")

        // Smallest |value| whose reference clears the engagement gate from idle.
        val engageFloor = (1..DriveCmd.FULL_SCALE).first { v ->
            (v.toLong() * cmdLimit / DriveCmd.FULL_SCALE) * refNum / refDen > gate
        }
        assertEquals(590, engageFloor, "engagement floor drifted")
    }

    /**
     * The gain fields' IDs and per-index defaults, against the two `IndexedField` handles in
     * crates/store/src/field.rs, and their ranges against `GAIN_MIN` / `DEFAULT_GAIN_MAX` in
     * crates/control/src/config.rs and the `CONTROL_GAIN_MAX` field (`specs/rider-ui.md` section 4).
     *
     * Three separate drift risks, all silent on the wire and all pinned here: an id that moves
     * leaves a tune UI writing some OTHER field; a default that moves makes the app show a fresh
     * board the wrong number; a range that widens or narrows makes it refuse values the board would
     * take, or send ones it will not. The Rust owns all three -- the store owns ids and defaults,
     * the control seam owns ranges -- and the Kotlin only mirrors them.
     */
    @Test
    fun theGainFieldsAgreeWithTheRustSource() {
        val field = rust("crates/store/src/field.rs")

        // `pub const NAME: IndexedField<i16, 3> = IndexedField::new(0x71, [6000, 2000, 40]);`
        fun family(name: String): Pair<Int, List<Int>> {
            val m = findOne(
                field,
                """^pub const $name: IndexedField<\w+, (\d+)> = IndexedField::new\(([^,]+), \[([^\]]+)\]\);""",
                name,
            )
            val declared = m.groupValues[1].toInt()
            val id = literal(name, m.groupValues[2], "gain field id")
            val defaults = m.groupValues[3].split(",").map { literal(name, it, "gain default") }
            check(defaults.size == declared) { "$name declares $declared indices but lists ${defaults.size} defaults" }
            return id to defaults
        }

        val (idA, defA) = family("CONTROL_GAIN_A")
        val (idB, defB) = family("CONTROL_GAIN_B")
        assertEquals(idA, Gains.CONTROL_GAIN_A, "CONTROL_GAIN_A id drifted")
        assertEquals(idB, Gains.CONTROL_GAIN_B, "CONTROL_GAIN_B id drifted")
        assertEquals(defA, Gains.DEFAULT_A, "profile A defaults drifted")
        assertEquals(defB, Gains.DEFAULT_B, "profile B defaults drifted")
        assertEquals(defA.size, Gains.PER_PROFILE)
        assertEquals(defB.size, Gains.PER_PROFILE)

        // `pub const GAIN_MIN: i16 = 0;` and
        // `pub const DEFAULT_GAIN_MAX: [i16; GAINS_PER_PROFILE] = [20000, 10000, 1000];`
        val config = rust("crates/control/src/config.rs")
        val min = findOne(config, """^pub const GAIN_MIN: i16 = ([^;]+);""", "GAIN_MIN").groupValues[1]
        assertEquals(literal("GAIN_MIN", min, "gain minimum"), Gains.MIN, "the tune seam's minimum drifted")
        val maxSrc = findOne(
            config,
            """^pub const DEFAULT_GAIN_MAX: \[i16; \w+\] = \[([^\]]+)\];""",
            "DEFAULT_GAIN_MAX",
        ).groupValues[1]
        val maxima = maxSrc.split(",").map { literal("DEFAULT_GAIN_MAX", it.trim().replace("_", ""), "gain maximum") }
        assertEquals(maxima, Gains.DEFAULT_MAX, "the tune seam's default maxima drifted")
        assertEquals(Gains.PER_PROFILE, Gains.DEFAULT_MAX.size)
        // The store field that carries the maxima defaults to exactly them (its id and defaults are
        // pinned against field.rs by the Fields.INDEXED loop).
        val (idMax, defMax) = family("CONTROL_GAIN_MAX")
        assertEquals(0x74, idMax)
        assertEquals(defMax, Gains.DEFAULT_MAX, "CONTROL_GAIN_MAX defaults drifted from the seam's")
        assertEquals(Gains.DEFAULT_MAX.map { Value.I16(it) }, Fields.CONTROL_GAIN_MAX.defaults)

        // The index names the Kotlin exposes are the positions the Rust triple is written in, and
        // every default is inside its own range (a default a client would refuse to send is a bug
        // in one of the two files this test reads).
        assertEquals(listOf(Gains.KP, Gains.BK, Gains.PR), listOf(0, 1, 2))
        for (i in 0 until Gains.PER_PROFILE) {
            assertTrue(Gains.inRange(i, Gains.DEFAULT_A[i]), "profile A default $i is outside its range")
            assertTrue(Gains.inRange(i, Gains.DEFAULT_B[i]), "profile B default $i is outside its range")
        }
    }

    /**
     * The live-gain ramp's cap derivation (`specs/rider-ui.md` section 4, `control::config::ramp`),
     * which [Gains.Ramp] mirrors so a tune screen can bound how long the engaged loop may still
     * differ from what it staged. A drift here makes that bound wrong in the direction that matters:
     * a smaller share or a larger bound in the Rust slows the ramp, and a mark that cleared on the
     * old numbers would claim convergence before it happened.
     */
    @Test
    fun theGainRampCapsAgreeWithTheRustSource() {
        val config = rust("crates/control/src/config.rs")
        fun c(name: String, ty: String = "i32"): Int {
            val raw = findOne(config, """^\s+pub const $name: $ty = ([^;]+);""", name).groupValues[1]
            return literal(name, raw.replace("_", ""), "ramp constant")
        }
        assertEquals(c("KP_SHARE"), Gains.Ramp.KP_SHARE, "KP_SHARE drifted")
        assertEquals(c("BK_SHARE"), Gains.Ramp.BK_SHARE, "BK_SHARE drifted")
        assertEquals(c("BV_BOUND"), Gains.Ramp.BV_BOUND, "BV_BOUND drifted")
        assertEquals(c("PROP_DIVISOR"), Gains.Ramp.PROP_DIVISOR, "PROP_DIVISOR drifted")
        assertEquals(c("BATT_DIVISOR"), Gains.Ramp.BATT_DIVISOR, "BATT_DIVISOR drifted")
        assertEquals(c("RAW_NUMERATOR"), Gains.Ramp.RAW_NUMERATOR, "RAW_NUMERATOR drifted")
        // PP_BOUND is the FSM's upright window by name; follow the name to its literal.
        findOne(config, """^\s+pub const PP_BOUND: i32 = fsm::UPRIGHT_LIMIT;""", "PP_BOUND")
        assertEquals(c("UPRIGHT_LIMIT"), Gains.Ramp.PP_BOUND, "PP_BOUND (fsm::UPRIGHT_LIMIT) drifted")

        // The formula: each cap is `share * divisor / (RAW_NUMERATOR * bound)` of the scale, kp and
        // bk with these operands, and the step is floored at 1 count.
        findOne(config, """^\s+const KP_FRAC: \(u32, u32\) = reduced\(KP_SHARE, pid::PROP_DIVISOR, PP_BOUND\);""", "KP_FRAC")
        findOne(config, """^\s+const BK_FRAC: \(u32, u32\) = reduced\(BK_SHARE, pid::BATT_DIVISOR, BV_BOUND\);""", "BK_FRAC")
        findOne(config, """^\s+let num = \(share \* divisor\) as u32;""", "reduced numerator")
        findOne(config, """^\s+let den = \(pid::RAW_NUMERATOR \* bound\) as u32;""", "reduced denominator")
        findOne(config, """^\s+let kp = s \* KP_FRAC\.0 / KP_FRAC\.1;""", "kp cap")
        findOne(config, """^\s+let bk = s \* BK_FRAC\.0 / BK_FRAC\.1;""", "bk cap")
        findOne(config, """^\s+let c = if cap == 0 \{\s*\n\s+1""", "the floor of one count")

        // The pass rate the bound counts passes in.
        val hz = findOne(rust("crates/scheduler/src/lib.rs"), """^pub const TICK_HZ: u32 = ([^;]+);""", "TICK_HZ")
        assertEquals(literal("TICK_HZ", hz.groupValues[1], "tick rate"), Gains.Ramp.PASS_HZ, "TICK_HZ drifted")

        // The Rust's own worked numbers (the ramp module doc): kp 4, bk 3 at a word of 2400; 6 and 4
        // at 3300.
        assertEquals(listOf(4L, 3L, 6L, 4L), listOf(Gains.Ramp.step(Gains.KP, 2400), Gains.Ramp.step(Gains.BK, 2400), Gains.Ramp.step(Gains.KP, 3300), Gains.Ramp.step(Gains.BK, 3300)))
    }

    /**
     * Every field the Rust registry declares reaches this mirror, enumerated FROM THE REGISTRY.
     *
     * [theSetupFieldsAgreeWithTheRustSource] walks the Kotlin `Fields` map and checks each entry it
     * finds against the Rust. That direction can only ever confirm what the mirror already carries:
     * a field in the registry and absent from the mirror is never asked about, which is the
     * [findAllClaiming] defect one level up, between the two sides rather than inside one region.
     * `store::CONTROL_DRIVE_LEAN` (0x73) was exactly that for as long as it existed: live,
     * consumed at the boot seam (`crates/control/src/drive.rs`, `DriveLean`), mirrored nowhere, and
     * invisible to every gate in this file.
     *
     * So the authority is the side that cannot be short. The registry is enumerated, each row's id
     * is checked against the id its own handle declares (the macro list and the handle are written
     * separately, so a row naming the wrong field fails here), and a row nothing mirrors fails BY
     * NAME.
     *
     * Two exemptions, and both are DERIVED rather than listed, because a list of names to skip is
     * the same defect again in a smaller box:
     *
     * - a handle declared as a `BlobField` is exempt. `Value` has no blob case a settings client
     *   can edit, so what the exemption follows is the Rust's choice of handle type. In the
     *   production registry it covers exactly one row, `SOME_BLOB` at 0x30, which is a real
     *   registered field that no client exercises and whose own Rust doc says so; the store-test
     *   `T_BLOB` is in the `test-fields` block, which this gate reads separately and does not
     *   require a mirror for;
     * - a row whose name is a `CONTROL_GAIN_*` constant of the Kotlin [Gains] object is mirrored
     *   THERE, which is where the gain families live, and its id is pinned by
     *   [theGainFieldsAgreeWithTheRustSource]. The names come out of `Gains` by reflection, so a
     *   family moved between mirrors changes this gate's answer without editing it, and the
     *   `CONTROL_GAIN_` prefix is what keeps the exemption to the gain families: `Gains` also holds
     *   the index names, the per-profile count and the range bounds, and every one of those would
     *   otherwise exempt a registry row that happened to share its name.
     *
     * A third kind of field, one deliberately not for clients, has no marker in the Rust and so is
     * not exempt: it fails here, and the answer is to mirror it or to teach this gate why its
     * family is elsewhere. That is the same "teach the test" contract [literal] states.
     */
    @Test
    fun everyRegisteredFieldReachesTheMirror() {
        val field = rust("crates/store/src/field.rs")
        val registry = registryRows(field, 0)
        val withTestFields = registryRows(field, 1)

        val mirrored = Fields.ALL.mapValues { it.value.id } + Fields.INDEXED.mapValues { it.value.id }
        val inGains = Gains::class.java.declaredFields
            .filter { it.type == Int::class.javaPrimitiveType && it.name.startsWith("CONTROL_GAIN_") }
            .associate { it.name to it.getInt(null) }
        check(inGains.isNotEmpty()) { "No CONTROL_GAIN_* constants read out of the Kotlin Gains object" }

        for ((id, name) in registry) {
            val decl = findOne(
                field,
                """^pub\s+const\s+$name\s*:\s*(\w+)[^=]*=\s*\w+::new\(\s*([^,)]+)""",
                "the handle `$name` that registry id 0x${id.toString(16)} names",
            )
            assertEquals(
                id,
                literal(name, decl.groupValues[2], "handle id"),
                "the registry row for `$name` and its own handle declare different ids",
            )
            val mirroredId = mirrored[name] ?: inGains[name]
            if (mirroredId != null) {
                assertEquals(id, mirroredId, "`$name` is mirrored under a different id than the registry declares")
                continue
            }
            check(decl.groupValues[1] == "BlobField") {
                "`$name` (0x${id.toString(16)}) is in the Rust registry and NOTHING here mirrors it: " +
                    "neither Fields.ALL, nor Fields.INDEXED, nor Gains carries that name. A client " +
                    "cannot read, display or write a field it cannot name, and the gate that walks " +
                    "this mirror cannot notice the absence, which is why this test reads the registry " +
                    "instead. Mirror it where its family belongs, or teach this gate why it is not a " +
                    "client field."
            }
        }

        // The count the Rust declares for the same set, which is what this file's own rule says to
        // do wherever a line census cannot see an item: `REGISTRY_LEN` is `<declared fields> +
        // <extra per-index entries>`, and the first term is the field count. It is a SECOND
        // WITNESS: a field registered with its id row left out of `field_ids!` still COMPILES (the
        // array's type pins the total, not the rows), and the Rust's own
        // `registry_has_every_declared_field_with_its_handle_type_and_default` asserts the same
        // relation from inside the crate. This is the mirror side's independent reading of it, so
        // the list this gate treats as the registry is checked against the Rust's own count without
        // depending on the Rust suite having been run.
        fun declaredCount(cfg: String, what: String): Int = literal(
            "REGISTRY_LEN",
            findOne(
                field,
                """^#\[cfg\(""" + cfg +
                    """\)\]\s*pub\s+const\s+REGISTRY_LEN\s*:\s*usize\s*=\s*(\d+)\s*\+\s*\d+\s*;""",
                what + " REGISTRY_LEN",
            ).groupValues[1],
            "the declared field count",
        )
        assertEquals(
            declaredCount("not\\(feature = \"test-fields\"\\)", "the production"),
            registry.size,
            "crates/store/src/field.rs declares a field count its `field_ids!` rows do not add up to, " +
                "so either a registered field has no id row or this gate could not read one",
        )
        assertEquals(
            declaredCount("feature = \"test-fields\"", "the test-fields"),
            withTestFields.size,
            "the same, for the `test-fields` registry",
        )

        // The production list is the authority, so it has to BE the authority: the `test-fields`
        // copy may only add reserved `T_*` ids, or a field added to one list and not the other
        // would leave this gate reading the shorter one.
        val names = registry.map { it.second }.toSet()
        val added = withTestFields.map { it.second }.filterNot { it in names }
        assertTrue(
            added.all { it.startsWith("T_") },
            "the test-fields `field_ids!` block adds non-reserved fields $added, so the production " +
                "block this gate reads is not the whole registry",
        )
        assertEquals(
            emptyList<String>(),
            registry.map { it.second }.filterNot { n -> withTestFields.any { it.second == n } },
            "a field is in the production `field_ids!` block but missing from the test-fields copy",
        )
    }

    /**
     * The Setup screen's fields (`specs/rider-ui.md` section 3.4): each [Fields.ALL] entry's id,
     * storage type and default, against the `Field<T>` / `StrField` handle of the same name in
     * crates/store/src/field.rs.
     *
     * The same three silent drifts the gain pin guards: an id that moves makes the screen write some
     * OTHER field, a type that changes makes every write a `CFG_TYPE_MISMATCH`, and a default that
     * moves makes the screen describe a fresh board wrongly. Not an exact set: the Kotlin mirrors the
     * fields a client exercises (the pin block and the gains live elsewhere), and a name the Rust no
     * longer declares fails [findOne]. That the mirror is not SHORT is
     * [everyRegisteredFieldReachesTheMirror]'s job, which is the same comparison from the registry's
     * end; this one is about each entry's id, type and default.
     */
    @Test
    fun theSetupFieldsAgreeWithTheRustSource() {
        val field = rust("crates/store/src/field.rs")
        for ((name, def) in Fields.ALL) {
            // `pub const NAME: Field<u32> = Field::new(0x20, 10_000);` or `StrField::new(0x10, "Hoverboard")`
            val m = findOne(
                field,
                """^pub\s+const\s+$name\s*:\s*(Field\s*<\s*(\w+)\s*>|StrField)\s*=\s*\w+::new\(\s*([^,]+?)\s*,\s*(.+?)\s*\)\s*;""",
                name,
            )
            val rustType = m.groupValues[2].ifEmpty { "str" }
            val type = when (rustType) {
                "u8" -> Type.U8
                "u16" -> Type.U16
                "u32" -> Type.U32
                "i16" -> Type.I16
                "i32" -> Type.I32
                "str" -> Type.Str
                else -> error("$name has Rust type $rustType, which this gate does not map yet")
            }
            assertEquals(type, def.type, "$name storage type drifted")
            assertEquals(literal(name, m.groupValues[3], "field id"), def.id, "$name id drifted")
            val raw = m.groupValues[4]
            fun int() = literal(name, constOrLiteral(field, name, raw), "field default")
            val default = when (type) {
                Type.Str -> {
                    check(raw.startsWith("\"") && raw.endsWith("\"")) { "$name default is not a string literal: $raw" }
                    Value.Str(raw.substring(1, raw.length - 1))
                }
                Type.U8 -> Value.U8(int())
                Type.U16 -> Value.U16(int())
                Type.U32 -> Value.U32(int().toLong())
                Type.I16 -> Value.I16(int())
                Type.I32 -> Value.I32(int())
                else -> error("unreachable: $type")
            }
            assertEquals(default, def.default, "$name default drifted")
        }

        // `pub const NAME: IndexedField<i16, 2> = IndexedField::new(0x69, [25200, -5]);`
        for ((name, def) in Fields.INDEXED) {
            val m = findOne(
                field,
                """^pub\s+const\s+$name\s*:\s*IndexedField\s*<\s*(\w+)\s*,\s*(\d+)\s*>\s*=\s*IndexedField::new\(\s*([^,]+?)\s*,\s*\[([^\]]+)\]\s*\)\s*;""",
                name,
            )
            val type = when (m.groupValues[1]) {
                "i16" -> Type.I16
                else -> error("$name has Rust type ${m.groupValues[1]}, which this gate does not map yet")
            }
            assertEquals(type, def.type, "$name storage type drifted")
            assertEquals(literal(name, m.groupValues[3], "field id"), def.id, "$name id drifted")
            val defaults = m.groupValues[4].split(",").map { raw ->
                val t = raw.trim().replace("_", "")
                Value.I16(if (t.startsWith("-")) -literal(name, t.drop(1), "default") else literal(name, t, "default"))
            }
            assertEquals(m.groupValues[2].toInt(), defaults.size, "$name declares a count its defaults do not match")
            assertEquals(defaults, def.defaults, "$name defaults drifted")
        }
    }

    /**
     * The `Name = N,` discriminants of `enum <name> {`, by variant name.
     *
     * Censused on the SAME line rule as [rustVariants], which is the point: a variant that carries
     * no explicit discriminant (Rust gives it the previous one plus one) is a variant this reader
     * cannot report, and leaving it out quietly would make an "exact set" comparison agree with a
     * mirror that is also missing it. It now fails by name. The value is taken as anything up to
     * the comma and read by [literal], so a hex, separated or expression-valued discriminant fails
     * loudly too instead of falling outside a decimal-only pattern.
     */
    private fun discriminants(text: String, name: String): Map<String, Int> =
        findAllClaiming(
            enumBlock(text, name),
            """^\s+(\w+)\s*=\s*([^,]+),""",
            VARIANT_LINE,
            "$name discriminants",
        ).associate { it.groupValues[1] to literal(it.groupValues[1], it.groupValues[2], "$name discriminant") }

    /**
     * The byte vocabularies the Setup screen offers as choices, against the enums and lookup that
     * decode them on the board, and the clamp the firmware applies to the current limit. Exact sets:
     * a new method, mode or IMU model in the firmware fails here until the screen can offer it, and a
     * renumbered one fails before the screen writes the wrong byte.
     */
    @Test
    fun theSetupChoiceVocabulariesAgreeWithTheRustSource() {
        assertEquals(
            mapOf("Throttle" to Fields.ControlMode.THROTTLE, "Balance" to Fields.ControlMode.BALANCE),
            discriminants(rust("crates/control/src/mode.rs"), "ControlMode"),
            "CONTROL_MODE vocabulary drifted",
        )
        assertEquals(
            mapOf(
                "SixStep" to Fields.MotorMethod.SIX_STEP,
                "Sine" to Fields.MotorMethod.SINE,
                "Foc" to Fields.MotorMethod.FOC,
            ),
            discriminants(rust("crates/commutation/src/lib.rs"), "CommutationMethod"),
            "MOTOR_METHOD vocabulary drifted",
        )

        // `pub fn model_from_index(index: u8) -> Option<Model> { match index { 1 => Some(MPU6050), ... } }`
        val imu = rust("crates/imu/src/lib.rs")
        val start = imu.indexOf("pub fn model_from_index(")
        check(start >= 0) { "No `pub fn model_from_index(` found" }
        val body = imu.substring(start, imu.indexOf("\n}", start))
        val models = findAllClaiming(
            body,
            """^\s*([^=\s]+)\s*=>\s*Some\(\s*(\w+)\s*\)""",
            ARM_LINE,
            "model_from_index arms",
        ).associate { it.groupValues[2] to literal(it.groupValues[2], it.groupValues[1], "IMU model index") }
        assertEquals(
            mapOf("MPU6050" to Fields.ImuModel.MPU6050, "CLONE_2E" to Fields.ImuModel.CLONE_2E),
            models,
            "IMU_MODEL vocabulary drifted",
        )
        assertTrue(Fields.ImuModel.NONE !in models.values, "index 0 must stay 'no IMU fitted'")

        // The staged limit's clamp is NOT mirrored, and this is where that is recorded. The
        // firmware bounds the limit in COUNTS at both ends, because the counts-per-amp scale is
        // per-board data (`MOTOR_CURRENT_CAL`, 0x67; `specs/motor-integration.md`, "The current
        // limit"), so there is no milliamp constant left to drift from: a milliamp bound would be a
        // bound at one assumed scale, which is the defect the count window replaced.
        // `Fields.CURRENT_LIMIT_MA` is therefore the client's own editor range, and what is pinned
        // here is the ceiling (`MAX_LIMIT_COUNTS`), that the window keeps the hard trip's 2x
        // expressible, and the band the per-board FLOOR is clamped into.
        val motor = rust("crates/firmware/src/motor.rs")
        fun bound(name: String) = findOne(
            motor,
            """^pub\s+const\s+$name\s*:\s*i16\s*=\s*([^;]+);""",
            name,
        ).groupValues[1].trim()
        assertTrue(
            !motor.contains("CURRENT_LIMIT_CEILING_MA"),
            "a milliamp current-limit bound is back in the firmware: Fields.CURRENT_LIMIT_MA is this client's editor range, not a mirror of one",
        )
        // Evaluated rather than read as a literal, which is how the derivation stays in the Rust:
        // half the comparison's full scale is exactly the largest limit whose 2x hard trip fits.
        assertEquals("i16::MAX / 2", bound("MAX_LIMIT_COUNTS"), "MAX_LIMIT_COUNTS stopped being half the comparison's full scale")
        val maxLimit = Short.MAX_VALUE / 2
        assertTrue(2 * maxLimit <= Short.MAX_VALUE.toInt(), "the hard trip's 2x no longer fits the comparison")
        // The window's FLOOR is per-board now (`store::MOTOR_NOISE_FLOOR`, 0x6B), so what is left in
        // the Rust is the plausibility band the boot seam clamps a staged floor into, and that band
        // is what `Fields.NOISE_FLOOR` mirrors. Both ends are pinned: its bottom against
        // `NOISE_FLOOR_MIN` and its top against `MAX_LIMIT_COUNTS`, which is the SAME ceiling the 2x
        // above rests on. A floor clamped to at most that ceiling is exactly what keeps the hard
        // trip's doubling exact at every floor a board can claim, so a band that grew past it would
        // break the relation two assertions up, and this is where that shows.
        val floorMin = literal("NOISE_FLOOR_MIN", bound("NOISE_FLOOR_MIN").replace("_", ""), "noise-floor band bottom")
        assertTrue(floorMin < maxLimit, "the noise floor's plausibility band is empty")
        assertEquals(
            floorMin.toLong()..maxLimit.toLong(),
            Fields.NOISE_FLOOR,
            "the noise floor's boot-seam band drifted from the mirror's",
        )
        assertTrue(
            !motor.contains("MIN_LIMIT_COUNTS"),
            "a compiled limit floor is back in the firmware: the floor is per-board (store::MOTOR_NOISE_FLOOR, 0x6B)",
        )
        // The registered default is inside the band, so an unstaged board needs no clamping, and it
        // IS the constant the field replaced (the id/type/default are pinned by the Setup-field gate).
        assertTrue(
            (Fields.MOTOR_NOISE_FLOOR.default as Value.U16).v.toLong() in Fields.NOISE_FLOOR,
            "MOTOR_NOISE_FLOOR's default is outside the band the firmware clamps it into",
        )
        // The seam that applies the band, read as text: a client reporting what a board will enforce
        // clamps what it read the same way, so a clamp that stopped happening would make every such
        // report a guess.
        findOne(
            motor,
            """stored\.clamp\(NOISE_FLOOR_MIN as u16, MAX_LIMIT_COUNTS as u16\)""",
            "noise_floor_counts' band clamp",
        )

        // `CONTROL_DRIVE_LEAN`'s two boot clamps are NOT mirrored, and this is where that is
        // recorded, on the precedent two paragraphs up. The boot seam (`crates/control/src/drive.rs`,
        // `DriveLean`) bounds `lean_max` and `lean_slew` as it reads them, and a client could mirror
        // those bounds the way `Fields.NOISE_FLOOR` mirrors the noise floor's band. Nothing in this
        // module exercises them: the field's id, type and defaults are pinned by the Setup-field
        // gate, and the editor that would offer the ranges is the app's. Adding the pin is a change
        // to what this mirror CARRIES, not a gate fix, so it is a queued item rather than a silent
        // absence.

        // The battery calibration's boot clamps (`VbattCal::new`), which the Setup rows offer as ranges.
        val battery = rust("crates/orchestrator/src/battery.rs")
        fun cal(name: String): Long {
            val raw = findOne(battery, """^pub\s+const\s+$name\s*:\s*i16\s*=\s*([^;]+);""", name)
                .groupValues[1].trim().replace("_", "")
            return if (raw.startsWith("-")) -literal(name, raw.drop(1), "vbatt clamp").toLong() else literal(name, raw, "vbatt clamp").toLong()
        }
        assertEquals(cal("SLOPE_MIN")..cal("SLOPE_MAX"), Fields.VBATT_SLOPE, "the vbatt slope clamp drifted")
        assertEquals(cal("OFFSET_MIN")..cal("OFFSET_MAX"), Fields.VBATT_OFFSET, "the vbatt offset clamp drifted")
        for ((i, range) in listOf(Fields.VBATT_SLOPE, Fields.VBATT_OFFSET).withIndex()) {
            val d = (Fields.BOARD_VBATT_CAL.defaults[i] as Value.I16).v.toLong()
            assertTrue(d in range, "BOARD_VBATT_CAL default $i is outside its clamp")
        }

        // CONTROL_RIDER_REQUIRED's decode: 0 is the only byte that waives the rider. The rule has
        // ONE home, `rider_required_from`, because the field is now read twice: at the boot seam
        // (`ControlDispatch::new`) and again at every arm (`ControlDispatch::re_apply_values`, the
        // arm-time re-read), and the two must not drift. So this checks the rule where it lives and
        // then checks that both sites go through it.
        val mode = rust("crates/control/src/mode.rs")
        assertTrue(
            Regex("""const\s+fn\s+rider_required_from\s*\(\s*byte\s*:\s*u8\s*\)\s*->\s*bool\s*\{\s*byte\s*!=\s*0\s*\}""")
                .containsMatchIn(mode),
            "control::mode::rider_required_from no longer decodes the rider byte as `!= 0`: review Fields.RiderRequired",
        )
        for (site in listOf(
            "rider_required: rider_required_from(rider_required_byte),",
            "self.rider_required = rider_required_from(rider_required_byte);",
        )) {
            assertTrue(
                mode.contains(site),
                "a rider-byte decode site no longer goes through rider_required_from: `$site`",
            )
        }
        assertEquals(0, Fields.RiderRequired.NOT_REQUIRED)
        assertEquals(Value.U8(Fields.RiderRequired.REQUIRED), Fields.CONTROL_RIDER_REQUIRED.default)

        // CONTROL_BATTERY_FLOOR's decode: known AND (no floor at `<= 0`, else at or above it).
        assertTrue(
            Regex("""battery\s*!=\s*0\s*&&\s*\(self\.battery_floor\s*<=\s*0\s*\|\|\s*battery\s*>=\s*self\.battery_floor\)""")
                .containsMatchIn(rust("crates/control/src/mode.rs")),
            "ControlDispatch::battery_ok no longer reads `<= 0` as no floor: review Fields.CONTROL_BATTERY_FLOOR and its Setup hint",
        )
    }

    /**
     * The orientation rule the Setup screen enforces before staging `IMU_AXIS_SIGN`
     * (`specs/rider-ui.md` section 3.4, D7), against `imu::Config` in crates/imu/src/lib.rs, which
     * since `IMU_AXIS_ROLE` (0x68, `specs/imu.md`) judges a frame by the signs AND the axis roles.
     *
     * The facts pinned. The reference map an unset sign falls back to, and [Orientation.DEFAULT_ROLES]
     * an unset role falls back to. `staged`'s signature (it returns `Result<Self, FrameError>`), its
     * unset rule on both fields, and its refusal order (roles, then accel, then gyro), with
     * `FrameError`'s variants equal to [Orientation.FrameError]'s. The rotation rule, pinned as the
     * Rust it is written in (`triple_is_rotation` delegating to `frame_is_rotation` under the default
     * roles, `frame_is_rotation`'s parity rule and `body_order`'s role check), normalised for
     * whitespace with comments dropped, so ANY change to the board's notion of a legal frame fails
     * here and forces a review of [Orientation.frameIsRotation]. And the shape of `Config` itself:
     * the signs, the gyro bias and the roles, nothing else, so a new member that changes which
     * frames are legal goes red here rather than leaving the screen refusing or staging by the wrong
     * rule.
     */
    @Test
    fun theOrientationRuleAgreesWithTheRustSource() {
        val imu = rust("crates/imu/src/lib.rs")

        val defStart = imu.indexOf("impl Default for Config {")
        check(defStart >= 0) { "No `impl Default for Config {` found" }
        val defBody = imu.substring(defStart, imu.indexOf("\n}", defStart))
        fun signed(what: String, v: String) =
            if (v.startsWith("-")) -literal(what, v.drop(1), what) else literal(what, v, what)
        val reference = findOne(defBody, """sign\s*:\s*\[([^\]]+)\]""", "reference sign map").groupValues[1]
            .split(",").map { signed("Config::default().sign", it.trim()) }
        assertEquals(reference, Orientation.REFERENCE, "the reference sign map drifted")
        assertTrue(
            Regex("""^\s*roles\s*:\s*DEFAULT_ROLES\s*,""", RegexOption.MULTILINE).containsMatchIn(defBody),
            "Config::default().roles is no longer DEFAULT_ROLES: review Orientation.effectiveRoles",
        )
        val defaultRoles = findOne(imu, """^pub\s+const\s+DEFAULT_ROLES\s*:\s*\[u8;\s*2\]\s*=\s*\[([^\]]+)\];""", "DEFAULT_ROLES")
            .groupValues[1].split(",").map { literal("DEFAULT_ROLES", it.trim(), "default role") }
        assertEquals(defaultRoles, Orientation.DEFAULT_ROLES, "the default axis roles drifted")

        val configImpl = implBlock(imu, "Config")
        fun fnBody(signature: String) = rustBody(configImpl, signature, "\n    }")

        assertEquals(
            "Self::frame_is_rotation(DEFAULT_ROLES, triple)",
            fnBody("pub fn triple_is_rotation("),
            "imu::Config::triple_is_rotation changed: review Orientation.tripleIsRotation",
        )
        assertEquals(
            "let Some(order) = body_order(roles) else { return false; }; " +
                "let parity = if order[2] == (order[1] + 1) % 3 { 1 } else { -1 }; " +
                "triple.iter().all(|s| *s == 1 || *s == -1) && triple[0] * triple[1] * triple[2] == parity",
            fnBody("pub fn frame_is_rotation(roles: [u8; 2], triple: [i32; 3]) -> bool"),
            "imu::Config::frame_is_rotation changed: review Orientation.frameIsRotation",
        )
        assertEquals(
            "let [up, pitch] = roles; " +
                "if !(1..=3).contains(&up) || !(1..=3).contains(&pitch) || up == pitch { return None; } " +
                "let (up, pitch) = (up - 1, pitch - 1); Some([3 - up - pitch, pitch, up])",
            rustBody(imu, "fn body_order(roles: [u8; 2]) -> Option<[u8; 3]>", "\n}"),
            "imu::body_order changed: review Orientation.bodyOrder",
        )

        val staged = fnBody("pub fn staged(sign: [i32; 6], gyro_bias: [i32; 3], roles: [u8; 2]) -> Result<Self, FrameError>")
        assertTrue(
            staged.contains("..Config::default()") &&
                staged.contains("for (dst, staged) in cfg.sign.iter_mut().zip(sign.iter()) { if *staged != 0 { *dst = *staged; } }") &&
                staged.contains("for (dst, staged) in cfg.roles.iter_mut().zip(roles.iter()) { if *staged != 0 { *dst = *staged; } }"),
            "imu::Config::staged's unset rule changed: review Orientation.effective / effectiveRoles. Got: $staged",
        )
        val order = listOf(
            "if body_order(cfg.roles).is_none() { Err(FrameError::Roles) }",
            "else if !Self::frame_is_rotation(cfg.roles, [s[0], s[1], s[2]]) { Err(FrameError::Accel) }",
            "else if !Self::frame_is_rotation(cfg.roles, [s[3], s[4], s[5]]) { Err(FrameError::Gyro) }",
            "else { Ok(cfg) }",
        ).map { staged.indexOf(it) }
        assertTrue(
            order.all { it >= 0 } && order == order.sorted(),
            "imu::Config::staged's refusal order changed: review Orientation.frameError. Got: $staged",
        )

        assertEquals(
            Orientation.FrameError.entries.map { it.name },
            rustVariants(imu, "FrameError").map { it.uppercase() },
            "imu::FrameError drifted",
        )

        assertEquals(
            listOf("sign", "gyro_bias", "roles"),
            fields(imu, "Config").map { it.first },
            "imu::Config grew or lost a member: the frames Orientation offers may no longer be the legal ones",
        )
    }

    // --- framing ----------------------------------------------------------------------------------

    @Test
    fun l2FrameConstantsAgreeWithTheRustSource() {
        val framer = rust("crates/link/src/framer.rs")
        fun c(name: String, ty: String) = num(
            findOne(framer, """^pub const $name: $ty = ([^;]+);""", name).groupValues[1].trim(),
        )

        assertEquals(c("SOF", "u8"), StreamFrame.SOF, "L2 SOF drifted")
        assertEquals(c("STREAM_HEADER_LEN", "usize"), StreamFrame.STREAM_HEADER_LEN)
        assertEquals(c("STREAM_CRC_LEN", "usize"), StreamFrame.STREAM_CRC_LEN)
        assertEquals(c("MAX_L2_LEN", "usize"), StreamFrame.MAX_L2_LEN)
        // MAX_STREAM_FRAME is an expression in the Rust, so check the arithmetic it stands for.
        assertEquals(
            StreamFrame.STREAM_HEADER_LEN + StreamFrame.MAX_L2_LEN + StreamFrame.STREAM_CRC_LEN,
            StreamFrame.MAX_STREAM_FRAME,
        )
    }

    @Test
    fun fragHeaderConstantsAgreeWithTheRustSource() {
        val frag = rust("crates/link/src/frag.rs")
        fun c(name: String) = num(
            findOne(frag, """^pub const $name: \w+ = ([^;]+);""", name).groupValues[1]
                .trim().replace("_", "").let { if (it.startsWith("0b")) it.drop(2).toInt(2).toString() else it },
        )

        assertEquals(c("MORE_BIT"), FragHdr.MORE_BIT, "frag MORE bit drifted")
        assertEquals(c("MAX_PID"), FragHdr.MAX_PID)
        assertEquals(c("MAX_FRAG_IDX"), FragHdr.MAX_FRAG_IDX)
        assertEquals(c("MAX_FRAGMENTS"), FragHdr.MAX_FRAGMENTS)
    }

    @Test
    fun l3HeaderLengthAgreesWithTheRustSource() {
        val pdu = rust("crates/net/src/pdu.rs")
        val fromRust = num(
            findOne(pdu, """^pub const HEADER_LEN: usize = (\d+);""", "L3 HEADER_LEN").groupValues[1],
        )
        assertEquals(fromRust, HEADER_LEN, "L3 header length drifted")
    }

    /**
     * The CRC the firmware actually computes. `crates/base/src/crc16.rs` no longer instantiates the
     * `crc` crate's named `CRC_16_MODBUS` algorithm: it computes the same algorithm itself from a
     * nibble table, so the pin is the two constants that DEFINE it, the reflected polynomial
     * (`POLY`) and the init value (`INIT`). Those are exactly what `l2.Crc16` mirrors
     * (`REFLECTED_POLY` / `INIT`, both private to that object, hence the literals here), and the
     * known-answer vectors in [WireDriftTest] pin the arithmetic that falls out of them.
     *
     * This reads MORE of the Rust than the algorithm name it replaced: a swapped polynomial or a
     * changed seed used to be invisible here as long as the `crc` crate expression kept its shape.
     */
    @Test
    fun crcAlgorithmAgreesWithTheRustSource() {
        val crc = rust("crates/base/src/crc16.rs")
        val poly = findOne(crc, """^const POLY: u16 = ([^;]+);""", "CRC polynomial")
        assertEquals(
            0xA001,
            literal("POLY", poly.groupValues[1], "CRC constant"),
            "the firmware's CRC polynomial changed",
        )
        val init = findOne(crc, """^const INIT: u16 = ([^;]+);""", "CRC init value")
        assertEquals(
            0xFFFF,
            literal("INIT", init.groupValues[1], "CRC constant"),
            "the firmware's CRC init value changed",
        )
    }

    // --- the board-layout validator mirror -------------------------------------------------------

    private val boardLib by lazy { rust("crates/board/src/lib.rs") }
    private val boardPlumbing by lazy { rust("crates/board/src/plumbing.rs") }
    private val boardTests by lazy { rust("crates/board/src/tests.rs") }

    /** `SelfHold` -> `SELF_HOLD`: a Rust variant name as the Kotlin enum spells it. */
    private fun camelToScreaming(s: String): String =
        s.replace(Regex("""(?<!^)([A-Z])"""), "_$1").uppercase()

    /**
     * The `Name,` or `Name(payload),` variants of `enum <name> {`, in declaration order.
     *
     * Censused, because a variant this does not recognise (a trailing comment after the comma, a
     * payload wrapped across lines, a struct variant) would otherwise leave the Rust side one
     * short and agree with a mirror that never carried it. `MockChip`'s own `F103RC, // the 12-FET
     * part` is exactly that shape, and it is why the three hand-rolled variant readers this
     * replaced each took a different pattern.
     */
    private fun rustVariants(text: String, name: String): List<String> =
        findAllClaiming(
            enumBlock(text, name),
            """^\s+(\w+)(?:\([^)]*\))?\s*,""",
            VARIANT_LINE,
            "$name variants",
        ).map { it.groupValues[1] }

    /**
     * A function body from [signature] to [close], comments dropped and whitespace collapsed.
     *
     * Where a rule is mirrored as Kotlin rather than as a number, the only pin that catches a
     * CHANGE to the rule is its own text, and a red here means "go and read both". Every verbatim
     * pin in the file routes through this one reader, [theOrientationRuleAgreesWithTheRustSource]
     * included: it carried its own copy of this function without the `check` on the closing
     * delimiter, so a renamed signature there died on a negative substring index instead of saying
     * what it could not find.
     */
    private fun rustBody(text: String, signature: String, close: String): String {
        val s = text.indexOf(signature)
        check(s >= 0) { "No `$signature` found" }
        val open = text.indexOf('{', s)
        val end = text.indexOf(close, open)
        check(end > open) { "Unterminated `$signature`" }
        return text.substring(open + 1, end)
            .replace(Regex("""//[^\n]*"""), "")
            .replace(Regex("""\s+"""), " ").trim()
    }

    /**
     * The field set a layout IS, against `board::BoardFields` and `board::MotorFields`.
     *
     * A member the Rust grows and this mirror does not is a field the board validates and a client's
     * prediction ignores, which turns an exact verdict into a guess. The expected lists are the
     * Kotlin data classes' own property names, so a red here is answered by editing them together.
     */
    @Test
    fun theBoardFieldStructsAgreeWithTheRustSource() {
        assertEquals(
            listOf(
                "selfHold" to "u8", "vbatt" to "u8", "buzzer" to "u8", "ledGreen" to "u8",
                "ledOrange" to "u8", "ledRed" to "u8", "padA" to "u8", "padB" to "u8",
                "button" to "u8", "imuScl" to "u8", "imuSda" to "u8", "imuModel" to "u8",
                "motors" to "[MotorFields; 2]",
            ),
            members(boardLib, "BoardFields"),
            "board::BoardFields drifted from BoardFields",
        )
        assertEquals(
            listOf(
                "hallA" to "u8", "hallB" to "u8", "hallC" to "u8",
                "gateHiA" to "u8", "gateHiB" to "u8", "gateHiC" to "u8",
                "gateLoA" to "u8", "gateLoB" to "u8", "gateLoC" to "u8",
                "deadTime" to "u8", "direction" to "u8", "alignOffset" to "u8",
                "currentSense" to "u8", "currentCal" to "u16", "phaseA" to "u8", "phaseB" to "u8",
            ),
            members(boardLib, "MotorFields"),
            "board::MotorFields drifted from MotorFields",
        )
        // The absent sentinel both sides spell: the crate's and the registry's, which are one value.
        val absent = literal("ABSENT", findOne(boardLib, """^pub\s+const\s+ABSENT\s*:\s*u8\s*=\s*([^;]+);""", "ABSENT").groupValues[1], "ABSENT")
        assertEquals(PIN_ABSENT, absent, "board::ABSENT drifted")
        assertEquals(
            PIN_ABSENT,
            literal("PIN_ABSENT", findOne(rust("crates/store/src/field.rs"), """^pub\s+const\s+PIN_ABSENT\s*:\s*u8\s*=\s*([^;]+);""", "PIN_ABSENT").groupValues[1], "PIN_ABSENT"),
            "store::PIN_ABSENT drifted",
        )
        assertEquals(
            BoardFields.MOTORS,
            literal("motors", findOne(boardLib, """pub\s+motors\s*:\s*\[MotorFields;\s*(\d+)\]""", "motor count").groupValues[1], "motor count"),
            "the number of motors a field set carries drifted",
        )
    }

    /**
     * [BoardField], against the Rust enum and against `plumbing::field_id`, which is what gives each
     * variant the registry id a `BOARD_OBS` record names it by.
     *
     * An exact ORDERED comparison: a variant the Rust adds fails here rather than leaving the mirror
     * silently short of a field a refusal can name, and the id mapping is read out of the firmware's
     * own match rather than copied, so a field repointed at another registry entry fails too.
     */
    @Test
    fun theBoardFieldEnumAgreesWithTheRustSource() {
        assertEquals(
            rustVariants(boardLib, "BoardField").map(::camelToScreaming),
            BoardField.entries.map { it.name },
            "board::BoardField drifted from BoardField",
        )
        val ids = findAll(boardPlumbing, """BoardField::(\w+)\s*=>\s*store::(\w+)\.id\(\)""", "field_id arms")
            .associate { camelToScreaming(it.groupValues[1]) to it.groupValues[2] }
        assertEquals(
            BoardField.entries.map { it.name }.toSet(),
            ids.keys,
            "plumbing::field_id names a different field set than BoardField",
        )
        for (field in BoardField.entries) {
            val handle = checkNotNull(ids[field.name])
            val def = checkNotNull(Fields.ALL[handle] ?: Fields.INDEXED[handle]?.at(0)) {
                "plumbing::field_id maps ${field.name} to store::$handle, which Fields does not mirror"
            }
            assertEquals(def.id, field.def.id, "${field.name} points at the wrong registry field")
        }
    }

    /**
     * [BoardErrorKind], against `board::BoardErrorKind` and the result codes
     * `plumbing::BoardObs::failure` reports each one as.
     *
     * The codes are the verdict a client renders: the whole claim of the mirror is that the refusal
     * it predicts is the record the board would write, so a code that moves on one side and not the
     * other makes a client describe a failure as some other failure.
     */
    @Test
    fun theValidatorRefusalsAndObsCodesAgreeWithTheRustSource() {
        val kinds: Map<String, BoardErrorKind> = listOf(
            BoardErrorKind.BadEncoding(0x40),
            BoardErrorKind.IncompleteGroup,
            BoardErrorKind.DeadTimeBelowFloor,
            BoardErrorKind.DuplicatePin(Pin.byName("PB3")!!),
            BoardErrorKind.ReservedPin(Pin.byName("PA2")!!),
            BoardErrorKind.UnknownPin(Pin.byName("PF15")!!),
            BoardErrorKind.GateCapableMisused(Pin.byName("PA8")!!),
            BoardErrorKind.InvalidGateSet,
            BoardErrorKind.NotAdcCapable(Pin.byName("PC13")!!),
            BoardErrorKind.NotI2cPair,
            BoardErrorKind.ImuFrame(3),
        ).associateBy { checkNotNull(it::class.simpleName) }
        assertEquals(
            rustVariants(boardLib, "BoardErrorKind").toSet(),
            kinds.keys,
            "board::BoardErrorKind drifted from BoardErrorKind",
        )
        val codes = findAll(boardPlumbing, """BoardErrorKind::(\w+)(?:\([^)]*\))?\s*=>\s*\((\d+),""", "BoardObs failure arms")
            .associate { it.groupValues[1] to it.groupValues[2].toInt() }
        assertEquals(kinds.keys, codes.keys, "BoardObs::failure covers a different refusal set")
        for ((name, kind) in kinds) {
            assertEquals(codes[name], kind.obsResult, "$name's BOARD_OBS result code drifted")
        }
        // The success code, which is the other half of the same record.
        assertEquals(
            0,
            literal("OBS_OK", findOne(boardPlumbing, """^pub\s+const\s+OBS_OK\s*:\s*u8\s*=\s*([^;]+);""", "OBS_OK").groupValues[1], "OBS_OK"),
            "OBS_OK is no longer 0, so a clean verdict no longer reads as clean",
        )
    }

    /**
     * [DEAD_TIME_MIN_DTG], against the Rust that owns it and against the two rider-app editors
     * that offer `motor.dead_time`.
     *
     * The floor is a SAFETY number, not a preference: below it both FETs of a leg conduct through
     * the transition. The firmware is the backstop, but an app that offered a sub-floor value
     * would let an operator stage one and watch it be refused at the next boot, so the editors
     * have to follow the same number rather than restate it. They are read as text for the same
     * reason the keepalive cadence is: the app is not built by this module and cannot be depended
     * on from here, and a literal creeping back into one of those ranges would otherwise be
     * caught by nothing.
     */
    @Test
    fun theDeadTimeFloorAgreesWithTheRustSourceAndTheAppsEditors() {
        assertEquals(
            DEAD_TIME_MIN_DTG,
            literal(
                "DEAD_TIME_MIN_DTG",
                findOne(boardLib, """^pub\s+const\s+DEAD_TIME_MIN_DTG\s*:\s*u8\s*=\s*([^;]+);""", "DEAD_TIME_MIN_DTG")
                    .groupValues[1],
                "the dead-time floor",
            ),
            "board::DEAD_TIME_MIN_DTG drifted from the mirror's floor",
        )
        // Both app rows take their lower bound FROM the constant above, imported from this module.
        for ((path, pattern) in listOf(
            "apps/rider/app/src/main/java/com/hoverboard/remote/model/LayoutRows.kt" to
                """LayoutEditor\.Number\(\s*DEAD_TIME_MIN_DTG\b""",
            "apps/rider/app/src/main/java/com/hoverboard/remote/model/SetupFields.kt" to
                """Editor\.Range\(\s*DEAD_TIME_MIN_DTG\b""",
        )) {
            val source = app(path)
            findOne(source, """^import com\.hoverboard\.protocol\.board\.DEAD_TIME_MIN_DTG$""", "$path's import of the floor")
            findOne(source, pattern, "$path's dead-time editor range")
        }
    }

    /**
     * The ORDER `validate` takes its fields in, which decides which of several planted mistakes a
     * client reports (first failure wins, in field order).
     *
     * The Rust order is read from its own call sites, in all three shapes it uses: the power latch,
     * which is taken on its own and first because taking it is what reserves its pin; the remaining
     * single-pin fields, which the Rust takes through a loop over a table of
     * `(fields.field, BoardField::Variant)` rows; and each motor's hall, gate and phase arrays,
     * through a loop over tuples of `(mf.field, BoardField::Variant)`. The Kotlin order is then
     * MEASURED rather than restated: a bad encoding is planted in every pin field at once, the
     * refused field must be the first in order, and that field is then set absent and the probe
     * repeated, which walks the whole set.
     */
    @Test
    fun theValidatorTakesItsFieldsInTheRustsOrder() {
        // ONE selector for both shapes the Rust takes a single-pin field in (the latch's own
        // `take(..)` call and the table's rows), in source order, which is the order the validator
        // runs them in. Censused on `fields.<name>,` alone, with no bracket, so a row wrapped across
        // lines is still censused; a row in a shape the selector does not reach fails by
        // name instead of shortening the Rust order silently and agreeing with a Layout that never
        // carried it.
        val singles = findAllClaiming(
            boardLib,
            """take\(\s*fields\.(\w+)\s*,\s*single\(BoardField::(\w+)\)""" +
                """|^\s+\(\s*fields\.(\w+)\s*,\s*BoardField::(\w+)\s*\)\s*,""",
            FIELD_TAKE_LINE,
            "the validator's single-pin takes",
        ).map { m ->
            val g = m.groupValues
            if (g[1].isNotEmpty()) g[1] to g[2] else g[3] to g[4]
        }
        val perMotor = findAllClaiming(
            boardLib,
            """\(\s*mf\.(\w+)\s*,\s*BoardField::(\w+)\s*\)""",
            MOTOR_TAKE_LINE,
            "per-motor field tuples",
        ).map { it.groupValues[1] to it.groupValues[2] }
        for ((snake, variant) in singles + perMotor) {
            assertEquals(
                snake.uppercase(),
                camelToScreaming(variant),
                "the Rust pairs $snake with BoardField::$variant, which name different fields",
            )
        }
        val expected = singles.map { BoardField.valueOf(camelToScreaming(it.second)) to null } +
            (0 until BoardFields.MOTORS).flatMap { m ->
                perMotor.map { (_, variant) -> BoardField.valueOf(camelToScreaming(variant)) to m }
            }
        assertEquals(
            Layout.SLOTS.count { it.isPin },
            expected.size,
            "the Rust takes a different number of pin fields than the layout carries",
        )

        // Every pin field planted with a byte the encoding does not define (port E).
        var fields = Layout.SLOTS.filter { it.isPin }.fold(BoardFields()) { acc, slot -> slot.on(acc, 0x40) }
        val measured = mutableListOf<Pair<BoardField, Int?>>()
        repeat(expected.size) {
            val err = checkNotNull(validate(fields, ChipFamily.F103C8, reservedSet(ChipFamily.F103C8, 0)).error) {
                "expected a refusal with ${expected.size - measured.size} bad pins still planted"
            }
            assertEquals(BoardErrorKind.BadEncoding(0x40), err.kind, "at step ${measured.size}")
            measured += err.field.field to err.field.motor
            fields = checkNotNull(Layout.forField(err.field)).on(fields, PIN_ABSENT)
        }
        assertEquals(expected, measured, "the validator's field order drifted from the Rust's")
        assertNull(
            validate(fields, ChipFamily.F103C8, reservedSet(ChipFamily.F103C8, 0)).error,
            "with every pin absent the layout is a valid empty one",
        )
    }

    /**
     * The layout's field set against `plumbing::read_fields`, the function that assembles the same
     * struct from the same registered fields at boot.
     *
     * Compared as a set of (registry handle, per-motor) pairs: a field added to the firmware's read
     * is a field the board validates, so a client that does not read it predicts from a layout that
     * is not the board's.
     */
    @Test
    fun theLayoutIsTheFieldSetTheFirmwareReads() {
        val from = boardPlumbing.indexOf("pub fn read_fields")
        check(from >= 0) { "No `pub fn read_fields` in crates/board/src/plumbing.rs" }
        val to = boardPlumbing.indexOf("/// One safe-USART allowlist entry", from)
        check(to > from) {
            "The doc comment that bounds `read_fields` (\"One safe-USART allowlist entry\") moved or " +
                "was reworded, so this gate cannot tell where the function ends"
        }
        val readFields = boardPlumbing.substring(from, to)
        // Censused on `store::`, so a read written in a third shape fails by name rather than
        // leaving the firmware's field set one short of what it really reads.
        val rust = findAllClaiming(
            readFields,
            """store::(\w+)\.at\(m\)|get\(store::(\w+)\)""",
            STORE_READ_LINE,
            "the registered fields read_fields reads",
        ).map { m ->
            if (m.groupValues[1].isNotEmpty()) m.groupValues[1] to true else m.groupValues[2] to false
        }.toSet()
        val handleOf = Fields.ALL.entries.associate { (name, def) -> def.id to name }
        val kotlin = Layout.SLOTS.map { slot ->
            checkNotNull(handleOf[slot.def.id]) { "${slot.key} has no Fields entry" } to (slot.motor != null)
        }.toSet()
        assertEquals(rust, kotlin, "Layout.SLOTS is not the field set plumbing::read_fields reads")
        assertEquals(
            Layout.SLOTS.size,
            Layout.SLOTS.map { it.key }.distinct().size,
            "two layout slots name the same key",
        )
    }

    /**
     * The capability tables, against `MockChip` in `crates/board/src/tests.rs`.
     *
     * Two things are pinned, and the second is the one that matters. The gate maps and the chip set
     * are values, compared directly. The five query bodies are pinned as the Rust they are written
     * in, normalised for whitespace with comments dropped, because the Kotlin mirrors a RULE rather
     * than a number: the F130's PF6/PF7 bonding has already been corrected once in that table, and
     * such a correction has to reach this mirror or the verdict is wrong on that part.
     *
     * What this does NOT pin is that `MockChip` equals the real silicon answers. That is the
     * `rcap_agreement` module's job, against runtime-hal, in the cargo jobs; this mirror's chain of
     * trust runs through it ([ChipFamily]).
     */
    @Test
    fun theCapabilityTablesAgreeWithTheRustMockTables() {
        fun gates(name: String): List<Int> =
            findOne(boardTests, """const $name\s*:\s*\[u8;\s*3\]\s*=\s*\[([^\]]+)\];""", name)
                .groupValues[1].split(",").map { literal(name, it.trim(), "gate pin") }
        assertEquals(gates("GATES_T0_HI"), ChipFamily.GATES_T0_HI, "the TIMER0 high-side map drifted")
        assertEquals(gates("GATES_T0_LO"), ChipFamily.GATES_T0_LO, "the TIMER0 low-side map drifted")
        assertEquals(gates("GATES_T8_HI"), ChipFamily.GATES_T8_HI, "the TIM8 high-side map drifted")
        assertEquals(gates("GATES_T8_LO"), ChipFamily.GATES_T8_LO, "the TIM8 low-side map drifted")

        assertEquals(
            rustVariants(boardTests, "MockChip"),
            ChipFamily.entries.map { it.name },
            "the fleet's modelled parts drifted",
        )

        val impl = boardTests.substring(
            boardTests.indexOf("impl Capabilities for MockChip {"),
            boardTests.indexOf("\n}", boardTests.indexOf("impl Capabilities for MockChip {")),
        )
        fun query(signature: String) = rustBody(impl, signature, "\n    }")
        assertEquals(
            "let (port, n) = (pin.port(), pin.pin()); match self { " +
                "MockChip::F103C8 => match port { 0 | 1 => true, 2 => n >= 13, 3 => n <= 1, _ => false, }, " +
                "MockChip::F130C8 => match port { 0 | 1 => true, 2 => n >= 13, 5 => matches!(n, 0 | 1 | 6 | 7), _ => false, }, " +
                "MockChip::F103RC => match port { 0..=2 => true, 3 => n <= 2, _ => false, }, }",
            query("fn pin_exists("),
            "the modelled pin bonding changed: review ChipFamily.pinExists",
        )
        assertEquals(
            "let b = pin.packed(); let t0 = GATES_T0_HI.contains(&b) || GATES_T0_LO.contains(&b); match self { " +
                "MockChip::F103C8 | MockChip::F130C8 => t0, " +
                "MockChip::F103RC => t0 || GATES_T8_HI.contains(&b) || GATES_T8_LO.contains(&b), }",
            query("fn gate_capable("),
            "the gate-capable denylist changed: review ChipFamily.gateCapable",
        )
        assertEquals(
            "if Self::set_matches(hi, lo, GATES_T0_HI, GATES_T0_LO) { return Some(0); } " +
                "if matches!(self, MockChip::F103RC) && Self::set_matches(hi, lo, GATES_T8_HI, GATES_T8_LO) { return Some(1); } None",
            query("fn gate_set("),
            "the gate-set derivation changed: review ChipFamily.gateSet",
        )
        assertEquals(
            "match (pin.port(), pin.pin()) { (0, n) if n <= 7 => Some(n), (1, n) if n <= 1 => Some(8 + n), " +
                "(2, n) if n <= 5 && self.pin_exists(pin) => Some(10 + n), _ => None, }",
            query("fn adc_channel("),
            "the analog map changed: review ChipFamily.adcChannel",
        )
        assertEquals(
            "let _ = self; match (scl.packed(), sda.packed()) { (0x16, 0x17) => Some(0), (0x1A, 0x1B) => Some(1), _ => None, }",
            query("fn i2c_pair("),
            "the I2C pair derivation changed: review ChipFamily.i2cPair",
        )
    }

    /**
     * The reserved set: the allowlist the firmware compiles, the SWD pins, the freeing rule, and
     * which BLE wiring each family can route.
     *
     * This is the mechanism that makes a layout which steals a link port impossible to EXPRESS, so a
     * mirror that reserved a different set would let a client stage exactly the layout the board
     * refuses. The allowlist entries come out of `SAFE_LINK_USARTS` and its `net` slot constants; the
     * rule comes out of `claims_pins` verbatim; and the per-family routability, which the firmware
     * asks the HAL rather than writing down, comes out of the assertion
     * `each_family_resolves_to_the_wiring_it_is_built_with` makes against it.
     */
    @Test
    fun theReservedSetRuleAgreesWithTheRustSource() {
        val main = rust("crates/firmware/src/main.rs")
        val slots = findAll(main, """^\s+const (PORT_IDX_\w+)\s*:\s*u8\s*=\s*([^;]+);""", "net port slots")
            .associate { it.groupValues[1] to literal(it.groupValues[1], it.groupValues[2], "net slot") }
        assertEquals(slots["PORT_IDX_UART"], NET_PORT_UART, "the inter-board link's net slot drifted")
        assertEquals(slots["PORT_IDX_BLE"], NET_PORT_BLE, "the BLE module's net slot drifted")

        // The array's own declared length against the number of entries read: the one census that
        // reaches inside a struct literal, where a line-based one cannot (a member reordered or
        // added in the literal drops the whole entry out of this pattern).
        val declared = literal(
            "SAFE_LINK_USARTS",
            findOne(main, """const SAFE_LINK_USARTS\s*:\s*\[SafeLinkUsart;\s*(\d+)\]""", "SAFE_LINK_USARTS length")
                .groupValues[1],
            "allowlist length",
        )
        val entries = findAll(
            main,
            """SafeLinkUsart\s*\{\s*link_set_bit:\s*([\w:]+),\s*net_port:\s*([\w:]+),\s*pins:\s*\[([^\]]+)\]""",
            "allowlist entries",
        ).map { m ->
            fun slot(v: String) = slots[v] ?: literal("link_set_bit", v, "allowlist bit")
            Triple(slot(m.groupValues[1]), slot(m.groupValues[2]), m.groupValues[3].split(",").map { literal("pins", it.trim(), "allowlist pin") })
        }
        assertEquals(
            declared,
            entries.size,
            "SAFE_LINK_USARTS declares $declared entries but this gate read ${entries.size} of them: " +
                "one is written in a shape the selector does not reach, so the mirrored allowlist is " +
                "being compared against part of the firmware's",
        )
        for (chip in ChipFamily.entries) {
            assertEquals(
                entries.map { (bit, slot, pins) -> Triple(bit, slot, pins) },
                allowlistFor(chip).map { Triple(it.linkSetBit, it.netPort, it.pins) },
                "$chip: the mirrored allowlist drifted from SAFE_LINK_USARTS",
            )
        }
        assertEquals(
            findOne(boardPlumbing, """pub const SWD_PINS\s*:\s*\[u8;\s*2\]\s*=\s*\[([^\]]+)\];""", "SWD_PINS")
                .groupValues[1].split(",").map { literal("SWD_PINS", it.trim(), "SWD pin") },
            SWD_PINS,
            "the SWD pins drifted",
        )
        assertEquals(
            "port.routable && (link_set == 0 || (link_set & (1 << port.link_set_bit)) != 0)",
            rustBody(boardPlumbing, "fn claims_pins(", "\n}"),
            "the LINK_SET freeing rule changed: review claimsPins",
        )
        val reservedBody = rustBody(boardPlumbing, "pub fn reserved_set(", "\n}")
        assertTrue(
            reservedBody.contains("for p in SWD_PINS") && reservedBody.contains("if claims_pins(port, link_set)"),
            "reserved_set is no longer SWD plus the claiming ports' pins: review reservedSet. Got: $reservedBody",
        )

        // Which allowlist entry carries the BLE slot per family, as the agreement suite asserts it
        // against runtime-hal's own pin model. Its `PORTS` table is the same allowlist in a
        // different order, so it is checked against the firmware's before its indices are used.
        val portsStart = boardTests.indexOf("const PORTS:")
        check(portsStart >= 0) { "No `const PORTS:` table in the routability agreement module" }
        val portsDeclared = literal(
            "PORTS",
            findOne(boardTests, """const PORTS\s*:\s*\[\([^)]*\);\s*(\d+)\]""", "PORTS length").groupValues[1],
            "agreement table length",
        )
        val ports = findAll(
            boardTests.substring(portsStart, boardTests.indexOf("];", portsStart)),
            """\(\s*(\d+)\w*,\s*(\d+)\w*,\s*\[([^\]]+)\]\s*\)""",
            "the agreement module's allowlist",
        ).map { m ->
            Triple(
                literal("PORTS", m.groupValues[1], "bit"),
                literal("PORTS", m.groupValues[2], "slot"),
                m.groupValues[3].split(",").map { literal("PORTS", it.trim().removeSuffix("u8"), "pin") },
            )
        }
        assertEquals(portsDeclared, ports.size, "the agreement module's PORTS table is longer than this gate reads")
        assertEquals(
            entries.toSet(),
            ports.toSet(),
            "the agreement module's PORTS table is no longer the firmware's allowlist, so its " +
                "per-family conclusion cannot be read against it",
        )
        val want = findOne(
            boardTests,
            """let want = if part == "F130C8" \{ (\d+) \} else \{ (\d+) \};""",
            "the per-family BLE wiring index",
        )
        val f1x0Pins = ports[literal("want", want.groupValues[1], "allowlist index")].third
        val f10xPins = ports[literal("want", want.groupValues[2], "allowlist index")].third
        for (chip in ChipFamily.entries) {
            val ble = allowlistFor(chip).filter { it.netPort == NET_PORT_BLE && it.routable }
            assertEquals(1, ble.size, "$chip: exactly one BLE wiring routes")
            assertEquals(
                if (chip.mcu == McuFamily.F1X0) f1x0Pins else f10xPins,
                ble.single().pins,
                "$chip: the routable BLE wiring is not the one the family is built with",
            )
        }
    }

    private companion object {
        /**
         * How many of the app's `INPUTS` keepalive periods the firmware's arm-mirror window must
         * span, checked by [theArmMirrorWindowOutlastsTheAppsKeepalive].
         *
         * Three, so a single lost keepalive is nowhere near a disarm and two consecutive losses
         * still leave the window intact. It is a floor on the RELATION, not a target: sending
         * faster is always safe, and it is the window shrinking or the cadence slowing that this
         * number exists to catch.
         */
        const val KEEPALIVE_MARGIN = 3

        // --- the censuses [findAllClaiming] checks its selectors against ------------------------
        //
        // Each one is what an item looks like at its LOOSEST, and each is deliberately dumber than
        // the selector it guards: the census has to recognise the shapes the selector does not, or
        // it cannot tell that one was missed. None of them tries to be correct about Rust, only
        // about "there is an item on this line".

        /** A struct member: a name, a colon, and something after it, `pub` or not. */
        const val FIELD_LINE = """^\s*(?:pub(?:\([^)]*\))?\s+)?\w+\s*:\s*\S"""

        /** An enum variant: a word at the start of the line, with or without a payload or value. */
        const val VARIANT_LINE = """^\s*\w+\s*[({=,]"""

        /**
         * A match arm: any line carrying `=>`, except a wildcard one. The wildcard is excluded
         * because every selector here enumerates the NAMED arms and the catch-all is the absence of
         * a name, so demanding it be read would be a false red on every match in the file.
         */
        const val ARM_LINE = """^(?!\s*_\s*=>).*=>"""

        /**
         * A supervision timeout declaration: a public line that names one and types it, with no
         * claim about `const`, about the type's shape, or about the value.
         *
         * It keeps [TIMEOUT_CONST]'s `pub` requirement rather than dropping it, because that
         * narrowing is a DECISION there and not an oversight: a module-private constant is an
         * internal detail of the crate, and demanding a mirror for one would fail this gate on a
         * refactor no consumer can observe. The census is dumber than the selector everywhere the
         * selector is not deliberate: a `pub static`, a generic or commented type, or a value this
         * gate cannot read now fails by name instead of dropping out of the exact-set comparison.
         */
        const val TIMEOUT_DECL_LINE = """^\s*pub\b.*\b\w+_TIMEOUT_TICKS\s*:"""

        /** An opcode declaration, on the same rule as [TIMEOUT_DECL_LINE]. */
        const val OPCODE_DECL_LINE = """^\s*pub\b.*\bOP_\w+\s*:"""

        /**
         * A single-pin take in the validator: any mention of a staged field as an argument, with no
         * claim about the brackets around it, so a row wrapped across lines is still censused.
         */
        const val FIELD_TAKE_LINE = """\bfields\.\w+\s*,"""

        /**
         * A per-motor take. Unlike [FIELD_TAKE_LINE] this one keeps the opening bracket, because
         * `mf.<field>` is also how the motor plan's own rows are built (`dead_time: mf.dead_time,`)
         * and a census without it would demand those be read as takes. The cost is that a per-motor
         * tuple wrapped across lines falls outside the census; what covers that is the count check
         * below, against the number of pin slots the layout carries.
         */
        const val MOTOR_TAKE_LINE = """\(\s*mf\.\w+\s*,"""

        /**
         * A row of a `field_ids!` block: a line that does not open with a comment, OR any line
         * carrying a hex literal and a comma anywhere in it. The block holds nothing but rows and
         * comments, so this is the strictest census in the file and the right one: the registry is
         * what every other field gate's authority rests on.
         *
         * The first branch catches a row written as a named constant, in decimal, or with no handle
         * comment. The second exists because the first reads only the START of the line, and a row
         * whose id sits behind a one-line block-comment prefix starts like a comment: that shape
         * escaped both the selector and the census, and took a registered, unmirrored field green
         * through the whole suite.
         *
         * The second branch takes ANY identifier or digit run before a comma rather than a `0x`
         * literal, because the sibling shapes (a decimal id, a named constant) behind the same
         * prefix escaped it too. They were caught either way, by the declared-count check below, so
         * what this buys is the diagnosis: the failure quotes the ROW instead of saying two numbers
         * do not add up.
         */
        const val REGISTRY_ROW_LINE = """^\s*[^/\s]|\w+\s*,"""

        /** A registered-field read: any mention of a `store::` handle. */
        const val STORE_READ_LINE = """store::\w+"""

        /** The selector for a supervision timeout declaration; see [rustSupervisionTimeouts]. */
        const val TIMEOUT_CONST =
            """^\s*pub(?:\((?:crate|super|in [^)]+)\))?\s+const\s+(\w+_TIMEOUT_TICKS)\s*:\s*[\w:]+\s*=\s*([^;]+);"""

        /**
         * The selector for an L7 opcode declaration; see [linkctlOpcodesAgreeWithTheRustSource].
         *
         * Scoped by the `OP_` prefix rather than by type or column, which is why it can afford to
         * take any indentation and any type: those four are the only `OP_`-named constants in the
         * crate, so nothing unrelated can be swept in and made a false failure.
         */
        const val OPCODE_CONST =
            """^\s*pub(?:\((?:crate|super|in [^)]+)\))?\s+const\s+OP_(\w+)\s*:\s*[\w:]+\s*=\s*([^;]+);"""

        /**
         * The selector for a `u8` wire constant declared at module level, used for the L3 address
         * constants in pdu.rs and the walk wire constants in walk.rs.
         *
         * Two deliberate narrowings, because here the TYPE is what scopes the set rather than a name
         * prefix. It stays `u8`, since that is what separates a wire value from `walk.rs`'s `usize`
         * buffer capacities, which are firmware sizing and not mirrored. And it stays anchored at
         * column 0, so a `u8` const inside a `mod tests` cannot be dragged in and demanded of the
         * mirror. Both bound what this catches: a wire constant added at module level in any spacing
         * or `pub` form fails until the Kotlin carries it, one added inside a nested module does not.
         */
        const val U8_WIRE_CONST =
            """^pub(?:\((?:crate|super|in [^)]+)\))?\s+const\s+(\w+)\s*:\s*u8\s*=\s*([^;]+);"""
    }
}
