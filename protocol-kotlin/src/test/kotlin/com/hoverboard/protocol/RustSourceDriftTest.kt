package com.hoverboard.protocol

import com.hoverboard.protocol.l2.FragHdr
import com.hoverboard.protocol.l2.StreamFrame
import com.hoverboard.protocol.l3.BROADCAST
import com.hoverboard.protocol.l3.GUEST_FIRST
import com.hoverboard.protocol.l3.GUEST_LAST
import com.hoverboard.protocol.l3.HEADER_LEN
import com.hoverboard.protocol.l3.NO_ADDRESS
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.linkctl.CYCLIC_TIMEOUT_TICKS
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
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Type
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertThrows
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

    /** Declared field order of a struct, as (name, rustType) pairs, doc comments skipped. */
    private fun fields(text: String, name: String): List<Pair<String, String>> =
        findAll(structBlock(text, name), """^\s+pub\s+(\w+)\s*:\s*(\w+),$""", "$name fields")
            .map { it.groupValues[1] to it.groupValues[2] }

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
        findAll(linkctl, TIMEOUT_CONST, "supervision timeouts").associate {
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
        val fromRust = findAll(linkctl, OPCODE_CONST, "linkctl opcodes")
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
        val pdu = rust("crates/net/src/pdu.rs")
        val enumStart = pdu.indexOf("pub enum Opcode {")
        val body = pdu.substring(enumStart, pdu.indexOf("\n}", enumStart))
        val fromRust = findAll(body, """^\s+(\w+)\s*=\s*([^,]+),""", "L3 opcodes")
            .associate { it.groupValues[1] to literal(it.groupValues[1], it.groupValues[2], "L3 opcode") }

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
     * deliberately not mirrored, so the pattern selects on the `u8` TYPE and takes whatever value
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

        val fromRust = findAll(
            tagFn,
            """^\s+Type::(\w+)\s*=>\s*([^,]+),""",
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
    }

    /**
     * Field ORDER and width, read out of the Rust struct declarations.
     *
     * This is the pin that catches the exact break the retired rider protocol had: a `battery` u16
     * inserted mid-struct in CyclicState. Every field keeps its name and every length stays the
     * same under a reorder, so only order-aware comparison sees it.
     *
     * The widths also have to add up to the committed LEN, which is checked here so a field that
     * changes type cannot slip through.
     */
    @Test
    fun payloadFieldOrderAgreesWithTheRustSource() {
        val widths = mapOf("i16" to 2, "u16" to 2, "u8" to 1, "DriveKind" to 1)

        val expected = mapOf(
            "CyclicState" to listOf(
                "pitch" to "i16", "roll" to "i16", "wheelSpeed" to "i16", "battery" to "u16",
                "mode" to "u8", "fault" to "u8", "flags" to "u8",
            ),
            "DriveCmd" to listOf("kind" to "DriveKind", "value" to "i16", "steer" to "i16"),
            "Inputs" to listOf("throttle" to "i16", "buttons" to "u8", "rider" to "u8"),
            "Fault" to listOf("code" to "u8", "action" to "u8"),
        )
        val lens = mapOf(
            "CyclicState" to CyclicState.LEN,
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
     * crates/store/src/field.rs, and their ranges against `GAIN_RANGE` in
     * crates/control/src/config.rs (`specs/rider-ui.md` section 4).
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

        // `pub const GAIN_RANGE: [(i16, i16); GAINS_PER_PROFILE] = [(0, 20000), ...];`
        val rangeSrc = findOne(
            rust("crates/control/src/config.rs"),
            """^pub const GAIN_RANGE: \[\(i16, i16\); \w+\] = \[([^\]]+)\];""",
            "GAIN_RANGE",
        ).groupValues[1]
        val ranges = Regex("""\(([^,]+),([^)]+)\)""").findAll(rangeSrc).map {
            literal("GAIN_RANGE", it.groupValues[1], "gain range")..literal("GAIN_RANGE", it.groupValues[2], "gain range")
        }.toList()
        assertEquals(ranges, Gains.RANGE, "the tune seam's ranges drifted")
        assertEquals(Gains.PER_PROFILE, Gains.RANGE.size)

        // The index names the Kotlin exposes are the positions the Rust triple is written in, and
        // every default is inside its own range (a default a client would refuse to send is a bug
        // in one of the two files this test reads).
        assertEquals(listOf(Gains.KP, Gains.BK, Gains.PR), listOf(0, 1, 2))
        for (i in 0 until Gains.PER_PROFILE) {
            assertTrue(Gains.inRange(i, Gains.DEFAULT_A[i]), "profile A default $i is outside its range")
            assertTrue(Gains.inRange(i, Gains.DEFAULT_B[i]), "profile B default $i is outside its range")
        }
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
     * The CRC the firmware actually instantiates. `crates/base/src/crc16.rs` builds its CRC from
     * the `crc` crate's named `CRC_16_MODBUS` algorithm, so the pin is that the name has not been
     * swapped for a different one; the known-answer vectors in [WireDriftTest] pin the arithmetic.
     */
    @Test
    fun crcAlgorithmAgreesWithTheRustSource() {
        val crc = rust("crates/base/src/crc16.rs")
        findOne(crc, """Crc::<u16>::new\(&(\w+)\)""", "CRC algorithm").groupValues[1].let {
            assertEquals("CRC_16_MODBUS", it, "the firmware's CRC algorithm changed")
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
