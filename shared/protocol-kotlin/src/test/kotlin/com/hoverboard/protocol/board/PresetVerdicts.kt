package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Key
import java.io.File

/**
 * The boot verdict over a batch of staged layouts, read from a file and written to a file, for the
 * host-side preset generator (`tools/robo-presets.py`).
 *
 * **Why the generator asks Kotlin rather than Rust.** The verdict is `validate`, a pure function of
 * the staged fields, the part's capability answers and the reserved set. This module already holds
 * all three for the fleet's parts in ORDINARY code: `validate` (a mirror of `validate` in
 * `crates/board/src/lib.rs`), [ChipFamily] (the only non-test implementation of [Capabilities]
 * anywhere in the tree) and [reservedSet]. The Rust equivalents of the capability tables are
 * `#[cfg(test)]`-private to the board crate, so reaching the Rust validator from the host would mean
 * writing a SECOND capability table for the fleet's parts, and a fact that is already modelled does
 * not get a second copy. The chain that makes this verdict trustworthy is two tests that already
 * exist: `RustSourceDriftTest` pins this mirror to the Rust, and the board crate's
 * `rcap_agreement` suite pins the Rust's mock tables to runtime-hal's real R-CAP queries on every
 * encodable pin. It is also exactly the verdict the rider app will show for the same preset
 * (`specs/rider-ui.md`, section 3.5), which is the verdict the consumer cares about.
 *
 * It lives in the TEST source set deliberately: it is host-side verification tooling, so it must not
 * ride in the library artifact the two Android apps consume. It parses a line format of its own
 * rather than the preset JSON, because the preset parser belongs to the app module
 * (`specs/robo-presets.md`, "The output: JSON, and its schema": `org.json` is an Android class and
 * this is a plain JVM module).
 *
 * Request line, tab separated: `<name>  <ChipFamily>  <linkSet>  <fieldId>:<index>=<raw>,...`
 * Reply line, tab separated and one per request, in request order:
 * `<name>  <ChipFamily>  <accepted|refused>  <selfHold>  <obsResult>  <field>  <motor>  <reason>`
 * where `-` is an empty column, `selfHold` is the pin the boot would drive or `-` for none, and a
 * refusal names the first failing field exactly as `BOARD_OBS` would report it.
 */
object PresetVerdicts {

    /** The column separator of both the request and the reply lines. */
    private const val SEP = '\t'

    /** An empty column. */
    private const val NONE = "-"

    /** The staged layout one request line describes. */
    private data class Request(
        val name: String,
        val part: ChipFamily,
        val linkSet: Int,
        val fields: BoardFields,
    )

    /**
     * The staged layout [line] describes.
     *
     * Every `<fieldId>:<index>` must name a layout slot ([Layout.forKey]), so a field the validator
     * does not judge is a hard error rather than a silently dropped value: the generator decides
     * what the verdict is a verdict ON, and a dropped field would make it a verdict on less.
     */
    private fun request(line: String): Request {
        val col = line.split(SEP)
        require(col.size == 4) { "a request line has 4 columns, got ${col.size}: $line" }
        var fields = BoardFields()
        if (col[3].isNotEmpty()) {
            for (entry in col[3].split(',')) {
                val (key, raw) = entry.split('=', limit = 2)
                val (id, index) = key.split(':', limit = 2)
                val slot = Layout.forKey(Key(id.toInt(), index.toInt()))
                    ?: error("field ${id.toInt()}:${index.toInt()} names no layout slot")
                fields = slot.on(fields, raw.toInt())
            }
        }
        return Request(col[0], ChipFamily.valueOf(col[1]), col[2].toInt(), fields)
    }

    /** The reply line for [req]: its verdict, rendered in the reply columns. */
    private fun reply(req: Request): String {
        val v = validate(req.fields, req.part, reservedSet(req.part, req.linkSet))
        val latch = v.selfHold?.name ?: NONE
        val err = v.error
        val col = if (err == null) {
            listOf("accepted", latch, "0", NONE, NONE, NONE)
        } else {
            listOf(
                "refused",
                latch,
                err.kind.obsResult.toString(),
                err.field.field.name,
                err.field.motor?.toString() ?: NONE,
                reason(err.kind),
            )
        }
        return (listOf(req.name, req.part.name) + col).joinToString(SEP.toString())
    }

    /**
     * A refusal in words, naming the pin where one is involved.
     *
     * Spelled here rather than taken from `toString()` because a data class's `toString` is a
     * Kotlin implementation detail and this string goes into a generated file that is diffed
     * between runs.
     */
    private fun reason(kind: BoardErrorKind): String = when (kind) {
        is BoardErrorKind.BadEncoding -> "BadEncoding(0x%02X)".format(kind.raw)
        BoardErrorKind.IncompleteGroup -> "IncompleteGroup"
        BoardErrorKind.DeadTimeBelowFloor -> "DeadTimeBelowFloor(min=$DEAD_TIME_MIN_DTG)"
        is BoardErrorKind.DuplicatePin -> "DuplicatePin(${kind.pin.name})"
        is BoardErrorKind.ReservedPin -> "ReservedPin(${kind.pin.name})"
        is BoardErrorKind.UnknownPin -> "UnknownPin(${kind.pin.name})"
        is BoardErrorKind.GateCapableMisused -> "GateCapableMisused(${kind.pin.name})"
        BoardErrorKind.InvalidGateSet -> "InvalidGateSet"
        is BoardErrorKind.NotAdcCapable -> "NotAdcCapable(${kind.pin.name})"
        BoardErrorKind.NotI2cPair -> "NotI2cPair"
        is BoardErrorKind.ImuFrame -> "ImuFrame(${kind.firstSignIndex})"
    }

    @JvmStatic
    fun main(args: Array<String>) {
        require(args.size == 2) { "usage: PresetVerdicts <requests> <replies>" }
        val replies = File(args[0]).readLines()
            .filter { it.isNotBlank() }
            .map { reply(request(it)) }
        File(args[1]).writeText(replies.joinToString("\n", postfix = "\n"))
    }
}
