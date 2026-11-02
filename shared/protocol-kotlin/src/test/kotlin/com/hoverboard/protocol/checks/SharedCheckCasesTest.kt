package com.hoverboard.protocol.checks

import com.google.gson.JsonArray
import com.google.gson.JsonElement
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import com.hoverboard.protocol.pincap.Chip
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Names
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value
import java.io.File
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The shared check cases (`shared/config-checks/`, `specs/config-read-once.md` section 4), run
 * against the Kotlin port of the checks, as `tools/hoverboard-cli/src/checks/tests.rs, every_shared_case_gets_its_verdict` runs them
 * against the CLI's. One file per check, named by its id, each a list of cases: a `name`, the
 * `chip`, a `board` settings fragment, and `refused`, the path of the key the first refusal names,
 * or null for a board every check passes. A refused case is refused by its file's check alone.
 *
 * The case files and [Check] are compared as sets, so a check one implementation adds without a
 * case file, or a case file for a check this port lacks, fails here.
 */
class SharedCheckCasesTest {

    private val dir: File by lazy {
        var d: File? = File(System.getProperty("user.dir")).absoluteFile
        while (d != null && !File(d, "shared/config-checks").isDirectory) d = d.parentFile
        File(checkNotNull(d) { "no shared/config-checks above ${System.getProperty("user.dir")}" }, "shared/config-checks")
    }

    /** A settings fragment as the board would hold it, and the `af` form of each `PinAf` it gives. */
    private class Board(val state: MutableMap<Key, Value> = linkedMapOf(), val forms: MutableList<PinForm> = mutableListOf())

    /** Read the board file object [top] into [into] (the CLI's `read_files`, with no problem reporting). */
    private fun read(top: JsonObject, into: Board = Board()): Board {
        for ((k, v) in top.entrySet()) {
            val array = Names.IndexKind.entries.mapNotNull { Names.arrayOf(it) }.firstOrNull { it == k }
            if (array != null) {
                v.asJsonArray.forEachIndexed { i, item -> obj(item.asJsonObject, "", array to i, into) }
            } else {
                member(k, v, "", null, into)
            }
        }
        return into
    }

    private fun obj(o: JsonObject, prefix: String, elem: Pair<String, Int>?, into: Board) {
        for ((k, v) in o.entrySet()) member(k, v, prefix, elem, into)
    }

    private fun member(k: String, v: JsonElement, prefix: String, elem: Pair<String, Int>?, into: Board) {
        val path = if (prefix.isEmpty()) k else "$prefix.$k"
        val array = elem?.first
        val row = Names.find(path, array)
        if (row != null) {
            if (row.index == Names.IndexKind.ARRAY) {
                v.asJsonArray.forEachIndexed { i, item -> add(Key(row.id, i), item, into) }
            } else {
                add(Key(row.id, elem?.second ?: 0), v, into)
            }
            return
        }
        val nests = Names.ALL.any { Names.arrayOf(it.index) == array && it.path.startsWith("$path.") }
        check(v.isJsonObject && nests) { "$path${array?.let { " in $it" } ?: ""} is no field of the name table" }
        obj(v.asJsonObject, path, elem, into)
    }

    /** The value [v] gives [key], encoded by the name table's kind into the field's storage type. */
    private fun add(key: Key, v: JsonElement, into: Board) {
        if (v.isJsonNull) return
        val row = checkNotNull(Names.byId(key.fieldId))
        val type = checkNotNull(Fields.typeOf(key.fieldId))
        into.state[key] = when (row.kind) {
            Names.Kind.PIN -> pin(key, type, v.asJsonObject, into)
            Names.Kind.STR -> Value.Str(v.asString)
            Names.Kind.BOOL -> integer(type, if (v.asBoolean) 1 else 0)
            Names.Kind.ENUM -> integer(
                type,
                if (v.isJsonPrimitive && v.asJsonPrimitive.isString) {
                    row.labels.single { it.second == v.asString }.first.toLong()
                } else {
                    v.asLong
                },
            )
            Names.Kind.BITS -> integer(
                type,
                if (v is JsonArray) {
                    v.fold(0L) { m, b -> m or (1L shl row.labels.single { it.second == b.asString }.first) }
                } else {
                    v.asLong
                },
            )
            Names.Kind.INT -> integer(type, v.asLong)
        }
    }

    private fun pin(key: Key, type: Type, o: JsonObject, into: Board): Value {
        val port = o.get("port").asString.single().uppercaseChar() - 'A'
        val pin = o.get("pin").asInt
        return when (type) {
            Type.Pin -> Value.Pin(port, pin)
            Type.PinAf -> {
                into.forms += PinForm(key, o.has("af"))
                Value.PinAf(port, pin, o.get("af")?.asInt ?: 0)
            }
            Type.PinAdc -> Value.PinAdc(port, pin, o.get("adc").asInt)
            else -> error("$type is not a pin type")
        }
    }

    private fun integer(type: Type, n: Long): Value = when (type) {
        Type.U8 -> Value.U8(n.toInt())
        Type.U16 -> Value.U16(n.toInt())
        Type.U32 -> Value.U32(n)
        Type.I16 -> Value.I16(n.toInt())
        Type.I32 -> Value.I32(n.toInt())
        else -> error("$type is not an integer type")
    }

    private fun refusals(chip: Chip, board: Board): List<Refusal> = Checks.run(chip, board.state, board.forms)

    @Test
    fun everySharedCaseGetsItsVerdict() {
        for (check in Check.entries) {
            val file = File(dir, "${check.id}.json")
            val cases = JsonParser.parseString(file.readText()).asJsonArray
            var passes = 0
            var refuses = 0
            for (c in cases.map { it.asJsonObject }) {
                val name = c.get("name").asString
                val chip = checkNotNull(Chip.fromId(c.get("chip").asString)) { "${check.id}: $name names no chip" }
                val got = refusals(chip, read(c.get("board").asJsonObject))
                val refused = c.get("refused")
                if (refused.isJsonNull) {
                    assertEquals(emptyList<Refusal>(), got, "${check.id}: $name")
                    passes++
                } else {
                    assertTrue(got.isNotEmpty(), "${check.id}: $name: nothing refused")
                    assertTrue(got.all { it.check == check }, "${check.id}: $name: $got")
                    assertEquals(refused.asString, Names.pathOf(got.first().key), "${check.id}: $name")
                    refuses++
                }
            }
            assertTrue(passes > 0 && refuses > 0, "${check.id} needs a passing and a refused case")
        }
    }

    @Test
    fun theCaseFilesAreExactlyTheChecks() {
        val files = dir.listFiles { f -> f.isFile && f.name.endsWith(".json") }.orEmpty().map { it.name }.sorted()
        assertEquals(Check.entries.map { "${it.id}.json" }.sorted(), files)
    }

    /**
     * `specs/config-read-once.md` section 9: the rover and offroad machine files pass on their
     * boards' chips, and each board's file is refused on the other chip.
     */
    @Test
    fun theFleetsMachineFilesPassOnTheirChipsAndNotOnTheOther() {
        val roverMaster = "8adc39323335381255343333"
        for (name in listOf("rover", "offroad")) {
            val top = JsonParser.parseString(File(dir, "machines/$name.json").readText()).asJsonObject
            val machine = JsonObject().apply { add("machine", top.get("machine")) }
            val boards = top.getAsJsonObject("boards").entrySet()
            assertTrue(boards.isNotEmpty())
            for ((uid, entry) in boards) {
                val (chip, other) = if (uid == roverMaster) {
                    Chip.GD32F103C8 to Chip.GD32F130C8
                } else {
                    Chip.GD32F130C8 to Chip.GD32F103C8
                }
                fun board() = read(entry.asJsonObject, read(machine))
                assertEquals(emptyList<Refusal>(), refusals(chip, board()), "$name $uid")
                val wrong = refusals(other, board())
                assertTrue(wrong.any { it.check == Check.AF_FORM }, "$name $uid on the ${other.id}: $wrong")
            }
        }
    }
}
