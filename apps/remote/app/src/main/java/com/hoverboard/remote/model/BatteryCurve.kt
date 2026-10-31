package com.hoverboard.remote.model

import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.BatteryReading.Unjudged.Why
import kotlin.math.roundToInt

/**
 * State of charge from a resting pack voltage, per series cell and per chemistry
 * (`specs/remote-ui.md` 3.6d).
 *
 * Each chemistry is a table of eleven resting cell voltages at 0, 10, ... 100 % charge, read
 * linearly between points and clamped at the ends. A pack's voltages are the table's multiplied
 * by how many cells it has ([com.hoverboard.protocol.store.Fields.BOARD_BATTERY_CELLS]), and which
 * table is the pack's own [com.hoverboard.protocol.store.Fields.BOARD_BATTERY_CHEMISTRY]. The
 * points are held in centivolts, the battery word's unit, so a pack's endpoints are exact.
 *
 * **The tables are typical datasheet shapes, not measurements of these packs.** A logged discharge
 * replaces a row when one is run (`specs/todo.md`, the open model gaps). They read RESTING voltage: under load a pack
 * sags and the gauge under-reads.
 */
object BatteryCurve {

    /** Battery is considered low at or below this fraction of remaining charge. */
    const val LOW_FRACTION: Float = 0.15f

    /** Each chemistry's resting cell voltage at 0, 10, ... 100 %, centivolts. */
    private val TABLES: Map<Int, IntArray> = mapOf(
        Fields.BatteryChemistry.LI_ION to
            intArrayOf(300, 345, 355, 362, 368, 374, 380, 388, 397, 407, 420),
        Fields.BatteryChemistry.LEV50 to
            intArrayOf(275, 350, 358, 363, 368, 373, 379, 386, 394, 402, 410),
        Fields.BatteryChemistry.LTO to
            intArrayOf(170, 215, 222, 226, 229, 232, 236, 240, 245, 252, 270),
    )

    /**
     * A pack of [cells] cells of [chemistry] from empty to full, centivolts, or null for a
     * chemistry this app has no table for.
     */
    fun packCentivolts(cells: Int, chemistry: Int): IntRange? =
        TABLES[chemistry]?.let { cells * it.first()..cells * it.last() }

    /**
     * Remaining charge in [0f, 1f] for [volts] across [cells] cells of [chemistry], clamped at the
     * table's ends, or null for a chemistry this app has no table for.
     */
    fun fraction(volts: Float, cells: Int, chemistry: Int): Float? {
        val table = TABLES[chemistry] ?: return null
        val perCell = volts * CENTIVOLTS_PER_VOLT / cells
        if (perCell <= table.first()) return 0f
        if (perCell >= table.last()) return 1f
        val upper = table.indexOfFirst { it > perCell }
        val lo = table[upper - 1]
        val step = (perCell - lo) / (table[upper] - lo)
        return (upper - 1 + step) / (table.size - 1)
    }

    private const val CENTIVOLTS_PER_VOLT = 100f
}

/**
 * Whether [floor] (`board.battery.floor`, centivolts) sits outside the stated pack: at or above
 * its full voltage, or below its empty one (`specs/remote-ui.md` 3.6d). Null when there is nothing
 * to judge: no floor (`<= 0`), no stated cell count, or a chemistry this app has no table for.
 */
fun floorOutsidePack(floor: Int?, cells: Int?, chemistry: Int?): FloorOutside? {
    if (floor == null || floor <= 0) return null
    if (cells == null || cells <= 0) return null
    val pack = chemistry?.let { BatteryCurve.packCentivolts(cells, it) } ?: return null
    return when {
        floor >= pack.last -> FloorOutside.AT_OR_ABOVE_FULL
        floor < pack.first -> FloorOutside.BELOW_EMPTY
        else -> null
    }
}

/** Which side of the stated pack a low-battery floor falls on ([floorOutsidePack]). */
enum class FloorOutside {
    /** No pack of this kind rests above the floor, so every balance engage is refused. */
    AT_OR_ABOVE_FULL,

    /** The floor is under an empty pack, so it never refuses an engage before the pack is flat. */
    BELOW_EMPTY,
}

/**
 * What one board's battery display says right now, the shape [CurrentReading] has and for the same
 * reason: a figure the app can only produce by assuming something about the machine is not a
 * figure, and the ways it can be absent are different facts an owner acts on differently.
 */
sealed interface BatteryReading {

    /** The board is reporting no pack voltage ([TelemetryUi.batteryVolts] has the firmware side). */
    data object NoReading : BatteryReading

    /**
     * A measured pack voltage with no pack to judge it against, for the reason [why]: there is a
     * number and no state of charge.
     *
     * Not a gauge at an assumed pack. The same 30 V is a flat ten-cell pack and an over-charged
     * eight-cell one, and a percentage drawn from a guess is the defect this type exists to end.
     */
    data class Unjudged(val volts: Float, val why: Why) : BatteryReading {
        /** What is missing. */
        enum class Why {
            /** Nothing has stated how many cells this machine runs. */
            NO_CELLS,

            /** The chemistry has not been read from the board yet. */
            NO_CHEMISTRY,

            /** The stated chemistry is a byte this app has no table for. */
            UNKNOWN_CHEMISTRY,
        }
    }

    /**
     * A measured pack voltage against the stated pack: the voltage, how full it is and whether
     * that is [BatteryCurve.LOW_FRACTION] or less.
     */
    data class Charge(val volts: Float, val fraction: Float, val low: Boolean) : BatteryReading {
        /** [fraction] as a whole percent, which is what the panel prints beside the voltage. */
        val percent: Int get() = (fraction * PERCENT_SCALE).roundToInt()

        private companion object {
            const val PERCENT_SCALE = 100
        }
    }
}

/**
 * What [telemetry] and that machine's own [packCells] (`board.battery.cells`) and [chemistry]
 * (`board.battery.chemistry`) come to.
 *
 * Both are read from the board rather than compiled in, because the machines here run different
 * packs and a client cannot derive one from a voltage: a resting 30 V is an eight-cell pack above
 * full and a ten-cell pack nearly empty. A cell count that is absent or 0, or a chemistry that is
 * unread or unknown, leaves the voltage [BatteryReading.Unjudged] rather than judged against a
 * guess.
 */
fun batteryReading(telemetry: TelemetryUi?, packCells: Int?, chemistry: Int?): BatteryReading {
    val volts = telemetry?.batteryVolts ?: return BatteryReading.NoReading
    val fraction = chemistry?.let { packCells?.takeIf { it > 0 }?.let { BatteryCurve.fraction(volts, it, chemistry) } }
    return when {
        packCells == null || packCells <= 0 -> BatteryReading.Unjudged(volts, Why.NO_CELLS)
        chemistry == null -> BatteryReading.Unjudged(volts, Why.NO_CHEMISTRY)
        fraction == null -> BatteryReading.Unjudged(volts, Why.UNKNOWN_CHEMISTRY)
        else -> BatteryReading.Charge(
            volts = volts,
            fraction = fraction,
            low = fraction <= BatteryCurve.LOW_FRACTION,
        )
    }
}
