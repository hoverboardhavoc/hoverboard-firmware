package com.hoverboard.remote.model

import kotlin.math.roundToInt

/**
 * State of charge from pack voltage, per series cell.
 *
 * The endpoints are a cell's, not a pack's: 4.2 V full and 3.0 V empty, which is the band the
 * machines here run their lithium packs over (the rover's LEV50 module is eight of them, about 24
 * to 33.6 V). A pack's endpoints are those multiplied by how many cells it has, which the board
 * stores and nothing else can work out ([com.hoverboard.protocol.store.Fields.BOARD_PACK_CELLS]).
 * It replaces a 29.4 V / 21.7 V pair compiled into this file, which described a seven-cell pack no
 * machine here runs and pinned both of them at 100%.
 *
 * **The map between the endpoints is linear, and that is still a placeholder.** A lithium cell is
 * flat through the middle and steep at both ends, so this over-reads the top of the discharge and
 * under-reads the bottom. Fitting the real shape needs a logged discharge of the pack in question
 * and nobody has run one; the endpoints were the part that made the gauge useless, and the shape
 * is tracked separately in `specs/todo.md`, the open model gaps.
 */
object BatteryCurve {

    /** A full cell, volts. Eight of these is the rover pack's 33.6 V. */
    const val FULL_VOLTS_PER_CELL: Float = 4.2f

    /** An empty cell, volts: the same 3.0 V per cell the firmware's low-battery floor defaults to. */
    const val EMPTY_VOLTS_PER_CELL: Float = 3.0f

    /** Battery is considered low at or below this fraction of remaining charge. */
    const val LOW_FRACTION: Float = 0.15f

    /** A full pack of [cells] series cells, volts. */
    fun fullVolts(cells: Int): Float = cells * FULL_VOLTS_PER_CELL

    /** An empty pack of [cells] series cells, volts. */
    fun emptyVolts(cells: Int): Float = cells * EMPTY_VOLTS_PER_CELL

    /** Remaining charge in [0f, 1f] for [volts] across [cells] cells, clamped to the endpoints. */
    fun fraction(volts: Float, cells: Int): Float {
        val empty = emptyVolts(cells)
        return ((volts - empty) / (fullVolts(cells) - empty)).coerceIn(0f, 1f)
    }
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
     * A measured pack voltage with no pack to judge it against: nothing has stated how many cells
     * this machine runs, so there is a number and no state of charge.
     *
     * Not a gauge at an assumed pack. The same 30 V is a flat ten-cell pack and an over-charged
     * eight-cell one, and a percentage drawn from a guess is the defect this type exists to end.
     */
    data class Unjudged(val volts: Float) : BatteryReading

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
 * What [telemetry] and that machine's own [packCells] (`board.pack_cells`) come to.
 *
 * The cell count is read from the board rather than compiled in, because the two machines here run
 * different packs and a client cannot derive one from a voltage: a resting 30 V is an eight-cell
 * pack above full and a ten-cell pack nearly empty. Absent or 0, the pack has not been stated and
 * the voltage is reported [BatteryReading.Unjudged] rather than judged against a guess.
 */
fun batteryReading(telemetry: TelemetryUi?, packCells: Int?): BatteryReading {
    val volts = telemetry?.batteryVolts ?: return BatteryReading.NoReading
    if (packCells == null || packCells <= 0) return BatteryReading.Unjudged(volts)
    val fraction = BatteryCurve.fraction(volts, packCells)
    return BatteryReading.Charge(
        volts = volts,
        fraction = fraction,
        low = fraction <= BatteryCurve.LOW_FRACTION,
    )
}
