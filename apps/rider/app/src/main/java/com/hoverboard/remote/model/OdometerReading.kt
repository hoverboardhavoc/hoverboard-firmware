package com.hoverboard.remote.model

import kotlin.math.PI
import kotlin.math.roundToLong

/**
 * One board's wheel geometry: what it takes to turn that wheel's counted hall edges into a
 * distance (`specs/rider-ui.md` 3.6c).
 *
 * Both halves are stored fields the app READS and never writes: the motor's pole-pair count and
 * the wheel's rolling diameter in millimetres. They are measured on the machine and written at a
 * bench with the host config tool, so a board that has not been through that bench has neither,
 * and both are unset on a fresh board.
 *
 * **Both or nothing**, which is what [of] enforces: a diameter with no pole-pair count converts
 * nothing, and a pole-pair count defaulted to some believed fleet value would scale every figure
 * by whatever the real hub turns out to be. A wrong distance stated confidently is worse than no
 * distance, so a half-told board shows what it counted.
 */
data class WheelGeometry(val polePairs: Int, val diameterMm: Int) {

    /**
     * What [edges] of this wheel come to, in whole millimetres.
     *
     * Six hall edges per ELECTRICAL revolution and [polePairs] electrical revolutions per
     * mechanical one, so `6 * polePairs` edges make one turn of the wheel, each worth one
     * circumference of `PI * diameterMm`.
     *
     * Millimetres in a Long because the counts are lifetime totals: a saturated 32-bit edge count
     * on a 6.5 inch hub is about 2.5e10 mm, ten times what an Int holds, and a Long carries it
     * whole.
     */
    fun millimetres(edges: Long): Long =
        (edges.toDouble() * PI * diameterMm / (EDGES_PER_ELECTRICAL_REV * polePairs)).roundToLong()

    companion object {
        /** Hall edges per electrical revolution: the six-state commutation sequence. */
        private const val EDGES_PER_ELECTRICAL_REV = 6

        /**
         * This board's geometry, or null when it cannot convert.
         *
         * Null for a half-told board and for a board told a zero, which is the same thing: 0 is
         * the unset value of both fields, 0 is not a motor and 0 mm is not a wheel. Guarding the
         * zero here is also what keeps [millimetres]' divisor off zero, since nothing else between
         * the stored value and the arithmetic would.
         */
        fun of(polePairs: Int?, diameterMm: Int?): WheelGeometry? {
            if (polePairs == null || polePairs <= 0) return null
            if (diameterMm == null || diameterMm <= 0) return null
            return WheelGeometry(polePairs, diameterMm)
        }
    }
}

/**
 * What one board's odometer display says right now (`specs/rider-ui.md` 3.6c).
 *
 * A distance, a count of hall edges, or nothing at all, and there is only one way to have
 * nothing: a board that has sent no state. Zero is NOT that way. Zero edges is a real and sayable
 * reading, a board whose wheel has never turned, and folding it into the absent case would hide a
 * board that is reporting perfectly well.
 */
sealed interface OdometerReading {

    /** Nothing has arrived from this board ([TelemetryUi.hasState]). */
    data object Silent : OdometerReading

    /**
     * This wheel's lifetime travel, in ABSOLUTE hall edges: every counted commutation edge adds 1
     * whichever way the wheel turned, so the figure never winds back.
     *
     * EDGES, and no distance, because this board has not been told the geometry one needs: its
     * motor's pole-pair count, its wheel's diameter, or both are unset ([WheelGeometry]).
     */
    data class Edges(val edges: Long) : OdometerReading

    /**
     * The same lifetime travel, converted against THIS board's own staged geometry: the edges it
     * counted and the [millimetres] they cover.
     *
     * The edges are kept beside the distance because they are what the board actually recorded;
     * the millimetres are a reading of them through a wheel size somebody measured and typed.
     */
    data class Travelled(val edges: Long, val millimetres: Long) : OdometerReading
}

/**
 * What [telemetry] reports for its own wheel, converted when [geometry] is this board's and
 * complete, and left in edges when it is not.
 */
fun odometerReading(telemetry: TelemetryUi?, geometry: WheelGeometry? = null): OdometerReading {
    val edges = telemetry?.odometerEdges ?: return OdometerReading.Silent
    return if (geometry == null) {
        OdometerReading.Edges(edges)
    } else {
        OdometerReading.Travelled(edges, geometry.millimetres(edges))
    }
}

/**
 * The MACHINE's travel: the MEAN of the boards' figures, not their sum.
 *
 * Both wheels cover the same ground, so adding them reports twice the travel. That is the opposite
 * of the current section, which sums because the two boards draw from one pack and their draws
 * genuinely add.
 *
 * **Both or neither, in the unit as well as in the figure.** The mean is a distance when every
 * reporting board converts and an edge count when none does; a pair where one board has been told
 * its wheel and the other has not gets an edge mean. Two rows carrying different units cannot be
 * averaged into one number, and averaging the edges of wheels that may differ in size is the
 * weaker of the two claims rather than a wrong one.
 *
 * Null when any board has not reported. A one-wheel mean is not a machine's travel, and nothing
 * distinguishes a second board that has not sent a sample yet from one that is not fitted, so the
 * figure is withheld rather than taken from the wheel that did report.
 *
 * The two wheels need not agree, and a difference between them is a real fact about the machine:
 * one wheel may have been fitted later, or spun on a stand. The mean is what the pair covered
 * together as far as the counts can say.
 */
fun machineOdometer(boards: List<OdometerReading>): MachineTravel? {
    if (boards.isEmpty()) return null
    var edges = 0L
    var millimetres = 0L
    var converted = true
    for (board in boards) {
        when (board) {
            is OdometerReading.Travelled -> {
                edges += board.edges
                millimetres += board.millimetres
            }
            is OdometerReading.Edges -> {
                edges += board.edges
                converted = false
            }
            OdometerReading.Silent -> return null
        }
    }
    return if (converted) {
        MachineTravel.Distance(millimetres / boards.size)
    } else {
        MachineTravel.Edges(edges / boards.size)
    }
}

/** The pair's averaged travel, in whichever unit every reporting board could give it in. */
sealed interface MachineTravel {

    /** The mean of the boards' edge counts, for a pair where at least one cannot convert. */
    data class Edges(val edges: Long) : MachineTravel

    /** The mean of the boards' distances, in millimetres, for a pair where every board converts. */
    data class Distance(val millimetres: Long) : MachineTravel
}
