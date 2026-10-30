package com.hoverboard.remote.model

import com.hoverboard.protocol.store.Fields
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * What one board brings to the machine's speed (`specs/rider-ui.md` 3.1a): its period-measured
 * wheel rate off `CYCLIC_STATE` and the three stored facts that turn it into a road speed.
 *
 * @param wheelRateQ12 `CYCLIC_STATE.wheel_rate`: Q12 of one hall edge per 320-period window, in
 *   the board's own counter sense.
 * @param polePairs `motors.0.pole_pairs`, null while unread and 0 while unset.
 * @param diameterMm `motors.0.wheel_diameter_mm`, null while unread and 0 while unset.
 * @param side `control.side`, null while unread; [Fields.ControlSide.RIGHT] mirrors the rate.
 */
data class SpeedFacts(val wheelRateQ12: Int, val polePairs: Int?, val diameterMm: Int?, val side: Int?)

/** What the Ride panel's speed says. */
sealed interface SpeedReading {

    /** No motor controller is reporting, so there is no rate to convert. */
    data object Silent : SpeedReading

    /**
     * A rate and no speed, because a fact the conversion needs is missing on a reporting board.
     * Each list holds field names as the host config tool spells them, once each: [unread] are
     * fields this session has not got from a board yet, [unset] fields a board holds as 0.
     */
    data class NoSpeed(val unread: List<String>, val unset: List<String>) : SpeedReading

    /** The machine's forward speed in km/h, negative backwards. */
    data class Kmh(val kmh: Double) : SpeedReading {
        /**
         * The figure as the panel prints it: tenths below 10 km/h and whole km/h at and above it,
         * decided on the rounded value so 9.96 prints as 10 and not as 10.0. A reading that rounds
         * to zero is zero, never "-0.0".
         */
        val shown: Shown
            get() {
                val tenths = (kmh * TENTHS).roundToInt()
                return when {
                    tenths == 0 -> Shown.Tenths(0.0)
                    abs(tenths) < WHOLE_FROM_TENTHS -> Shown.Tenths(tenths / TENTHS)
                    else -> Shown.Whole(kmh.roundToInt())
                }
            }
    }

    /** A printed speed: to a tenth, or whole. */
    sealed interface Shown {
        /** A figure to one decimal. */
        data class Tenths(val kmh: Double) : Shown

        /** A whole number of km/h. */
        data class Whole(val kmh: Int) : Shown
    }

    companion object {
        /** The CLI name of [SpeedFacts.polePairs]'s field. */
        const val POLE_PAIRS = "motors.0.pole_pairs"

        /** The CLI name of [SpeedFacts.diameterMm]'s field. */
        const val WHEEL_DIAMETER = "motors.0.wheel_diameter_mm"

        /** The CLI name of [SpeedFacts.side]'s field. */
        const val SIDE = "control.side"

        private const val TENTHS = 10.0
        private const val WHOLE_FROM_TENTHS = 100
    }
}

/** One board's forward speed, km/h, from complete facts (`specs/rider-ui.md` 3.1a). */
private fun kmh(rateQ12: Int, polePairs: Int, diameterMm: Int, right: Boolean): Double {
    val forward = if (right) -rateQ12.toDouble() else rateQ12.toDouble()
    val edgesPerSecond = forward / Q12 * WINDOWS_PER_SECOND
    val revsPerSecond = edgesPerSecond / (EDGES_PER_ELECTRICAL_REV * polePairs)
    return revsPerSecond * PI * diameterMm * MM_PER_S_TO_KMH
}

/**
 * The machine's speed: the MEAN of the reporting boards' speeds, each turned into the machine's
 * forward sense by its own `control.side` (the fold `control::wheel_forward` applies on the board),
 * since the two wheels of a machine turn opposite ways in their own senses.
 *
 * A speed only when every reporting board has all three facts; otherwise [SpeedReading.NoSpeed]
 * naming what is missing, and never a speed taken from the one board that could convert, since a
 * one-wheel figure on a turning machine is not the machine's.
 */
fun machineSpeed(boards: List<SpeedFacts>): SpeedReading {
    if (boards.isEmpty()) return SpeedReading.Silent
    val unread = linkedSetOf<String>()
    val unset = linkedSetOf<String>()
    fun need(value: Int?, name: String) {
        when {
            value == null -> unread += name
            value <= 0 -> unset += name
        }
    }
    for (b in boards) {
        need(b.polePairs, SpeedReading.POLE_PAIRS)
        need(b.diameterMm, SpeedReading.WHEEL_DIAMETER)
        if (b.side == null) unread += SpeedReading.SIDE
    }
    val speeds = boards.mapNotNull { b ->
        val polePairs = b.polePairs?.takeIf { it > 0 }
        val diameterMm = b.diameterMm?.takeIf { it > 0 }
        if (polePairs == null || diameterMm == null || b.side == null) {
            null
        } else {
            kmh(b.wheelRateQ12, polePairs, diameterMm, b.side == Fields.ControlSide.RIGHT)
        }
    }
    return if (speeds.size == boards.size) {
        SpeedReading.Kmh(speeds.average())
    } else {
        SpeedReading.NoSpeed(unread.toList(), unset.toList())
    }
}

/** `wheel_rate`'s Q12 unit: 4096 is one hall edge per window. */
private const val Q12 = 4096.0

/** 320-period windows per second at the 16 kHz PWM: 16,000 / 320. */
private const val WINDOWS_PER_SECOND = 50.0

/** Hall edges per electrical revolution: the six-state commutation sequence. */
private const val EDGES_PER_ELECTRICAL_REV = 6

/** Millimetres per second to km/h: 3600 / 1,000,000. */
private const val MM_PER_S_TO_KMH = 0.0036
