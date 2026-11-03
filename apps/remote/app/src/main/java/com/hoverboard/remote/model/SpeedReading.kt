package com.hoverboard.remote.model

import com.hoverboard.protocol.store.Fields
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * What one wheel brings to the machine's speed (`specs/remote-ui.md` 3.1a): its period-measured
 * rate off `CYCLIC_STATE` and the three stored facts that turn it into a road speed, each its own
 * motor's.
 *
 * @param address the wheel's board's L3 address: whom a missing fact is missing on (3.1, "Missing
 *   settings").
 * @param wheelRateQ12 the motor's `wheel_rate` (motor 0's in the base, motor 1's in the motor-1
 *   block): Q12 of one hall edge per 320-period window, in the motor's own counter sense.
 * @param polePairs `motors.<motor>.pole_pairs`, null while unread and 0 while unset.
 * @param diameterMm `motors.<motor>.wheel_diameter_mm`, null while unread and 0 while unset.
 * @param side the motor's `side`, null while unread; [Fields.ControlSide.RIGHT] mirrors the rate.
 * @param motor the wheel's motor on its board: 0, or 1 on a two-motor board.
 */
data class SpeedFacts(
    val address: Int,
    val wheelRateQ12: Int,
    val polePairs: Int?,
    val diameterMm: Int?,
    val side: Int?,
    val motor: Int = 0,
)

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
        /** The CLI name of motor [motor]'s [SpeedFacts.polePairs] field. */
        fun polePairs(motor: Int): String = "motors.$motor.pole_pairs"

        /** The CLI name of motor [motor]'s [SpeedFacts.diameterMm] field. */
        fun wheelDiameter(motor: Int): String = "motors.$motor.wheel_diameter_mm"

        /** The CLI name of motor [motor]'s [SpeedFacts.side] field. */
        fun side(motor: Int): String = "motors.$motor.side"

        /** Motor 0's [polePairs] name. */
        val POLE_PAIRS = polePairs(0)

        /** Motor 0's [wheelDiameter] name. */
        val WHEEL_DIAMETER = wheelDiameter(0)

        /** Motor 0's [side] name. */
        val SIDE = side(0)

        private const val TENTHS = 10.0
        private const val WHOLE_FROM_TENTHS = 100
    }
}

/** One board's forward speed, km/h, from complete facts (`specs/remote-ui.md` 3.1a). */
private fun kmh(rateQ12: Int, polePairs: Int, diameterMm: Int, right: Boolean): Double {
    val forward = if (right) -rateQ12.toDouble() else rateQ12.toDouble()
    val edgesPerSecond = forward / Q12 * WINDOWS_PER_SECOND
    val revsPerSecond = edgesPerSecond / (EDGES_PER_ELECTRICAL_REV * polePairs)
    return revsPerSecond * PI * diameterMm * MM_PER_S_TO_KMH
}

/**
 * The machine's speed: the MEAN of the reporting wheels' speeds, each turned into the machine's
 * forward sense by its own motor's `side` (the fold `control::wheel_forward` applies on the board),
 * since the two wheels of a machine turn opposite ways in their own senses. A wheel is a board's
 * motor: a 12-FET reports two.
 *
 * A speed only when every reporting wheel has all three facts; otherwise [SpeedReading.NoSpeed]
 * naming what is missing, and never a speed taken from the one wheel that could convert, since a
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
        need(b.polePairs, SpeedReading.polePairs(b.motor))
        need(b.diameterMm, SpeedReading.wheelDiameter(b.motor))
        if (b.side == null) unread += SpeedReading.side(b.motor)
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

/** A speed's magnitude as Ride prints it: never a minus sign (`specs/remote-ui.md` 3.1). */
fun speedText(shown: SpeedReading.Shown): String = when (shown) {
    is SpeedReading.Shown.Tenths -> "%.1f".format(abs(shown.kmh))
    is SpeedReading.Shown.Whole -> abs(shown.kmh).toString()
}
