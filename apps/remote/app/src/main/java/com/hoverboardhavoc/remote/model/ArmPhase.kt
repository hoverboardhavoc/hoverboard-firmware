package com.hoverboardhavoc.remote.model

/**
 * Where the arm control is (`specs/arm-confirmed.md` section 3): what the app asked for, and
 * whether every board it latched has confirmed it with its own `FLAG_MOE`.
 *
 * - [IDLE]: no arm level held. The control reads ARM.
 * - [ARMING]: the arm level is on the wire and not every latched board reports `FLAG_MOE` yet.
 *   The throttle stays zero.
 * - [ARMED]: every latched board reported `FLAG_MOE`. The only phase the throttle is live in.
 * - [DISARMING]: the arm level is off the wire and not every latched board reports `FLAG_MOE`
 *   clear yet. The stop goes out as it does in [IDLE]; only the label waits.
 */
enum class ArmPhase {
    IDLE,
    ARMING,
    ARMED,
    DISARMING,
    ;

    /** Whether the app puts the arm level on the wire in this phase. */
    val holdsLevel: Boolean get() = this == ARMING || this == ARMED
}

/**
 * Why an arm ended in refusal: no confirmation within the window. [address] is the first latched
 * board that did not report `FLAG_MOE`, and [lamp] the first bit it reports that refuses an arm,
 * or null when it reports none or is silent.
 */
data class ArmRefusal(val address: Int, val lamp: FaultLamp?)

/**
 * A board that dropped `FLAG_MOE` while the app held it armed: the board's own disarm. [lamp] is
 * the first lamp its fault word carries, or null when it carries none.
 */
data class BoardDisarm(val address: Int, val lamp: FaultLamp?)

/** Whether every one of [boards] reports, and reports `FLAG_MOE` as [moe]. A silent board has not. */
fun allReportMoe(boards: List<Int>, telemetry: Map<Int, TelemetryUi>, moe: Boolean): Boolean =
    boards.all { telemetry[it]?.cyclic?.moe() == moe }

/** The first of [boards] that reports `FLAG_MOE` clear; a silent board reports nothing. */
fun firstDroppedMoe(boards: List<Int>, telemetry: Map<Int, TelemetryUi>): Int? =
    boards.firstOrNull { telemetry[it]?.cyclic?.moe() == false }
