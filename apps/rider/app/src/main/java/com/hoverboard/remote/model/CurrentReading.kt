package com.hoverboard.remote.model

import com.hoverboard.protocol.linkctl.CyclicObs

/**
 * What one board's current display says right now (`specs/rider-ui.md` 3.6).
 *
 * Two amp figures or no figure at all, and the four ways there can be no figure are kept apart
 * because they are different facts about the machine and an owner acts on them differently. The
 * one thing this type cannot produce is a number from a window nothing refreshed: a current
 * display showing a reading from thirty seconds ago is worse than one showing nothing.
 */
sealed interface CurrentReading {

    /** Nothing has arrived from this board, or it has stopped being heard ([TelemetryUi.hasState]). */
    data object Silent : CurrentReading

    /**
     * The board's motor path is not up, so nothing is measuring.
     *
     * The window words are written by the period interrupt alone (`crates/firmware/src/motor.rs`,
     * `OBS_CURRENT` and `OBS_MEAN_DUTY`), which does not run while the bridge is down, so what the
     * board keeps sending is whatever the last live window left there. That is the stale number
     * this state exists to refuse.
     */
    data object MotorsOff : CurrentReading

    /**
     * The window closed on a period that drove nothing ([CyclicObs.dutyOn] is 0).
     *
     * No phase conducted, whatever the duty registers held (`motor::on_duty`), so the link current
     * is nothing and the measured phase magnitude is the sense chain's own rest noise rather than
     * a current the motor took. Neither is worth a number.
     */
    data object Coasting : CurrentReading

    /** No counts-per-amp for this board yet, so a count cannot be turned into amps. */
    data object Unscaled : CurrentReading

    /**
     * The two figures, amps.
     *
     * @param motor the PHASE current, which is what the motor feels and what the soft limit chops
     *   on: the window MEAN over the board's counts per amp. The mean and not the peak, because
     *   the peak is a maximum over ADC samples and reads high near the noise floor
     *   (`specs/rider-ui.md` 3.6).
     * @param supply the DC-LINK current, which is what a bench supply displays and what the pack
     *   pays: the phase current scaled by the duty that was applied. It is the bridge's share
     *   alone and does not carry the board's own draw, measured at 84 mA on 2026-10-09 and
     *   modelled in no field.
     */
    data class Amps(val motor: Float, val supply: Float) : CurrentReading
}

/**
 * What [telemetry] and that board's own [countsPerAmp] (`motor.current_cal`) come to.
 *
 * The scale is per board, because it is a property of the shunt and amplifier chain fitted rather
 * than of the firmware, so one board's figure is never derived from the other's constant. A
 * missing or zero scale yields [CurrentReading.Unscaled] rather than a figure at an assumed one.
 */
fun currentReading(telemetry: TelemetryUi?, countsPerAmp: Int?): CurrentReading {
    val mean = telemetry?.phaseMeanCounts
    val duty = telemetry?.dutyOn
    return when {
        mean == null || duty == null -> CurrentReading.Silent
        telemetry.mode !in MEASURING_MODES -> CurrentReading.MotorsOff
        duty == 0 -> CurrentReading.Coasting
        countsPerAmp == null || countsPerAmp <= 0 -> CurrentReading.Unscaled
        else -> {
            val motor = mean.toFloat() / countsPerAmp
            CurrentReading.Amps(motor = motor, supply = motor * duty / CyclicObs.DUTY_FULL)
        }
    }
}

/**
 * The machine's DC-LINK current: the boards' link figures added, which is the only figure a bench
 * supply's own reading can be compared against.
 *
 * The supply feeds both boards, so what it displays is each board's bridge plus both boards' own
 * electronics:
 *
 * ```
 * PSU reading  ~=  I_link(board 1) + I_link(board 2) + 2 x I_board
 * ```
 *
 * One board's figure is a SHARE of that and nothing more, which is why the panel carries the sum
 * as well as the two rows. The two terms are independent derivations that happen to be added: each
 * board's own counts, its own `motor.current_cal` and its own duty, and the two constants need not
 * agree.
 *
 * The board's own draw is NOT folded in. It is 84 mA per board, measured once on 2026-10-09 and
 * modelled in no field, so the sum is the two bridges and the panel says so in words beside it.
 *
 * Null when a board cannot contribute, and a board that is reporting is never one of those: a
 * board whose motors are down and a board whose last period coasted both drove nothing through
 * their bridge, so each contributes a real zero rather than an unknown. What is unknown is a board
 * nothing has arrived from and a board whose scale this session has not read, and a sum that
 * quietly became one board's figure would be a wrong number presented confidently.
 */
fun machineSupplyAmps(boards: List<CurrentReading>): Float? {
    var total = 0f
    for (board in boards) {
        total += when (board) {
            is CurrentReading.Amps -> board.supply
            CurrentReading.MotorsOff, CurrentReading.Coasting -> 0f
            CurrentReading.Silent, CurrentReading.Unscaled -> return null
        }
    }
    return total
}

/**
 * The modes in which the period interrupt is running, so the window words are this board's current
 * state rather than the last one it was in.
 *
 * [BoardMode.OFF] is the bridge down and the motor path not brought up. [BoardMode.SHUTDOWN] is
 * the one-shot pass that forces the outputs safe on the way there, which drives nothing worth
 * reporting either. [BoardMode.UNRECOGNISED] is a mode byte this build does not name, and a board
 * whose state the app cannot name is not a board it can say is measuring.
 */
private val MEASURING_MODES = setOf(BoardMode.INIT, BoardMode.READY, BoardMode.RUN)
