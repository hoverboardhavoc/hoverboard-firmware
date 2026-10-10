package com.hoverboard.remote.model

import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicState

/**
 * UI-facing telemetry, derived from the board's CYCLIC_STATE PDUs (opcode 0x12's sibling, 0x10).
 *
 * CYCLIC_STATE is the telemetry source because the firmware has no telemetry opcode: the
 * 0x40..0x6F block is reserved and unimplemented (`crates/net/src/pdu.rs:41`). This app used to
 * decode a TELEMETRY 0x20 frame, per motor, that nothing ever sent.
 *
 * That swap changes what the panel can show. CYCLIC_STATE is one board-level state record, not a
 * per-motor one, so there is no per-wheel current in it. What it does carry
 * (`crates/linkctl/src/lib.rs`, `CyclicState`), with the units the firmware puts on the wire:
 *  - `battery` is CENTIVOLTS (`orchestrator::dispatch`, `BlockWords::battery`), so
 *    volts = cV / 100. The retired code read millivolts here, which would have shown a 36 V pack
 *    as 3.6 V. Zero is not a reading: see [batteryVolts].
 *  - `pitch` and `roll` are centidegrees.
 *  - `wheelSpeed` is the signed HALL EDGE COUNT per 320-period window (`motor::SPEED` saturated to
 *    i16), explicitly not the stock unit and not a speed: see [hallEdgesPerWindow].
 *  - `fault` is the latched fault LEVEL, 0 = healthy. See [anyFault] for why a zero here is not
 *    evidence of a healthy board.
 *  - `flags` bit0 rider present, bit7 lockdown.
 *  - the appended `CyclicObs` block: the last 64-period current window (peak and mean, stock
 *    current counts), the on-duty that closed it, the boot counter's low byte and the part the
 *    board detected. Every payload carries it, so the readers below are nullable only for the
 *    same reason [cyclic] is: nothing has arrived yet. See [phaseMeanCounts].
 *
 * [faultStop] and [faultCode] come from a separate FAULT PDU (`linkctl`, `OP_FAULT`), which is a
 * latch-edge notification rather than a cyclic one.
 */
data class TelemetryUi(
    val cyclic: CyclicState? = null,
    val faultStop: Boolean = false,
    val faultCode: Int = 0,
) {
    /**
     * Pack voltage in volts (battery centivolts / 100), or NULL when there is no reading.
     *
     * Null covers the two ways a reading can be absent, which a display has to render the same way
     * and must not render as zero volts:
     *  - nothing has arrived from the board yet ([hasState] is false), and
     *  - the board sent `battery = 0`, which is the firmware's UNKNOWN.
     *
     * UNKNOWN is broader than "this board cannot sense its rail", and nothing on the wire says
     * which case it is. `orchestrator::battery` holds the word at 0 while `VBATT_RAW` is 0, so a
     * master whose `board.vbatt` IS staged publishes UNKNOWN until its motor is brought up, and
     * with `board.vbatt` defaulting to absent that is the ordinary state of a part-configured
     * master (`specs/sensing-and-safety.md`, "The battery word"). A consumer may therefore say
     * there is no reading; it may not say why.
     *
     * A board reporting no measurement is not a board measuring zero. 0.00 V on the panel was both
     * a measurement claim the board never made and, read as a number, a flat pack.
     */
    val batteryVolts: Float? get() = cyclic?.battery?.takeIf { it != 0 }?.div(CENTIVOLTS_PER_VOLT)

    /**
     * How full to draw the charge bar, or null when there is nothing to draw ([batteryVolts]).
     *
     * The fill is a derived judgement about the pack in exactly the way the percent and the
     * green/amber/red colouring are, so it is absent whenever they are: a grey bar at some fill
     * would still be a bar claiming a state of charge.
     */
    val batteryFraction: Float? get() = batteryVolts?.let(BatteryCurve::fraction)

    /**
     * Battery-low at or below [LOW_VOLTAGE_THRESHOLD]. False when there is no reading: an absent
     * reading is not a low one.
     */
    val batteryLow: Boolean get() = batteryVolts?.let { it <= LOW_VOLTAGE_THRESHOLD } == true

    /**
     * `CYCLIC_STATE.wheel_speed`: the signed count of hall edges in the last 320-period window
     * (`motor::SPEED` as the period ISR produces it, saturated to i16, copied to the wire with no
     * rescaling).
     *
     * It is a COUNT, not a speed, and the owner decision of 2026-10-09 put the raw count on the
     * wire deliberately (`specs/link-control.md`, the CYCLIC_STATE mirror section). A road speed
     * needs `motor.pole_pairs` and a wheel diameter, and neither is a registered field, so the app
     * cannot derive one and does not imply one in the label.
     *
     * The word also has no writer yet on any board (bench 2026-10-09): the copy from `motor::SPEED`
     * into the control block is one line that has not been written, so this reads 0 from every
     * board regardless of what the wheel is doing.
     */
    val hallEdgesPerWindow: Int get() = cyclic?.wheelSpeed ?: 0

    /** Pitch in degrees (centidegrees / 100). */
    val pitchDegrees: Float get() = (cyclic?.pitch ?: 0) / CENTIDEGREES_PER_DEGREE

    /** Roll in degrees (centidegrees / 100). */
    val rollDegrees: Float get() = (cyclic?.roll ?: 0) / CENTIDEGREES_PER_DEGREE

    /** The board's rider-present flag, as the board sees it (not the phone's pad state). */
    val riderPresent: Boolean get() = cyclic?.riderPresent() ?: false

    /** The board's lockdown flag: the stock master-shutdown semantic. */
    val lockdown: Boolean get() = cyclic?.lockdown() ?: false

    /**
     * The last completed current window's PEAK phase-current magnitude, stock current counts, or
     * null when no state has arrived at all.
     *
     * Counts, not amps: amps are `counts / MOTOR_CURRENT_CAL`, and the calibration is per board
     * (`protocol-kotlin`, `Fields.MOTOR_CURRENT_CAL`), so the conversion belongs to whatever
     * holds the board's own stored fields rather than to this record.
     */
    val phasePeakCounts: Int? get() = cyclic?.obs?.phasePeak

    /**
     * The SAME window's MEAN magnitude, counts; null as [phasePeakCounts].
     *
     * **Null is not zero, and a display must keep them apart.** A zero here is a board carrying
     * no current; a null is a board nothing has arrived from yet. Rendering "nothing has arrived"
     * as 0.0 A is the mistake this nullability exists to prevent. Every payload that does arrive
     * carries the block, so the null is this record's own "no sample", not a sender's omission.
     *
     * The mean rather than the peak is what a calibration cross-check compares: the peak is a
     * maximum over ADC samples and reads high near the noise floor
     * (`specs/rider-ui.md`, 3.6).
     */
    val phaseMeanCounts: Int? get() = cyclic?.obs?.phaseMean

    /**
     * The on-duty applied in the period that closed the window, `0..ARR`; null as
     * [phasePeakCounts], and 0 for a window closed by a coasting period.
     *
     * With the mean and the board's calibration this is what gives the DC-LINK current a bench
     * PSU displays, `(mean / cal) * (dutyOn / ARR)`, as against the PHASE current the limiter
     * acts on.
     */
    val dutyOn: Int? get() = cyclic?.obs?.dutyOn

    /**
     * The board's boot counter, low byte; null as [phasePeakCounts].
     *
     * A CHANGE in it is a board that rebooted under the app, which is the fact the Setup screen
     * has been asking an operator to confirm. One byte wraps at 256 boots between two
     * observations.
     */
    val bootTag: Int? get() = cyclic?.obs?.bootTag

    /**
     * The part the board detected at boot, or null when no state has arrived yet.
     * [ChipTag.Unknown] is different again: the board reported a byte this build does not
     * allocate, so it named a part the app cannot act on.
     */
    val chip: ChipTag? get() = cyclic?.obs?.chip

    /** True once any CYCLIC_STATE has been seen, so the panel can tell "waiting" from "zeroed". */
    val hasState: Boolean get() = cyclic != null

    /**
     * A FAULT stop-all edge, a non-zero edge code, or a non-zero cyclic fault level.
     *
     * All three sources are read, and NONE of them can fire against today's firmware:
     * `orchestrator::dispatch::cyclic_state` builds every `CYCLIC_STATE` with `fault: 0`, and
     * `OP_FAULT` has no emitter anywhere in `crates/` at all (it is decode-only, for a producer
     * that has not been written). So a false here means "nothing has reported a fault", which is
     * not the same claim as "the board is healthy", and the UI must not render it as one: see
     * [com.hoverboard.remote.ui.components.TelemetryPanel].
     *
     * The reads stay because they are the whole consumer side of a contract whose producer is
     * missing; the day either producer lands, this fires with no app change.
     */
    val anyFault: Boolean
        get() = faultStop || faultCode != 0 || faultLevel != 0

    /** The cyclic fault LEVEL, 0 = nothing reported. Hardcoded 0 by the emitter; see [anyFault]. */
    val faultLevel: Int get() = cyclic?.fault ?: 0

    /** Fold a freshly decoded [CyclicState] in. Latest-wins: the board sends one state, cyclically. */
    fun merge(state: CyclicState): TelemetryUi = copy(cyclic = state)

    companion object {
        private const val CENTIVOLTS_PER_VOLT = 100f
        private const val CENTIDEGREES_PER_DEGREE = 100f

        /**
         * Battery-low at or below 3.3 V/cell on a 7s pack (~23.1 V). The 7s endpoints live in
         * [BatteryCurve]; this is the in-panel warning trip, kept conservative.
         */
        private const val LOW_VOLTAGE_THRESHOLD = 23.1f
    }
}
