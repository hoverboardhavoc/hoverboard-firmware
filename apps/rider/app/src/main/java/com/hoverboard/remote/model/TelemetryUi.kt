package com.hoverboard.remote.model

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
 * (`crates/linkctl/src/lib.rs:88-106`), with the units the firmware puts on the wire:
 *  - `battery` is CENTIVOLTS (`orchestrator::dispatch`, `BATTERY_PLACEHOLDER_CENTIVOLT` and
 *    `BlockWords::battery`), so volts = cV / 100. The retired code read millivolts here, which
 *    would have shown a 36 V pack as 3.6 V. It is not a measurement yet: see [batteryPlaceholder].
 *  - `pitch` and `roll` are centidegrees.
 *  - `wheelSpeed` is the stock-native speed word; its scale is still open in the firmware spec,
 *    so it is surfaced raw.
 *  - `fault` is the latched fault LEVEL, 0 = healthy. See [anyFault] for why a zero here is not
 *    evidence of a healthy board.
 *  - `flags` bit0 rider present, bit7 lockdown.
 *
 * [faultStop] and [faultCode] come from a separate FAULT PDU (`linkctl`, `OP_FAULT`), which is a
 * latch-edge notification rather than a cyclic one.
 */
data class TelemetryUi(
    val cyclic: CyclicState? = null,
    val faultStop: Boolean = false,
    val faultCode: Int = 0,
) {
    /** Pack voltage in volts (battery centivolts / 100). */
    val batteryVolts: Float get() = (cyclic?.battery ?: 0) / CENTIVOLTS_PER_VOLT

    /**
     * Whether the battery word is the firmware's stand-in rather than a measurement of this pack.
     *
     * There is no VBATT producer in the firmware at all. `orchestrator::dispatch::BlockWords` seeds
     * `battery` with [BATTERY_PLACEHOLDER_CENTIVOLT] and the sensing task that would overwrite it
     * is not built, so every board on every rail reports exactly 36.00 V, and the peer word the
     * master prefers as the PID scale is the same constant coming back off the other board. It is
     * a constant that LOOKS like a plausible reading of a healthy 36 V pack, which is why it has
     * already been read as one on the bench.
     *
     * Matching the constant rather than hardcoding "battery is never real" is what makes this
     * retire itself: the day a sensing task puts a measured word on the wire, the tag stops
     * appearing without anyone having to remember to delete it. The cost is that a genuinely
     * measured 36.00 V would be tagged too, which errs toward claiming less than is known.
     */
    val batteryPlaceholder: Boolean get() = cyclic?.battery == BATTERY_PLACEHOLDER_CENTIVOLT

    /**
     * How full to draw the charge bar: [BatteryCurve]'s fraction, or EMPTY while the reading is the
     * placeholder.
     *
     * The fill is a derived judgement about the pack in exactly the way the percent and the
     * green/amber/red colouring are, and it has to be withheld with them. Suppressing only the
     * colour leaves a full grey bar, and a full bar still says "full": [BatteryCurve] tops out at
     * 29.4 V, so the 36.00 V placeholder clamps to 1.0 and draws the bar hard against its end.
     */
    val batteryFraction: Float
        get() = if (batteryPlaceholder) 0f else BatteryCurve.fraction(batteryVolts)

    /**
     * Battery-low at or below [LOW_VOLTAGE_THRESHOLD], guarded above 0.1 V so a missing
     * state (0 cV) does not read as a low battery.
     */
    val batteryLow: Boolean get() = batteryVolts in BATTERY_PRESENT_MIN..LOW_VOLTAGE_THRESHOLD

    /**
     * Raw wheel-speed word. Units are open in the firmware spec, so this is the unscaled link
     * integer, as before.
     */
    val speedRaw: Int get() = cyclic?.wheelSpeed ?: 0

    /** Pitch in degrees (centidegrees / 100). */
    val pitchDegrees: Float get() = (cyclic?.pitch ?: 0) / CENTIDEGREES_PER_DEGREE

    /** Roll in degrees (centidegrees / 100). */
    val rollDegrees: Float get() = (cyclic?.roll ?: 0) / CENTIDEGREES_PER_DEGREE

    /** The board's rider-present flag, as the board sees it (not the phone's pad state). */
    val riderPresent: Boolean get() = cyclic?.riderPresent() ?: false

    /** The board's lockdown flag: the stock master-shutdown semantic. */
    val lockdown: Boolean get() = cyclic?.lockdown() ?: false

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
        /**
         * The battery word every board sends today, whatever its real rail.
         *
         * Mirrors `orchestrator::dispatch::BATTERY_PLACEHOLDER_CENTIVOLT`. It is not on the wire
         * contract, so `protocol-kotlin`'s drift gate does not pin it; it is here because
         * [batteryPlaceholder] has to recognise it, and it is cited so the next reader can check
         * it against the Rust.
         */
        const val BATTERY_PLACEHOLDER_CENTIVOLT: Int = 3_600

        private const val CENTIVOLTS_PER_VOLT = 100f
        private const val CENTIDEGREES_PER_DEGREE = 100f
        private const val BATTERY_PRESENT_MIN = 0.1f

        /**
         * Battery-low at or below 3.3 V/cell on a 7s pack (~23.1 V). The 7s endpoints live in
         * [BatteryCurve]; this is the in-panel warning trip, kept conservative.
         */
        private const val LOW_VOLTAGE_THRESHOLD = 23.1f
    }
}
