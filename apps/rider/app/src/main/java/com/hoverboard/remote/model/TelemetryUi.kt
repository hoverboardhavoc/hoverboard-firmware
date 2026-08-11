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
 *  - `battery` is CENTIVOLTS (`crates/orchestrator/src/dispatch.rs:42,113`), so volts = cV / 100.
 *    The retired code read millivolts here, which would have shown a 36 V pack as 3.6 V.
 *  - `pitch` and `roll` are centidegrees.
 *  - `wheelSpeed` is the stock-native speed word; its scale is still open in the firmware spec,
 *    so it is surfaced raw.
 *  - `fault` is the latched fault LEVEL, 0 = healthy. Note the current emitter hardcodes it to 0
 *    (`crates/orchestrator/src/dispatch.rs:435`), so today the level never trips and only a FAULT
 *    edge PDU sets [faultCode].
 *  - `flags` bit0 rider present, bit7 lockdown.
 *
 * [faultStop] and [faultCode] come from a separate FAULT PDU (opcode 0x13), which is a latch-edge
 * notification rather than a cyclic one.
 */
data class TelemetryUi(
    val cyclic: CyclicState? = null,
    val faultStop: Boolean = false,
    val faultCode: Int = 0,
) {
    /** Pack voltage in volts (battery centivolts / 100). */
    val batteryVolts: Float get() = (cyclic?.battery ?: 0) / CENTIVOLTS_PER_VOLT

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

    /** A FAULT stop-all edge, a non-zero edge code, or a non-zero cyclic fault level. */
    val anyFault: Boolean
        get() = faultStop || faultCode != 0 || (cyclic?.fault ?: 0) != 0

    /** Fold a freshly decoded [CyclicState] in. Latest-wins: the board sends one state, cyclically. */
    fun merge(state: CyclicState): TelemetryUi = copy(cyclic = state)

    companion object {
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
