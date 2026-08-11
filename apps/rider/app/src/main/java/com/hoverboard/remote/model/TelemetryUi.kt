package com.hoverboard.remote.model

import com.hoverboard.remote.link.Telemetry

/**
 * UI-facing telemetry, derived from one or more link [Telemetry] frames (opcode 0x20).
 *
 * The board emits one Telemetry frame per motor (`motorIndex`). This model keeps the latest
 * frame seen for each index (see [merge]) and exposes display-scaled values for the panel.
 *
 * Field units follow the link contract (crates/link payload.rs), NOT the retired legacy
 * centivolt/centi-km/h encodings:
 *  - `batteryMv` is millivolts; [batteryVolts] = mV / 1000.
 *  - `currentCa` is centiamps; [currentAmps] = cA / 100.
 *  - `speed` units are still open in the firmware spec, so it is surfaced as a raw integer
 *    (TODO: pin the speed unit/scale once the firmware telemetry field map is finalized).
 *  - `flags` / `faultCode` are health indicators.
 *
 * The merge keeps a stable index order so wheel A / wheel B map to the two lowest indices.
 */
data class TelemetryUi(
    val motors: Map<Int, Telemetry> = emptyMap(),
    val faultStop: Boolean = false,
    val faultCode: Int = 0,
) {
    /** Frames ordered by motor index, so wheel A is the lowest index, wheel B the next. */
    val orderedMotors: List<Telemetry> get() = motors.entries.sortedBy { it.key }.map { it.value }

    private val primary: Telemetry? get() = orderedMotors.firstOrNull()
    private val secondary: Telemetry? get() = orderedMotors.getOrNull(1)

    /** Pack voltage in volts (battery_mv / 1000), from the lowest-index motor frame. */
    val batteryVolts: Float get() = (primary?.batteryMv ?: 0) / MV_PER_VOLT

    /**
     * Battery-low at or below [LOW_VOLTAGE_THRESHOLD], guarded above 0.1 V so a missing
     * frame (0 mV) does not read as a low battery.
     */
    val batteryLow: Boolean get() = batteryVolts in BATTERY_PRESENT_MIN..LOW_VOLTAGE_THRESHOLD

    /**
     * Raw speed value of the lowest-index motor. Units are open in the firmware spec, so this
     * is the unscaled link integer. TODO: scale/label once the speed unit is pinned.
     */
    val speedRaw: Int get() = primary?.speed ?: 0

    /** Current of wheel A (lowest index) in amps (current_ca / 100). */
    val currentAmpsA: Float get() = (primary?.currentCa ?: 0) / CA_PER_AMP

    /** Current of wheel B (next index) in amps, or wheel A's value if only one motor is present. */
    val currentAmpsB: Float get() = ((secondary ?: primary)?.currentCa ?: 0) / CA_PER_AMP

    /** Any non-zero per-motor fault code, OR a [Fault] stop-all reflected via [faultStop]. */
    val anyFault: Boolean
        get() = faultStop || faultCode != 0 || motors.values.any { it.faultCode != 0 }

    /**
     * Fold a freshly decoded link [Telemetry] in, replacing the entry for its `motorIndex`
     * and keeping every other motor's latest frame.
     */
    fun merge(frame: Telemetry): TelemetryUi =
        copy(motors = motors + (frame.motorIndex to frame))

    companion object {
        private const val MV_PER_VOLT = 1000f
        private const val CA_PER_AMP = 100f
        private const val BATTERY_PRESENT_MIN = 0.1f

        /**
         * Battery-low at or below 3.3 V/cell on a 7s pack (~23.1 V). The 7s endpoints live in
         * [BatteryCurve]; this is the in-panel warning trip, kept conservative.
         */
        private const val LOW_VOLTAGE_THRESHOLD = 23.1f
    }
}
