package com.hoverboard.remote.model

import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicState

/**
 * UI-facing telemetry, derived from the board's CYCLIC_STATE PDUs (opcode 0x12's sibling, 0x10).
 *
 * CYCLIC_STATE is the telemetry source because the firmware has no telemetry opcode: the
 * 0x40..0x6F block is reserved and unimplemented (`crates/net/src/pdu.rs`, `Opcode`). This app
 * used to decode a TELEMETRY 0x20 frame, per motor, that nothing ever sent.
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
 *    current counts), the on-duty that closed it, the boot counter's low byte, the part the
 *    board detected and this wheel's lifetime travel in hall edges. Every payload carries it, so
 *    the readers below are nullable only for the same reason [cyclic] is: nothing has arrived
 *    yet. See [phaseMeanCounts].
 */
data class TelemetryUi(
    val cyclic: CyclicState? = null,
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
     * which case it is. A sensing board's word is 0 until a real conversion PRIMES its filter
     * (`orchestrator::battery`, `LocalSense::step`; a zero count after priming leaves the word
     * where it was, rather than returning it to 0), and only a converting injected ADC group ever
     * writes that count, so a master whose `board.vbatt` IS staged publishes UNKNOWN until its
     * motor is brought up. With `board.vbatt` defaulting to absent, that is the ordinary state of
     * a part-configured master (`specs/sensing-and-safety.md`, "The battery word"). A consumer may
     * therefore say there is no reading; it may not say why.
     *
     * A board reporting no measurement is not a board measuring zero. 0.00 V on the panel was both
     * a measurement claim the board never made and, read as a number, a flat pack.
     */
    val batteryVolts: Float? get() = cyclic?.battery?.takeIf { it != 0 }?.div(CENTIVOLTS_PER_VOLT)

    /**
     * How full the pack is does NOT live here, because it is not a fact about this record: it
     * takes the machine's cell count as well, which arrives from the board's store rather than on
     * the wire ([batteryReading], which pairs this voltage with it).
     */

    /**
     * `CYCLIC_STATE.wheel_speed`: the signed count of hall edges in the last 320-period window
     * (`motor::SPEED` as the period ISR produces it, saturated to i16, copied to the wire with no
     * rescaling).
     *
     * It is a COUNT, not a speed, and the owner decision of 2026-10-09 put the raw count on the
     * wire deliberately (`specs/link-control.md`, the CYCLIC_STATE mirror section). A road speed
     * needs the motor's pole-pair count and the wheel's diameter, which are registered fields
     * defaulting to unset, so no board carries them until a bench measures that hub. The label
     * says count and implies no speed; converting this word is not worth a second read pass for a
     * figure the Ride panel refreshes many times a second.
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

    /**
     * What the board says it is doing ([BoardMode]), or null when nothing has arrived from it.
     *
     * The byte is the mode machine's sole output and the board's own statement about its power
     * path, which is a different fact from the app's arm control: the app asks, and this is the
     * answer. On the second board it is the only answer there has ever been.
     */
    val mode: BoardMode? get() = cyclic?.let { BoardMode.of(it.mode) }

    /** The board's rider-present flag, as the board sees it (not the phone's pad state). */
    val riderPresent: Boolean get() = cyclic?.riderPresent() ?: false

    /**
     * Whether the board says it is LEADING a rigid frame: `CYCLIC_STATE.flags` bit1
     * (`specs/rover-one-imu.md` section 2). False, not unknown, when nothing has arrived: a board
     * that has said nothing is not leading anything as far as this app can act on it.
     *
     * This is the resolved producer and not a stored setting, which is why the app reads it here
     * rather than off `control.frame`. The frame byte says what the machine is meant to be; the
     * flag says what the board actually resolved to this boot, which also folds in the fact the
     * store cannot answer: which of the two has the IMU. Only the board with one leads.
     */
    val leadsRigidFrame: Boolean get() = cyclic?.leader() ?: false

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
     * This wheel's LIFETIME travel in absolute hall edges, as the board reports it live; null as
     * [phasePeakCounts], and 0 from a board whose wheel has never turned.
     *
     * **Null is not zero here either.** Zero edges is a board that has reported and has never
     * moved, which is a reading worth drawing; null is a board nothing has arrived from.
     *
     * It is the board's stored total plus everything counted since it booted, so it climbs
     * through a ride and survives a power cycle. A Long because the wire word is a `u32`
     * (`crates/linkctl/src/lib.rs`, `CyclicObs`).
     *
     * COUNTS, not a distance. The conversion needs the motor's pole-pair count and the wheel's
     * diameter, both registered fields defaulting to unset, and both read per board on attach
     * ([com.hoverboard.remote.model.WheelGeometry]). This property stays the raw count whatever
     * they say: converting is the display's job
     * ([com.hoverboard.remote.model.odometerReading]), and a board told neither has nothing to
     * convert with.
     */
    val odometerEdges: Long? get() = cyclic?.obs?.odometer

    /**
     * The part the board detected at boot, or null when no state has arrived yet.
     * [ChipTag.Unknown] is different again: the board reported a byte this build does not
     * allocate, so it named a part the app cannot act on.
     */
    val chip: ChipTag? get() = cyclic?.obs?.chip

    /** True once any CYCLIC_STATE has been seen, so the panel can tell "waiting" from "zeroed". */
    val hasState: Boolean get() = cyclic != null

    /**
     * A fault-A producer asserted in the cyclic fault mask.
     *
     * `orchestrator::dispatch::cyclic_state` puts the board's gating-producer level mask in
     * `CYCLIC_STATE.fault` on every emission, and [faultLevel] is that mask with the two
     * non-fault bits (the mode fault and the power request) removed.
     *
     * The mask is the board's OWN producers (`orchestrator::events::Levels::own_mask`), so a
     * false here means "no producer is asserted on the board this app is attached to", not "the
     * machine is healthy", and the UI does not render it as the latter: see
     * [com.hoverboard.remote.ui.components.TelemetryPanel].
     */
    val anyFault: Boolean
        get() = faultLevel != 0

    /**
     * The cyclic fault mask's FAULT-A bits (`CyclicState.FAULT_A_BITS`, mirroring the firmware's
     * `events::FAULT_A_BITS`), 0 = no fault producer asserted. The mask also carries the mode
     * fault and the power request, which are not faults and are masked out here so that an armed
     * board does not light the chip by holding its own power request.
     */
    val faultLevel: Int get() = (cyclic?.fault ?: 0) and CyclicState.FAULT_A_BITS

    /** Fold a freshly decoded [CyclicState] in. Latest-wins: the board sends one state, cyclically. */
    fun merge(state: CyclicState): TelemetryUi = copy(cyclic = state)

    companion object {
        private const val CENTIVOLTS_PER_VOLT = 100f
        private const val CENTIDEGREES_PER_DEGREE = 100f
    }
}

/**
 * The `CYCLIC_STATE.mode` byte, named (`crates/state/src/mode.rs`, `Mode`).
 *
 * The firmware's mode machine has exactly these five states and emits its mode byte on every
 * cyclic frame. [UNRECOGNISED] is the app's own: a byte this build does not know, which is a board
 * running something newer, and is shown as such rather than guessed at.
 *
 * **The order of the entries is the wire's, not a reading order:** the five are declared in the
 * firmware enum's discriminant order so the byte indexes them directly, and [UNRECOGNISED] is last
 * so that every byte past the five lands on it. [of] is the one reader of that property.
 */
enum class BoardMode {
    /** Resting: motor path not brought up, bridge disabled. */
    OFF,

    /** The one-shot power-on: bringing the motor path up. */
    INIT,

    /** Powered, promoting to [RUN] on the next pass; torque still zero. */
    READY,

    /** Powered, and free to put torque on the wheel. */
    RUN,

    /** The one-shot safe-down: outputs forced safe on the way back to [OFF]. */
    SHUTDOWN,

    /** A mode byte this build does not name. */
    UNRECOGNISED,
    ;

    companion object {
        /** The mode byte as the board sent it, [UNRECOGNISED] for anything outside the five. */
        fun of(byte: Int): BoardMode = entries.getOrElse(byte) { UNRECOGNISED }
    }
}

/**
 * Which board is leading the rigid frame, as the two boards report themselves, or null when the
 * app cannot name one (`specs/rover-one-imu.md` section 2).
 *
 * Null covers three states a caller must treat alike, because none of them names a board that can
 * be driven as a rigid machine: neither board claims the lead (the ordinary pivoting pair, and a
 * rigid pair whose second half has gone quiet), or BOTH do, which is the two-leader staging error
 * the leader's own mode fault already refuses to engage on.
 */
fun leaderNode(master: TelemetryUi?, slave: TelemetryUi?): Node? {
    val masterLeads = master?.leadsRigidFrame == true
    val slaveLeads = slave?.leadsRigidFrame == true
    return when {
        masterLeads && !slaveLeads -> Node.MASTER
        slaveLeads && !masterLeads -> Node.SLAVE
        else -> null
    }
}
