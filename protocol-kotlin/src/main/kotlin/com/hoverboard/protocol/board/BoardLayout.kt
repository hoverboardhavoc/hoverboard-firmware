package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key

/**
 * The board-layout boot validator, mirrored in Kotlin: the packed port|pin encoding, the field set
 * a layout is made of, the set-level coherence checks and the chip-capability checks, as
 * `validate` in `crates/board/src/lib.rs` runs them at boot: the pure function
 * `specs/board-model.md` (section 2) calls "the judge".
 *
 * **Why a mirror exists at all.** `validate` is a pure function of three inputs: the staged fields,
 * the detected chip's capability answers, and the reserved pin set the caller computes. A client
 * that holds all three can predict the boot verdict EXACTLY rather than heuristically, which is
 * what lets a layout editor refuse a layout before it is written instead of discovering the refusal
 * as a board that booted link-only (`specs/rider-ui.md`, section 3.5). `RustSourceDriftTest` pins
 * this file against the Rust it mirrors, the same discipline the wire and registry mirrors run
 * under.
 *
 * **What it does NOT predict.** Whether the hardware then behaves (the IMU acknowledging on the
 * staged bus, halls reading, motors turning) is only knowable after a reboot, and so is the
 * board's own verdict: `BOARD_OBS` is an SWD-only symbol. A client renders THIS verdict and says
 * which it is.
 *
 * Pin encoding (`specs/board-model.md`, "The field vocabulary"): one byte, `(port << 4) | pin`,
 * port A = 0, B = 1, C = 2, D = 3, F = 5, and [PIN_ABSENT] (`0xFF`) means the function is absent on
 * this board, a valid state rather than an error.
 */

/** The unset sentinel: the function is absent on this board (`board::ABSENT`, `store::PIN_ABSENT`). */
const val PIN_ABSENT = 0xFF

/**
 * The dead-time FLOOR a CONFIGURED gate group must meet, in `motor.dead_time` DTG ticks
 * (`crates/board/src/lib.rs`, `DEAD_TIME_MIN_DTG`, which carries the full derivation).
 *
 * The arithmetic in short: the gate timer is clocked at the 72 MHz sysclk with its dead-time
 * generator on the timer clock divided by 2, so one tick is 27.8 ns, and the 500 ns reference
 * figure, the shortest dead time any board in the fleet is measured running, is 18 ticks
 * (`specs/commutation.md`, the dead-time contracts). The floor admits every value the fleet runs
 * (25 on the 6-FET split board, 28 in the offroad stock dump, 32 on the 12-FET) and refuses the
 * range below it: 1 tick is 28 ns, roughly 25x too short, which is shoot-through.
 *
 * Zero is not measured against it. That is the right value for a board with no motor, so it is
 * legal with the gate group unset and the floor applies only where the gates are claimed
 * (`specs/board-model.md`, section 2, check 2).
 *
 * The editor range of any client that offers this field FOLLOWS this constant rather than
 * restating it, so a client cannot offer a value the board would refuse.
 */
const val DEAD_TIME_MIN_DTG = 18

/** The port letters the encoding defines, by port index; `null` where it defines none. */
private val PORT_LETTERS = listOf('A', 'B', 'C', 'D', null, 'F')

/**
 * A parsed, encoding-valid pin (`board::Pin`), held as the [packed] byte the field stores.
 *
 * The constructor is internal because the type's whole claim is that its byte decodes: [parse] is
 * the only way in, so a `Pin` in hand is an encoding-valid one and the `0xFF` sentinel and the
 * undefined ports are [Parsed] cases instead of representable pins.
 */
@JvmInline
value class Pin internal constructor(val packed: Int) {
    /** The port index (A = 0, B = 1, C = 2, D = 3, F = 5). */
    val port: Int get() = packed shr 4

    /** The pin number within the port (0..15). */
    val pin: Int get() = packed and 0x0F

    /**
     * The pin as a bench name, `PB12`. A client-side spelling of the encoding (the preset contract
     * of `specs/board-model.md`, section 4, writes pins this way); the firmware carries no formatter.
     */
    val name: String get() = "P${PORT_LETTERS[port]}$pin"

    override fun toString(): String = name

    companion object {
        /**
         * Parse a packed field byte the way `board::Pin::parse` does: [Parsed.Absent] for
         * [PIN_ABSENT], [Parsed.Valid] for a byte whose port nibble the encoding defines, and
         * [Parsed.Invalid] otherwise (port 4 = the nonexistent port E, or anything above F).
         */
        fun parse(raw: Int): Parsed {
            if (raw == PIN_ABSENT) return Parsed.Absent
            val port = raw shr 4
            val valid = port in 0..3 || port == 5
            return if (valid) Parsed.Valid(Pin(raw)) else Parsed.Invalid
        }

        /** The pin [name] spells, or null when it spells none (the inverse of [name]). */
        fun byName(name: String): Pin? {
            val letter = name.removePrefix("P").firstOrNull() ?: return null
            val port = PORT_LETTERS.indexOf(letter).takeIf { it >= 0 } ?: return null
            val n = name.removePrefix("P").drop(1).toIntOrNull() ?: return null
            return if (n in 0..15) Pin((port shl 4) or n) else null
        }
    }
}

/** The three outcomes of parsing a packed field byte (`board::Parsed`). */
sealed interface Parsed {
    /** `0xFF`: the function is absent (valid). */
    data object Absent : Parsed

    /** A byte the encoding does not define. */
    data object Invalid : Parsed

    /** An encoding-valid pin. */
    data class Valid(val pin: Pin) : Parsed
}

/**
 * The chip-capability seam (`board::Capabilities`): what the validator asks of the detected chip.
 * [ChipFamily] answers it for the fleet's parts.
 */
interface Capabilities {
    /** Does this pin exist on the detected chip? */
    fun pinExists(pin: Pin): Boolean

    /**
     * Is this pin gate-capable (an advanced-timer main or complementary channel)? Gate-capable pins
     * refuse non-gate functions (the denylist rule).
     */
    fun gateCapable(pin: Pin): Boolean

    /**
     * Do these six pins form a known-valid complementary assignment of ONE advanced timer? Returns
     * the advanced-timer index (0 = TIMER0, 1 = TIMER7/TIM8), or null.
     */
    fun gateSet(hi: List<Pin>, lo: List<Pin>): Int?

    /** The ADC channel behind an analog-capable pin, if any. */
    fun adcChannel(pin: Pin): Int?

    /**
     * Does this (SCL, SDA) pair form a hardware-I2C instance? Returns the instance index in the
     * GD numbering (PB6/PB7 = I2C0 = 0, PB10/PB11 = I2C1 = 1), never the ST 1-based names.
     */
    fun i2cPair(scl: Pin, sda: Pin): Int?
}

/**
 * Which field a validator failure names (`board::BoardField`), as the registered store field it
 * reads from.
 *
 * [def] is the registry entry, which owns the field's id, type and default ([Fields], which
 * `RustSourceDriftTest` pins against `crates/store/src/field.rs`, the `Field` handles); this enum owns only the mapping from a validator field to
 * that entry, which is the mapping `board::plumbing`'s `field_id` makes on the firmware side and
 * `RustSourceDriftTest` pins against it. [perMotor] is whether the field is addressed per motor
 * (`Key.index` = the motor; a singleton uses index 0).
 *
 * The two IMU frame fields are members because the Rust enum has them, and like the Rust they are
 * never named by [validate]: the frame refusal is the firmware's own boot check over
 * `IMU_AXIS_SIGN` / `IMU_AXIS_ROLE` (`imu::Config::frame_is_rotation`, mirrored by
 * [com.hoverboard.protocol.imu.Orientation]), which fails the IMU rather than the layout.
 */
enum class BoardField(val def: FieldDef, val perMotor: Boolean = false) {
    SELF_HOLD(Fields.BOARD_SELF_HOLD),
    VBATT(Fields.BOARD_VBATT),
    BUZZER(Fields.BOARD_BUZZER),
    LED_GREEN(Fields.LED_GREEN),
    LED_ORANGE(Fields.LED_ORANGE),
    LED_RED(Fields.LED_RED),
    PAD_A(Fields.PAD_A),
    PAD_B(Fields.PAD_B),
    BUTTON(Fields.BOARD_BUTTON),
    IMU_SCL(Fields.IMU_SCL_PIN),
    IMU_SDA(Fields.IMU_SDA_PIN),
    IMU_MODEL(Fields.IMU_MODEL),
    IMU_AXIS_SIGN(Fields.IMU_AXIS_SIGN),
    IMU_AXIS_ROLE(Fields.IMU_AXIS_ROLE),
    HALL_A(Fields.MOTOR_HALL_A, perMotor = true),
    HALL_B(Fields.MOTOR_HALL_B, perMotor = true),
    HALL_C(Fields.MOTOR_HALL_C, perMotor = true),
    GATE_HI_A(Fields.MOTOR_GATE_HI_A, perMotor = true),
    GATE_HI_B(Fields.MOTOR_GATE_HI_B, perMotor = true),
    GATE_HI_C(Fields.MOTOR_GATE_HI_C, perMotor = true),
    GATE_LO_A(Fields.MOTOR_GATE_LO_A, perMotor = true),
    GATE_LO_B(Fields.MOTOR_GATE_LO_B, perMotor = true),
    GATE_LO_C(Fields.MOTOR_GATE_LO_C, perMotor = true),
    DEAD_TIME(Fields.MOTOR_DEAD_TIME, perMotor = true),
    PHASE_A(Fields.MOTOR_PHASE_A, perMotor = true),
    PHASE_B(Fields.MOTOR_PHASE_B, perMotor = true),
    CURRENT_SENSE(Fields.MOTOR_CURRENT_SENSE, perMotor = true),
    ;

    /** The registry id the `BOARD_OBS` record names this field by. */
    val id: Int get() = def.id

    /** The key naming this field on [motor] (a singleton ignores it and uses index 0). */
    fun key(motor: Int? = null): Key = def.key(if (perMotor) motor ?: 0 else 0)

    /** Whether this field holds one of a motor's six gate pins (the denylist rule's exemption). */
    val isGate: Boolean
        get() = this in setOf(GATE_HI_A, GATE_HI_B, GATE_HI_C, GATE_LO_A, GATE_LO_B, GATE_LO_C)
}

/** The offending field of a failure: which field, and for a per-motor field, which motor. */
data class FieldRef(val field: BoardField, val motor: Int? = null)

/**
 * What went wrong (`board::BoardErrorKind`), with the `BOARD_OBS` result code and detail word the
 * firmware would report it as (`board::plumbing`, `BoardObs::failure`). The codes are what makes
 * this mirror's verdict comparable to the board's own, once that record is readable.
 */
sealed interface BoardErrorKind {
    /** The `BOARD_OBS` result code (1..11; 0 is the success record). */
    val obsResult: Int

    /** The `BOARD_OBS` detail word: the packed pin, the raw byte, or 0 where no pin is involved. */
    val obsDetail: Int

    /** The byte is not a valid pin encoding (carries the raw byte). */
    data class BadEncoding(val raw: Int) : BoardErrorKind {
        override val obsResult = 1
        override val obsDetail = raw
    }

    /** A function group is partially present (all-or-none rule). */
    data object IncompleteGroup : BoardErrorKind {
        override val obsResult = 2
        override val obsDetail = 0
    }

    /**
     * A configured gate group's `motor.dead_time` is below [DEAD_TIME_MIN_DTG] (the unset 0
     * among them: a claimed gate group has to carry a dead time).
     */
    data object DeadTimeBelowFloor : BoardErrorKind {
        override val obsResult = 3
        override val obsDetail = 0
    }

    /** The pin is already assigned to another field. */
    data class DuplicatePin(val pin: Pin) : BoardErrorKind {
        override val obsResult = 4
        override val obsDetail = pin.packed
    }

    /** The pin collides with the caller-computed reserved set (the live link ports / SWD). */
    data class ReservedPin(val pin: Pin) : BoardErrorKind {
        override val obsResult = 5
        override val obsDetail = pin.packed
    }

    /** The detected chip has no such pin. */
    data class UnknownPin(val pin: Pin) : BoardErrorKind {
        override val obsResult = 6
        override val obsDetail = pin.packed
    }

    /** A gate-capable pin appears in a non-gate field (the denylist rule). */
    data class GateCapableMisused(val pin: Pin) : BoardErrorKind {
        override val obsResult = 7
        override val obsDetail = pin.packed
    }

    /** The six gate pins form no valid advanced-timer assignment on this chip. */
    data object InvalidGateSet : BoardErrorKind {
        override val obsResult = 8
        override val obsDetail = 0
    }

    /** The pin has no ADC channel behind it (vbatt and each phase-current pin must be analog). */
    data class NotAdcCapable(val pin: Pin) : BoardErrorKind {
        override val obsResult = 9
        override val obsDetail = pin.packed
    }

    /** The pin pair forms no hardware-I2C instance on this chip (the software bus is not built). */
    data object NotI2cPair : BoardErrorKind {
        override val obsResult = 10
        override val obsDetail = 0
    }

    /**
     * The staged IMU frame is not a proper rotation. Never produced by [validate], exactly as in
     * the Rust: it is the firmware's own IMU-frame refusal, which fails the IMU rather than the
     * layout. Mirrored so the result codes are the complete set a `BOARD_OBS` read can carry.
     */
    data class ImuFrame(val firstSignIndex: Int) : BoardErrorKind {
        override val obsResult = 11
        override val obsDetail = firstSignIndex
    }
}

/** A validator failure: the first failing check, naming the offending field. */
data class BoardError(val field: FieldRef, val kind: BoardErrorKind)

/**
 * One motor's raw field values (`board::MotorFields`), as read from the registry (defaults already
 * applied: this mirror carries no defaults of its own, exactly as the crate carries none).
 */
data class MotorFields(
    val hallA: Int = PIN_ABSENT,
    val hallB: Int = PIN_ABSENT,
    val hallC: Int = PIN_ABSENT,
    val gateHiA: Int = PIN_ABSENT,
    val gateHiB: Int = PIN_ABSENT,
    val gateHiC: Int = PIN_ABSENT,
    val gateLoA: Int = PIN_ABSENT,
    val gateLoB: Int = PIN_ABSENT,
    val gateLoC: Int = PIN_ABSENT,
    /** Raw DTG; 0 = unset. Required nonzero exactly when the gate group is configured. */
    val deadTime: Int = 0,
    /** 0 = Forward, nonzero = Reverse. Carried into the plan, never validated. */
    val direction: Int = 0,
    /** The six-step align offset. Carried, not range-checked (the crate takes it mod 6). */
    val alignOffset: Int = 0,
    /**
     * The phase-current capability DECLARATION: 0 = none. Validated to agree with [phaseA] /
     * [phaseB], so a board cannot declare current sense with no channels behind it.
     */
    val currentSense: Int = 0,
    /** Counts per amp. Carried raw; the firmware's boot seam is what clamps it. */
    val currentCal: Int = 0,
    val phaseA: Int = PIN_ABSENT,
    val phaseB: Int = PIN_ABSENT,
) {
    companion object {
        /** An all-absent motor: no halls, no gates, no dead-time, the carried facts at zero. */
        val ABSENT = MotorFields()
    }
}

/** The raw board field set (`board::BoardFields`): what [validate] judges. */
data class BoardFields(
    val selfHold: Int = PIN_ABSENT,
    val vbatt: Int = PIN_ABSENT,
    val buzzer: Int = PIN_ABSENT,
    val ledGreen: Int = PIN_ABSENT,
    val ledOrange: Int = PIN_ABSENT,
    val ledRed: Int = PIN_ABSENT,
    val padA: Int = PIN_ABSENT,
    val padB: Int = PIN_ABSENT,
    val button: Int = PIN_ABSENT,
    val imuScl: Int = PIN_ABSENT,
    val imuSda: Int = PIN_ABSENT,
    /** 0 = no IMU fitted; nonzero = the imu crate's model index. */
    val imuModel: Int = 0,
    val motors: List<MotorFields> = listOf(MotorFields.ABSENT, MotorFields.ABSENT),
) {
    init {
        require(motors.size == MOTORS) { "a board has $MOTORS motor field sets, got ${motors.size}" }
    }

    companion object {
        /** How many motors the field set carries (`board::BoardFields::motors`). */
        const val MOTORS = 2
    }
}

/** The validated hall group of one motor. */
data class HallSet(val a: Pin, val b: Pin, val c: Pin)

/** The validated gate group of one motor, with its dead-time and the advanced timer derived for it. */
data class GateSet(val hi: List<Pin>, val lo: List<Pin>, val deadTime: Int, val timer: Int)

/** A validated analog input, with the ADC channel derived behind its pin. */
data class AdcInput(val pin: Pin, val channel: Int)

/** The validated IMU group, with the hardware-I2C instance derived for its pair. */
data class ImuPlan(val scl: Pin, val sda: Pin, val model: Int, val bus: Int)

/**
 * The validated phase-current group of one motor: the two pins and their derived ADC channels, in
 * injected-rank order. Its presence IS the motor's current-sense fact.
 */
data class PhaseCurrentSet(val pins: List<Pin>, val channels: List<Int>)

/** One motor's validated plan plus the carried per-motor config facts (`board::MotorPlan`). */
data class MotorPlan(
    val halls: HallSet? = null,
    val gates: GateSet? = null,
    val direction: Boolean = false,
    val alignOffset: Int = 0,
    val currentCal: Int = 0,
    val phaseCurrent: PhaseCurrentSet? = null,
)

/**
 * The validated board plan (`board::BoardPlan`): absent means the function does not exist on this
 * board. The power latch is deliberately not here; it rides on [Validated].
 */
data class BoardPlan(
    val vbatt: AdcInput? = null,
    val buzzer: Pin? = null,
    val ledGreen: Pin? = null,
    val ledOrange: Pin? = null,
    val ledRed: Pin? = null,
    val padA: Pin? = null,
    val padB: Pin? = null,
    val button: Pin? = null,
    val imu: ImuPlan? = null,
    val motors: List<MotorPlan> = listOf(MotorPlan(), MotorPlan()),
)

/** The verdict on everything but the power latch: one plan, or the first failure. */
sealed interface Verdict {
    /** The layout is coherent on this chip; the bring-up consumes [plan]. */
    data class Valid(val plan: BoardPlan) : Verdict

    /** The layout is refused, and the board boots link-only. */
    data class Invalid(val error: BoardError) : Verdict
}

/**
 * What applying a staged layout yields (`board::Validated`): the power-latch pin, and the verdict
 * on everything else.
 *
 * Two results rather than one, because the latch has to survive a failure elsewhere: a board that
 * boots link-only on a bad layout can only be reached to be corrected while its rail is up. So
 * `board.self_hold` is resolved FIRST, and reported here whatever happens next. [selfHold] is null
 * in exactly two cases: the field is staged absent, or the staged byte named no usable pin, in
 * which case [verdict] carries that failure against [BoardField.SELF_HOLD].
 */
data class Validated(val selfHold: Pin?, val verdict: Verdict) {
    /** The plan, or null when the layout was refused. */
    val plan: BoardPlan? get() = (verdict as? Verdict.Valid)?.plan

    /** The first failure, or null when the layout validated. */
    val error: BoardError? get() = (verdict as? Verdict.Invalid)?.error
}

/**
 * The boot validation (`validate` in `crates/board/src/lib.rs`;
 * `specs/board-model.md`, "The boot validator", checks 1-4):
 * parse plus chip-existence validity, group completeness including the rule that a configured gate
 * group carries a dead time at or above [DEAD_TIME_MIN_DTG], duplicates across the whole set,
 * reserved-pin collisions, then the capability stage (gate-capable pins refuse non-gate functions;
 * vbatt and each phase-current pin must be ADC-capable; the IMU pair must form a hardware-I2C
 * instance; a configured gate set must form a valid advanced-timer assignment).
 *
 * @param caps the detected chip's capability answers ([ChipFamily] for the fleet's parts).
 * @param reserved the caller-computed reserved pins no field may claim, packed: the safe-USART
 *   allowlist minus the ports `LINK_SET` has freed, plus SWD ([reservedSet]).
 *
 * The power latch needs no entry in [reserved]: `self_hold` is the first field taken, so its claim
 * refuses every later claimant and the pin held is the STAGED one by construction.
 *
 * First failure wins in check order: per-field failures in field order, then the capability stage.
 */
fun validate(fields: BoardFields, caps: Capabilities, reserved: List<Int>): Validated =
    Validator(caps, reserved).run(fields)

/** The first failure, thrown so the check order reads as the Rust's `?` chain does. */
private class Refused(val error: BoardError) : Exception(null, null, false, false)

/**
 * One run of [validate]: the claims table plus the checks over it, in the Rust's order.
 *
 * A class rather than nested closures for one reason: the duplicate check and the gate-capable
 * denylist read the SAME claims table, the first as it fills and the second once it is full, and
 * that table is the only state a validation has.
 */
private class Validator(private val caps: Capabilities, private val reserved: List<Int>) {

    /** Every assigned pin, in field order, so the SECOND claimant is the named offender. */
    private val claimed = mutableListOf<Pair<Pin, FieldRef>>()

    fun run(fields: BoardFields): Validated {
        // The latch FIRST, and outside the rest of the pass: its answer is needed whatever the rest
        // of the layout turns out to be. Taking it here is also what reserves its pin, since the
        // claim it leaves in the table refuses every later claimant.
        val selfHold = try {
            take(fields.selfHold, single(BoardField.SELF_HOLD))
        } catch (e: Refused) {
            return Validated(selfHold = null, verdict = Verdict.Invalid(e.error))
        }
        // Everything else, in check order, short-circuiting the REST of the layout without
        // discarding the latch above.
        val verdict = try {
            Verdict.Valid(rest(fields))
        } catch (e: Refused) {
            Verdict.Invalid(e.error)
        }
        return Validated(selfHold, verdict)
    }

    /** Checks 1 and 3 for one field: parse, chip existence, the reserved set, then duplicates. */
    private fun take(raw: Int, ref: FieldRef): Pin? {
        val pin = when (val p = Pin.parse(raw)) {
            Parsed.Invalid -> throw Refused(BoardError(ref, BoardErrorKind.BadEncoding(raw)))
            Parsed.Absent -> return null
            is Parsed.Valid -> p.pin
        }
        if (!caps.pinExists(pin)) throw Refused(BoardError(ref, BoardErrorKind.UnknownPin(pin)))
        if (reserved.any { it == pin.packed }) throw Refused(BoardError(ref, BoardErrorKind.ReservedPin(pin)))
        if (claimed.any { (other, _) -> other == pin }) {
            throw Refused(BoardError(ref, BoardErrorKind.DuplicatePin(pin)))
        }
        claimed += pin to ref
        return pin
    }

    private fun single(f: BoardField) = FieldRef(f, null)

    @Suppress("CyclomaticComplexMethod", "LongMethod", "ThrowsCount")
    private fun rest(fields: BoardFields): BoardPlan {
        // Singletons (the pure-pin ones assemble directly; vbatt waits for its ADC derivation).
        val vbattPin = take(fields.vbatt, single(BoardField.VBATT))
        val buzzer = take(fields.buzzer, single(BoardField.BUZZER))
        val ledGreen = take(fields.ledGreen, single(BoardField.LED_GREEN))
        val ledOrange = take(fields.ledOrange, single(BoardField.LED_ORANGE))
        val ledRed = take(fields.ledRed, single(BoardField.LED_RED))
        val padA = take(fields.padA, single(BoardField.PAD_A))
        val padB = take(fields.padB, single(BoardField.PAD_B))
        val button = take(fields.button, single(BoardField.BUTTON))

        // The IMU group (check 2: both pins plus a nonzero model, all-or-none). The hardware-I2C
        // derivation waits for the capability stage.
        val imuScl = take(fields.imuScl, single(BoardField.IMU_SCL))
        val imuSda = take(fields.imuSda, single(BoardField.IMU_SDA))
        val imuGroup: Triple<Pin, Pin, Int>? = when {
            imuScl == null && imuSda == null && fields.imuModel == 0 -> null
            imuScl != null && imuSda != null && fields.imuModel != 0 -> Triple(imuScl, imuSda, fields.imuModel)
            // Partially present: name the first absent or odd member of the group.
            imuScl == null -> throw Refused(BoardError(single(BoardField.IMU_SCL), BoardErrorKind.IncompleteGroup))
            imuSda == null -> throw Refused(BoardError(single(BoardField.IMU_SDA), BoardErrorKind.IncompleteGroup))
            else -> throw Refused(BoardError(single(BoardField.IMU_MODEL), BoardErrorKind.IncompleteGroup))
        }

        // The motor groups (check 2: halls all-or-none; gates all-or-none; configured gates require
        // a dead time at or above the floor; the phase pair all-or-none AND present exactly when
        // current_sense is nonzero).
        val gateGroups = arrayOfNulls<Triple<List<Pin>, List<Pin>, Int>>(BoardFields.MOTORS)
        val phaseGroups = arrayOfNulls<List<Pin>>(BoardFields.MOTORS)
        val motors = MutableList(BoardFields.MOTORS) { MotorPlan() }
        for ((m, mf) in fields.motors.withIndex()) {
            // The carried per-motor config facts (not validated; the fold-back of
            // `specs/motor-integration.md`, the fold-back). They ride the plan whatever the
            // groups turn out to be.
            motors[m] = MotorPlan(
                direction = mf.direction != 0,
                alignOffset = mf.alignOffset,
                currentCal = mf.currentCal,
            )
            motors[m] = motors[m].copy(halls = halls(mf, m))
            gateGroups[m] = gates(mf, m)
            phaseGroups[m] = phases(mf, m)
        }

        // --- Check 4, the capability stage (after every set-level check, spec order) ---

        // 4a. Gate-capable pins refuse non-gate functions (the denylist rule), over every claimed
        // NON-gate pin in field order. Gate pins are exempt by field identity.
        for ((pin, ref) in claimed) {
            if (!ref.field.isGate && caps.gateCapable(pin)) {
                throw Refused(BoardError(ref, BoardErrorKind.GateCapableMisused(pin)))
            }
        }

        // 4b. vbatt must have an ADC channel behind it; the derived channel rides in the plan.
        val vbatt = vbattPin?.let { pin ->
            val channel = caps.adcChannel(pin)
                ?: throw Refused(BoardError(single(BoardField.VBATT), BoardErrorKind.NotAdcCapable(pin)))
            AdcInput(pin, channel)
        }

        // 4c. The IMU pair must form a hardware-I2C instance (the bus-kind derivation of
        // `specs/imu.md`, the bus kind; the software-I2C variant is not built, so no pair
        // is invalid).
        val imu = imuGroup?.let { (scl, sda, model) ->
            val bus = caps.i2cPair(scl, sda)
                ?: throw Refused(BoardError(single(BoardField.IMU_SCL), BoardErrorKind.NotI2cPair))
            ImuPlan(scl, sda, model, bus)
        }

        // 4d. A configured gate set must form a valid advanced-timer assignment; the derived timer
        // rides in the plan.
        for ((m, group) in gateGroups.withIndex()) {
            val (hi, lo, deadTime) = group ?: continue
            val timer = caps.gateSet(hi, lo)
                ?: throw Refused(BoardError(FieldRef(BoardField.GATE_HI_A, m), BoardErrorKind.InvalidGateSet))
            motors[m] = motors[m].copy(gates = GateSet(hi, lo, deadTime, timer))
        }

        // 4e. Each phase-current pin must have an ADC channel behind it (the same query vbatt uses);
        // the derived channels ride in the plan, in injected-rank order.
        for ((m, pins) in phaseGroups.withIndex()) {
            if (pins == null) continue
            val channels = pins.mapIndexed { i, pin ->
                caps.adcChannel(pin) ?: throw Refused(
                    BoardError(
                        FieldRef(if (i == 0) BoardField.PHASE_A else BoardField.PHASE_B, m),
                        BoardErrorKind.NotAdcCapable(pin),
                    ),
                )
            }
            motors[m] = motors[m].copy(phaseCurrent = PhaseCurrentSet(pins, channels))
        }

        return BoardPlan(vbatt, buzzer, ledGreen, ledOrange, ledRed, padA, padB, button, imu, motors)
    }

    /** One motor's hall group: all three or none, naming the first absent member of a partial one. */
    private fun halls(mf: MotorFields, m: Int): HallSet? {
        val group = listOf(
            mf.hallA to BoardField.HALL_A,
            mf.hallB to BoardField.HALL_B,
            mf.hallC to BoardField.HALL_C,
        )
        val pins = group.map { (raw, f) -> take(raw, FieldRef(f, m)) }
        return when (pins.count { it != null }) {
            0 -> null
            group.size -> HallSet(pins[0]!!, pins[1]!!, pins[2]!!)
            else -> throw Refused(incomplete(group, pins, m))
        }
    }

    /** One motor's gate group: all six with a dead-time at or above [DEAD_TIME_MIN_DTG], or none. */
    private fun gates(mf: MotorFields, m: Int): Triple<List<Pin>, List<Pin>, Int>? {
        val group = listOf(
            mf.gateHiA to BoardField.GATE_HI_A,
            mf.gateHiB to BoardField.GATE_HI_B,
            mf.gateHiC to BoardField.GATE_HI_C,
            mf.gateLoA to BoardField.GATE_LO_A,
            mf.gateLoB to BoardField.GATE_LO_B,
            mf.gateLoC to BoardField.GATE_LO_C,
        )
        val pins = group.map { (raw, f) -> take(raw, FieldRef(f, m)) }
        return when (pins.count { it != null }) {
            0 -> null
            group.size -> {
                // The table's rule, carried in check 2: a configured gate group requires a dead
                // time at or above the floor, not merely a nonzero one. The unset 0 is one of the
                // values this refuses, and it stays legal in the `0` branch above, where the gates
                // are not claimed.
                if (mf.deadTime < DEAD_TIME_MIN_DTG) {
                    throw Refused(BoardError(FieldRef(BoardField.DEAD_TIME, m), BoardErrorKind.DeadTimeBelowFloor))
                }
                Triple(pins.subList(0, 3).map { it!! }, pins.subList(3, group.size).map { it!! }, mf.deadTime)
            }
            else -> throw Refused(incomplete(group, pins, m))
        }
    }

    /**
     * One motor's phase-current group: the two pins all-or-none, AND present exactly when the
     * `motor.current_sense` declaration is nonzero. A declaration and the data realizing it may not
     * disagree, so the bring-up never sees a half-state.
     */
    private fun phases(mf: MotorFields, m: Int): List<Pin>? {
        val group = listOf(mf.phaseA to BoardField.PHASE_A, mf.phaseB to BoardField.PHASE_B)
        val pins = group.map { (raw, f) -> take(raw, FieldRef(f, m)) }
        val declared = mf.currentSense != 0
        return when {
            pins.all { it == null } && !declared -> null
            pins.all { it != null } && declared -> pins.map { it!! }
            // Declared but not wired (or partially wired): name the first absent pin.
            declared -> throw Refused(incomplete(group, pins, m))
            // Wired (fully or partially) but not declared: name the declaration.
            else -> throw Refused(
                BoardError(FieldRef(BoardField.CURRENT_SENSE, m), BoardErrorKind.IncompleteGroup),
            )
        }
    }

    /** The incomplete-group refusal naming the first ABSENT member of a partially present group. */
    private fun incomplete(group: List<Pair<Int, BoardField>>, pins: List<Pin?>, m: Int): BoardError =
        BoardError(FieldRef(group[pins.indexOfFirst { it == null }].second, m), BoardErrorKind.IncompleteGroup)
}
