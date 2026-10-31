package com.hoverboard.protocol.board

/**
 * The board layout's vocabulary, mirrored in Kotlin: the packed port|pin encoding
 * (`motorcontroller/board/src/lib.rs`, `Pin` and `ABSENT`), the dead-time floor the motor bring-up
 * holds a gate group to, and the field set a layout is made of.
 *
 * There is no verdict here. The firmware does not validate a layout as a whole: each subsystem
 * reads its own fields at boot and brings up what it can, and what did not come up is listed in
 * the board's own status record, read over the link as `board.status` ([BoardStatus],
 * `specs/configured-links.md` sections 3 and 5). A client names and picks pins with this file and
 * learns what the board made of them from that record after the next boot.
 *
 * Pin encoding (`specs/board-model.md`, "The field vocabulary"): one byte, `(port << 4) | pin`,
 * port A = 0, B = 1, C = 2, D = 3, F = 5, and [PIN_ABSENT] (`0xFF`) means the function is absent on
 * this board, a valid state rather than an error.
 */

/** The unset sentinel: the function is absent on this board (`board::ABSENT`, `store::PIN_ABSENT`). */
const val PIN_ABSENT = 0xFF

/**
 * The dead-time FLOOR a CONFIGURED gate group must meet, in `motor.dead_time` DTG ticks
 * (`motorcontroller/board/src/lib.rs`, `DEAD_TIME_MIN_DTG`, which carries the full derivation). The
 * motor bring-up does not bring up a gate group below it, and the status record says so
 * ([BoardStatus.Reason.DEAD_TIME]).
 *
 * The arithmetic in short: the gate timer is clocked at the 72 MHz sysclk with its dead-time
 * generator on the timer clock divided by 2, so one tick is 27.8 ns, and the 500 ns reference
 * figure, the shortest dead time any board in the fleet is measured running, is 18 ticks
 * (`specs/commutation.md`, the dead-time contracts). The floor admits every value the fleet runs
 * (25 on the 6-FET split board, 28 in the offroad stock dump, 32 on the 12-FET) and refuses the
 * range below it: 1 tick is 28 ns, roughly 25x too short, which is shoot-through.
 *
 * Zero is not measured against it. That is the right value for a board with no motor, so it is
 * legal with the gate group unset and the floor applies only where the gates are claimed.
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
 * One motor's layout field values, as read from the board's store (defaults already applied by
 * the board). The layout editor stages these; nothing here judges them.
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
    /** Raw DTG; 0 = unset. A configured gate group needs at least [DEAD_TIME_MIN_DTG]. */
    val deadTime: Int = 0,
    /** 0 = Forward, nonzero = Reverse. */
    val direction: Int = 0,
    /** The six-step align offset. */
    val alignOffset: Int = 0,
    /** The phase-current DECLARATION: 0 = none; nonzero brings up [phaseA] / [phaseB]. */
    val currentSense: Int = 0,
    /** Counts per amp. */
    val currentCal: Int = 0,
    val phaseA: Int = PIN_ABSENT,
    val phaseB: Int = PIN_ABSENT,
) {
    companion object {
        /** An all-absent motor: no halls, no gates, no dead-time, the carried facts at zero. */
        val ABSENT = MotorFields()
    }
}

/** A board's layout field values: the whole set the layout editor stages as one object. */
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
    /** The IMU bus pins, inert unless [imuModel] is nonzero. */
    val imuScl: Int = PIN_ABSENT,
    val imuSda: Int = PIN_ABSENT,
    /** 0 = no IMU fitted; nonzero = the imu crate's model index, the IMU group's enable. */
    val imuModel: Int = 0,
    val motors: List<MotorFields> = listOf(MotorFields.ABSENT, MotorFields.ABSENT),
) {
    init {
        require(motors.size == MOTORS) { "a board has $MOTORS motor field sets, got ${motors.size}" }
    }

    companion object {
        /** How many motors the field set carries (`shared/store/src/field.rs`, `MOTORS`). */
        const val MOTORS = 2
    }
}
