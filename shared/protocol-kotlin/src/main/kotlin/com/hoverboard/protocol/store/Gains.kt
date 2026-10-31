package com.hoverboard.protocol.store

/**
 * The live-tunable words of the state-feedback balance law (`specs/state-feedback.md` section 6), a
 * mirror of two store handles, `shared/store/src/field.rs` (`CONTROL_SF_GAINS`,
 * `CONTROL_REFERENCE`), and of the seam that guards them, `motorcontroller/control/src/config.rs`
 * (`GainShadow`, `REFERENCE_MAX`).
 *
 * Two keys of the live tune lane: the four gains, addressed as one `field_id` with the gain as the
 * key's `index`, and the two reference words the same way. A tune UI needs the facts this object
 * carries and no others: which key names a word, what a fresh board holds, and what the firmware
 * will accept.
 *
 * The RANGES are mirrored so a client can refuse a value before sending it, not so it can decide
 * one: the board enforces them at its own seam and answers `CFG_BAD` regardless of what a client
 * believes. `RustSourceDriftTest` pins every number here against the Rust that owns it.
 */
object Gains {
    /** The gains' `field_id` (`store::CONTROL_SF_GAINS`, `control::SF_GAINS_FIELD`). */
    const val CONTROL_SF_GAINS = 0x79

    /** The reference row's `field_id` (`store::CONTROL_REFERENCE`, `control::REFERENCE_FIELD`). */
    const val CONTROL_REFERENCE = 0x7B

    /** Gain index 0: `gp`, numerator counts per degree of pitch. */
    const val GP = 0

    /** Gain index 1: `gr`, numerator counts per 0.1 rad/s of pitch rate. */
    const val GR = 1

    /** Gain index 2: `gx`, numerator counts per hall edge of position error. */
    const val GX = 2

    /** Gain index 3: `gv`, numerator counts per sixteenth of a count of speed error. */
    const val GV = 3

    /** Indices of [CONTROL_SF_GAINS], `[GP, GR, GX, GV]` order (`control::config::sf::GAINS`). */
    const val COUNT = 4

    /** The gains' defaults: zero, the inert board (a zero word in balance mode). */
    val DEFAULT = listOf(0, 0, 0, 0)

    /** The inclusive lower bound of every gain index (`control::GAIN_MIN`). */
    const val MIN = 0

    /**
     * The default inclusive upper bound of each gain index (`control::DEFAULT_GAIN_MAX`, the
     * defaults of [Fields.CONTROL_GAIN_MAX]). A board's own maxima are that field as read from it;
     * this is what a client assumes while they are unread.
     */
    val DEFAULT_MAX = listOf(20000, 20000, 2000, 10000)

    /** Reference index 0: `speed_max`, sixteenths of a count at full stick; 0 = a dead stick. */
    const val SPEED_MAX = 0

    /** Reference index 1: `speed_slew`, 1/4096 count per control tick. */
    const val SPEED_SLEW = 1

    /** The reference row's defaults (`control::config::sf::REFERENCE_DEFAULT`). */
    val REFERENCE_DEFAULT = listOf(0, 41)

    /** The reference row's compiled inclusive minima (`control::config::sf::REFERENCE_MIN`). */
    val REFERENCE_MINIMA = listOf(0, 1)

    /** The reference row's compiled inclusive maxima (`control::config::sf::REFERENCE_MAX`). */
    val REFERENCE_MAXIMA = listOf(512, 4096)

    /** The [Key] naming one word: [CONTROL_SF_GAINS] or [CONTROL_REFERENCE] at an index. */
    fun key(field: Int, index: Int) = Key(field, index)

    /**
     * Whether [value] is one a board whose maximum at gain [index] is [max] will accept (the
     * client-side half of the seam); [max] defaults to [DEFAULT_MAX]. A negative [max] reads as
     * [MIN], as the firmware reads it. False for an index that names no gain.
     */
    fun inRange(index: Int, value: Int, max: Int? = DEFAULT_MAX.getOrNull(index)): Boolean =
        index in 0 until COUNT && max != null && value in MIN..maxOf(max, MIN)

    /** Whether [value] is one the board accepts at reference [index]. False for no such index. */
    fun referenceInRange(index: Int, value: Int): Boolean {
        val lo = REFERENCE_MINIMA.getOrNull(index) ?: return false
        return value in lo..REFERENCE_MAXIMA[index]
    }

    /**
     * The firmware's live-gain RAMP, as a client can know it (`specs/state-feedback.md` section 6,
     * `control::config::sf::RAMP_CAPS`).
     *
     * On every RUN pass of the 250 Hz control task the law steps each of its live gains toward the
     * staged shadow by at most a FIXED per-gain cap. So a staged change of `d` counts reaches the
     * running law within `ceil(d / cap)` RUN passes. A client uses this to bound how long the
     * engaged value can still differ from what it staged; it is a bound, never an observation: the
     * engaged gains are not on the wire. The reference row is not ramped.
     */
    object Ramp {
        /** Control passes per second: `scheduler::TICK_HZ`. */
        const val PASS_HZ = 250

        /** The per-pass step cap of each gain, `[GP, GR, GX, GV]` order. */
        val CAPS = listOf(2, 1, 1, 1)

        /** The per-pass step of gain [index]. */
        fun step(index: Int): Long = CAPS[index].toLong()

        /**
         * The longest the ramp can take, in milliseconds, to carry gain [index] across [distance]
         * counts: `ceil(distance / step)` passes, rounded up to whole milliseconds.
         */
        fun boundMs(index: Int, distance: Int): Long {
            if (distance <= 0) return 0
            val step = step(index)
            val passes = (distance + step - 1) / step
            return (passes * MS_PER_S + PASS_HZ - 1) / PASS_HZ
        }

        private const val MS_PER_S = 1000L
    }

    /** The default of one word, or null if the key names none. */
    fun default(field: Int, index: Int): Int? = when (field) {
        CONTROL_SF_GAINS -> DEFAULT.getOrNull(index)
        CONTROL_REFERENCE -> REFERENCE_DEFAULT.getOrNull(index)
        else -> null
    }
}
