package com.hoverboard.protocol.store

/**
 * The balance-PID gain fields, a mirror of the two `CONTROL_GAIN_*` handles in
 * `crates/store/src/field.rs` and of the seam that guards them in `crates/control/src/config.rs`
 * (`specs/rider-ui.md` section 4).
 *
 * Two profiles, three gains each, addressed as one `field_id` per profile with the gain as the
 * key's `index`. A tune UI needs all four facts this object carries and no others: which key names
 * a gain, what a fresh board holds, and what the firmware will accept.
 *
 * The RANGES are mirrored so a client can refuse a value before sending it, not so it can decide
 * one: the board enforces them at its own seam and answers `CFG_BAD` regardless of what a client
 * believes. `RustSourceDriftTest` pins every number here against the Rust that owns it.
 */
object Gains {
    /** Profile A's `field_id`: the rider-present gains (`store::CONTROL_GAIN_A`). */
    const val CONTROL_GAIN_A = 0x71

    /** Profile B's `field_id`: the rider-absent gains (`store::CONTROL_GAIN_B`). */
    const val CONTROL_GAIN_B = 0x72

    /** Key index 0: the proportional gain `kp`. */
    const val KP = 0

    /** Key index 1: the battery-normalisation / rate coefficient `bk`. */
    const val BK = 1

    /** Key index 2: the derivative rate word `pr`. */
    const val PR = 2

    /** Indices per profile, in `[KP, BK, PR]` order (`control::GAINS_PER_PROFILE`). */
    const val PER_PROFILE = 3

    /** Profile A's defaults, index by index: what a board that has never been tuned runs. */
    val DEFAULT_A = listOf(6000, 2000, 40)

    /** Profile B's defaults, index by index. */
    val DEFAULT_B = listOf(3000, 1000, 30)

    /** The inclusive range each index accepts, mirrored from `control::GAIN_RANGE`. */
    val RANGE = listOf(0..20000, 0..10000, 0..1000)

    /** The [Key] naming one gain: [CONTROL_GAIN_A] or [CONTROL_GAIN_B] at [KP] / [BK] / [PR]. */
    fun key(profile: Int, index: Int) = Key(profile, index)

    /** Whether [value] is one this board will accept at [index] (the client-side half of the seam). */
    fun inRange(index: Int, value: Int): Boolean = RANGE.getOrNull(index)?.contains(value) == true

    /**
     * The firmware's live-gain RAMP, as far as a client can know it (`specs/rider-ui.md` section 4,
     * "Mid-loop discipline"; `control::config::ramp`).
     *
     * On every RUN pass of the 250 Hz control task the engagement machine steps each gain of the
     * triple the loop runs (ENGAGED) toward the staged shadow by at most a per-pass cap, floored at 1
     * count. So a staged change of `d` counts reaches the running loop within `ceil(d / cap)` RUN
     * passes, and never sooner than the firmware allows. A client uses this to bound how long the
     * engaged value can still differ from what it staged; it is a bound, never an observation: the
     * engaged triple is not on the wire.
     *
     * The caps of `kp` and `bk` depend only on the board's effective battery word (the PID's
     * `scale`, which `CYCLIC_STATE.battery` carries), so they are mirrored. `pr`'s cap also divides
     * by the live `kd`, which no wire key carries, so a client can only rely on the floor of one
     * count per pass for it.
     */
    object Ramp {
        /** Control passes per second: `scheduler::TICK_HZ`. */
        const val PASS_HZ = 250

        /** The torque-count share of the slew limit `kp`'s step may use (`ramp::KP_SHARE`). */
        const val KP_SHARE = 180

        /** The share `bk`'s step may use (`ramp::BK_SHARE`). */
        const val BK_SHARE = 45

        /** The worst-case `|pp|`, centidegrees: `ramp::PP_BOUND`, which is `fsm::UPRIGHT_LIMIT`. */
        const val PP_BOUND = 2499

        /** The worst-case `|bv|`, rad/s x 10000 (`ramp::BV_BOUND`). */
        const val BV_BOUND = 87_266

        /** `pid::PROP_DIVISOR`, the divisor of the `kp` term. */
        const val PROP_DIVISOR = 100

        /** `pid::BATT_DIVISOR`, the divisor of the `bk` term. */
        const val BATT_DIVISOR = 10_000

        /** `pid::RAW_NUMERATOR`, the numerator of the PID's `* 3900 / scale`. */
        const val RAW_NUMERATOR = 3900

        /**
         * The per-pass step of gain [index] at battery word [scale] (centivolts; null or not
         * positive = unknown), floored at 1 as the firmware floors it. `pr` is always 1: its cap
         * depends on `kd`, which is not readable, and 1 is the step the firmware guarantees.
         */
        fun step(index: Int, scale: Int?): Long {
            val s = scale?.takeIf { it > 0 }?.toLong() ?: 0L
            val cap = when (index) {
                KP -> s * KP_SHARE * PROP_DIVISOR / (RAW_NUMERATOR.toLong() * PP_BOUND)
                BK -> s * BK_SHARE * BATT_DIVISOR / (RAW_NUMERATOR.toLong() * BV_BOUND)
                else -> 0L
            }
            return maxOf(cap, 1L)
        }

        /**
         * The longest the ramp can take, in milliseconds, to carry gain [index] across [distance]
         * counts at battery word [scale] (null = unknown, which assumes the floor of one count per
         * pass): `ceil(distance / step)` passes, rounded up to whole milliseconds.
         */
        fun boundMs(index: Int, distance: Int, scale: Int?): Long {
            if (distance <= 0) return 0
            val step = step(index, scale)
            val passes = (distance + step - 1) / step
            return (passes * MS_PER_S + PASS_HZ - 1) / PASS_HZ
        }

        private const val MS_PER_S = 1000L
    }

    /** The default of one gain, or null if the key names no gain. */
    fun default(profile: Int, index: Int): Int? = when (profile) {
        CONTROL_GAIN_A -> DEFAULT_A.getOrNull(index)
        CONTROL_GAIN_B -> DEFAULT_B.getOrNull(index)
        else -> null
    }
}
