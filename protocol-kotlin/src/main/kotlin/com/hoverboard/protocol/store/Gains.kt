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

    /** The default of one gain, or null if the key names no gain. */
    fun default(profile: Int, index: Int): Int? = when (profile) {
        CONTROL_GAIN_A -> DEFAULT_A.getOrNull(index)
        CONTROL_GAIN_B -> DEFAULT_B.getOrNull(index)
        else -> null
    }
}
