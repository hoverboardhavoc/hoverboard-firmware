package com.hoverboard.protocol.imu

/**
 * The IMU orientation as the `IMU_AXIS_SIGN` field (0x65) can hold it, and the rule that decides
 * which values are legal: a mirror of `imu::Config::default().sign`, `imu::Config::staged` (the
 * unset rule) and `imu::Config::triple_is_rotation` in `crates/imu/src/lib.rs`, which
 * `RustSourceDriftTest` pins.
 *
 * The field is six signs, `[ax, ay, az, gx, gy, gz]`, a diagonal map per vector with no axis
 * permutation, so the frames it can express are exactly the four proper diagonal rotations in
 * [ROTATIONS] (identity and the three 180 degree turns), applied to the accel and the gyro triple.
 * A mount that needs a permutation (a board standing on edge) is not expressible in it.
 */
object Orientation {
    /** Number of indices in the sign map (`ax, ay, az, gx, gy, gz`). */
    const val INDICES = 6

    /** Indices per vector (the accel triple is 0..2, the gyro triple 3..5). */
    const val PER_VECTOR = 3

    /**
     * The compiled reference map (`imu::Config::default().sign`): the stock board's mount, a
     * 180 degree rotation about Y. What an UNSET (0) index falls back to; not a fleet constant.
     */
    val REFERENCE: List<Int> = listOf(-1, 1, -1, -1, 1, -1)

    /** One proper rotation the field can express, as the three signs applied to both vectors. */
    enum class Rotation(val triple: List<Int>) {
        IDENTITY(listOf(1, 1, 1)),
        HALF_TURN_X(listOf(1, -1, -1)),
        HALF_TURN_Y(listOf(-1, 1, -1)),
        HALF_TURN_Z(listOf(-1, -1, 1)),
        ;

        /** The six-index sign map that stages this rotation for both the accel and the gyro. */
        val signs: List<Int> get() = triple + triple

        companion object {
            /** The rotation a full six-index map is, or null when the two triples differ or are not one. */
            fun of(signs: List<Int>): Rotation? {
                if (signs.size != INDICES) return null
                val accel = signs.subList(0, PER_VECTOR)
                if (accel != signs.subList(PER_VECTOR, INDICES)) return null
                return entries.firstOrNull { it.triple == accel }
            }
        }
    }

    /** Every proper rotation the field can express. */
    val ROTATIONS: List<Rotation> = Rotation.entries

    /**
     * Whether a three-axis sign triple is a proper rotation (determinant +1): every entry is +-1 and
     * their product is +1. The mirror of `imu::Config::triple_is_rotation`.
     */
    fun tripleIsRotation(triple: List<Int>): Boolean =
        triple.size == PER_VECTOR && triple.all { it == 1 || it == -1 } && triple[0] * triple[1] * triple[2] == 1

    /**
     * The map a board RUNS for a staged six-index value: each staged 0 (unset) falls back to that
     * index of [REFERENCE], every other value is taken as staged. The mirror of `imu::Config::staged`.
     */
    fun effective(staged: List<Int>): List<Int> {
        require(staged.size == INDICES) { "a sign map has $INDICES indices, got ${staged.size}" }
        return staged.mapIndexed { i, s -> if (s != 0) s else REFERENCE[i] }
    }

    /** Why a staged map is refused. */
    enum class Refusal {
        /** A value other than -1, 0 (unset) or +1. */
        NOT_A_SIGN,

        /** The accel triple, after the unset rule, is a reflection (determinant -1). */
        ACCEL_MIRRORED,

        /** The gyro triple, after the unset rule, is a reflection (determinant -1). */
        GYRO_MIRRORED,
    }

    /**
     * Check a staged six-index map the way the board will run it, or null when it is legal: every
     * value is -1, 0 or +1, and both triples of the [effective] map are proper rotations.
     */
    fun check(staged: List<Int>): Refusal? {
        if (staged.any { it !in -1..1 }) return Refusal.NOT_A_SIGN
        val run = effective(staged)
        if (!tripleIsRotation(run.subList(0, PER_VECTOR))) return Refusal.ACCEL_MIRRORED
        if (!tripleIsRotation(run.subList(PER_VECTOR, INDICES))) return Refusal.GYRO_MIRRORED
        return null
    }
}
