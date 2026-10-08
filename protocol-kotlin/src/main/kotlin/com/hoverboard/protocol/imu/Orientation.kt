package com.hoverboard.protocol.imu

/**
 * The IMU orientation as the `IMU_AXIS_SIGN` (0x65) and `IMU_AXIS_ROLE` (0x68) fields can hold it,
 * and the rule that decides which frames are legal: a mirror of `imu::Config::default()`,
 * `imu::DEFAULT_ROLES`, `imu::Config::staged` (the unset rule and its refusal order),
 * `imu::Config::frame_is_rotation` and `imu::Config::triple_is_rotation` in `crates/imu/src/lib.rs`,
 * which `RustSourceDriftTest` pins.
 *
 * The sign field is six signs, `[ax, ay, az, gx, gy, gz]`, per CHIP axis. The roles field is
 * `[UP, PITCH_RATE]`, the chip axis (`1 = X`, `2 = Y`, `3 = Z`) playing each body role, with FORWARD
 * the remaining one. Under the default roles the frame is a diagonal map, so the frames the sign
 * field alone can express are exactly the four proper diagonal rotations in [ROTATIONS] (identity
 * and the three 180 degree turns). A mount that needs a permutation (a board standing on edge)
 * needs the roles as well, so a client judges a sign map together with the roles the board holds
 * ([check]), never under assumed default roles.
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
     * The compiled axis roles `[UP, PITCH_RATE]` = `[Z, Y]` (`imu::DEFAULT_ROLES`): the stock flat
     * mount, under which body order is chip order. What an UNSET (0) role falls back to.
     */
    val DEFAULT_ROLES: List<Int> = listOf(3, 2)

    /**
     * The body-order permutation the resolved [roles] select (`imu::body_order`): element `i` is the
     * 0-based CHIP axis that becomes body axis `i` (FORWARD, PITCH_RATE, UP), or null when a role is
     * outside `1..3` or the two name the same axis.
     */
    private fun bodyOrder(roles: List<Int>): List<Int>? {
        if (roles.size != 2) return null
        val (up, pitch) = roles
        if (up !in 1..3 || pitch !in 1..3 || up == pitch) return null
        return listOf(3 - (up - 1) - (pitch - 1), pitch - 1, up - 1)
    }

    /**
     * Whether a sign triple (chip axes) read through the resolved [roles] is a proper rotation
     * (determinant +1): both roles in `1..3` and distinct, every sign +-1, and the sign product equal
     * to the permutation's parity (+1 when UP's chip axis is the cyclic successor of PITCH_RATE's,
     * as for the identity and the two 3-cycles; -1 for the three transpositions). The mirror of
     * `imu::Config::frame_is_rotation`.
     */
    fun frameIsRotation(roles: List<Int>, triple: List<Int>): Boolean {
        val order = bodyOrder(roles) ?: return false
        val parity = if (order[2] == (order[1] + 1) % 3) 1 else -1
        return triple.size == PER_VECTOR && triple.all { it == 1 || it == -1 } &&
            triple[0] * triple[1] * triple[2] == parity
    }

    /**
     * Whether a three-axis sign triple is a proper rotation under [DEFAULT_ROLES]: every entry is
     * +-1 and their product is +1. The mirror of `imu::Config::triple_is_rotation`.
     */
    fun tripleIsRotation(triple: List<Int>): Boolean = frameIsRotation(DEFAULT_ROLES, triple)

    /**
     * The map a board RUNS for a staged six-index value: each staged 0 (unset) falls back to that
     * index of [REFERENCE], every other value is taken as staged. The mirror of `imu::Config::staged`.
     */
    fun effective(staged: List<Int>): List<Int> {
        require(staged.size == INDICES) { "a sign map has $INDICES indices, got ${staged.size}" }
        return staged.mapIndexed { i, s -> if (s != 0) s else REFERENCE[i] }
    }

    /**
     * The roles a board RUNS for a staged `[UP, PITCH_RATE]`: each staged 0 (unset) falls back to
     * that index of [DEFAULT_ROLES]. The roles half of `imu::Config::staged`'s unset rule.
     */
    fun effectiveRoles(staged: List<Int>): List<Int> {
        require(staged.size == 2) { "the roles are [UP, PITCH_RATE], got ${staged.size}" }
        return staged.mapIndexed { i, r -> if (r != 0) r else DEFAULT_ROLES[i] }
    }

    /** Why the board refuses a staged frame at boot: `imu::FrameError`, one entry per variant. */
    enum class FrameError {
        /** A role is outside `1..3`, or the two roles name the same chip axis. */
        ROLES,

        /** The accel triple read through the roles is a reflection, or a sign is not +-1. */
        ACCEL,

        /** The gyro triple read through the roles is a reflection, or a sign is not +-1. */
        GYRO,
    }

    /**
     * The refusal `imu::Config::staged` returns for a staged sign map and staged roles, or null
     * when the board accepts the frame: the unset rule on both fields, then the roles, the accel
     * triple and the gyro triple checked in that order.
     */
    fun frameError(stagedSigns: List<Int>, stagedRoles: List<Int>): FrameError? {
        val signs = effective(stagedSigns)
        val roles = effectiveRoles(stagedRoles)
        return when {
            bodyOrder(roles) == null -> FrameError.ROLES
            !frameIsRotation(roles, signs.subList(0, PER_VECTOR)) -> FrameError.ACCEL
            !frameIsRotation(roles, signs.subList(PER_VECTOR, INDICES)) -> FrameError.GYRO
            else -> null
        }
    }

    /** Why a client refuses a staged frame (signs and roles) before writing it. */
    enum class Refusal {
        /** A sign other than -1, 0 (unset) or +1. */
        NOT_A_SIGN,

        /** A role outside `0..3`, or the two roles (after the unset rule) name the same chip axis. */
        ROLES,

        /** The accel triple, read through the roles, is a reflection (determinant -1). */
        ACCEL_MIRRORED,

        /** The gyro triple, read through the roles, is a reflection (determinant -1). */
        GYRO_MIRRORED,
    }

    /**
     * Check a staged six-index sign map together with the staged `[UP, PITCH_RATE]` roles the way the
     * board will run them, or null when the frame is legal: every sign is -1, 0 or +1, and
     * [frameError] accepts the pair. The client refuses an out-of-range sign itself
     * ([Refusal.NOT_A_SIGN]) where the board folds it into the triple's refusal, so the notice can
     * say which.
     */
    fun check(stagedSigns: List<Int>, stagedRoles: List<Int>): Refusal? {
        if (stagedSigns.any { it !in -1..1 }) return Refusal.NOT_A_SIGN
        return when (frameError(stagedSigns, stagedRoles)) {
            null -> null
            FrameError.ROLES -> Refusal.ROLES
            FrameError.ACCEL -> Refusal.ACCEL_MIRRORED
            FrameError.GYRO -> Refusal.GYRO_MIRRORED
        }
    }
}
