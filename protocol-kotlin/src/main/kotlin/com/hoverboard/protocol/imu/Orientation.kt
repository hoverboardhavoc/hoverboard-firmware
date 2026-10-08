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
 * the remaining one. Under the default roles the frame is a diagonal map, so the sign field alone
 * can express only the four proper diagonal rotations (identity and the three 180 degree turns). A
 * mount that needs a permutation (a board standing on edge) needs the roles as well, so a client
 * judges a sign map together with the roles the board holds ([check]), never under assumed default
 * roles.
 *
 * The two fields together express the 24 proper rotations, and a user names one as a [Pose] of the
 * board in its machine (`specs/rider-ui.md` 3.4, the orientation picker): [frameOf] composes a pose
 * into the two fields and [poseOf] reads two stored fields back as a pose.
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

    /**
     * The face of the board that points UP in the machine, named relative to the stock flat mount
     * (the one frame every reader can place). [up] is that face's outward normal in BOARD axes: the
     * board frame is the body frame of the stock flat mount (`X` stock-forward, `Y` stock-left, `Z`
     * up out of the component side), so under [COMPONENT_UP] the body frame is the board frame.
     */
    enum class Face(val up: List<Int>) {
        /** The stock mount. */
        COMPONENT_UP(listOf(0, 0, 1)),
        COMPONENT_DOWN(listOf(0, 0, -1)),
        STOCK_FORWARD_EDGE_DOWN(listOf(-1, 0, 0)),
        STOCK_REAR_EDGE_DOWN(listOf(1, 0, 0)),
        STOCK_LEFT_EDGE_DOWN(listOf(0, -1, 0)),
        STOCK_RIGHT_EDGE_DOWN(listOf(0, 1, 0)),
    }

    /**
     * What part of the board points FORWARD in the machine (the direction it drives under a positive
     * pitch command), as its outward normal [forward] in board axes. Four of the six are possible for
     * any one [Face], the four perpendicular to its normal ([headingsFor]): for a face whose normal is
     * board `Z` they are the four stock edges, and for an edge-down face they are the two remaining
     * edges and the two sides, since the board's thin axis then lies horizontal.
     */
    enum class Heading(val forward: List<Int>) {
        STOCK_FORWARD(listOf(1, 0, 0)),
        STOCK_REAR(listOf(-1, 0, 0)),
        STOCK_LEFT(listOf(0, 1, 0)),
        STOCK_RIGHT(listOf(0, -1, 0)),
        COMPONENT_SIDE(listOf(0, 0, 1)),
        BACK_SIDE(listOf(0, 0, -1)),
    }

    /** How the board sits in its machine: which face points up and what points forward. */
    data class Pose(val face: Face, val heading: Heading) {
        init {
            require(dot(face.up, heading.forward) == 0) { "$heading cannot point forward with $face" }
        }
    }

    /** The two field values a pose stages: `IMU_AXIS_ROLE` ([roles]) and `IMU_AXIS_SIGN` ([signs]). */
    data class Frame(val roles: List<Int>, val signs: List<Int>)

    /** The four headings possible for [face], in [Heading] order. */
    fun headingsFor(face: Face): List<Heading> = Heading.entries.filter { dot(face.up, it.forward) == 0 }

    /** All 24 poses: every face with each of its four headings. */
    val POSES: List<Pose> = Face.entries.flatMap { f -> headingsFor(f).map { Pose(f, it) } }

    /** The stock flat mount, the pose an unset board runs ([DEFAULT_ROLES] and [REFERENCE]). */
    val STOCK_POSE = Pose(Face.COMPONENT_UP, Heading.STOCK_FORWARD)

    /**
     * Board-to-body for [pose], as rows: `body[i] = sum_j R[i][j] * board[j]`, with body `X` forward,
     * `Y` left, `Z` up. Row 0 is the board direction that points forward, row 2 the one that points
     * up, and row 1 their cross product `up x forward` (body `Y = Z x X`), so the matrix is a proper
     * rotation for every pose.
     */
    fun boardToBody(pose: Pose): List<List<Int>> {
        val up = pose.face.up
        val forward = pose.heading.forward
        return listOf(forward, cross(up, forward), up)
    }

    /**
     * The two fields that make the board run [pose]. Chip-to-board is the compiled reference under
     * the compiled roles, `diag(REFERENCE)` per vector (the standard board family's constant: under
     * the stock mount an unset board runs exactly that frame). The composition `M = boardToBody(pose)
     * * diag(ref)` is a signed permutation, and `body = P * S * chip` (`specs/imu.md`, the
     * `IMU_AXIS_ROLE` entry) factors it: UP is the chip axis in `M`'s body-Z row, PITCH_RATE the one
     * in its body-Y row, and each chip axis's sign is the nonzero entry of its column. The accel and
     * the gyro triples are factored alike, each from its own half of [REFERENCE].
     */
    fun frameOf(pose: Pose): Frame {
        val r = boardToBody(pose)
        fun axisOf(row: List<Int>): Int = row.indexOfFirst { it != 0 }
        val roles = listOf(axisOf(r[2]) + 1, axisOf(r[1]) + 1)
        val signs = (0 until INDICES).map { i ->
            val chip = i % PER_VECTOR
            r.sumOf { it[chip] } * REFERENCE[i]
        }
        return Frame(roles, signs)
    }

    /**
     * The pose two staged fields make the board run, after the unset rule on both ([effectiveRoles],
     * [effective]), so unset fields read as [STOCK_POSE]; or null for a frame that is none of the 24
     * (a mirrored one, or a proper rotation with different accel and gyro triples).
     */
    fun poseOf(stagedRoles: List<Int>, stagedSigns: List<Int>): Pose? {
        val frame = Frame(effectiveRoles(stagedRoles), effective(stagedSigns))
        return POSES.firstOrNull { frameOf(it) == frame }
    }

    private fun dot(a: List<Int>, b: List<Int>): Int = a.zip(b).sumOf { (x, y) -> x * y }

    private fun cross(a: List<Int>, b: List<Int>): List<Int> = listOf(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )
}
