package com.hoverboardhavoc.remote.model

/**
 * One wheel: motor [motor] of the motor controller at [node] (`specs/twelve-fet.md` section 6.2).
 * A one-motor board is one wheel, motor 0; the 12-FET is two wheels on one node. Each wheel gets its
 * own `DRIVE_CMD`, with [motor] on the wire.
 */
data class Wheel(val node: Int, val motor: Int) : Comparable<Wheel> {
    override fun compareTo(other: Wheel): Int = compareValuesBy(this, other, Wheel::node, Wheel::motor)
}

/**
 * How the joystick's demand is turned into what each wheel is told
 * (`specs/remote-ui.md` 3.2). DERIVED, never stored (`specs/machine-settings.md` section 7): RIGID
 * is `machine.mode` balance with `machine.frame` rigid; SINGLE is a machine of one wheel;
 * with two or more and not rigid, BOUND or DIFFERENTIAL by whether the pad's turn axis is used,
 * which is an app setting on the Ride screen ([com.hoverboardhavoc.remote.ble.LinkSettings.turnAxis]).
 * The mode the app drives in is [DriveInForce], which derives it.
 */
enum class DriveMode {
    /** The demand to the one wheel of a one-wheel machine. */
    SINGLE,

    /**
     * The SAME demand to both wheels, each addressed directly. This is "bind the two
     * wheels", and in pivoting balance mode it is the mapping `specs/control.md` (h) sets: the same
     * value to both boards with steer 0.
     */
    BOUND,

    /**
     * A demand PER WHEEL, mixed in the app from the joystick's two axes ([DriveMix]) and sent to
     * each wheel at its node and motor. Which wheel is the right one is its motor's own `side`, so
     * the mode needs exactly one wheel to say it is.
     */
    DIFFERENTIAL,

    /**
     * One machine, one command: the joystick's two axes go to the rigid frame's LEADER as the lean
     * and the turn of a single `DRIVE_CMD`, and the follower is told nothing but the arm level
     * (`specs/rover-one-imu.md` section 7, "The app, and what it must stop doing").
     *
     * This is the mode that inverts [DIFFERENTIAL]. On a rigid body the two wheel demands are not
     * the app's to choose: the leader holds the machine's common torque, which is what keeps it
     * upright, and it is the only thing that may decide how much of that torque to spend on yaw. So
     * the app states what the rider asked for, once, and mixes nothing.
     *
     * Both boards are addressed because both run their own mode machine and need their own arm
     * level; only the leader is driven.
     */
    RIGID,
    ;

    /**
     * Whether the mode drives every wheel found, and so needs exactly two of them (RIGID: two motor
     * controllers).
     */
    val drivesBoth: Boolean get() = this != SINGLE

    /**
     * Whether the rider's control has a turn axis in this mode, i.e. whether the screen puts the
     * two-axis stick under the thumb instead of the throttle pad. The two that steer do; the two
     * that send one value per board have nowhere to spend a turn.
     */
    val hasTurnAxis: Boolean
        get() = when (this) {
            SINGLE, BOUND -> false
            DIFFERENTIAL, RIGID -> true
        }

}

/**
 * Which wheels a drive addresses and how (`specs/remote-ui.md` section 2, "The motor controllers
 * the drive addresses"; `specs/twelve-fet.md` section 6.2). Built by [DriveInForce] from what the
 * session has read, and the only thing [DriveFrame.of] takes: a frame never decides for itself
 * which wheels a mode reaches.
 *
 * @param wheels every wheel the drive addresses. Never a bridge's. Empty for a drive the app
 *   refuses ([DriveInForce.refusal]), which still names its mode so the screen draws that mode's
 *   pad.
 * @param right the wheel whose `control.side` is right, for [DriveMode.DIFFERENTIAL] only.
 * @param leader the board stored as the rigid frame's leader, for [DriveMode.RIGID] only
 *   (`specs/rigid-leader.md` section 7).
 */
data class DriveTargets(
    val mode: DriveMode,
    val wheels: List<Wheel>,
    val right: Wheel? = null,
    val leader: Int? = null,
) {
    /**
     * Every motor controller the drive addresses, each once, in wheel order: every board the arm
     * control arms, because arming is per board (each runs one mode machine for its motors).
     */
    val boards: List<Int> get() = wheels.map { it.node }.distinct()

    init {
        require(mode.drivesBoth || wheels.size <= 1) { "SINGLE addresses one wheel" }
        require(
            mode != DriveMode.DIFFERENTIAL || wheels.isEmpty() ||
                (right != null && right in wheels && wheels.size == 2),
        ) {
            "DIFFERENTIAL needs its right wheel among its two wheels"
        }
        require(mode != DriveMode.RIGID || wheels.isEmpty() || (leader != null && leader in boards)) {
            "RIGID needs its leader among its boards"
        }
    }


    companion object {
        /** Nothing addressed: no motor controller known yet. */
        val NONE = DriveTargets(DriveMode.SINGLE, emptyList())
    }
}

/**
 * One tick of what each wheel is told: a [RiderCommand] per [Wheel], and which of them are being
 * DRIVEN.
 *
 * [driven] is not always [commands]`.keys`. A wheel in [commands] but outside [driven] is told its
 * board's arm level and nothing else: the `INPUTS` mirror reaches the board and no `DRIVE_CMD`
 * reaches the wheel, which is what the rigid frame's follower gets. That is a real distinction on
 * the wire and not a zero demand dressed up as one, because a zero demand is still a demand.
 *
 * The arm level is per BOARD (`INPUTS` has no motor): every wheel of one board carries the same
 * levels, which the `init` holds, so the one `INPUTS` a board is sent states them all.
 *
 * The rule that must survive any edit of 3.2 holds here by construction, and the `init` below is
 * it: the app never sends a nonzero `steer` while also deciding what each wheel gets, or the
 * correction is applied twice, once by the app and once by the firmware's own mixer. A frame either
 * drives ONE wheel, whose board may then be told a turn and split it itself, or it drives two and
 * has already spent the turn on the difference between them.
 */
data class DriveFrame(val commands: Map<Wheel, RiderCommand>, val driven: Set<Wheel> = commands.keys) {
    init {
        require(commands.keys.containsAll(driven)) { "a wheel can only be driven if it is addressed" }
        require(driven.size <= 1 || commands.values.all { it.steer == 0 }) {
            "a frame that drives two wheels has spent the turn on the pair and states none"
        }
        require(commands.entries.groupBy({ it.key.node }, { it.value.inputs }).values.all { it.toSet().size == 1 }) {
            "every wheel of one board carries that board's one arm level"
        }
    }

    /** The boards addressed, each once, in address order: each is sent one `INPUTS`. */
    val boards: List<Int> get() = commands.keys.map { it.node }.distinct().sorted()

    /** The board at [node]'s wheels, in motor order, with what each is told. */
    fun wheelsOf(node: Int): List<Pair<Wheel, RiderCommand>> =
        commands.entries.filter { it.key.node == node }.sortedBy { it.key }.map { it.key to it.value }

    companion object {
        /** The all-stop frame: nobody addressed. The pump still owes a disarm to a board it armed. */
        val DISARMED = DriveFrame(emptyMap())

        /**
         * [command] to the wheels [targets] names, with [steer] spent the way the mode spends it.
         *
         * [steer] is the joystick's turn axis. SINGLE and BOUND send one value to their wheels, so
         * a turn has nowhere to go in them and is dropped here rather than left to a caller to
         * remember to zero. DIFFERENTIAL spends it on the difference between two per-wheel demands.
         * RIGID puts it on the wire, on the leader's command alone: motor 0 of
         * [DriveTargets.leader], the board stored as the rigid frame's leader.
         */
        fun of(targets: DriveTargets, command: RiderCommand, steer: Int = 0): DriveFrame =
            when (targets.mode) {
                DriveMode.SINGLE, DriveMode.BOUND -> DriveFrame(targets.wheels.associateWith { command })
                DriveMode.DIFFERENTIAL -> {
                    val pair = DriveMix.of(command.demand, steer)
                    // The arm level is the command's and goes to both; each wheel's demand goes to
                    // the wheel whose own motor's `side` says it is that wheel.
                    DriveFrame(
                        targets.wheels.associateWith { wheel ->
                            command.withDemand(if (wheel == targets.right) pair.right else pair.left)
                        },
                    )
                }
                DriveMode.RIGID -> {
                    val lead = targets.wheels.filter { it.node == targets.leader }.minOrNull()
                    DriveFrame(
                        commands = targets.wheels.associateWith { wheel ->
                            if (wheel == lead) command.withTurn(steer) else command.withDemand(0)
                        },
                        driven = setOfNotNull(lead),
                    )
                }
            }
    }
}
