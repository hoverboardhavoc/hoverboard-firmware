package com.hoverboard.remote.model

import com.hoverboard.protocol.store.Fields

/**
 * How the joystick's demand is turned into what each motor controller is told
 * (`specs/remote-ui.md` 3.2). A setting of the MACHINE, stored as `machine.drive_mode`
 * ([Fields.MACHINE_DRIVE_MODE]) on every motor controller, and never a choice made on the phone:
 * the mode the app drives in is [DriveInForce], which reads the stored copies.
 *
 * [byte] is the stored value, [Fields.MachineDriveMode]'s vocabulary.
 */
enum class DriveMode(val byte: Int) {
    /** The demand to the motor controller selected in the Tune/Setup selector, and nothing else. */
    SINGLE(Fields.MachineDriveMode.SINGLE),

    /**
     * The SAME demand to both motor controllers, each addressed directly. This is "bind the two
     * wheels", and in pivoting balance mode it is the mapping `specs/control.md` (h) sets: the same
     * value to both boards with steer 0.
     */
    BOUND(Fields.MachineDriveMode.BOUND),

    /**
     * A demand PER WHEEL, mixed in the app from the joystick's two axes ([DriveMix]) and sent to
     * each board at its own address. Which board is the right wheel is that board's own
     * `control.side`, so the mode needs exactly one board to say it is.
     */
    DIFFERENTIAL(Fields.MachineDriveMode.DIFFERENTIAL),

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
    RIGID(Fields.MachineDriveMode.RIGID),
    ;

    /** Whether the mode drives every motor controller found, and so needs exactly two of them. */
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

    companion object {
        /** The mode a stored byte names, or null for a byte outside the vocabulary. */
        fun of(byte: Int): DriveMode? = entries.firstOrNull { it.byte == byte }
    }
}

/**
 * Which motor controllers a drive addresses and how, by address (`specs/remote-ui.md` section 2,
 * "The motor controllers the drive addresses"). Built by [DriveInForce] from what the session has
 * read, and the only thing [DriveFrame.of] takes: a frame never decides for itself which boards a
 * mode reaches.
 *
 * @param boards every motor controller the drive addresses, which is also every board the arm
 *   control arms. Never a bridge. Empty for a drive the app refuses ([DriveInForce.refusal]),
 *   which still names its mode so the screen draws that mode's pad.
 * @param right the board that is the right wheel, for [DriveMode.DIFFERENTIAL] only.
 */
data class DriveTargets(val mode: DriveMode, val boards: List<Int>, val right: Int? = null) {
    init {
        require(mode.drivesBoth || boards.size <= 1) { "SINGLE addresses one board" }
        require(
            mode != DriveMode.DIFFERENTIAL || boards.isEmpty() ||
                (right != null && right in boards && boards.size == 2),
        ) {
            "DIFFERENTIAL needs its right wheel among its two boards"
        }
    }


    companion object {
        /** Nothing addressed: no motor controller known yet. */
        val NONE = DriveTargets(DriveMode.SINGLE, emptyList())
    }
}

/**
 * One tick of what each board is told: a [RiderCommand] per board address, and which of them are
 * being DRIVEN.
 *
 * [driven] is not always [commands]`.keys`. A board in [commands] but outside [driven] is told its
 * arm level and nothing else: the `INPUTS` mirror reaches it and no `DRIVE_CMD` does, which is what
 * the rigid frame's follower gets. That is a real distinction on the wire and not a zero demand
 * dressed up as one, because a zero demand is still a demand.
 *
 * The rule that must survive any edit of 3.2 holds here by construction, and the `init` below is
 * it: the app never sends a nonzero `steer` while also deciding what each wheel gets, or the
 * correction is applied twice, once by the app and once by the firmware's own mixer. A frame either
 * drives ONE board, which may then be told a turn and split it itself, or it drives two and has
 * already spent the turn on the difference between them.
 */
data class DriveFrame(val commands: Map<Int, RiderCommand>, val driven: Set<Int> = commands.keys) {
    init {
        require(commands.keys.containsAll(driven)) { "a board can only be driven if it is addressed" }
        require(driven.size <= 1 || commands.values.all { it.steer == 0 }) {
            "a frame that drives two boards has spent the turn on the pair and states none"
        }
    }

    companion object {
        /** The all-stop frame: nobody addressed. The pump still owes a disarm to a board it armed. */
        val DISARMED = DriveFrame(emptyMap())

        /**
         * [command] to the boards [targets] names, with [steer] spent the way the mode spends it.
         *
         * [steer] is the joystick's turn axis. SINGLE and BOUND send one value to their boards, so
         * a turn has nowhere to go in them and is dropped here rather than left to a caller to
         * remember to zero. DIFFERENTIAL spends it on the difference between two per-wheel demands.
         * RIGID puts it on the wire, on the leader's command alone.
         *
         * [leader] is which board reports itself the rigid frame's leader, and only RIGID reads it.
         * Null means the app cannot currently see one, and a rigid frame then drives NOBODY: both
         * boards still get their arm level, so a disarm always lands, and neither gets a demand the
         * app cannot say is for the right wheel.
         */
        fun of(targets: DriveTargets, command: RiderCommand, steer: Int = 0, leader: Int? = null): DriveFrame =
            when (targets.mode) {
                DriveMode.SINGLE, DriveMode.BOUND -> DriveFrame(targets.boards.associateWith { command })
                DriveMode.DIFFERENTIAL -> {
                    val wheels = DriveMix.of(command.demand, steer)
                    // The arm level is the command's and goes to both; each wheel's demand goes to
                    // the board whose own `control.side` says it is that wheel.
                    DriveFrame(
                        targets.boards.associateWith { board ->
                            command.withDemand(if (board == targets.right) wheels.right else wheels.left)
                        },
                    )
                }
                DriveMode.RIGID -> {
                    val lead = leader?.takeIf { it in targets.boards }
                    DriveFrame(
                        commands = targets.boards.associateWith { board ->
                            if (board == lead) command.withTurn(steer) else command.withDemand(0)
                        },
                        driven = setOfNotNull(lead),
                    )
                }
            }
    }
}
