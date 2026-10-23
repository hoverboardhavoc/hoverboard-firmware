package com.hoverboard.remote.model

/**
 * The boards one session can drive (`specs/rider-ui.md` section 2): the [MASTER] the phone's BLE link
 * attaches to, and the [SLAVE] reached through it, whose address the session's discovery learned.
 * Which address each one has is a session fact the transport holds; this names the role only.
 */
enum class Node { MASTER, SLAVE }

/**
 * What the joystick's demand is turned into, per board (`specs/rider-ui.md` 3.2, the drive-mode
 * selector). App-local: no store field, and reset with the session.
 *
 * [nodes] is the mode's own answer to which boards it addresses, so neither the frame below nor the
 * screen's chips restate it as a list of mode names. Addressing a board is not the same as driving
 * it: [RIGID] names both boards because both hold their own arm level, and drives one.
 */
enum class DriveMode(val nodes: Set<Node>) {
    /** The demand to the attached master only. */
    SINGLE(setOf(Node.MASTER)),

    /**
     * The SAME demand to both boards, each addressed directly (master direct, slave two-hop). This
     * is "bind the two wheels", and in pivoting balance mode it is the mapping `specs/control.md`
     * (h) sets: the same value to both boards with steer 0.
     */
    BOUND(setOf(Node.MASTER, Node.SLAVE)),

    /**
     * A demand PER WHEEL, mixed in the app from the joystick's two axes ([DriveMix]) and sent to
     * each board at its own address. This is the mode that steers a split pair: the two boards are
     * told different things, and the difference between them is the turn.
     */
    DIFFERENTIAL(setOf(Node.MASTER, Node.SLAVE)),

    /**
     * One machine, one command: the joystick's two axes go to the rigid frame's LEADER as the lean
     * and the turn of a single `DRIVE_CMD`, and the follower is told nothing but the arm level
     * (`specs/rover-one-imu.md` section 7, "The app, and what it must stop doing").
     *
     * This is the mode that inverts [DIFFERENTIAL]. On a rigid body the two wheel demands are not
     * the app's to choose: the leader holds the machine's common torque, which is what keeps it
     * upright, and it is the only thing that may decide how much of that torque to spend on yaw
     * (section 5's saturation rule shrinks the TURN and keeps the torque, where [DriveMix] does the
     * opposite). So the app states what the rider asked for, once, and mixes nothing.
     *
     * Both boards are addressed because both run their own mode machine and need their own arm
     * level; only the leader is driven. The follower reads its wheel's number off the leader's
     * cyclic state, not off this link, and a demand sent to it would either be ignored or, on a pair
     * staged wrong, drive one wheel of a machine the other is balancing.
     */
    RIGID(setOf(Node.MASTER, Node.SLAVE)),
    ;

    /**
     * Whether this mode needs a slave the session actually found. A mode that addresses one is
     * refused until discovery has one ([com.hoverboard.remote.MainViewModel.setDriveMode]) and its
     * chip is not offered.
     */
    val needsSlave: Boolean get() = Node.SLAVE in nodes

    /**
     * Whether the mode names a board by its ROLE rather than by its address, so the app must be
     * able to SEE that role before the mode can be chosen. Only [RIGID] does: which board leads is
     * decided by which one has a working IMU, not by configuration, and the boards publish the
     * answer in their own cyclic state ([leaderNode]).
     */
    val needsLeader: Boolean
        get() = when (this) {
            SINGLE, BOUND, DIFFERENTIAL -> false
            RIGID -> true
        }

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
 * One tick of what each board is told: a [RiderCommand] per [Node], and which of them are being
 * DRIVEN.
 *
 * [driven] is not always [commands]`.keys`. A board in [commands] but outside [driven] is told its
 * arm level and nothing else: the `INPUTS` mirror reaches it and no `DRIVE_CMD` does, which is what
 * the rigid frame's follower gets. That is a real distinction on the wire and not a zero demand
 * dressed up as one, because a zero demand is still a demand: it keeps a board that was staged into
 * the wrong frame hearing a controller it should not be taking orders from, and it spends a quarter
 * of the module's metered bandwidth saying nothing.
 *
 * The rule that must survive any edit of 3.2 holds here by construction, and the `init` below is
 * it: the app never sends a nonzero `steer` while also deciding what each wheel gets, or the
 * correction is applied twice, once by the app and once by the firmware's own mixer. A frame either
 * drives ONE board, which may then be told a turn and split it itself, or it drives two and has
 * already spent the turn on the difference between them.
 */
data class DriveFrame(val commands: Map<Node, RiderCommand>, val driven: Set<Node> = commands.keys) {
    init {
        require(Node.MASTER in commands) { "every frame tells the attached master something" }
        require(commands.keys.containsAll(driven)) { "a board can only be driven if it is addressed" }
        require(driven.size <= 1 || commands.values.all { it.steer == 0 }) {
            "a frame that drives two boards has spent the turn on the pair and states none"
        }
    }

    /** The master's command: the board the app's telemetry comes from. */
    val master: RiderCommand get() = commands.getValue(Node.MASTER)

    companion object {
        /** The all-stop frame: the master disarmed, nothing else addressed. */
        val DISARMED = DriveFrame(mapOf(Node.MASTER to RiderCommand.DISARMED))

        /**
         * [command] to the boards [mode] names, with [steer] spent the way the mode spends it.
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
        fun of(
            mode: DriveMode,
            command: RiderCommand,
            steer: Int = 0,
            leader: Node? = null,
        ): DriveFrame = when (mode) {
            DriveMode.SINGLE, DriveMode.BOUND -> DriveFrame(mode.nodes.associateWith { command })
            DriveMode.DIFFERENTIAL -> {
                val wheels = DriveMix.of(command.demand, steer)
                // The rover's build fixes the sides: the master drives the right wall and the
                // slave the left (3.2). The arm level is the command's and goes to both.
                DriveFrame(
                    mode.nodes.associateWith { node ->
                        command.withDemand(if (node == Node.MASTER) wheels.right else wheels.left)
                    },
                )
            }
            DriveMode.RIGID -> DriveFrame(
                commands = mode.nodes.associateWith { node ->
                    if (node == leader) command.withTurn(steer) else command.withDemand(0)
                },
                driven = setOfNotNull(leader),
            )
        }
    }
}
