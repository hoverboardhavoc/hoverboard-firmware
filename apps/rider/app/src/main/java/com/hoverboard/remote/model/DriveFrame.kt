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
 * screen's chips restate it as a list of mode names.
 */
enum class DriveMode(val nodes: Set<Node>) {
    /** The demand to the attached master only. */
    SINGLE(setOf(Node.MASTER)),

    /**
     * The SAME demand to both boards, each addressed directly (master direct, slave two-hop). This
     * is "bind the two wheels", and in balance mode it is the mapping `specs/control.md` (h) sets:
     * the same value to both boards with steer 0, until the one-loop-or-two measurement.
     */
    BOUND(setOf(Node.MASTER, Node.SLAVE)),

    /**
     * A demand PER WHEEL, mixed in the app from the joystick's two axes ([DriveMix]) and sent to
     * each board at its own address. This is the mode that steers a split pair: the two boards are
     * told different things, and the difference between them is the turn.
     */
    DIFFERENTIAL(setOf(Node.MASTER, Node.SLAVE));

    /**
     * Whether this mode needs a slave the session actually found. A mode that addresses one is
     * refused until discovery has one ([com.hoverboard.remote.MainViewModel.setDriveMode]) and its
     * chip is not offered.
     */
    val needsSlave: Boolean get() = Node.SLAVE in nodes
}

/**
 * One tick of what each board is told: a [RiderCommand] per [Node].
 *
 * The rule that must survive any edit of 3.2 holds here by construction: the app never sends a
 * nonzero `steer` while also deciding what each wheel gets, or the firmware's mixer would apply the
 * correction a second time. Every [RiderCommand] carries `steer = 0` ([RiderCommand.drive]), and the
 * turn the rider asked for is spent HERE, on the difference between two boards' values, rather than
 * put on the wire as a word for the firmware to mix again.
 */
data class DriveFrame(val commands: Map<Node, RiderCommand>) {
    init {
        require(Node.MASTER in commands) { "every frame tells the attached master something" }
    }

    /** The master's command: the board the app's telemetry comes from. */
    val master: RiderCommand get() = commands.getValue(Node.MASTER)

    companion object {
        /** The all-stop frame: the master disarmed, nothing else addressed. */
        val DISARMED = DriveFrame(mapOf(Node.MASTER to RiderCommand.DISARMED))

        /**
         * [command] to the boards [mode] names, with [steer] mixed in where the mode asks for it.
         *
         * [steer] is the joystick's turn axis, read only by DIFFERENTIAL: the other two modes send
         * one value to their boards, so a turn has nowhere to go in them and is dropped here rather
         * than left to a caller to remember to zero.
         */
        fun of(mode: DriveMode, command: RiderCommand, steer: Int = 0): DriveFrame = DriveFrame(
            when (mode) {
                DriveMode.SINGLE, DriveMode.BOUND -> mode.nodes.associateWith { command }
                DriveMode.DIFFERENTIAL -> {
                    val wheels = DriveMix.of(command.demand, steer)
                    // The rover's build fixes the sides: the master drives the right wall and the
                    // slave the left (3.2). The arm level is the command's and goes to both.
                    mode.nodes.associateWith { node ->
                        command.withDemand(if (node == Node.MASTER) wheels.right else wheels.left)
                    }
                }
            },
        )
    }
}
