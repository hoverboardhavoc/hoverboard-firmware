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
 * DIFFERENTIAL (the per-wheel mix `left = v + s/2`, `right = v - s/2`) is not here: 3.2 holds it
 * until the two-hop silicon gate, and the joystick's turn axis it would consume waits with it.
 */
enum class DriveMode {
    /** The demand to the attached master only. */
    SINGLE,

    /**
     * The SAME demand to both boards, each addressed directly (master direct, slave two-hop). This
     * is "bind the two wheels", and in balance mode it is the mapping `specs/control.md` (h) sets:
     * the same value to both boards with steer 0, until the one-loop-or-two measurement.
     */
    BOUND,
}

/**
 * One tick of what each board is told: a [RiderCommand] per [Node].
 *
 * The rule that must survive any edit of 3.2 holds here by construction: the app never sends a
 * nonzero `steer` while also deciding what each wheel gets, or the firmware's mixer would apply the
 * correction a second time. Every [RiderCommand] carries `steer = 0` ([RiderCommand.drive]), and the
 * only per-board decision a frame makes today is WHICH boards get the one demand.
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

        /** [command] to the boards [mode] names. */
        fun of(mode: DriveMode, command: RiderCommand): DriveFrame = DriveFrame(
            when (mode) {
                DriveMode.SINGLE -> mapOf(Node.MASTER to command)
                DriveMode.BOUND -> mapOf(Node.MASTER to command, Node.SLAVE to command)
            },
        )
    }
}
