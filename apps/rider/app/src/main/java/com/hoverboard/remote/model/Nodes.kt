package com.hoverboard.remote.model

import com.hoverboard.protocol.store.Fields

/**
 * One node the session's walk found (`specs/rider-ui.md` section 2): its address, its `node.class`
 * and its `device.name`, the last two null until the read of each has answered.
 *
 * Nodes are named by name and address, never by role or by which one the phone attached to: the
 * node on the phone's own link is a bridge when it came in through an ESP and a motor controller
 * when it came in through a board's own BLE module (`specs/l3.md`, "Node class").
 */
data class NodeInfo(val address: Int, val nodeClass: Int?, val name: String?) {
    /** A hoverboard mainboard: it drives a wheel, reports telemetry and is armed. */
    val isMotorController: Boolean get() = nodeClass == Fields.NodeClass.MOTOR_CONTROLLER

    /** An ESP bridge: forwards between its BLE link and a board, and drives nothing. */
    val isBridge: Boolean get() = nodeClass == Fields.NodeClass.BRIDGE
}

/**
 * Why the app drives in SINGLE although the machine stores another mode, or cannot trust what it
 * stores (`specs/rider-ui.md` 3.2): the Ride status strip says which.
 */
enum class DriveFallback {
    /** A node's class, or a motor controller's stored mode, has not been read this session. */
    UNREAD,

    /** The motor controllers store different modes. */
    DISAGREE,

    /** The stored byte is not a mode this app knows. */
    UNKNOWN_MODE,

    /** The stored mode drives two motor controllers and the walk found another number of them. */
    NOT_TWO,

    /** RIGID, and not exactly one motor controller reports itself the rigid frame's leader. */
    NO_LEADER,

    /** DIFFERENTIAL, and not exactly one motor controller is set as the right wheel. */
    NO_RIGHT_WHEEL,
}

/**
 * The drive mode in force and the motor controllers it addresses (`specs/rider-ui.md` section 2 and
 * 3.2): the stored mode when every copy agrees and its precondition holds on the connected machine,
 * else SINGLE with [fallback] saying why.
 *
 * @param stored the mode the machine stores, when every copy agrees on one this app knows, else null.
 */
data class DriveInForce(val targets: DriveTargets, val stored: DriveMode?, val fallback: DriveFallback?) {
    val mode: DriveMode get() = targets.mode

    companion object {
        /** No node attached: nothing to drive and nothing to say. */
        val NONE = DriveInForce(DriveTargets.NONE, stored = null, fallback = null)

        /**
         * The mode in force on the machine [nodes] describes.
         *
         * @param storedMode each motor controller's stored `machine.drive_mode` byte, null while unread.
         * @param side each motor controller's stored `control.side` byte, null while unread.
         * @param leaders the addresses whose own telemetry claims the rigid frame's lead.
         * @param selected the motor controller SINGLE drives: the Tune/Setup selector's.
         */
        fun of(
            nodes: List<NodeInfo>,
            storedMode: (Int) -> Int?,
            side: (Int) -> Int?,
            leaders: Set<Int>,
            selected: Int?,
        ): DriveInForce {
            val mcs = nodes.filter { it.isMotorController }.map { it.address }
            val single = DriveTargets(DriveMode.SINGLE, listOfNotNull(selected?.takeIf { it in mcs }))
            val (mode, unread) = stored(nodes, mcs, storedMode)
            if (mode == null) return DriveInForce(single, stored = null, fallback = unread)
            val why = precondition(mode, mcs, side, leaders)
            return when {
                why != null -> DriveInForce(single, mode, why)
                mode == DriveMode.SINGLE -> DriveInForce(single, mode, fallback = null)
                else -> DriveInForce(DriveTargets(mode, mcs, rightWheel(mode, mcs, side)), mode, fallback = null)
            }
        }

        /**
         * The mode every motor controller stores, or null with the reason it cannot be trusted. Null
         * with no reason on a machine with no motor controller, which stores nothing to read.
         */
        private fun stored(
            nodes: List<NodeInfo>,
            mcs: List<Int>,
            storedMode: (Int) -> Int?,
        ): Pair<DriveMode?, DriveFallback?> {
            val copies = mcs.map(storedMode)
            val distinct = copies.toSet()
            val why = when {
                nodes.any { it.nodeClass == null } || null in copies -> DriveFallback.UNREAD
                mcs.isEmpty() -> null
                distinct.size > 1 -> DriveFallback.DISAGREE
                else -> DriveFallback.UNKNOWN_MODE.takeIf { DriveMode.of(checkNotNull(distinct.single())) == null }
            }
            val mode = if (why == null && mcs.isNotEmpty()) DriveMode.of(checkNotNull(distinct.single())) else null
            return mode to why
        }

        /** Why [mode] cannot run on this machine, or null when it can. */
        private fun precondition(
            mode: DriveMode,
            mcs: List<Int>,
            side: (Int) -> Int?,
            leaders: Set<Int>,
        ): DriveFallback? =
            when {
                !mode.drivesBoth -> null
                mcs.size != 2 -> DriveFallback.NOT_TWO
                mode == DriveMode.RIGID && leaders.count { it in mcs } != 1 -> DriveFallback.NO_LEADER
                mode == DriveMode.DIFFERENTIAL && rightWheel(mode, mcs, side) == null -> DriveFallback.NO_RIGHT_WHEEL
                else -> null
            }

        /**
         * The board whose own `control.side` says it is the right wheel, for DIFFERENTIAL: exactly one
         * such board with every copy read, else null.
         */
        private fun rightWheel(mode: DriveMode, mcs: List<Int>, side: (Int) -> Int?): Int? {
            if (mode != DriveMode.DIFFERENTIAL || mcs.any { side(it) == null }) return null
            return mcs.filter { side(it) == Fields.ControlSide.RIGHT }.singleOrNull()
        }
    }
}
