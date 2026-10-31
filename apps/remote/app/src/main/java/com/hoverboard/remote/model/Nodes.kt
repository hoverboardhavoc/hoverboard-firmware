package com.hoverboard.remote.model

import com.hoverboard.protocol.store.Fields

/**
 * One node the session's walk found (`specs/remote-ui.md` section 2): its address, its `node.class`
 * and its `device.name`, the last two null until the read of each has answered.
 *
 * Nodes are named by name and address, never by role or by which one the phone attached to: the
 * node on the phone's own link is a bridge when it came in through a bridge and a motor controller
 * when it came in through a board's own BLE module (`specs/l3.md`, "Node class").
 */
data class NodeInfo(val address: Int, val nodeClass: Int?, val name: String?) {
    /** A hoverboard mainboard: it drives a wheel, reports telemetry and is armed. */
    val isMotorController: Boolean get() = nodeClass == Fields.NodeClass.MOTOR_CONTROLLER

    /** A hoverboard bridge: forwards between its BLE link and a board, and drives nothing. */
    val isBridge: Boolean get() = nodeClass == Fields.NodeClass.BRIDGE
}

/**
 * Why the app refuses to arm the machine it is attached to although it can read what the machine
 * stores, or cannot trust what it stores (`specs/remote-ui.md` 3.1c): the Ride statements say which
 * in short, and Setup's machine status section in full. The app never drives a mode the machine
 * does not store, so none of these falls back to driving one board.
 */
enum class DriveRefusal {
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
 * The drive mode in force and the motor controllers it addresses (`specs/remote-ui.md` section 2,
 * 3.2 and 3.1c): the stored mode and its boards when every copy agrees and its precondition holds
 * on the connected machine. Else [refusal] says why, and [targets] carries the stored mode where
 * one is known (SINGLE where it is not) and NO boards, so the arm is refused through the rule
 * [com.hoverboard.remote.UiState.canArm] already has and the pad drawn is the stored mode's.
 *
 * @param stored the mode the machine stores, when every copy agrees on one this app knows, else null.
 */
data class DriveInForce(val targets: DriveTargets, val stored: DriveMode?, val refusal: DriveRefusal?) {
    val mode: DriveMode get() = targets.mode

    companion object {
        /** No node attached: nothing to drive and nothing to say. */
        val NONE = DriveInForce(DriveTargets.NONE, stored = null, refusal = null)

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
            val (mode, unread) = stored(nodes, mcs, storedMode)
            if (unread != null) return refused(mode, unread)
            if (mode == null) return DriveInForce(DriveTargets.NONE, stored = null, refusal = null)
            val why = precondition(mode, mcs, side, leaders)
            return when {
                why != null -> refused(mode, why)
                mode == DriveMode.SINGLE -> DriveInForce(
                    DriveTargets(DriveMode.SINGLE, listOfNotNull(selected?.takeIf { it in mcs })),
                    mode,
                    refusal = null,
                )
                else -> DriveInForce(DriveTargets(mode, mcs, rightWheel(mode, mcs, side)), mode, refusal = null)
            }
        }

        /** A refusal: the stored mode where it is known, for the pad, and no board to arm. */
        private fun refused(stored: DriveMode?, why: DriveRefusal): DriveInForce =
            DriveInForce(DriveTargets(stored ?: DriveMode.SINGLE, emptyList()), stored, why)


        /**
         * The mode every motor controller stores, or null with the reason it cannot be trusted. Null
         * with no reason on a machine with no motor controller, which stores nothing to read.
         */
        private fun stored(
            nodes: List<NodeInfo>,
            mcs: List<Int>,
            storedMode: (Int) -> Int?,
        ): Pair<DriveMode?, DriveRefusal?> {

            val copies = mcs.map(storedMode)
            val distinct = copies.toSet()
            val why = when {
                nodes.any { it.nodeClass == null } || null in copies -> DriveRefusal.UNREAD
                mcs.isEmpty() -> null
                distinct.size > 1 -> DriveRefusal.DISAGREE
                else -> DriveRefusal.UNKNOWN_MODE.takeIf { DriveMode.of(checkNotNull(distinct.single())) == null }
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
        ): DriveRefusal? =
            when {
                !mode.drivesBoth -> null
                mcs.size != 2 -> DriveRefusal.NOT_TWO
                mode == DriveMode.RIGID && leaders.count { it in mcs } != 1 -> DriveRefusal.NO_LEADER
                mode == DriveMode.DIFFERENTIAL && rightWheel(mode, mcs, side) == null -> DriveRefusal.NO_RIGHT_WHEEL
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
