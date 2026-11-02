package com.hoverboard.remote.model

import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.store.Fields

/**
 * One node the session's walk found (`specs/remote-ui.md` section 2): its address, its class from
 * its `IDENTITY` (`specs/machine-settings.md` section 6.1) and its `device.name`, the last two null
 * until the request for each has answered.
 *
 * Nodes are named by name and address, never by role or by which one the phone attached to: the
 * node on the phone's own link is a bridge when it came in through a bridge and a motor controller
 * when it came in through a board's own BLE module (`specs/l3.md`, "Node class").
 */
data class NodeInfo(val address: Int, val nodeClass: Int?, val name: String?) {
    /** A hoverboard mainboard: it drives a wheel, reports telemetry and is armed. */
    val isMotorController: Boolean get() = nodeClass == Walk.CLASS_MOTOR

    /** A hoverboard bridge: forwards between its BLE link and a board, and drives nothing. */
    val isBridge: Boolean get() = nodeClass == Walk.CLASS_BRIDGE
}

/**
 * Why the app refuses to arm the machine it is attached to although it can read what the machine
 * stores, or cannot trust what it stores (`specs/remote-ui.md` 3.1c): the Ride statements say which
 * in short, and Setup's machine status section in full. The app never drives a mode the machine
 * does not derive to, so none of these falls back to driving one board.
 */
enum class DriveRefusal {
    /**
     * A node's identity, or a motor controller's `machine.mode` or `machine.frame` (or, for RIGID,
     * its `control.leader`), has not been read this session.
     */
    UNREAD,

    /** The motor controllers hold different `machine.mode` or `machine.frame` copies. */
    DISAGREE,

    /** The mode drives two motor controllers and the walk found another number of them. */
    NOT_TWO,

    /** RIGID, and not exactly one motor controller stores `control.leader` set. */
    NO_LEADER,

    /** DIFFERENTIAL, and not exactly one motor controller is set as the right wheel. */
    NO_RIGHT_WHEEL,
}

/**
 * The drive mode in force and the motor controllers it addresses (`specs/remote-ui.md` section 2,
 * 3.2 and 3.1c; `specs/machine-settings.md` section 7): the mode the machine derives to and its
 * boards when its precondition holds on the connected machine. Else [refusal] says why, and
 * [targets] carries the derived mode where one is known (SINGLE where it is not) and NO boards, so
 * the arm is refused through the rule [com.hoverboard.remote.UiState.canArm] already has and the pad
 * drawn is the derived mode's.
 *
 * @param derived the mode the machine derives to, when every copy it is derived from is read and
 *   agrees, else null.
 */
data class DriveInForce(val targets: DriveTargets, val derived: DriveMode?, val refusal: DriveRefusal?) {
    val mode: DriveMode get() = targets.mode

    companion object {
        /** No node attached: nothing to drive and nothing to say. */
        val NONE = DriveInForce(DriveTargets.NONE, derived = null, refusal = null)

        /**
         * The mode in force on the machine [nodes] describes.
         *
         * @param mode each motor controller's stored `machine.mode` byte, null while unread.
         * @param frame each motor controller's stored `machine.frame` byte, null while unread.
         * @param side each motor controller's motor 0 `side` byte, null while unread.
         * @param leader each motor controller's stored `control.leader` byte, null while unread
         *   (`specs/rigid-leader.md` section 7).
         * @param turnAxis whether a machine of two or more motor controllers that is not rigid uses
         *   the pad's turn axis: DIFFERENTIAL with it, BOUND without (the Ride screen's setting).
         */
        fun of(
            nodes: List<NodeInfo>,
            mode: (Int) -> Int?,
            frame: (Int) -> Int?,
            side: (Int) -> Int?,
            leader: (Int) -> Int?,
            turnAxis: Boolean,
        ): DriveInForce {
            val mcs = nodes.filter { it.isMotorController }.map { it.address }
            val (derived, unread) = derive(nodes, mcs, mode, frame, turnAxis)
            if (unread != null) return refused(derived, unread)
            if (derived == null) return NONE
            val why = precondition(derived, mcs, side, leader)
            return when {
                why != null -> refused(derived, why)
                derived == DriveMode.SINGLE -> DriveInForce(DriveTargets(DriveMode.SINGLE, mcs), derived, null)
                else -> DriveInForce(
                    DriveTargets(derived, mcs, rightWheel(derived, mcs, side), rigidLeader(derived, mcs, leader)),
                    derived,
                    null,
                )
            }
        }

        /** A refusal: the derived mode where it is known, for the pad, and no board to arm. */
        private fun refused(derived: DriveMode?, why: DriveRefusal): DriveInForce =
            DriveInForce(DriveTargets(derived ?: DriveMode.SINGLE, emptyList()), derived, why)

        /**
         * The mode the machine derives to, or null with the reason it cannot be trusted. Null with
         * no reason on a machine with no motor controller, which has nothing to derive from.
         */
        private fun derive(
            nodes: List<NodeInfo>,
            mcs: List<Int>,
            mode: (Int) -> Int?,
            frame: (Int) -> Int?,
            turnAxis: Boolean,
        ): Pair<DriveMode?, DriveRefusal?> {
            // Effective values: the firmware runs every mode byte but BALANCE as throttle and every
            // frame byte but RIGID as pivoting.
            val balance = mcs.map { a -> mode(a)?.let { it == Fields.ControlMode.BALANCE } }
            val rigidFrame = mcs.map { a -> frame(a)?.let { it == Fields.ControlFrame.RIGID } }
            val unread = nodes.any { it.nodeClass == null } || null in balance || null in rigidFrame
            return when {
                unread -> null to DriveRefusal.UNREAD
                mcs.isEmpty() -> null to null
                balance.toSet().size > 1 || rigidFrame.toSet().size > 1 -> null to DriveRefusal.DISAGREE
                else -> modeOf(balance.first() == true && rigidFrame.first() == true, mcs.size, turnAxis) to null
            }
        }

        /** The mode a machine whose copies agree derives to (`specs/machine-settings.md` section 7). */
        private fun modeOf(rigid: Boolean, motorControllers: Int, turnAxis: Boolean): DriveMode = when {
            rigid -> DriveMode.RIGID
            motorControllers == 1 -> DriveMode.SINGLE
            turnAxis -> DriveMode.DIFFERENTIAL
            else -> DriveMode.BOUND
        }

        /** Why [mode] cannot run on this machine, or null when it can. */
        private fun precondition(
            mode: DriveMode,
            mcs: List<Int>,
            side: (Int) -> Int?,
            leader: (Int) -> Int?,
        ): DriveRefusal? =
            when {
                !mode.drivesBoth -> null
                mcs.size != 2 -> DriveRefusal.NOT_TWO
                mode == DriveMode.RIGID && mcs.any { leader(it) == null } -> DriveRefusal.UNREAD
                mode == DriveMode.RIGID && rigidLeader(mode, mcs, leader) == null -> DriveRefusal.NO_LEADER
                mode == DriveMode.DIFFERENTIAL && rightWheel(mode, mcs, side) == null -> DriveRefusal.NO_RIGHT_WHEEL
                else -> null
            }

        /**
         * The board that stores `control.leader` set, for RIGID: exactly one such board with every
         * copy read, else null (`specs/rigid-leader.md` section 7). The stored flag is what each
         * board installs at its next arm, so it names the leader whatever the boards ran since.
         */
        private fun rigidLeader(mode: DriveMode, mcs: List<Int>, leader: (Int) -> Int?): Int? {
            if (mode != DriveMode.RIGID || mcs.any { leader(it) == null }) return null
            return mcs.filter { leader(it) != 0 }.singleOrNull()
        }

        /**
         * The board whose own motor's `side` says it is the right wheel, for DIFFERENTIAL: exactly
         * one such board with every copy read, else null.
         */
        private fun rightWheel(mode: DriveMode, mcs: List<Int>, side: (Int) -> Int?): Int? {
            if (mode != DriveMode.DIFFERENTIAL || mcs.any { side(it) == null }) return null
            return mcs.filter { side(it) == Fields.ControlSide.RIGHT }.singleOrNull()
        }
    }
}
