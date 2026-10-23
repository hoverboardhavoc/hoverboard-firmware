package com.hoverboard.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.asLong

/**
 * The Machine section's chosen but unwritten values (`specs/rider-ui.md` 3.7a). Deliberately not
 * the per-board PENDING basket: these two go to BOTH boards, one of them with a different value on
 * each, and the write has an order the basket has no way to express.
 *
 * @param frame the `control.frame` byte to write to both boards, or null while none was chosen.
 * @param rightWheel the board to make the right wheel, or null while none was chosen. The other
 *   board is made the left wheel by the same write.
 */
data class MachineEdit(val frame: Int? = null, val rightWheel: Node? = null) {
    /** Whether anything is waiting to be written. */
    val any: Boolean get() = frame != null || rightWheel != null
}

/**
 * What `control.frame` (0x25) reads across the pair (`specs/rider-ui.md` 3.7a,
 * `specs/rover-one-imu.md` section 7, "Configuring the pair from the phone").
 *
 * The frame is MACHINE policy: it wants the same value on both boards, and a pair that disagrees is
 * one wheel balancing and the other coasting. No board can see its peer's store, so the app reading
 * both is the only thing that can tell.
 *
 * The comparison is of EFFECTIVE values, not raw bytes: the firmware runs every byte that is not
 * [Fields.ControlFrame.RIGID] as pivoting, so two boards holding 0 and 7 run the same frame and are
 * [Agree], not [Differ].
 */
sealed interface FramePair {
    /** A board's value has not been read this session, or this session found no slave. */
    data object Unknown : FramePair

    /** Both boards run the same frame. */
    data class Agree(val rigid: Boolean) : FramePair

    /** The boards run different frames; [masterRigid] is the master's, and the slave's is its opposite. */
    data class Differ(val masterRigid: Boolean) : FramePair
}

/**
 * What `control.side` (0x26) reads across the pair (`specs/rider-ui.md` 3.7a). This one is a BOARD
 * fact and wants to DIFFER: the only correct pair is one right wheel and one left.
 *
 * As for [FramePair], the comparison is of effective values: the firmware reads 1 as the right wheel
 * and every other byte as the left.
 */
sealed interface SidePair {
    /** A board's value has not been read this session, or this session found no slave. */
    data object Unknown : SidePair

    /** The correct pair: [right] drives the right wheel and the other board the left. */
    data class OneRight(val right: Node) : SidePair

    /** Both boards are the left wheel. The default on a fresh pair, and inert until a turn is staged. */
    data object NoneRight : SidePair

    /** Both boards are the right wheel, which the picker cannot express and a raw edit can. */
    data object BothRight : SidePair
}

/**
 * What the arm control says about the pair before it will arm (`specs/rider-ui.md` 3.7a,
 * "Disagreement: refuse for the frame, warn for the side").
 *
 * [FramesDiffer] REFUSES the arm, like a held throttle: the firmware cannot catch a frame mismatch
 * (a pivoting peer sends a cyclic indistinguishable from a healthy follower's), and the consequence
 * is half an actuator. The rest are warnings in the shape of [RiderWaiver.POSSIBLY]: the side is
 * never a refusal, because both-left is the default and refusing it would refuse every fresh pair,
 * where the side means nothing.
 */
sealed interface MachineArm {
    /** Whether this statement refuses the arm rather than only warning. */
    val refuses: Boolean get() = this is FramesDiffer

    /** Nothing to say. */
    data object None : MachineArm

    /** Both frames were read this session and they differ; [masterRigid] is the master's value. */
    data class FramesDiffer(val masterRigid: Boolean) : MachineArm

    /**
     * A board's frame has not been read, so the app cannot tell whether the two match.
     * [peerFound] separates the two unknowns: a second board that was found and did not answer is
     * read again, while a session that found none at all needs the other board powered first.
     */
    data class FrameUnread(val peerFound: Boolean) : MachineArm

    /** The pair runs rigid and [pair] is not exactly one right wheel. */
    data class SideNotOneRight(val pair: SidePair) : MachineArm
}

/**
 * The two rover settings as this session has read them on both boards, and what follows for the arm
 * control and the disagreement badge (`specs/rider-ui.md` 3.7a).
 */
data class MachinePair(val frame: FramePair, val side: SidePair, val peerFound: Boolean = false) {

    /**
     * The one statement the arm control makes, strongest first: a known frame difference refuses,
     * an unread frame warns, and a rigid pair without exactly one right wheel warns. The side is
     * judged only once the pair is known to run rigid, because in pivoting mode it means nothing.
     */
    val armStatement: MachineArm
        get() = when {
            frame is FramePair.Differ -> MachineArm.FramesDiffer(frame.masterRigid)
            frame is FramePair.Unknown -> MachineArm.FrameUnread(peerFound)
            frame is FramePair.Agree && frame.rigid && side !is SidePair.OneRight ->
                MachineArm.SideNotOneRight(side)
            else -> MachineArm.None
        }

    /**
     * Whether the pair is in a state the badge of 3.7 calls out: the two boards hold frames that
     * differ, both claim the right wheel, or the machine runs rigid with neither board on the right.
     *
     * A fresh pivoting pair with both boards on the left is NOT badged: that is the default, it is
     * inert while no turn bound is staged, and badging it would mark every unconfigured hoverboard.
     */
    val disagrees: Boolean
        get() = frame is FramePair.Differ ||
            side is SidePair.BothRight ||
            (frame is FramePair.Agree && frame.rigid && side is SidePair.NoneRight)

    companion object {
        /** Nothing read: what a session with no board attached knows. */
        val UNKNOWN = MachinePair(FramePair.Unknown, SidePair.Unknown, peerFound = false)
    }
}

/**
 * The pair's two rover settings as this session has read them. [MachinePair.UNKNOWN] while no board
 * is attached, and either verdict is unknown while the board it needs is unread or was never found
 * (`specs/rider-ui.md` 3.7a, "The follower out of reach").
 */
val SetupState.machine: MachinePair
    get() {
        if (master == null) return MachinePair.UNKNOWN
        return MachinePair(framePair(), sidePair(), peerFound = slave != null)
    }

private fun SetupState.framePair(): FramePair {
    val m = storedByte(Node.MASTER, SetupFields.CONTROL_FRAME.key) ?: return FramePair.Unknown
    val s = storedByte(Node.SLAVE, SetupFields.CONTROL_FRAME.key) ?: return FramePair.Unknown
    val masterRigid = m == Fields.ControlFrame.RIGID
    val slaveRigid = s == Fields.ControlFrame.RIGID
    return if (masterRigid == slaveRigid) FramePair.Agree(masterRigid) else FramePair.Differ(masterRigid)
}

private fun SetupState.sidePair(): SidePair {
    val m = storedByte(Node.MASTER, SetupFields.CONTROL_SIDE.key) ?: return SidePair.Unknown
    val s = storedByte(Node.SLAVE, SetupFields.CONTROL_SIDE.key) ?: return SidePair.Unknown
    val masterRight = m == Fields.ControlSide.RIGHT
    val slaveRight = s == Fields.ControlSide.RIGHT
    return when {
        masterRight && slaveRight -> SidePair.BothRight
        masterRight -> SidePair.OneRight(Node.MASTER)
        slaveRight -> SidePair.OneRight(Node.SLAVE)
        else -> SidePair.NoneRight
    }
}

/** [node]'s stored byte for [key], or null while that board is out of reach or the key is unread. */
private fun SetupState.storedByte(node: Node, key: Key): Int? {
    if (addressOf(node) == null) return null
    return boards[node]?.values?.get(key)?.asLong()?.toInt()
}
