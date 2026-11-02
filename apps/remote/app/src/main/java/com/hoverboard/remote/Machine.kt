package com.hoverboard.remote

import com.hoverboard.protocol.checks.Checks
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Scope
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.DriveInForce
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.asLong

/**
 * What `machine.frame` reads across the machine's motor controllers (`specs/remote-ui.md`
 * 3.7a, `specs/rover-one-imu.md` section 7, "Configuring the pair from the phone").
 *
 * The frame is MACHINE policy: it wants the same value on every motor controller, and a machine
 * that disagrees is one wheel balancing and the other coasting. No board can see another's store,
 * so the app reading every copy is the only thing that can tell.
 *
 * The comparison is of EFFECTIVE values, not raw bytes: the firmware runs every byte that is not
 * [Fields.ControlFrame.RIGID] as pivoting, so two boards holding 0 and 7 run the same frame and are
 * [Agree], not [Differ].
 */
sealed interface FramePair {
    /** A copy has not been read this session, or the walk found fewer than two motor controllers. */
    data object Unknown : FramePair

    /** Every motor controller runs the same frame. */
    data class Agree(val rigid: Boolean) : FramePair

    /** The motor controllers run different frames: [rigid] is each one's, by address. */
    data class Differ(val rigid: Map<Int, Boolean>) : FramePair
}

/**
 * What motor 0's `side` reads across the motor controllers (`specs/remote-ui.md` 3.7a). A wheel fact,
 * and it wants to DIFFER: the only correct machine is one right wheel and one left.
 *
 * As for [FramePair], the comparison is of effective values: the firmware reads 1 as the right
 * wheel and every other byte as the left.
 */
sealed interface SidePair {
    /** A copy has not been read this session, or the walk found fewer than two motor controllers. */
    data object Unknown : SidePair

    /** The correct machine: the board at [right] drives the right wheel and the other the left. */
    data class OneRight(val right: Int) : SidePair

    /** Every board is the left wheel. The default on a fresh machine, and inert until a turn is staged. */
    data object NoneRight : SidePair

    /** More than one board is the right wheel, which the picker cannot express and a raw edit can. */
    data object BothRight : SidePair
}

/**
 * What `control.leader` reads across the motor controllers (`specs/rigid-leader.md` section 7). A
 * node fact that wants to DIFFER: a rigid balance machine needs exactly one board flagged. Any
 * nonzero byte reads as set, as the firmware reads it.
 */
sealed interface LeaderPair {
    /** A copy has not been read this session, or the walk found fewer than two motor controllers. */
    data object Unknown : LeaderPair

    /** The board at [leader] is flagged and the other is not. */
    data class One(val leader: Int) : LeaderPair

    /** Neither board is flagged: the default, and all a machine needs outside rigid balance. */
    data object None : LeaderPair

    /** Both boards are flagged, which the picker cannot express and a raw edit past the check can. */
    data object Both : LeaderPair
}

/**
 * What the arm control says about the machine before it will arm (`specs/remote-ui.md` 3.7a,
 * "Disagreement: refuse for the frame, warn for the side").
 *
 * [FramesDiffer] REFUSES the arm, like a held throttle: the firmware cannot catch a frame mismatch
 * (a pivoting peer sends a cyclic indistinguishable from a healthy follower's), and the consequence
 * is half an actuator. The rest are warnings in the shape of [RiderWaiver.POSSIBLY]: the side is
 * never a refusal, because both-left is the default and refusing it would refuse every fresh
 * machine, where the side means nothing.
 */
sealed interface MachineArm {
    /** Whether this statement refuses the arm rather than only warning. */
    val refuses: Boolean get() = this is FramesDiffer

    /** Nothing to say. */
    data object None : MachineArm

    /** Every frame was read this session and they differ; [rigid] is each board's, by address. */
    data class FramesDiffer(val rigid: Map<Int, Boolean>) : MachineArm

    /**
     * A frame has not been read, so the app cannot tell whether they match. [peerFound] separates
     * the two unknowns: a second motor controller that was found and did not answer is read again,
     * while a session that found only one needs the other board powered first.
     */
    data class FrameUnread(val peerFound: Boolean) : MachineArm

    /** The machine runs rigid and [pair] is not exactly one right wheel. */
    data class SideNotOneRight(val pair: SidePair) : MachineArm
}

/**
 * The machine settings as this session has read them on every motor controller, and what follows
 * for the arm control and the disagreement badge (`specs/remote-ui.md` 3.7a).
 */
data class Machine(
    val frame: FramePair,
    val side: SidePair,
    val peerFound: Boolean = false,
    val leader: LeaderPair = LeaderPair.Unknown,
) {

    /**
     * The one statement the arm control makes, strongest first: a known frame difference refuses,
     * an unread frame warns, and a rigid machine without exactly one right wheel warns. The side is
     * judged only once the machine is known to run rigid, because in pivoting mode it means nothing.
     */
    val armStatement: MachineArm
        get() = when {
            frame is FramePair.Differ -> MachineArm.FramesDiffer(frame.rigid)
            frame is FramePair.Unknown -> MachineArm.FrameUnread(peerFound)
            frame is FramePair.Agree && frame.rigid && side !is SidePair.OneRight ->
                MachineArm.SideNotOneRight(side)
            else -> MachineArm.None
        }

    /**
     * Whether the machine is in a state the badge of 3.7 calls out: the boards hold frames that
     * differ, more than one claims the right wheel, or the machine runs rigid with none on the right.
     *
     * A fresh pivoting machine with every board on the left is NOT badged: that is the default, it
     * is inert while no turn bound is staged, and badging it would mark every unconfigured board.
     */
    val disagrees: Boolean
        get() = frame is FramePair.Differ ||
            side is SidePair.BothRight ||
            (frame is FramePair.Agree && frame.rigid && side is SidePair.NoneRight)

    companion object {
        /** Nothing read: what a session with no board attached knows. */
        val UNKNOWN = Machine(FramePair.Unknown, SidePair.Unknown, peerFound = false)
    }
}

/**
 * The machine settings as this session has read them. [Machine.UNKNOWN] while no node is attached,
 * and a verdict is unknown while a copy it needs is unread or its board was never found
 * (`specs/remote-ui.md` 3.7a, "The follower out of reach").
 */
val SetupState.machine: Machine
    get() {
        if (attached == null) return Machine.UNKNOWN
        val mcs = motorControllers
        return Machine(framePair(mcs), sidePair(mcs), peerFound = mcs.size >= 2, leader = leaderPair(mcs))
    }

private fun SetupState.leaderPair(mcs: List<Int>): LeaderPair {
    if (mcs.size < 2) return LeaderPair.Unknown
    val set = mcs.filter { (storedByte(it, SetupFields.CONTROL_LEADER.key) ?: return LeaderPair.Unknown) != 0 }
    return when (set.size) {
        0 -> LeaderPair.None
        1 -> LeaderPair.One(set.single())
        else -> LeaderPair.Both
    }
}

/**
 * Whether the machine as stored fails the leader check (`specs/rigid-leader.md` section 6.1): a
 * Balance machine in the rigid frame without exactly one board flagged, reachable only by a write
 * past the clients. False while any of the three keys is unread on a motor controller.
 */
val SetupState.leaderCheckFails: Boolean
    get() {
        val boards = motorControllers.map { a ->
            a to Checks.LEADER_KEYS.associateWith { setupOf(a).values[it] ?: return false }
        }
        return boards.isNotEmpty() && Checks.checkLeader(boards) != null
    }

/** The board a write in flight is making the rigid frame's leader, or null while none is. */
val SetupState.leaderSending: Int?
    get() = motorControllers.firstOrNull { sendingByte(it, SetupFields.CONTROL_LEADER.key) == 1 }

private fun SetupState.framePair(mcs: List<Int>): FramePair {
    if (mcs.size < 2) return FramePair.Unknown
    val rigid = mcs.associateWith {
        (storedByte(it, SetupFields.CONTROL_FRAME.key) ?: return FramePair.Unknown) == Fields.ControlFrame.RIGID
    }
    return if (rigid.values.toSet().size == 1) FramePair.Agree(rigid.values.first()) else FramePair.Differ(rigid)
}

private fun SetupState.sidePair(mcs: List<Int>): SidePair {
    if (mcs.size < 2) return SidePair.Unknown
    val right = mcs.filter {
        (storedByte(it, SetupFields.CONTROL_SIDE.key) ?: return SidePair.Unknown) == Fields.ControlSide.RIGHT
    }
    return when (right.size) {
        0 -> SidePair.NoneRight
        1 -> SidePair.OneRight(right.single())
        else -> SidePair.BothRight
    }
}

/** The stored byte for [key] on the board at [address], or null while it is unread. */
internal fun SetupState.storedByte(address: Int, key: Key): Int? =
    setupOf(address).values[key]?.asLong()?.toInt()

/**
 * The byte a write in flight is taking every motor controller's [key] to, or null while none is
 * (`specs/remote-ui.md` 3.6a: the chips show what the board will hold, as every other row does).
 * One value for the machine, because [key] is one that goes to every board with the same value.
 */
fun SetupState.machineSending(key: Key): Int? =
    motorControllers.firstNotNullOfOrNull { sendingByte(it, key) }

/** The board a write in flight is making the right wheel, or null while none is. */
val SetupState.rightWheelSending: Int?
    get() = motorControllers.firstOrNull { sendingByte(it, SetupFields.CONTROL_SIDE.key) == Fields.ControlSide.RIGHT }

/** The value the board at [address]'s queued or in-flight write of [key] will store, or null. */
private fun SetupState.sendingByte(address: Int, key: Key): Int? =
    setupOf(address).writes[key]?.takeIf { it.sending }?.want?.asLong()?.toInt()

/**
 * The drive mode in force on the machine this session sees (`specs/remote-ui.md` section 2 and 3.2;
 * `specs/machine-settings.md` section 7), derived from the stored `machine.mode` and `machine.frame`
 * on the motor controllers, the number of them, the Ride screen's turn-axis setting [turnAxis], each
 * board's motor 0 `side`, and each board's stored `control.leader` (`specs/rigid-leader.md` section
 * 7). [DriveInForce.NONE] while no node is attached.
 */
fun SetupState.driveInForce(turnAxis: Boolean): DriveInForce =
    if (attached == null) {
        DriveInForce.NONE
    } else {
        DriveInForce.of(
            nodes = nodes,
            mode = { storedByte(it, SetupFields.CONTROL_MODE.key) },
            frame = { storedByte(it, SetupFields.CONTROL_FRAME.key) },
            side = { storedByte(it, SetupFields.CONTROL_SIDE.key) },
            leader = { storedByte(it, SetupFields.CONTROL_LEADER.key) },
            turnAxis = turnAxis,
        )
    }

/**
 * Every key of every machine field (`specs/machine-settings.md` section 3.2), each index: what the
 * machine digest covers, and what Setup reads from each motor controller once a board reports that
 * the boards disagree (`EV_MACHINE`), to list the fields that differ.
 */
val MACHINE_KEYS: List<Key> =
    (
        Fields.ALL.values.filter { it.scope == Scope.MACHINE }.map { it.key() } +
            Fields.INDEXED.values.filter { it.scope == Scope.MACHINE }
                .flatMap { f -> (0 until f.size).map { f.at(it).key(it) } }
        ).sortedWith(compareBy({ it.fieldId }, { it.index }))

/**
 * The machine fields whose copies differ across the motor controllers, as `machine diff` lists them
 * (`specs/machine-settings.md` sections 6.3 and 7): each key with every board's value by address,
 * for the keys read on every motor controller. Empty while the copies agree or are not read apart.
 */
val SetupState.machineDifferences: Map<Key, Map<Int, Value>>
    get() {
        val mcs = motorControllers
        if (mcs.size < 2) return emptyMap()
        return MACHINE_KEYS.mapNotNull { key ->
            val each = mcs.associateWith { setupOf(it).values[key] ?: return@mapNotNull null }
            if (each.values.toSet().size > 1) key to each else null
        }.toMap()
    }
