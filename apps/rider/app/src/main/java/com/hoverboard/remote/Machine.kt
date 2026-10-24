package com.hoverboard.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.remote.model.DriveInForce
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.asLong

/**
 * What `control.frame` (0x25) reads across the machine's motor controllers (`specs/rider-ui.md`
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
 * What `control.side` (0x26) reads across the motor controllers (`specs/rider-ui.md` 3.7a). This one
 * is a BOARD fact and wants to DIFFER: the only correct machine is one right wheel and one left.
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
 * What `machine.drive_mode` (0x78) reads across the motor controllers (`specs/rider-ui.md` 3.2):
 * the stored setting the app drives by. Unlike [FramePair] it is judged on a machine with ONE motor
 * controller too, because SINGLE is a mode a one-board machine stores and drives in.
 */
sealed interface ModePair {
    /** A motor controller's copy has not been read this session, or none was found. */
    data object Unknown : ModePair

    /** Every motor controller stores [byte]. */
    data class Agree(val byte: Int) : ModePair

    /** The copies differ: [bytes] is each motor controller's, by address. */
    data class Differ(val bytes: Map<Int, Int>) : ModePair
}

/**
 * What the arm control says about the machine before it will arm (`specs/rider-ui.md` 3.7a,
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
 * for the arm control and the disagreement badge (`specs/rider-ui.md` 3.7a).
 */
data class Machine(
    val frame: FramePair,
    val side: SidePair,
    val mode: ModePair,
    val peerFound: Boolean = false,
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
     * Whether the machine is in a state the badge of 3.7 calls out: the boards hold frames or drive
     * modes that differ, more than one claims the right wheel, or the machine runs rigid with none
     * on the right.
     *
     * A fresh pivoting machine with every board on the left is NOT badged: that is the default, it
     * is inert while no turn bound is staged, and badging it would mark every unconfigured board.
     */
    val disagrees: Boolean
        get() = frame is FramePair.Differ ||
            mode is ModePair.Differ ||
            side is SidePair.BothRight ||
            (frame is FramePair.Agree && frame.rigid && side is SidePair.NoneRight)

    companion object {
        /** Nothing read: what a session with no board attached knows. */
        val UNKNOWN = Machine(FramePair.Unknown, SidePair.Unknown, ModePair.Unknown, peerFound = false)
    }
}

/**
 * The machine settings as this session has read them. [Machine.UNKNOWN] while no node is attached,
 * and a verdict is unknown while a copy it needs is unread or its board was never found
 * (`specs/rider-ui.md` 3.7a, "The follower out of reach").
 */
val SetupState.machine: Machine
    get() {
        if (attached == null) return Machine.UNKNOWN
        val mcs = motorControllers
        return Machine(framePair(mcs), sidePair(mcs), modePair(mcs), peerFound = mcs.size >= 2)
    }

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

private fun SetupState.modePair(mcs: List<Int>): ModePair {
    if (mcs.isEmpty()) return ModePair.Unknown
    val bytes = mcs.associateWith { storedByte(it, SetupFields.DRIVE_MODE.key) ?: return ModePair.Unknown }
    return if (bytes.values.toSet().size == 1) ModePair.Agree(bytes.values.first()) else ModePair.Differ(bytes)
}

/** The stored byte for [key] on the board at [address], or null while it is unread. */
internal fun SetupState.storedByte(address: Int, key: Key): Int? =
    setupOf(address).values[key]?.asLong()?.toInt()

/**
 * The byte a write in flight is taking every motor controller's [key] to, or null while none is
 * (`specs/rider-ui.md` 3.6a: the chips show what the board will hold, as every other row does).
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
 * The drive mode in force on the machine this session sees (`specs/rider-ui.md` section 2 and 3.2),
 * from the stored copies of `machine.drive_mode` and `control.side` on every motor controller, the
 * rigid-frame leader flags in each board's own [telemetry], and the selected motor controller.
 * [DriveInForce.NONE] while no node is attached.
 */
fun SetupState.driveInForce(telemetry: Map<Int, TelemetryUi>): DriveInForce =
    if (attached == null) {
        DriveInForce.NONE
    } else {
        DriveInForce.of(
            nodes = nodes,
            storedMode = { storedByte(it, SetupFields.DRIVE_MODE.key) },
            side = { storedByte(it, SetupFields.CONTROL_SIDE.key) },
            leaders = telemetry.filterValues { it.leadsRigidFrame }.keys,
            selected = selectedMotor,
        )
    }
