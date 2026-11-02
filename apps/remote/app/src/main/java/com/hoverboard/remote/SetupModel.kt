package com.hoverboard.remote

import com.hoverboard.protocol.board.BoardStatus
import com.hoverboard.protocol.checks.Checks
import com.hoverboard.protocol.checks.Refusal
import com.hoverboard.protocol.config.Busy
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.ConfigReadResult
import com.hoverboard.protocol.config.ConfigWriteResult
import com.hoverboard.protocol.config.Identified
import com.hoverboard.protocol.config.Malformed
import com.hoverboard.protocol.config.ReadValue
import com.hoverboard.protocol.config.Refused
import com.hoverboard.protocol.config.StatusEntries
import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.config.WriteMismatch
import com.hoverboard.protocol.config.WriteVerified
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.l3.Identity
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Effect
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Scope
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.model.Editor
import com.hoverboard.remote.model.FloorOutside
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.SpeedFacts
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.WheelGeometry
import com.hoverboard.remote.model.asLong
import com.hoverboard.remote.model.floorOutsidePack
import kotlin.math.abs
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex

/**
 * What became of one field's write (`specs/remote-ui.md` 3.6a: per-FIELD state, not one verdict).
 *
 * Every edit is its own transaction, so every one of these belongs to a row and is reported there.
 */
sealed interface WriteState {
    /** Queued or in flight: the settle window is open, or the lane has it now. */
    data object Sending : WriteState

    /** The board echoed the value back: it is stored. */
    data object Written : WriteState

    /** The board named a refusal. `CFG_ARMED` is the common one (R4 refuses every config write). */
    data class Refused(val refusal: CfgRefusal) : WriteState

    /** The board took the write but echoes a different stored value (or none). */
    data class Mismatch(val stored: Value?) : WriteState

    /** The write went unanswered through the whole retransmit budget. */
    data object Unanswered : WriteState

    /** The board answered with something the wire contract does not define. */
    data object Garbled : WriteState

    /**
     * The session's one request slot stayed taken by the Tune model's operations through the whole
     * wait ([awaitSlot]), so nothing was sent. Not an answer from the board; trying again is.
     */
    data object SlotBusy : WriteState

    /** No board was attached when this reached the lane, so nothing was sent. */
    data object NotSent : WriteState

    /**
     * Held back on purpose: a write this one had to FOLLOW did not land, and sending it alone
     * would leave the pair in the state the order exists to avoid.
     *
     * The right wheel is the one user (`specs/remote-ui.md` 3.7a): motor 0's `side` 0 goes to the
     * other board first and 1 to the chosen board second, so a sequence that stops between the two
     * leaves both boards on the left, never both on the right.
     */
    data object Skipped : WriteState
}

/**
 * One field's write this session: the value the app sent (or is about to) and what came of it.
 *
 * The record is what a row reports. It outlives a link drop, because the board keeps what it
 * stored, and it is dropped when the wire shows the board restarted.
 */
data class FieldWrite(val want: Value, val state: WriteState) {
    /** Whether the row is still waiting for an answer (or for its settle window to close). */
    val sending: Boolean get() = state == WriteState.Sending

    /** Whether the row has something to report other than success. */
    val failed: Boolean get() = !sending && state != WriteState.Written

    /**
     * Whether this write may have reached the store: the board said it did, said nothing, or said
     * something unexpected, and a write still in flight might at any moment.
     *
     * A named refusal, a taken slot, an unattached board and a held-back write are writes that
     * CERTAINLY did not land, so for those the value last read is still what the board holds.
     */
    val mayHaveLanded: Boolean get() = when (state) {
        is WriteState.Refused, WriteState.SlotBusy, WriteState.NotSent, WriteState.Skipped -> false
        else -> true
    }
}

/** What the last Setup operation came to, where it is not one field's business. */
sealed interface SetupNotice {
    /** The staged frame (signs and roles) is not a proper rotation, so nothing was written. */
    data class FrameRefused(val refusal: Orientation.Refusal) : SetupNotice

    /** The value is outside what the field takes. */
    data class OutOfRange(val key: Key) : SetupNotice

    /**
     * The configuration checks refused the write (`specs/config-read-once.md` section 4), so
     * nothing was written: [refusal] names the field and the reason.
     */
    data class CheckRefused(val refusal: Refusal) : SetupNotice

    /**
     * The `imu.model` write was not sent: the IMU bus it switches on (`imu.scl`, `imu.sda`,
     * `imu.i2c`) has not been read from this board, so the group check could not run.
     */
    data class BusUnread(val key: Key) : SetupNotice

    /**
     * A write that moves `machine.mode`, `machine.frame` or `control.leader` was not sent: the
     * motor controller at [address] has not had those three read this session, so the leader check
     * (`specs/rigid-leader.md` section 6.1) could not run.
     */
    data class LeaderUnread(val address: Int) : SetupNotice

    /** "Set level" could not compute a trim. */
    data class LevelUnavailable(val reason: Blocked) : SetupNotice

    /** The rotation check could not run. */
    data class CheckUnavailable(val reason: Blocked) : SetupNotice

}

/** Why a telemetry-reading flow (set level, the rotation check) cannot run. */
enum class Blocked {
    /** The board being edited has reported nothing this session, or has gone quiet since. */
    NO_TELEMETRY,

    /** The stored value it builds on has not been read. */
    NOT_READ,

    /**
     * A field it depends on was written this session, the board re-reads that field when it arms
     * ([Effect.ARM]), and its `FLAG_REARM` says it has not armed since: it runs the old value.
     */
    AWAITING_ARM,

    /**
     * A field it depends on was written this session, the board reads that field only at boot
     * ([Effect.BOOT]), and its `FLAG_REBOOT` says it has not restarted since: it runs the old value.
     */
    AWAITING_RESTART,
}

/** One step of the rotation check. */
enum class CheckResult {
    PASS,
    FAIL,

    /** The reading was too small to say either way (a lean not far enough to judge its sign). */
    INCONCLUSIVE,
}

/** What the arm control says about the rider requirement (`specs/control.md` (i)). */
enum class RiderWaiver {
    /** Nothing to say: the board requires a rider, or nothing this app knows says it might not. */
    NONE,

    /**
     * The board stores `CONTROL_RIDER_REQUIRED` at 0. The field takes effect at every arm
     * ([Effect.ARM]), so the arm this control makes runs it waived.
     */
    WAIVED,

    /**
     * What the board stores is unread, and a write of 0 to it this session may have landed (it went
     * unanswered, or answered with something the app could not read): it may arm waived.
     */
    POSSIBLY,
}

/** The rotation check's two steps, each null until run. */
data class RotationCheck(val level: CheckResult? = null, val forwardLean: CheckResult? = null)

/**
 * One board's Setup state: its store as this app last read it, and what this app has written to it
 * (`specs/remote-ui.md` sections 3.4 and 3.6a). The screen shows one of these at a time
 * ([SetupState.current]); the model keeps one per node address, because a write record made against
 * one board is not a statement about another.
 *
 * @param values the stored value of each key, as last read or verified-written. A machine field
 *   read once stands for every motor controller ([SetupState.machineApart]).
 * @param unread keys whose last read failed to arrive at all.
 * @param garbled keys the board ANSWERED and this app could not read the answer to. It is a
 *   separate set from [unread] because "Not read" is a false statement about a board that
 *   replied, and the two want different words. Two things land here. A `STR` record whose bytes
 *   are not valid UTF-8: the board stopped validating them in shrink round 2 and now stores and
 *   serves whatever was written (`specs/storage-layer.md`, the record types), while this app's
 *   value model carries
 *   text, so such a record decodes to nothing. And a refusal code this app's vocabulary does not
 *   have. A key is in exactly one of the two sets, never both.
 * @param writes what the app wrote to this board this session, per field, and what came of each
 *   ([FieldWrite]).
 * @param countsPerAmp this board's `motor.current_cal`, read on attach as a ride fact, or null
 *   while this session has not got it. Kept out of [values] because no screen edits it: it is read
 *   so the Ride screen can turn this board's current counts into amps
 *   ([com.hoverboard.remote.model.currentReading]), and a field with no row must not show up in
 *   the Setup screen's read bookkeeping.
 * @param polePairs this board's motor pole-pair count, and @param diameterMm its wheel's rolling
 *   diameter in millimetres, read on attach as ride facts the same way and kept out of [values]
 *   for the same reason; null for a field this session has not got, and 0 for a board that has
 *   never been told it. Together they are what turns the odometer's hall edges into a distance
 *   ([com.hoverboard.remote.model.WheelGeometry]).
 * @param status what this board's boot status record listed when it was read on attach with
 *   `STATUS_READ` (`specs/machine-settings.md` section 6.1): the functions its last boot configured
 *   and could not bring up (`specs/configured-links.md` section 5), empty when they all came up,
 *   null while this session has not read it.
 * @param identity what the node said it is in its `IDENTITY` (class, chip, firmware version, factory
 *   id), null while this session has not had the answer.
 * @param rearm the board's `FLAG_REARM` as its last frame carried it (an arm-effect field written
 *   since it last armed), null while it has sent none this session.
 * @param reboot the board's `FLAG_REBOOT` likewise (a boot-effect field written since it booted).
 */
data class BoardSetup(
    val values: Map<Key, Value> = emptyMap(),
    val unread: Set<Key> = emptySet(),
    val garbled: Set<Key> = emptySet(),
    val reading: Boolean = false,
    val writes: Map<Key, FieldWrite> = emptyMap(),
    val rotationCheck: RotationCheck = RotationCheck(),
    val countsPerAmp: Int? = null,
    val polePairs: Int? = null,
    val diameterMm: Int? = null,
    val status: List<BoardStatus.Entry>? = null,
    val identity: Identity? = null,
    val rearm: Boolean? = null,
    val reboot: Boolean? = null,
    /**
     * The keys of the IMU bus `imu.model` switches on (`imu.scl`, `imu.sda`, `imu.i2c`) that this
     * board stores, a key it does not store absent; null until read. The group check of an
     * `imu.model` write runs on them ([SetupModel]'s enqueue).
     */
    val imuBus: Map<Key, Value>? = null,
) {
    /** Whether any field's write is queued or in flight. */
    val writing: Boolean get() = writes.values.any { it.sending }

    val busy: Boolean get() = reading || writing

    /**
     * What this board's values say about the rider requirement (`CONTROL_RIDER_REQUIRED`,
     * `specs/control.md` (i)), for the arm control.
     *
     * The field takes effect at every arm, so the arm this control makes runs what the board STORES:
     * no guess about what it ran before is needed (`specs/machine-settings.md` section 5; while armed
     * the board refuses every write, so stored and running cannot part). [RiderWaiver.POSSIBLY] is
     * the one unknown left: the stored value unread after a write of 0 that may have landed.
     */
    val riderWaiver: RiderWaiver
        get() {
            val key = SetupFields.RIDER_REQUIRED.key
            val waived = Value.U8(Fields.RiderRequired.NOT_REQUIRED)
            val stored = values[key]
            if (stored != null) return if (stored == waived) RiderWaiver.WAIVED else RiderWaiver.NONE
            val write = writes[key]
            val maybe = write != null && write.mayHaveLanded && write.want == waived
            return if (maybe) RiderWaiver.POSSIBLY else RiderWaiver.NONE
        }

    /**
     * Whether the stored frame (the sign map read through the stored roles) is one the board will
     * refuse: it is not a proper rotation, or part of it is unread while a frame index was written
     * (a frame write stopped part way and the rest is in doubt).
     *
     * The firmware refuses such a frame and does not bring up the IMU, and the board's boot status record
     * lists it (reason 7, `specs/configured-links.md` section 5), so balance is lost until it is
     * corrected. No write order avoids an illegal
     * intermediate: any two of the four flat rotations differ in two signs of a triple, and the
     * writes go one at a time. So a frame write that stops part way can leave one stored, and only
     * completing it makes it safe. A board whose stored frame was already illegal is flagged the same.
     */
    val storedFrameUnsafe: Boolean
        get() {
            val signs = storedSigns
            val roles = storedRoles
            if (signs != null && roles != null) return Orientation.check(signs, roles) != null
            return SetupFields.FRAME.any { writes[it.key]?.mayHaveLanded == true }
        }

    /** The stored sign map, or null while any index is unread. */
    val storedSigns: List<Int>? get() = ints(SetupFields.AXIS_SIGN) { values[it] }

    /** The stored axis roles `[UP, PITCH_RATE]`, or null while either index is unread. */
    val storedRoles: List<Int>? get() = ints(SetupFields.AXIS_ROLE) { values[it] }

    /** The map the board will hold once the writes in flight are through, or null if unknown. */
    val intendedSigns: List<Int>? get() = ints(SetupFields.AXIS_SIGN) { sending(it) ?: values[it] }

    /** The roles the board will hold once the writes in flight are through, or null if unknown. */
    val intendedRoles: List<Int>? get() = ints(SetupFields.AXIS_ROLE) { sending(it) ?: values[it] }

    private fun sending(key: Key): Value? = writes[key]?.takeIf { it.sending }?.want

    /**
     * The pose the stored fields make the board run once it re-reads them (`Orientation.poseOf`),
     * or null while either field is unread, any sign is unset (the fallback is no picked pose), or
     * the stored frame is none of the 24.
     */
    val storedPose: Orientation.Pose? get() = pose(storedRoles, storedSigns)

    /** The pose the board will hold once the writes in flight are through, or null as [storedPose]. */
    val intendedPose: Orientation.Pose? get() = pose(intendedRoles, intendedSigns)

    private fun pose(roles: List<Int>?, signs: List<Int>?): Orientation.Pose? =
        if (roles == null || signs == null) null else Orientation.poseOf(roles, signs)

    /**
     * Whether the stored orientation is what the board is running: no sign or role index written
     * this session is still waiting for the arm that takes it up. The board says which, with
     * `FLAG_REARM` ([rearm]); a board that has sent no frame is taken as not having armed.
     */
    val orientationSettled: Boolean
        get() = SetupFields.FRAME.none { pending(it) }

    /**
     * Whether a write of [field] this session may have landed and the board has not taken it up
     * yet: an arm-effect field while its [rearm] flag is set, a boot-effect one while [reboot] is.
     * A flag not yet heard counts as set. Other effects never wait on the board.
     */
    fun pending(field: SetupField): Boolean {
        if (writes[field.key]?.mayHaveLanded != true) return false
        return when (field.takesEffect) {
            Effect.ARM -> rearm != false
            Effect.BOOT -> reboot != false
            else -> false
        }
    }

    /** Whether any index of the IMU frame has a write queued or in flight. */
    val frameWriting: Boolean get() = SetupFields.FRAME.any { writes[it.key]?.sending == true }

    private fun ints(rows: List<SetupField>, of: (Key) -> Value?): List<Int>? =
        rows.map { (of(it.key)?.asLong() ?: return null).toInt() }

    /**
     * This board once the phone link has dropped: nothing read stands. The write records stay: the
     * board keeps what it stored, whether or not the phone can still see it.
     */
    fun dropped(): BoardSetup = copy(
        values = emptyMap(),
        unread = emptySet(),
        garbled = emptySet(),
        rotationCheck = RotationCheck(),
        countsPerAmp = null,
        polePairs = null,
        diameterMm = null,
        identity = null,
        rearm = null,
        reboot = null,
    )
}

/**
 * The Setup screen's state (`specs/remote-ui.md` sections 2, 3.4, 3.6a and 3.7a): the nodes this
 * session's walk found, which one is selected, and each node's [BoardSetup].
 *
 * It is also where the app's NODE LIST lives: each node's class from its `IDENTITY` and its
 * `device.name` ([nodes]), so the list and the Setup rows cannot tell two stories.
 *
 * @param addresses every node the walk addressed, ascending; empty while no session is attached.
 * @param attached the node on the phone's own link, or null while none is attached. The link
 *   endpoint and nothing more: a power-cycle of it drops the phone link, which no other node's does.
 * @param target the node the Setup selector last picked, of any class, or null for the default.
 * @param motor the motor controller the selectors last picked, which is the one SINGLE drives and
 *   Tune tunes ([selectedMotor]), or null for the default.
 * @param boards each node's state, by address; a node the session has not touched has none yet.
 * @param machineApart whether the machine fields have been read from each motor controller this
 *   session, because a board reported `EV_MACHINE` (`specs/machine-settings.md` sections 4.2 and 7).
 *   Until then a machine field is read once and stands for every motor controller.
 */
data class SetupState(
    val addresses: List<Int> = emptyList(),
    val attached: Int? = null,
    val target: Int? = null,
    val motor: Int? = null,
    val boards: Map<Int, BoardSetup> = emptyMap(),
    val notice: SetupNotice? = null,
    val rideFactsRead: Boolean = false,
    val machineApart: Boolean = false,
) {
    /** Every node, in address order, with its class and name as far as this session has read them. */
    val nodes: List<NodeInfo>
        get() = addresses.map { address ->
            val values = setupOf(address).values
            NodeInfo(
                address = address,
                nodeClass = setupOf(address).identity?.nodeClass,
                name = (values[SetupFields.DEVICE_NAME.key] as? Value.Str)?.v,
            )
        }

    /** The node at [address], or null when the walk did not find one there. */
    fun nodeAt(address: Int): NodeInfo? = nodes.firstOrNull { it.address == address }

    /** The motor controllers, in address order: the nodes whose class says they drive a wheel. */
    val motorControllers: List<Int> get() = nodes.filter { it.isMotorController }.map { it.address }

    /**
     * The motor controller the selectors point at: the one last picked while it is still found, else
     * the node on the phone's own link when it is a motor controller (so a connect through a board's
     * own module drives that board, as it always has), else the first. What SINGLE drives and what
     * Tune tunes. Null while no motor controller is known.
     */
    val selectedMotor: Int?
        get() = motorControllers.let { mcs ->
            motor?.takeIf { it in mcs } ?: attached?.takeIf { it in mcs } ?: mcs.firstOrNull()
        }

    /**
     * The node the Setup screen edits: the one last picked while it is still found, else the
     * selected motor controller, else the first node. Null while no node is attached.
     */
    val board: Int?
        get() = target?.takeIf { it in addresses } ?: selectedMotor ?: addresses.firstOrNull()

    /**
     * The target node's state. With no session attached that is the node last edited ([target]),
     * so what was written to it stays on screen after a drop.
     */
    val current: BoardSetup get() = (board ?: target)?.let { boards[it] } ?: BoardSetup()

    /** The target node's class, or null while it is unread. */
    val currentClass: Int? get() = board?.let { nodeAt(it)?.nodeClass }

    /** The rows the screen shows for the target node ([SetupFields.rowsFor]). */
    val rows: List<SetupField> get() = SetupFields.rowsFor(currentClass)

    /** The node at [address]'s state, empty for a node this session has not touched. */
    fun setupOf(address: Int): BoardSetup = boards[address] ?: BoardSetup()

    /**
     * The board at [address]'s counts per amp (`motor.current_cal`), or null while this session has
     * not read it. The Ride screen's current display needs it per board, because the scale belongs
     * to the board's own sense chain rather than to the firmware.
     */
    fun countsPerAmpOf(address: Int): Int? = setupOf(address).countsPerAmp

    /**
     * The board at [address]'s stored pack cell count (`board.battery.cells`), or null while this
     * session has not read it from that board. A stored 0 is passed through as the 0 it is: it means
     * nobody has said what pack this machine runs, and the Ride screen's gauge treats that as a pack
     * it cannot judge rather than as an eight-cell one.
     */
    fun packCellsOf(address: Int): Int? =
        setupOf(address).values[SetupFields.PACK_CELLS.key]?.asLong()?.toInt()

    /**
     * The board at [address]'s stored pack chemistry (`board.battery.chemistry`), or null while
     * this session has not read it from that board. A byte the app has no table for is passed
     * through, and the gauge reads it as a pack it cannot judge.
     */
    fun packChemistryOf(address: Int): Int? =
        setupOf(address).values[SetupFields.BATTERY_CHEMISTRY.key]?.asLong()?.toInt()

    /**
     * The board at [address]'s stored ride fact [field] (one of [SetupFields.MOTOR_FACTS]), or null
     * while this session has not read it from that board.
     */
    fun rideFactOf(address: Int, field: SetupField): Int? =
        setupOf(address).values[field.key]?.asLong()?.toInt()

    /**
     * Whether the target node's stored low-battery floor sits outside its own stated pack
     * ([floorOutsidePack], `specs/remote-ui.md` 3.6d), which the floor row warns about.
     */
    val floorOutside: FloorOutside?
        get() = current.values.let { v ->
            floorOutsidePack(
                floor = v[SetupFields.BATTERY_FLOOR.key]?.asLong()?.toInt(),
                cells = v[SetupFields.PACK_CELLS.key]?.asLong()?.toInt(),
                chemistry = v[SetupFields.BATTERY_CHEMISTRY.key]?.asLong()?.toInt(),
            )
        }

    /**
     * The board at [address]'s wheel geometry, or null while it cannot convert: a board this session
     * has not read both facts from, and a board that has been told neither or only one of them. Per
     * board, because a machine may carry two different wheels, and because a figure converted
     * against ANOTHER board's hub would be wrong with nothing to show it.
     */
    fun wheelGeometryOf(address: Int): WheelGeometry? =
        setupOf(address).let { WheelGeometry.of(it.polePairs, it.diameterMm) }

    /**
     * What the board at [address] brings to the Ride panel's speed beside its [wheelRateQ12]
     * (`specs/remote-ui.md` 3.1a): its pole pairs, wheel diameter and motor 0's `side`, each null
     * while this session has not read it.
     */
    fun speedFactsOf(address: Int, wheelRateQ12: Int): SpeedFacts = setupOf(address).let {
        SpeedFacts(
            address,
            wheelRateQ12,
            it.polePairs,
            it.diameterMm,
            storedByte(address, SetupFields.CONTROL_SIDE.key),
        )
    }

    val values: Map<Key, Value> get() = current.values
    val unread: Set<Key> get() = current.unread
    val garbled: Set<Key> get() = current.garbled
    val reading: Boolean get() = current.reading
    val writes: Map<Key, FieldWrite> get() = current.writes
    val rotationCheck: RotationCheck get() = current.rotationCheck

    /**
     * Whether a write is queued or in flight to ANY node. The Machine section writes to every motor
     * controller (`specs/remote-ui.md` 3.7a), so the screen's one busy statement is the machine's,
     * not the selected node's.
     */
    val writing: Boolean get() = boards.values.any { it.writing }

    val busy: Boolean get() = reading || writing

    val storedFrameUnsafe: Boolean get() = current.storedFrameUnsafe
    val storedSigns: List<Int>? get() = current.storedSigns
    val storedRoles: List<Int>? get() = current.storedRoles
    val storedPose: Orientation.Pose? get() = current.storedPose
    val intendedPose: Orientation.Pose? get() = current.intendedPose
    val orientationSettled: Boolean get() = current.orientationSettled
    val frameWriting: Boolean get() = current.frameWriting

    /** What the board at [address] says about its rider requirement ([BoardSetup.riderWaiver]). */
    fun riderWaiverOf(address: Int): RiderWaiver = setupOf(address).riderWaiver

    /**
     * What the arm control says about the rider requirement of the boards it arms ([armed]): the
     * strongest statement any of them makes, WAIVED over POSSIBLY over NONE, so a board running
     * waived is never hidden behind one that requires a rider (`specs/remote-ui.md` 3.4: the
     * statement is read from the boards the drive mode in force arms).
     */
    fun riderWaiver(armed: Collection<Int>): RiderWaiver {
        val each = armed.map { riderWaiverOf(it) }
        return when {
            RiderWaiver.WAIVED in each -> RiderWaiver.WAIVED
            RiderWaiver.POSSIBLY in each -> RiderWaiver.POSSIBLY
            else -> RiderWaiver.NONE
        }
    }
}

/**
 * What the Setup screen can ask of its model. One method per thing the screen offers, which is more
 * than detekt's interface threshold; splitting the screen's single contract to satisfy a count would
 * only scatter it.
 */
@Suppress("TooManyFunctions")
interface SetupActions {
    /** The screen came into view: read the target once per attached session. */
    fun onShown()

    /** The screen left view. */
    fun onHidden()

    /**
     * Edit the store of the node at [address], one the walk found. Picking a motor controller also
     * makes it the one SINGLE drives and Tune tunes ([SetupState.selectedMotor]).
     */
    fun selectTarget(address: Int)

    /** Read every field of the target again. */
    fun refresh()

    /**
     * Take [key] to [value] on the target board, which writes it once the settle window closes; a
     * machine field goes to every motor controller ([SetupField.machine]).
     */
    fun edit(key: Key, value: Value)

    /** Send [key]'s last edited value to the target board again, after a write of it failed. */
    fun retry(key: Key)

    /**
     * Set the machine's frame ([com.hoverboard.protocol.store.Fields.ControlFrame]) on EVERY motor
     * controller (`specs/remote-ui.md` 3.7a). The same value to each, so there is no order between
     * them.
     */
    fun chooseFrame(value: Int)

    /**
     * Make the board at [address] the right wheel and every other motor controller the left
     * (`specs/remote-ui.md` 3.7a). The others' writes go FIRST, so a sequence that stops between them
     * leaves no board on the right rather than two.
     */
    fun chooseRightWheel(address: Int)

    /**
     * Make the board at [address] the rigid frame's leader and clear every other motor
     * controller's flag (`specs/rigid-leader.md` section 7). The clears go FIRST, so a sequence that
     * stops between them leaves no board flagged rather than two; the pair is checked as one write.
     */
    fun chooseLeader(address: Int)

    /**
     * Write one whole frame: the three [signs] and the two [roles] (`specs/imu.md`, a pose writes
     * both fields). A role index whose stored value already resolves to the wanted role is left
     * alone rather than rewritten.
     */
    fun editFrame(roles: List<Int>, signs: List<Int>)

    /** Write the level trims that zero the current pitch and roll. */
    fun setLevel()

    /** Rotation check, step one: the board is level. */
    fun checkLevel()

    /** Rotation check, step two: the board is leaning forward. */
    fun checkForwardLean()

    /** Clear the notice. */
    fun dismissNotice()
}

/** One queued write: whose store (a node address), and which field of it. */
private data class Target(val node: Int, val key: Key)

/**
 * The Setup screen's model: a per-node store editor over `CONFIG_READ` / `CONFIG_WRITE`
 * (`specs/remote-ui.md` sections 2, 3.4, 3.6a and 3.7a), for any node the walk found (the node
 * selector). Each node's store is its own; the model keeps a [BoardSetup] per address and every
 * operation names the node it is for.
 *
 * It owns the app's node list ([SetupState.nodes]): on attach it asks every node what it is
 * (`IDENTIFY`) and reads its `device.name` before anything else, and only then what the motor
 * controllers store for the machine ([readRideFacts]).
 *
 * There is no Apply. Each field is written when it is edited, and the row reports what the board
 * said about that field alone ([FieldWrite]). What made a batch and a power-cycle look necessary is
 * gone: the arm path re-reads the value row (`specs/integration.md`, the arm-time re-read), so most
 * fields take effect at the next arm, and `boot_count` on the wire lets the app CONFIRM a restart
 * for the rest instead of asking the operator to assert one.
 *
 * Writes leave after a settle window ([EDIT_SETTLE_MS]) so a field edited twice inside it is sent
 * once, with the later value: the one request slot the config lane has would otherwise be fed a
 * write per stepper repeat, which Apply used to hide by batching.
 *
 * A machine field's write goes to EVERY motor controller (`specs/machine-settings.md` section 7), and
 * so does the right wheel's pair of side writes (3.7a). They are queued like any other edit, in the
 * order the machine has to take them, and the right wheel's write is held back if a left wheel's did
 * not land ([WriteState.Skipped]).
 *
 * Every operation runs one at a time (one read pass or one drain of the write queue, never both)
 * and never in the background: the config path is the firmware's deepest stack path (section 1.7).
 *
 * While [isArmed] no edit is sent at all: the board would refuse it (`CFG_ARMED`) and the screen
 * says so on every row instead of letting taps fail.
 */
@Suppress("TooManyFunctions") // implements SetupActions, plus the private steps each action needs
class SetupModel(
    private val transport: HoverboardTransport,
    private val scope: CoroutineScope,
    private val isArmed: () -> Boolean,
) : SetupActions {

    private val _state = MutableStateFlow(SetupState())
    val state: StateFlow<SetupState> = _state.asStateFlow()

    /** One operation at a time: a read pass or a drain of [queue]. */
    private val op = Mutex()

    private var visible = false

    /** The nodes whose read pass this attached session has had. */
    private val loaded = mutableSetOf<Int>()

    /** Whether this attached session still owes the ride-facts read ([readRideFacts]). */
    private var rideFactsDue = false

    /** Whether the machine fields are due to be read from each motor controller ([readMachineApart]). */
    private var machineDue = false

    /** Edited values not yet handed to the lane, in send order; the latest value per target wins. */
    private val queue = LinkedHashMap<Target, Value>()

    /** The writes each queued write must FOLLOW, for the one row that has an order ([chooseRightWheel]). */
    private val waitsFor = mutableMapOf<Target, List<Target>>()

    /** Bumped by every edit, so the settle window can tell it was disturbed. */
    private var edits = 0

    /** Whether the settle-and-drain loop is running. */
    private var draining = false

    /**
     * The boot tag ([TelemetryUi.bootTag]) last seen from each board, absent before any arrived. A
     * CHANGE in it is the restart that puts a boot-effect write into effect, which is the fact this
     * screen used to ask an operator to assert.
     */
    private val bootTag = mutableMapOf<Int, Int>()

    /** The sources the last telemetry came from: whom a re-walk probes from ([rewalk]). */
    private var heard: Set<Int> = emptySet()

    /** A re-walk for a source the node list does not hold, and the 3 s it holds off the next one. */
    private var unknownSourceWalk: Job? = null

    init {
        combine(transport.attachedBoard, transport.nodes, ::Pair)
            .onEach { (attached, nodes) -> onNodes(attached, nodes) }
            .launchIn(scope)
        transport.telemetry.onEach(::onTelemetry).launchIn(scope)
        // Boards switched on after the connect (`specs/remote-ui.md` 3.1, "Boards that come up after
        // the connect"): the walk runs once, so while the identity read has found no node that is,
        // or may be, a motor controller (a class not read is not taken for a bridge), the model
        // walks again every [REWALK_INTERVAL_MS].
        scope.launch {
            _state.map { s ->
                s.attached != null && s.rideFactsRead && s.nodes.none { it.isMotorController || it.nodeClass == null }
            }
                .distinctUntilChanged()
                .collectLatest { looking ->
                    while (looking) {
                        delay(REWALK_INTERVAL_MS)
                        rewalk()
                    }
                }
        }
    }

    /**
     * Walk again behind the attached node, from it and every motor controller that is reporting,
     * under the one-operation lock as the ride-facts read is; a node list that grows comes back
     * through [onNodes]. Never while armed, and not while another operation holds the lock: the
     * next cadence, or the next unknown source, tries again.
     */
    private suspend fun rewalk() {
        val s = _state.value
        val attached = s.attached ?: return
        if (isArmed() || !op.tryLock()) return
        try {
            transport.rewalk(listOf(attached) + s.motorControllers.filter { it in heard })
        } finally {
            op.unlock()
            next()
        }
    }

    /**
     * The session's nodes changed: a new session, the end of one, or a walk that found a different
     * set. Write records are kept by address across a drop, because a board keeps what it stored and
     * a node keeps its address from one session to the next; what was READ is dropped with the link.
     */
    private fun onNodes(attached: Int?, nodes: List<Int>) {
        val s = _state.value
        val session = if (attached == null) emptyList() else nodes
        if (attached == s.attached && session == s.addresses) return
        if (attached == null) {
            loaded.clear()
            rideFactsDue = false
            machineDue = false
            // The selection is frozen at what it resolved to, so the screen goes on showing the
            // node it was editing, and a reconnect to the same machine comes back to it.
            _state.update { st ->
                st.copy(
                    addresses = emptyList(),
                    attached = null,
                    rideFactsRead = false,
                    machineApart = false,
                    target = st.board ?: st.target,
                    motor = st.selectedMotor ?: st.motor,
                    boards = st.boards.mapValues { (_, b) -> b.dropped() },
                )
            }
            return
        }
        loaded.retainAll(session.toSet())
        rideFactsDue = true
        // A grown list (a re-walk found a board) is read again from the start: its facts are not in
        // until that read finishes, and the lamp test waits for them (`specs/remote-ui.md` 3.1b).
        _state.update { it.copy(addresses = session, attached = attached, rideFactsRead = false) }
        next()
    }

    /**
     * A board whose boot tag differs from the one last seen has restarted under the app. The boot
     * sequence reads every field, the arm-time row included, so every write this session is now in
     * effect and no record of one is left to report. What the board is RUNNING is then its store, so
     * the read pass runs again and the booted-with values are learned afresh.
     *
     * This is the whole of what the "I power-cycled the board" button used to do, taken from the
     * operator's word and given to the wire. Each board is judged by the telemetry its own address
     * keys (`specs/link-control.md`, the peer forward), so a restart of a board not on the
     * phone's own link is seen too.
     *
     * A missing tag is no observation (no state has arrived, or the board is quiet), never a change.
     * One byte wraps at 256 boots between two observations, which no bench session reaches.
     */
    private fun onTelemetry(reported: Map<Int, TelemetryUi>) {
        val s = _state.value
        heard = reported.keys
        // A frame from a source the walk did not find: a board that came up since, already holding
        // an address. Walk again, at most once in [REWALK_INTERVAL_MS].
        if (s.attached != null && reported.keys.any { it !in s.addresses } && unknownSourceWalk?.isActive != true) {
            unknownSourceWalk = scope.launch {
                rewalk()
                delay(REWALK_INTERVAL_MS)
            }
        }
        val restarted = s.addresses.count { node -> observeBoot(node, reported[node]?.bootTag) }
        observeFlags(reported)
        // A board finds the machine's boards in disagreement: read the machine fields from each
        // motor controller, once per session, so Setup can list the ones that differ.
        val apart = s.attached != null && !s.machineApart &&
            s.motorControllers.any { reported[it]?.machineDisagrees == true }
        if (apart) {
            _state.update { it.copy(machineApart = true) }
            machineDue = true
        }
        if (restarted > 0 || apart) next()
    }

    /** Fold each board's `FLAG_REARM` / `FLAG_REBOOT` in, where they changed. */
    private fun observeFlags(reported: Map<Int, TelemetryUi>) {
        for ((node, t) in reported.filterValues { it.hasState }) {
            val b = _state.value.boards[node]
            if (b == null || b.rearm != t.rearm || b.reboot != t.reboot) {
                update(node) { it.copy(rearm = t.rearm, reboot = t.reboot) }
            }
        }
    }

    /** Fold one board's boot tag in, true when it names a restart this app had not seen. */
    private fun observeBoot(node: Int, tag: Int?): Boolean {
        if (tag == null) return false
        val previous = bootTag.put(node, tag)
        if (previous == null || previous == tag) return false
        onRestarted(node)
        return true
    }

    private fun onRestarted(node: Int) {
        update(node) { b ->
            b.copy(writes = b.writes.filterValues { it.sending }, rotationCheck = RotationCheck())
        }
        // The arm control needs the facts the board now runs whether or not Setup is open, which is
        // what the ride-facts read is for.
        rideFactsDue = true
        loaded -= node
    }

    /** Fold [f] into the state of the node at [node]. */
    private fun update(node: Int, f: (BoardSetup) -> BoardSetup) = _state.update { s ->
        s.copy(boards = s.boards + (node to f(s.boards[node] ?: BoardSetup())))
    }

    private fun notice(n: SetupNotice?) = _state.update { it.copy(notice = n) }

    /**
     * The facts the app needs whether or not Setup is ever opened, read once per attached session
     * under the same one-operation lock as everything else, and before any read pass if both are
     * due. While another operation holds the lock (a drain still unwinding at re-attach), it stays
     * due and runs when that operation ends ([next]).
     *
     * First what every node IS: its `IDENTITY` (`specs/machine-settings.md` section 6.1) and its
     * `device.name` (`specs/remote-ui.md` section 2), which is the node list. Then, from the nodes
     * whose class says they are motor controllers, what the screens state before Setup is opened
     * ([SetupFields.MOTOR_FACTS]: the mode and frame the drive mode is derived from, the side the
     * arm control acts on (3.7a), the pack that turns the battery word into a state of charge (3.6b)
     * and the rider requirement the arm control states), the counts per amp and wheel geometry that
     * turn its telemetry words into amps and a distance, and its boot status record (`STATUS_READ`),
     * which Setup's machine status lists under the board (`specs/remote-ui.md` 3.1c). A machine
     * field is read once ([readFact]). A node whose identity has not answered is asked nothing more:
     * it shows its address alone, and it is asked again with the next pass.
     */
    private fun readRideFacts() {
        if (_state.value.attached == null) return
        if (!op.tryLock()) return
        rideFactsDue = false
        scope.launch {
            try {
                val nodes = _state.value.addresses
                for (node in nodes) {
                    identify(node)
                    // A node that did not answer is not asked its name: a silent node costs a
                    // whole retransmit budget per request, and it shows its address alone.
                    if (_state.value.nodeAt(node)?.nodeClass != null) readOne(node, SetupFields.DEVICE_NAME.key)
                }
                val mcs = _state.value.motorControllers
                for (field in SetupFields.MOTOR_FACTS) readFact(mcs, field.key)
                for (node in mcs) readCountsPerAmp(node)
                for (node in mcs) readWheelGeometry(node)
                for (node in mcs) readBoardStatus(node)
                if (_state.value.machineApart) readMachineKeys(mcs)
                // The Ride screen's lamp test runs once per connection, when this first completes
                // (`specs/remote-ui.md` 3.1b): the facts that decide which lamps apply are in.
                _state.update { it.copy(rideFactsRead = true) }
            } finally {
                op.unlock()
                next()
            }
        }
    }

    /**
     * Read [key] from the motor controllers [mcs]: a machine field from the first alone, standing for
     * every one ([fanOut]), until a board reports the copies disagree; any other field from each.
     */
    private suspend fun readFact(mcs: List<Int>, key: Key) {
        val once = isMachine(key) && !_state.value.machineApart
        for (node in if (once) mcs.take(1) else mcs) readOne(node, key)
    }

    /**
     * Read every machine key from each motor controller (`EV_MACHINE`, `specs/machine-settings.md`
     * section 7), so [machineDifferences] can list the fields that differ by board. A read that fails
     * leaves that board's copy as it was: no machine key outside Setup's rows has a row to report it.
     */
    private suspend fun readMachineKeys(mcs: List<Int>) {
        for (key in MACHINE_KEYS) {
            for (node in mcs.filter { it in _state.value.addresses }) {
                val r = awaitSlot { transport.readConfig(key, node) } ?: return
                if (r is ReadValue) update(node) { it.copy(values = it.values + (key to r.value)) }
            }
        }
    }

    /** Read the machine fields from each motor controller, once a board has reported they disagree. */
    private fun readMachineApart() {
        if (_state.value.attached == null) return
        if (!op.tryLock()) return
        machineDue = false
        scope.launch {
            try {
                readMachineKeys(_state.value.motorControllers)
            } finally {
                op.unlock()
                next()
            }
        }
    }

    /** Whether [key] is a machine field (`specs/machine-settings.md` section 2). */
    private fun isMachine(key: Key): Boolean = Scope.of(key.fieldId) == Scope.MACHINE

    /**
     * The nodes a value of [key] read from or written to [node] stands for: every motor controller
     * for a machine field while the copies are taken to agree (`specs/machine-settings.md` section
     * 7, "Setup reads the machine fields once"), else [node] alone.
     */
    private fun fanOut(node: Int, key: Key): List<Int> {
        val s = _state.value
        val mcs = s.motorControllers
        return if (isMachine(key) && !s.machineApart && node in mcs) mcs else listOf(node)
    }

    /** Ask [node] what it is, into [BoardSetup.identity]; nothing when this session does not reach it. */
    private suspend fun identify(node: Int) {
        if (node !in _state.value.addresses) return
        val r = awaitSlot { transport.identify(node) } ?: return
        if (r is Identified) update(node) { it.copy(identity = r.identity) }
    }

    /** Read [key] from the node at [node], or do nothing when this session does not reach it. */
    private suspend fun readOne(node: Int, key: Key) {
        if (node !in _state.value.addresses) return
        awaitSlot { transport.readConfig(key, node) }?.let { record(node, key, it) }
    }

    /**
     * Read [node]'s `motor.current_cal` into [BoardSetup.countsPerAmp].
     *
     * Not [readOne]: the field has no row on any screen, so recording it as a read value would put
     * a key the Setup screen cannot show into that screen's values, and a failed read would put
     * one into its unread set, where it would be counted against rows that do not exist. A read
     * that fails leaves the scale unknown, which the panel says rather than papering over with the
     * registry default: the constant is a property of the shunt and amplifier chain on THAT board.
     */
    private suspend fun readCountsPerAmp(node: Int) {
        val board = node.takeIf { it in _state.value.addresses } ?: return
        val r = awaitSlot { transport.readConfig(Fields.MOTOR_CURRENT_CAL.key(), board) } ?: return
        if (r is ReadValue) update(node) { it.copy(countsPerAmp = r.value.asLong()?.toInt()) }
    }

    /**
     * Read [node]'s pole-pair count and wheel diameter into [BoardSetup.polePairs] and
     * [BoardSetup.diameterMm], which is what lets the Travel section show a distance for it.
     *
     * Not [readOne], for [readCountsPerAmp]'s reason: neither field has a row on any screen, so
     * recording them as read values would put keys the Setup screen cannot show into that screen's
     * bookkeeping. **And there is no row by intent, not by omission.** Both are measured on the
     * machine (one full wheel revolution against the odometer's edge delta, divided by six; the
     * rolling diameter under load) and written at a bench with the host config tool. A typo here
     * is a confidently wrong distance on every screen that shows one, and nothing an operator can
     * read off a phone would tell them apart.
     *
     * A read that fails or a field that is 0 leaves the board unconvertible, which the Travel
     * section says rather than falling back on a believed fleet wheel.
     */
    private suspend fun readWheelGeometry(node: Int) {
        val board = node.takeIf { it in _state.value.addresses } ?: return
        val pp = awaitSlot { transport.readConfig(Fields.MOTOR_POLE_PAIRS.key(), board) }
        if (pp is ReadValue) update(node) { it.copy(polePairs = pp.value.asLong()?.toInt()) }
        val wd = awaitSlot { transport.readConfig(Fields.WHEEL_DIAMETER_MM.key(), board) }
        if (wd is ReadValue) update(node) { it.copy(diameterMm = wd.value.asLong()?.toInt()) }
    }

    /**
     * Read [node]'s boot status record (`STATUS_READ`) into [BoardSetup.status]: what its last boot
     * could not bring up. A request that is not answered leaves the status unread.
     */
    private suspend fun readBoardStatus(node: Int) {
        val board = node.takeIf { it in _state.value.addresses } ?: return
        val r = awaitSlot { transport.readStatus(board) } ?: return
        if (r is StatusEntries) update(node) { it.copy(status = r.entries) }
    }

    /** Start whatever operation is due, the ride facts first: called on attach and whenever the lock frees. */
    private fun next() {
        when {
            rideFactsDue -> readRideFacts()
            machineDue -> readMachineApart()
            else -> maybeRefresh()
        }
    }

    override fun onShown() {
        visible = true
        maybeRefresh()
    }

    override fun onHidden() {
        visible = false
    }

    override fun selectTarget(address: Int) {
        val node = _state.value.nodeAt(address) ?: return
        _state.update {
            it.copy(target = address, motor = if (node.isMotorController) address else it.motor, notice = null)
        }
        maybeRefresh()
    }

    /**
     * Pick the motor controller at [address] for the Tune screen and for SINGLE, without moving the
     * Setup screen off a node it is editing that is not one. The Tune selector's call.
     */
    fun selectMotor(address: Int) {
        if (_state.value.nodeAt(address)?.isMotorController != true) return
        _state.update {
            it.copy(target = if (it.target in it.motorControllers) address else it.target, motor = address)
        }
        maybeRefresh()
    }

    private fun maybeRefresh() {
        val s = _state.value
        val board = s.board ?: return
        if (visible && board !in loaded) refresh()
    }

    override fun refresh() {
        val s = _state.value
        val board = s.board ?: return
        if (!op.tryLock()) return
        loaded += board
        update(board) { it.copy(reading = true) }
        scope.launch {
            try {
                readAll(board)
                resolveWrites(board)
            } finally {
                update(board) { it.copy(reading = false) }
                op.unlock()
                next()
            }
        }
    }

    /**
     * Read every row the node shows, after asking what it is when that is not known yet: its class
     * decides the rows, so a node whose identity was unknown shows only the rows every node has
     * until it answers. A machine row read here stands for every motor controller ([fanOut]).
     */
    private suspend fun readAll(board: Int) {
        if (_state.value.nodeAt(board)?.nodeClass == null) identify(board)
        val rows = SetupFields.rowsFor(_state.value.nodeAt(board)?.nodeClass)
        for (field in rows) {
            if (board !in _state.value.addresses) return
            val r = awaitSlot { transport.readConfig(field.key, board) } ?: return
            record(board, field.key, r)
        }
        if (SetupFields.IMU_MODEL in rows) readImuBus(board)
    }

    /**
     * Read the IMU bus `imu.model` switches on into [BoardSetup.imuBus]: a key the board answers
     * with its default is not stored. Any other answer leaves the bus unread.
     */
    private suspend fun readImuBus(board: Int) {
        val bus = mutableMapOf<Key, Value>()
        for (key in IMU_BUS) {
            if (board !in _state.value.addresses) return
            val r = awaitSlot { transport.readConfig(key, board) } as? ReadValue ?: return
            if (!r.isDefault) bus[key] = r.value
        }
        update(board) { it.copy(imuBus = bus) }
    }

    /** Fold one read of [key] from [node] into every node its value stands for ([fanOut]). */
    private fun record(node: Int, key: Key, r: ConfigReadResult) {
        for (n in fanOut(node, key)) update(n) { recorded(it, key, r) }
    }

    private fun recorded(b: BoardSetup, key: Key, r: ConfigReadResult): BoardSetup = when (r) {
        is ReadValue -> b.copy(values = b.values + (key to r.value), unread = b.unread - key, garbled = b.garbled - key)
        // The board answered and the answer did not decode, which is not the same as silence and
        // must not be reported as it. A STR record the board no longer validates as text reaches
        // here, and so does a refusal code this app does not know.
        is Malformed -> b.copy(values = b.values - key, unread = b.unread - key, garbled = b.garbled + key)
        else -> b.copy(values = b.values - key, unread = b.unread + key, garbled = b.garbled - key)
    }

    /**
     * After a read pass: a write whose answer was lost, whose value the board turns out to hold,
     * landed. That is the case a link drop leaves behind, where no re-read was possible at the time.
     */
    private fun resolveWrites(node: Int) = update(node) { b ->
        val landed = b.writes.filter { (k, w) -> w.failed && b.values[k] == w.want }
        if (landed.isEmpty()) {
            b
        } else {
            b.copy(writes = b.writes + landed.mapValues { (_, w) -> w.copy(state = WriteState.Written) })
        }
    }

    override fun edit(key: Key, value: Value) {
        // A field with a flow editor is written only by the flow that owns it (`specs/imu.md`: the
        // orientation is never a bare picker), so a row's own edit never reaches one. One sign of a
        // rotation is not a value anything can accept on its own.
        val field = SetupFields.forKey(key) ?: return
        if (field.editor is Editor.Flow) return
        if (field.machine) return writeToMachine(key, value)
        send(_state.value.board ?: return, key, value)
    }

    override fun retry(key: Key) {
        val node = _state.value.board ?: return
        val write = _state.value.setupOf(node).writes[key] ?: return
        if (write.sending) return
        if (SetupFields.forKey(key)?.machine == true) return writeToMachine(key, write.want)
        // The one write a flow-owned field takes on its own: finishing a frame write that stopped.
        send(node, key, write.want)
    }

    /** Queue [value] for [node]'s [key] and get the drain going, if there is anything to send. */
    private fun send(node: Int, key: Key, value: Value) {
        if (isArmed()) return
        if (leaderRefused(mapOf(node to mapOf(key to value)))) return
        if (!enqueue(node, key, value)) return
        update(node) { it.copy(rotationCheck = checkAfterEdit(it, key)) }
        notice(null)
        startDraining()
    }

    /**
     * Queue [value] for [node]'s [key], true when something is now due to be sent.
     *
     * A value the board is known to hold is not written: tapping the chip a row already shows must
     * not append a flash record. If a failed write is on that row, taking it back to the stored
     * value drops the record instead, because there is nothing left to report.
     *
     * A re-edit moves the entry to the BACK of the queue rather than leaving it where it was, so
     * the send order is always the order the edits were last made in. The right wheel depends on
     * that: picking another board must not leave the previous pick's writes in their old order.
     *
     * The value checks (`specs/config-read-once.md` section 4: the range table and the IMU frame)
     * run first, on [node]'s values as [together] would leave them: the one write, or every write
     * of a frame that goes as one. The firmware applies what is stored, so a value a check refuses
     * is never queued. A refusal of a key this write does not touch (a value the board already
     * holds) does not stop it: each edit is its own transaction, and the refused value is the one
     * its own row's edit must fix. A frame goes as one, so all five of its keys are in [together]
     * and the frame check's refusal, which names the first sign or the first role, names one of them.
     */
    private fun enqueue(node: Int, key: Key, value: Value, together: Map<Key, Value> = mapOf(key to value)): Boolean {
        val field = SetupFields.forKey(key)?.takeIf { it.takesEffect != null } ?: return false
        val board = _state.value.setupOf(node)
        val refused = when {
            !field.accepts(value) -> SetupNotice.OutOfRange(key)
            key == SetupFields.IMU_MODEL.key -> imuGroupRefusal(board, key, value)
            else -> Checks.checkValues(board.values + together).firstOrNull { it.key in together }
                ?.let { SetupNotice.CheckRefused(it) }
        }
        if (refused != null) {
            notice(refused)
            return false
        }
        val target = Target(node, key)
        queue.remove(target)
        waitsFor.remove(target)
        if (board.values[key] != value) {
            queue[target] = value
            edits++
            update(node) { it.copy(writes = it.writes + (key to FieldWrite(value, WriteState.Sending))) }
            return true
        }
        // Taken back to what the board holds: nothing to send, and nothing left to report.
        if (board.writes[key] != null) update(node) { it.copy(writes = it.writes - key) }
        return false
    }

    /**
     * The group check of an `imu.model` write (`checks::check_pins`, the IMU group switched by
     * `imu.model`): a model with the bus incomplete is refused as the CLI refuses it, naming the
     * group's first pin. Model 0 (the IMU off) always passes; any other model with the bus unread is
     * refused ([SetupNotice.BusUnread]).
     */
    private fun imuGroupRefusal(board: BoardSetup, key: Key, value: Value): SetupNotice? {
        if (value == Value.U8(Fields.ImuModel.NONE)) return null
        val bus = board.imuBus ?: return SetupNotice.BusUnread(key)
        return Checks.checkGroups(bus + (key to value)).firstOrNull()?.let { SetupNotice.CheckRefused(it) }
    }

    /**
     * Wait out the settle window, then drain the queue under the one-operation lock, until nothing
     * is left to send.
     */
    private fun startDraining() {
        if (draining) return
        draining = true
        scope.launch {
            try {
                while (queue.isNotEmpty()) {
                    awaitSettle()
                    op.lock()
                    try {
                        drain()
                    } finally {
                        op.unlock()
                    }
                }
            } finally {
                draining = false
                next()
            }
        }
    }

    /**
     * Hold until no edit has arrived for [EDIT_SETTLE_MS]. A stepper held down keeps disturbing the
     * window, so it emits no write per repeat, and a field edited twice inside one window is sent
     * once with the later value (the queue holds the latest value per target).
     */
    private suspend fun awaitSettle() {
        var seen: Int
        do {
            seen = edits
            delay(EDIT_SETTLE_MS)
        } while (edits != seen)
    }

    /**
     * Write each queued field in turn, one at a time, verified against the stored value echoed.
     *
     * A field's failure does not stop the others: each edit is its own transaction. Two exceptions,
     * and both are about a state the pair must not be left in rather than about one field:
     * `CFG_ARMED`, which R4 answers to EVERY config write while the board is armed, so the rest of
     * the queue is reported refused rather than asked one at a time; and a write that had to follow
     * another ([waitsFor]), which is held back when the one it follows did not land.
     */
    private suspend fun drain() {
        while (queue.isNotEmpty()) {
            val target = queue.keys.first()
            val want = checkNotNull(queue.remove(target))
            val state = sendOne(target, want)
            mark(target, state)
            if (stopsTheQueue(state)) return dropQueue(state)
        }
    }

    /**
     * Send one queued write and say what its row reports. Nothing is sent for a write held back by
     * one it follows, nor for a node this session does not reach.
     */
    private suspend fun sendOne(target: Target, want: Value): WriteState {
        val follows = waitsFor.remove(target).orEmpty()
        if (!follows.all { landed(it) }) return WriteState.Skipped
        val board = target.node.takeIf { it in _state.value.addresses } ?: return WriteState.NotSent
        val r = awaitSlot { transport.writeConfig(target.key, want, board) } ?: return WriteState.NotSent
        return fold(target, want, r, board)
    }

    /**
     * Whether [state] ends the whole drain rather than this one row: `CFG_ARMED`, which R4 answers
     * to EVERY config write while the board is armed, and a session with no node attached, which
     * can send nothing at all. A node this session does not reach is NOT one of these: a Machine
     * write whose second board is powered off still applies to the one it reaches.
     */
    private fun stopsTheQueue(state: WriteState): Boolean =
        state == WriteState.Refused(CfgRefusal.ARMED) ||
            (state == WriteState.NotSent && _state.value.attached == null)

    /** Whether [target]'s write is in the store: the board echoed the value back. */
    private fun landed(target: Target): Boolean =
        _state.value.setupOf(target.node).writes[target.key]?.state == WriteState.Written

    /** Fold one write's answer into the state and say what the row reports. */
    private suspend fun fold(target: Target, want: Value, r: ConfigWriteResult, board: Int): WriteState {
        val (node, key) = target
        return when (r) {
            is WriteVerified -> {
                update(node) {
                    it.copy(
                        values = it.values + (key to r.stored),
                        unread = it.unread - key,
                        garbled = it.garbled - key,
                    )
                }
                WriteState.Written
            }
            is WriteMismatch -> {
                // The stored value is in doubt: read it, so the row shows what the board holds
                // rather than what the app hoped.
                reread(node, key, board)
                WriteState.Mismatch(r.stored)
            }
            is Refused -> WriteState.Refused(r.refusal)
            TimedOut -> {
                reread(node, key, board)
                // A write that landed with its answer lost is a write, not a failure.
                if (_state.value.setupOf(node).values[key] == want) WriteState.Written else WriteState.Unanswered
            }
            is Malformed -> WriteState.Garbled
            Busy -> WriteState.SlotBusy
        }
    }

    private suspend fun reread(node: Int, key: Key, board: Int) =
        awaitSlot { transport.readConfig(key, board) }?.let { record(node, key, it) }

    /** Report [state] on [target], keeping the value that was sent. */
    private fun mark(target: Target, state: WriteState) = update(target.node) { b ->
        val want = b.writes[target.key]?.want ?: return@update b
        b.copy(writes = b.writes + (target.key to FieldWrite(want, state)))
    }

    /** Report [state] on everything still queued, without sending any of it. */
    private fun dropQueue(state: WriteState) {
        val rest = queue.toMap()
        queue.clear()
        waitsFor.clear()
        for ((target, want) in rest) {
            update(target.node) { it.copy(writes = it.writes + (target.key to FieldWrite(want, state))) }
        }
    }

    private fun checkAfterEdit(b: BoardSetup, key: Key): RotationCheck =
        if (SetupFields.FRAME.any { it.key == key }) RotationCheck() else b.rotationCheck

    override fun chooseFrame(value: Int) = writeToMachine(SetupFields.CONTROL_FRAME.key, Value.U8(value))

    /**
     * [value] to the machine field [key] on every motor controller, in address order; there is no
     * order to keep (`specs/machine-settings.md` section 7).
     */
    private fun writeToMachine(key: Key, value: Value) {
        if (isArmed()) return
        if (leaderRefused(_state.value.motorControllers.associateWith { mapOf(key to value) })) return
        var due = false
        for (node in _state.value.motorControllers) due = enqueue(node, key, value) || due
        if (!due) return
        notice(null)
        startDraining()
    }

    /**
     * Motor 0's `side`: the LEFT wheel to every other motor controller first, the right wheel to the
     * chosen board last (`specs/remote-ui.md` 3.7a). The right-wheel write FOLLOWS the left ones, so
     * a left-wheel write that is refused or goes unanswered holds the right-wheel write back rather
     * than leaving the machine with two right wheels. With no other board in reach there is no order
     * to keep and the chosen board is written on its own.
     */
    override fun chooseLeader(address: Int) {
        if (isArmed()) return
        val mcs = _state.value.motorControllers
        if (address !in mcs) return
        val key = SetupFields.CONTROL_LEADER.key
        val flag = { a: Int -> Value.U8(if (a == address) 1 else 0) }
        if (leaderRefused(mcs.associateWith { mapOf(key to flag(it)) })) return
        val clears = mcs.filter { it != address }.filter { enqueue(it, key, flag(it)) }
        val setDue = enqueue(address, key, flag(address))
        if (clears.isNotEmpty() && setDue) waitsFor[Target(address, key)] = clears.map { Target(it, key) }
        if (clears.isEmpty() && !setDue) return
        notice(null)
        startDraining()
    }

    /**
     * The leader check (`specs/rigid-leader.md` section 6.1) on a write that moves `machine.mode`,
     * `machine.frame` or `control.leader`: every motor controller's three values as [writes] (each
     * board's writes by address) would leave them. True, with the notice set, when the write is
     * refused, or when a board's three have not been read ([SetupNotice.LeaderUnread]). A write of
     * no other key passes untouched.
     */
    private fun leaderRefused(writes: Map<Int, Map<Key, Value>>): Boolean {
        if (writes.values.none { w -> w.keys.any { Checks.changesLeader(it) } }) return false
        val s = _state.value
        val boards = s.motorControllers.map { a ->
            val stored = Checks.LEADER_KEYS.associateWith { s.setupOf(a).values[it] }
            val missing = stored.filterValues { it == null }.keys - writes[a].orEmpty().keys
            if (missing.isNotEmpty()) {
                notice(SetupNotice.LeaderUnread(a))
                return true
            }
            a to stored.mapNotNull { (k, v) -> v?.let { k to it } }.toMap() + writes[a].orEmpty()
        }
        val refusal = Checks.checkLeader(boards) ?: return false
        notice(SetupNotice.CheckRefused(refusal))
        return true
    }

    override fun chooseRightWheel(address: Int) {
        if (isArmed()) return
        val mcs = _state.value.motorControllers
        if (address !in mcs) return
        val key = SetupFields.CONTROL_SIDE.key
        val lefts = mcs.filter { it != address }.filter { enqueue(it, key, Value.U8(Fields.ControlSide.LEFT)) }
        val rightDue = enqueue(address, key, Value.U8(Fields.ControlSide.RIGHT))
        if (lefts.isNotEmpty() && rightDue) waitsFor[Target(address, key)] = lefts.map { Target(it, key) }
        if (lefts.isEmpty() && !rightDue) return
        notice(null)
        startDraining()
    }

    override fun editFrame(roles: List<Int>, signs: List<Int>) {
        if (isArmed()) return
        require(roles.size == SetupFields.AXIS_ROLE.size && signs.size == SetupFields.AXIS_SIGN.size)
        val node = _state.value.board ?: return
        // Unset and the compiled role run the same: when the stored roles already resolve to the
        // wanted ones, keep them, so the pair is not rewritten for nothing.
        val stored = _state.value.setupOf(node).storedRoles
        val keep = stored != null && Orientation.effectiveRoles(stored) == Orientation.effectiveRoles(roles)
        val target = if (keep) checkNotNull(stored) else roles
        // The whole frame is judged before any of it is queued. No write order avoids an illegal
        // intermediate, so the app refuses an illegal DESTINATION rather than a step of the way.
        Orientation.check(signs, target)?.let { refusal ->
            notice(SetupNotice.FrameRefused(refusal))
            return
        }
        val frame = SetupFields.AXIS_SIGN.mapIndexed { i, f -> f.key to Value.I32(signs[i]) } +
            SetupFields.AXIS_ROLE.mapIndexed { i, f -> f.key to Value.U8(target[i]) }
        var due = false
        for ((key, value) in frame) due = enqueue(node, key, value, together = frame.toMap()) || due
        if (!due) return
        update(node) { it.copy(rotationCheck = RotationCheck()) }
        notice(null)
        startDraining()
    }

    override fun setLevel() {
        if (isArmed()) return
        val s = _state.value
        val node = s.board ?: return
        val b = s.current
        // Pitch and roll come out of the frame the board is running, and the trim the board applies
        // is the one it booted with: a write of either means a reading computed now would be wrong.
        // They are the SELECTED board's own readings, by its address: either board can be the one
        // being levelled, and a trim computed from the other board's lean would be nonsense on both.
        val attitude = attitudeOf(s)
        val trim = SetupFields.LEVEL_TRIM.firstOrNull { b.pending(it) }
        val frame = SetupFields.FRAME.firstOrNull { b.pending(it) }
        val blocked = when {
            trim != null -> blockedBy(trim)
            frame != null -> blockedBy(frame)
            attitude == null -> Blocked.NO_TELEMETRY
            SetupFields.LEVEL_TRIM.any { b.values[it.key] !is Value.I16 } -> Blocked.NOT_READ
            else -> null
        }
        if (blocked != null || attitude == null) {
            notice(SetupNotice.LevelUnavailable(blocked ?: Blocked.NO_TELEMETRY))
            return
        }
        // The board publishes `smoothed - trim`, so the trim that zeroes this reading is the trim it
        // is running plus the reading. The running trim is the stored one: nothing was written.
        val readings = listOf(attitude.pitch, attitude.roll)
        SetupFields.LEVEL_TRIM.forEachIndexed { i, f ->
            val running = (b.values.getValue(f.key) as Value.I16).v
            val wanted = (running + readings[i]).coerceIn(Short.MIN_VALUE.toInt(), Short.MAX_VALUE.toInt())
            send(node, f.key, Value.I16(wanted))
        }
    }

    /** What a flow says while a write of [field] is not in effect ([BoardSetup.pending]): what the board waits for. */
    private fun blockedBy(field: SetupField): Blocked =
        if (field.takesEffect == Effect.ARM) Blocked.AWAITING_ARM else Blocked.AWAITING_RESTART

    override fun checkLevel() = runCheck { check, pitch, roll ->
        val level = abs(pitch) <= LEVEL_TOLERANCE_CENTIDEG && abs(roll) <= LEVEL_TOLERANCE_CENTIDEG
        check.copy(level = if (level) CheckResult.PASS else CheckResult.FAIL)
    }

    override fun checkForwardLean() = runCheck { check, pitch, _ ->
        val result = when {
            pitch <= -LEAN_MIN_CENTIDEG -> CheckResult.PASS
            pitch >= LEAN_MIN_CENTIDEG -> CheckResult.FAIL
            else -> CheckResult.INCONCLUSIVE
        }
        check.copy(forwardLean = result)
    }

    /**
     * Run one rotation-check step against the selected board's own attitude. Only meaningful when
     * the stored orientation is what that board runs, so it refuses while a frame index written
     * this session has not been taken up.
     */
    private fun runCheck(step: (RotationCheck, Int, Int) -> RotationCheck) {
        val s = _state.value
        val b = s.current
        val cyclic = attitudeOf(s)
        val frame = SetupFields.FRAME.firstOrNull { b.pending(it) }
        val blocked = when {
            frame != null -> blockedBy(frame)
            cyclic == null -> Blocked.NO_TELEMETRY
            b.storedSigns == null -> Blocked.NOT_READ
            else -> null
        }
        if (blocked != null || cyclic == null) {
            notice(SetupNotice.CheckUnavailable(blocked ?: Blocked.NO_TELEMETRY))
            return
        }
        val node = s.board ?: return
        update(node) { it.copy(rotationCheck = step(it.rotationCheck, cyclic.pitch, cyclic.roll)) }
        notice(null)
    }

    /**
     * The SELECTED board's last reported state, by its address (`specs/link-control.md`, "The
     * peer's state on the BLE port: the forward": each board's readings arrive under its own). Null
     * while that board has sent nothing or has gone quiet, and always for a bridge, which reports
     * none.
     */
    private fun attitudeOf(s: SetupState): CyclicState? =
        s.board?.let { transport.telemetry.value[it]?.cyclic }

    override fun dismissNotice() = notice(null)

    companion object {
        /**
         * The shortest a field is left alone before its value goes to the board: 200 ms, the same
         * budget the tune lane's drag cadence is set by ([TuneModel.DRAG_SEND_INTERVAL_MS], whose
         * 5 Hz is what the CC2541 bridge's metered UART leaves room for). One lane, one number.
         *
         * Setup uses it as a TRAILING window rather than a ceiling, which the Tune sliders need
         * because the board follows a drag live. Nothing here is followed live and every write
         * appends a flash record, so the value worth sending is the one the operator stopped on,
         * not the ones passed through on the way.
         */
        const val EDIT_SETTLE_MS = TuneModel.DRAG_SEND_INTERVAL_MS

        /** The bus `imu.model` switches on: its group's pins and instance key ([Checks.GROUPS]). */
        val IMU_BUS: List<Key> = Checks.GROUPS.first { it.switch == SetupFields.IMU_MODEL.key }
            .let { g -> g.pins.map { it.first.key } + listOfNotNull(g.instance?.first) }

        /** The level step passes with pitch and roll both within 10 degrees of zero. */
        const val LEVEL_TOLERANCE_CENTIDEG = 1_000

        /**
         * The forward-lean step judges the pitch sign only past 10 degrees: a lean that small is
         * clearly a lean, and a mirrored or wrong frame reads it with the opposite sign or not at all.
         */
        const val LEAN_MIN_CENTIDEG = 1_000

        /**
         * How often the model walks again while no motor controller is known, and the least time
         * between two re-walks for an unknown source (`specs/remote-ui.md` 3.1).
         */
        const val REWALK_INTERVAL_MS = 3_000L
    }
}
