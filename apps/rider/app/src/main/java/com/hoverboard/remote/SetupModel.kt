package com.hoverboard.remote

import com.hoverboard.protocol.config.Busy
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.ConfigReadResult
import com.hoverboard.protocol.config.ConfigWriteResult
import com.hoverboard.protocol.config.Malformed
import com.hoverboard.protocol.config.ReadValue
import com.hoverboard.protocol.config.Refused
import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.config.WriteMismatch
import com.hoverboard.protocol.config.WriteVerified
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.model.Editor
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TakesEffect
import com.hoverboard.remote.model.TelemetryUi
import com.hoverboard.remote.model.WheelGeometry
import com.hoverboard.remote.model.asLong
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlin.math.abs

/**
 * What became of one field's write (`specs/rider-ui.md` 3.6a: per-FIELD state, not one verdict).
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
     * The right wheel is the one user (`specs/rider-ui.md` 3.7a): `control.side` 0 goes to the
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

    /** "Set level" could not compute a trim. */
    data class LevelUnavailable(val reason: Blocked) : SetupNotice

    /** The rotation check could not run. */
    data class CheckUnavailable(val reason: Blocked) : SetupNotice

    /** A different board took a role: the write records belonged to [previous] and were dropped. */
    data class BoardChanged(val previous: Int) : SetupNotice
}

/** Why a telemetry-reading flow (set level, the rotation check) cannot run. */
enum class Blocked {
    /** The board being edited has reported nothing this session, or has gone quiet since. */
    NO_TELEMETRY,

    /** The stored value it builds on has not been read. */
    NOT_READ,

    /**
     * A field it depends on was written this session and the board re-reads that field when it
     * arms ([TakesEffect.NEXT_ARM]), so until it next arms the board is running the old value.
     */
    AWAITING_ARM,

    /**
     * A field it depends on was written this session and the board only reads that field at boot
     * ([TakesEffect.RESTART]), so until it restarts the board is running the old value.
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
     * The board is known to be running with `CONTROL_RIDER_REQUIRED` at 0: it booted with it and
     * its store still holds it, so no arm can change what it runs.
     */
    WAIVED,

    /**
     * What the board is running is unknown, and the value it last ran or the value it stores is 0:
     * it may be running waived. Two ways to get here, and the field being arm-time is the first:
     * once the stored value differs from the booted one, the board runs the booted value until it
     * next arms and the stored value from then on, and nothing it sends says which side of that it
     * is on. The second is a write followed by a link drop, where the app cannot tell whether the
     * board was power-cycled in the gap.
     */
    POSSIBLY,
}

/** The rotation check's two steps, each null until run. */
data class RotationCheck(val level: CheckResult? = null, val forwardLean: CheckResult? = null)

/**
 * One board's Setup state: its store as this app last read it, and what this app has written to it
 * (`specs/rider-ui.md` sections 3.4 and 3.6a). The screen shows one of these at a time
 * ([SetupState.current]); the model keeps one per [Node], because a write record made against the
 * master is not a statement about the slave.
 *
 * @param values the stored value of each key, as last read or verified-written.
 * @param running the value each key had when the board last booted, as far as this attached session
 *   knows: a read taken while the app had not written the key. Dropped with the link, and reset by
 *   a restart the wire confirmed.
 * @param lastRunning what [running] held when the link last dropped, for the keys it does not know
 *   again: the board may still be running those values (a power-cycle drops the link, but a drop is
 *   not a power-cycle). Cleared by a confirmed restart and by a different board.
 * @param unread keys whose last read failed.
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
 */
data class BoardSetup(
    val values: Map<Key, Value> = emptyMap(),
    val running: Map<Key, Value> = emptyMap(),
    val lastRunning: Map<Key, Value> = emptyMap(),
    val unread: Set<Key> = emptySet(),
    val reading: Boolean = false,
    val writes: Map<Key, FieldWrite> = emptyMap(),
    val rotationCheck: RotationCheck = RotationCheck(),
    val countsPerAmp: Int? = null,
    val polePairs: Int? = null,
    val diameterMm: Int? = null,
) {
    /** Whether any field's write is queued or in flight. */
    val writing: Boolean get() = writes.values.any { it.sending }

    val busy: Boolean get() = reading || writing

    /**
     * What this board's values say about the rider requirement (`CONTROL_RIDER_REQUIRED`,
     * `specs/control.md` (i)).
     *
     * Certain only while the booted value and the stored one AGREE: the field is re-read every
     * time the board arms, so once a write has moved the store apart from what the board booted
     * with, the board runs the booted value until its next arm and the stored value from then on.
     * The app sees neither the arm nor the value in anything the board sends, so it says
     * [RiderWaiver.POSSIBLY] whenever either side of that is 0, which also covers the other
     * unknown: a write followed by a link drop that may or may not have been a power-cycle.
     */
    val riderWaiver: RiderWaiver
        get() {
            val key = SetupFields.RIDER_REQUIRED.key
            val waived = Value.U8(Fields.RiderRequired.NOT_REQUIRED)
            val booted = running[key]
            if (booted != null && values[key] == booted) {
                return if (booted == waived) RiderWaiver.WAIVED else RiderWaiver.NONE
            }
            val maybe = booted == waived || lastRunning[key] == waived || values[key] == waived
            return if (maybe) RiderWaiver.POSSIBLY else RiderWaiver.NONE
        }

    /**
     * Whether the stored frame (the sign map read through the stored roles) is one the board will
     * refuse: it is not a proper rotation, or part of it is unread while a frame index was written
     * (a frame write stopped part way and the rest is in doubt).
     *
     * The firmware refuses such a frame and does not bring up the IMU (`specs/imu.md`, `BOARD_OBS`
     * result 11), so balance is lost until it is corrected. No write order avoids an illegal
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
     * Whether the stored orientation is what the board is running: no sign or role index was
     * written this session. The board re-reads the frame when it arms, which the app cannot see, so
     * a written frame stays unsettled for the session.
     */
    val orientationSettled: Boolean
        get() = SetupFields.FRAME.none { writes[it.key]?.mayHaveLanded == true }

    /** Whether any index of the IMU frame has a write queued or in flight. */
    val frameWriting: Boolean get() = SetupFields.FRAME.any { writes[it.key]?.sending == true }

    private fun ints(rows: List<SetupField>, of: (Key) -> Value?): List<Int>? =
        rows.map { (of(it.key)?.asLong() ?: return null).toInt() }

    /**
     * This board once the phone link has dropped: nothing read stands, and what was running may
     * still be ([lastRunning]). The write records stay: the board keeps what it stored, whether or
     * not the phone can still see it.
     */
    fun dropped(): BoardSetup = copy(
        values = emptyMap(),
        running = emptyMap(),
        lastRunning = lastRunning + running,
        unread = emptySet(),
        rotationCheck = RotationCheck(),
        countsPerAmp = null,
        polePairs = null,
        diameterMm = null,
    )
}

/**
 * The Setup screen's state (`specs/rider-ui.md` sections 3.4, 3.6a and 3.7a): which board is
 * selected, the pair's addresses this session, and each board's [BoardSetup].
 *
 * @param target which board the screen edits ([Node.MASTER], the attached one, or [Node.SLAVE],
 *   reached through it). The addresses are the session's, never assumed from the role: on one pair
 *   the master is node 0x01, on another 0x02.
 * @param master the attached board's address this session, or null while none is attached.
 * @param slave the slave's address from this session's discovery, or null when it found none.
 * @param boards each board's state, by role; a board the session has not touched has none yet.
 */
data class SetupState(
    val target: Node = Node.MASTER,
    val master: Int? = null,
    val slave: Int? = null,
    val boards: Map<Node, BoardSetup> = emptyMap(),
    val notice: SetupNotice? = null,
) {
    /** The address of [node] this session, or null while it is not reachable. */
    fun addressOf(node: Node): Int? = if (node == Node.MASTER) master else slave

    /** The target's address this session, or null while it is not reachable. */
    val board: Int? get() = addressOf(target)

    /** The target board's state. */
    val current: BoardSetup get() = boards[target] ?: BoardSetup()

    /** [node]'s state, empty for a board this session has not touched. */
    fun setupOf(node: Node): BoardSetup = boards[node] ?: BoardSetup()

    /**
     * [node]'s counts per amp (`motor.current_cal`), or null while this session has not read it.
     * The Ride screen's current display needs it per board, because the scale belongs to the
     * board's own sense chain rather than to the firmware.
     */
    fun countsPerAmpOf(node: Node): Int? = setupOf(node).countsPerAmp

    /**
     * [node]'s stored pack cell count (`board.pack_cells`), or null while this session has not
     * read it from that board. A stored 0 is passed through as the 0 it is: it means nobody has
     * said what pack this machine runs, and the Ride screen's gauge treats that as a pack it
     * cannot judge rather than as an eight-cell one.
     */
    fun packCellsOf(node: Node): Int? =
        setupOf(node).values[SetupFields.PACK_CELLS.key]?.asLong()?.toInt()

    /**
     * [node]'s wheel geometry, or null while it cannot convert: a board this session has not read
     * both facts from, and a board that has been told neither or only one of them. Per board,
     * because a machine may carry two different wheels, and because a figure converted against
     * the OTHER board's hub would be wrong with nothing to show it.
     */
    fun wheelGeometryOf(node: Node): WheelGeometry? =
        setupOf(node).let { WheelGeometry.of(it.polePairs, it.diameterMm) }

    val values: Map<Key, Value> get() = current.values
    val running: Map<Key, Value> get() = current.running
    val unread: Set<Key> get() = current.unread
    val reading: Boolean get() = current.reading
    val writes: Map<Key, FieldWrite> get() = current.writes
    val rotationCheck: RotationCheck get() = current.rotationCheck

    /**
     * Whether a write is queued or in flight to EITHER board. The Machine section writes to both
     * (`specs/rider-ui.md` 3.7a), so the screen's one busy statement is the pair's, not the
     * selected board's.
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

    /**
     * What the arm control says about the rider requirement: the MASTER's ([BoardSetup.riderWaiver]),
     * whichever board is selected, because the master is the board the arm control arms and the one
     * whose telemetry the Ride screen shows. Nothing while no board is attached.
     */
    val riderWaiver: RiderWaiver
        get() = if (master == null) RiderWaiver.NONE else setupOf(Node.MASTER).riderWaiver
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

    /** Edit [node]'s store (the slave only once this session's discovery found one). */
    fun selectTarget(node: Node)

    /** Read every field of the target again. */
    fun refresh()

    /** Take [key] to [value] on the target board, which writes it once the settle window closes. */
    fun edit(key: Key, value: Value)

    /** Send [key]'s last edited value to the target board again, after a write of it failed. */
    fun retry(key: Key)

    /**
     * Set the machine's frame ([com.hoverboard.protocol.store.Fields.ControlFrame]) on BOTH boards
     * (`specs/rider-ui.md` 3.7a). The same value to each, so there is no order between them.
     */
    fun chooseFrame(value: Int)

    /**
     * Make [node] the right wheel and the other board the left (`specs/rider-ui.md` 3.7a). The
     * other board's write goes FIRST, so a sequence that stops between the two leaves both boards
     * on the left rather than both on the right.
     */
    fun chooseRightWheel(node: Node)

    /**
     * Write one whole frame: the six [signs] and the two [roles] (`specs/imu.md`, a pose writes
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

/** One queued write: whose store, and which field of it. */
private data class Target(val node: Node, val key: Key)

/**
 * The Setup screen's model: a per-board store editor over `CONFIG_READ` / `CONFIG_WRITE`
 * (`specs/rider-ui.md` sections 3.4, 3.6a and 3.7a), for the master the phone is attached to or the
 * slave reached through it (the board selector). Each board's store is its own; the model keeps a
 * [BoardSetup] per [Node] and every operation names the board it is for, at the address the session
 * learned for it.
 *
 * There is no Apply. Each field is written when it is edited, and the row reports what the board
 * said about that field alone ([FieldWrite]). What made a batch and a power-cycle look necessary is
 * gone: the arm path re-reads the value row (`specs/integration.md`), so most fields take effect at
 * the next arm, and `boot_count` on the wire lets the app CONFIRM a restart for the rest instead of
 * asking the operator to assert one.
 *
 * Writes leave after a settle window ([EDIT_SETTLE_MS]) so a field edited twice inside it is sent
 * once, with the later value: the one request slot the config lane has would otherwise be fed a
 * write per stepper repeat, which Apply used to hide by batching.
 *
 * Two of the writes go to BOTH boards (the Machine section of 3.7a). They are queued like any other
 * edit, in the order the pair has to take them, and the right wheel's write is held back if the
 * left wheel's did not land ([WriteState.Skipped]).
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

    /** The boards whose read pass this attached session has had. */
    private val loaded = mutableSetOf<Node>()

    /** Whether this attached session still owes the ride-facts read ([readRideFacts]). */
    private var rideFactsDue = false

    /**
     * The address each role's write records belong to. Survives a link drop, unlike the addresses
     * in the state, so a reconnect to the same pair keeps them and a different board in the role
     * drops them.
     */
    private val recordBoard = mutableMapOf<Node, Int>()

    /** Edited values not yet handed to the lane, in send order; the latest value per target wins. */
    private val queue = LinkedHashMap<Target, Value>()

    /** The write each queued write must FOLLOW, for the one pair that has an order ([writeSide]). */
    private val waitsFor = mutableMapOf<Target, Target>()

    /** Bumped by every edit, so the settle window can tell it was disturbed. */
    private var edits = 0

    /** Whether the settle-and-drain loop is running. */
    private var draining = false

    /**
     * The boot tag ([TelemetryUi.bootTag]) last seen from each board, absent before any arrived. A
     * CHANGE in it is the restart that puts a [TakesEffect.RESTART] write into effect, which is the
     * fact this screen used to ask an operator to assert.
     */
    private val bootTag = mutableMapOf<Node, Int>()

    init {
        combine(transport.attachedBoard, transport.slaveBoard, ::Pair)
            .onEach { (m, s) -> onBoards(m, s) }
            .launchIn(scope)
        transport.telemetry.onEach(::onTelemetry).launchIn(scope)
    }

    private fun onBoards(master: Int?, slave: Int?) {
        val s = _state.value
        if (master == s.master && slave == s.slave) return
        if (master == null) {
            loaded.clear()
            rideFactsDue = false
            _state.update { st ->
                st.copy(master = null, slave = null, boards = st.boards.mapValues { (_, b) -> b.dropped() })
            }
            return
        }
        val boards = s.boards.toMutableMap()
        val changed = Node.entries.mapNotNull { node ->
            adopt(node, if (node == Node.MASTER) master else slave, s, boards)
        }
        // A session that found no slave has nothing to select it for.
        val target = if (slave == null && s.target == Node.SLAVE) Node.MASTER else s.target
        _state.update {
            it.copy(
                target = target,
                master = master,
                slave = slave,
                boards = boards,
                notice = changed.firstOrNull() ?: it.notice,
            )
        }
        next()
    }

    /**
     * [node] now answers at [now] (or nowhere): its read pass is owed again, and if a different
     * board took the role its state in [boards] is dropped, which the returned notice says.
     */
    private fun adopt(node: Node, now: Int?, s: SetupState, boards: MutableMap<Node, BoardSetup>): SetupNotice? {
        if (now == s.addressOf(node)) return null
        loaded -= node
        if (node == Node.MASTER) rideFactsDue = true
        // The slave going away while the master stays keeps its state: it may come back.
        if (now == null) return null
        val previous = recordBoard.put(node, now)
        if (previous == null || previous == now) return null
        boards[node] = BoardSetup()
        forget(node)
        return SetupNotice.BoardChanged(previous)
    }

    /** Drop everything held outside the state for [node]: a different board is in that role now. */
    private fun forget(node: Node) {
        queue.keys.removeAll { it.node == node }
        waitsFor.keys.removeAll { it.node == node }
        waitsFor.values.removeAll { it.node == node }
        bootTag -= node
    }

    /**
     * A board whose boot tag differs from the one last seen has restarted under the app. The boot
     * sequence reads every field, the arm-time row included, so every write this session is now in
     * effect and no record of one is left to report. What the board is RUNNING is then its store, so
     * the read pass runs again and the booted-with values are learned afresh.
     *
     * This is the whole of what the "I power-cycled the board" button used to do, taken from the
     * operator's word and given to the wire. Each board is judged by the telemetry its own address
     * keys (`specs/link-control.md`, the peer forward), so the slave's restart is seen too.
     *
     * A missing tag is no observation (no state has arrived, or the board is quiet), never a change.
     * One byte wraps at 256 boots between two observations, which no bench session reaches.
     */
    private fun onTelemetry(reported: Map<Int, TelemetryUi>) {
        val s = _state.value
        val restarted = Node.entries.count { node ->
            observeBoot(node, s.addressOf(node)?.let { reported[it]?.bootTag })
        }
        if (restarted > 0) next()
    }

    /** Fold one board's boot tag in, true when it names a restart this app had not seen. */
    private fun observeBoot(node: Node, tag: Int?): Boolean {
        if (tag == null) return false
        val previous = bootTag.put(node, tag)
        if (previous == null || previous == tag) return false
        onRestarted(node)
        return true
    }

    private fun onRestarted(node: Node) {
        update(node) { b ->
            b.copy(
                writes = b.writes.filterValues { it.sending },
                running = emptyMap(),
                lastRunning = emptyMap(),
                rotationCheck = RotationCheck(),
            )
        }
        // The arm control needs the facts the board now runs whether or not Setup is open, which is
        // what the ride-facts read is for.
        rideFactsDue = true
        loaded -= node
    }

    /** Fold [f] into [node]'s state. */
    private fun update(node: Node, f: (BoardSetup) -> BoardSetup) = _state.update { s ->
        s.copy(boards = s.boards + (node to f(s.boards[node] ?: BoardSetup())))
    }

    private fun notice(n: SetupNotice?) = _state.update { it.copy(notice = n) }

    /**
     * The fields the Ride screen needs whether or not Setup is ever opened, because what it shows
     * states what they come to: the master's rider requirement ([SetupState.riderWaiver]),
     * `control.frame` and `control.side` on BOTH boards (`specs/rider-ui.md` 3.7a, the frame
     * refusal and the side warning), the pack's cell count, which is what turns the battery word
     * into a state of charge (3.6b), and each board's counts per amp, which is what turns the
     * current words every `CYCLIC_STATE` carries into the amps the panel shows (3.6). Read once
     * per attached session, under the same one-operation lock as everything else, and before any
     * read pass if both are due. While another operation holds the lock (a drain still unwinding
     * at re-attach), it stays due and runs when that operation ends ([next]).
     */
    private fun readRideFacts() {
        if (_state.value.master == null) return
        if (!op.tryLock()) return
        rideFactsDue = false
        scope.launch {
            try {
                readOne(Node.MASTER, SetupFields.RIDER_REQUIRED.key)
                for (field in SetupFields.BOTH_BOARDS) for (node in Node.entries) readOne(node, field.key)
                for (node in Node.entries) readCountsPerAmp(node)
                for (node in Node.entries) readWheelGeometry(node)
            } finally {
                op.unlock()
                next()
            }
        }
    }

    /** Read [key] from [node], or do nothing when this session has no address for that board. */
    private suspend fun readOne(node: Node, key: Key) {
        val board = _state.value.addressOf(node) ?: return
        awaitSlot { transport.readConfig(key, board) }?.let { record(node, key, it) }
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
    private suspend fun readCountsPerAmp(node: Node) {
        val board = _state.value.addressOf(node) ?: return
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
    private suspend fun readWheelGeometry(node: Node) {
        val board = _state.value.addressOf(node) ?: return
        val pp = awaitSlot { transport.readConfig(Fields.MOTOR_POLE_PAIRS.key(), board) }
        if (pp is ReadValue) update(node) { it.copy(polePairs = pp.value.asLong()?.toInt()) }
        val wd = awaitSlot { transport.readConfig(Fields.WHEEL_DIAMETER_MM.key(), board) }
        if (wd is ReadValue) update(node) { it.copy(diameterMm = wd.value.asLong()?.toInt()) }
    }

    /** Start whatever operation is due, the ride facts first: called on attach and whenever the lock frees. */
    private fun next() {
        if (rideFactsDue) readRideFacts() else maybeRefresh()
    }

    override fun onShown() {
        visible = true
        maybeRefresh()
    }

    override fun onHidden() {
        visible = false
    }

    override fun selectTarget(node: Node) {
        if (node == Node.SLAVE && _state.value.slave == null) return
        _state.update { it.copy(target = node, notice = null) }
        maybeRefresh()
    }

    private fun maybeRefresh() {
        val s = _state.value
        if (visible && s.target !in loaded && s.board != null) refresh()
    }

    override fun refresh() {
        val s = _state.value
        val node = s.target
        val board = s.board ?: return
        if (!op.tryLock()) return
        loaded += node
        update(node) { it.copy(reading = true) }
        scope.launch {
            try {
                readAll(node, board)
                resolveWrites(node)
            } finally {
                update(node) { it.copy(reading = false) }
                op.unlock()
                next()
            }
        }
    }

    private suspend fun readAll(node: Node, board: Int) {
        for (field in SetupFields.ALL) {
            if (_state.value.addressOf(node) != board) return
            val r = awaitSlot { transport.readConfig(field.key, board) } ?: return
            record(node, field.key, r)
        }
    }

    private fun record(node: Node, key: Key, r: ConfigReadResult) = update(node) {
        if (r is ReadValue) {
            // A read of a key this session wrote is the store, not what the board booted with.
            val written = it.writes[key]?.mayHaveLanded == true
            val running = if (written) it.running else it.running + (key to r.value)
            it.copy(values = it.values + (key to r.value), running = running, unread = it.unread - key)
        } else {
            it.copy(values = it.values - key, unread = it.unread + key)
        }
    }

    /**
     * After a read pass: a write whose answer was lost, whose value the board turns out to hold,
     * landed. That is the case a link drop leaves behind, where no re-read was possible at the time.
     */
    private fun resolveWrites(node: Node) = update(node) { b ->
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
        if (SetupFields.forKey(key)?.editor is Editor.Flow) return
        send(_state.value.target, key, value)
    }

    override fun retry(key: Key) {
        val node = _state.value.target
        val write = _state.value.setupOf(node).writes[key] ?: return
        if (write.sending) return
        // The one write a flow-owned field takes on its own: finishing a frame write that stopped.
        send(node, key, write.want)
    }

    /** Queue [value] for [node]'s [key] and get the drain going, if there is anything to send. */
    private fun send(node: Node, key: Key, value: Value) {
        if (isArmed()) return
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
     * that: picking the other board must not leave the previous pick's two writes in their old
     * order.
     */
    private fun enqueue(node: Node, key: Key, value: Value): Boolean {
        val field = SetupFields.forKey(key)?.takeIf { it.takesEffect != null } ?: return false
        if (!field.accepts(value)) {
            notice(SetupNotice.OutOfRange(key))
            return false
        }
        val target = Target(node, key)
        queue.remove(target)
        waitsFor.remove(target)
        val board = _state.value.setupOf(node)
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
     * the one it follows, nor for a board this session has no address for.
     */
    private suspend fun sendOne(target: Target, want: Value): WriteState {
        val follows = waitsFor.remove(target)
        if (follows != null && !landed(follows)) return WriteState.Skipped
        val board = _state.value.addressOf(target.node) ?: return WriteState.NotSent
        val r = awaitSlot { transport.writeConfig(target.key, want, board) } ?: return WriteState.NotSent
        return fold(target, want, r, board)
    }

    /**
     * Whether [state] ends the whole drain rather than this one row: `CFG_ARMED`, which R4 answers
     * to EVERY config write while the board is armed, and a session with no board attached, which
     * can send nothing at all. A board this session has no address for is NOT one of these: a
     * Machine write whose second board is powered off still applies to the one it reaches.
     */
    private fun stopsTheQueue(state: WriteState): Boolean =
        state == WriteState.Refused(CfgRefusal.ARMED) ||
            (state == WriteState.NotSent && _state.value.master == null)

    /** Whether [target]'s write is in the store: the board echoed the value back. */
    private fun landed(target: Target): Boolean =
        _state.value.setupOf(target.node).writes[target.key]?.state == WriteState.Written

    /** Fold one write's answer into the state and say what the row reports. */
    private suspend fun fold(target: Target, want: Value, r: ConfigWriteResult, board: Int): WriteState {
        val (node, key) = target
        return when (r) {
            is WriteVerified -> {
                update(node) { it.copy(values = it.values + (key to r.stored), unread = it.unread - key) }
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

    private suspend fun reread(node: Node, key: Key, board: Int) =
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

    override fun chooseFrame(value: Int) {
        if (isArmed()) return
        val key = SetupFields.CONTROL_FRAME.key
        var due = false
        for (node in reachable()) due = enqueue(node, key, Value.U8(value)) || due
        if (!due) return
        notice(null)
        startDraining()
    }

    /**
     * `control.side`: the LEFT wheel to the other board first, the right wheel to the chosen board
     * second (`specs/rider-ui.md` 3.7a). The second write FOLLOWS the first, so a left-wheel write
     * that is refused or goes unanswered holds the right-wheel write back rather than leaving the
     * pair with two right wheels. Without the other board in reach there is no order to keep and
     * the chosen board is written on its own.
     */
    override fun chooseRightWheel(node: Node) {
        if (isArmed()) return
        if (_state.value.addressOf(node) == null) return
        val key = SetupFields.CONTROL_SIDE.key
        val other = if (node == Node.MASTER) Node.SLAVE else Node.MASTER
        val left = Target(other, key)
        val leftDue = other in reachable() && enqueue(other, key, Value.U8(Fields.ControlSide.LEFT))
        val rightDue = enqueue(node, key, Value.U8(Fields.ControlSide.RIGHT))
        if (leftDue && rightDue) waitsFor[Target(node, key)] = left
        if (!leftDue && !rightDue) return
        notice(null)
        startDraining()
    }

    /** The boards this session can reach, in write order: the master first. */
    private fun reachable(): List<Node> = Node.entries.filter { _state.value.addressOf(it) != null }

    override fun editFrame(roles: List<Int>, signs: List<Int>) {
        if (isArmed()) return
        require(roles.size == SetupFields.AXIS_ROLE.size && signs.size == SetupFields.AXIS_SIGN.size)
        val node = _state.value.target
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
        var due = false
        SetupFields.AXIS_SIGN.forEachIndexed { i, f -> due = enqueue(node, f.key, Value.I32(signs[i])) || due }
        SetupFields.AXIS_ROLE.forEachIndexed { i, f -> due = enqueue(node, f.key, Value.U8(target[i])) || due }
        if (!due) return
        update(node) { it.copy(rotationCheck = RotationCheck()) }
        notice(null)
        startDraining()
    }

    override fun setLevel() {
        if (isArmed()) return
        val s = _state.value
        val node = s.target
        val b = s.current
        // Pitch and roll come out of the frame the board is running, and the trim the board applies
        // is the one it booted with: a write of either means a reading computed now would be wrong.
        // They are the SELECTED board's own readings, by its address: either board can be the one
        // being levelled, and a trim computed from the other board's lean would be nonsense on both.
        val attitude = attitudeOf(s)
        val trim = notInEffect(b, SetupFields.LEVEL_TRIM)
        val frame = notInEffect(b, SetupFields.FRAME)
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

    /** The first of [rows] whose write may have landed, so the board is not running it yet. */
    private fun notInEffect(b: BoardSetup, rows: List<SetupField>): SetupField? =
        rows.firstOrNull { b.writes[it.key]?.mayHaveLanded == true }

    /**
     * What a flow says while a write of [field] is not in effect: when the board will start using
     * it. A null field, and a read-only row that cannot be written, cannot block a flow; both answer
     * with the boot case, which is the one that asks the operator for something.
     */
    private fun blockedBy(field: SetupField?): Blocked =
        if (field?.takesEffect == TakesEffect.NEXT_ARM) Blocked.AWAITING_ARM else Blocked.AWAITING_RESTART

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
        val frame = notInEffect(b, SetupFields.FRAME)
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
        update(s.target) { it.copy(rotationCheck = step(it.rotationCheck, cyclic.pitch, cyclic.roll)) }
        notice(null)
    }

    /**
     * The SELECTED board's last reported state, by the address this session learned for it
     * (`specs/link-control.md`, "The peer's state on the BLE port: the forward": the attached board
     * forwards its partner's state under the partner's address, so each board's readings arrive
     * under its own). Null while that board has sent nothing or has gone quiet.
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

        /** The level step passes with pitch and roll both within 10 degrees of zero. */
        const val LEVEL_TOLERANCE_CENTIDEG = 1_000

        /**
         * The forward-lean step judges the pitch sign only past 10 degrees: a lean that small is
         * clearly a lean, and a mirrored or wrong frame reads it with the opposite sign or not at all.
         */
        const val LEAN_MIN_CENTIDEG = 1_000
    }
}
