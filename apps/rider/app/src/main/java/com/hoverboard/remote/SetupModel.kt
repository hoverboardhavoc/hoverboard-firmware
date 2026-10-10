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
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.model.Editor
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TakesEffect
import com.hoverboard.remote.model.asLong
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
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
}

/**
 * One field's write this session: the value the app sent (or is about to) and what came of it.
 *
 * The record is what a row reports. It outlives a link drop, because the board keeps what it stored,
 * and a [TakesEffect.RESTART] record is dropped when the wire shows the board restarted.
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
     * A named refusal, a taken slot and an unattached board are writes that CERTAINLY did not land,
     * so for those the value last read is still what the board holds.
     */
    val mayHaveLanded: Boolean get() = when (state) {
        is WriteState.Refused, WriteState.SlotBusy, WriteState.NotSent -> false
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

    /** A different board attached: the write records belonged to [previous] and were dropped. */
    data class BoardChanged(val previous: Int) : SetupNotice
}

/** Why a telemetry-reading flow (set level, the rotation check) cannot run. */
enum class Blocked {
    /** No `CYCLIC_STATE` has arrived this session. */
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

    /** The board is known to be running with `CONTROL_RIDER_REQUIRED` at 0: it booted with it. */
    WAIVED,

    /**
     * What the board is running is unknown, and the value it last ran or the value it stores is 0:
     * it may be running waived (a write this session went in, then the link dropped, and the app
     * cannot tell whether the board was power-cycled in between).
     */
    POSSIBLY,
}

/** The rotation check's two steps, each null until run. */
data class RotationCheck(val level: CheckResult? = null, val forwardLean: CheckResult? = null)

/**
 * The Setup screen's state (`specs/rider-ui.md` sections 3.4 and 3.6a).
 *
 * @param board the target: the attached board's address, or null while none is attached.
 * @param values the stored value of each key, as last read or verified-written.
 * @param running the value each key had when the board last booted, as far as this attached session
 *   knows: a read taken while the app had not written the key. Dropped with the link, and reset by a
 *   restart the wire confirmed.
 * @param lastRunning what [running] held when the link last dropped, for the keys it does not know
 *   again: the board may still be running those values (a power-cycle drops the link, but a drop is
 *   not a power-cycle). Cleared by a confirmed restart and by a different board.
 * @param unread keys whose last read failed.
 * @param writes what the app wrote this session, per field, and what came of each ([FieldWrite]).
 */
data class SetupState(
    val board: Int? = null,
    val values: Map<Key, Value> = emptyMap(),
    val running: Map<Key, Value> = emptyMap(),
    val lastRunning: Map<Key, Value> = emptyMap(),
    val unread: Set<Key> = emptySet(),
    val reading: Boolean = false,
    val writes: Map<Key, FieldWrite> = emptyMap(),
    val rotationCheck: RotationCheck = RotationCheck(),
    val notice: SetupNotice? = null,
) {
    /** Whether any field's write is queued or in flight. */
    val writing: Boolean get() = writes.values.any { it.sending }

    val busy: Boolean get() = reading || writing

    /**
     * What the arm control says about the rider requirement (`CONTROL_RIDER_REQUIRED`,
     * `specs/control.md` (i)): arming a board that runs it waived in balance mode is the engage act.
     * [RiderWaiver.WAIVED] when this session read the value the board booted with and it is 0.
     * When that is unknown (a key the app wrote is not re-read as running), the board may still be
     * running the value it last ran, or already the stored one after a restart the app did not see,
     * so [RiderWaiver.POSSIBLY] if either is 0. Nothing while no board is attached.
     */
    val riderWaiver: RiderWaiver
        get() {
            if (board == null) return RiderWaiver.NONE
            val key = SetupFields.RIDER_REQUIRED.key
            val waived = Value.U8(Fields.RiderRequired.NOT_REQUIRED)
            running[key]?.let { return if (it == waived) RiderWaiver.WAIVED else RiderWaiver.NONE }
            return if (lastRunning[key] == waived || values[key] == waived) RiderWaiver.POSSIBLY else RiderWaiver.NONE
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
}

/**
 * What the Setup screen can ask of its model. One method per thing the screen offers, which is more
 * than detekt's interface threshold; splitting the screen's single contract to satisfy a count would
 * only scatter it.
 */
@Suppress("TooManyFunctions")
interface SetupActions {
    /** The screen came into view: read every field once per attached session. */
    fun onShown()

    /** The screen left view. */
    fun onHidden()

    /** Read every field again. */
    fun refresh()

    /** Take [key] to [value], which writes it once the settle window closes. */
    fun edit(key: Key, value: Value)

    /** Send [key]'s last edited value again, after a write of it failed. */
    fun retry(key: Key)

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

/**
 * The Setup screen's model: a per-board store editor over `CONFIG_READ` / `CONFIG_WRITE`
 * (`specs/rider-ui.md` sections 3.4 and 3.6a).
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

    /** Whether this attached session has had its read pass. */
    private var loaded = false

    /** Whether this attached session still owes the ride-facts read ([readRideFacts]). */
    private var rideFactsDue = false

    /** The board the write records belong to; survives a link drop, unlike [SetupState.board]. */
    private var writeBoard: Int? = null

    /** Edited values not yet handed to the lane, in edit order; the latest value per key wins. */
    private val queue = LinkedHashMap<Key, Value>()

    /** Bumped by every edit, so the settle window can tell it was disturbed. */
    private var edits = 0

    /** Whether the settle-and-drain loop is running. */
    private var draining = false

    /**
     * The boot tag ([com.hoverboard.remote.model.TelemetryUi.bootTag]) last seen from this board, or
     * null before any arrived. A CHANGE in it is the restart that puts a [TakesEffect.RESTART] write
     * into effect, which is the fact this screen used to ask an operator to assert.
     */
    private var bootTag: Int? = null

    init {
        transport.attachedBoard.onEach(::onAttachedBoard).launchIn(scope)
        transport.telemetry.onEach { t -> onBootTag(t?.bootTag) }.launchIn(scope)
    }

    /**
     * A board whose boot tag differs from the one last seen has restarted under the app. The boot
     * sequence reads every field, the arm-time row included, so every write this session is now in
     * effect and no record of one is left to report. What the board is RUNNING is then its store, so
     * the read pass runs again and the booted-with values are learned afresh.
     *
     * This is the whole of what the "I power-cycled the board" button used to do, taken from the
     * operator's word and given to the wire.
     *
     * A null tag is no observation (no state has arrived, or the link is down), never a change. One
     * byte wraps at 256 boots between two observations, which no bench session reaches.
     */
    private fun onBootTag(tag: Int?) {
        if (tag == null) return
        val previous = bootTag
        bootTag = tag
        if (previous == null || previous == tag) return
        _state.update { s ->
            s.copy(
                writes = s.writes.filterValues { it.sending },
                running = emptyMap(),
                lastRunning = emptyMap(),
                rotationCheck = RotationCheck(),
            )
        }
        // The arm control needs the rider requirement the board now runs whether or not Setup is
        // open, which is what the ride-facts read is for.
        rideFactsDue = true
        loaded = false
        next()
    }

    private fun onAttachedBoard(board: Int?) {
        loaded = false
        rideFactsDue = board != null
        if (board == null) {
            _state.update {
                it.copy(
                    board = null,
                    values = emptyMap(),
                    running = emptyMap(),
                    lastRunning = it.lastRunning + it.running,
                    unread = emptySet(),
                    rotationCheck = RotationCheck(),
                )
            }
            return
        }
        val previous = writeBoard
        writeBoard = board
        if (previous != null && previous != board) {
            queue.clear()
            bootTag = null
            _state.update { SetupState(board = board, notice = SetupNotice.BoardChanged(previous)) }
        } else {
            _state.update { it.copy(board = board) }
        }
        next()
    }

    /**
     * The one field the Ride screen needs whether or not Setup is ever opened: the rider requirement,
     * which the arm control states ([SetupState.riderWaiver]). Read once per attached session, under
     * the same one-operation lock as everything else, and before the Setup read pass if both are due.
     * While another operation holds the lock (a drain still unwinding at re-attach), it stays due and
     * runs when that operation ends ([next]).
     */
    private fun readRideFacts() {
        val board = _state.value.board ?: return
        if (!op.tryLock()) return
        rideFactsDue = false
        scope.launch {
            try {
                val key = SetupFields.RIDER_REQUIRED.key
                awaitSlot { transport.readConfig(key, board) }?.let { record(key, it) }
            } finally {
                op.unlock()
                next()
            }
        }
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

    private fun maybeRefresh() {
        if (visible && !loaded && _state.value.board != null) refresh()
    }

    override fun refresh() {
        val board = _state.value.board ?: return
        if (!op.tryLock()) return
        loaded = true
        _state.update { it.copy(reading = true) }
        scope.launch {
            try {
                readAll(board)
                resolveWrites()
            } finally {
                _state.update { it.copy(reading = false) }
                op.unlock()
                next()
            }
        }
    }

    private suspend fun readAll(board: Int) {
        for (field in SetupFields.ALL) {
            if (_state.value.board != board) return
            val r = awaitSlot { transport.readConfig(field.key, board) } ?: return
            record(field.key, r)
        }
    }

    private fun record(key: Key, r: ConfigReadResult) {
        _state.update {
            if (r is ReadValue) {
                // A read of a key this session wrote is the store, not what the board booted with.
                val written = it.writes[key]?.mayHaveLanded == true
                val running = if (written) it.running else it.running + (key to r.value)
                it.copy(values = it.values + (key to r.value), running = running, unread = it.unread - key)
            } else {
                it.copy(values = it.values - key, unread = it.unread + key)
            }
        }
    }

    /**
     * After a read pass: a write whose answer was lost, whose value the board turns out to hold,
     * landed. That is the case a link drop leaves behind, where no re-read was possible at the time.
     */
    private fun resolveWrites() = _state.update { s ->
        val landed = s.writes.filter { (k, w) -> w.failed && s.values[k] == w.want }
        if (landed.isEmpty()) {
            s
        } else {
            s.copy(writes = s.writes + landed.mapValues { (_, w) -> w.copy(state = WriteState.Written) })
        }
    }

    override fun edit(key: Key, value: Value) {
        // A field with a flow editor is written only by the flow that owns it (`specs/imu.md`: the
        // orientation is never a bare picker), so a row's own edit never reaches one. One sign of a
        // rotation is not a value anything can accept on its own.
        if (SetupFields.forKey(key)?.editor is Editor.Flow) return
        send(key, value)
    }

    override fun retry(key: Key) {
        val write = _state.value.writes[key] ?: return
        if (write.sending) return
        // The one write a flow-owned field takes on its own: finishing a frame write that stopped.
        send(key, write.want)
    }

    /** Queue [value] for [key] and get the drain going, if there is anything to send. */
    private fun send(key: Key, value: Value) {
        if (isArmed()) return
        if (!enqueue(key, value)) return
        _state.update { it.copy(notice = null, rotationCheck = checkAfterEdit(it, key)) }
        startDraining()
    }

    /**
     * Queue [value] for [key], true when something is now due to be sent.
     *
     * A value the board is known to hold is not written: tapping the chip a row already shows must
     * not append a flash record. If a failed write is on that row, taking it back to the stored
     * value drops the record instead, because there is nothing left to report.
     */
    private fun enqueue(key: Key, value: Value): Boolean {
        val field = SetupFields.forKey(key)?.takeIf { it.takesEffect != null } ?: return false
        if (!field.accepts(value)) {
            _state.update { it.copy(notice = SetupNotice.OutOfRange(key)) }
            return false
        }
        val s = _state.value
        if (s.values[key] != value) {
            queue[key] = value
            edits++
            _state.update { it.copy(writes = it.writes + (key to FieldWrite(value, WriteState.Sending))) }
            return true
        }
        // Taken back to what the board holds: nothing to send, and nothing left to report.
        queue.remove(key)
        if (s.writes[key] != null) _state.update { it.copy(writes = it.writes - key) }
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
     * once with the later value (the queue holds the latest value per key).
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
     * A field's failure does not stop the others: each edit is its own transaction. The exception is
     * `CFG_ARMED`, which R4 answers to EVERY config write while the board is armed, so the rest of
     * the queue is reported refused rather than asked one at a time.
     */
    private suspend fun drain() {
        while (queue.isNotEmpty()) {
            val key = queue.keys.first()
            val want = checkNotNull(queue.remove(key))
            val board = _state.value.board
            if (board == null) {
                mark(key, WriteState.NotSent)
                return dropQueue(WriteState.NotSent)
            }
            val r = awaitSlot { transport.writeConfig(key, want, board) }
            if (r == null) {
                mark(key, WriteState.NotSent)
                return dropQueue(WriteState.NotSent)
            }
            val state = fold(key, want, r, board)
            mark(key, state)
            if (state == WriteState.Refused(CfgRefusal.ARMED)) return dropQueue(state)
        }
    }

    /** Fold one write's answer into the state and say what the row reports. */
    private suspend fun fold(key: Key, want: Value, r: ConfigWriteResult, board: Int): WriteState = when (r) {
        is WriteVerified -> {
            _state.update { it.copy(values = it.values + (key to r.stored), unread = it.unread - key) }
            WriteState.Written
        }
        is WriteMismatch -> {
            // The stored value is in doubt: read it, so the row shows what the board holds rather
            // than what the app hoped.
            reread(key, board)
            WriteState.Mismatch(r.stored)
        }
        is Refused -> WriteState.Refused(r.refusal)
        TimedOut -> {
            reread(key, board)
            // A write that landed with its answer lost is a write, not a failure.
            if (_state.value.values[key] == want) WriteState.Written else WriteState.Unanswered
        }
        is Malformed -> WriteState.Garbled
        Busy -> WriteState.SlotBusy
    }

    private suspend fun reread(key: Key, board: Int) =
        awaitSlot { transport.readConfig(key, board) }?.let { record(key, it) }

    /** Report [state] on [key], keeping the value that was sent. */
    private fun mark(key: Key, state: WriteState) = _state.update { s ->
        val want = s.writes[key]?.want ?: return@update s
        s.copy(writes = s.writes + (key to FieldWrite(want, state)))
    }

    /** Report [state] on everything still queued, without sending any of it. */
    private fun dropQueue(state: WriteState) {
        val rest = queue.toMap()
        queue.clear()
        if (rest.isEmpty()) return
        _state.update { s ->
            s.copy(writes = s.writes + rest.mapValues { (_, want) -> FieldWrite(want, state) })
        }
    }

    private fun checkAfterEdit(s: SetupState, key: Key): RotationCheck =
        if (SetupFields.FRAME.any { it.key == key }) RotationCheck() else s.rotationCheck

    override fun editFrame(roles: List<Int>, signs: List<Int>) {
        if (isArmed()) return
        require(roles.size == SetupFields.AXIS_ROLE.size && signs.size == SetupFields.AXIS_SIGN.size)
        // Unset and the compiled role run the same: when the stored roles already resolve to the
        // wanted ones, keep them, so the pair is not rewritten for nothing.
        val stored = _state.value.storedRoles
        val keep = stored != null && Orientation.effectiveRoles(stored) == Orientation.effectiveRoles(roles)
        val target = if (keep) checkNotNull(stored) else roles
        // The whole frame is judged before any of it is queued. No write order avoids an illegal
        // intermediate, so the app refuses an illegal DESTINATION rather than a step of the way.
        Orientation.check(signs, target)?.let { refusal ->
            _state.update { it.copy(notice = SetupNotice.FrameRefused(refusal)) }
            return
        }
        var due = false
        SetupFields.AXIS_SIGN.forEachIndexed { i, f -> due = enqueue(f.key, Value.I32(signs[i])) || due }
        SetupFields.AXIS_ROLE.forEachIndexed { i, f -> due = enqueue(f.key, Value.U8(target[i])) || due }
        if (!due) return
        _state.update { it.copy(notice = null, rotationCheck = RotationCheck()) }
        startDraining()
    }

    override fun setLevel() {
        if (isArmed()) return
        val s = _state.value
        // Pitch and roll come out of the frame the board is running, and the trim the board applies
        // is the one it booted with: a write of either means a reading computed now would be wrong.
        val trim = notInEffect(s, SetupFields.LEVEL_TRIM)
        val frame = notInEffect(s, SetupFields.FRAME)
        val blocked = when {
            trim != null -> blockedBy(trim)
            frame != null -> blockedBy(frame)
            transport.telemetry.value?.cyclic == null -> Blocked.NO_TELEMETRY
            SetupFields.LEVEL_TRIM.any { s.values[it.key] !is Value.I16 } -> Blocked.NOT_READ
            else -> null
        }
        if (blocked != null) {
            _state.update { it.copy(notice = SetupNotice.LevelUnavailable(blocked)) }
            return
        }
        val cyclic = checkNotNull(transport.telemetry.value?.cyclic)
        // The board publishes `smoothed - trim`, so the trim that zeroes this reading is the trim it
        // is running plus the reading. The running trim is the stored one: nothing was written.
        val readings = listOf(cyclic.pitch, cyclic.roll)
        SetupFields.LEVEL_TRIM.forEachIndexed { i, f ->
            val running = (s.values.getValue(f.key) as Value.I16).v
            val trim = (running + readings[i]).coerceIn(Short.MIN_VALUE.toInt(), Short.MAX_VALUE.toInt())
            send(f.key, Value.I16(trim))
        }
    }

    /** The first of [rows] whose write may have landed, so the board is not running it yet. */
    private fun notInEffect(s: SetupState, rows: List<SetupField>): SetupField? =
        rows.firstOrNull { s.writes[it.key]?.mayHaveLanded == true }

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
     * Run one rotation-check step against the current telemetry. Only meaningful when the stored
     * orientation is what the board runs, so it refuses while a frame index written this session has
     * not been taken up.
     */
    private fun runCheck(step: (RotationCheck, Int, Int) -> RotationCheck) {
        val s = _state.value
        val cyclic = transport.telemetry.value?.cyclic
        val frame = notInEffect(s, SetupFields.FRAME)
        val blocked = when {
            frame != null -> blockedBy(frame)
            cyclic == null -> Blocked.NO_TELEMETRY
            s.storedSigns == null -> Blocked.NOT_READ
            else -> null
        }
        if (blocked != null || cyclic == null) {
            _state.update { it.copy(notice = SetupNotice.CheckUnavailable(blocked ?: Blocked.NO_TELEMETRY)) }
            return
        }
        _state.update { it.copy(rotationCheck = step(it.rotationCheck, cyclic.pitch, cyclic.roll), notice = null) }
    }

    override fun dismissNotice() = _state.update { it.copy(notice = null) }

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
