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
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.asLong
import kotlinx.coroutines.CoroutineScope
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

/** What the last Setup operation came to, for the screen to say. */
sealed interface SetupNotice {
    /** The app refused the edit itself: the board is armed, and every config write would be refused. */
    data object ReadOnlyWhileArmed : SetupNotice

    /** The board answered a write to [key] with a named refusal (`CFG_ARMED` among them). */
    data class BoardRefused(val key: Key, val refusal: CfgRefusal) : SetupNotice

    /** The board accepted the write to [key] but echoes a different stored value (or none). */
    data class Mismatch(val key: Key, val wrote: Value, val stored: Value?) : SetupNotice

    /** The write to [key] went unanswered through the whole retransmit budget. */
    data class Unanswered(val key: Key) : SetupNotice

    /** The board answered the write to [key] with something the wire contract does not define. */
    data class Garbled(val key: Key) : SetupNotice

    /**
     * The session's one request slot stayed taken by the Tune model's operations through the whole
     * wait ([awaitSlot]), so nothing was sent. Not an answer from the board; applying again is.
     */
    data object SlotBusy : SetupNotice

    /** No board is attached, so nothing was sent. */
    data object NotAttached : SetupNotice

    /** The staged frame (signs and roles) is not a proper rotation, so nothing was written. */
    data class FrameRefused(val refusal: Orientation.Refusal) : SetupNotice

    /** A sign or role index has neither a stored nor a staged value, so the frame cannot be checked. */
    data object FrameUnknown : SetupNotice

    /** The value is outside what the field takes. */
    data class OutOfRange(val key: Key) : SetupNotice

    /** "Set level" could not compute a trim. */
    data class LevelUnavailable(val reason: Blocked) : SetupNotice

    /** The rotation check could not run. */
    data class CheckUnavailable(val reason: Blocked) : SetupNotice

    /** A different board took a role: the basket and the staged marks belonged to [previous] and were dropped. */
    data class BoardChanged(val previous: Int) : SetupNotice
}

/** Why a telemetry-reading flow (set level, the rotation check) cannot run. */
enum class Blocked {
    /** No `CYCLIC_STATE` has arrived this session. */
    NO_TELEMETRY,

    /** The stored value it builds on has not been read. */
    NOT_READ,

    /**
     * A field it depends on (its own, or the orientation its readings come through) has a value
     * pending or staged but not yet applied by a power-cycle, so the board is running a value the
     * app cannot know.
     */
    NOT_APPLIED,

    /**
     * The target is the slave, whose attitude never reaches the app: telemetry is master-only
     * (`specs/rider-ui.md` section 2), so the readings on hand are the other board's.
     */
    NOT_REPORTED,
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
 * One board's Setup state: its store as this app last read it, and what this app has done to it
 * (`specs/rider-ui.md` section 3.4). The screen shows one of these at a time ([SetupState.current]);
 * the model keeps one per [Node], because a basket built for the master must not be written to the
 * slave, and a staged mark on the slave must survive the operator looking at the master.
 *
 * @param values the stored value of each key, as last read or verified-written. Never a staged one.
 * @param running the value each key had when the board last booted, as far as this attached session
 *   knows: a read taken while the key was neither staged nor holding its pending value. A write does
 *   not change it (no field applies live); a power-cycle does, which is why it is dropped with the
 *   link.
 * @param lastRunning what [running] held when the link last dropped, for the keys it does not know
 *   again: the board may still be running those values (a power-cycle drops the link, but a drop is
 *   not a power-cycle). Cleared by a confirmed power-cycle and by a different board.
 * @param unread keys whose last read failed.
 * @param pending the PENDING basket: edits made in the app and not yet written, in edit order.
 * @param staged written AND verified this session but not yet applied: the firmware reads every
 *   field once at boot, so these run only after a power-cycle. Derived from the app having written
 *   them, never from anything the board says.
 * @param linkDroppedSinceApply whether the link has dropped since the last Apply finished. A
 *   power-cycle of the MASTER takes the Bluetooth module down with it, so without a drop there
 *   cannot have been one; the slave's power-cycle is invisible to the phone link, and this is not
 *   consulted for it ([SetupState.canConfirmPowerCycle]).
 */
data class BoardSetup(
    val values: Map<Key, Value> = emptyMap(),
    val running: Map<Key, Value> = emptyMap(),
    val lastRunning: Map<Key, Value> = emptyMap(),
    val unread: Set<Key> = emptySet(),
    val reading: Boolean = false,
    val applying: Boolean = false,
    val pending: Map<Key, Value> = emptyMap(),
    val staged: Map<Key, Value> = emptyMap(),
    val linkDroppedSinceApply: Boolean = false,
    val rotationCheck: RotationCheck = RotationCheck(),
) {
    val busy: Boolean get() = reading || applying

    /**
     * What this board's values say about the rider requirement (`CONTROL_RIDER_REQUIRED`,
     * `specs/control.md` (i)). [RiderWaiver.WAIVED] when this session read the value the board booted
     * with and it is 0. When that is unknown (a key staged this session is not re-read as running),
     * the board may still be running the value it last ran, or already the stored one after a
     * power-cycle the app did not see, so [RiderWaiver.POSSIBLY] if either is 0.
     */
    val riderWaiver: RiderWaiver
        get() {
            val key = SetupFields.RIDER_REQUIRED.key
            val waived = Value.U8(Fields.RiderRequired.NOT_REQUIRED)
            running[key]?.let { return if (it == waived) RiderWaiver.WAIVED else RiderWaiver.NONE }
            return if (lastRunning[key] == waived || values[key] == waived) RiderWaiver.POSSIBLY else RiderWaiver.NONE
        }

    /**
     * The one power-cycle instruction: shown once the whole batch verified, nothing is pending, and
     * the stored frame is safe to boot ([storedFrameUnsafe]).
     */
    val awaitingPowerCycle: Boolean get() = pending.isEmpty() && staged.isNotEmpty() && !storedFrameUnsafe

    /** Staged changes wait on a power-cycle, but the stored frame must not be booted yet. */
    val powerCycleHeld: Boolean get() = staged.isNotEmpty() && storedFrameUnsafe

    /**
     * Whether the stored frame (the sign map read through the stored roles) must not be booted: it
     * is not a proper rotation, or part of it is unread while a frame index is staged (an Apply wrote
     * part of a frame and the rest is in doubt). The firmware refuses such a frame at boot and does
     * not bring the IMU up (`specs/imu.md`, `BOARD_OBS` result 11), so balance is lost until it is
     * corrected. No write order avoids an illegal intermediate: any two of the four flat rotations
     * differ in two signs of a triple, and the writes go one at a time, stopping at the first
     * failure. So a partial Apply can leave one stored, and only completing it makes it safe. A
     * board whose stored frame was already illegal before any Apply is held the same way.
     */
    val storedFrameUnsafe: Boolean
        get() {
            val signs = storedSigns
            val roles = storedRoles
            if (signs != null && roles != null) return Orientation.check(signs, roles) != null
            return SetupFields.FRAME.any { it.key in staged }
        }

    /** The stored sign map, or null while any index is unread. */
    val storedSigns: List<Int>? get() = ints(SetupFields.AXIS_SIGN) { values[it] }

    /** The stored axis roles `[UP, PITCH_RATE]`, or null while either index is unread. */
    val storedRoles: List<Int>? get() = ints(SetupFields.AXIS_ROLE) { values[it] }

    /** The map the board will hold once the basket is written: pending over stored, or null if unknown. */
    val intendedSigns: List<Int>? get() = ints(SetupFields.AXIS_SIGN) { pending[it] ?: values[it] }

    /** The roles the board will hold once the basket is written, or null if unknown. */
    val intendedRoles: List<Int>? get() = ints(SetupFields.AXIS_ROLE) { pending[it] ?: values[it] }

    /**
     * The pose the stored fields make the board run once booted (`Orientation.poseOf`), or null while
     * either field is unread, any sign is unset (the fallback is no picked pose), or the stored frame
     * is none of the 24.
     */
    val storedPose: Orientation.Pose? get() = pose(storedRoles, storedSigns)

    /** The pose the board will hold once the basket is written, or null as for [storedPose]. */
    val intendedPose: Orientation.Pose? get() = pose(intendedRoles, intendedSigns)

    private fun pose(roles: List<Int>?, signs: List<Int>?): Orientation.Pose? =
        if (roles == null || signs == null) null else Orientation.poseOf(roles, signs)

    /** Whether the stored orientation is what the board runs: no sign or role index pending or staged. */
    val orientationSettled: Boolean
        get() = SetupFields.FRAME.none { it.key in pending || it.key in staged }

    private fun ints(rows: List<SetupField>, of: (Key) -> Value?): List<Int>? =
        rows.map { (of(it.key)?.asLong() ?: return null).toInt() }

    /**
     * This board once the phone link has dropped: nothing read stands, what was running may still
     * be ([lastRunning]), and a batch that was staged or mid-write may have been followed by the
     * power-cycle the drop can be.
     */
    fun dropped(): BoardSetup = copy(
        values = emptyMap(),
        running = emptyMap(),
        lastRunning = lastRunning + running,
        unread = emptySet(),
        rotationCheck = RotationCheck(),
        linkDroppedSinceApply = linkDroppedSinceApply || staged.isNotEmpty() || applying,
    )
}

/**
 * The Setup screen's state (`specs/rider-ui.md` section 3.4): which board is selected, the pair's
 * addresses this session, and each board's [BoardSetup].
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

    val values: Map<Key, Value> get() = current.values
    val running: Map<Key, Value> get() = current.running
    val lastRunning: Map<Key, Value> get() = current.lastRunning
    val unread: Set<Key> get() = current.unread
    val reading: Boolean get() = current.reading
    val applying: Boolean get() = current.applying
    val pending: Map<Key, Value> get() = current.pending
    val staged: Map<Key, Value> get() = current.staged
    val linkDroppedSinceApply: Boolean get() = current.linkDroppedSinceApply
    val rotationCheck: RotationCheck get() = current.rotationCheck
    val busy: Boolean get() = current.busy
    val awaitingPowerCycle: Boolean get() = current.awaitingPowerCycle
    val powerCycleHeld: Boolean get() = current.powerCycleHeld
    val storedFrameUnsafe: Boolean get() = current.storedFrameUnsafe
    val storedSigns: List<Int>? get() = current.storedSigns
    val storedRoles: List<Int>? get() = current.storedRoles
    val intendedSigns: List<Int>? get() = current.intendedSigns
    val intendedRoles: List<Int>? get() = current.intendedRoles
    val storedPose: Orientation.Pose? get() = current.storedPose
    val intendedPose: Orientation.Pose? get() = current.intendedPose
    val orientationSettled: Boolean get() = current.orientationSettled

    /**
     * What the arm control says about the rider requirement: the MASTER's ([BoardSetup.riderWaiver]),
     * whichever board is selected, because the master is the board the arm control arms and the one
     * whose telemetry the Ride screen shows. Nothing while no board is attached.
     */
    val riderWaiver: RiderWaiver
        get() = if (master == null) RiderWaiver.NONE else (boards[Node.MASTER] ?: BoardSetup()).riderWaiver

    /**
     * Whether the operator's "I power-cycled the board" is accepted for the target now. For the
     * master, only once the phone link has dropped since the Apply: its power-cycle takes the
     * Bluetooth module down, so a link that never dropped rules one out. The slave's power-cycle
     * leaves the phone link to the master untouched, so there is nothing the app could check against
     * and the operator's word is taken.
     */
    val canConfirmPowerCycle: Boolean
        get() = current.awaitingPowerCycle && !current.busy && (target == Node.SLAVE || current.linkDroppedSinceApply)
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

    /** Put [value] for [key] in the target's basket (or take the key out, if [value] is what is stored). */
    fun stage(key: Key, value: Value)

    /** Take [key] out of the basket. */
    fun discard(key: Key)

    /** Empty the basket. */
    fun discardAll()

    /** Write every basket entry, one at a time, each verified against the stored value echoed. */
    fun apply()

    /**
     * Put one whole frame in the basket: the six [signs] and the two [roles] (`specs/rider-ui.md`
     * 3.4, a pose writes both fields). A role index whose stored value already resolves to the
     * wanted role is left as stored rather than rewritten.
     */
    fun stageFrame(roles: List<Int>, signs: List<Int>)

    /** Put the level trims that zero the current pitch and roll in the basket. */
    fun setLevel()

    /** Rotation check, step one: the board is level. */
    fun checkLevel()

    /** Rotation check, step two: the board is leaning forward. */
    fun checkForwardLean()

    /** The operator power-cycled the target: its staged marks go and its fields are read again. */
    fun confirmPowerCycled()

    /** Clear the notice. */
    fun dismissNotice()
}

/**
 * The Setup screen's model: a per-board store editor over `CONFIG_READ` / `CONFIG_WRITE`
 * (`specs/rider-ui.md` section 3.4), for the master the phone is attached to or the slave reached
 * through it (section 3.7, the board selector). Each board's store is its own; the model keeps a
 * [BoardSetup] per [Node] and every operation names the board it is for, at the address the session
 * learned for it.
 *
 * Edits accumulate in a PENDING basket and nothing is written until [apply], which writes each entry
 * through the transport's config client, one at a time, verified against the stored value the board
 * echoes. A verified entry moves to the staged set, which is all it is: the firmware reads every
 * field once at boot, so the screen shows one power-cycle instruction for the batch and never says a
 * value is running.
 *
 * Every operation runs one at a time (one read pass or one apply, never both) and never in the
 * background: the config path is the firmware's deepest stack path (section 1.7).
 *
 * While [isArmed], every edit and every write is refused here, before anything is sent. The board
 * would refuse them anyway (`CFG_ARMED`); the screen says so instead of letting taps fail.
 */
@Suppress("TooManyFunctions") // implements SetupActions, plus the private steps each action needs
class SetupModel(
    private val transport: HoverboardTransport,
    private val scope: CoroutineScope,
    private val isArmed: () -> Boolean,
) : SetupActions {

    private val _state = MutableStateFlow(SetupState())
    val state: StateFlow<SetupState> = _state.asStateFlow()

    /** One operation at a time: a read pass or an apply. */
    private val op = Mutex()

    private var visible = false

    /** The boards whose read pass this attached session has had. */
    private val loaded = mutableSetOf<Node>()

    /** Whether this attached session still owes the ride-facts read ([readRideFacts]). */
    private var rideFactsDue = false

    /**
     * The address each role's basket and staged marks belong to. Survives a link drop, unlike the
     * addresses in the state, so a reconnect to the same pair keeps them and a different board in
     * the role drops them.
     */
    private val basketBoard = mutableMapOf<Node, Int>()

    init {
        combine(transport.attachedBoard, transport.slaveBoard, ::Pair)
            .onEach { (m, s) -> onBoards(m, s) }
            .launchIn(scope)
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
        val previous = basketBoard.put(node, now)
        if (previous == null || previous == now) return null
        boards[node] = BoardSetup()
        return SetupNotice.BoardChanged(previous)
    }

    /** Fold [f] into [node]'s state. */
    private fun update(node: Node, f: (BoardSetup) -> BoardSetup) = _state.update { s ->
        s.copy(boards = s.boards + (node to f(s.boards[node] ?: BoardSetup())))
    }

    private fun notice(n: SetupNotice?) = _state.update { it.copy(notice = n) }

    /**
     * The one field the Ride screen needs whether or not Setup is ever opened: the master's rider
     * requirement, which the arm control states ([SetupState.riderWaiver]). Read once per attached
     * session, under the same one-operation lock as everything else, and before any read pass if
     * both are due. While another operation holds the lock (a previous session's Apply still
     * unwinding at re-attach), it stays due and runs when that operation ends ([next]).
     */
    private fun readRideFacts() {
        val board = _state.value.master ?: return
        if (!op.tryLock()) return
        rideFactsDue = false
        scope.launch {
            try {
                val key = SetupFields.RIDER_REQUIRED.key
                awaitSlot { transport.readConfig(key, board) }?.let { record(Node.MASTER, key, it) }
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
                resolvePending(node)
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
            // A read of a key this session wrote is the store, not what the board booted with:
            // staged, or holding the basket's value (a write that landed with its answer lost,
            // which [resolvePending] stages after a read pass).
            val written = key in it.staged || it.pending[key] == r.value
            val running = if (written) it.running else it.running + (key to r.value)
            it.copy(values = it.values + (key to r.value), running = running, unread = it.unread - key)
        } else {
            it.copy(values = it.values - key, unread = it.unread + key)
        }
    }

    /**
     * After a read pass: a basket entry the board already stores was written before a link drop cut
     * the Apply short, so it is staged, not re-written (`specs/rider-ui.md` 3.4, "re-writes only
     * the diff").
     */
    private fun resolvePending(node: Node) = update(node) { b ->
        val landed = b.pending.filter { (k, v) -> b.values[k] == v }
        if (landed.isEmpty()) b else b.copy(pending = b.pending - landed.keys, staged = b.staged + landed)
    }

    override fun stage(key: Key, value: Value) {
        if (refuseWhileArmed()) return
        val field = SetupFields.forKey(key) ?: return
        if (!field.accepts(value)) {
            notice(SetupNotice.OutOfRange(key))
            return
        }
        val node = _state.value.target
        update(node) { b ->
            val pending = if (b.values[key] == value) b.pending - key else b.pending + (key to value)
            b.copy(pending = pending, rotationCheck = checkAfterEdit(b, key))
        }
        notice(null)
    }

    override fun discard(key: Key) = update(_state.value.target) {
        it.copy(pending = it.pending - key, rotationCheck = checkAfterEdit(it, key))
    }

    override fun discardAll() = update(_state.value.target) {
        it.copy(pending = emptyMap(), rotationCheck = RotationCheck())
    }

    private fun checkAfterEdit(b: BoardSetup, key: Key): RotationCheck =
        if (SetupFields.FRAME.any { it.key == key }) RotationCheck() else b.rotationCheck

    override fun stageFrame(roles: List<Int>, signs: List<Int>) {
        if (refuseWhileArmed()) return
        require(roles.size == SetupFields.AXIS_ROLE.size && signs.size == SetupFields.AXIS_SIGN.size)
        SetupFields.AXIS_SIGN.forEachIndexed { i, f -> stage(f.key, Value.I32(signs[i])) }
        // Unset and the compiled role run the same: when the stored roles already resolve to the
        // wanted ones, keep them (staging the stored value takes the key out of the basket).
        val stored = _state.value.storedRoles
        val keep = stored != null && Orientation.effectiveRoles(stored) == Orientation.effectiveRoles(roles)
        val target = if (keep) checkNotNull(stored) else roles
        SetupFields.AXIS_ROLE.forEachIndexed { i, f -> stage(f.key, Value.U8(target[i])) }
    }

    override fun apply() {
        if (refuseWhileArmed()) return
        val s = _state.value
        val node = s.target
        val refusal = when {
            s.board == null -> SetupNotice.NotAttached
            else -> frameRefusal(s.current)
        }
        if (refusal != null) {
            notice(refusal)
            return
        }
        val board = s.board ?: return
        if (s.pending.isEmpty() || !op.tryLock()) return
        update(node) { it.copy(applying = true) }
        notice(null)
        scope.launch {
            try {
                writeAll(node, board)
            } finally {
                update(node) { it.copy(applying = false) }
                op.unlock()
                next()
            }
        }
    }

    /**
     * The whole-frame check (`specs/rider-ui.md` 3.4, D7), run whenever a sign or role index is
     * pending OR staged: after a partial Apply the stored frame can be illegal with nothing pending,
     * and an Apply of any other field must not carry on as if the frame were whole. The signs are
     * judged through the roles the board will hold, never under assumed default roles.
     */
    private fun frameRefusal(b: BoardSetup): SetupNotice? {
        if (SetupFields.FRAME.none { it.key in b.pending || it.key in b.staged }) return null
        val signs = b.intendedSigns ?: return SetupNotice.FrameUnknown
        val roles = b.intendedRoles ?: return SetupNotice.FrameUnknown
        return Orientation.check(signs, roles)?.let { SetupNotice.FrameRefused(it) }
    }

    private suspend fun writeAll(node: Node, board: Int) {
        val pending = (_state.value.boards[node] ?: BoardSetup()).pending.toList()
        for ((key, want) in pending) {
            if (refuseWhileArmed()) return
            val r = awaitSlot { transport.writeConfig(key, want, board) }
            if (r == null) {
                notice(SetupNotice.NotAttached)
                return
            }
            if (!settle(node, key, want, r, board)) return
        }
        update(node) { it.copy(linkDroppedSinceApply = false) }
    }

    /** Fold one write's result into [node]'s state; false when the Apply must stop here. */
    private suspend fun settle(node: Node, key: Key, want: Value, r: ConfigWriteResult, board: Int): Boolean {
        val n = when (r) {
            is WriteVerified -> {
                update(node) {
                    it.copy(
                        pending = it.pending - key,
                        staged = it.staged + (key to r.stored),
                        values = it.values + (key to r.stored),
                    )
                }
                return true
            }
            is WriteMismatch -> SetupNotice.Mismatch(key, want, r.stored)
            is Refused -> SetupNotice.BoardRefused(key, r.refusal)
            TimedOut -> SetupNotice.Unanswered(key)
            is Malformed -> SetupNotice.Garbled(key)
            Busy -> SetupNotice.SlotBusy
        }
        notice(n)
        // A mismatch or a timeout leaves the stored value in doubt: read it, so the screen shows
        // what the board holds rather than what the app hoped.
        if (r is WriteMismatch || r == TimedOut) {
            awaitSlot { transport.readConfig(key, board) }?.let { record(node, key, it) }
        }
        return false
    }

    override fun setLevel() {
        if (refuseWhileArmed()) return
        val s = _state.value
        val b = s.current
        // Pitch and roll come out of the frame the board booted with: a pending or staged orientation
        // changes them at the power-cycle, so a trim computed now would be wrong afterwards. And they
        // are the master's: the slave's attitude is not on the wire, so a trim computed for it from
        // the readings on hand would be the other board's.
        val blocked = when {
            s.target == Node.SLAVE -> Blocked.NOT_REPORTED
            SetupFields.LEVEL_TRIM.any { it.key in b.pending || it.key in b.staged } -> Blocked.NOT_APPLIED
            !b.orientationSettled -> Blocked.NOT_APPLIED
            transport.telemetry.value?.cyclic == null -> Blocked.NO_TELEMETRY
            SetupFields.LEVEL_TRIM.any { b.values[it.key] !is Value.I16 } -> Blocked.NOT_READ
            else -> null
        }
        if (blocked != null) {
            notice(SetupNotice.LevelUnavailable(blocked))
            return
        }
        val cyclic = checkNotNull(transport.telemetry.value?.cyclic)
        // The board publishes `smoothed - trim`, so the trim that zeroes this reading is the trim it
        // is running plus the reading. The running trim is the stored one: nothing is staged.
        val readings = listOf(cyclic.pitch, cyclic.roll)
        SetupFields.LEVEL_TRIM.forEachIndexed { i, f ->
            val running = (b.values.getValue(f.key) as Value.I16).v
            val trim = (running + readings[i]).coerceIn(Short.MIN_VALUE.toInt(), Short.MAX_VALUE.toInt())
            stage(f.key, Value.I16(trim))
        }
    }

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
     * orientation is what the board runs, so it refuses while a sign or role index is pending or
     * staged; and only for the master, whose attitude the telemetry is.
     */
    private fun runCheck(step: (RotationCheck, Int, Int) -> RotationCheck) {
        val s = _state.value
        val b = s.current
        val cyclic = transport.telemetry.value?.cyclic
        val blocked = when {
            s.target == Node.SLAVE -> Blocked.NOT_REPORTED
            !b.orientationSettled -> Blocked.NOT_APPLIED
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

    override fun confirmPowerCycled() {
        val s = _state.value
        if (!s.canConfirmPowerCycle) return
        update(s.target) {
            it.copy(
                staged = emptyMap(),
                running = emptyMap(),
                lastRunning = emptyMap(),
                linkDroppedSinceApply = false,
                rotationCheck = RotationCheck(),
            )
        }
        loaded -= s.target
        maybeRefresh()
    }

    override fun dismissNotice() = notice(null)

    private fun refuseWhileArmed(): Boolean {
        if (!isArmed()) return false
        notice(SetupNotice.ReadOnlyWhileArmed)
        return true
    }

    companion object {
        /** The level step passes with pitch and roll both within 10 degrees of zero. */
        const val LEVEL_TOLERANCE_CENTIDEG = 1_000

        /**
         * The forward-lean step judges the pitch sign only past 10 degrees: a lean that small is
         * clearly a lean, and a mirrored or wrong frame reads it with the opposite sign or not at all.
         */
        const val LEAN_MIN_CENTIDEG = 1_000
    }
}
