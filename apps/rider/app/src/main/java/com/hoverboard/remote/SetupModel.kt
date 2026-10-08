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
import com.hoverboard.remote.model.SetupField
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.asLong
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
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

    /**
     * The board answered the write to [key] with something the wire contract does not define, or
     * the client did not send it because another config operation was in flight (which cannot
     * happen while this model is the client's only caller).
     */
    data class Garbled(val key: Key) : SetupNotice

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

    /** A different board attached: the basket and the staged marks belonged to [previous] and were dropped. */
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
 * The Setup screen's state (`specs/rider-ui.md` section 3.4).
 *
 * @param board the target: the attached board's address, or null while none is attached.
 * @param values the stored value of each key, as last read or verified-written. Never a staged one.
 * @param running the value each key had when the board last booted, as far as this attached session
 *   knows: a read taken while the key was not staged. A write does not change it (no field applies
 *   live); a power-cycle does, which is why it is dropped with the link.
 * @param lastRunning what [running] held when the link last dropped, for the keys it does not know
 *   again: the board may still be running those values (a power-cycle drops the link, but a drop is
 *   not a power-cycle). Cleared by a confirmed power-cycle and by a different board.
 * @param unread keys whose last read failed.
 * @param pending the PENDING basket: edits made in the app and not yet written, in edit order.
 * @param staged written AND verified this session but not yet applied: the firmware reads every
 *   field once at boot, so these run only after a power-cycle. Derived from the app having written
 *   them, never from anything the board says.
 * @param linkDroppedSinceApply whether the link has dropped since the last Apply finished. A
 *   power-cycle takes the Bluetooth module down with the board, so without a drop there cannot
 *   have been one.
 */
data class SetupState(
    val board: Int? = null,
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
    val notice: SetupNotice? = null,
) {
    val busy: Boolean get() = reading || applying

    /**
     * What the arm control says about the rider requirement (`CONTROL_RIDER_REQUIRED`,
     * `specs/control.md` (i)): arming a board that runs it waived in balance mode is the engage act.
     * [RiderWaiver.WAIVED] when this session read the value the board booted with and it is 0.
     * When that is unknown (a key staged this session is not re-read as running), the board may
     * still be running the value it last ran, or already the stored one after a power-cycle the app
     * did not see, so [RiderWaiver.POSSIBLY] if either is 0. Nothing while no board is attached.
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

    /** Whether the stored orientation is what the board runs: no sign or role index pending or staged. */
    val orientationSettled: Boolean
        get() = SetupFields.FRAME.none { it.key in pending || it.key in staged }

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

    /** Put [value] for [key] in the basket (or take the key out, if [value] is what is stored). */
    fun stage(key: Key, value: Value)

    /** Take [key] out of the basket. */
    fun discard(key: Key)

    /** Empty the basket. */
    fun discardAll()

    /** Write every basket entry, one at a time, each verified against the stored value echoed. */
    fun apply()

    /**
     * Put one whole frame in the basket: the six [signs] and the two [roles] (`specs/rider-ui.md`
     * 3.4, a preset writes both fields). A role index whose stored value already resolves to the
     * wanted role is left as stored rather than rewritten.
     */
    fun stageFrame(roles: List<Int>, signs: List<Int>)

    /** Put the level trims that zero the current pitch and roll in the basket. */
    fun setLevel()

    /** Rotation check, step one: the board is level. */
    fun checkLevel()

    /** Rotation check, step two: the board is leaning forward. */
    fun checkForwardLean()

    /** The operator power-cycled the board: the staged marks go and the fields are read again. */
    fun confirmPowerCycled()

    /** Clear the notice. */
    fun dismissNotice()
}

/**
 * The Setup screen's model: a per-board store editor over `CONFIG_READ` / `CONFIG_WRITE`
 * (`specs/rider-ui.md` section 3.4).
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

    /** Whether this attached session has had its read pass. */
    private var loaded = false

    /** Whether this attached session still owes the ride-facts read ([readRideFacts]). */
    private var rideFactsDue = false

    /** The board the basket and the staged marks belong to; survives a link drop, unlike [SetupState.board]. */
    private var basketBoard: Int? = null

    init {
        transport.attachedBoard.onEach(::onAttachedBoard).launchIn(scope)
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
                    linkDroppedSinceApply = it.linkDroppedSinceApply || it.staged.isNotEmpty() || it.applying,
                )
            }
            return
        }
        val previous = basketBoard
        basketBoard = board
        _state.update {
            if (previous != null && previous != board) {
                SetupState(board = board, notice = SetupNotice.BoardChanged(previous))
            } else {
                it.copy(board = board)
            }
        }
        next()
    }

    /**
     * The one field the Ride screen needs whether or not Setup is ever opened: the rider requirement,
     * which the arm control states ([SetupState.riderWaiver]). Read once per attached session, under
     * the same one-operation lock as everything else, and before the Setup read pass if both are due.
     * While another operation holds the lock (a previous session's Apply still unwinding at
     * re-attach), it stays due and runs when that operation ends ([next]).
     */
    private fun readRideFacts() {
        val board = _state.value.board ?: return
        if (!op.tryLock()) return
        rideFactsDue = false
        scope.launch {
            try {
                val key = SetupFields.RIDER_REQUIRED.key
                transport.readConfig(key, board)?.let { record(key, it) }
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
                resolvePending()
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
            val r = transport.readConfig(field.key, board) ?: return
            record(field.key, r)
        }
    }

    private fun record(key: Key, r: ConfigReadResult) {
        _state.update {
            if (r is ReadValue) {
                // A read of a key this session wrote is the store, not what the board booted with.
                val running = if (key in it.staged) it.running else it.running + (key to r.value)
                it.copy(values = it.values + (key to r.value), running = running, unread = it.unread - key)
            } else {
                it.copy(values = it.values - key, unread = it.unread + key)
            }
        }
    }

    /**
     * After a read pass: a basket entry the board already stores was written before a link drop cut
     * the Apply short, so it is staged, not re-written (`specs/rider-ui.md` 3.4, "re-writes only
     * the diff").
     */
    private fun resolvePending() = _state.update { s ->
        val landed = s.pending.filter { (k, v) -> s.values[k] == v }
        if (landed.isEmpty()) s else s.copy(pending = s.pending - landed.keys, staged = s.staged + landed)
    }

    override fun stage(key: Key, value: Value) {
        if (refuseWhileArmed()) return
        val field = SetupFields.forKey(key) ?: return
        if (!field.accepts(value)) {
            _state.update { it.copy(notice = SetupNotice.OutOfRange(key)) }
            return
        }
        _state.update { s ->
            val pending = if (s.values[key] == value) s.pending - key else s.pending + (key to value)
            s.copy(pending = pending, notice = null, rotationCheck = checkAfterEdit(s, key))
        }
    }

    override fun discard(key: Key) = _state.update {
        it.copy(pending = it.pending - key, rotationCheck = checkAfterEdit(it, key))
    }

    override fun discardAll() = _state.update { it.copy(pending = emptyMap(), rotationCheck = RotationCheck()) }

    private fun checkAfterEdit(s: SetupState, key: Key): RotationCheck =
        if (SetupFields.FRAME.any { it.key == key }) RotationCheck() else s.rotationCheck

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
        val refusal = when {
            s.board == null -> SetupNotice.NotAttached
            else -> frameRefusal(s)
        }
        if (refusal != null) {
            _state.update { it.copy(notice = refusal) }
            return
        }
        val board = s.board ?: return
        if (s.pending.isEmpty() || !op.tryLock()) return
        _state.update { it.copy(applying = true, notice = null) }
        scope.launch {
            try {
                writeAll(board)
            } finally {
                _state.update { it.copy(applying = false) }
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
    private fun frameRefusal(s: SetupState): SetupNotice? {
        if (SetupFields.FRAME.none { it.key in s.pending || it.key in s.staged }) return null
        val signs = s.intendedSigns ?: return SetupNotice.FrameUnknown
        val roles = s.intendedRoles ?: return SetupNotice.FrameUnknown
        return Orientation.check(signs, roles)?.let { SetupNotice.FrameRefused(it) }
    }

    private suspend fun writeAll(board: Int) {
        for ((key, want) in _state.value.pending.toList()) {
            if (refuseWhileArmed()) return
            val r = transport.writeConfig(key, want, board)
            if (r == null) {
                _state.update { it.copy(notice = SetupNotice.NotAttached) }
                return
            }
            if (!settle(key, want, r, board)) return
        }
        _state.update { it.copy(linkDroppedSinceApply = false) }
    }

    /** Fold one write's result into the state; false when the Apply must stop here. */
    private suspend fun settle(key: Key, want: Value, r: ConfigWriteResult, board: Int): Boolean {
        val notice = when (r) {
            is WriteVerified -> {
                _state.update {
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
            is Malformed, Busy -> SetupNotice.Garbled(key)
        }
        _state.update { it.copy(notice = notice) }
        // A mismatch or a timeout leaves the stored value in doubt: read it, so the screen shows
        // what the board holds rather than what the app hoped.
        if (r is WriteMismatch || r == TimedOut) transport.readConfig(key, board)?.let { record(key, it) }
        return false
    }

    override fun setLevel() {
        if (refuseWhileArmed()) return
        val s = _state.value
        // Pitch and roll come out of the frame the board booted with: a pending or staged orientation
        // changes them at the power-cycle, so a trim computed now would be wrong afterwards.
        val blocked = when {
            SetupFields.LEVEL_TRIM.any { it.key in s.pending || it.key in s.staged } -> Blocked.NOT_APPLIED
            !s.orientationSettled -> Blocked.NOT_APPLIED
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
        // is running plus the reading. The running trim is the stored one: nothing is staged.
        val readings = listOf(cyclic.pitch, cyclic.roll)
        SetupFields.LEVEL_TRIM.forEachIndexed { i, f ->
            val running = (s.values.getValue(f.key) as Value.I16).v
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
     * orientation is what the board runs, so it refuses while a sign or role index is pending or staged.
     */
    private fun runCheck(step: (RotationCheck, Int, Int) -> RotationCheck) {
        val s = _state.value
        val cyclic = transport.telemetry.value?.cyclic
        val blocked = when {
            !s.orientationSettled -> Blocked.NOT_APPLIED
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

    override fun confirmPowerCycled() {
        val s = _state.value
        if (!s.linkDroppedSinceApply || !s.awaitingPowerCycle || s.busy) return
        _state.update {
            it.copy(
                staged = emptyMap(),
                running = emptyMap(),
                lastRunning = emptyMap(),
                linkDroppedSinceApply = false,
                rotationCheck = RotationCheck(),
            )
        }
        loaded = false
        maybeRefresh()
    }

    override fun dismissNotice() = _state.update { it.copy(notice = null) }

    private fun refuseWhileArmed(): Boolean {
        if (!isArmed()) return false
        _state.update { it.copy(notice = SetupNotice.ReadOnlyWhileArmed) }
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
