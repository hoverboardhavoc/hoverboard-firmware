package com.hoverboard.remote

import com.hoverboard.protocol.config.Busy
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.Malformed
import com.hoverboard.protocol.config.ReadValue
import com.hoverboard.protocol.config.Refused
import com.hoverboard.protocol.config.StagedValue
import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.config.TuneMismatch
import com.hoverboard.protocol.config.TuneVerified
import com.hoverboard.protocol.config.TuneWriteResult
import com.hoverboard.protocol.config.WriteMismatch
import com.hoverboard.protocol.config.WriteVerified
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.model.Node
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
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

/** What the last Tune operation came to, for the screen to say. */
sealed interface TuneNotice {
    /** SAVE was asked for while armed: it writes flash, which the board refuses armed. Nothing was sent. */
    data object SaveWhileArmed : TuneNotice

    /** The board answered a write to [key] with a named refusal (`CFG_BAD`: outside the seam's range). */
    data class BoardRefused(val key: Key, val refusal: CfgRefusal) : TuneNotice

    /** The board accepted a write to [key] but echoes [echoed] (or nothing), not [wrote]. */
    data class Mismatch(val key: Key, val wrote: Int, val echoed: Int?) : TuneNotice

    /** A request about [key] went unanswered through the whole retransmit budget. */
    data class Unanswered(val key: Key) : TuneNotice

    /** The board answered a request about [key] with something the wire contract does not define. */
    data class Garbled(val key: Key) : TuneNotice

    /**
     * The session's one request slot stayed taken by the other screen's operations through the whole
     * wait ([awaitSlot]), so nothing was sent. Not an answer from the board; trying again is.
     */
    data object SlotBusy : TuneNotice

    /** No board is attached, so nothing was sent. */
    data object NotAttached : TuneNotice
}

/**
 * One board's gains as this app last read them.
 *
 * @param staged the RAM shadow (`TUNE_READ`, or a verified `TUNE_WRITE` echo) per gain key.
 * @param flash the stored value (`CONFIG_READ`, or a verified `CONFIG_WRITE` echo) per gain key.
 * @param riderRequired the board's `CONTROL_RIDER_REQUIRED` as read from it (the slave only; the
 *   master's is the Setup model's to know, see [TuneState.profileBHidden]); null until read.
 * @param stale true when these are values from before: no attached session reaches the board, or
 *   the last request to it went unanswered. Writes are disabled until a read pass succeeds.
 * @param converging per gain key, the bound in milliseconds within which the running loop reaches
 *   the staged value (the firmware's ramp, [Gains.Ramp]), set when the staged value changed and
 *   cleared once the bound has elapsed. A derived bound, never an observation.
 */
data class BoardGains(
    val staged: Map<Key, Int> = emptyMap(),
    val flash: Map<Key, Int> = emptyMap(),
    val riderRequired: Boolean? = null,
    val stale: Boolean = true,
    val converging: Map<Key, Long> = emptyMap(),
) {
    /** Whether [key]'s staged value differs from flash (a reboot would revert it). Known only when both are read. */
    fun unsaved(key: Key): Boolean {
        val s = staged[key] ?: return false
        val f = flash[key] ?: return false
        return s != f
    }
}

/**
 * The Tune screen's state (`specs/rider-ui.md` section 3.3).
 *
 * @param target which board the screen tunes ([Node.MASTER], the attached one, or [Node.SLAVE]).
 * @param profile the gain field the screen shows: [Gains.CONTROL_GAIN_A] or [Gains.CONTROL_GAIN_B].
 * @param master the attached board's address this session, or null.
 * @param slave the slave's address from this session's discovery, or null.
 * @param masterRiderWaiver what the Setup model knows of the master's running
 *   `CONTROL_RIDER_REQUIRED` ([SetupState.riderWaiver]).
 */
data class TuneState(
    val target: Node = Node.MASTER,
    val profile: Int = Gains.CONTROL_GAIN_A,
    val master: Int? = null,
    val slave: Int? = null,
    val masterRiderWaiver: RiderWaiver = RiderWaiver.NONE,
    val boards: Map<Node, BoardGains> = emptyMap(),
    val reading: Boolean = false,
    val writing: Boolean = false,
    val notice: TuneNotice? = null,
) {
    val busy: Boolean get() = reading || writing

    /** The target's address this session, or null when it is not reachable. */
    val address: Int? get() = if (target == Node.MASTER) master else slave

    /** The target's gains as last read. */
    val gains: BoardGains get() = boards[target] ?: BoardGains()

    /** Whether a write to the target can be sent now: attached, read this session, and nothing in flight. */
    val writable: Boolean get() = address != null && !gains.stale && !busy

    /**
     * Whether the target board runs `CONTROL_RIDER_REQUIRED` at 0, so its rider-gated profile select
     * always picks A and Profile B is unreachable (`specs/control.md` (i)): the screen hides B. Only
     * when known: a board that may still be running the required default shows both.
     *
     * The slave's is an assumption the master's is not: it is the slave's STORED value, read once
     * per session ([BoardGains.riderRequired]) and taken to be what the slave runs, because nothing
     * on the wire reports what it booted with. The screen says so.
     */
    val profileBHidden: Boolean
        get() = if (target == Node.MASTER) masterRiderWaiver == RiderWaiver.WAIVED else gains.riderRequired == false

    /**
     * Whether the master may be running its rider requirement waived ([RiderWaiver.POSSIBLY]), so the
     * pads level the screen shows is not necessarily the rider level its profile select acts on.
     */
    val masterRiderMaybeWaived: Boolean
        get() = target == Node.MASTER && masterRiderWaiver == RiderWaiver.POSSIBLY

    /** The profile the screen shows: [profile], or A while B is hidden. */
    val shownProfile: Int get() = if (profileBHidden) Gains.CONTROL_GAIN_A else profile

    /** The shown profile's three gain keys, `[kp, bk, pr]`. */
    val keys: List<Key> get() = (0 until Gains.PER_PROFILE).map { Gains.key(shownProfile, it) }

    /** The shown profile's keys whose staged value differs from flash. */
    val unsavedKeys: List<Key> get() = keys.filter { gains.unsaved(it) }
}

/** What the Tune screen can ask of its model. */
interface TuneActions {
    /** The screen came into view: read the target once per attached session. */
    fun onShown()

    /** The screen left view. */
    fun onHidden()

    /** Tune [node] (the slave only once this session's discovery found one). */
    fun selectTarget(node: Node)

    /** Show [fieldId]'s gains ([Gains.CONTROL_GAIN_A] or [Gains.CONTROL_GAIN_B]). */
    fun selectProfile(fieldId: Int)

    /** Read the target's gains again. */
    fun refresh()

    /**
     * Stage gain [index] of the shown profile one tap [up] or down from its staged value: the
     * per-tap step [TuneModel.TAP_STEP], clamped to the seam's range.
     */
    fun step(index: Int, up: Boolean)

    /** Persist every unsaved gain of the shown profile (`CONFIG_WRITE`, disarmed only). */
    fun save()

    /** Stage every unsaved gain of the shown profile back to its flash value (`TUNE_WRITE`). */
    fun revert()

    /** Clear the notice. */
    fun dismissNotice()
}

/**
 * The Tune screen's model (`specs/rider-ui.md` section 3.3): the balance gains of one named board,
 * over the live tune lane (`TUNE_READ` / `TUNE_WRITE`, RAM only) and the config lane
 * (`CONFIG_READ`, and `CONFIG_WRITE` for SAVE).
 *
 * Three values per gain, kept apart: FLASH (what a reboot restores), STAGED (the RAM shadow the lane
 * writes), and ENGAGED (what the balance loop multiplies now), which no wire key carries. The
 * firmware ramps engaged toward staged on every RUN pass, so after a staged value changes this
 * model marks the gain converging for the bound the ramp guarantees ([Gains.Ramp]) and clears the
 * mark when it elapses. That is all it claims: it never says engaged equals staged.
 *
 * The bound is taken over every value the gain was staged at since the mark was last clear (the
 * engaged value lies between them), at the lowest battery word this session saw from the master
 * (the ramp is slower at a lower word), and at the ramp's floor of one count per pass for the slave,
 * whose word is not reported, and for `pr`, whose cap divides by a `kd` that is not on the wire.
 *
 * One operation at a time and never in the background, as on Setup: each tap is one `TUNE_WRITE`
 * and a re-read, nothing streams (section 1.7). Tune writes are allowed armed; SAVE is not.
 */
@Suppress("TooManyFunctions") // implements TuneActions, plus the private steps each action needs
class TuneModel(
    private val transport: HoverboardTransport,
    private val scope: CoroutineScope,
    private val isArmed: () -> Boolean,
    riderWaiver: Flow<RiderWaiver>,
) : TuneActions {

    private val _state = MutableStateFlow(TuneState())
    val state: StateFlow<TuneState> = _state.asStateFlow()

    private val op = Mutex()
    private var visible = false

    /** The boards read this attached session. */
    private val loaded = mutableSetOf<Node>()

    /** Per (board, key): every value staged since the converging mark was last clear, as a range. */
    private val hulls = mutableMapOf<Pair<Node, Key>, IntRange>()

    /** Per (board, key): which mark the pending clear belongs to, so a later write's mark outlives it. */
    private val markTokens = mutableMapOf<Pair<Node, Key>, Int>()
    private var nextToken = 0

    /** The lowest nonzero battery word the master reported this session, or null. */
    private var minMasterBattery: Int? = null

    init {
        combine(transport.attachedBoard, transport.slaveBoard, ::Pair).onEach { (m, s) -> onBoards(m, s) }
            .launchIn(scope)
        riderWaiver.onEach { w -> _state.update { it.copy(masterRiderWaiver = w) } }
            .launchIn(scope)
        transport.telemetry.onEach { t ->
            val word = t?.cyclic?.battery?.takeIf { it > 0 } ?: return@onEach
            minMasterBattery = minOf(word, minMasterBattery ?: word)
        }.launchIn(scope)
    }

    private fun onBoards(master: Int?, slave: Int?) {
        val s = _state.value
        if (master == s.master && slave == s.slave) return
        loaded.clear()
        if (master != s.master) minMasterBattery = null
        // A new session (or none): what was read is now from before, until read again.
        _state.update { st ->
            st.copy(master = master, slave = slave, boards = st.boards.mapValues { (_, b) -> b.copy(stale = true) })
        }
        maybeRefresh()
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

    override fun selectProfile(fieldId: Int) {
        if (fieldId != Gains.CONTROL_GAIN_A && fieldId != Gains.CONTROL_GAIN_B) return
        _state.update { it.copy(profile = fieldId) }
    }

    override fun dismissNotice() = _state.update { it.copy(notice = null) }

    private fun maybeRefresh() {
        val s = _state.value
        if (visible && s.target !in loaded && s.address != null) refresh()
    }

    override fun refresh() {
        val s = _state.value
        val board = s.address ?: return
        if (!op.tryLock()) return
        val node = s.target
        loaded += node
        _state.update { it.copy(reading = true, notice = null) }
        scope.launch {
            try {
                val ok = readAll(node, board)
                // A pass the taken slot cut short asked the board nothing it failed to answer: what
                // was read stands as it was, and the busy notice alone says why the pass stopped.
                if (ok || _state.value.notice != TuneNotice.SlotBusy) update(node) { it.copy(stale = !ok) }
            } finally {
                _state.update { it.copy(reading = false) }
                op.unlock()
                maybeRefresh()
            }
        }
    }

    /**
     * Read every gain of both profiles, all six staged values then all six flash values, and (on the
     * slave) its rider requirement.
     *
     * The order is the point: the two lanes answer with byte-identical `CONFIG_RESP`s, so a late
     * duplicate reply to a `TUNE_READ` would be taken as the answer to a `CONFIG_READ` of the same key
     * sent right after it (the staged value recorded as flash, the unsaved mark lost), and the other
     * way round. No two consecutive requests of the pass share a key.
     */
    private suspend fun readAll(node: Node, board: Int): Boolean {
        val keys = listOf(Gains.CONTROL_GAIN_A, Gains.CONTROL_GAIN_B).flatMap { p ->
            (0 until Gains.PER_PROFILE).map { Gains.key(p, it) }
        }
        if (!keys.all { readStaged(node, board, it) } || !keys.all { readFlash(node, board, it) }) return false
        if (node == Node.SLAVE) {
            val r = awaitSlot { transport.readConfig(Fields.CONTROL_RIDER_REQUIRED.key(0), board) } ?: return false
            if (r !is ReadValue) return answerFailed(Fields.CONTROL_RIDER_REQUIRED.key(0), r)
            val v = (r.value as? Value.U8)?.v
            update(node) { it.copy(riderRequired = v != Fields.RiderRequired.NOT_REQUIRED) }
        }
        return true
    }

    private suspend fun readStaged(node: Node, board: Int, key: Key): Boolean {
        val r = awaitSlot { transport.readTune(key, board) } ?: return notAttached()
        if (r !is StagedValue) return answerFailed(key, r)
        staged(node, key, r.value)
        return true
    }

    private suspend fun readFlash(node: Node, board: Int, key: Key): Boolean {
        val r = awaitSlot { transport.readConfig(key, board) } ?: return notAttached()
        val v = ((r as? ReadValue)?.value as? Value.I16)?.v
        if (v == null) return answerFailed(key, r)
        update(node) { it.copy(flash = it.flash + (key to v)) }
        return true
    }

    override fun step(index: Int, up: Boolean) {
        val s = _state.value
        if (!s.writable || index !in 0 until Gains.PER_PROFILE) return
        val key = Gains.key(s.shownProfile, index)
        val now = s.gains.staged[key] ?: return
        val delta = if (up) TAP_STEP[index] else -TAP_STEP[index]
        val want = (now + delta).coerceIn(Gains.RANGE[index])
        if (want == now) return
        write(s) { node, board -> stageAndReread(node, board, key, want) }
    }

    override fun revert() {
        val s = _state.value
        if (!s.writable) return
        val targets = s.unsavedKeys.associateWith { s.gains.flash.getValue(it) }
        if (targets.isEmpty()) return
        write(s) { node, board ->
            for ((key, v) in targets) if (!stageAndReread(node, board, key, v)) break
        }
    }

    override fun save() {
        if (isArmed()) {
            _state.update { it.copy(notice = TuneNotice.SaveWhileArmed) }
            return
        }
        val s = _state.value
        if (!s.writable) return
        val dirty = s.unsavedKeys.associateWith { s.gains.staged.getValue(it) }
        if (dirty.isEmpty()) return
        write(s) { node, board ->
            for ((key, v) in dirty) if (!persist(key, v, board)) break
            // The divergence comes from a re-read of both, never from the write's CFG_OK: a save of
            // a value flash already held leaves a diverging staged value where it was (3.3). All flash
            // then all staged, so that with two or more keys no request follows one of the same key
            // (see readAll); a single key's write and two reads are covered by the exchange's settle.
            if (dirty.keys.all { readFlash(node, board, it) }) {
                for (key in dirty.keys) if (!readStaged(node, board, key)) break
            }
        }
    }

    /** One `CONFIG_WRITE`, unless the board was armed since SAVE began; false when the save must stop here. */
    private suspend fun persist(key: Key, v: Int, board: Int): Boolean {
        if (isArmed()) {
            _state.update { it.copy(notice = TuneNotice.SaveWhileArmed) }
            return false
        }
        val notice = when (val r = awaitSlot { transport.writeConfig(key, Value.I16(v), board) }) {
            null -> TuneNotice.NotAttached
            is WriteVerified -> return true
            is WriteMismatch -> TuneNotice.Mismatch(key, v, (r.stored as? Value.I16)?.v)
            is Refused -> TuneNotice.BoardRefused(key, r.refusal)
            TimedOut -> TuneNotice.Unanswered(key)
            Busy -> TuneNotice.SlotBusy
            is Malformed -> TuneNotice.Garbled(key)
        }
        _state.update { it.copy(notice = notice) }
        return false
    }

    /** One `TUNE_WRITE` of [v] and the re-read of [key] that follows every tap; false to stop. */
    private suspend fun stageAndReread(node: Node, board: Int, key: Key, v: Int): Boolean {
        val r: TuneWriteResult = awaitSlot { transport.writeTune(key, v, board) } ?: return notAttached()
        val notice = when (r) {
            is TuneVerified -> null
            is TuneMismatch -> TuneNotice.Mismatch(key, v, r.staged)
            is Refused -> TuneNotice.BoardRefused(key, r.refusal)
            TimedOut -> TuneNotice.Unanswered(key)
            Busy -> TuneNotice.SlotBusy
            is Malformed -> TuneNotice.Garbled(key)
        }
        if (notice != null) _state.update { it.copy(notice = notice) }
        if (r is TuneVerified) staged(node, key, r.staged)
        // Re-read whatever the write came to: a refusal or a mismatch leaves the shadow in doubt,
        // and a verified one is confirmed the way 3.3 asks (each tap writes and re-reads).
        val reread = readStaged(node, board, key)
        if (r == TimedOut && !reread) update(node) { it.copy(stale = true) }
        return notice == null && reread
    }

    /** Run [block] on the target as the one operation, marking the screen writing. */
    private fun write(s: TuneState, block: suspend (Node, Int) -> Unit) {
        val board = s.address ?: return
        if (!op.tryLock()) return
        val node = s.target
        _state.update { it.copy(writing = true, notice = null) }
        scope.launch {
            try {
                block(node, board)
            } finally {
                _state.update { it.copy(writing = false) }
                op.unlock()
            }
        }
    }

    /**
     * Record that [node]'s [key] is staged at [v]. When that changes a known staged value, the
     * running loop has a new target to ramp to: mark it converging for the ramp's bound across every
     * value staged since the mark was last clear.
     */
    private fun staged(node: Node, key: Key, v: Int) {
        val old = _state.value.boards[node]?.staged?.get(key)
        update(node) { it.copy(staged = it.staged + (key to v)) }
        val id = node to key
        val hull = hulls[id]
        if (old == null || (old == v && hull == null)) return
        val span = (hull ?: old..old).let { minOf(it.first, v)..maxOf(it.last, v) }
        hulls[id] = span
        val distance = maxOf(abs(v - span.first), abs(v - span.last))
        val scale = if (node == Node.MASTER) minMasterBattery else null
        val bound = Gains.Ramp.boundMs(key.index, distance, scale)
        val token = ++nextToken
        markTokens[id] = token
        update(node) { it.copy(converging = it.converging + (key to bound)) }
        scope.launch {
            delay(bound)
            if (markTokens[id] != token) return@launch
            markTokens -= id
            hulls -= id
            update(node) { it.copy(converging = it.converging - key) }
        }
    }

    private fun answerFailed(key: Key, r: Any): Boolean {
        val notice = when (r) {
            is Refused -> TuneNotice.BoardRefused(key, r.refusal)
            TimedOut -> TuneNotice.Unanswered(key)
            Busy -> TuneNotice.SlotBusy
            else -> TuneNotice.Garbled(key)
        }
        _state.update { it.copy(notice = notice) }
        return false
    }

    private fun notAttached(): Boolean {
        _state.update { it.copy(notice = TuneNotice.NotAttached) }
        return false
    }

    private fun update(node: Node, f: (BoardGains) -> BoardGains) =
        _state.update { it.copy(boards = it.boards + (node to f(it.boards[node] ?: BoardGains()))) }

    companion object {
        /**
         * The per-tap step of `[kp, bk, pr]` (`specs/rider-ui.md` 3.3: per-tap step limits, no typed
         * entry). A tap moves a gain by at most this; the firmware's ramp carries it to the loop.
         */
        val TAP_STEP = listOf(100, 50, 5)
    }
}
