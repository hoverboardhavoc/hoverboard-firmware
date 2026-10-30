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
import com.hoverboard.remote.model.NodeInfo
import com.hoverboard.remote.model.SetupFields
import kotlin.math.abs
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex

/** What the last Tune operation came to, for the screen to say. */
sealed interface TuneNotice {
    /** SAVE was asked for while armed: it writes flash, which the board refuses armed. Nothing was sent. */
    data object SaveWhileArmed : TuneNotice

    /**
     * A drive bound was set while armed. The bounds are stored values, not staged ones
     * (`specs/rider-ui.md` 3.3b), so the board refuses the write exactly as it refuses SAVE, and
     * nothing was sent. The screen disables the controls armed; this is the backstop.
     */
    data object BoundWhileArmed : TuneNotice

    /** A drive bound was set to a value outside the range the board keeps, so nothing was sent. */
    data class BoundOutOfRange(val key: Key) : TuneNotice

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
 * One board's live-lane values as this app last READ them (`specs/state-feedback.md` section 10,
 * "Live readback").
 *
 * @param staged the RAM shadow per live-lane key, as `TUNE_READ` last reported it. Only a read sets
 *   it: a value this app sent is never shown as the board's, so a board whose shadow was changed by
 *   anything else (another client, a RAM write over SWD) shows what it holds at the next read.
 * @param flash the stored value (`CONFIG_READ`) per live-lane key.
 * @param stale true when these are values from before: no attached session reaches the board, or
 *   the last request to it went unanswered. Writes are disabled until a read pass succeeds.
 * @param converging per gain key, the bound in milliseconds within which the running law reaches
 *   the staged value (the firmware's ramp, [Gains.Ramp]), set when the staged value changed and
 *   cleared once the bound has elapsed. A derived bound, never an observation.
 * @param maxima per gain index, the board's `CONTROL_GAIN_MAX` as read from it; an index absent
 *   until read. The four gains only: the reference row's bounds are compiled.
 * @param bounds the drive bounds as STORED on this board (`specs/rider-ui.md` 3.3b), by key; a key
 *   absent until read. There is one value each and no staged counterpart: they are not on the live
 *   tune lane, so what is written is flash and the board takes it up at its next arm.
 */
data class BoardGains(
    val staged: Map<Key, Int> = emptyMap(),
    val flash: Map<Key, Int> = emptyMap(),
    val stale: Boolean = true,
    val converging: Map<Key, Long> = emptyMap(),
    val maxima: Map<Int, Int> = emptyMap(),
    val bounds: Map<Key, Int> = emptyMap(),
) {
    /**
     * The inclusive lower bound of the value at [key]: [Gains.MIN] for a gain, the compiled
     * [Gains.REFERENCE_MINIMA] for a reference word (`speed_slew`'s floor is 1).
     */
    fun min(key: Key): Int =
        if (key.fieldId == Gains.CONTROL_REFERENCE) Gains.REFERENCE_MINIMA[key.index] else Gains.MIN

    /**
     * The inclusive upper bound of the value at [key], which is a different fact for the two rows
     * this lane carries.
     *
     * A gain takes the maximum read from the board (a negative one as [Gains.MIN], as the firmware
     * reads it), else [Gains.DEFAULT_MAX] while unread. The board re-reads `CONTROL_GAIN_MAX` at
     * every arm, so a maximum written since applies from the next arm; the board's refusal is the
     * backstop either way.
     *
     * A reference word takes [Gains.REFERENCE_MAXIMA], COMPILED into the firmware
     * (`control::config::sf::REFERENCE_MAX`) and so the same on every board.
     */
    fun max(key: Key): Int =
        if (key.fieldId == Gains.CONTROL_REFERENCE) {
            Gains.REFERENCE_MAXIMA[key.index]
        } else {
            maxOf(maxima[key.index] ?: Gains.DEFAULT_MAX[key.index], Gains.MIN)
        }

    /** Whether [key]'s staged value differs from flash (a reboot would revert it). Known only when both are read. */
    fun unsaved(key: Key): Boolean {
        val s = staged[key] ?: return false
        val f = flash[key] ?: return false
        return s != f
    }
}

/**
 * Which motor controllers the Tune screen offers and which it tunes: the Setup model's node list
 * with the bridges left out and its selected motor controller (`specs/rider-ui.md` 3.3 and 3.4;
 * [SetupState]). One owner of the selection, so the board Tune tunes is the board SINGLE drives.
 */
data class TuneSelection(
    val motors: List<NodeInfo> = emptyList(),
    val selected: Int? = null,
)

/**
 * The Tune screen's state (`specs/rider-ui.md` section 3.3, `specs/state-feedback.md` section 10).
 *
 * @param target the address of the motor controller the screen tunes, or null while none is known.
 * @param motors the motor controllers this session found, in address order: the selector's chips.
 */
data class TuneState(
    val target: Int? = null,
    val motors: List<NodeInfo> = emptyList(),
    val boards: Map<Int, BoardGains> = emptyMap(),
    val reading: Boolean = false,
    val writing: Boolean = false,
    val notice: TuneNotice? = null,
) {
    val busy: Boolean get() = reading || writing

    /** The target's address this session, or null when it is not reachable. */
    val address: Int? get() = target

    /** The target's values as last read. */
    val gains: BoardGains get() = target?.let { boards[it] } ?: BoardGains()

    /** Whether a write to the target can be sent now: attached, read this session, and nothing in flight. */
    val writable: Boolean get() = address != null && !gains.stale && !busy

    /**
     * Whether the live-lane sliders may be dragged: attached and read this session.
     *
     * Deliberately NOT gated on [busy], which [writable] is. A drag holds the lane for as long as
     * the finger is down, so a slider disabled while its own writes are in flight would be disabled
     * from the first sample onward: Compose stops delivering the gesture to a disabled control, and
     * the drag would die under the finger. What a busy lane does to a drag is drop samples
     * ([TuneModel.slide]), never take the instrument away mid-gesture.
     */
    val tunable: Boolean get() = address != null && !gains.stale

    /** The four gain keys, `[gp, gr, gx, gv]` (`specs/state-feedback.md` section 6). */
    val keys: List<Key> get() = GAIN_KEYS

    /** The reference row's two keys, `[speed_max, speed_slew]` (`specs/state-feedback.md` section 4). */
    val referenceKeys: List<Key> get() = REFERENCE_KEYS

    /** Every key the screen writes over the live tune lane: the gains, then the reference row. */
    val liveKeys: List<Key> get() = LIVE_KEYS

    /** The live-lane keys whose staged value differs from flash, which is what Save and Revert act on. */
    val unsavedKeys: List<Key> get() = liveKeys.filter { gains.unsaved(it) }

    /**
     * Whether the stick does nothing, which is `speed_max == 0` (`specs/state-feedback.md` section
     * 4: the machine holds position); null until that word has been read.
     *
     * Stated of the RUNNING law rather than of a staged value: the reference row is not ramped, so
     * the shadow this reads is what the law uses on the next tick.
     */
    val stickDead: Boolean? get() = gains.staged[referenceKeys[Gains.SPEED_MAX]]?.let { it == 0 }

    /** What the target board stores for drive bound [key], or null until it has been read. */
    fun bound(key: Key): Int? = gains.bounds[key]

    private companion object {
        val GAIN_KEYS = (0 until Gains.COUNT).map { Gains.key(Gains.CONTROL_SF_GAINS, it) }
        val REFERENCE_KEYS = Gains.REFERENCE_DEFAULT.indices.map { Gains.key(Gains.CONTROL_REFERENCE, it) }
        val LIVE_KEYS = GAIN_KEYS + REFERENCE_KEYS
    }
}

/**
 * What the Tune screen can ask of its model: the gains, the reference row, the drive bounds, and the reads and
 * saves around them. More methods than detekt's threshold for an interface, for [SetupActions]'s
 * reason: it is one screen's single contract, and splitting it to satisfy a count would scatter it.
 */
@Suppress("TooManyFunctions")
interface TuneActions {
    /**
     * The screen came into view: read the target once per attached session, and poll its live-lane
     * values every [TuneModel.POLL_INTERVAL_MS] while it stays in view.
     */
    fun onShown()

    /** The screen left view: the poll stops. */
    fun onHidden()

    /** Tune the motor controller at [address], which also makes it the one SINGLE drives. */
    fun selectTarget(address: Int)

    /** Read the target's values again. */
    fun refresh()

    /**
     * [key]'s slider is being dragged and is at [value], clamped to the seam's range
     * ([BoardGains.min] to [BoardGains.max], which is this board's for a gain and the compiled one
     * for a reference word). A gain or a reference word: the two kinds of row share the lane and
     * therefore share this call.
     *
     * NOT one write per call. A dragged slider emits a value per frame and the tune lane will not
     * take that, so the model sends the latest value at most once per
     * [TuneModel.DRAG_SEND_INTERVAL_MS] and the board's ramp interpolates between what it receives
     * (`specs/rider-ui.md` section 3.3a). Nothing is smoothed on this side: the ramp is doing the
     * smoothing the slider appears to do. A reference word is not ramped at all, so what the board
     * receives is what the law runs on its next tick.
     */
    fun slide(key: Key, value: Int)

    /**
     * The drag of [key]'s slider ended at [value]: stage exactly that.
     *
     * Always sent, whatever the cadence last sampled, so the value the board is left holding is the
     * one the finger left the slider at and never an artifact of where a sample fell. With no drag
     * before it, this is a single staged value and its re-read, which is also what a typed entry
     * sends.
     */
    fun slideEnd(key: Key, value: Int)

    /**
     * Store drive bound [key] at [value] (`specs/rider-ui.md` 3.3b): one `CONFIG_WRITE`, verified,
     * disarmed only, and the board takes it up at its next arm.
     *
     * Called when a slider's drag ENDS and when a typed value is set, never per drag sample: each
     * write appends a flash record, so a dragged slider that wrote at the gains' cadence would
     * spend the store's page on one gesture. A value outside the field's range is not sent.
     */
    fun setBound(key: Key, value: Int)

    /**
     * Persist every unsaved live-lane value, gains and reference row (`CONFIG_WRITE`, disarmed
     * only): the tune lane is RAM, so without a save a value is gone at the next power cycle.
     */
    fun save()

    /** Stage every unsaved live-lane value back to flash (`TUNE_WRITE`). */
    fun revert()

    /** Clear the notice. */
    fun dismissNotice()
}

/**
 * The Tune screen's model (`specs/rider-ui.md` section 3.3, `specs/state-feedback.md` section 10):
 * the four state-feedback gains and the two reference words of one named board over the live tune
 * lane (`TUNE_READ` / `TUNE_WRITE`, RAM only), their stored copies over the config lane
 * (`CONFIG_READ`, and `CONFIG_WRITE` for SAVE), and the drive bounds, which are stored values only.
 *
 * Three values per gain, kept apart: FLASH (what a reboot restores), STAGED (the RAM shadow the lane
 * writes), and ENGAGED (what the law multiplies now), which no wire key carries. The firmware ramps
 * engaged toward staged by a fixed cap per RUN pass, so after a staged value changes this model
 * marks the gain converging for the bound the ramp guarantees ([Gains.Ramp]) and clears the mark
 * when it elapses. That is all it claims: it never says engaged equals staged. The reference row is
 * not ramped and carries no mark.
 *
 * STAGED is what the board last REPORTED (`specs/state-feedback.md` section 10, "Live readback"):
 * a value this model sent is never displayed as the board's. Every settled write is followed by a
 * re-read of its key, and while the screen is shown the model reads every live-lane key again once
 * per [POLL_INTERVAL_MS], so a shadow changed by anything else is on screen within a second. The
 * poll pauses while any of this model's operations holds the lane (a drag, a save, a read pass) and
 * stops when the screen is hidden.
 *
 * The converging bound is taken over every value the gain was read or sent at since the mark was
 * last clear (the engaged value lies between them, whoever wrote the shadow). A sent value widens
 * that range and is shown nowhere; a read that differs from what was sent simply replaces the
 * displayed value, since the display holds reads only.
 *
 * One operation at a time, as on Setup: a settled value is one `TUNE_WRITE` and a re-read. A DRAG
 * is that one operation for as long as the finger is down, sending the slider's latest value at
 * most once per [DRAG_SEND_INTERVAL_MS] and the value it ends at always (section 3.3a,
 * [dragLoop]). Tune writes are allowed armed; SAVE is not.
 */
@Suppress("TooManyFunctions") // implements TuneActions, plus the private steps each action needs
class TuneModel(
    private val transport: HoverboardTransport,
    private val scope: CoroutineScope,
    private val isArmed: () -> Boolean,
    selection: Flow<TuneSelection>,
    private val onSelect: (Int) -> Unit,
) : TuneActions {

    private val _state = MutableStateFlow(TuneState())
    val state: StateFlow<TuneState> = _state.asStateFlow()

    private val op = Mutex()
    private var visible = false

    /** The boards read this attached session. */
    private val loaded = mutableSetOf<Int>()

    /** Per (board, key): every value read or sent since the converging mark was last clear, as a range. */
    private val hulls = mutableMapOf<Pair<Int, Key>, IntRange>()

    /**
     * Per (board, key): a value was sent since the last read, so the next read is a change even if
     * it equals the value shown before.
     */
    private val sentSinceRead = mutableSetOf<Pair<Int, Key>>()

    /** Per (board, key): which mark the pending clear belongs to, so a later write's mark outlives it. */
    private val markTokens = mutableMapOf<Pair<Int, Key>, Int>()
    private var nextToken = 0

    /** The node on the phone's link: a change of it is a new session. */
    private var session: Int? = null

    /** The 1 s live-readback poll, running while the screen is shown. */
    private var pollJob: Job? = null

    /** True while a poll round holds the lane, so an operation that finds it taken waits instead of being dropped. */
    private var polling = false

    /** Set by an operation waiting on a poll round, which then gives the lane up after its current read. */
    private var laneWanted = false

    /**
     * One slider's drag in progress, as its send loop reads it: where the finger is ([want]), what
     * was last handed to the lane ([sent], the staged value at the start, so a drag that moves
     * nowhere sends nothing), whether that value came back from a re-read ([confirmed]), and
     * whether the finger is up ([released]).
     */
    private class Drag(
        val node: Int,
        val board: Int,
        val key: Key,
        var want: Int,
        var sent: Int,
        var released: Boolean,
    ) {
        var confirmed: Boolean = true
    }

    /** The drag whose send loop holds the lane, or null. */
    private var drag: Drag? = null

    /** A release that arrived while this model's lane was taken, kept until it can be sent. */
    private var deferredRelease: Pair<Key, Int>? = null

    /**
     * The slider whose drag the lane refused or left unanswered, until the finger lifts.
     *
     * Without it the next frame's sample would start a fresh drag and write again, so a gesture over
     * a lane that is saying no would hammer it at the cadence for as long as the finger moved. One
     * refusal ends the gesture; the notice says why and the next gesture is free to try.
     */
    private var spentDrag: Key? = null

    init {
        transport.attachedBoard.onEach(::onSession).launchIn(scope)
        selection.onEach(::onSelection).launchIn(scope)
    }

    /** A new session (or none): what was read is now from before, until read again. */
    private fun onSession(attached: Int?) {
        if (attached == session) return
        session = attached
        loaded.clear()
        sentSinceRead.clear()
        deferredRelease = null // a release held for a lane that no longer reaches the same board
        _state.update { st -> st.copy(boards = st.boards.mapValues { (_, b) -> b.copy(stale = true) }) }
        maybeRefresh()
    }

    private fun onSelection(sel: TuneSelection) {
        val before = _state.value.target
        // A drop empties the selection; the board last tuned stays the target, greyed, with what was
        // read from it, until a session names another.
        _state.update {
            it.copy(
                target = sel.selected ?: it.target,
                motors = sel.motors,
                notice = if (sel.selected != null && sel.selected != before) null else it.notice,
            )
        }
        maybeRefresh()
    }

    override fun onShown() {
        visible = true
        maybeRefresh()
        if (pollJob?.isActive != true) {
            pollJob = scope.launch {
                while (true) {
                    delay(POLL_INTERVAL_MS)
                    poll()
                }
            }
        }
    }

    override fun onHidden() {
        visible = false
        pollJob?.cancel()
        pollJob = null
        // A screen that leaves composition mid-drag never delivers the finger-up, and the drag's
        // send loop holds this model's lane until it gets one. Treat the departure as the release:
        // the board keeps the value the slider was last at, which is where the drag had taken it.
        drag?.released = true
    }

    override fun selectTarget(address: Int) {
        if (_state.value.motors.none { it.address == address }) return
        onSelect(address)
    }

    override fun dismissNotice() = _state.update { it.copy(notice = null) }

    private fun maybeRefresh() {
        val board = _state.value.address ?: return
        if (visible && session != null && board !in loaded) refresh()
    }

    override fun refresh() {
        val board = _state.value.address ?: return
        val node = board
        loaded += node
        val started = launchOp({ it.copy(reading = true, notice = null) }, { it.copy(reading = false) }) {
            val ok = readAll(node, board)
            // A pass the taken slot cut short asked the board nothing it failed to answer: what
            // was read stands as it was, and the busy notice alone says why the pass stopped.
            if (ok || _state.value.notice != TuneNotice.SlotBusy) update(node) { it.copy(stale = !ok) }
        }
        if (!started) loaded -= node
    }

    /**
     * One round of the live-readback poll: a `TUNE_READ` of every live-lane key of the target, so
     * what each row displays is what the board holds (`specs/state-feedback.md` section 10).
     *
     * Only on a board read this session and not stale, and only when the lane is free: a drag, a
     * save or a read pass pauses it. The round gives the lane up early when an operation is waiting
     * for it. A busy slot (the Setup model's request in flight) ends the round silently, since
     * nothing was asked; an unanswered or refused read says so and an unanswered one marks the board
     * stale, which stops the poll until a read pass succeeds.
     */
    private suspend fun poll() {
        val s = _state.value
        val board = s.address ?: return
        if (session == null || s.gains.stale || board !in loaded) return
        if (!op.tryLock()) return
        polling = true
        try {
            for (key in s.liveKeys) {
                if (laneWanted || !pollOne(board, key)) break
            }
        } finally {
            polling = false
            op.unlock()
            flushDeferredRelease()
            maybeRefresh()
        }
    }

    /** One polled `TUNE_READ`; false to end the round. */
    private suspend fun pollOne(board: Int, key: Key): Boolean = when (val r = transport.readTune(key, board)) {
        null -> false
        is StagedValue -> {
            staged(board, key, r.value)
            true
        }
        Busy -> false
        else -> {
            answerFailed(key, r)
            if (r == TimedOut) update(board) { it.copy(stale = true) }
            false
        }
    }

    /**
     * Read every value on the live tune lane, all six staged then all six flash: the four gains and
     * the two reference words. Then the four gain maxima and the drive bounds.
     *
     * No maximum is read for the reference row: its bounds are compiled into the firmware
     * ([Gains.REFERENCE_MAXIMA]), so there is nothing on the board to ask for.
     *
     * The order is the point: the two lanes answer with byte-identical `CONFIG_RESP`s, so a late
     * duplicate reply to a `TUNE_READ` would be taken as the answer to a `CONFIG_READ` of the same key
     * sent right after it (the staged value recorded as flash, the unsaved mark lost), and the other
     * way round. No two consecutive requests of the pass share a key.
     */
    private suspend fun readAll(node: Int, board: Int): Boolean {
        val keys = _state.value.liveKeys
        val read = keys.all { readStaged(node, board, it) } &&
            keys.all { readFlash(node, board, it) } &&
            (0 until Gains.COUNT).all { readMax(node, board, it) } &&
            SetupFields.DRIVE_BOUNDS.all { readBound(node, board, it.key) }
        return read
    }

    private suspend fun readStaged(node: Int, board: Int, key: Key): Boolean {
        val r = awaitSlot { transport.readTune(key, board) } ?: return notAttached()
        if (r !is StagedValue) return answerFailed(key, r)
        staged(node, key, r.value)
        return true
    }

    private suspend fun readFlash(node: Int, board: Int, key: Key): Boolean {
        val r = awaitSlot { transport.readConfig(key, board) } ?: return notAttached()
        val v = ((r as? ReadValue)?.value as? Value.I16)?.v
        if (v == null) return answerFailed(key, r)
        update(node) { it.copy(flash = it.flash + (key to v)) }
        return true
    }

    /**
     * Read `CONTROL_GAIN_MAX` [index]. A board that answers it unknown leaves the index unread (the
     * default bound) rather than failing the pass.
     */
    private suspend fun readMax(node: Int, board: Int, index: Int): Boolean {
        val key = Key(Fields.CONTROL_GAIN_MAX.id, index)
        val r = awaitSlot { transport.readConfig(key, board) } ?: return notAttached()
        if (r is Refused && r.refusal == CfgRefusal.UNKNOWN_KEY) {
            update(node) { it.copy(maxima = it.maxima - index) }
            return true
        }
        val v = ((r as? ReadValue)?.value as? Value.I16)?.v ?: return answerFailed(key, r)
        update(node) { it.copy(maxima = it.maxima + (index to v)) }
        return true
    }

    /** Read one drive bound's stored value (`CONFIG_READ`, `i16` like every one of them). */
    private suspend fun readBound(node: Int, board: Int, key: Key): Boolean {
        val r = awaitSlot { transport.readConfig(key, board) } ?: return notAttached()
        val v = ((r as? ReadValue)?.value as? Value.I16)?.v ?: return answerFailed(key, r)
        update(node) { it.copy(bounds = it.bounds + (key to v)) }
        return true
    }

    /**
     * One drive bound, written to flash and re-read nowhere: `CONFIG_WRITE` verifies its own echo,
     * so a verified write IS the board's statement of what it now stores.
     *
     * Refused while armed before anything is sent, which is the same rule SAVE follows and the same
     * one the board enforces (R4 refuses every config write while armed).
     */
    override fun setBound(key: Key, value: Int) {
        val field = SetupFields.DRIVE_BOUNDS.firstOrNull { it.key == key } ?: return
        if (isArmed()) {
            _state.update { it.copy(notice = TuneNotice.BoundWhileArmed) }
            return
        }
        val want = Value.I16(value)
        if (!field.accepts(want)) {
            _state.update { it.copy(notice = TuneNotice.BoundOutOfRange(key)) }
            return
        }
        val s = _state.value
        if (!s.writable || s.bound(key) == value) return
        write(s) { node, board -> storeBound(node, board, key, value) }
    }

    /** One verified `CONFIG_WRITE` of a drive bound; the notice says what else it came to. */
    private suspend fun storeBound(node: Int, board: Int, key: Key, v: Int) {
        if (isArmed()) {
            _state.update { it.copy(notice = TuneNotice.BoundWhileArmed) }
            return
        }
        val notice = when (val r = awaitSlot { transport.writeConfig(key, Value.I16(v), board) }) {
            null -> TuneNotice.NotAttached
            is WriteVerified -> {
                update(node) { it.copy(bounds = it.bounds + (key to v)) }
                return
            }
            is WriteMismatch -> TuneNotice.Mismatch(key, v, (r.stored as? Value.I16)?.v)
            is Refused -> TuneNotice.BoardRefused(key, r.refusal)
            TimedOut -> TuneNotice.Unanswered(key)
            Busy -> TuneNotice.SlotBusy
            is Malformed -> TuneNotice.Garbled(key)
        }
        _state.update { it.copy(notice = notice) }
    }

    override fun slide(key: Key, value: Int) = dragTo(key, value, released = false)

    override fun slideEnd(key: Key, value: Int) = dragTo(key, value, released = true)

    /**
     * Take [key] to [value], as a drag sample or as the release that ends one.
     *
     * While a drag's send loop is running this only moves its target, which is the whole point: the
     * loop owns the lane for the gesture and decides what gets sent. A sample that arrives with no
     * loop running starts one; a sample that cannot start one (this model's own read or save holds
     * the lane) is dropped, because another is a frame away. A RELEASE is never dropped: it is the
     * committed value, so it waits for the lane in [deferredRelease].
     */
    private fun dragTo(key: Key, value: Int, released: Boolean) {
        val s = _state.value
        if (key !in s.liveKeys) return
        if (key == spentDrag) {
            if (released) spentDrag = null
            return
        }
        val want = value.coerceIn(s.gains.min(key)..s.gains.max(key))
        val running = drag
        if (running != null && running.key == key) {
            running.want = want
            running.released = released
        } else {
            startDrag(s, key, want, released)
        }
    }

    /**
     * Take the lane for a new drag of [key] at [want] and run its send loop ([dragLoop]).
     *
     * Nothing starts on a board that is not reachable or not read this session. A drag that cannot
     * have the lane because this model's own read, save or poll holds it is dropped unless it is a
     * [released] one, which waits in [deferredRelease] instead.
     */
    private fun startDrag(s: TuneState, key: Key, want: Int, released: Boolean) {
        val board = s.address ?: return
        val staged = s.gains.staged[key]
        if (s.gains.stale || staged == null) return
        if (!op.tryLock()) {
            // The poll's round gives the lane up after its current read; the next sample, or the
            // deferred release, then has it.
            if (polling) laneWanted = true
            if (released) deferredRelease = key to want
            return
        }
        val d = Drag(board, board, key, want, sent = staged, released = released)
        drag = d
        _state.update { it.copy(writing = true, notice = null) }
        scope.launch {
            try {
                dragLoop(d)
            } finally {
                drag = null
                _state.update { it.copy(writing = false) }
                op.unlock()
                flushDeferredRelease()
            }
        }
    }

    /**
     * Send the drag's latest value, at most one `TUNE_WRITE` per [DRAG_SEND_INTERVAL_MS], until the
     * finger is up and the value it ended at has been staged (`specs/rider-ui.md` section 3.3a).
     *
     * A sample is a write alone: at a drag's rate a re-read per sample would halve what the lane
     * can carry, and the next sample supersedes it in a fraction of a second anyway. The value the
     * gesture ENDS at is the one that stays on the board, so that one is written and re-read the way
     * every settled value is, and a value already sent but never confirmed is re-read on release
     * even when the finger ended where the last sample fell.
     *
     * A refused or unanswered sample stops the loop with its notice: the release is not pushed
     * through a lane that has just said no.
     */
    private suspend fun dragLoop(d: Drag) {
        while (true) {
            if (d.released) {
                if (d.want != d.sent) {
                    d.sent = d.want
                    stageAndReread(d.node, d.board, d.key, d.want)
                } else if (!d.confirmed) {
                    readStaged(d.node, d.board, d.key)
                }
                return
            }
            if (d.want != d.sent) {
                d.sent = d.want
                d.confirmed = false
                if (!stage(d.node, d.board, d.key, d.want)) {
                    spentDrag = d.key
                    return
                }
            }
            delay(DRAG_SEND_INTERVAL_MS)
        }
    }

    /** Send a release the lane was not free for when it arrived; its value is never dropped. */
    private fun flushDeferredRelease() {
        val (key, value) = deferredRelease ?: return
        deferredRelease = null
        dragTo(key, value, released = true)
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

    /**
     * One `TUNE_WRITE` of [v] and the re-read of [key] that follows every settled value; false to
     * stop. The write's echo is not displayed: the re-read is what the row shows.
     */
    private suspend fun stageAndReread(node: Int, board: Int, key: Key, v: Int): Boolean {
        val r: TuneWriteResult = awaitSlot { transport.writeTune(key, v, board) } ?: return notAttached()
        val notice = writeNotice(key, v, r)
        if (notice != null) _state.update { it.copy(notice = notice) }
        if (r is TuneVerified) sent(node, key, r.staged)
        // Re-read whatever the write came to: a refusal or a mismatch leaves the shadow in doubt,
        // and a verified one is confirmed the way 3.3 asks (a settled value is written and re-read).
        val reread = readStaged(node, board, key)
        if (r == TimedOut && !reread) update(node) { it.copy(stale = true) }
        return notice == null && reread
    }

    /**
     * One `TUNE_WRITE` of [v] with no re-read: a drag sample, which the next sample supersedes
     * ([dragLoop]); the release is re-read. False to stop the drag.
     *
     * An unanswered sample marks the board stale where a settled value's write would not have: with
     * no re-read of its own, nothing here can say what the shadow now holds.
     */
    private suspend fun stage(node: Int, board: Int, key: Key, v: Int): Boolean {
        val r: TuneWriteResult = awaitSlot { transport.writeTune(key, v, board) } ?: return notAttached()
        if (r is TuneVerified) {
            sent(node, key, r.staged)
            return true
        }
        _state.update { it.copy(notice = writeNotice(key, v, r)) }
        if (r == TimedOut) update(node) { it.copy(stale = true) }
        return false
    }

    /** What a `TUNE_WRITE` of [v] to [key] answering [r] leaves the screen to say, or null when verified. */
    private fun writeNotice(key: Key, v: Int, r: TuneWriteResult): TuneNotice? = when (r) {
        is TuneVerified -> null
        is TuneMismatch -> TuneNotice.Mismatch(key, v, r.staged)
        is Refused -> TuneNotice.BoardRefused(key, r.refusal)
        TimedOut -> TuneNotice.Unanswered(key)
        Busy -> TuneNotice.SlotBusy
        is Malformed -> TuneNotice.Garbled(key)
    }

    /** Run [block] on the target as the one operation, marking the screen writing. */
    private fun write(s: TuneState, block: suspend (Int, Int) -> Unit) {
        val board = s.address ?: return
        val node = board
        launchOp({ it.copy(writing = true, notice = null) }, { it.copy(writing = false) }) { block(node, board) }
    }

    /**
     * Run [block] as this model's one operation, [mark]ing the state while it runs: now if the lane
     * is free, after the poll's current read if a poll round holds it (a tap is never lost to the
     * poll), and not at all, returning false, if another operation holds it.
     */
    private fun launchOp(
        mark: (TuneState) -> TuneState,
        unmark: (TuneState) -> TuneState,
        block: suspend () -> Unit,
    ): Boolean {
        val now = op.tryLock()
        if (!now && !polling) return false
        _state.update(mark)
        scope.launch {
            if (!now) {
                laneWanted = true
                op.lock()
                laneWanted = false
            }
            try {
                block()
            } finally {
                _state.update(unmark)
                op.unlock()
                flushDeferredRelease()
                maybeRefresh()
            }
        }
        return true
    }

    /**
     * Record that the board verified [v] as [node]'s [key]: the range the converging bound covers
     * grows to include it, since the law may already be ramping toward it. Nothing is displayed:
     * the row shows the read that follows.
     */
    private fun sent(node: Int, key: Key, v: Int) {
        if (key.fieldId != Gains.CONTROL_SF_GAINS) return
        val id = node to key
        sentSinceRead += id
        val old = _state.value.boards[node]?.staged?.get(key) ?: return
        val hull = hulls[id] ?: old..old
        hulls[id] = minOf(hull.first, v)..maxOf(hull.last, v)
    }

    /**
     * Record that [node]'s [key] READ as [v]. When that changes a known staged GAIN, the running law
     * has a new target to ramp to: mark it converging for the ramp's bound across every value read
     * or sent since the mark was last clear.
     *
     * A reference word is recorded and nothing more: the reference row is not ramped, so a write is
     * live on the next tick and there is no interval to mark.
     */
    private fun staged(node: Int, key: Key, v: Int) {
        val old = _state.value.boards[node]?.staged?.get(key)
        update(node) { it.copy(staged = it.staged + (key to v)) }
        if (key.fieldId != Gains.CONTROL_SF_GAINS) return
        val id = node to key
        // A read of the value already shown, with nothing sent since, is a poll finding nothing new:
        // it moves no target and must not re-arm the mark.
        val wrote = sentSinceRead.remove(id)
        if (old == null || (old == v && !wrote)) return
        val hull = hulls[id]
        val span = (hull ?: old..old).let { minOf(it.first, v)..maxOf(it.last, v) }
        hulls[id] = span
        val distance = maxOf(abs(v - span.first), abs(v - span.last))
        if (distance == 0) return
        val bound = Gains.Ramp.boundMs(key.index, distance)
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

    private fun update(node: Int, f: (BoardGains) -> BoardGains) =
        _state.update { it.copy(boards = it.boards + (node to f(it.boards[node] ?: BoardGains()))) }

    companion object {
        /**
         * The shortest gap between two `TUNE_WRITE`s of a dragged slider: 200 ms, so 5 Hz
         * (`specs/rider-ui.md` section 3.3a, whose starting proposal is 5 to 10 Hz).
         *
         * The low end of that range is the one the link's measured budget leaves room for, and
         * tuning happens WHILE ARMED on a balancing machine, so the drive stream is what must not be
         * squeezed. A `TUNE_WRITE` is 13 bytes on the wire (an 8-byte PDU in a stream frame,
         * `Controller.buildTuneWrite` and `StreamFrame`), the same as a `DRIVE_CMD`. The armed
         * demand stream already costs about 280 B/s of the CC2541 bridge's metered UART, and the
         * module was measured overrunning its BLE-to-UART buffer at about 360 B/s
         * ([com.hoverboard.remote.ble.LinkConfig.SEND_INTERVAL_MS]). That leaves roughly 80 B/s:
         * 5 Hz of writes is 65 B/s and fits, 10 Hz is 130 B/s and does not.
         *
         * The cadence is a CEILING on the rate, not a schedule. Each send also waits for the board's
         * answer on the one request slot the config lane shares, so a slower link simply sends less,
         * and the ramp interpolates whatever arrives.
         */
        const val DRAG_SEND_INTERVAL_MS = 200L

        /**
         * The live-readback cadence: one round of `TUNE_READ`s of every live-lane key per second
         * while the screen is shown (`specs/state-feedback.md` section 10). Six reads of 13 bytes
         * each, about 80 B/s for the round's second while it runs; a round that finds the lane
         * taken is skipped, not queued.
         */
        const val POLL_INTERVAL_MS = 1_000L
    }
}
