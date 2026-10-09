package com.hoverboard.remote

import com.hoverboard.protocol.board.BoardFields
import com.hoverboard.protocol.board.ChipFamily
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutPreset
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.board.PIN_ABSENT
import com.hoverboard.protocol.board.Validated
import com.hoverboard.protocol.board.reservedSet
import com.hoverboard.protocol.board.validate
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
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.HoverboardTransport
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

/** What the last layout operation came to, for the screen to say. */
sealed interface LayoutNotice {
    /** The app refused the edit itself: the board is armed, and every config write would be refused. */
    data object ReadOnlyWhileArmed : LayoutNotice

    /** The board answered a write to [key] with a named refusal (`CFG_ARMED` among them). */
    data class BoardRefused(val key: Key, val refusal: CfgRefusal) : LayoutNotice

    /** The board accepted the write to [key] but echoes a different stored value (or none). */
    data class Mismatch(val key: Key, val wrote: Value, val stored: Value?) : LayoutNotice

    /** The write to [key] went unanswered through the whole retransmit budget. */
    data class Unanswered(val key: Key) : LayoutNotice

    /** The board answered the write to [key] with something the wire contract does not define. */
    data class Garbled(val key: Key) : LayoutNotice

    /** The session's one request slot stayed taken through the whole wait, so nothing was sent. */
    data object SlotBusy : LayoutNotice

    /** No board is attached, so nothing was sent. */
    data object NotAttached : LayoutNotice

    /**
     * The board's part is not known, so there is no verdict to apply against: a layout is only
     * valid or invalid ON a part. Either the board sends no part at all, or it sends a tag this app
     * has no capability table for ([LayoutState.chip]).
     */
    data object PartUnknown : LayoutNotice

    /** The staged layout is one the board would refuse. There is no override for a failing verdict. */
    data object VerdictNotClean : LayoutNotice

    /** There is no layout to edit yet: the read pass has not produced one. */
    data object NotRead : LayoutNotice

    /** A different board attached: the edits belonged to [previous] and were dropped. */
    data class BoardChanged(val previous: Int) : LayoutNotice
}

/**
 * The board layout editor's state (`specs/rider-ui.md` section 3.5).
 *
 * @param board the target: the attached board's address, or null while none is attached.
 * @param chip the part the board itself reports, from the `chip` tag of its `CYCLIC_STATE`
 *   (`crates/linkctl/src/lib.rs`, `CyclicObs`). Null until the board has sent one carrying the
 *   appended block, which an image from before it existed never does. It survives a link drop the
 *   way the staged layout does, because power-cycling the board is part of applying a layout, and
 *   it is dropped when a DIFFERENT board attaches: the part is that board's fact, not the session's.
 * @param values the stored value of each layout field, as last read or verified-written.
 * @param unread fields whose last read failed.
 * @param linkSet the board's `LINK_SET` mask, which decides which allowlist pins are reserved
 *   against a layout field and which are freed for one. Read with the rest.
 * @param staged the layout being edited: ONE object, the whole field set, never a basket of
 *   independent values. Null until a read pass has produced a layout to edit.
 * @param written fields written AND verified this session: the firmware reads the layout once at
 *   boot, so these run only after a power-cycle.
 * @param linkDroppedSinceApply whether the link has dropped since the last Apply finished. A
 *   power-cycle takes the Bluetooth module down with the board.
 * @param pendingLatch a power-latch pin a preset would change, waiting on an explicit confirmation.
 *   The latch is the one true brick: on battery that pin is what holds the board's own rail up, so a
 *   wrong one powers the board off at boot and only SWD recovers it. Until it is confirmed the
 *   staged layout keeps the latch the board already holds, which is the pin the board is known to
 *   come up on.
 */
data class LayoutState(
    val board: Int? = null,
    val chip: ChipTag? = null,
    val values: Map<Key, Value> = emptyMap(),
    val unread: Set<Key> = emptySet(),
    val linkSet: Int? = null,
    val staged: BoardFields? = null,
    val reading: Boolean = false,
    val applying: Boolean = false,
    val written: Set<Key> = emptySet(),
    val linkDroppedSinceApply: Boolean = false,
    val pendingLatch: Int? = null,
    val notice: LayoutNotice? = null,
) {
    val busy: Boolean get() = reading || applying

    /**
     * The capability table the verdict is computed against: the one the board's own [chip] tag
     * names, or null when the board named no part this app models ([ChipFamily.forTag]).
     *
     * Null is the whole unknown case, and it has no manual override. A layout is only valid or
     * invalid ON a part, so with no part there is no verdict to show and nothing to edit, which is
     * how the screen already behaves with no board attached.
     */
    val part: ChipFamily? get() = chip?.let { ChipFamily.forTag(it) }

    /** The layout the board stores, or null while any field of it is unread. */
    val stored: BoardFields? get() = Layout.fieldsFrom(values)

    /** The reserved pins no field may claim on this board, or null while the inputs are not known. */
    val reserved: List<Int>? get() = part?.let { p -> linkSet?.let { reservedSet(p, it) } }

    /**
     * The verdict the board would reach on the staged layout: what [com.hoverboard.protocol.board.validate]
     * answers, which is what its own boot will answer. Null while a part, the link set or the layout
     * itself is not known.
     */
    val verdict: Validated? get() = judge(staged)

    /**
     * The verdict on what the board STORES right now, which is what it would boot into.
     *
     * Worth showing in its own right: writes go one field at a time, so an Apply cut short leaves a
     * layout that is neither the old one nor the new one, and a layout is one object, so that state
     * is an invalid board rather than a partial success. The screen says so rather than implying the
     * board is simply behind.
     */
    val storedVerdict: Validated? get() = judge(stored)

    private fun judge(fields: BoardFields?): Validated? {
        val p = part ?: return null
        val r = reserved ?: return null
        return fields?.let { validate(it, p, r) }
    }

    /** The fields that differ between the board's layout and the staged one, in field order. */
    val delta: List<LayoutSlot>
        get() {
            val from = stored ?: return emptyList()
            val to = staged ?: return emptyList()
            return Layout.delta(from, to)
        }

    /** Whether the staged layout is one the board would accept. */
    val clean: Boolean get() = verdict?.error == null && verdict != null

    /** Whether Apply may run: a clean verdict, something to write, and nothing in the way. */
    fun canApply(armed: Boolean): Boolean =
        board != null && !armed && !busy && delta.isNotEmpty() && clean

    /** The one power-cycle instruction: the whole delta is written and nothing is left to write. */
    val awaitingPowerCycle: Boolean get() = written.isNotEmpty() && delta.isEmpty()

    /**
     * Whether the staged layout claims any pin for [motor]: a hall, a gate or a phase sense.
     *
     * The editor offers a motor's tab on this (`specs/rider-ui.md` section 3.5a). A motor with no
     * pin at all is not a motor half-configured, it is a board with one motor, which is the
     * all-or-none rule the validator enforces asked as a question.
     */
    fun motorHoldsAPin(motor: Int): Boolean {
        val fields = staged ?: return false
        return Layout.SLOTS.any { it.motor == motor && it.isPin && it.of(fields) != PIN_ABSENT }
    }
}

/**
 * What the layout screen can ask of its model. One method per thing the screen offers, which is more
 * than detekt's interface threshold; splitting the screen's single contract to satisfy a count would
 * only scatter it.
 */
@Suppress("TooManyFunctions")
interface LayoutActions {
    /** The screen came into view: read the layout once per attached session. */
    fun onShown()

    /** The screen left view. */
    fun onHidden()

    /** Read the layout again. */
    fun refresh()

    /** Set one field of the staged layout. */
    fun stage(slot: LayoutSlot, raw: Int)

    /**
     * Stage a whole known-good layout, which is the normal way to configure a board. It is judged
     * on the part the BOARD reports, whatever part the preset was written for, and a power-latch
     * pin it would change waits on [confirmLatchChange].
     */
    fun stagePreset(preset: LayoutPreset)

    /** Stage the power-latch pin a preset asked for, having read what that means. */
    fun confirmLatchChange()

    /** Keep the power-latch pin the board already holds, and apply the rest of the preset. */
    fun cancelLatchChange()

    /** Put one field back to what the board stores. */
    fun revert(slot: LayoutSlot)

    /** Put the whole staged layout back to what the board stores. */
    fun revertAll()

    /** Write the delta, one field at a time, each verified against the value the board echoes. */
    fun apply()

    /** The operator power-cycled the board: the written marks go and the layout is read again. */
    fun confirmPowerCycled()

    /** Clear the notice. */
    fun dismissNotice()
}

/**
 * The board layout editor's model (`specs/rider-ui.md` section 3.5): read the attached board's
 * layout, edit it locally, show the verdict its own boot validator would reach, and write it.
 *
 * Two things make this not the Setup screen next door. The verdict is EXACT rather than advisory,
 * because `board::validate` is a pure function of the staged fields, the part's capabilities and the
 * reserved set, and all three are in hand here ([LayoutState.verdict]): the part comes off the wire
 * too, from the board's own `chip` tag, so nothing about the verdict rests on what a user stated.
 * And the thing being edited is ONE object with ownership rules across its fields, so there is no
 * basket of independent values: [stage] edits a copy of the whole layout, [LayoutState.delta] is
 * what it would take to make the board hold it, and Apply is refused outright unless the verdict is
 * clean. There is no override.
 *
 * Writes still go one field at a time, verified, because that is what the wire offers. A write that
 * fails mid-Apply therefore leaves the board holding a layout that is neither: the model keeps the
 * rest staged and the screen reports the stored layout's own verdict, which is the honest statement
 * of that state.
 */
@Suppress("TooManyFunctions") // implements LayoutActions, plus the private steps each action needs
class LayoutModel(
    private val transport: HoverboardTransport,
    private val scope: CoroutineScope,
    private val isArmed: () -> Boolean,
) : LayoutActions {

    private val _state = MutableStateFlow(LayoutState())
    val state: StateFlow<LayoutState> = _state.asStateFlow()

    /** One operation at a time: a read pass or an apply. */
    private val op = Mutex()

    private var visible = false

    /** Whether this attached session has had its read pass. */
    private var loaded = false

    /** The board the staged layout belongs to; survives a link drop, unlike [LayoutState.board]. */
    private var editedBoard: Int? = null

    init {
        transport.attachedBoard.onEach(::onAttachedBoard).launchIn(scope)
        // The part is the board's own statement, carried in every CYCLIC_STATE
        // (`crates/linkctl/src/lib.rs`, `CyclicObs`), so the editor reads it instead of asking.
        // Only a report is folded in: a telemetry stream that goes away (the link dropped, the
        // session ended) says nothing new about the part of the board being edited, and the
        // power-cycle an Apply ends in is exactly such a gap.
        transport.telemetry
            .onEach { t -> t?.chip?.let { tag -> _state.update { it.copy(chip = tag) } } }
            .launchIn(scope)
    }

    private fun onAttachedBoard(board: Int?) {
        loaded = false
        if (board == null) {
            _state.update {
                it.copy(
                    board = null,
                    linkDroppedSinceApply = it.linkDroppedSinceApply || it.written.isNotEmpty() || it.applying,
                )
            }
            return
        }
        val previous = editedBoard
        editedBoard = board
        _state.update {
            if (previous != null && previous != board) {
                // A layout belongs to the board it was read from: the pins of another board are
                // another board's facts, and staging them here would be editing the wrong machine.
                // The part goes with them, and comes back when this board reports its own.
                LayoutState(board = board, notice = LayoutNotice.BoardChanged(previous))
            } else {
                it.copy(board = board)
            }
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
                resolveStaged()
            } finally {
                _state.update { it.copy(reading = false) }
                op.unlock()
            }
        }
    }

    /** The layout's own fields, plus the `LINK_SET` mask the reserved set is computed from. */
    private suspend fun readAll(board: Int) {
        for (key in listOf(Fields.LINK_SET.key()) + Layout.SLOTS.map { it.key }) {
            if (_state.value.board != board) return
            val r = awaitSlot { transport.readConfig(key, board) } ?: return
            record(key, r)
        }
    }

    private fun record(key: Key, r: ConfigReadResult) {
        _state.update {
            if (r is ReadValue) {
                val linkSet = if (key == Fields.LINK_SET.key()) r.value.asLong()?.toInt() else it.linkSet
                it.copy(values = it.values + (key to r.value), unread = it.unread - key, linkSet = linkSet)
            } else {
                it.copy(values = it.values - key, unread = it.unread + key)
            }
        }
    }

    /**
     * After a read pass: the staged layout starts as the board's own, and an edit the board already
     * holds is no longer an edit (a write that landed with its answer lost, which this folds in).
     *
     * A staged layout that is still being edited is kept, with the fields the read just confirmed
     * left as the user set them: the read pass tells the model what the BOARD holds, which is what
     * the delta is measured from, and discarding an operator's unwritten edits because a refresh ran
     * would lose work the screen never warned about.
     */
    private fun resolveStaged() = _state.update { s ->
        val stored = s.stored ?: return@update s
        s.copy(staged = s.staged ?: stored)
    }

    override fun stage(slot: LayoutSlot, raw: Int) {
        if (refuseWhileArmed()) return
        _state.update { s ->
            val staged = s.staged ?: return@update s
            s.copy(staged = slot.on(staged, raw), notice = null)
        }
    }

    /**
     * Stage [preset] over the board's own layout.
     *
     * A preset is a whole layout rather than a patch, so nothing of a previous staging survives in a
     * field it does not mention; the three per-motor facts a pin map cannot state are kept as the
     * board holds them ([LayoutPreset.applyTo]). What a preset does NOT state here is the part: the
     * board reports its own, and a preset written for another one cannot overrule the silicon the
     * layout is going to run on.
     *
     * The power latch is held back. A preset that would move it leaves [LayoutState.pendingLatch]
     * set and the staged layout on the pin the board is known to come up on, so the operator decides
     * that one field having read what it costs.
     */
    override fun stagePreset(preset: LayoutPreset) {
        if (refuseWhileArmed()) return
        _state.update { s ->
            val stored = s.stored ?: return@update s.copy(notice = LayoutNotice.NotRead)
            val wanted = Layout.LATCH.of(preset.fields)
            s.copy(
                staged = Layout.LATCH.on(preset.applyTo(stored), Layout.LATCH.of(stored)),
                pendingLatch = wanted.takeIf { it != Layout.LATCH.of(stored) },
                notice = null,
            )
        }
    }

    override fun confirmLatchChange() {
        if (refuseWhileArmed()) return
        _state.update { s ->
            val raw = s.pendingLatch ?: return@update s
            val staged = s.staged ?: return@update s
            s.copy(staged = Layout.LATCH.on(staged, raw), pendingLatch = null)
        }
    }

    override fun cancelLatchChange() = _state.update { it.copy(pendingLatch = null) }

    override fun revert(slot: LayoutSlot) = _state.update { s ->
        val stored = s.stored ?: return@update s
        val staged = s.staged ?: return@update s
        s.copy(staged = slot.on(staged, slot.of(stored)))
    }

    override fun revertAll() = _state.update { s -> s.copy(staged = s.stored ?: s.staged) }

    override fun apply() {
        if (refuseWhileArmed()) return
        val s = _state.value
        val refusal = when {
            s.board == null -> LayoutNotice.NotAttached
            s.part == null -> LayoutNotice.PartUnknown
            // The other half of the reserved set: the part is known and the board's LINK_SET is
            // not, so the pins the link holds are not known either and there is nothing to judge.
            s.linkSet == null -> LayoutNotice.NotRead
            !s.clean -> LayoutNotice.VerdictNotClean
            else -> null
        }
        if (refusal != null) {
            _state.update { it.copy(notice = refusal) }
            return
        }
        val board = s.board ?: return
        if (s.delta.isEmpty() || !op.tryLock()) return
        _state.update { it.copy(applying = true, notice = null) }
        scope.launch {
            try {
                writeAll(board)
            } finally {
                _state.update { it.copy(applying = false) }
                op.unlock()
            }
        }
    }

    /**
     * Write the delta, field by field, in field order.
     *
     * A field the board already holds is skipped rather than rewritten, which is also the resume
     * rule: an Apply cut short by a link drop is finished by applying again, since the delta is
     * measured from what the board stores and the fields that landed are no longer in it.
     */
    private suspend fun writeAll(board: Int) {
        val staged = _state.value.staged ?: return
        for (slot in _state.value.delta) {
            // A field the board already holds is skipped, which is also what makes applying again
            // the way to finish an Apply a link drop cut short.
            if (slot.of(staged) == _state.value.stored?.let(slot::of)) continue
            if (refuseWhileArmed()) return
            val want = slot.value(slot.of(staged))
            val answer = awaitSlot { transport.writeConfig(slot.key, want, board) }
            if (answer == null) {
                _state.update { it.copy(notice = LayoutNotice.NotAttached) }
                return
            }
            if (!settle(slot.key, want, answer, board)) return
        }
        _state.update { it.copy(linkDroppedSinceApply = false) }
    }

    /** Fold one write's result into the state; false when the Apply must stop here. */
    private suspend fun settle(key: Key, want: Value, r: ConfigWriteResult, board: Int): Boolean {
        val notice = when (r) {
            is WriteVerified -> {
                _state.update {
                    it.copy(values = it.values + (key to r.stored), written = it.written + key)
                }
                return true
            }
            is WriteMismatch -> LayoutNotice.Mismatch(key, want, r.stored)
            is Refused -> LayoutNotice.BoardRefused(key, r.refusal)
            TimedOut -> LayoutNotice.Unanswered(key)
            is Malformed -> LayoutNotice.Garbled(key)
            Busy -> LayoutNotice.SlotBusy
        }
        _state.update { it.copy(notice = notice) }
        // A mismatch or a timeout leaves the stored value in doubt: read it, so the screen shows
        // what the board holds rather than what the app hoped, and the delta is measured from that.
        if (r is WriteMismatch || r == TimedOut) awaitSlot { transport.readConfig(key, board) }?.let { record(key, it) }
        return false
    }

    override fun confirmPowerCycled() {
        val s = _state.value
        if (!s.linkDroppedSinceApply || !s.awaitingPowerCycle || s.busy) return
        _state.update { it.copy(written = emptySet(), linkDroppedSinceApply = false) }
        loaded = false
        maybeRefresh()
    }

    override fun dismissNotice() = _state.update { it.copy(notice = null) }

    private fun refuseWhileArmed(): Boolean {
        if (!isArmed()) return false
        _state.update { it.copy(notice = LayoutNotice.ReadOnlyWhileArmed) }
        return true
    }
}
