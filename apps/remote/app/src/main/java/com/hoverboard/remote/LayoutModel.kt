package com.hoverboard.remote

import com.hoverboard.protocol.board.BoardFields
import com.hoverboard.protocol.board.ChipFamily
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.board.PIN_ABSENT
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
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.model.TelemetryUi
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

    /** There is no layout to edit yet: the read pass has not produced one. */
    data object NotRead : LayoutNotice

    /** A different board attached: the edits belonged to [previous] and were dropped. */
    data class BoardChanged(val previous: Int) : LayoutNotice
}

/**
 * The board layout editor's state (`specs/remote-ui.md` section 3.5).
 *
 * @param board the target: the attached board's address, or null while none is attached.
 * @param chip the part the board itself reports, from the `chip` tag of its `CYCLIC_STATE`
 *   (`shared/linkctl/src/lib.rs`, `CyclicObs`). Null until the board has sent a `CYCLIC_STATE` at
 *   all, since every one of them carries the part. It survives a link drop the way the staged
 *   layout does, because power-cycling the board is part of applying a layout, and it is dropped
 *   when a DIFFERENT board attaches: the part is that board's fact, not the session's.
 * @param values the stored value of each layout field, as last read or verified-written.
 * @param unread fields whose last read failed.
 * @param staged the layout being edited: ONE object, the whole field set, never a basket of
 *   independent values. Null until a read pass has produced a layout to edit.
 * @param written fields written AND verified this session: the firmware reads the layout once at
 *   boot, so these run only after a power-cycle. Dropped when the board's boot counter on the wire
 *   shows it restarted, which is the app observing what it used to ask the operator to assert.
 */
data class LayoutState(
    val board: Int? = null,
    val chip: ChipTag? = null,
    val values: Map<Key, Value> = emptyMap(),
    val unread: Set<Key> = emptySet(),
    val staged: BoardFields? = null,
    val reading: Boolean = false,
    val applying: Boolean = false,
    val written: Set<Key> = emptySet(),
    val notice: LayoutNotice? = null,
) {
    val busy: Boolean get() = reading || applying

    /**
     * The part the pin picker offers pins of: the one the board's own [chip] tag names, or null
     * when the board named no part this app models ([ChipFamily.forTag]). With no part the pin
     * rows cannot be picked, since which pins exist is a fact about the silicon.
     */
    val part: ChipFamily? get() = chip?.let { ChipFamily.forTag(it) }

    /** The layout the board stores, or null while any field of it is unread. */
    val stored: BoardFields? get() = Layout.fieldsFrom(values)

    /** The fields that differ between the board's layout and the staged one, in field order. */
    val delta: List<LayoutSlot>
        get() {
            val from = stored ?: return emptyList()
            val to = staged ?: return emptyList()
            return Layout.delta(from, to)
        }

    /**
     * Whether Apply may run: a board attached and disarmed, nothing in flight, and something to
     * write. Nothing judges the layout first: the board brings up what it can at its next boot and
     * lists what it could not in its status (Setup's machine status).
     */
    fun canApply(armed: Boolean): Boolean =
        board != null && !armed && !busy && delta.isNotEmpty()

    /** The one power-cycle instruction: the whole delta is written and nothing is left to write. */
    val awaitingPowerCycle: Boolean get() = written.isNotEmpty() && delta.isEmpty()

    /**
     * Whether the staged layout claims any pin for [motor]: a hall, a gate or a phase sense.
     *
     * The editor offers a motor's tab on this (`specs/remote-ui.md` section 3.5a). A motor with no
     * pin at all is not a motor half-configured, it is a board with one motor.
     */
    fun motorHoldsAPin(motor: Int): Boolean {
        val fields = staged ?: return false
        return Layout.SLOTS.any { it.motor == motor && it.isPin && it.of(fields) != PIN_ABSENT }
    }
}

/** What the layout screen can ask of its model: one method per thing the screen offers. */
interface LayoutActions {
    /** The screen came into view: read the layout once per attached session. */
    fun onShown()

    /** The screen left view. */
    fun onHidden()

    /** Read the layout again. */
    fun refresh()

    /** Set one field of the staged layout. */
    fun stage(slot: LayoutSlot, raw: Int)

    /** Put one field back to what the board stores. */
    fun revert(slot: LayoutSlot)

    /** Put the whole staged layout back to what the board stores. */
    fun revertAll()

    /** Write the delta, one field at a time, each verified against the value the board echoes. */
    fun apply()

    /** Clear the notice. */
    fun dismissNotice()
}

/**
 * The board layout editor's model (`specs/remote-ui.md` section 3.5): read the attached board's
 * layout, edit it locally, and write it.
 *
 * The thing being edited is ONE object, so there is no basket of independent values: [stage] edits
 * a copy of the whole layout, and [LayoutState.delta] is what it would take to make the board hold
 * it. Nothing judges the layout here. The firmware brings up each function from its own fields at
 * boot and lists what did not come up in `board.status`, which Setup's machine status shows
 * (`specs/configured-links.md` sections 3 and 5).
 *
 * Writes go one field at a time, verified, because that is what the wire offers. A write that
 * fails mid-Apply leaves the rest staged, and applying again finishes it.
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

    /**
     * The boot tag ([TelemetryUi.bootTag]) last seen from the board being edited, absent before any
     * arrived. A CHANGE in it is the restart that puts a written layout into effect ([observeBoot]).
     * Held here rather than in the state because it must survive the link drop a power-cycle is.
     */
    private var bootTag: Int? = null

    init {
        transport.attachedBoard.onEach(::onAttachedBoard).launchIn(scope)
        // The board's own statements, carried in every CYCLIC_STATE (`shared/linkctl/src/lib.rs`,
        // `CyclicObs`), so the editor reads them instead of asking: the part it is, and the boot
        // it is on. The ATTACHED board's, by its address: the other board's state reaches the
        // phone on the same link, and this screen edits the layout of one board, so a part taken
        // from whatever arrived last would offer one board the pins of the other board's
        // silicon.
        //
        // Only a report is folded in: a telemetry stream that goes away (the link dropped, the
        // session ended) says nothing new about the board being edited, and the power-cycle an
        // Apply ends in is exactly such a gap.
        combine(transport.attachedBoard, transport.telemetry) { board, byBoard ->
            board?.let { byBoard[it] }
        }
            .onEach { report -> report?.let(::onReport) }
            .launchIn(scope)
    }

    /** Fold one `CYCLIC_STATE` from the board being edited in: the part it names, and its boot. */
    private fun onReport(report: TelemetryUi) {
        report.chip?.let { tag -> _state.update { s -> s.copy(chip = tag) } }
        observeBoot(report.bootTag)
    }

    /**
     * A board whose boot tag differs from the one last seen has restarted, and the firmware reads
     * the layout once at boot, so every verified write of this session is now in effect and there
     * is nothing left to report. The layout is read again, because what the board is RUNNING is
     * what it now stores.
     *
     * This is the whole of what the "I power-cycled it" button used to do, taken from the
     * operator's word and given to the wire, as the Setup screen's own boot watch already does
     * (`SetupModel.observeBoot`).
     *
     * A missing tag is no observation, never a change, and the first tag of a session is not one
     * either. Neither strands an Apply: every `CYCLIC_STATE` carries the tag, so a session that
     * wrote anything has seen one.
     */
    private fun observeBoot(tag: Int?) {
        if (tag == null) return
        val previous = bootTag
        bootTag = tag
        if (previous == null || previous == tag) return
        _state.update { it.copy(written = emptySet()) }
        loaded = false
        maybeRefresh()
    }

    private fun onAttachedBoard(board: Int?) {
        loaded = false
        if (board == null) {
            _state.update { it.copy(board = null) }
            return
        }
        val previous = editedBoard
        editedBoard = board
        if (previous != null && previous != board) bootTag = null
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

    /** The layout's own fields. */
    private suspend fun readAll(board: Int) {
        for (key in Layout.SLOTS.map { it.key }) {
            if (_state.value.board != board) return
            val r = awaitSlot { transport.readConfig(key, board) } ?: return
            record(key, r)
        }
    }

    private fun record(key: Key, r: ConfigReadResult) {
        _state.update {
            if (r is ReadValue) {
                it.copy(values = it.values + (key to r.value), unread = it.unread - key)
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

    override fun revert(slot: LayoutSlot) = _state.update { s ->
        val stored = s.stored ?: return@update s
        val staged = s.staged ?: return@update s
        s.copy(staged = slot.on(staged, slot.of(stored)))
    }

    override fun revertAll() = _state.update { s -> s.copy(staged = s.stored ?: s.staged) }

    override fun apply() {
        if (refuseWhileArmed()) return
        val s = _state.value
        val board = s.board
        if (board == null) {
            _state.update { it.copy(notice = LayoutNotice.NotAttached) }
            return
        }
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

    override fun dismissNotice() = _state.update { it.copy(notice = null) }

    private fun refuseWhileArmed(): Boolean {
        if (!isArmed()) return false
        _state.update { it.copy(notice = LayoutNotice.ReadOnlyWhileArmed) }
        return true
    }
}
