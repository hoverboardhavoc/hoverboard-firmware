package com.hoverboard.remote

import com.hoverboard.protocol.board.BoardField
import com.hoverboard.protocol.board.FieldRef
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutPresets
import com.hoverboard.protocol.board.LayoutSlot
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent

/**
 * A [LayoutModel] over the fake transport, whose board answers every layout field's registry
 * default: a board nobody has staged, which is the state an editor most often opens on.
 */
internal class LayoutRig(scope: TestScope) {
    var armed = false
    val transport = FakeHoverboardTransport(scope.testScheduler).apply {
        defaults[Fields.LINK_SET.key()] = Fields.LINK_SET.default
        for (slot in Layout.SLOTS) defaults[slot.key] = slot.def.default
    }
    val model = LayoutModel(transport, scope.backgroundScope) { armed }
    val state: LayoutState get() = model.state.value

    /** The slot for [field] on [motor], the way a test names a layout field. */
    fun slot(field: BoardField, motor: Int? = null): LayoutSlot =
        checkNotNull(Layout.forField(FieldRef(field, motor)))

    /** What the board stores for [field], as a raw byte. */
    fun stored(field: BoardField, motor: Int? = null): Int =
        slot(field, motor).of(checkNotNull(state.stored))

    /** What is staged for [field]. */
    fun staged(field: BoardField, motor: Int? = null): Int =
        slot(field, motor).of(checkNotNull(state.staged))

    /** Set [field] in the staged layout. */
    fun stage(field: BoardField, raw: Int, motor: Int? = null) = model.stage(slot(field, motor), raw)

    /**
     * The board reports [chip] in its cyclic state, which is where the editor's part comes from
     * (`crates/linkctl/src/lib.rs`, `CyclicObs`). The rest of the payload is a board sitting still.
     */
    fun reports(chip: ChipTag) = transport.emitCyclicState(
        CyclicState(
            pitch = 0,
            roll = 0,
            wheelSpeed = 0,
            battery = 0,
            mode = 0,
            fault = 0,
            flags = 0,
            obs = CyclicObs(phasePeak = 0, phaseMean = 0, dutyOn = 0, bootTag = 1, chip = chip),
        ),
    )

    /**
     * The board reports a cyclic state with NO appended block, as an image from before the block
     * existed does: every committed field is there and the part is not.
     */
    fun reportsNoBlock() = transport.emitCyclicState(
        CyclicState(pitch = 0, roll = 0, wheelSpeed = 0, battery = 0, mode = 0, fault = 0, flags = 0),
    )

    /** Put [value] in the fake board's store for [field], as if it had been staged earlier. */
    fun preset(board: Int, field: BoardField, raw: Int, motor: Int? = null) {
        val s = slot(field, motor)
        transport.store[board to s.key] = s.value(raw)
    }

    /** The keys written this session, in order. */
    val written: List<com.hoverboard.protocol.store.Key> get() = transport.writes.map { it.second }

    companion object {
        /** The `LINK_SET` mask of a board whose inter-board link and USART2 BLE port are live. */
        const val LINK_SET_STANDARD = LayoutPresets.LINK_SET_STANDARD

        /** The mask of an offroad board: the inter-board link plus the USART0 BLE wiring. */
        const val LINK_SET_OFFROAD = LayoutPresets.LINK_SET_OFFROAD
    }
}

/**
 * A rig attached to [board] with the layout screen shown, its read pass done, and the board
 * reporting [chip] in its cyclic state. A null [chip] is a board that reports no part at all, which
 * is the case the editor cannot judge.
 */
@OptIn(ExperimentalCoroutinesApi::class)
internal fun TestScope.layoutRig(
    board: Int,
    chip: ChipTag? = ChipTag.F103C8,
    linkSet: Int? = null,
    stage: LayoutRig.() -> Unit = {},
): LayoutRig = LayoutRig(this).also {
    linkSet?.let { mask -> it.transport.store[board to Fields.LINK_SET.key()] = Value.U8(mask) }
    it.stage()
    it.transport.setAttachedBoard(board)
    it.model.onShown()
    runCurrent()
    if (chip == null) it.reportsNoBlock() else it.reports(chip)
    runCurrent()
}
