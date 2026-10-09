package com.hoverboard.remote

import com.hoverboard.protocol.board.BoardErrorKind
import com.hoverboard.protocol.board.BoardField
import com.hoverboard.protocol.board.ChipFamily
import com.hoverboard.protocol.board.FieldRef
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.LayoutPresets
import com.hoverboard.protocol.board.PIN_ABSENT
import com.hoverboard.protocol.board.Pin
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.Refused
import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.remote.model.LayoutEditor
import com.hoverboard.remote.model.LayoutRows
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The board layout editor's model (`specs/rider-ui.md` section 3.5).
 *
 * The properties here are the ones that make the editor worth having, rather than the ones that
 * make it look right: the verdict is the board's own, a layout that steals a link port cannot be
 * applied, a group is all-or-none, Apply writes the delta and nothing else, and a half-written
 * layout is reported as an invalid board rather than as progress.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class LayoutModelTest {

    private fun pin(name: String) = checkNotNull(Pin.byName(name)).packed

    @Test
    fun aReadPassProducesTheBoardsOwnLayoutWithNothingStaged() = runTest {
        val rig = layoutRig(BOARD)

        assertEquals(BOARD, rig.state.board)
        assertNotNull("the whole field set was read", rig.state.stored)
        assertEquals("the staged layout starts as the board's", rig.state.stored, rig.state.staged)
        assertEquals(emptyList<Any>(), rig.state.delta)
        assertEquals(emptySet<Any>(), rig.state.unread)
        // Every field of the layout was asked for, plus the link set the reserved pins come from.
        assertTrue(Fields.LINK_SET.key() in rig.transport.reads.map { it.second })
        assertTrue(Layout.SLOTS.all { it.key in rig.transport.reads.map { r -> r.second } })
        // A board nobody has staged validates: that is what the benign registry defaults are for.
        assertTrue(rig.state.clean)
        assertNull(rig.state.verdict?.error)
        assertEquals(pin("PB12"), rig.state.verdict?.selfHold?.packed)
    }

    @Test
    fun thereIsNoVerdictUntilThePartIsStated() = runTest {
        // The chip is not readable over the link, and the same layout is valid on one part and
        // refused on the next, so the app does not guess.
        val rig = layoutRig(BOARD, part = null)

        assertNotNull(rig.state.stored)
        assertNull("no part, no verdict", rig.state.verdict)
        assertFalse(rig.state.clean)
        assertFalse(rig.state.canApply(armed = false))
        rig.stage(BoardField.BUZZER, pin("PB5"))
        rig.model.apply()
        runCurrent()
        assertEquals(LayoutNotice.NoPartSelected, rig.state.notice)
        assertEquals(emptyList<Any>(), rig.transport.writes)

        rig.model.selectPart(ChipFamily.F103C8)
        assertTrue("stating the part is what makes the verdict possible", rig.state.clean)
    }

    @Test
    fun aLayoutThatStealsALinkPortCannotBeApplied() = runTest {
        val rig = layoutRig(BOARD)

        rig.stage(BoardField.BUZZER, pin("PA2")) // the inter-board link's TX

        val error = checkNotNull(rig.state.verdict?.error)
        assertEquals(FieldRef(BoardField.BUZZER), error.field)
        assertEquals(BoardErrorKind.ReservedPin(checkNotNull(Pin.byName("PA2"))), error.kind)
        assertFalse(rig.state.canApply(armed = false))
        rig.model.apply()
        runCurrent()
        assertEquals(LayoutNotice.VerdictNotClean, rig.state.notice)
        assertEquals("nothing was written", emptyList<Any>(), rig.transport.writes)
    }

    @Test
    fun whichPinsAreReservedFollowsTheBoardsLinkSet() = runTest {
        // The same pins, the same part, two masks. The offroad board's BLE module is on PB6/PB7, so
        // an IMU there is refused; the bench mask leaves that port clear, which is what frees those
        // pins for the IMU.
        val offroad = layoutRig(BOARD, part = ChipFamily.F130C8, linkSet = LayoutRig.LINK_SET_OFFROAD)
        offroad.stage(BoardField.IMU_SCL, pin("PB6"))
        offroad.stage(BoardField.IMU_SDA, pin("PB7"))
        offroad.stage(BoardField.IMU_MODEL, Fields.ImuModel.CLONE_2E)
        assertEquals(
            BoardErrorKind.ReservedPin(checkNotNull(Pin.byName("PB6"))),
            offroad.state.verdict?.error?.kind,
        )

        val bench = layoutRig(BOARD, part = ChipFamily.F130C8, linkSet = LayoutRig.LINK_SET_STANDARD)
        bench.stage(BoardField.IMU_SCL, pin("PB6"))
        bench.stage(BoardField.IMU_SDA, pin("PB7"))
        bench.stage(BoardField.IMU_MODEL, Fields.ImuModel.CLONE_2E)
        assertTrue(bench.state.clean)
        assertEquals("PB6/PB7 is I2C0", 0, bench.state.verdict?.plan?.imu?.bus)
    }

    @Test
    fun aGateCapablePinRefusesANonGateFunction() = runTest {
        val rig = layoutRig(BOARD)

        rig.stage(BoardField.LED_GREEN, pin("PA8")) // TIMER0 CH0

        val error = checkNotNull(rig.state.verdict?.error)
        assertEquals(FieldRef(BoardField.LED_GREEN), error.field)
        assertEquals(BoardErrorKind.GateCapableMisused(checkNotNull(Pin.byName("PA8"))), error.kind)
        assertFalse(rig.state.canApply(armed = false))
    }

    @Test
    fun aGroupIsAllOrNoneSoAHalfStagedOneIsRefused() = runTest {
        val rig = layoutRig(BOARD)

        // Halls: one pin is not a third of a working function, it is a refused layout.
        rig.stage(BoardField.HALL_A, pin("PC13"), motor = 0)
        assertEquals(
            "the refusal names the first missing member",
            FieldRef(BoardField.HALL_B, 0),
            rig.state.verdict?.error?.field,
        )
        assertEquals(BoardErrorKind.IncompleteGroup, rig.state.verdict?.error?.kind)
        rig.stage(BoardField.HALL_B, pin("PA1"), motor = 0)
        rig.stage(BoardField.HALL_C, pin("PC14"), motor = 0)
        assertTrue("the whole group validates", rig.state.clean)

        // Gates: six pins AND a non-zero dead time.
        for ((field, name) in listOf(
            BoardField.GATE_HI_A to "PA8", BoardField.GATE_HI_B to "PA9", BoardField.GATE_HI_C to "PA10",
            BoardField.GATE_LO_A to "PB13", BoardField.GATE_LO_B to "PB14", BoardField.GATE_LO_C to "PB15",
        )) {
            rig.stage(field, pin(name), motor = 0)
        }
        assertEquals(
            "a configured gate group with no dead time",
            BoardErrorKind.MissingDeadTime,
            rig.state.verdict?.error?.kind,
        )
        rig.stage(BoardField.DEAD_TIME, 25, motor = 0)
        assertTrue(rig.state.clean)

        // The IMU pair needs its model, and the model needs its pair.
        rig.stage(BoardField.IMU_SCL, pin("PB6"))
        rig.stage(BoardField.IMU_SDA, pin("PB7"))
        assertEquals(FieldRef(BoardField.IMU_MODEL), rig.state.verdict?.error?.field)
        rig.stage(BoardField.IMU_MODEL, Fields.ImuModel.CLONE_2E)
        assertTrue(rig.state.clean)

        // The phase-current declaration and the pins that realize it may not disagree.
        rig.stage(BoardField.CURRENT_SENSE, 1, motor = 0)
        assertEquals(FieldRef(BoardField.PHASE_A, 0), rig.state.verdict?.error?.field)
        rig.stage(BoardField.PHASE_A, pin("PB0"), motor = 0)
        rig.stage(BoardField.PHASE_B, pin("PA0"), motor = 0)
        assertTrue(rig.state.clean)
        assertEquals(listOf(8, 0), rig.state.verdict?.plan?.motors?.get(0)?.phaseCurrent?.channels)
    }

    @Test
    fun applyWritesTheDeltaInFieldOrderAndMarksItUnapplied() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(BoardField.BUZZER, pin("PB5"))
        rig.stage(BoardField.LED_RED, pin("PB1"))

        assertEquals(2, rig.state.delta.size)
        rig.model.apply()
        runCurrent()

        assertEquals(
            "one write per changed field, in field order, and nothing else",
            listOf(Fields.BOARD_BUZZER.key(), Fields.LED_RED.key()),
            rig.written,
        )
        assertEquals("the board holds what was staged", rig.state.stored, rig.state.staged)
        assertEquals(emptyList<Any>(), rig.state.delta)
        assertEquals(setOf(Fields.BOARD_BUZZER.key(), Fields.LED_RED.key()), rig.state.written)
        // The layout is read once at boot, so a verified write is not a running value.
        assertTrue(rig.state.awaitingPowerCycle)
        assertFalse("no drop yet, so no power-cycle can have happened", rig.state.linkDroppedSinceApply)
    }

    @Test
    fun aWriteRefusedMidApplyLeavesTheBoardHoldingAnInvalidLayout() = runTest {
        // A layout is one object, so a half-written one is not progress: the board stores something
        // it would itself refuse, and the model says so rather than implying it is merely behind.
        val rig = layoutRig(BOARD)
        rig.stage(BoardField.HALL_A, pin("PC13"), motor = 0)
        rig.stage(BoardField.HALL_B, pin("PA1"), motor = 0)
        rig.stage(BoardField.HALL_C, pin("PC14"), motor = 0)
        assertTrue(rig.state.clean)
        rig.transport.writeHook = { key, _ ->
            Refused(CfgRefusal.STORE_ERR).takeIf { key == Fields.MOTOR_HALL_B.key(0) }
        }

        rig.model.apply()
        runCurrent()

        assertEquals(
            LayoutNotice.BoardRefused(Fields.MOTOR_HALL_B.key(0), CfgRefusal.STORE_ERR),
            rig.state.notice,
        )
        assertEquals(
            "it stopped at the refusal",
            listOf(Fields.MOTOR_HALL_A.key(0), Fields.MOTOR_HALL_B.key(0)),
            rig.written,
        )
        assertEquals(
            "what the board now stores is a layout it would refuse",
            BoardErrorKind.IncompleteGroup,
            rig.state.storedVerdict?.error?.kind,
        )
        assertEquals("the rest is still to write", 2, rig.state.delta.size)

        // Applying again finishes it: the delta is measured from what the board holds, so the field
        // that landed is not written twice.
        rig.transport.writeHook = { _, _ -> null }
        rig.model.apply()
        runCurrent()
        assertEquals(
            listOf(
                Fields.MOTOR_HALL_A.key(0),
                Fields.MOTOR_HALL_B.key(0),
                Fields.MOTOR_HALL_B.key(0),
                Fields.MOTOR_HALL_C.key(0),
            ),
            rig.written,
        )
        assertNull(rig.state.storedVerdict?.error)
        assertEquals(emptyList<Any>(), rig.state.delta)
    }

    @Test
    fun anUnansweredWriteReadsTheFieldBackRatherThanAssumingIt() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(BoardField.BUZZER, pin("PB5"))
        // The write lands, its answer is lost: the board holds the new value and the app does not
        // know it. The read-back is what settles it, and the delta then empties on its own.
        rig.transport.writeHook = { key, value ->
            TimedOut.also { rig.transport.store[BOARD to key] = value }
        }

        rig.model.apply()
        runCurrent()

        assertEquals(LayoutNotice.Unanswered(Fields.BOARD_BUZZER.key()), rig.state.notice)
        assertEquals("read back, not assumed", pin("PB5"), rig.stored(BoardField.BUZZER))
        assertEquals(emptyList<Any>(), rig.state.delta)
    }

    @Test
    fun armedRefusesEveryEditAndEveryWrite() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(BoardField.BUZZER, pin("PB5"))
        rig.armed = true

        rig.stage(BoardField.LED_RED, pin("PB1"))
        assertEquals(LayoutNotice.ReadOnlyWhileArmed, rig.state.notice)
        assertEquals("the edit was refused, not staged", rig.stored(BoardField.LED_RED), rig.staged(BoardField.LED_RED))
        assertFalse(rig.state.canApply(armed = true))

        rig.model.apply()
        runCurrent()
        assertEquals(LayoutNotice.ReadOnlyWhileArmed, rig.state.notice)
        assertEquals(emptyList<Any>(), rig.transport.writes)
    }

    @Test
    fun revertPutsAFieldAndTheWholeLayoutBack() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(BoardField.BUZZER, pin("PB5"))
        rig.stage(BoardField.LED_RED, pin("PB1"))

        rig.model.revert(rig.slot(BoardField.BUZZER))
        assertEquals(rig.stored(BoardField.BUZZER), rig.staged(BoardField.BUZZER))
        assertEquals(1, rig.state.delta.size)

        rig.model.revertAll()
        assertEquals(rig.state.stored, rig.state.staged)
        assertEquals(emptyList<Any>(), rig.state.delta)
    }

    @Test
    fun aDifferentBoardDropsTheLayoutReadFromTheOldOne() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(BoardField.BUZZER, pin("PB5"))

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(OTHER_BOARD)
        runCurrent()

        assertEquals(LayoutNotice.BoardChanged(BOARD), rig.state.notice)
        assertEquals(OTHER_BOARD, rig.state.board)
        assertEquals(
            "the part is a statement about the editor's session, not about one board",
            ChipFamily.F103C8,
            rig.state.part,
        )
    }

    @Test
    fun aPowerCycleConfirmationClearsTheWrittenMarksAndReadsAgain() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(BoardField.BUZZER, pin("PB5"))
        rig.model.apply()
        runCurrent()
        assertTrue(rig.state.awaitingPowerCycle)

        // The button is only live once the link has dropped, because a power-cycle takes the module
        // down with the board.
        rig.model.confirmPowerCycled()
        assertEquals("not confirmable before a drop", setOf(Fields.BOARD_BUZZER.key()), rig.state.written)

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(BOARD)
        runCurrent()
        assertTrue(rig.state.linkDroppedSinceApply)
        rig.model.confirmPowerCycled()
        runCurrent()

        assertEquals(emptySet<Any>(), rig.state.written)
        assertFalse(rig.state.awaitingPowerCycle)
        assertEquals("re-read after the reboot", pin("PB5"), rig.stored(BoardField.BUZZER))
    }

    @Test
    fun anUnreadableFieldMeansNoLayoutAtAll() = runTest {
        // A layout is one object: a field that could not be read is not a layout with a gap in it,
        // so there is nothing to judge and nothing to edit.
        val rig = LayoutRig(this)
        rig.transport.unreadable += Fields.PAD_B.key()
        rig.transport.setAttachedBoard(BOARD)
        rig.model.onShown()
        runCurrent()
        rig.model.selectPart(ChipFamily.F103C8)

        assertEquals(setOf(Fields.PAD_B.key()), rig.state.unread)
        assertNull(rig.state.stored)
        assertNull(rig.state.staged)
        assertNull(rig.state.verdict)
        assertFalse(rig.state.canApply(armed = false))
    }

    @Test
    fun aLayoutStagedOnOnePartCanBeRefusedOnAnother() = runTest {
        // The 12-FET second motor's map, which only the 64-pin part bonds.
        val rig = layoutRig(BOARD, part = ChipFamily.F103RC)
        for ((field, name) in listOf(
            BoardField.HALL_A to "PC10", BoardField.HALL_B to "PC11", BoardField.HALL_C to "PC12",
            BoardField.GATE_HI_A to "PC6", BoardField.GATE_HI_B to "PC7", BoardField.GATE_HI_C to "PC8",
            BoardField.GATE_LO_A to "PA7", BoardField.GATE_LO_B to "PB0", BoardField.GATE_LO_C to "PB1",
        )) {
            rig.stage(field, pin(name), motor = 1)
        }
        rig.stage(BoardField.DEAD_TIME, 32, motor = 1)
        assertTrue(rig.state.clean)
        assertEquals("TIM8", 1, rig.state.verdict?.plan?.motors?.get(1)?.gates?.timer)

        rig.model.selectPart(ChipFamily.F103C8)
        assertEquals(
            "a 48-pin part does not bond those",
            BoardErrorKind.UnknownPin(checkNotNull(Pin.byName("PC10"))),
            rig.state.verdict?.error?.kind,
        )
    }

    @Test
    fun theLatchIsReportedEvenWhenTheRestOfTheLayoutIsRefused() = runTest {
        // The property the firmware's split exists for, carried through to the screen: a board
        // mis-staged elsewhere still latches its own rail, so it stays powered and reachable.
        val rig = layoutRig(BOARD)
        rig.stage(BoardField.LED_RED, rig.stored(BoardField.LED_GREEN))

        assertEquals(pin("PB12"), rig.state.verdict?.selfHold?.packed)
        assertEquals(FieldRef(BoardField.LED_RED), rig.state.verdict?.error?.field)
    }

    @Test
    fun theLatchFieldIsTheOneFieldTheEditorDoesNotOffer() {
        // The safety rail: on battery that pin is what holds the board's own rail up, so a wrong one
        // powers the board off at boot and only SWD recovers it. It is shown and not editable, and
        // it is the only such row, which makes it a decision rather than an omission.
        val readOnly = LayoutRows.ALL.filter { it.editor == LayoutEditor.ReadOnly }
        assertEquals(1, readOnly.size)
        assertEquals(BoardField.SELF_HOLD, readOnly.single().slot.boardField)
        // And every other field of the layout that the validator can name has an editor.
        val offered = LayoutRows.ALL.filterNot { it.editor == LayoutEditor.ReadOnly }.map { it.slot.key }
        for (slot in Layout.SLOTS) {
            val named = slot.boardField != null && slot.boardField != BoardField.SELF_HOLD
            assertEquals("${slot.key} offered?", named, slot.key in offered)
        }
    }

    @Test
    fun aPresetStagesAWholeKnownGoodLayoutAndItsOwnPart() = runTest {
        // The normal case: one tap, a layout that validates, and no pin entered by hand.
        val rig = layoutRig(BOARD, part = null, linkSet = LayoutPresets.LINK_SET_STANDARD)

        rig.model.stagePreset(LayoutPresets.BENCH_MASTER)

        assertEquals(
            "the preset states the part, so the verdict needs no guess",
            ChipFamily.F103C8,
            rig.state.part,
        )
        assertNull(rig.state.verdict?.error)
        assertEquals(
            "the whole layout, with the facts a pin map cannot state left as the board has them",
            LayoutPresets.BENCH_MASTER.applyTo(checkNotNull(rig.state.stored)),
            rig.state.staged,
        )
        // And what it would write is exactly what this board does not already hold.
        val delta = rig.state.delta.map { it.key }
        assertTrue("the IMU pins are staged", Fields.IMU_SCL_PIN.key() in delta)
        assertTrue("the gate set is staged", Fields.MOTOR_GATE_HI_A.key(0) in delta)
        assertFalse("the fleet pins it already holds are not", Fields.BOARD_BUZZER.key() in delta)
        assertNull("no latch change to confirm", rig.state.pendingLatch)
    }

    @Test
    fun aPresetLeavesTheFactsAPinMapCannotStateAsTheBoardHasThem() = runTest {
        // Drive direction and align offset are a wiring fact and a bench sweep: a preset that
        // overwrote them would undo a bench session with a tap.
        val rig = layoutRig(BOARD, linkSet = LayoutPresets.LINK_SET_STANDARD) {
            preset(BOARD, BoardField.DEAD_TIME, 25, motor = 0)
        }
        rig.transport.store[BOARD to Fields.MOTOR_DIRECTION.key(0)] = Value.U8(1)
        rig.transport.store[BOARD to Fields.MOTOR_ALIGN_OFFSET.key(0)] = Value.U8(3)
        rig.model.refresh()
        runCurrent()

        rig.model.stagePreset(LayoutPresets.BENCH_MASTER)

        val staged = checkNotNull(rig.state.staged)
        assertEquals(1, staged.motors[0].direction)
        assertEquals(3, staged.motors[0].alignOffset)
        assertFalse(Fields.MOTOR_DIRECTION.key(0) in rig.state.delta.map { it.key })
    }

    @Test
    fun aPresetThatMovesThePowerLatchWaitsToBeTold() = runTest {
        // The one true brick, so the one confirmation: the staged layout keeps the pin the board is
        // known to come up on until an operator says otherwise.
        val rig = layoutRig(BOARD, linkSet = LayoutPresets.LINK_SET_STANDARD) {
            preset(BOARD, BoardField.SELF_HOLD, 0x15) // PB5, not the fleet pin
        }

        rig.model.stagePreset(LayoutPresets.BENCH_MASTER)

        assertEquals("the pin it would move to", pin("PB12"), rig.state.pendingLatch)
        assertEquals("staged on the pin the board latches today", pin("PB5"), rig.staged(BoardField.SELF_HOLD))
        assertFalse(Fields.BOARD_SELF_HOLD.key() in rig.state.delta.map { it.key })
        assertNull("the rest of the preset is staged and valid", rig.state.verdict?.error)

        rig.model.confirmLatchChange()
        assertNull(rig.state.pendingLatch)
        assertEquals(pin("PB12"), rig.staged(BoardField.SELF_HOLD))
        assertTrue(Fields.BOARD_SELF_HOLD.key() in rig.state.delta.map { it.key })
    }

    @Test
    fun aLatchChangeCanBeDeclinedWithoutLosingThePreset() = runTest {
        val rig = layoutRig(BOARD, linkSet = LayoutPresets.LINK_SET_STANDARD) {
            preset(BOARD, BoardField.SELF_HOLD, 0x15)
        }
        rig.model.stagePreset(LayoutPresets.BENCH_MASTER)

        rig.model.cancelLatchChange()

        assertNull(rig.state.pendingLatch)
        assertEquals("the board keeps its own latch", pin("PB5"), rig.staged(BoardField.SELF_HOLD))
        assertNull(rig.state.verdict?.error)
        assertTrue("and the rest of the preset is still staged", rig.state.delta.isNotEmpty())
    }

    @Test
    fun aPresetIsRefusedWhileArmedAndBeforeTheLayoutIsRead() = runTest {
        val rig = layoutRig(BOARD, linkSet = LayoutPresets.LINK_SET_STANDARD)
        rig.armed = true
        rig.model.stagePreset(LayoutPresets.OFFROAD_MASTER)
        assertEquals(LayoutNotice.ReadOnlyWhileArmed, rig.state.notice)
        assertEquals(rig.state.stored, rig.state.staged)

        val unread = LayoutRig(this)
        unread.model.stagePreset(LayoutPresets.OFFROAD_MASTER)
        assertEquals(LayoutNotice.NotRead, unread.state.notice)
        assertNull(unread.state.staged)
    }

    @Test
    fun applyingAPresetWritesTheWholeLayoutItDescribes() = runTest {
        val rig = layoutRig(BOARD, linkSet = LayoutPresets.LINK_SET_STANDARD)
        rig.model.stagePreset(LayoutPresets.BENCH_MASTER)
        val delta = rig.state.delta.map { it.key }

        rig.model.apply()
        runCurrent()

        assertEquals("one write per changed field and nothing else", delta, rig.written)
        assertEquals(rig.state.staged, rig.state.stored)
        assertNull(rig.state.storedVerdict?.error)
        assertTrue(rig.state.awaitingPowerCycle)
    }

    private companion object {
        const val BOARD = 0x01
        const val OTHER_BOARD = 0x02
    }
}
