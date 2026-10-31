package com.hoverboard.remote

import com.hoverboard.protocol.board.ChipFamily
import com.hoverboard.protocol.board.DEAD_TIME_MIN_DTG
import com.hoverboard.protocol.board.Layout
import com.hoverboard.protocol.board.Pin
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.Refused
import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.store.Fields
import com.hoverboard.remote.model.LayoutEditor
import com.hoverboard.remote.model.LayoutRows
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
 * The board layout editor's model (`specs/remote-ui.md` section 3.5).
 *
 * The properties here: the layout is read as one object, Apply writes the delta and nothing else
 * whenever the board is attached and disarmed, nothing judges the staged layout first (the board
 * reports what it could not bring up in `board.status` after the reboot), and a half-written layout
 * is finished by applying again.
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
        // Every field of the layout was asked for.
        assertTrue(Layout.SLOTS.all { it.key in rig.transport.reads.map { r -> r.second } })
        assertFalse("nothing to write", rig.state.canApply(armed = false))
    }

    @Test
    fun thePartComesFromTheBoardsOwnTagAndApplyDoesNotNeedIt() = runTest {
        // The part is what the pin picker offers pins of. A board that has sent nothing yet, or a
        // tag this app models no part for, has none; the layout can still be written.
        val rig = layoutRig(BOARD, chip = null)

        assertNotNull(rig.state.stored)
        assertNull(rig.state.part)
        rig.stage(Fields.BOARD_BUZZER, pin("PB5"))
        assertTrue(rig.state.canApply(armed = false))

        rig.reports(ChipTag.Unknown)
        runCurrent()
        assertEquals("the tag is recorded as the board sent it", ChipTag.Unknown, rig.state.chip)
        assertNull("a tag this app models no part for is no part", rig.state.part)

        rig.reports(ChipTag.F103C8)
        runCurrent()
        assertEquals(ChipFamily.F103C8, rig.state.part)

        rig.model.apply()
        runCurrent()
        assertEquals(listOf(Fields.BOARD_BUZZER.key()), rig.written)
    }

    @Test
    fun nothingJudgesTheLayoutBeforeItIsWritten() = runTest {
        // A pin another function would want, a gate-capable pin on an LED and a half-staged hall
        // group: the board brings up what it can and lists the rest in its status after the reboot,
        // so Apply writes them as staged.
        val rig = layoutRig(BOARD)
        rig.stage(Fields.BOARD_BUZZER, pin("PA2"))
        rig.stage(Fields.LED_GREEN, pin("PA8"))
        rig.stage(Fields.MOTOR_HALL_A, pin("PC13"), motor = 0)

        assertTrue(rig.state.canApply(armed = false))
        rig.model.apply()
        runCurrent()

        assertEquals(
            listOf(Fields.BOARD_BUZZER.key(), Fields.LED_GREEN.key(), Fields.MOTOR_HALL_A.key(0)),
            rig.written,
        )
        assertNull(rig.state.notice)
        assertTrue(rig.state.awaitingPowerCycle)
    }

    @Test
    fun applyWritesTheDeltaInFieldOrderAndMarksItUnapplied() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(Fields.BOARD_BUZZER, pin("PB5"))
        rig.stage(Fields.LED_RED, pin("PB1"))

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
    }

    @Test
    fun aWriteRefusedMidApplyLeavesTheRestStagedAndApplyingAgainFinishesIt() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(Fields.MOTOR_HALL_A, pin("PC13"), motor = 0)
        rig.stage(Fields.MOTOR_HALL_B, pin("PA1"), motor = 0)
        rig.stage(Fields.MOTOR_HALL_C, pin("PC14"), motor = 0)
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
        assertEquals("the rest is still to write", 2, rig.state.delta.size)

        // Applying again finishes it: the delta is measured from what the board holds, so the field
        // that went through is not written twice.
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
        assertEquals(emptyList<Any>(), rig.state.delta)
    }

    @Test
    fun anUnansweredWriteReadsTheFieldBackRatherThanAssumingIt() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(Fields.BOARD_BUZZER, pin("PB5"))
        // The write is stored, its answer is lost: the board holds the new value and the app does
        // not know it. The read-back is what settles it, and the delta then empties on its own.
        rig.transport.writeHook = { key, value ->
            TimedOut.also { rig.transport.store[BOARD to key] = value }
        }

        rig.model.apply()
        runCurrent()

        assertEquals(LayoutNotice.Unanswered(Fields.BOARD_BUZZER.key()), rig.state.notice)
        assertEquals("read back, not assumed", pin("PB5"), rig.stored(Fields.BOARD_BUZZER))
        assertEquals(emptyList<Any>(), rig.state.delta)
    }

    @Test
    fun armedRefusesEveryEditAndEveryWrite() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(Fields.BOARD_BUZZER, pin("PB5"))
        rig.armed = true

        rig.stage(Fields.LED_RED, pin("PB1"))
        assertEquals(LayoutNotice.ReadOnlyWhileArmed, rig.state.notice)
        assertEquals("the edit was refused, not staged", rig.stored(Fields.LED_RED), rig.staged(Fields.LED_RED))
        assertFalse(rig.state.canApply(armed = true))

        rig.model.apply()
        runCurrent()
        assertEquals(LayoutNotice.ReadOnlyWhileArmed, rig.state.notice)
        assertEquals(emptyList<Any>(), rig.transport.writes)
    }

    @Test
    fun noBoardAttachedWritesNothing() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(Fields.BOARD_BUZZER, pin("PB5"))
        rig.transport.setAttachedBoard(null)
        runCurrent()

        assertFalse(rig.state.canApply(armed = false))
        rig.model.apply()
        runCurrent()
        assertEquals(LayoutNotice.NotAttached, rig.state.notice)
        assertEquals(emptyList<Any>(), rig.transport.writes)
    }

    @Test
    fun revertPutsAFieldAndTheWholeLayoutBack() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(Fields.BOARD_BUZZER, pin("PB5"))
        rig.stage(Fields.LED_RED, pin("PB1"))

        rig.model.revert(rig.slot(Fields.BOARD_BUZZER))
        assertEquals(rig.stored(Fields.BOARD_BUZZER), rig.staged(Fields.BOARD_BUZZER))
        assertEquals(1, rig.state.delta.size)

        rig.model.revertAll()
        assertEquals(rig.state.stored, rig.state.staged)
        assertEquals(emptyList<Any>(), rig.state.delta)
    }

    @Test
    fun aDifferentBoardDropsTheLayoutReadFromTheOldOne() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(Fields.BOARD_BUZZER, pin("PB5"))

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(OTHER_BOARD)
        runCurrent()

        assertEquals(LayoutNotice.BoardChanged(BOARD), rig.state.notice)
        assertEquals(OTHER_BOARD, rig.state.board)
        assertNull("the part went with the layout: it was the other board's fact", rig.state.chip)

        rig.reports(ChipTag.F130C8)
        runCurrent()
        assertEquals("and comes back as the board that is attached now reports it", ChipFamily.F130C8, rig.state.part)
    }

    @Test
    fun aRestartOnTheWireClearsTheWrittenMarksAndReadsAgain() = runTest {
        val rig = layoutRig(BOARD)
        rig.stage(Fields.BOARD_BUZZER, pin("PB5"))
        rig.model.apply()
        runCurrent()
        assertTrue(rig.state.awaitingPowerCycle)

        // More state from the same boot is not a restart, and nothing is asked of the operator.
        rig.reports(ChipTag.F103C8)
        runCurrent()
        assertEquals("same boot, same marks", setOf(Fields.BOARD_BUZZER.key()), rig.state.written)

        // The power-cycle: the link goes down with the board and comes back on the next boot.
        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(BOARD)
        runCurrent()
        assertEquals("a drop alone is not a restart", setOf(Fields.BOARD_BUZZER.key()), rig.state.written)
        rig.reports(ChipTag.F103C8, boot = 2)
        runCurrent()

        assertEquals(emptySet<Any>(), rig.state.written)
        assertFalse(rig.state.awaitingPowerCycle)
        assertEquals("re-read after the reboot", pin("PB5"), rig.stored(Fields.BOARD_BUZZER))
    }

    @Test
    fun anUnreadableFieldMeansNoLayoutAtAll() = runTest {
        // A layout is one object: a field that could not be read is not a layout with a gap in it,
        // so there is nothing to edit.
        val rig = LayoutRig(this)
        rig.transport.unreadable += Fields.PAD_B.key()
        rig.transport.setAttachedBoard(BOARD)
        rig.model.onShown()
        runCurrent()
        rig.reports(ChipTag.F103C8)
        runCurrent()

        assertEquals(setOf(Fields.PAD_B.key()), rig.state.unread)
        assertNull(rig.state.stored)
        assertNull(rig.state.staged)
        assertFalse(rig.state.canApply(armed = false))
    }

    /**
     * The dead-time row offers what the board takes and nothing else: the floor upwards, plus the
     * unset 0 that a motor with no gates holds. A row that offered 1 would let an operator stage
     * 28 ns of dead time, which is shoot-through; the board would leave the motor down and say so
     * in its status. (`SetupFieldsTest` holds the Setup screen's row of the same field to the rule.)
     */
    @Test
    fun theDeadTimeRowOffersOnlyWhatTheBoardTakes() {
        val row = LayoutRows.ALL
            .single { it.slot.def == Fields.MOTOR_DEAD_TIME && it.slot.motor == 0 }
            .editor
        assertTrue("the row is a number editor", row is LayoutEditor.Number)
        val number = row as LayoutEditor.Number
        for (dtg in listOf(0L, DEAD_TIME_MIN_DTG.toLong(), 25L, 28L, 32L, 255L)) {
            assertTrue("the row takes $dtg", number.accepts(dtg))
        }
        for (dtg in listOf(1L, DEAD_TIME_MIN_DTG - 1L, 256L, -1L)) {
            assertFalse("the row refuses $dtg", number.accepts(dtg))
        }
    }

    @Test
    fun theLatchFieldIsTheOneFieldTheEditorDoesNotOffer() {
        // The safety rail: on battery that pin is what holds the board's own rail up, so a wrong one
        // powers the board off at boot and only SWD recovers it. It is shown and not editable, and
        // it is the only such row.
        val readOnly = LayoutRows.ALL.filter { it.editor == LayoutEditor.ReadOnly }
        assertEquals(1, readOnly.size)
        assertEquals(Fields.BOARD_SELF_HOLD, readOnly.single().slot.def)
        // And every other field of the layout has an editor, but the three carried motor facts,
        // which are the Setup screen's rows.
        val carried = setOf(Fields.MOTOR_DIRECTION, Fields.MOTOR_ALIGN_OFFSET, Fields.MOTOR_CURRENT_CAL)
        val offered = LayoutRows.ALL.filterNot { it.editor == LayoutEditor.ReadOnly }.map { it.slot.key }
        for (slot in Layout.SLOTS) {
            val edited = slot.def !in carried && slot.def != Fields.BOARD_SELF_HOLD
            assertEquals("${slot.key} offered?", edited, slot.key in offered)
        }
    }

    @Test
    fun applyingAWholeLayoutWritesEveryFieldItChangesAndNothingElse() = runTest {
        // A board taken from the blank defaults to a full single-motor layout in one Apply: battery
        // sense, the IMU, halls, the six gates with their dead time, and the phase-current pair.
        val rig = layoutRig(BOARD)
        rig.stage(Fields.BOARD_BATTERY_PIN, pin("PA4"))
        rig.stage(Fields.IMU_SCL_PIN, pin("PB6"))
        rig.stage(Fields.IMU_SDA_PIN, pin("PB7"))
        rig.stage(Fields.IMU_MODEL, Fields.ImuModel.CLONE_2E)
        for ((field, name) in listOf(
            Fields.MOTOR_HALL_A to "PC13", Fields.MOTOR_HALL_B to "PA1", Fields.MOTOR_HALL_C to "PC14",
            Fields.MOTOR_GATE_HI_A to "PA8", Fields.MOTOR_GATE_HI_B to "PA9", Fields.MOTOR_GATE_HI_C to "PA10",
            Fields.MOTOR_GATE_LO_A to "PB13", Fields.MOTOR_GATE_LO_B to "PB14", Fields.MOTOR_GATE_LO_C to "PB15",
            Fields.MOTOR_PHASE_A to "PA0", Fields.MOTOR_PHASE_B to "PB0",
        )) {
            rig.stage(field, pin(name), motor = 0)
        }
        rig.stage(Fields.MOTOR_DEAD_TIME, 25, motor = 0)
        rig.stage(Fields.MOTOR_CURRENT_SENSE, 1, motor = 0)
        val delta = rig.state.delta.map { it.key }
        assertFalse("the fleet pins it already holds are not in it", Fields.BOARD_BUZZER.key() in delta)

        rig.model.apply()
        runCurrent()

        assertEquals("one write per changed field and nothing else", delta, rig.written)
        assertEquals(rig.state.staged, rig.state.stored)
        assertTrue(rig.state.awaitingPowerCycle)
    }

    private companion object {
        const val BOARD = 0x01
        const val OTHER_BOARD = 0x02
    }
}
