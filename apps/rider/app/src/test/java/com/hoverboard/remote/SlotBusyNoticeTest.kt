package com.hoverboard.remote

import com.hoverboard.protocol.config.Busy
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * A request slot that stayed taken through the whole [awaitSlot] wait is said as such by both
 * models: nothing was sent, so it is neither a board answer the app failed to understand nor a
 * board that failed to answer.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SlotBusyNoticeTest {

    private val board = 0x01
    private val mode = SetupFields.CONTROL_MODE.key
    private val kpA = Gains.key(Gains.CONTROL_GAIN_A, Gains.KP)

    @Test
    fun `Setup says the slot stayed taken, not that the answer was garbled, and Apply again writes`() = runTest {
        val rig = shownRig(board)
        rig.transport.writeHook = { _, _ -> Busy }
        rig.model.stage(mode, Value.U8(1))
        rig.model.apply()
        runCurrent()
        advanceTimeBy(SLOT_RETRY_MS * (SLOT_RETRIES + 1))
        runCurrent()
        assertEquals(SetupNotice.SlotBusy, rig.state.notice)
        assertEquals(Value.U8(1), rig.state.pending[mode], "the unwritten value left the basket")
        assertEquals(SetupFields.CONTROL_MODE.def.default, rig.state.values[mode])

        rig.transport.writeHook = { _, _ -> null }
        rig.model.apply()
        runCurrent()
        assertNull(rig.state.notice)
        assertTrue(rig.state.pending.isEmpty())
        assertEquals(Value.U8(1), rig.state.values[mode])
    }

    @Test
    fun `a Tune pass the taken slot cut short leaves read values as they were, not unreachable`() = runTest {
        val rig = shownTune(slave = null)
        assertFalse(rig.state.gains.stale)

        rig.transport.tuneBusy = SLOT_RETRIES + 1
        rig.model.refresh()
        runCurrent()
        advanceTimeBy(SLOT_RETRY_MS * (SLOT_RETRIES + 1))
        runCurrent()
        assertEquals(TuneNotice.SlotBusy, rig.state.notice)
        assertFalse(rig.state.gains.stale, "a board that was not asked was marked unreachable")
        assertTrue(rig.state.writable)
    }

    @Test
    fun `a Tune pass the taken slot cut short before any read leaves the gains unread`() = runTest {
        val rig = TuneRig(this)
        rig.transport.tuneBusy = SLOT_RETRIES + 1
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        advanceTimeBy(SLOT_RETRY_MS * (SLOT_RETRIES + 1))
        runCurrent()
        assertEquals(TuneNotice.SlotBusy, rig.state.notice)
        assertTrue(rig.state.gains.stale, "values never read were shown as current")
    }

    @Test
    fun `a Tune pass with an unanswered request still marks the values stale`() = runTest {
        val rig = shownTune(slave = null)
        assertFalse(rig.state.gains.stale)

        rig.transport.tuneSilent += board
        rig.model.refresh()
        runCurrent()
        assertEquals(TuneNotice.Unanswered(kpA), rig.state.notice)
        assertTrue(rig.state.gains.stale)
        assertFalse(rig.state.writable)
    }
}
