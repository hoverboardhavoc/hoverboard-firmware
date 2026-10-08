package com.hoverboard.remote

import com.hoverboard.protocol.config.Busy
import com.hoverboard.protocol.config.ReadValue
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Test

/**
 * The session's one request slot, shared by the Setup and Tune models ([awaitSlot]): a model that
 * finds it taken by the other waits for it instead of losing its request.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class RequestSlotTest {

    @Test
    fun `a call answered Busy is retried until the slot frees`() = runTest {
        var busy = 2
        var calls = 0
        val r = awaitSlot {
            calls++
            if (busy-- > 0) Busy else ReadValue(Value.U8(1))
        }
        assertEquals(ReadValue(Value.U8(1)), r)
        assertEquals(3, calls)
    }

    @Test
    fun `a slot that never frees gives up with Busy after the retry budget`() = runTest {
        var calls = 0
        assertEquals(Busy, awaitSlot { calls++; Busy })
        assertEquals(SLOT_RETRIES + 1, calls)
    }

    @Test
    fun `Setup's ride-facts read survives the Tune model holding the slot at attach`() = runTest {
        val rig = SetupRig(this)
        rig.transport.configBusy = 2
        rig.transport.setAttachedBoard(0x01)
        runCurrent()
        advanceTimeBy(SLOT_RETRY_MS * 3)
        assertEquals(SetupFields.RIDER_REQUIRED.def.default, rig.state.values[SetupFields.RIDER_REQUIRED.key])
    }

    @Test
    fun `Tune says the slot stayed taken, not that the board's answer was garbled, and a retry reads`() = runTest {
        val rig = TuneRig(this)
        rig.transport.tuneBusy = SLOT_RETRIES + 1
        rig.transport.setAttachedBoard(0x01)
        rig.model.onShown()
        runCurrent()
        advanceTimeBy(SLOT_RETRY_MS * (SLOT_RETRIES + 1))
        assertEquals(TuneNotice.SlotBusy, rig.state.notice)
        assertEquals(0, rig.transport.tuneReads.size, "a read reached the board")

        rig.model.refresh()
        runCurrent()
        assertEquals(null, rig.state.notice)
        assertFalse(rig.state.gains.stale)
        assertEquals(6000, rig.state.gains.staged[Gains.key(Gains.CONTROL_GAIN_A, Gains.KP)])
    }
}
