package com.hoverboard.remote

import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * What [SetupModel] lets the arm control say about the rider requirement ([SetupState.riderWaiver],
 * `specs/control.md` (i)): what the board runs, as against what it stores, across writes, lost
 * answers and reconnects. The scheduler is turned as in [SetupModelTest].
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupRiderWaiverTest {

    private val board = 0x01
    private val mode = SetupFields.CONTROL_MODE.key
    private val rider = SetupFields.RIDER_REQUIRED.key

    private fun TestScope.shown(): SetupRig = shownRig(board)

    /**
     * `specs/control.md` (i): the arm control says the rider requirement is waived when the board
     * runs with the field at 0, and it is certain of that only while the booted value and the
     * stored one agree. The field is re-read at every arm, so a write moves the board off the
     * booted value at a moment the app does not see, and the control drops to "may be waived"
     * rather than claiming either.
     */
    @Test
    fun `the rider requirement the board runs is read on attach and goes uncertain on a write`() =
        runTest {
            val rig = SetupRig(this)
            rig.transport.store[board to rider] = Value.U8(0)
            rig.transport.setAttachedBoard(board)
            runCurrent()
            val never = "never shown, and the arm control still has to know"
            assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver, never)

            rig.model.onShown()
            runCurrent()
            rig.model.stage(rider, Value.U8(1))
            rig.model.apply()
            runCurrent()
            assertEquals(Value.U8(1), rig.state.values[rider])
            val runs = "the board waives until its next arm and requires from then on, and nothing says which"
            assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver, runs)

            rig.transport.setAttachedBoard(null)
            assertEquals(RiderWaiver.NONE, rig.state.riderWaiver, "nothing is known about a board that is not attached")
        }

    /**
     * The board booted waived, the operator wrote 1, and the link dropped and came back: the board
     * may still be waiving and arming may still engage, and the app cannot tell. A key written this
     * session is not re-read as running, a drop is not a power-cycle, and the arm it would be
     * re-read at is not something the app sees either. The arm control says the requirement MAY be
     * waived rather than nothing.
     */
    @Test
    fun `a reconnect after writing the rider requirement says it may still be waived`() = runTest {
        val rig = SetupRig(this)
        rig.transport.store[board to rider] = Value.U8(0)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        rig.model.stage(rider, Value.U8(1))
        rig.model.apply()
        runCurrent()
        rig.model.onHidden()

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(Value.U8(1), rig.state.values[rider], "the ride facts re-read the store")
        assertNull(rig.state.running[rider], "a key written this session is not re-read as running")
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)
    }

    /**
     * The other way round: the board booted requiring a rider, the operator wrote 0, and the link
     * dropped. The board may have been power-cycled in the gap and now run the stored 0.
     */
    @Test
    fun `a reconnect after waiving the rider requirement says it may be waived`() = runTest {
        val rig = shown()
        rig.model.stage(rider, Value.U8(0))
        rig.model.apply()
        runCurrent()
        val arms = "the board requires a rider until its next arm and waives from then on"
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver, arms)

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)
    }

    /** Every write to [key] is stored by the board, and its answer is lost. */
    private fun SetupRig.loseAnswer(key: Key) {
        transport.writeHook = { k, v ->
            if (k == key) {
                transport.store[board to k] = v
                TimedOut
            } else {
                null
            }
        }
    }

    /**
     * The board booted waived; the operator wrote 1 and the board stored it, but the answer was
     * lost. The read-back after the timeout finds the app's own unverified 1: that is the store,
     * not what the board runs, so the running map keeps the 0 the board booted with. The control
     * still says the requirement MAY be waived, because the store and the booted value have come
     * apart and the next arm is what resolves them.
     */
    @Test
    fun `the read-back of a write whose answer was lost is the store, not what the board runs`() = runTest {
        val rig = SetupRig(this)
        rig.transport.store[board to rider] = Value.U8(0)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        rig.loseAnswer(rider)
        rig.model.stage(rider, Value.U8(1))
        rig.model.apply()
        runCurrent()
        assertEquals(SetupNotice.Unanswered(rider), rig.state.notice)
        assertEquals(Value.U8(1), rig.state.values[rider], "the read-back found the stored 1")
        assertEquals(Value.U8(0), rig.state.running[rider], "the stored 1 was taken as running")
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver, "the 0 the board ran was lost on the drop")
    }

    /**
     * As above, but the read-back fails too, so the first read of the stored 1 is the ride-facts read
     * on reconnect, while the key is still pending (it moves to staged only after a Setup read pass).
     */
    @Test
    fun `the ride facts on reconnect do not take a write whose answer was lost as running`() = runTest {
        val rig = SetupRig(this)
        rig.transport.store[board to rider] = Value.U8(0)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        rig.transport.writeHook = { k, v ->
            rig.transport.store[board to k] = v
            rig.transport.unreadable += k
            TimedOut
        }
        rig.model.stage(rider, Value.U8(1))
        rig.model.apply()
        runCurrent()
        assertTrue(rider in rig.state.unread, "the read-back was meant to fail")
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver, "the board booted waived, and the write may be live")

        rig.transport.unreadable.clear()
        rig.model.onHidden()
        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(mapOf(rider to Value.U8(1)), rig.state.pending, "only the ride facts were read")
        assertNull(rig.state.running[rider], "the stored 1 was taken as running")
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)

        rig.model.onShown()
        runCurrent()
        assertEquals(mapOf(rider to Value.U8(1)), rig.state.staged, "the Setup read pass stages the landed write")
        assertNull(rig.state.running[rider])
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)
    }

    /** The mirror: booted requiring a rider, a write of 0 whose answer was lost. */
    @Test
    fun `a waiver whose answer was lost is not taken as running`() = runTest {
        val rig = shown()
        rig.loseAnswer(rider)
        rig.model.stage(rider, Value.U8(0))
        rig.model.apply()
        runCurrent()
        assertEquals(Value.U8(0), rig.state.values[rider], "the read-back found the stored 0")
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver, "the stored 0 runs from the next arm")

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertNull(rig.state.running[rider])
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)
    }

    @Test
    fun `a board on the default requires a rider, and the arm control says nothing`() = runTest {
        val rig = SetupRig(this)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(Value.U8(1), rig.state.running[rider])
        assertEquals(RiderWaiver.NONE, rig.state.riderWaiver)
    }

    /**
     * A previous session's Apply still holds the one-operation lock when the board re-attaches: the
     * ride-facts read waits for it rather than being skipped, so the arm control learns the rider
     * requirement without the Setup screen ever being opened again.
     */
    @Test
    fun `the ride facts are read once a held operation frees the lock`() = runTest {
        val rig = shown()
        rig.transport.store[board to rider] = Value.U8(0)
        rig.model.onHidden()
        val gate = CompletableDeferred<Unit>()
        rig.transport.writeGate = gate
        rig.model.stage(mode, Value.U8(1))
        rig.model.apply()
        runCurrent()
        assertTrue(rig.state.applying, "the Apply is held mid-write")

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        val readsBefore = rig.transport.reads.size

        gate.complete(Unit)
        runCurrent()
        assertEquals(
            listOf(board to rider) + SetupFields.MACHINE.map { board to it.key },
            rig.transport.reads.drop(readsBefore),
            "the ride facts were skipped",
        )
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver)
    }

    /** The arm control arms the master, so the waiver it states is the master's whichever board Setup shows. */
    @Test
    fun `the waiver is the master's whichever board is selected`() = runTest {
        val slave = 0x02
        val rig = SetupRig(this)
        rig.transport.store[board to rider] = Value.U8(0)
        rig.transport.store[slave to rider] = Value.U8(1)
        rig.transport.setAttachedBoard(board, slave)
        rig.model.onShown()
        runCurrent()

        rig.model.selectTarget(Node.SLAVE)
        runCurrent()

        assertEquals(Value.U8(1), rig.state.values[rider], "the slave's own value is shown")
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver, "the waiver is still the master's")
    }
}
