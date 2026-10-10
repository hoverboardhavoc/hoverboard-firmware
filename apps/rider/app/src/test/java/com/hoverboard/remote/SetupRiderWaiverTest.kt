package com.hoverboard.remote

import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
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
     * runs with the field at 0. What the board RUNS is what it last read: a write this session
     * changes the store, and the board takes the field up at its next arm or boot, so until then the
     * arm control still has to warn about the value in force.
     */
    @Test
    fun `the rider requirement the board runs is read on attach and survives a write`() = runTest {
        val rig = SetupRig(this)
        rig.transport.store[board to rider] = Value.U8(0)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        val never = "never shown, and the arm control still has to know"
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver, never)

        rig.model.onShown()
        runCurrent()
        rig.emitBoot(bootTag = 7)
        runCurrent()
        rig.model.edit(rider, Value.U8(1))
        flush()
        assertEquals(Value.U8(1), rig.state.values[rider])
        val runs = "the board runs the waived value until it takes the new one up"
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver, runs)

        rig.transport.setAttachedBoard(null)
        assertEquals(RiderWaiver.NONE, rig.state.riderWaiver, "nothing is known about a board that is not attached")
        rig.transport.setAttachedBoard(board)
        runCurrent()
        // The board restarted, which the wire says and nothing asks the operator to assert.
        rig.emitBoot(bootTag = 8)
        runCurrent()
        assertEquals(RiderWaiver.NONE, rig.state.riderWaiver)
        assertEquals(Value.U8(1), rig.state.running[rider])
    }

    /**
     * The board booted waived, the operator wrote 1, and the link dropped and came back with no
     * restart: the board still runs waived and arming still engages, but the app can no longer tell
     * (a drop is not a restart, and a key written this session is not re-read as running). The arm
     * control says the requirement MAY be waived rather than nothing.
     */
    @Test
    fun `a reconnect after writing the rider requirement says it may still be waived`() = runTest {
        val rig = SetupRig(this)
        rig.transport.store[board to rider] = Value.U8(0)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        rig.emitBoot(bootTag = 7)
        runCurrent()
        rig.model.edit(rider, Value.U8(1))
        flush()
        rig.model.onHidden()

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(Value.U8(1), rig.state.values[rider], "the ride facts re-read the store")
        assertNull(rig.state.running[rider], "a key written this session is not re-read as running")
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)

        // The same board on a later boot: it now runs the stored 1.
        rig.model.onShown()
        runCurrent()
        rig.emitBoot(bootTag = 8)
        runCurrent()
        assertEquals(Value.U8(1), rig.state.running[rider])
        assertEquals(RiderWaiver.NONE, rig.state.riderWaiver)
    }

    /**
     * The other way round: the board booted requiring a rider, the operator wrote 0, and the link
     * dropped. The board may have restarted in the gap and now run the stored 0.
     */
    @Test
    fun `a reconnect after waiving the rider requirement says it may be waived`() = runTest {
        val rig = shown()
        rig.model.edit(rider, Value.U8(0))
        flush()
        assertEquals(RiderWaiver.NONE, rig.state.riderWaiver, "the board still runs the 1 it booted with")

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
     * lost. The read-back after the timeout finds the app's own 1: that is the store, not what the
     * board runs, so the arm control still says the board runs waived.
     */
    @Test
    fun `the read-back of a write whose answer was lost is the store, not what the board runs`() = runTest {
        val rig = SetupRig(this)
        rig.transport.store[board to rider] = Value.U8(0)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        rig.loseAnswer(rider)
        rig.model.edit(rider, Value.U8(1))
        flush()
        assertEquals(WriteState.Written, rig.state.writes[rider]?.state, "the read-back found the stored 1")
        assertEquals(Value.U8(1), rig.state.values[rider])
        assertEquals(Value.U8(0), rig.state.running[rider], "the stored 1 was taken as running")
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver)

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver, "the 0 the board ran was lost on the drop")
    }

    /**
     * As above, but the read-back fails too, so the first read of the stored 1 is the ride-facts read
     * on reconnect, while the row still reports the write unanswered. The Setup read pass then finds
     * the value in the store and resolves it as written.
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
        rig.model.edit(rider, Value.U8(1))
        flush()
        assertTrue(rider in rig.state.unread, "the read-back was meant to fail")
        assertEquals(WriteState.Unanswered, rig.state.writes[rider]?.state)
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver)

        rig.transport.unreadable.clear()
        rig.model.onHidden()
        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(WriteState.Unanswered, rig.state.writes[rider]?.state, "only the ride facts were read")
        assertNull(rig.state.running[rider], "the stored 1 was taken as running")
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)

        rig.model.onShown()
        runCurrent()
        val resolved = "the Setup read pass resolves the landed write"
        assertEquals(WriteState.Written, rig.state.writes[rider]?.state, resolved)
        assertNull(rig.state.running[rider])
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)
    }

    /** The mirror: booted requiring a rider, a write of 0 whose answer was lost. */
    @Test
    fun `a waiver whose answer was lost is not taken as running`() = runTest {
        val rig = shown()
        rig.loseAnswer(rider)
        rig.model.edit(rider, Value.U8(0))
        flush()
        assertEquals(Value.U8(0), rig.state.values[rider], "the read-back found the stored 0")
        assertEquals(RiderWaiver.NONE, rig.state.riderWaiver, "the board still runs the 1 it booted with")

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertNull(rig.state.running[rider])
        assertEquals(RiderWaiver.POSSIBLY, rig.state.riderWaiver)
    }

    /** A restart with the Setup screen closed still teaches the arm control what the board now runs. */
    @Test
    fun `a restart re-reads the ride facts even with the Setup screen closed`() = runTest {
        val rig = SetupRig(this)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        rig.emitBoot(bootTag = 2)
        runCurrent()
        assertEquals(Value.U8(1), rig.state.running[rider])

        // The store was waived from elsewhere (an earlier session, or over SWD) and the board has
        // restarted onto it.
        rig.transport.store[board to rider] = Value.U8(0)
        rig.emitBoot(bootTag = 3)
        runCurrent()

        assertEquals(Value.U8(0), rig.state.running[rider], "the ride facts were not re-read after the restart")
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver)
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
     * A previous session's write still holds the one-operation lock when the board re-attaches: the
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
        rig.model.edit(mode, Value.U8(1))
        flush()
        assertTrue(rig.state.writing, "the write is held mid-flight")

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        val readsBefore = rig.transport.reads.size

        gate.complete(Unit)
        runCurrent()
        assertEquals(listOf(board to rider), rig.transport.reads.drop(readsBefore), "the ride facts were skipped")
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver)
    }
}
