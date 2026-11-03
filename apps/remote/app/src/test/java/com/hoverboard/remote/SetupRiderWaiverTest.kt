package com.hoverboard.remote

import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * What [SetupModel] lets the arm control say about the rider requirement ([SetupState.riderWaiver],
 * `specs/control.md` (i)) across writes, lost answers and reconnects. The field takes effect at every
 * arm (`specs/machine-settings.md` section 5), so the arm this control makes runs what the board
 * stores: the statement is the stored value's, and "may be waived" is left for a stored value the
 * app cannot read after a write of 0 that may have landed. The scheduler is turned as in
 * [SetupModelTest].
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupRiderWaiverTest {

    private val board = 0x01
    private val mode = SetupFields.CONTROL_MODE.key
    private val rider = SetupFields.RIDER_REQUIRED.key

    private fun TestScope.shown(): SetupRig = shownRig(board)

    /**
     * What the arm control would state with every motor controller of the session armed
     * ([SetupState.riderWaiver]): with one board attached, that board's own; with none, nothing.
     */
    private val SetupRig.waiver: RiderWaiver get() = state.riderWaiver(state.motorControllers)

    @Test
    fun `the rider requirement is read on attach and stated from the store`() = runTest {
        val rig = SetupRig(this)
        rig.transport.store[board to rider] = Value.U8(0)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(RiderWaiver.WAIVED, rig.waiver, "never shown, and the arm control still has to know")

        rig.model.onShown()
        runCurrent()
        rig.model.edit(rider, Value.U8(1))
        flush()
        assertEquals(Value.U8(1), rig.state.values[rider])
        assertEquals(RiderWaiver.NONE, rig.waiver, "the arm re-reads the stored 1")

        rig.transport.setAttachedBoard(null)
        assertEquals(RiderWaiver.NONE, rig.waiver, "nothing is known about a board that is not attached")
    }

    /**
     * A write of 0 whose answer was lost and whose read-back failed too: the store may hold 0, and
     * the arm would run it, so the control says the requirement MAY be waived. The next read that
     * answers settles it.
     */
    @Test
    fun `a waiver whose answer was lost and could not be read back may be waived`() = runTest {
        val rig = shown()
        rig.transport.writeHook = { k, v ->
            rig.transport.store[board to k] = v
            rig.transport.unreadable += k
            TimedOut
        }
        rig.model.edit(rider, Value.U8(0))
        flush()
        assertTrue(rider in rig.state.unread, "the read-back was meant to fail")
        assertEquals(WriteState.Unanswered, rig.state.writes[rider]?.state)
        assertEquals(RiderWaiver.POSSIBLY, rig.waiver)

        rig.transport.unreadable.clear()
        rig.model.refresh()
        runCurrent()
        assertEquals(WriteState.Written, rig.state.writes[rider]?.state, "the read pass resolves the landed write")
        assertEquals(RiderWaiver.WAIVED, rig.waiver)
    }

    /** A write of 1 that went unanswered says nothing: no stored 0 can come of it. */
    @Test
    fun `a requirement whose answer was lost is not a waiver`() = runTest {
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
        assertEquals(RiderWaiver.NONE, rig.waiver)
    }

    /** A restart with the Setup screen closed still teaches the arm control what the board stores. */
    @Test
    fun `a restart re-reads the ride facts even with the Setup screen closed`() = runTest {
        val rig = SetupRig(this)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        rig.emitBoot(bootTag = 2)
        runCurrent()
        assertEquals(RiderWaiver.NONE, rig.waiver)

        // The store was waived from elsewhere (an earlier session, or the CLI) and the board has
        // restarted onto it.
        rig.transport.store[board to rider] = Value.U8(0)
        rig.emitBoot(bootTag = 3)
        runCurrent()

        assertEquals(Value.U8(0), rig.state.values[rider], "the ride facts were not re-read after the restart")
        assertEquals(RiderWaiver.WAIVED, rig.waiver)
    }

    @Test
    fun `a board on the default requires a rider, and the arm control says nothing`() = runTest {
        val rig = SetupRig(this)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(Value.U8(1), rig.state.values[rider])
        assertEquals(RiderWaiver.NONE, rig.waiver)
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
        val identifiesBefore = rig.transport.identifies.size

        gate.complete(Unit)
        runCurrent()
        assertEquals(listOf(board), rig.transport.identifies.drop(identifiesBefore), "the identity was skipped")
        assertEquals(
            listOf(board to SetupFields.DEVICE_NAME.key) +
                SetupFields.MOTOR_FACTS.map { board to it.key } +
                listOf(
                    board to Fields.MOTOR_GATE_HI_A.key(1),
                    board to Fields.MOTOR_CURRENT_CAL.key(),
                    board to Fields.MOTOR_POLE_PAIRS.key(),
                    board to Fields.WHEEL_DIAMETER_MM.key(),
                ),
            rig.transport.reads.drop(readsBefore),
            "the ride facts were skipped",
        )
        assertEquals(RiderWaiver.WAIVED, rig.waiver)
    }

    /**
     * The rider requirement is a machine field: read once, it stands for every motor controller
     * (`specs/machine-settings.md` section 7). Once a board reports the copies disagree
     * (`EV_MACHINE`), each board's own copy is read, and the arm control states the strongest of the
     * boards it arms (`specs/remote-ui.md` 3.4): a board that would arm waived is never hidden behind
     * one that requires a rider.
     */
    @Test
    fun `the waiver is the machine's until the boards disagree, then each board's own`() = runTest {
        val other = 0x02
        val rig = SetupRig(this)
        rig.transport.store[board to rider] = Value.U8(0)
        rig.transport.store[other to rider] = Value.U8(1)
        rig.transport.setAttachedBoard(board, other)
        runCurrent()
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiverOf(other), "read once, from the first board")

        rig.transport.emitCyclicState(QUIET_STATE.copy(fault = CyclicState.EV_MACHINE), other)
        runCurrent()

        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiverOf(board))
        assertEquals(RiderWaiver.NONE, rig.state.riderWaiverOf(other))
        assertEquals(RiderWaiver.NONE, rig.state.riderWaiver(listOf(other)), "arming only the other board")
        assertEquals(RiderWaiver.WAIVED, rig.state.riderWaiver(listOf(board, other)), "arming both")
    }
}
