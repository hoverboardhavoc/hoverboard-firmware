package com.hoverboard.remote

import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * What a dragged gain slider puts on the tune lane (`specs/rider-ui.md` section 3.3a).
 *
 * The slider is an instrument change, not a behaviour change: a gain already applied the moment it
 * was staged, and still does. What has to hold is the lane's side of it. A drag emits a value per
 * frame and the lane will not take that (one request slot shared with the config lane, a settle
 * window after a retransmit, and the CC2541's metered UART), so the model sends on a cadence; and
 * the value the gesture ENDS at is the committed one, so it is never left to where a sample fell.
 *
 * Virtual time is advanced with `advanceTimeBy` throughout, never `advanceUntilIdle`: the drag's
 * send loop is background work, and `advanceUntilIdle` will not move the clock for background work
 * alone (a test that used it would read as if the drag had sent nothing).
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneDragTest {

    private val master = 0x01
    private val kpA = Gains.key(Gains.CONTROL_GAIN_A, Gains.KP)
    private val maxKp = Key(Fields.CONTROL_GAIN_MAX.id, Gains.KP)

    /** One cadence interval and a tick, which is as long as any single send has to wait. */
    private val interval = TuneModel.DRAG_SEND_INTERVAL_MS + 1

    @Test
    fun `a drag sends on the cadence, not once per frame`() = runTest {
        val rig = shownTune()

        // One second of a 60 Hz drag: 60 samples, which at one write each would be 60 TUNE_WRITEs.
        dragGain(rig, Gains.KP, to = 9000, samples = 60, frameMs = 16)

        val writes = rig.transport.tuneWrites.size
        val cadence = (1_000 / TuneModel.DRAG_SEND_INTERVAL_MS).toInt()
        // The samples of one second, plus the release: nothing above the cadence it was sent at.
        assertTrue(writes <= cadence + 1, "the drag sent $writes writes, above the cadence")
        assertTrue(writes > 1, "the drag sent $writes writes: nothing was streamed during the drag")
        assertEquals(9000, rig.state.gains.staged[kpA], "the value let go at is not what the board holds")
    }

    @Test
    fun `the value the finger lets go at is staged exactly, whatever the cadence sampled`() = runTest {
        val rig = shownTune()

        // The release lands between two cadence samples, which is where a dropped release shows up.
        rig.model.slide(Gains.KP, 7000)
        runCurrent()
        advanceTimeBy(TuneModel.DRAG_SEND_INTERVAL_MS / 2)
        rig.model.slide(Gains.KP, 8000)
        rig.model.slideEnd(Gains.KP, 8137)
        advanceTimeBy(interval)
        runCurrent()

        assertEquals(8137, rig.transport.tuneWrites.last().third)
        assertEquals(8137, rig.state.gains.staged[kpA])
        assertEquals(master to kpA, rig.transport.tuneReads.last(), "the settled value was not re-read")
        assertFalse(rig.state.writing, "the drag did not let go of the lane")
    }

    @Test
    fun `a release at the value the last sample already sent is still re-read`() = runTest {
        val rig = shownTune()

        rig.model.slide(Gains.KP, 7000)
        runCurrent()
        val reads = rig.transport.tuneReads.size
        rig.model.slideEnd(Gains.KP, 7000) // the finger held still before lifting
        advanceTimeBy(interval)
        runCurrent()

        assertEquals(1, rig.transport.tuneWrites.size, "the release re-wrote a value already staged")
        assertEquals(listOf(master to kpA), rig.transport.tuneReads.drop(reads))
        assertEquals(7000, rig.state.gains.staged[kpA])
    }

    @Test
    fun `a release that arrives while a read pass holds the lane is sent after it, not dropped`() = runTest {
        val rig = shownTune()
        // The reads of the pass answer Busy for a while: it is in flight, holding the model's lane.
        rig.transport.tuneBusy = 4
        rig.model.refresh()
        runCurrent()
        assertTrue(rig.state.reading)

        rig.model.slide(Gains.KP, 7000)
        rig.model.slideEnd(Gains.KP, 7211)
        runCurrent()
        assertTrue(rig.transport.tuneWrites.isEmpty(), "a write went out while the read pass held the lane")

        advanceTimeBy(1_000)
        runCurrent()
        assertEquals(7211, rig.transport.tuneWrites.single().third)
        assertEquals(7211, rig.state.gains.staged[kpA])
    }

    @Test
    fun `a drag is clamped to the maximum read from this board`() = runTest {
        val rig = TuneRig(this)
        rig.transport.store[master to maxKp] = Value.I16(6050)
        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()
        assertEquals(6050, rig.state.gains.max(Gains.KP))

        dragGain(rig, Gains.KP, to = 20_000, samples = 10)

        assertTrue(rig.transport.tuneWrites.all { it.third <= 6050 }, "a drag wrote past the board's maximum")
        assertEquals(6050, rig.state.gains.staged[kpA])
    }

    @Test
    fun `the converging bound covers the whole sweep, not the last sample of it`() = runTest {
        val rig = shownTune()
        // No battery word known: the ramp's floor of one count per pass, 250 counts per second.
        dragGain(rig, Gains.KP, to = 6500, samples = 10)

        // 500 counts from where the sweep began = 500 passes = 2000 ms, not the 200 of one sample:
        // the bound is taken over every value the gain was staged at, which is what makes it a
        // bound on a target that was moving rather than on the last step of it.
        assertEquals(2_000L, rig.state.gains.converging[kpA])
        advanceTimeBy(2_001)
        assertFalse(kpA in rig.state.gains.converging)
    }

    @Test
    fun `a drag goes through armed, writes no flash, and is bounded at the session's lowest word`() = runTest {
        val rig = shownTune()
        rig.armed = true
        for (word in listOf(3300, 2400)) {
            rig.transport.emitCyclicState(CyclicState(0, 0, 0, word, 0, 0, 0, QUIET_OBS))
            runCurrent()
        }

        // A release alone, so the bound is exact: 400 counts at the 4 per pass kp gets at 2400
        // (not the 6 it would get at 3300) is 100 passes, 400 ms.
        rig.model.slideEnd(Gains.KP, 6400)
        advanceTimeBy(interval)
        runCurrent()

        assertEquals(6400, rig.state.gains.staged[kpA])
        assertTrue(rig.transport.writes.isEmpty(), "a drag wrote flash")
        assertEquals(400L, rig.state.gains.converging[kpA])
    }

    /**
     * A sweep whose ramp is fast enough that an early mark elapses mid-gesture is bounded from
     * there, not from where the finger started: by then the loop has provably passed the values
     * before it, and claiming otherwise would overstate the bound rather than keep it honest.
     */
    @Test
    fun `a mark that elapses mid-sweep rebases the bound on what is left to converge`() = runTest {
        val rig = shownTune()
        rig.transport.emitCyclicState(CyclicState(0, 0, 0, 2400, 0, 0, 0, QUIET_OBS))
        runCurrent()

        dragGain(rig, Gains.KP, to = 6400, samples = 8)

        val bound = rig.state.gains.converging.getValue(kpA)
        assertTrue(bound in 1 until 400L, "the whole sweep was claimed as still converging: $bound")
        assertEquals(6400, rig.state.gains.staged[kpA])
    }

    @Test
    fun `a refused sample ends the gesture with its notice, and nothing more is sent`() = runTest {
        val rig = shownTune()
        rig.transport.tuneRefusal = CfgRefusal.BAD

        dragGain(rig, Gains.KP, to = 9000, samples = 20)

        assertEquals(TuneNotice.BoardRefused(kpA, CfgRefusal.BAD), rig.state.notice)
        assertEquals(1, rig.transport.tuneWrites.size, "the drag kept writing to a lane that refused it")
        assertEquals(6000, rig.state.gains.staged[kpA])
        assertFalse(rig.state.writing, "the refused drag did not let go of the lane")

        // The next gesture is free to try: one refusal ends a gesture, not the screen.
        rig.transport.tuneRefusal = null
        dragGain(rig, Gains.KP, to = 6300, samples = 4)
        assertEquals(6300, rig.state.gains.staged[kpA])
    }

    @Test
    fun `a screen that leaves mid-drag lets go of the lane at the value it had reached`() = runTest {
        val rig = shownTune()
        rig.model.slide(Gains.KP, 7000)
        runCurrent()
        rig.model.slide(Gains.KP, 7400)

        rig.model.onHidden() // the drag never gets its finger-up
        advanceTimeBy(interval)
        runCurrent()

        assertFalse(rig.state.writing, "the abandoned drag still holds the lane")
        assertEquals(7400, rig.state.gains.staged[kpA])
    }
}
