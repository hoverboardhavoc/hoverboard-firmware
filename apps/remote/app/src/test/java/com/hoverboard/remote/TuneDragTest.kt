package com.hoverboard.remote

import com.hoverboard.protocol.config.CfgRefusal
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
 * What a dragged gain slider puts on the tune lane (`specs/remote-ui.md` section 3.3a).
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

    private val board = 0x01
    private val grKey = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GR)
    private val maxGr = Key(Fields.CONTROL_GAIN_MAX.id, Gains.GR)

    /** `gr` held at 6000, a round number the sweeps below move from; `gr` steps one count per pass. */
    private val at6000: TuneRig.() -> Unit = { hold(board, grKey, 6000) }

    /** One cadence interval and a tick, which is as long as any single send has to wait. */
    private val interval = TuneModel.DRAG_SEND_INTERVAL_MS + 1

    @Test
    fun `a drag sends on the cadence, not once per frame`() = runTest {
        val rig = shownTune(setup = at6000)

        // One second of a 60 Hz drag: 60 samples, which at one write each would be 60 TUNE_WRITEs.
        dragGain(rig, Gains.GR, to = 9000, samples = 60, frameMs = 16)

        val writes = rig.transport.tuneWrites.size
        val cadence = (1_000 / TuneModel.DRAG_SEND_INTERVAL_MS).toInt()
        // The samples of one second, plus the release: nothing above the cadence it was sent at.
        assertTrue(writes <= cadence + 1, "the drag sent $writes writes, above the cadence")
        assertTrue(writes > 1, "the drag sent $writes writes: nothing was streamed during the drag")
        assertEquals(9000, rig.state.gains.staged[grKey], "the value let go at is not what the board holds")
    }

    @Test
    fun `the value the finger lets go at is staged exactly, whatever the cadence sampled`() = runTest {
        val rig = shownTune(setup = at6000)

        // The release lands between two cadence samples, which is where a dropped release shows up.
        rig.model.slide(grKey, 7000)
        runCurrent()
        advanceTimeBy(TuneModel.DRAG_SEND_INTERVAL_MS / 2)
        rig.model.slide(grKey, 8000)
        rig.model.slideEnd(grKey, 8137)
        advanceTimeBy(interval)
        runCurrent()

        assertEquals(8137, rig.transport.tuneWrites.last().third)
        assertEquals(8137, rig.state.gains.staged[grKey])
        assertEquals(board to grKey, rig.transport.tuneReads.last(), "the settled value was not re-read")
        assertFalse(rig.state.writing, "the drag did not let go of the lane")
    }

    @Test
    fun `a release at the value the last sample already sent is still re-read`() = runTest {
        val rig = shownTune(setup = at6000)

        rig.model.slide(grKey, 7000)
        runCurrent()
        val reads = rig.transport.tuneReads.size
        rig.model.slideEnd(grKey, 7000) // the finger held still before lifting
        advanceTimeBy(interval)
        runCurrent()

        assertEquals(1, rig.transport.tuneWrites.size, "the release re-wrote a value already staged")
        assertEquals(listOf(board to grKey), rig.transport.tuneReads.drop(reads))
        assertEquals(7000, rig.state.gains.staged[grKey])
    }

    @Test
    fun `a release that arrives while a read pass holds the lane is sent after it, not dropped`() = runTest {
        val rig = shownTune(setup = at6000)
        // The reads of the pass answer Busy for a while: it is in flight, holding the model's lane.
        rig.transport.tuneBusy = 4
        rig.model.refresh()
        runCurrent()
        assertTrue(rig.state.reading)

        rig.model.slide(grKey, 7000)
        rig.model.slideEnd(grKey, 7211)
        runCurrent()
        assertTrue(rig.transport.tuneWrites.isEmpty(), "a write went out while the read pass held the lane")

        advanceTimeBy(1_000)
        runCurrent()
        assertEquals(7211, rig.transport.tuneWrites.single().third)
        assertEquals(7211, rig.state.gains.staged[grKey])
    }

    @Test
    fun `a drag is clamped to the maximum read from this board`() = runTest {
        val rig = TuneRig(this)
        rig.transport.store[board to maxGr] = Value.I16(6050)
        rig.attach(board)
        rig.model.onShown()
        runCurrent()
        assertEquals(6050, rig.state.gains.max(grKey))

        dragGain(rig, Gains.GR, to = 20_000, samples = 10)

        assertTrue(rig.transport.tuneWrites.all { it.third <= 6050 }, "a drag wrote past the board's maximum")
        assertEquals(6050, rig.state.gains.staged[grKey])
    }

    @Test
    fun `the converging bound covers the whole sweep, not the last sample of it`() = runTest {
        val rig = shownTune(setup = at6000)
        // gr's cap is one count per pass, 250 counts per second.
        dragGain(rig, Gains.GR, to = 6500, samples = 10)

        // 500 counts from where the sweep began = 500 passes = 2000 ms, not the 200 of one sample:
        // the bound is taken over every value the gain was staged at, which is what makes it a
        // bound on a target that was moving rather than on the last step of it.
        assertEquals(2_000L, rig.state.gains.converging[grKey])
        advanceTimeBy(2_001)
        assertFalse(grKey in rig.state.gains.converging)
    }

    @Test
    fun `a drag goes through armed and writes no flash`() = runTest {
        val rig = shownTune(setup = at6000)
        rig.armed = true

        // A release alone, so the bound is exact: 400 counts at gr's one count per pass is 400
        // passes, 1600 ms.
        rig.model.slideEnd(grKey, 6400)
        advanceTimeBy(interval)
        runCurrent()

        assertEquals(6400, rig.state.gains.staged[grKey])
        assertTrue(rig.transport.writes.isEmpty(), "a drag wrote flash")
        assertEquals(1_600L, rig.state.gains.converging[grKey])
    }

    @Test
    fun `a refused sample ends the gesture with its notice, and nothing more is sent`() = runTest {
        val rig = shownTune(setup = at6000)
        rig.transport.tuneRefusal = CfgRefusal.BAD

        dragGain(rig, Gains.GR, to = 9000, samples = 20)

        assertEquals(TuneNotice.BoardRefused(grKey, CfgRefusal.BAD), rig.state.notice)
        assertEquals(1, rig.transport.tuneWrites.size, "the drag kept writing to a lane that refused it")
        assertEquals(6000, rig.state.gains.staged[grKey])
        assertFalse(rig.state.writing, "the refused drag did not let go of the lane")

        // The next gesture is free to try: one refusal ends a gesture, not the screen.
        rig.transport.tuneRefusal = null
        dragGain(rig, Gains.GR, to = 6300, samples = 4)
        assertEquals(6300, rig.state.gains.staged[grKey])
    }

    @Test
    fun `a screen that leaves mid-drag lets go of the lane at the value it had reached`() = runTest {
        val rig = shownTune(setup = at6000)
        rig.model.slide(grKey, 7000)
        runCurrent()
        rig.model.slide(grKey, 7400)

        rig.model.onHidden() // the drag never gets its finger-up
        advanceTimeBy(interval)
        runCurrent()

        assertFalse(rig.state.writing, "the abandoned drag still holds the lane")
        assertEquals(7400, rig.state.gains.staged[grKey])
    }
}
