package com.hoverboard.remote

import com.hoverboard.protocol.config.TuneVerified
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The live readback (`specs/state-feedback.md` section 10): what a row displays is the value the
 * board last REPORTED, never the value sent, and while the screen is shown every live-lane key is
 * read again once a second. On 2026-10-26 a RAM write over SWD left the screen showing a stale 513;
 * these are the cases that made that possible.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneReadbackTest {

    private val board = 0x01
    private val gp = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GP)
    private val gr = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GR)
    private val speedMax = Gains.key(Gains.CONTROL_REFERENCE, Gains.SPEED_MAX)

    /** One poll period and a tick: a round has gone out. */
    private val period = TuneModel.POLL_INTERVAL_MS + 1

    @Test
    fun `a shadow changed behind the app's back is on screen within one poll`() = runTest {
        val rig = shownTune()
        assertEquals(3491, rig.state.gains.staged[gp])

        // A RAM write over SWD: the app sent nothing and is told nothing.
        rig.transport.shadow[board to gp] = 350
        rig.transport.shadow[board to speedMax] = 0
        advanceTimeBy(period)
        runCurrent()

        assertEquals(350, rig.state.gains.staged[gp])
        assertEquals(0, rig.state.gains.staged[speedMax])
        assertEquals(true, rig.state.stickDead)
        assertTrue(rig.state.gains.unsaved(gp), "the poke diverges from flash and was not marked unsaved")
        assertTrue(rig.transport.writes.isEmpty() && rig.transport.tuneWrites.isEmpty())
    }

    @Test
    fun `each round reads every live-lane key once, one round per second, and nothing from flash`() = runTest {
        val rig = shownTune()
        val tuneReads = rig.transport.tuneReads.size
        val reads = rig.transport.reads.size

        advanceTimeBy(3 * TuneModel.POLL_INTERVAL_MS + 1)
        runCurrent()

        val polled = rig.transport.tuneReads.drop(tuneReads)
        assertEquals(List(3) { rig.state.liveKeys }.flatten().map { board to it }, polled)
        assertEquals(reads, rig.transport.reads.size, "the poll read the config lane")
    }

    @Test
    fun `the poll stops when the screen is hidden and starts again when it is shown`() = runTest {
        val rig = shownTune()
        rig.model.onHidden()
        val tuneReads = rig.transport.tuneReads.size
        advanceTimeBy(5 * TuneModel.POLL_INTERVAL_MS)
        runCurrent()
        assertEquals(tuneReads, rig.transport.tuneReads.size, "a hidden screen kept polling")

        rig.model.onShown()
        advanceTimeBy(period)
        runCurrent()
        assertEquals(tuneReads + rig.state.liveKeys.size, rig.transport.tuneReads.size)
    }

    @Test
    fun `a written value is displayed from the re-read, not from the write's echo`() = runTest {
        val rig = shownTune()
        // The board echoes the value it was sent but its shadow does not hold it: whatever the
        // lane says on the write, the row shows what the read finds.
        rig.transport.tuneWriteHook = { _, v -> TuneVerified(v) }

        rig.nudge(Gains.GP, 100)
        runCurrent()

        assertEquals(Triple(board, gp, 3591), rig.transport.tuneWrites.single())
        assertEquals(3491, rig.state.gains.staged[gp], "the sent value was shown as the board's")
    }

    @Test
    fun `the poll pauses during a drag and resumes after the release`() = runTest {
        val rig = shownTune()
        val tuneReads = rig.transport.tuneReads.size

        rig.model.slide(gr, 4100)
        runCurrent()
        for (i in 1..15) {
            rig.model.slide(gr, 4100 + i)
            advanceTimeBy(100)
        }
        // 1.5 s of drag: no round went out, only the drag's own writes.
        assertTrue(rig.transport.tuneReads.size == tuneReads, "the poll read during a drag")

        rig.model.slideEnd(gr, 4200)
        advanceTimeBy(TuneModel.DRAG_SEND_INTERVAL_MS + 1)
        runCurrent()
        assertEquals(listOf(board to gr), rig.transport.tuneReads.drop(tuneReads), "the release was not re-read")

        advanceTimeBy(period)
        runCurrent()
        assertEquals(1 + rig.state.liveKeys.size, rig.transport.tuneReads.size - tuneReads)
        assertEquals(4200, rig.state.gains.staged[gr])
    }

    @Test
    fun `the poll pauses during Save`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.GP, 100)
        runCurrent()
        val gate = CompletableDeferred<Unit>()
        rig.transport.writeGate = gate
        val tuneReads = rig.transport.tuneReads.size

        rig.model.save()
        advanceTimeBy(3 * TuneModel.POLL_INTERVAL_MS)
        runCurrent()
        assertEquals(tuneReads, rig.transport.tuneReads.size, "the poll read while Save held the lane")
        assertTrue(rig.state.writing)

        gate.complete(Unit)
        runCurrent()
        assertFalse(rig.state.writing)
        assertFalse(rig.state.gains.unsaved(gp))
    }

    @Test
    fun `a tap that arrives during a poll round waits for its current read, then goes out`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.GP, 100)
        runCurrent()
        val gate = CompletableDeferred<Unit>()
        rig.transport.tuneReadGate = gate
        val tuneReads = rig.transport.tuneReads.size
        advanceTimeBy(period)
        runCurrent() // the round's first read is in flight, held at the gate

        rig.model.save()
        runCurrent()
        assertTrue(rig.state.writing, "Save was dropped because a poll round held the lane")
        assertTrue(rig.transport.writes.isEmpty())

        rig.transport.tuneReadGate = null
        gate.complete(Unit)
        runCurrent()
        assertEquals(listOf(board to gp), rig.transport.tuneReads.drop(tuneReads).take(1))
        assertEquals(listOf(Triple(board, gp, Value.I16(3591))), rig.transport.writes)
        assertEquals(
            listOf(board to gp, board to gp),
            rig.transport.tuneReads.drop(tuneReads),
            "the round kept reading after the tap asked for the lane",
        )
        assertFalse(rig.state.writing)
        assertFalse(rig.state.gains.unsaved(gp))
    }

    @Test
    fun `an unanswered poll marks the board stale and the poll stops asking`() = runTest {
        val rig = shownTune()
        rig.transport.tuneSilent += board
        advanceTimeBy(period)
        runCurrent()
        assertTrue(rig.state.gains.stale)
        assertEquals(TuneNotice.Unanswered(gp), rig.state.notice)

        val tuneReads = rig.transport.tuneReads.size
        advanceTimeBy(3 * TuneModel.POLL_INTERVAL_MS)
        runCurrent()
        assertEquals(tuneReads, rig.transport.tuneReads.size, "a stale board was still polled")
    }

    @Test
    fun `a busy slot skips the round without a notice`() = runTest {
        val rig = shownTune()
        rig.transport.tuneBusy = 1
        advanceTimeBy(period)
        runCurrent()
        assertEquals(null, rig.state.notice)
        assertFalse(rig.state.gains.stale)
    }
}
