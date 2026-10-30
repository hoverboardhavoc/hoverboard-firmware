package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.remote.model.FrameStats
import com.hoverboard.remote.model.TelemetryInbox
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The keyed telemetry inbox: two boards reach the phone over one link, and what keeps them apart
 * is the source address their frames arrive under.
 */
class TelemetryInboxTest {

    private val master = 0x01
    private val slave = 0x02

    /** Two boards saying different things stay two boards. Merged, they were one board flickering. */
    @Test
    fun `each source keeps its own state`() {
        val inbox = TelemetryInbox()
        inbox.cyclic(master, state(battery = 3_780, mode = 3), nowMs = 0)
        inbox.cyclic(slave, state(battery = 3_612, mode = 0), nowMs = 0)

        val boards = inbox.boards.value
        assertEquals(3_780, boards.getValue(master).cyclic?.battery)
        assertEquals(3, boards.getValue(master).cyclic?.mode)
        assertEquals(3_612, boards.getValue(slave).cyclic?.battery)
        assertEquals(0, boards.getValue(slave).cyclic?.mode)
    }

    /** Latest-wins is per board: a frame from one of them does not age or replace the other's. */
    @Test
    fun `a board's newest frame replaces only its own`() {
        val inbox = TelemetryInbox()
        inbox.cyclic(master, state(battery = 3_780, mode = 3), nowMs = 0)
        inbox.cyclic(slave, state(battery = 3_612, mode = 0), nowMs = 0)
        inbox.cyclic(master, state(battery = 3_600, mode = 4), nowMs = 10)

        assertEquals(3_600, inbox.boards.value.getValue(master).cyclic?.battery)
        assertEquals(3_612, inbox.boards.value.getValue(slave).cyclic?.battery)
    }

    /** A fault level belongs to the board that sent it: the record is per source. */
    @Test
    fun `a fault level is filed under its own board`() {
        val inbox = TelemetryInbox()
        inbox.cyclic(master, state(), nowMs = 0)
        inbox.cyclic(slave, state(fault = 2), nowMs = 0)

        assertTrue(inbox.boards.value.getValue(slave).anyFault)
        assertEquals(2, inbox.boards.value.getValue(slave).faultLevel)
        assertEquals(false, inbox.boards.value.getValue(master).anyFault)
    }

    /**
     * A board that stops speaking is dropped, and the one still speaking is not. The attached
     * board's stream is the faster of the two and the forward of its partner's is the slower, so
     * the window has to be read from the last frame of each source separately.
     */
    @Test
    fun `a quiet board is dropped and a speaking one is kept`() {
        val inbox = TelemetryInbox(quietMs = 1_000)
        inbox.cyclic(master, state(), nowMs = 0)
        inbox.cyclic(slave, state(), nowMs = 0)

        inbox.cyclic(master, state(), nowMs = 800)
        inbox.expire(nowMs = 800)
        assertTrue("the slave is still inside the window", slave in inbox.boards.value)

        inbox.cyclic(master, state(), nowMs = 1_100)
        inbox.expire(nowMs = 1_100)
        assertTrue("the master is still speaking", master in inbox.boards.value)
        assertNull("the slave stopped and is not claimed to be anything", inbox.boards.value[slave])
    }

    /** A dropped board comes back as itself when it speaks again. */
    @Test
    fun `a board that resumes is heard again`() {
        val inbox = TelemetryInbox(quietMs = 1_000)
        inbox.cyclic(slave, state(battery = 3_612), nowMs = 0)
        inbox.expire(nowMs = 1_000)
        assertNull(inbox.boards.value[slave])

        inbox.cyclic(slave, state(battery = 3_590), nowMs = 1_200)
        assertEquals(3_590, inbox.boards.value.getValue(slave).cyclic?.battery)
    }

    /** A session that has ended knows nothing about either board. */
    @Test
    fun `clear forgets every board`() {
        val inbox = TelemetryInbox()
        inbox.cyclic(master, state(), nowMs = 0)
        inbox.cyclic(slave, state(), nowMs = 0)

        inbox.clear()

        assertTrue(inbox.boards.value.isEmpty())
    }

    /** Each source counts its own frames: the forward's count is not the attached board's. */
    @Test
    fun `frames are counted per source`() {
        val inbox = TelemetryInbox()
        repeat(4) { inbox.cyclic(master, state(), nowMs = it * 200L) }
        repeat(2) { inbox.cyclic(slave, state(), nowMs = it * 400L) }

        assertEquals(4, inbox.frameStats.value.getValue(master).frames)
        assertEquals(2, inbox.frameStats.value.getValue(slave).frames)
    }

    /** The longest gap is the largest step between two consecutive frames, kept once it passes. */
    @Test
    fun `the longest gap between two frames is kept`() {
        val inbox = TelemetryInbox()
        inbox.cyclic(slave, state(), nowMs = 1_000)
        assertEquals("one frame has no gap yet", 0L, inbox.frameStats.value.getValue(slave).longestGapMs)

        inbox.cyclic(slave, state(), nowMs = 1_400)
        inbox.cyclic(slave, state(), nowMs = 2_710)
        inbox.cyclic(slave, state(), nowMs = 3_110)

        assertEquals(1_310L, inbox.frameStats.value.getValue(slave).longestGapMs)
    }

    /**
     * The gap that dropped a source is the one the line exists to show, so the stats outlive the
     * quiet window: a source that went silent and came back reports the silence as its gap.
     */
    @Test
    fun `the stats survive a source being dropped for silence`() {
        val inbox = TelemetryInbox(quietMs = 1_000)
        inbox.cyclic(slave, state(), nowMs = 0)
        inbox.expire(nowMs = 1_000)
        assertNull(inbox.boards.value[slave])
        assertEquals(1, inbox.frameStats.value.getValue(slave).frames)

        inbox.cyclic(slave, state(), nowMs = 1_500)
        assertEquals(2, inbox.frameStats.value.getValue(slave).frames)
        assertEquals(1_500L, inbox.frameStats.value.getValue(slave).longestGapMs)
    }

    /** A new session starts its counts from nothing. */
    @Test
    fun `clear forgets the stats`() {
        val inbox = TelemetryInbox()
        inbox.cyclic(master, state(), nowMs = 0)
        inbox.cyclic(master, state(), nowMs = 900)

        inbox.clear()
        assertTrue(inbox.frameStats.value.isEmpty())

        inbox.cyclic(master, state(), nowMs = 5_000)
        assertEquals(FrameStats(frames = 1, longestGapMs = 0), inbox.frameStats.value.getValue(master))
    }

    private fun state(battery: Int = 3_700, mode: Int = 0, fault: Int = 0) = CyclicState(
        pitch = 0,
        roll = 0,
        wheelSpeed = 0,
        battery = battery,
        mode = mode,
        fault = fault,
        flags = 0,
        peerDemand = 0,
        wheelPos = 0,
        wheelRate = 0,
        obs = QUIET_OBS,
    )
}
