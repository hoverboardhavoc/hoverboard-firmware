package com.hoverboard.remote

import com.hoverboard.protocol.linkctl.CyclicState
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

    /** A fault edge belongs to the board that sent it, and sits with that board's cyclic record. */
    @Test
    fun `a fault edge is filed under its own board`() {
        val inbox = TelemetryInbox()
        inbox.cyclic(master, state(), nowMs = 0)
        inbox.cyclic(slave, state(), nowMs = 0)
        inbox.fault(slave, stop = true, code = 0x11, nowMs = 0)

        assertTrue(inbox.boards.value.getValue(slave).faultStop)
        assertEquals(0x11, inbox.boards.value.getValue(slave).faultCode)
        assertEquals(false, inbox.boards.value.getValue(master).faultStop)
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

    private fun state(battery: Int = 3_700, mode: Int = 0) = CyclicState(
        pitch = 0,
        roll = 0,
        wheelSpeed = 0,
        battery = battery,
        mode = mode,
        fault = 0,
        flags = 0,
        peerDemand = 0,
        obs = QUIET_OBS,
    )
}
