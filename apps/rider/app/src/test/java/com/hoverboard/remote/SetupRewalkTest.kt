package com.hoverboard.remote

import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The Setup model walks again for boards that come up after the connect (`specs/rider-ui.md` 3.1,
 * "Boards that come up after the connect"): on a cadence while no motor controller is known, on a
 * frame from a source the node list does not hold, never while armed, and a grown node list reads
 * the new boards' facts.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupRewalkTest {

    private val interval = SetupModel.REWALK_INTERVAL_MS

    @Test
    fun `a bridge alone is walked again every 3 s`() = runTest {
        val rig = SetupRig(this)
        rig.transport.makeBridge(0x01)
        rig.transport.setAttachedBoard(0x01)
        tick(1)
        assertTrue(rig.state.rideFactsRead)
        assertTrue(rig.transport.rewalks.isEmpty())

        tick(interval)
        assertEquals(listOf(listOf(0x01)), rig.transport.rewalks)
        tick(interval)
        assertEquals(2, rig.transport.rewalks.size)
    }

    @Test
    fun `no re-walk while armed`() = runTest {
        val rig = SetupRig(this)
        rig.armed = true
        rig.transport.makeBridge(0x01)
        rig.transport.setAttachedBoard(0x01)
        tick(interval * 3)
        assertTrue(rig.transport.rewalks.isEmpty())
    }

    @Test
    fun `a node whose class is not read is not taken for a bridge`() = runTest {
        val rig = SetupRig(this)
        rig.transport.unreadable += SetupFields.NODE_CLASS.key
        rig.transport.setAttachedBoard(0x01)
        tick(interval * 3)
        assertTrue(rig.transport.rewalks.isEmpty())
    }

    /** A grown list is read again: the new board's class, then its ride facts, and rideFactsRead waits for them. */
    @Test
    fun `a re-walk that finds a board reads its facts`() = runTest {
        val rig = SetupRig(this)
        rig.transport.makeBridge(0x01)
        rig.transport.setAttachedBoard(0x01)
        tick(1)
        rig.transport.rewalkFinds = listOf(0x01, 0x02)
        tick(interval)

        assertEquals(listOf(0x01, 0x02), rig.state.addresses)
        assertEquals(listOf(0x02), rig.state.motorControllers)
        assertTrue(rig.state.rideFactsRead)
        for (field in SetupFields.MOTOR_FACTS) {
            assertTrue(0x02 to field.key in rig.transport.reads, "${field.key} read from the new board")
        }
        val walks = rig.transport.rewalks.size
        tick(interval * 3)
        assertEquals(walks, rig.transport.rewalks.size, "a motor controller is known: no cadence")
    }

    /** A frame from a source the walk did not find asks for a re-walk, at most once in 3 s. */
    @Test
    fun `an unknown source asks for one re-walk`() = runTest {
        val rig = SetupRig(this)
        rig.transport.setAttachedBoard(0x01)
        tick(1)
        rig.emitAttitude(0, 0, from = 0x05)
        tick(1)
        assertEquals(listOf(listOf(0x01)), rig.transport.rewalks)
        rig.emitAttitude(100, 0, from = 0x05)
        tick(interval / 2)
        assertEquals(1, rig.transport.rewalks.size, "inside the 3 s")
        tick(interval)
        rig.emitAttitude(200, 0, from = 0x05)
        tick(1)
        assertEquals(2, rig.transport.rewalks.size)
        assertFalse(0x05 in rig.state.addresses, "the fake's walk found nothing new")
    }
}
