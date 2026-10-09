package com.hoverboard.remote

import com.hoverboard.protocol.store.Gains
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The order of the Tune model's requests: the two lanes answer with byte-identical `CONFIG_RESP`s,
 * so a late duplicate reply is taken by the next request of the same (board, key), whichever lane
 * asks. A read pass and a SAVE re-read never send two consecutive requests of one key.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneRequestOrderTest {

    private val master = 0x01
    private val kpA = Gains.key(Gains.CONTROL_GAIN_A, Gains.KP)
    private val bkA = Gains.key(Gains.CONTROL_GAIN_A, Gains.BK)

    private fun noRepeats(requests: List<Pair<Int, *>>) =
        assertTrue(requests.zipWithNext().none { (a, b) -> a == b }, "two consecutive requests of one key: $requests")

    @Test
    fun `the unsaved mark survives a late duplicate of a TUNE_READ reply`() = runTest {
        val rig = TuneRig(this)
        rig.transport.shadow[master to kpA] = 6100 // staged, flash holds the default 6000
        rig.transport.duplicateTuneReplies = true
        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()

        assertFalse(rig.state.gains.stale)
        assertEquals(6100, rig.state.gains.staged[kpA])
        assertEquals(6000, rig.state.gains.flash[kpA], "a duplicate TUNE_READ reply was taken as flash")
        assertTrue(rig.state.gains.unsaved(kpA))
        noRepeats(rig.transport.requests)
    }

    @Test
    fun `a SAVE of two gains writes both, then reads all flash, then all staged`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.KP, 100)
        runCurrent()
        rig.nudge(Gains.BK, 50)
        runCurrent()
        val from = rig.transport.requests.size

        rig.model.save()
        runCurrent()
        val save = rig.transport.requests.drop(from)
        assertEquals(listOf(kpA, bkA, kpA, bkA, kpA, bkA).map { master to it }, save)
        assertEquals(listOf(kpA, bkA).map { master to it }, rig.transport.writes.map { it.first to it.second })
        noRepeats(save)
        assertFalse(rig.state.gains.unsaved(kpA) || rig.state.gains.unsaved(bkA))
    }
}
