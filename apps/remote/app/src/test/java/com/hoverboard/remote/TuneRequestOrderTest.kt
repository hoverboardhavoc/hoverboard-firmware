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

    private val board = 0x01
    private val gp = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GP)
    private val gr = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GR)

    private fun noRepeats(requests: List<Pair<Int, *>>) =
        assertTrue(requests.zipWithNext().none { (a, b) -> a == b }, "two consecutive requests of one key: $requests")

    @Test
    fun `the unsaved mark survives a late duplicate of a TUNE_READ reply`() = runTest {
        val rig = TuneRig(this)
        rig.transport.shadow[board to gp] = 3591 // staged, flash holds the default 0
        rig.transport.duplicateTuneReplies = true
        rig.attach(board)
        rig.model.onShown()
        runCurrent()

        assertFalse(rig.state.gains.stale)
        assertEquals(3591, rig.state.gains.staged[gp])
        assertEquals(0, rig.state.gains.flash[gp], "a duplicate TUNE_READ reply was taken as flash")
        assertTrue(rig.state.gains.unsaved(gp))
        noRepeats(rig.transport.requests)
    }

    @Test
    fun `a SAVE of two gains writes both, then reads all flash, then all staged`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.GP, 100)
        runCurrent()
        rig.nudge(Gains.GR, 50)
        runCurrent()
        val from = rig.transport.requests.size

        rig.model.save()
        runCurrent()
        val save = rig.transport.requests.drop(from)
        assertEquals(listOf(gp, gr, gp, gr, gp, gr).map { board to it }, save)
        assertEquals(listOf(gp, gr).map { board to it }, rig.transport.writes.map { it.first to it.second })
        noRepeats(save)
        assertFalse(rig.state.gains.unsaved(gp) || rig.state.gains.unsaved(gr))
    }
}
