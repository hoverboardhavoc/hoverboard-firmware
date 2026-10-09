package com.hoverboard.remote

import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.WriteVerified
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.Node
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * [TuneModel] against the fake transport's boards: what is sent, to which board, and what the screen
 * may then say about the three values (`specs/rider-ui.md` section 3.3).
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneModelTest {

    private val master = 0x01
    private val slave = 0x02
    private val kpA = Gains.key(Gains.CONTROL_GAIN_A, Gains.KP)
    private val bkA = Gains.key(Gains.CONTROL_GAIN_A, Gains.BK)
    private val kpB = Gains.key(Gains.CONTROL_GAIN_B, Gains.KP)
    private val allKeys = listOf(Gains.CONTROL_GAIN_A, Gains.CONTROL_GAIN_B).flatMap { p ->
        (0 until Gains.PER_PROFILE).map { Gains.key(p, it) }
    }
    private val maxKeys = (0 until Gains.PER_PROFILE).map { Key(Fields.CONTROL_GAIN_MAX.id, it) }

    private fun battery(rig: TuneRig, centivolts: Int) =
        rig.transport.emitCyclicState(CyclicState(0, 0, 0, centivolts, 0, 0, 0))

    @Test
    fun `showing reads the target's staged and flash gains once per session, the master's only`() = runTest {
        val rig = shownTune()
        assertEquals(allKeys.map { master to it }, rig.transport.tuneReads)
        assertEquals((allKeys + maxKeys).map { master to it }, rig.transport.reads)
        assertFalse(rig.state.gains.stale)
        assertEquals(6000, rig.state.gains.staged[kpA])
        assertEquals(6000, rig.state.gains.flash[kpA])

        rig.model.onHidden()
        rig.model.onShown()
        runCurrent()
        assertEquals(allKeys.size, rig.transport.tuneReads.size, "a second showing re-read the board")
    }

    @Test
    fun `a settled value is one TUNE_WRITE and a re-read, and touches no flash`() = runTest {
        val rig = shownTune()
        val reads = rig.transport.tuneReads.size
        rig.nudge(Gains.KP, 100)
        runCurrent()

        assertEquals(listOf(Triple(master, kpA, 6100)), rig.transport.tuneWrites)
        assertEquals(master to kpA, rig.transport.tuneReads.drop(reads).single())
        assertTrue(rig.transport.writes.isEmpty(), "a settled value wrote flash")
        assertEquals(6100, rig.state.gains.staged[kpA])
        assertTrue(rig.state.gains.unsaved(kpA))
        assertEquals(6000, rig.state.gains.flash[kpA])
    }

    @Test
    fun `a changed staged value is marked converging for the ramp's bound, then the mark clears`() = runTest {
        val rig = shownTune()
        // No battery word known: the floor of one count per pass, 100 counts = 100 passes = 400 ms.
        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(400L, rig.state.gains.converging[kpA])
        advanceTimeBy(399)
        assertTrue(kpA in rig.state.gains.converging, "the mark cleared before the bound elapsed")
        advanceTimeBy(2)
        assertFalse(kpA in rig.state.gains.converging)
    }

    @Test
    fun `the master's bound uses the lowest battery word this session reported`() = runTest {
        val rig = shownTune()
        for (word in listOf(3300, 2400, 3300)) {
            battery(rig, word)
            runCurrent()
        }
        // kp steps 4 per pass at 2400 (not 6 at 3300): 100 counts = 25 passes = 100 ms.
        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(100L, rig.state.gains.converging[kpA])
    }

    @Test
    fun `a second change before the mark clears is bounded across both and outlives the first clear`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.KP, 100)
        runCurrent()
        advanceTimeBy(200)
        rig.nudge(Gains.KP, -100) // back to 6000; the loop may be anywhere in 6000..6100
        runCurrent()
        assertEquals(400L, rig.state.gains.converging[kpA])
        advanceTimeBy(250) // past the first mark's 400 ms
        assertTrue(kpA in rig.state.gains.converging, "the first change's clear removed the second's mark")
        advanceTimeBy(200)
        assertFalse(kpA in rig.state.gains.converging)
    }

    @Test
    fun `a change is clamped to the seam's range and one that cannot move sends nothing`() = runTest {
        val rig = TuneRig(this)
        rig.transport.shadow[master to kpA] = 19_950
        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()

        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(20_000, rig.transport.tuneWrites.single().third)
        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(1, rig.transport.tuneWrites.size)
    }

    @Test
    fun `a change is bounded by the maximum read from the board`() = runTest {
        val rig = TuneRig(this)
        rig.transport.store[master to maxKeys[Gains.KP]] = Value.I16(6050)
        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()
        assertEquals(6050, rig.state.gains.max(Gains.KP))
        assertEquals(Gains.DEFAULT_MAX[Gains.BK], rig.state.gains.max(Gains.BK))

        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(6050, rig.transport.tuneWrites.single().third, "6100 bounded by the read maximum")
        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(1, rig.transport.tuneWrites.size, "at the maximum: nothing sent")
    }

    @Test
    fun `a negative maximum read from the board bounds a change at zero`() = runTest {
        val rig = TuneRig(this)
        rig.transport.store[master to maxKeys[Gains.PR]] = Value.I16(-3)
        rig.transport.shadow[master to Gains.key(Gains.CONTROL_GAIN_A, Gains.PR)] = 0
        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()
        assertEquals(0, rig.state.gains.max(Gains.PR))
        rig.nudge(Gains.PR, 5)
        runCurrent()
        assertTrue(rig.transport.tuneWrites.isEmpty())
    }

    @Test
    fun `firmware without the maxima field falls back to the default bound and still reads`() = runTest {
        val rig = TuneRig(this)
        for (k in maxKeys) rig.transport.defaults -= k // the board answers the key unknown
        rig.transport.shadow[master to kpA] = 19_950
        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()
        assertFalse(rig.state.gains.stale)
        assertEquals(null, rig.state.notice)
        assertTrue(rig.state.gains.maxima.isEmpty())
        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(Gains.DEFAULT_MAX[Gains.KP], rig.transport.tuneWrites.single().third)
    }

    @Test
    fun `staged values go through armed, and SAVE is refused armed with nothing sent`() = runTest {
        val rig = shownTune()
        rig.armed = true
        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(6100, rig.state.gains.staged[kpA])

        rig.model.save()
        runCurrent()
        assertEquals(TuneNotice.SaveWhileArmed, rig.state.notice)
        assertTrue(rig.transport.writes.isEmpty())
    }

    @Test
    fun `SAVE writes each unsaved gain of the shown profile only, then re-reads both values`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.KP, 100)
        runCurrent()
        rig.model.selectProfile(Gains.CONTROL_GAIN_B)
        rig.nudge(Gains.KP, -100)
        runCurrent()
        rig.model.selectProfile(Gains.CONTROL_GAIN_A)
        val tuneReads = rig.transport.tuneReads.size
        val reads = rig.transport.reads.size

        rig.model.save()
        runCurrent()
        assertEquals(listOf(Triple(master, kpA, Value.I16(6100))), rig.transport.writes)
        assertEquals(listOf(master to kpA), rig.transport.tuneReads.drop(tuneReads))
        assertEquals(listOf(master to kpA), rig.transport.reads.drop(reads))
        assertFalse(rig.state.gains.unsaved(kpA))
        assertTrue(rig.state.gains.unsaved(kpB), "SAVE persisted the profile not shown")
    }

    @Test
    fun `the unsaved mark comes from the re-read, never from the write's CFG_OK`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.KP, 100)
        runCurrent()
        // The board answers CFG_OK but flash still holds the old value (as a save of a value flash
        // already held would leave a diverging staged value in place).
        rig.transport.writeHook = { _, v -> WriteVerified(v) }

        rig.model.save()
        runCurrent()
        assertTrue(rig.state.gains.unsaved(kpA))
        assertEquals(6000, rig.state.gains.flash[kpA])
    }

    @Test
    fun `REVERT stages the flash values back through the tune lane, and the loop ramps back`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.KP, 100)
        runCurrent()
        rig.nudge(Gains.BK, 50)
        runCurrent()
        advanceTimeBy(1_000)

        rig.model.revert()
        runCurrent()
        assertEquals(listOf(6100, 2050, 6000, 2000), rig.transport.tuneWrites.map { it.third })
        assertTrue(rig.transport.writes.isEmpty(), "REVERT wrote flash")
        assertFalse(rig.state.gains.unsaved(kpA) || rig.state.gains.unsaved(bkA))
        assertTrue(kpA in rig.state.gains.converging && bkA in rig.state.gains.converging)
    }

    @Test
    fun `a refusal is shown against its gain, and the shadow is re-read`() = runTest {
        val rig = shownTune()
        rig.transport.tuneRefusal = CfgRefusal.BAD
        val reads = rig.transport.tuneReads.size
        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(TuneNotice.BoardRefused(kpA, CfgRefusal.BAD), rig.state.notice)
        assertEquals(6000, rig.state.gains.staged[kpA])
        assertEquals(master to kpA, rig.transport.tuneReads.drop(reads).single())
        assertFalse(kpA in rig.state.gains.converging)
    }

    @Test
    fun `Profile B is hidden while the master is known to run the rider requirement waived`() = runTest {
        val rig = shownTune()
        rig.model.selectProfile(Gains.CONTROL_GAIN_B)
        assertEquals(Gains.CONTROL_GAIN_B, rig.state.shownProfile)

        rig.waiver.value = RiderWaiver.POSSIBLY
        runCurrent()
        assertFalse(rig.state.profileBHidden, "B hidden on a board that may still be running the default")

        rig.waiver.value = RiderWaiver.WAIVED
        runCurrent()
        assertTrue(rig.state.profileBHidden)
        assertEquals(Gains.CONTROL_GAIN_A, rig.state.shownProfile)
        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(kpA, rig.transport.tuneWrites.single().second)
    }

    @Test
    fun `the slave is tunable only once discovered, by its own address, with its own rider requirement`() = runTest {
        val rig = shownTune(slave = null)
        rig.model.selectTarget(Node.SLAVE)
        assertEquals(Node.MASTER, rig.state.target)

        rig.transport.setAttachedBoard(master, slave)
        rig.transport.store[slave to Fields.CONTROL_RIDER_REQUIRED.key(0)] = Value.U8(Fields.RiderRequired.NOT_REQUIRED)
        battery(rig, 2400) // the master's word: not the slave's
        rig.model.selectTarget(Node.SLAVE)
        runCurrent()
        assertEquals(slave, rig.transport.tuneReads.last().first)
        assertEquals(slave to Fields.CONTROL_RIDER_REQUIRED.key(0), rig.transport.reads.last())
        assertTrue(rig.state.profileBHidden)

        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(Triple(slave, kpA, 6100), rig.transport.tuneWrites.single())
        assertEquals(400L, rig.state.gains.converging[kpA], "the slave's bound used the master's battery word")
        assertEquals(6000, rig.state.boards.getValue(Node.MASTER).staged[kpA], "a slave change moved the master")
    }

    @Test
    fun `a dropped link keeps the last-read values greyed, with writes off, and the unsaved mark kept`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.KP, 100)
        runCurrent()
        rig.transport.setAttachedBoard(null)
        assertTrue(rig.state.gains.stale)
        assertTrue(rig.state.gains.unsaved(kpA))
        rig.nudge(Gains.KP, 100)
        runCurrent()
        assertEquals(1, rig.transport.tuneWrites.size)

        // The board kept the staged value; reattaching re-reads and re-derives the mark.
        rig.transport.setAttachedBoard(master, slave)
        runCurrent()
        assertFalse(rig.state.gains.stale)
        assertTrue(rig.state.gains.unsaved(kpA))
        assertEquals(6100, rig.state.gains.staged[kpA])
    }

    @Test
    fun `a board that does not answer the tune lane is stale, says so, and takes no writes`() = runTest {
        val rig = TuneRig(this)
        rig.transport.tuneSilent += master
        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()
        assertTrue(rig.state.gains.stale)
        assertEquals(TuneNotice.Unanswered(kpA), rig.state.notice)
        assertFalse(rig.state.writable)
    }

    @Test
    fun `a read that finds the request slot taken waits for it rather than failing`() = runTest {
        val rig = TuneRig(this)
        rig.transport.tuneBusy = 3
        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()
        advanceTimeBy(SLOT_RETRY_MS * 4)
        assertFalse(rig.state.gains.stale)
        assertEquals(allKeys.size, rig.transport.tuneReads.size)
    }
}
