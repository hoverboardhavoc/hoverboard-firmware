package com.hoverboard.remote

import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.WriteVerified
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
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
 * may then say about the three values (`specs/rider-ui.md` section 3.3, `specs/state-feedback.md`
 * section 10).
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneModelTest {

    private val board = 0x01
    private val other = 0x02
    private val gp = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GP)
    private val gr = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GR)
    private val gx = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GX)
    private val gv = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GV)
    private val speedMax = Gains.key(Gains.CONTROL_REFERENCE, Gains.SPEED_MAX)

    /** Every key the live lane carries: the four gains, then the reference row. */
    private val allKeys = (0 until Gains.COUNT).map { Gains.key(Gains.CONTROL_SF_GAINS, it) } +
        Gains.REFERENCE_DEFAULT.indices.map { Gains.key(Gains.CONTROL_REFERENCE, it) }
    private val maxKeys = (0 until Gains.COUNT).map { Key(Fields.CONTROL_GAIN_MAX.id, it) }

    @Test
    fun `showing reads the target's staged and flash values once per session, the target's only`() = runTest {
        val rig = shownTune()
        assertEquals(allKeys.map { board to it }, rig.transport.tuneReads)
        // The config lane reads the same keys for flash, then the four gain maxima, then the drive
        // bounds the screen's other half owns (`specs/rider-ui.md` 3.3b).
        assertEquals(
            (allKeys + maxKeys + SetupFields.DRIVE_BOUNDS.map { it.key }).map { board to it },
            rig.transport.reads,
        )
        assertFalse(rig.state.gains.stale)
        assertEquals(START_GAINS, rig.state.keys.map { rig.state.gains.staged[it] })
        assertEquals(START_GAINS, rig.state.keys.map { rig.state.gains.flash[it] })
        assertEquals(START_REFERENCE, rig.state.referenceKeys.map { rig.state.gains.staged[it] })

        rig.model.onHidden()
        rig.model.onShown()
        runCurrent()
        assertEquals(allKeys.size, rig.transport.tuneReads.size, "a second showing re-read the board")
    }

    @Test
    fun `a board that never staged a gain shows the inert zeros`() = runTest {
        val rig = TuneRig(this)
        rig.attach(board)
        rig.model.onShown()
        runCurrent()
        assertEquals(Gains.DEFAULT, rig.state.keys.map { rig.state.gains.staged[it] })
        assertEquals(Gains.REFERENCE_DEFAULT, rig.state.referenceKeys.map { rig.state.gains.staged[it] })
    }

    @Test
    fun `a settled value is one TUNE_WRITE and a re-read, and touches no flash`() = runTest {
        val rig = shownTune()
        val reads = rig.transport.tuneReads.size
        rig.nudge(Gains.GP, 100)
        runCurrent()

        assertEquals(listOf(Triple(board, gp, 3591)), rig.transport.tuneWrites)
        assertEquals(board to gp, rig.transport.tuneReads.drop(reads).single())
        assertTrue(rig.transport.writes.isEmpty(), "a settled value wrote flash")
        assertEquals(3591, rig.state.gains.staged[gp])
        assertTrue(rig.state.gains.unsaved(gp))
        assertEquals(3491, rig.state.gains.flash[gp])
    }

    @Test
    fun `a changed staged gain is marked converging for its fixed cap's bound, then the mark clears`() = runTest {
        val rig = shownTune()
        // gr steps one count per pass: 100 counts = 100 passes = 400 ms.
        rig.nudge(Gains.GR, 100)
        runCurrent()
        assertEquals(400L, rig.state.gains.converging[gr])
        advanceTimeBy(399)
        assertTrue(gr in rig.state.gains.converging, "the mark cleared before the bound elapsed")
        advanceTimeBy(2)
        assertFalse(gr in rig.state.gains.converging)
    }

    @Test
    fun `each gain's bound is its own cap's, with no battery term`() = runTest {
        val rig = shownTune()
        // gp steps two counts per pass: 100 counts = 50 passes = 200 ms.
        rig.nudge(Gains.GP, 100)
        runCurrent()
        assertEquals(200L, rig.state.gains.converging[gp])
        // gx and gv step one: 100 counts = 400 ms.
        rig.nudge(Gains.GX, 100)
        runCurrent()
        assertEquals(400L, rig.state.gains.converging[gx])
        rig.nudge(Gains.GV, 100)
        runCurrent()
        assertEquals(400L, rig.state.gains.converging[gv])
    }

    @Test
    fun `a second change before the mark clears is bounded across both and outlives the first clear`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.GR, 100)
        runCurrent()
        advanceTimeBy(200)
        rig.nudge(Gains.GR, -100) // back to 4002; the law may be anywhere in 4002..4102
        runCurrent()
        assertEquals(400L, rig.state.gains.converging[gr])
        advanceTimeBy(250) // past the first mark's 400 ms
        assertTrue(gr in rig.state.gains.converging, "the first change's clear removed the second's mark")
        advanceTimeBy(200)
        assertFalse(gr in rig.state.gains.converging)
    }

    @Test
    fun `a change is clamped to the seam's range and one that cannot move sends nothing`() = runTest {
        val rig = shownTune { hold(board, gp, 19_950) }

        rig.nudge(Gains.GP, 100)
        runCurrent()
        assertEquals(20_000, rig.transport.tuneWrites.single().third)
        rig.nudge(Gains.GP, 100)
        runCurrent()
        assertEquals(1, rig.transport.tuneWrites.size)
    }

    @Test
    fun `a change is bounded by the maximum read from the board`() = runTest {
        val rig = shownTune { transport.store[board to maxKeys[Gains.GX]] = Value.I16(250) }
        assertEquals(250, rig.state.gains.max(gx))
        assertEquals(Gains.DEFAULT_MAX[Gains.GV], rig.state.gains.max(gv))

        rig.nudge(Gains.GX, 100)
        runCurrent()
        assertEquals(250, rig.transport.tuneWrites.single().third, "297 bounded by the read maximum")
        rig.nudge(Gains.GX, 100)
        runCurrent()
        assertEquals(1, rig.transport.tuneWrites.size, "at the maximum: nothing sent")
    }

    @Test
    fun `a negative maximum read from the board bounds a change at zero`() = runTest {
        val rig = TuneRig(this)
        rig.transport.store[board to maxKeys[Gains.GV]] = Value.I16(-3)
        rig.attach(board)
        rig.model.onShown()
        runCurrent()
        assertEquals(0, rig.state.gains.max(gv))
        rig.nudge(Gains.GV, 5)
        runCurrent()
        assertTrue(rig.transport.tuneWrites.isEmpty())
    }

    @Test
    fun `a board that answers the maxima unknown falls back to the default bound and still reads`() = runTest {
        val rig = TuneRig(this)
        for (k in maxKeys) rig.transport.defaults -= k // the board answers the key unknown
        rig.hold(board, gp, 19_950)
        rig.attach(board)
        rig.model.onShown()
        runCurrent()
        assertFalse(rig.state.gains.stale)
        assertEquals(null, rig.state.notice)
        assertTrue(rig.state.gains.maxima.isEmpty())
        rig.nudge(Gains.GP, 100)
        runCurrent()
        assertEquals(Gains.DEFAULT_MAX[Gains.GP], rig.transport.tuneWrites.single().third)
    }

    @Test
    fun `staged values go through armed, and SAVE is refused armed with nothing sent`() = runTest {
        val rig = shownTune()
        rig.armed = true
        rig.nudge(Gains.GP, 100)
        runCurrent()
        assertEquals(3591, rig.state.gains.staged[gp])

        rig.model.save()
        runCurrent()
        assertEquals(TuneNotice.SaveWhileArmed, rig.state.notice)
        assertTrue(rig.transport.writes.isEmpty())
    }

    @Test
    fun `SAVE writes each unsaved live value, gains and reference alike, then re-reads both copies`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.GP, 100)
        runCurrent()
        rig.model.slideEnd(speedMax, 120)
        runCurrent()
        val tuneReads = rig.transport.tuneReads.size
        val reads = rig.transport.reads.size

        rig.model.save()
        runCurrent()
        assertEquals(
            listOf(Triple(board, gp, Value.I16(3591)), Triple(board, speedMax, Value.I16(120))),
            rig.transport.writes,
        )
        assertEquals(listOf(board to gp, board to speedMax), rig.transport.reads.drop(reads))
        assertEquals(listOf(board to gp, board to speedMax), rig.transport.tuneReads.drop(tuneReads))
        assertTrue(rig.state.unsavedKeys.isEmpty())
    }

    @Test
    fun `the unsaved mark comes from the re-read, never from the write's CFG_OK`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.GP, 100)
        runCurrent()
        // The board answers CFG_OK but flash still holds the old value (as a save of a value flash
        // already held would leave a diverging staged value in place).
        rig.transport.writeHook = { _, v -> WriteVerified(v) }

        rig.model.save()
        runCurrent()
        assertTrue(rig.state.gains.unsaved(gp))
        assertEquals(3491, rig.state.gains.flash[gp])
    }

    @Test
    fun `REVERT stages the flash values back through the tune lane, and the law ramps back`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.GP, 100)
        runCurrent()
        rig.nudge(Gains.GR, 50)
        runCurrent()
        advanceTimeBy(500)

        rig.model.revert()
        runCurrent()
        assertEquals(listOf(3591, 4052, 3491, 4002), rig.transport.tuneWrites.map { it.third })
        assertTrue(rig.transport.writes.isEmpty(), "REVERT wrote flash")
        assertFalse(rig.state.gains.unsaved(gp) || rig.state.gains.unsaved(gr))
        assertTrue(gp in rig.state.gains.converging && gr in rig.state.gains.converging)
    }

    @Test
    fun `a refusal is shown against its gain, and the shadow is re-read`() = runTest {
        val rig = shownTune()
        rig.transport.tuneRefusal = CfgRefusal.BAD
        val reads = rig.transport.tuneReads.size
        rig.nudge(Gains.GP, 100)
        runCurrent()
        assertEquals(TuneNotice.BoardRefused(gp, CfgRefusal.BAD), rig.state.notice)
        assertEquals(3491, rig.state.gains.staged[gp])
        assertEquals(board to gp, rig.transport.tuneReads.drop(reads).single())
        assertFalse(gp in rig.state.gains.converging)
    }

    @Test
    fun `a node the walk did not find cannot be picked`() = runTest {
        val rig = shownTune(other = null)
        rig.model.selectTarget(other)
        runCurrent()
        assertEquals(board, rig.state.target)
        assertTrue(rig.transport.tuneReads.all { it.first == board })
    }

    @Test
    fun `a second motor controller is tuned at its own address`() = runTest {
        val rig = shownTune(board, other)
        rig.model.selectTarget(other)
        runCurrent()
        assertEquals(other, rig.state.target)
        assertEquals(other, rig.transport.tuneReads.last().first)

        rig.nudge(Gains.GP, 100)
        runCurrent()
        assertEquals(Triple(other, gp, 3591), rig.transport.tuneWrites.single())
        assertEquals(3491, rig.state.boards.getValue(board).staged[gp], "a change to one board moved the other")
    }

    @Test
    fun `a dropped link keeps the last-read values greyed, with writes off, and the unsaved mark kept`() = runTest {
        val rig = shownTune()
        rig.nudge(Gains.GP, 100)
        runCurrent()
        rig.transport.setAttachedBoard(null)
        assertTrue(rig.state.gains.stale)
        assertTrue(rig.state.gains.unsaved(gp))
        rig.nudge(Gains.GP, 100)
        runCurrent()
        assertEquals(1, rig.transport.tuneWrites.size)

        // The board kept the staged value; reattaching re-reads and re-derives the mark.
        rig.attach(board, other)
        runCurrent()
        assertFalse(rig.state.gains.stale)
        assertTrue(rig.state.gains.unsaved(gp))
        assertEquals(3591, rig.state.gains.staged[gp])
    }

    @Test
    fun `a board that does not answer the tune lane is stale, says so, and takes no writes`() = runTest {
        val rig = TuneRig(this)
        rig.transport.tuneSilent += board
        rig.attach(board)
        rig.model.onShown()
        runCurrent()
        assertTrue(rig.state.gains.stale)
        assertEquals(TuneNotice.Unanswered(gp), rig.state.notice)
        assertFalse(rig.state.writable)
    }

    @Test
    fun `a read that finds the request slot taken waits for it rather than failing`() = runTest {
        val rig = TuneRig(this)
        rig.transport.tuneBusy = 3
        rig.attach(board)
        rig.model.onShown()
        runCurrent()
        advanceTimeBy(SLOT_RETRY_MS * 4)
        assertFalse(rig.state.gains.stale)
        assertEquals(allKeys.size, rig.transport.tuneReads.size)
    }
}
