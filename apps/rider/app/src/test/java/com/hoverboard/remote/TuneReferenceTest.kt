package com.hoverboard.remote

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
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The reference row on the Tune model (`specs/state-feedback.md` sections 4 and 10): `speed_max`
 * and `speed_slew`, read with the gains, written over the same RAM lane, saved by the same Save,
 * bounded by the COMPILED minima and maxima, and never marked converging, because the firmware does
 * not ramp this row.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneReferenceTest {

    private val board = 0x01
    private val gp = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GP)
    private val gr = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GR)
    private val speedMax = Gains.key(Gains.CONTROL_REFERENCE, Gains.SPEED_MAX)
    private val speedSlew = Gains.key(Gains.CONTROL_REFERENCE, Gains.SPEED_SLEW)
    private val maxGp = Key(Fields.CONTROL_GAIN_MAX.id, Gains.GP)

    /** One cadence interval and a tick, which is as long as any single send has to wait. */
    private val interval = TuneModel.DRAG_SEND_INTERVAL_MS + 1

    @Test
    fun `the read pass reads both reference words, staged and flash, and no maximum for them`() = runTest {
        val rig = shownTune()
        val keys = rig.state.referenceKeys

        assertEquals(keys, rig.transport.tuneReads.takeLast(keys.size).map { it.second })
        assertEquals(START_REFERENCE, keys.map { rig.state.gains.staged[it] })
        assertEquals(START_REFERENCE, keys.map { rig.state.gains.flash[it] })
        assertEquals(
            Gains.COUNT,
            rig.transport.reads.count { it.second.fieldId == Fields.CONTROL_GAIN_MAX.id },
            "a maximum was read for the reference row",
        )
    }

    @Test
    fun `a reference word is bounded by the compiled range, whatever this board's gain maxima are`() = runTest {
        val rig = shownTune { transport.store[board to maxGp] = Value.I16(100) }
        assertEquals(100, rig.state.gains.max(gp))
        assertEquals(Gains.REFERENCE_MAXIMA[Gains.SPEED_MAX], rig.state.gains.max(speedMax))

        rig.model.slideEnd(speedMax, 9_000)
        advanceTimeBy(interval)
        runCurrent()
        val ceiling = Gains.REFERENCE_MAXIMA[Gains.SPEED_MAX]
        assertEquals(Triple(board, speedMax, ceiling), rig.transport.tuneWrites.single())
    }

    @Test
    fun `speed_slew is held at its floor of 1, never sent as 0`() = runTest {
        val rig = shownTune()
        assertEquals(Gains.REFERENCE_MINIMA[Gains.SPEED_SLEW], rig.state.gains.min(speedSlew))

        rig.model.slideEnd(speedSlew, 0)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(1, rig.transport.tuneWrites.single().third)
        assertEquals(1, rig.state.gains.staged[speedSlew])
    }

    @Test
    fun `a staged reference word carries no converging mark, because the firmware does not ramp this row`() =
        runTest {
            val rig = shownTune()

            rig.model.slideEnd(speedMax, 120)
            advanceTimeBy(interval)
            runCurrent()
            assertEquals(120, rig.state.gains.staged[speedMax])
            assertFalse(speedMax in rig.state.gains.converging, "a reference word was marked converging")

            // The gains still are: this is a property of the row. gr, so the 400 ms bound outlasts
            // the interval waited.
            rig.nudge(Gains.GR, 100)
            advanceTimeBy(interval)
            runCurrent()
            assertTrue(gr in rig.state.gains.converging)
        }

    @Test
    fun `a reference word is unsaved until Save writes it to flash`() = runTest {
        val rig = shownTune()

        rig.model.slideEnd(speedSlew, 60)
        advanceTimeBy(interval)
        runCurrent()
        assertTrue(rig.state.gains.unsaved(speedSlew), "a tune write reached flash, or was not counted unsaved")
        assertEquals(listOf(speedSlew), rig.state.unsavedKeys)
        assertTrue(rig.transport.writes.isEmpty(), "the tune lane wrote flash")

        rig.model.save()
        runCurrent()

        assertEquals(listOf(board to speedSlew), rig.transport.writes.map { it.first to it.second })
        assertEquals(60, rig.state.gains.flash[speedSlew])
        assertFalse(rig.state.gains.unsaved(speedSlew))
    }

    @Test
    fun `Revert stages the reference row back to flash with the gains`() = runTest {
        val rig = shownTune()
        rig.model.slideEnd(speedMax, 120)
        advanceTimeBy(interval)
        runCurrent()
        rig.nudge(Gains.GP, 100)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(setOf(gp, speedMax), rig.state.unsavedKeys.toSet())

        rig.model.revert()
        runCurrent()

        assertEquals(80, rig.state.gains.staged[speedMax])
        assertEquals(3491, rig.state.gains.staged[gp])
        assertTrue(rig.state.unsavedKeys.isEmpty())
    }

    @Test
    fun `the stick is dead at speed_max 0, live above it, and unknown before the read`() = runTest {
        val rig = TuneRig(this)
        assertNull(rig.state.stickDead, "the stick's state was claimed before the board was read")

        rig.attach(board)
        rig.model.onShown()
        runCurrent()
        assertEquals(true, rig.state.stickDead, "a board at the default reference row is not reported dead")

        rig.model.slideEnd(speedMax, 80)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(false, rig.state.stickDead)

        rig.model.slideEnd(speedMax, 0)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(true, rig.state.stickDead)
    }

    /** The reference row is on the live lane, so it is written armed exactly as a gain is. */
    @Test
    fun `a reference word is written while armed and writes no flash`() = runTest {
        val rig = shownTune()
        rig.armed = true

        rig.model.slideEnd(speedMax, 100)
        advanceTimeBy(interval)
        runCurrent()

        assertEquals(100, rig.state.gains.staged[speedMax])
        assertTrue(rig.transport.writes.isEmpty(), "an armed tune write reached flash")

        rig.model.save()
        runCurrent()
        assertEquals(TuneNotice.SaveWhileArmed, rig.state.notice)
        assertTrue(rig.transport.writes.isEmpty())
    }
}
