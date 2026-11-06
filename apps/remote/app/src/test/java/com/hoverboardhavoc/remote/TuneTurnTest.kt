package com.hoverboardhavoc.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
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
 * The heading hold's five live words on the Tune model (`specs/heading-hold.md` section 10): `gh`,
 * `ghv`, `gff` of `CONTROL_TURN_GAINS` and `rate_max`, `rate_slew` of `CONTROL_TURN_REFERENCE`,
 * read with the gains, written over the same RAM lane, saved by the same Save, bounded by the
 * COMPILED ranges, and marked converging on the three gains (the firmware ramps them at a cap of 1)
 * and never on the reference pair.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneTurnTest {

    private val board = 0x01
    private val gh = Gains.key(Gains.CONTROL_TURN_GAINS, Gains.GH)
    private val ghv = Gains.key(Gains.CONTROL_TURN_GAINS, Gains.GHV)
    private val gff = Gains.key(Gains.CONTROL_TURN_GAINS, Gains.GFF)
    private val rateMax = Gains.key(Gains.CONTROL_TURN_REFERENCE, Gains.RATE_MAX)
    private val rateSlew = Gains.key(Gains.CONTROL_TURN_REFERENCE, Gains.RATE_SLEW)

    /** One cadence interval and a tick, which is as long as any single send has to wait. */
    private val interval = TuneModel.DRAG_SEND_INTERVAL_MS + 1

    @Test
    fun `the five turn keys follow the reference row on the lane`() {
        val s = TuneState()
        assertEquals(listOf(gh, ghv, gff, rateMax, rateSlew), s.turnKeys)
        assertEquals(s.keys + s.referenceKeys + s.turnKeys, s.liveKeys)
        assertEquals(11, s.liveKeys.size)
    }

    @Test
    fun `the read pass reads the five turn words, staged and flash, and no maximum for them`() = runTest {
        val rig = shownTune()
        val keys = rig.state.turnKeys

        assertEquals(START_TURN, keys.map { rig.state.gains.staged[it] })
        assertEquals(START_TURN, keys.map { rig.state.gains.flash[it] })
        for (k in keys) {
            assertTrue(board to k in rig.transport.tuneReads, "$k was not read staged")
            assertTrue(board to k in rig.transport.reads, "$k was not read from flash")
        }
        assertEquals(
            Gains.COUNT,
            rig.transport.reads.count { it.second.fieldId == Fields.CONTROL_GAIN_MAX.id },
            "a maximum was read for the turn rows",
        )
    }

    @Test
    fun `the turn rows take the compiled ranges, whatever this board's gain maxima are`() = runTest {
        val rig = shownTune { transport.store[board to Fields.CONTROL_GAIN_MAX.at(0).key(0)] = Value.I16(100) }
        val g = rig.state.gains
        assertEquals(Gains.TURN_GAIN_MAXIMA, listOf(gh, ghv, gff).map { g.max(it) })
        assertEquals(Gains.TURN_REFERENCE_MAXIMA, listOf(rateMax, rateSlew).map { g.max(it) })
        assertEquals(listOf(0, 0, 0, 0, 1), rig.state.turnKeys.map { g.min(it) })

        rig.model.slideEnd(gh, 9_000)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(Triple(board, gh, Gains.TURN_GAIN_MAXIMA[Gains.GH]), rig.transport.tuneWrites.last())

        rig.model.slideEnd(rateSlew, 0)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(Triple(board, rateSlew, 1), rig.transport.tuneWrites.last(), "rate_slew went below its floor")
        assertNull(rig.state.notice)
    }

    /**
     * Each turn gain ramps at a cap of 1 per 250 Hz pass (`Gains.Ramp.TURN_CAPS`), so a move of 100
     * counts is marked converging for 400 ms; the reference pair is not ramped and gets no mark.
     */
    @Test
    fun `a staged turn gain is marked converging for the ramp's bound and a rate word is not`() = runTest {
        val rig = shownTune()

        rig.model.slideEnd(ghv, 160)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(160, rig.state.gains.staged[ghv])
        assertEquals(400L, rig.state.gains.converging[ghv])

        rig.model.slideEnd(rateMax, 80)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(80, rig.state.gains.staged[rateMax])
        assertFalse(rateMax in rig.state.gains.converging, "a turn reference word was marked converging")

        advanceTimeBy(400)
        runCurrent()
        assertFalse(ghv in rig.state.gains.converging, "the mark outlived the ramp's bound")
    }

    @Test
    fun `a turn word is unsaved until Save writes it to every board`() = runTest {
        val rig = shownTune()

        rig.model.slideEnd(gff, 60)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(listOf(gff), rig.state.unsavedKeys)
        assertTrue(rig.transport.writes.isEmpty(), "the tune lane wrote flash")

        rig.model.save()
        runCurrent()

        assertEquals(
            listOf(Triple(board, gff, Value.I16(60)), Triple(0x02, gff, Value.I16(60))),
            rig.transport.writes,
        )
        assertEquals(60, rig.state.gains.flash[gff])
        assertTrue(rig.state.unsavedKeys.isEmpty())
    }

    @Test
    fun `Revert stages the turn words back to flash with the rest of the lane`() = runTest {
        val rig = shownTune()
        rig.model.slideEnd(gh, 120)
        advanceTimeBy(interval)
        runCurrent()
        rig.model.slideEnd(rateSlew, 100)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(setOf(gh, rateSlew), rig.state.unsavedKeys.toSet())

        rig.model.revert()
        runCurrent()

        assertEquals(START_TURN[0], rig.state.gains.staged[gh])
        assertEquals(START_TURN[4], rig.state.gains.staged[rateSlew])
        assertTrue(rig.state.unsavedKeys.isEmpty())
    }

    /** The turn rows are on the live lane, so they are written armed exactly as a gain is. */
    @Test
    fun `a turn word is written while armed and writes no flash`() = runTest {
        val rig = shownTune()
        rig.armed = true

        rig.model.slideEnd(gh, 80)
        advanceTimeBy(interval)
        runCurrent()

        assertEquals(80, rig.state.gains.staged[gh])
        assertTrue(rig.transport.writes.isEmpty(), "an armed tune write reached flash")
    }

    @Test
    fun `a board at the defaults reads the inert turn rows`() = runTest {
        val rig = TuneRig(this)
        rig.attach(board)
        rig.model.onShown()
        runCurrent()
        val staged = rig.state.turnKeys.map { rig.state.gains.staged[it] }
        assertEquals(Gains.TURN_DEFAULT + Gains.TURN_REFERENCE_DEFAULT, staged)
    }
}
