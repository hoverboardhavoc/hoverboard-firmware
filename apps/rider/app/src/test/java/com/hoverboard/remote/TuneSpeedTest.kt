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
 * The outer speed loop's three words on the Tune model (`specs/rider-ui.md` 3.3c): read with the
 * gains, written over the same RAM lane, saved by the same Save, bounded by the COMPILED maxima,
 * and never marked converging, because the firmware does not ramp this row.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneSpeedTest {

    private val master = 0x01
    private val kpA = Gains.key(Gains.CONTROL_GAIN_A, Gains.KP)
    private val speedKp = Gains.key(Gains.CONTROL_SPEED_LOOP, Gains.SPEED_KP)
    private val speedMax = Gains.key(Gains.CONTROL_SPEED_LOOP, Gains.SPEED_MAX)
    private val maxKp = Key(Fields.CONTROL_GAIN_MAX.id, Gains.KP)

    /** One cadence interval and a tick, which is as long as any single send has to wait. */
    private val interval = TuneModel.DRAG_SEND_INTERVAL_MS + 1

    @Test
    fun `the read pass reads all three speed words, staged and flash`() = runTest {
        val rig = shownTune()
        val keys = rig.state.speedKeys

        assertEquals(keys, rig.transport.tuneReads.takeLast(keys.size).map { it.second })
        for (key in keys) {
            assertEquals(0, rig.state.gains.staged[key])
            assertEquals(0, rig.state.gains.flash[key])
        }
        // Three maxima read and no more: the speed row's are compiled, so the pass asks the board
        // for nothing on their account.
        assertEquals(
            Gains.PER_PROFILE,
            rig.transport.reads.count { it.second.fieldId == Fields.CONTROL_GAIN_MAX.id },
            "a maximum was read for the speed row",
        )
    }

    @Test
    fun `a speed word is bounded by the compiled maximum, whatever this board's gain maximum is`() = runTest {
        val rig = TuneRig(this)
        // A board whose PID gains are bounded well below the speed row's compiled 2000.
        rig.transport.store[master to maxKp] = Value.I16(600)
        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()

        assertEquals(600, rig.state.gains.max(kpA))
        assertEquals(Gains.SPEED_MAXIMA[Gains.SPEED_KP], rig.state.gains.max(speedKp))

        rig.model.slideEnd(speedKp, 9_000)
        advanceTimeBy(interval)
        runCurrent()

        val write = rig.transport.tuneWrites.single()
        assertEquals(speedKp, write.second)
        assertEquals(Gains.SPEED_MAXIMA[Gains.SPEED_KP], write.third, "a speed word went past the compiled maximum")
    }

    @Test
    fun `a staged speed word carries no converging mark, because the firmware does not ramp this row`() = runTest {
        val rig = shownTune()

        rig.model.slideEnd(speedMax, 80)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(80, rig.state.gains.staged[speedMax])
        assertFalse(speedMax in rig.state.gains.converging, "a speed word was marked converging")

        // The gains still are: this is a property of the row, not a feature that was removed.
        rig.nudge(Gains.KP, 100)
        advanceTimeBy(interval)
        runCurrent()
        assertTrue(kpA in rig.state.gains.converging)
    }

    @Test
    fun `a speed word is unsaved until Save writes it to flash`() = runTest {
        val rig = shownTune()

        rig.model.slideEnd(speedKp, 100)
        advanceTimeBy(interval)
        runCurrent()
        assertTrue(rig.state.gains.unsaved(speedKp), "a tune write reached flash, or was not counted unsaved")
        assertEquals(listOf(speedKp), rig.state.unsavedKeys)
        assertTrue(rig.transport.writes.isEmpty(), "the tune lane wrote flash")

        rig.model.save()
        runCurrent()

        assertEquals(listOf(master to speedKp), rig.transport.writes.map { it.first to it.second })
        assertEquals(100, rig.state.gains.flash[speedKp])
        assertFalse(rig.state.gains.unsaved(speedKp))
    }

    @Test
    fun `Revert stages the speed row back to flash with the gains`() = runTest {
        val rig = shownTune()
        rig.model.slideEnd(speedKp, 100)
        advanceTimeBy(interval)
        runCurrent()
        rig.nudge(Gains.KP, 100)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(setOf(kpA, speedKp), rig.state.unsavedKeys.toSet())

        rig.model.revert()
        runCurrent()

        assertEquals(0, rig.state.gains.staged[speedKp])
        assertEquals(6000, rig.state.gains.staged[kpA])
        assertTrue(rig.state.unsavedKeys.isEmpty())
    }

    @Test
    fun `the loop is off at speed_kp 0, running above it, and unknown before the read`() = runTest {
        val rig = TuneRig(this)
        assertNull(rig.state.speedLoopOff, "the loop's state was claimed before the board was read")

        rig.transport.setAttachedBoard(master)
        rig.model.onShown()
        runCurrent()
        assertEquals(true, rig.state.speedLoopOff, "a board at the default speed row is not reported off")

        rig.model.slideEnd(speedKp, 100)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(false, rig.state.speedLoopOff)

        rig.model.slideEnd(speedKp, 0)
        advanceTimeBy(interval)
        runCurrent()
        assertEquals(true, rig.state.speedLoopOff)
    }

    /** The speed row is on the live lane, so it is written armed exactly as a gain is. */
    @Test
    fun `a speed word is written while armed and writes no flash`() = runTest {
        val rig = shownTune()
        rig.armed = true

        rig.model.slideEnd(speedKp, 100)
        advanceTimeBy(interval)
        runCurrent()

        assertEquals(100, rig.state.gains.staged[speedKp])
        assertTrue(rig.transport.writes.isEmpty(), "an armed tune write reached flash")

        rig.model.save()
        runCurrent()
        assertEquals(TuneNotice.SaveWhileArmed, rig.state.notice)
        assertTrue(rig.transport.writes.isEmpty())
    }
}
