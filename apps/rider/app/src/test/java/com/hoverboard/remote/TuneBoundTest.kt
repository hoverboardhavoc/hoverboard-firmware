package com.hoverboard.remote

import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.model.TakesEffect
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The four drive bounds on the Tune screen (`specs/rider-ui.md` 3.3b): read with the gains, written
 * to FLASH one value at a time, refused while armed, and taken up by the board at its next arm.
 *
 * They are not on the live tune lane, so none of the gains' staged/engaged machinery applies to
 * them and these tests are about the config lane alone.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class TuneBoundTest {

    private val board = 0x01
    private val leanMax = SetupFields.LEAN_MAX.key
    private val turnSlew = SetupFields.TURN_SLEW.key

    @Test
    fun `the read pass reads all four bounds from the board`() = runTest {
        val rig = shownTune()
        for (field in SetupFields.DRIVE_BOUNDS) {
            assertTrue(board to field.key in rig.transport.reads, "${field.key} was not read")
            assertEquals(
                (field.def.default as Value.I16).v,
                rig.state.bound(field.key),
                "${field.key} did not reach the state",
            )
        }
    }

    /** The evening's values: one write each, verified, and the state follows the board. */
    @Test
    fun `setting a bound writes it to flash once and keeps what the board echoed`() = runTest {
        val rig = shownTune()
        rig.model.setBound(leanMax, 600)
        runCurrent()
        rig.model.setBound(turnSlew, 240)
        runCurrent()

        assertEquals(
            listOf(Triple(board, leanMax, Value.I16(600)), Triple(board, turnSlew, Value.I16(240))),
            rig.transport.writes,
        )
        assertEquals(600, rig.state.bound(leanMax))
        assertEquals(240, rig.state.bound(turnSlew))
        assertNull(rig.state.notice)
    }

    /** A value the board already holds appends no flash record, as on Setup. */
    @Test
    fun `setting a bound to the value the board holds sends nothing`() = runTest {
        val rig = shownTune()
        rig.model.setBound(leanMax, 600)
        runCurrent()
        rig.model.setBound(leanMax, 600)
        runCurrent()
        assertEquals(1, rig.transport.writes.size)
    }

    /**
     * Armed, nothing is sent and the screen says why. The board refuses every config write while
     * armed (`specs/integration.md`, R4), so the app refuses first rather than collecting a
     * refusal per slider release.
     */
    @Test
    fun `a bound cannot be set while armed`() = runTest {
        val rig = shownTune()
        rig.armed = true
        rig.model.setBound(leanMax, 600)
        runCurrent()
        assertEquals(emptyList<Any>(), rig.transport.writes)
        assertEquals(TuneNotice.BoundWhileArmed, rig.state.notice)
        assertEquals((SetupFields.LEAN_MAX.def.default as Value.I16).v, rig.state.bound(leanMax))
    }

    /** Outside the clamp the firmware keeps, nothing is sent: the row's range is what it offers. */
    @Test
    fun `a value outside the seam clamp is not sent`() = runTest {
        val rig = shownTune()
        rig.model.setBound(leanMax, 1_600)
        runCurrent()
        assertEquals(emptyList<Any>(), rig.transport.writes)
        assertEquals(TuneNotice.BoundOutOfRange(leanMax), rig.state.notice)
    }

    /** A key no row owns is not a bound write, whatever asks for it. */
    @Test
    fun `a key that is not a drive bound is ignored`() = runTest {
        val rig = shownTune()
        rig.model.setBound(rig.state.keys[0], 100)
        runCurrent()
        assertEquals(emptyList<Any>(), rig.transport.writes)
    }

    /**
     * All four are in the firmware's arm-time row, so every row says the board takes the value up
     * at its next arm. Read off the drift-pinned mirror, never declared here: a field the firmware
     * moves out of that row moves this assertion with it.
     */
    @Test
    fun `every drive bound takes effect at the next arm`() {
        for (field in SetupFields.DRIVE_BOUNDS) {
            assertEquals(TakesEffect.NEXT_ARM, field.takesEffect, "${field.key}")
        }
    }

    /** The bounds are the Tune screen's rows, so Setup neither lists nor reads them. */
    @Test
    fun `the drive bounds are not Setup rows`() {
        for (field in SetupFields.DRIVE_BOUNDS) {
            assertTrue(field !in SetupFields.ALL, "${field.key} is in the Setup screen's list")
            assertNull(field.group, "${field.key} carries a Setup group")
        }
    }

    /** The evening's values all fit their rows, and the clamp's far side does not. */
    @Test
    fun `the rows take the values a tuning session uses`() {
        for (v in listOf(300, 900, 600, 1_500, 0)) {
            assertEquals(Value.I16(v), SetupFields.LEAN_MAX.parse("$v"))
        }
        assertNull(SetupFields.LEAN_MAX.parse("1501"))
        for (v in listOf(4, 12, 100)) assertEquals(Value.I16(v), SetupFields.LEAN_SLEW.parse("$v"))
        assertNull(SetupFields.LEAN_SLEW.parse("0"), "a rate of nothing would leave the limit dead")
        for (v in listOf(1_200, 6_000, 12_000, 28_500)) {
            assertEquals(Value.I16(v), SetupFields.TURN_MAX.parse("$v"))
        }
        assertNull(SetupFields.TURN_MAX.parse("28501"))
        for (v in listOf(24, 240, 1_000)) assertEquals(Value.I16(v), SetupFields.TURN_SLEW.parse("$v"))
        assertNull(SetupFields.TURN_SLEW.parse("1001"))
    }
}
