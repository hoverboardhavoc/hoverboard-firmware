package com.hoverboard.remote

import com.hoverboard.protocol.config.Busy
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.config.WriteMismatch
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * [SetupModel] against the fake transport's in-memory board: what is sent, when, and what the
 * screen is then allowed to say about it (`specs/rider-ui.md` section 3.4).
 *
 * The model runs in `backgroundScope` (its attach collector never ends), and `advanceUntilIdle`
 * does not wait for background work, so these tests turn the scheduler with `runCurrent`: the fake
 * board answers without delay, so every operation finishes at the current virtual time.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupModelTest {

    private val board = 0x01
    private val mode = SetupFields.CONTROL_MODE.key
    private val limit = SetupFields.MOTOR_CURRENT_LIMIT.key
    private val method = SetupFields.MOTOR_METHOD.key
    private val trims = SetupFields.LEVEL_TRIM.map { it.key }
    private val signs = SetupFields.AXIS_SIGN.map { it.key }

    private class Rig(scope: TestScope) {
        var armed = false
        val transport = FakeHoverboardTransport(scope.testScheduler).apply {
            for (f in SetupFields.ALL) defaults[f.key] = f.def.default
        }
        val model = SetupModel(transport, scope.backgroundScope) { armed }
        val state: SetupState get() = model.state.value
    }

    /** A rig attached to [board] with the Setup screen shown and its read pass done. */
    private fun TestScope.shown(): Rig = Rig(this).also {
        it.transport.setAttachedBoard(board)
        it.model.onShown()
        runCurrent()
    }

    private fun telemetry(rig: Rig, pitch: Int, roll: Int) =
        rig.transport.emitCyclicState(CyclicState(pitch, roll, 0, 0, 0, 0, 0))

    @Test
    fun `every field is read once per attached session, and only once the screen is shown`() = runTest {
        val rig = Rig(this)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertTrue(rig.transport.reads.isEmpty(), "read before the screen was shown")

        rig.model.onShown()
        runCurrent()
        assertEquals(SetupFields.ALL.map { board to it.key }, rig.transport.reads)
        assertEquals(SetupFields.ALL.associate { it.key to it.def.default }, rig.state.values)

        rig.model.onHidden()
        rig.model.onShown()
        runCurrent()
        assertEquals(SetupFields.ALL.size, rig.transport.reads.size, "a second showing re-read the board")
    }

    @Test
    fun `nothing is written until Apply, and each write is verified then marked staged`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.model.stage(limit, Value.U32(15_000))
        runCurrent()
        assertTrue(rig.transport.writes.isEmpty())
        assertEquals(listOf(mode, limit), rig.state.pending.keys.toList())

        rig.model.apply()
        runCurrent()

        assertEquals(
            listOf(Triple(board, mode, Value.U8(1)), Triple(board, limit, Value.U32(15_000))),
            rig.transport.writes,
        )
        assertTrue(rig.state.pending.isEmpty())
        assertEquals(mapOf(mode to Value.U8(1), limit to Value.U32(15_000)), rig.state.staged)
        assertEquals(Value.U8(1), rig.state.values[mode])
        assertTrue(rig.state.awaitingPowerCycle)
        assertNull(rig.state.notice)
    }

    @Test
    fun `staging the stored value leaves nothing pending, and a value outside the range is refused`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.model.stage(mode, Value.U8(0))
        assertTrue(rig.state.pending.isEmpty())

        rig.model.stage(limit, Value.U32(999))
        assertTrue(rig.state.pending.isEmpty())
        assertEquals(SetupNotice.OutOfRange(limit), rig.state.notice)
    }

    @Test
    fun `while armed every edit and every Apply is refused before anything is sent`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.armed = true

        rig.model.stage(limit, Value.U32(15_000))
        rig.model.stageRotation(Orientation.Rotation.IDENTITY)
        rig.model.apply()
        runCurrent()

        assertEquals(listOf(mode), rig.state.pending.keys.toList())
        assertTrue(rig.transport.writes.isEmpty())
        assertEquals(SetupNotice.ReadOnlyWhileArmed, rig.state.notice)
    }

    @Test
    fun `a board that refuses as armed stops the Apply and keeps the basket`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.model.stage(limit, Value.U32(15_000))
        rig.transport.boardArmed = true

        rig.model.apply()
        runCurrent()

        assertEquals(1, rig.transport.writes.size, "the Apply went on after a refusal")
        assertEquals(SetupNotice.BoardRefused(mode, CfgRefusal.ARMED), rig.state.notice)
        assertEquals(listOf(mode, limit), rig.state.pending.keys.toList())
        assertTrue(rig.state.staged.isEmpty())
        assertFalse(rig.state.awaitingPowerCycle)
    }

    @Test
    fun `a mismatched echo is not a success, and the key is read again`() = runTest {
        val rig = shown()
        rig.model.stage(method, Value.U8(2))
        rig.transport.writeHook = { _, _ -> WriteMismatch(wrote = Value.U8(2), stored = Value.U8(0)) }
        val readsBefore = rig.transport.reads.size

        rig.model.apply()
        runCurrent()

        assertEquals(SetupNotice.Mismatch(method, Value.U8(2), Value.U8(0)), rig.state.notice)
        assertEquals(mapOf(method to Value.U8(2)), rig.state.pending)
        assertTrue(rig.state.staged.isEmpty())
        assertEquals(listOf(board to method), rig.transport.reads.drop(readsBefore))
    }

    /**
     * The resume pass (`specs/rider-ui.md` 3.4, "Link drop mid-edit"): a write that landed but whose
     * answer was lost is found by the re-read on reconnect and is staged, not written again; only
     * what the board does not hold is written on the next Apply.
     */
    @Test
    fun `after a drop mid-Apply only the diff is written again`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.model.stage(limit, Value.U32(15_000))
        rig.model.stage(method, Value.U8(1))
        rig.transport.writeHook = { key, value ->
            if (key == limit) {
                rig.transport.store[board to key] = value
                TimedOut
            } else {
                null
            }
        }
        rig.model.apply()
        runCurrent()
        assertEquals(SetupNotice.Unanswered(limit), rig.state.notice)
        assertEquals(listOf(limit, method), rig.state.pending.keys.toList())

        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(listOf(method), rig.state.pending.keys.toList(), "the landed write is still pending")
        assertEquals(setOf(mode, limit), rig.state.staged.keys)

        rig.transport.writeHook = { _, _ -> null }
        val writesBefore = rig.transport.writes.size
        rig.model.apply()
        runCurrent()
        assertEquals(listOf(Triple(board, method, Value.U8(1))), rig.transport.writes.drop(writesBefore))
        assertTrue(rig.state.awaitingPowerCycle)
    }

    @Test
    fun `a mirrored sign map is refused before any write`() = runTest {
        val rig = shown()
        // Stored map all unset (the reference, a half turn about Y). Flipping one sign mirrors it.
        rig.model.stage(signs[0], Value.I32(1))

        rig.model.apply()
        runCurrent()

        assertEquals(SetupNotice.FrameRefused(Orientation.Refusal.ACCEL_MIRRORED), rig.state.notice)
        assertTrue(rig.transport.writes.isEmpty())
    }

    @Test
    fun `a rotation stages all six signs and they are written as one checked frame`() = runTest {
        val rig = shown()
        rig.model.stageRotation(Orientation.Rotation.HALF_TURN_Z)
        assertEquals(Orientation.Rotation.HALF_TURN_Z.signs, rig.state.intendedSigns)
        assertFalse(rig.state.orientationSettled)

        rig.model.apply()
        runCurrent()

        assertEquals(
            signs.zip(Orientation.Rotation.HALF_TURN_Z.signs.map { Value.I32(it) }),
            rig.transport.writes.map { it.second to it.third },
        )
        assertEquals(Orientation.Rotation.HALF_TURN_Z.signs, rig.state.storedSigns)
    }

    @Test
    fun `set level stages the trim the board is running plus the reading`() = runTest {
        val rig = Rig(this)
        rig.transport.store[board to trims[0]] = Value.I16(10)
        rig.transport.store[board to trims[1]] = Value.I16(-20)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        telemetry(rig, pitch = 305, roll = -50)

        rig.model.setLevel()

        assertEquals(mapOf(trims[0] to Value.I16(315), trims[1] to Value.I16(-70)), rig.state.pending)
        assertTrue(rig.transport.writes.isEmpty(), "set level wrote without an Apply")
    }

    @Test
    fun `set level refuses while the trim is staged and the running trim is unknown`() = runTest {
        val rig = shown()
        telemetry(rig, pitch = 305, roll = -50)
        rig.model.setLevel()
        rig.model.apply()
        runCurrent()
        assertEquals(setOf(trims[0], trims[1]), rig.state.staged.keys)

        rig.model.setLevel()

        assertEquals(SetupNotice.LevelUnavailable(Blocked.NOT_APPLIED), rig.state.notice)
        assertTrue(rig.state.pending.isEmpty())
    }

    @Test
    fun `set level with no telemetry says so`() = runTest {
        val rig = shown()
        rig.model.setLevel()
        assertEquals(SetupNotice.LevelUnavailable(Blocked.NO_TELEMETRY), rig.state.notice)
    }

    @Test
    fun `the power-cycle confirmation needs the link to have dropped since the Apply`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.model.apply()
        runCurrent()

        rig.model.confirmPowerCycled()
        assertEquals(setOf(mode), rig.state.staged.keys, "confirmed a power-cycle the link never saw")

        rig.transport.setAttachedBoard(null)
        assertTrue(rig.state.linkDroppedSinceApply)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        val readsBefore = rig.transport.reads.size
        rig.model.confirmPowerCycled()
        runCurrent()

        assertTrue(rig.state.staged.isEmpty())
        assertFalse(rig.state.awaitingPowerCycle)
        assertEquals(SetupFields.ALL.size, rig.transport.reads.size - readsBefore, "no re-read after the power-cycle")
    }

    @Test
    fun `the rotation check judges level and the forward-lean sign from telemetry`() = runTest {
        val rig = shown()
        telemetry(rig, pitch = 120, roll = -300)
        rig.model.checkLevel()
        assertEquals(CheckResult.PASS, rig.state.rotationCheck.level)

        telemetry(rig, pitch = 0, roll = 17_900) // the upside-down reading a wrong map gives
        rig.model.checkLevel()
        assertEquals(CheckResult.FAIL, rig.state.rotationCheck.level)

        telemetry(rig, pitch = -1_500, roll = 0)
        rig.model.checkForwardLean()
        assertEquals(CheckResult.PASS, rig.state.rotationCheck.forwardLean)
        telemetry(rig, pitch = 1_500, roll = 0)
        rig.model.checkForwardLean()
        assertEquals(CheckResult.FAIL, rig.state.rotationCheck.forwardLean)
        telemetry(rig, pitch = -500, roll = 0)
        rig.model.checkForwardLean()
        assertEquals(CheckResult.INCONCLUSIVE, rig.state.rotationCheck.forwardLean)
    }

    @Test
    fun `the rotation check refuses while the orientation is staged but not applied`() = runTest {
        val rig = shown()
        telemetry(rig, pitch = 0, roll = 0)
        rig.model.stageRotation(Orientation.Rotation.IDENTITY)
        rig.model.apply()
        runCurrent()

        rig.model.checkLevel()

        assertEquals(SetupNotice.CheckUnavailable(Blocked.NOT_APPLIED), rig.state.notice)
        assertNull(rig.state.rotationCheck.level)
    }

    @Test
    fun `a different board drops the basket and the staged marks`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(0x02)
        runCurrent()

        assertTrue(rig.state.pending.isEmpty())
        assertEquals(SetupNotice.BoardChanged(board), rig.state.notice)
        assertEquals(0x02, rig.state.board)
    }

    @Test
    fun `writes name the attached board`() = runTest {
        val rig = Rig(this)
        rig.transport.setAttachedBoard(0x05)
        rig.model.onShown()
        runCurrent()
        rig.model.stage(mode, Value.U8(1))
        rig.model.apply()
        runCurrent()

        assertTrue(rig.transport.reads.all { it.first == 0x05 })
        assertEquals(listOf(0x05), rig.transport.writes.map { it.first })
    }

    /** A rig whose board stores [rotation] as its sign map, shown and read. */
    private fun TestScope.storing(rotation: Orientation.Rotation): Rig = Rig(this).also { rig ->
        signs.forEachIndexed { i, k -> rig.transport.store[board to k] = Value.I32(rotation.signs[i]) }
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
    }

    /**
     * P1-2 of the Setup audit: a partial rotation Apply leaves the stored frame mirrored, and a
     * "Discard all" must not then turn the screen into a power-cycle instruction over it.
     */
    @Test
    fun `discarding after a partial rotation Apply holds the power-cycle instruction back`() = runTest {
        val rig = storing(Orientation.Rotation.HALF_TURN_Y)
        rig.model.stageRotation(Orientation.Rotation.IDENTITY)
        // ax is written and verified; az goes unanswered and the board keeps its old sign.
        rig.transport.writeHook = { key, _ -> if (key == signs[2]) TimedOut else null }
        rig.model.apply()
        runCurrent()
        assertEquals(setOf(signs[0]), rig.state.staged.keys)
        assertEquals(Orientation.Refusal.ACCEL_MIRRORED, Orientation.check(checkNotNull(rig.state.storedSigns)))

        rig.model.discardAll()

        assertTrue(rig.state.pending.isEmpty())
        assertTrue(rig.state.storedFrameUnsafe)
        assertFalse(rig.state.awaitingPowerCycle, "a power-cycle instruction over a mirrored stored frame")
        assertTrue(rig.state.powerCycleHeld)

        // An Apply of another field runs the frame check too, and refuses.
        rig.transport.writeHook = { _, _ -> null }
        val writesBefore = rig.transport.writes.size
        rig.model.stage(mode, Value.U8(1))
        rig.model.apply()
        runCurrent()
        assertEquals(SetupNotice.FrameRefused(Orientation.Refusal.ACCEL_MIRRORED), rig.state.notice)
        assertEquals(writesBefore, rig.transport.writes.size)
        rig.model.discardAll()

        // Nor does a link drop let the power-cycle be confirmed.
        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        rig.model.confirmPowerCycled()
        assertEquals(setOf(signs[0]), rig.state.staged.keys)

        // Completing the frame brings the instruction back.
        rig.model.stageRotation(Orientation.Rotation.IDENTITY)
        rig.model.apply()
        runCurrent()
        assertEquals(Orientation.Rotation.IDENTITY.signs, rig.state.storedSigns)
        assertFalse(rig.state.storedFrameUnsafe)
        assertTrue(rig.state.awaitingPowerCycle)
    }

    /** P1-3: pitch and roll come through the booted frame, so a pending orientation blocks set level. */
    @Test
    fun `set level refuses while the orientation is pending or staged`() = runTest {
        val rig = shown()
        telemetry(rig, pitch = 305, roll = -50)
        rig.model.stageRotation(Orientation.Rotation.IDENTITY)

        rig.model.setLevel()
        assertEquals(SetupNotice.LevelUnavailable(Blocked.NOT_APPLIED), rig.state.notice)
        assertTrue(trims.none { it in rig.state.pending })

        rig.model.apply()
        runCurrent()
        rig.model.setLevel()
        assertEquals(SetupNotice.LevelUnavailable(Blocked.NOT_APPLIED), rig.state.notice)
        assertTrue(trims.none { it in rig.state.pending })
    }

    @Test
    fun `a field whose read fails is marked unread, and set level then says the trims are not read`() = runTest {
        val rig = Rig(this)
        rig.transport.unreadable += trims[1]
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        assertEquals(setOf(trims[1]), rig.state.unread)
        assertNull(rig.state.values[trims[1]])

        telemetry(rig, pitch = 305, roll = -50)
        rig.model.setLevel()

        assertEquals(SetupNotice.LevelUnavailable(Blocked.NOT_READ), rig.state.notice)
        assertTrue(rig.state.pending.isEmpty())
    }

    @Test
    fun `a sign index that was never read leaves the frame unknown, and nothing is written`() = runTest {
        val rig = Rig(this)
        rig.transport.unreadable += signs[1]
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        rig.model.stage(signs[0], Value.I32(-1))

        rig.model.apply()
        runCurrent()

        assertEquals(SetupNotice.FrameUnknown, rig.state.notice)
        assertTrue(rig.transport.writes.isEmpty())
    }

    @Test
    fun `an Apply with no board attached sends nothing and says so`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.transport.setAttachedBoard(null)

        rig.model.apply()
        runCurrent()

        assertEquals(SetupNotice.NotAttached, rig.state.notice)
        assertTrue(rig.transport.writes.isEmpty())
        assertEquals(listOf(mode), rig.state.pending.keys.toList())
    }

    @Test
    fun `an answer outside the wire contract stops the Apply as garbled`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.model.stage(limit, Value.U32(15_000))
        rig.transport.writeHook = { _, _ -> Busy }

        rig.model.apply()
        runCurrent()

        assertEquals(SetupNotice.Garbled(mode), rig.state.notice)
        assertEquals(1, rig.transport.writes.size)
        assertEquals(listOf(mode, limit), rig.state.pending.keys.toList())
        assertTrue(rig.state.staged.isEmpty())
    }

    @Test
    fun `arming mid-Apply stops it before the next write and keeps the rest of the basket`() = runTest {
        val rig = shown()
        rig.model.stage(mode, Value.U8(1))
        rig.model.stage(limit, Value.U32(15_000))
        rig.transport.writeHook = { _, _ ->
            rig.armed = true
            null
        }

        rig.model.apply()
        runCurrent()

        assertEquals(listOf(Triple(board, mode, Value.U8(1))), rig.transport.writes)
        assertEquals(SetupNotice.ReadOnlyWhileArmed, rig.state.notice)
        assertEquals(listOf(limit), rig.state.pending.keys.toList())
        assertEquals(setOf(mode), rig.state.staged.keys)
        assertFalse(rig.state.awaitingPowerCycle)
    }
}
