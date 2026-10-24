package com.hoverboard.remote

import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.Malformed
import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.config.WriteMismatch
import com.hoverboard.protocol.config.WriteVerified
import com.hoverboard.protocol.imu.Orientation
import com.hoverboard.protocol.l3.ConfigResp
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.Node
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
 * screen is then allowed to say about it (`specs/rider-ui.md` sections 3.4 and 3.6a).
 *
 * The model runs in `backgroundScope` (its attach collector never ends), and `advanceUntilIdle`
 * does not wait for background work, so these tests turn the scheduler with `runCurrent` and with
 * [flush], which advances past the settle window every edit waits out.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupModelTest {

    private val board = 0x01
    private val slave = 0x02
    private val mode = SetupFields.CONTROL_MODE.key
    private val limit = SetupFields.MOTOR_CURRENT_LIMIT.key
    private val method = SetupFields.MOTOR_METHOD.key
    private val name = SetupFields.DEVICE_NAME.key
    private val trims = SetupFields.LEVEL_TRIM.map { it.key }
    private val signs = SetupFields.AXIS_SIGN.map { it.key }
    private val roles = SetupFields.AXIS_ROLE.map { it.key }
    private val rider = SetupFields.RIDER_REQUIRED.key

    private fun TestScope.shown(): SetupRig = shownRig(board)

    private fun state(rig: SetupRig, key: Key): WriteState? = rig.state.writes[key]?.state

    /**
     * What an attach reads before the screen is ever shown: the rider requirement, the two
     * Machine fields the arm control states (`specs/rider-ui.md` 3.7a), the counts per amp the
     * Ride screen's current display needs (3.6) and the two facts the Travel section converts
     * hall edges with (3.6c). With no slave in the session they are read on the attached board
     * alone.
     */
    private val rideFacts = listOf(board to rider) +
        SetupFields.BOTH_BOARDS.map { board to it.key } +
        listOf(
            board to Fields.MOTOR_CURRENT_CAL.key(),
            board to Fields.MOTOR_POLE_PAIRS.key(),
            board to Fields.WHEEL_DIAMETER_MM.key(),
        )

    @Test
    fun `every field is read once per attached session, and only once the screen is shown`() = runTest {
        val rig = SetupRig(this)
        rig.transport.setAttachedBoard(board)
        runCurrent()
        // The Ride screen's own facts are read on attach; nothing else before the screen is shown.
        assertEquals(rideFacts, rig.transport.reads, "read more than the ride facts before showing")

        rig.model.onShown()
        runCurrent()
        assertEquals(rideFacts + SetupFields.ALL.map { board to it.key }, rig.transport.reads)
        assertEquals(SetupFields.ALL.associate { it.key to it.def.default }, rig.state.values)

        rig.model.onHidden()
        rig.model.onShown()
        runCurrent()
        val once = SetupFields.ALL.size + rideFacts.size
        assertEquals(once, rig.transport.reads.size, "a second showing re-read the board")
    }

    /**
     * 3.6: the Ride screen's current display turns each board's current counts into amps with
     * that board's OWN `motor.current_cal`, so the scale is read on attach for both boards,
     * whether or not Setup is ever opened.
     *
     * It is kept out of the Setup screen's bookkeeping on purpose. The field has no row on any
     * screen, so a value recorded in [BoardSetup.values] would be a row the screen cannot show,
     * and a failed read recorded in its unread set would be counted against rows that do not
     * exist.
     */
    @Test
    fun `each board's counts per amp is read on attach, apart from the Setup rows`() = runTest {
        val rig = SetupRig(this)
        val cal = Fields.MOTOR_CURRENT_CAL.key()
        rig.transport.store[board to cal] = Value.U16(500)
        rig.transport.store[slave to cal] = Value.U16(410)

        rig.transport.setAttachedBoard(board, slave)
        runCurrent()

        assertEquals(500, rig.state.countsPerAmpOf(Node.MASTER), "the attached board's own scale")
        assertEquals(410, rig.state.countsPerAmpOf(Node.SLAVE), "and never the other board's")
        assertFalse(cal in rig.state.setupOf(Node.MASTER).values, "a field with no row is not a Setup value")
        assertFalse(cal in rig.state.setupOf(Node.MASTER).unread, "nor one of its unread rows")

        rig.transport.setAttachedBoard(null)
        runCurrent()
        assertNull(rig.state.countsPerAmpOf(Node.MASTER), "a board the phone cannot reach has no scale")
    }

    /**
     * A scale that could not be read leaves the app with none, rather than with the registry
     * default: 455 counts per amp is a property of the shunt and amplifier chain on the board it
     * was measured on, and spending it on another board would put a figure on the panel that no
     * board's own data supports.
     */
    @Test
    fun `a counts-per-amp read that fails leaves the scale unknown`() = runTest {
        val rig = SetupRig(this)
        rig.transport.unreadable += Fields.MOTOR_CURRENT_CAL.key()

        rig.transport.setAttachedBoard(board)
        runCurrent()

        assertNull(rig.state.countsPerAmpOf(Node.MASTER))
    }

    /** 3.6a: there is no Apply. An edit is a write of its own field, once its settle window closes. */
    @Test
    fun `each edit writes its own field with no Apply, and the row reports it`() = runTest {
        val rig = shown()
        rig.model.edit(mode, Value.U8(1))
        rig.model.edit(limit, Value.U32(15_000))
        runCurrent()
        assertTrue(rig.transport.writes.isEmpty(), "a write left before the settle window closed")
        assertEquals(listOf(WriteState.Sending, WriteState.Sending), listOf(state(rig, mode), state(rig, limit)))
        assertTrue(rig.state.writing)

        flush()

        assertEquals(
            listOf(Triple(board, mode, Value.U8(1)), Triple(board, limit, Value.U32(15_000))),
            rig.transport.writes,
        )
        assertEquals(WriteState.Written, state(rig, mode))
        assertEquals(WriteState.Written, state(rig, limit))
        assertEquals(Value.U8(1), rig.state.values[mode])
        assertFalse(rig.state.writing)
        assertNull(rig.state.notice)
    }

    /**
     * 3.6a, the one request slot bounds the edit rate: the window is TRAILING, so a stepper held
     * down emits no write per repeat, and the value that goes is the one it stopped on.
     */
    @Test
    fun `two edits inside one settle window send once, with the later value`() = runTest {
        val rig = shown()
        rig.model.edit(limit, Value.U32(12_000))
        tick(SetupModel.EDIT_SETTLE_MS / 2)
        rig.model.edit(limit, Value.U32(15_000))
        assertTrue(rig.transport.writes.isEmpty())

        flush()

        assertEquals(listOf(Triple(board, limit, Value.U32(15_000))), rig.transport.writes)
        assertEquals(WriteState.Written, state(rig, limit))
    }

    @Test
    fun `a field edited through the whole window sends nothing until the edits stop`() = runTest {
        val rig = shown()
        repeat(6) { step ->
            rig.model.edit(limit, Value.U32(12_000L + step * 100))
            tick(SetupModel.EDIT_SETTLE_MS - 50)
        }
        assertTrue(rig.transport.writes.isEmpty(), "a held stepper emitted a write per repeat")

        flush()

        assertEquals(listOf(Triple(board, limit, Value.U32(12_500))), rig.transport.writes)
    }

    @Test
    fun `editing a field to the value the board holds writes nothing, and an out-of-range value is refused`() =
        runTest {
            val rig = shown()
            rig.model.edit(mode, Value.U8(1))
            rig.model.edit(mode, Value.U8(0))
            flush()
            assertTrue(rig.transport.writes.isEmpty())
            assertNull(rig.state.writes[mode])

            rig.model.edit(limit, Value.U32(999))
            flush()
            assertTrue(rig.transport.writes.isEmpty())
            assertEquals(SetupNotice.OutOfRange(limit), rig.state.notice)
        }

    /** A failed row taken back to the value the board holds has nothing left to report. */
    @Test
    fun `taking a refused row back to the stored value drops its result`() = runTest {
        val rig = shown()
        rig.transport.boardArmed = true
        rig.model.edit(mode, Value.U8(1))
        flush()
        assertEquals(WriteState.Refused(CfgRefusal.ARMED), state(rig, mode))

        rig.transport.boardArmed = false
        rig.model.edit(mode, Value.U8(0))
        flush()

        assertNull(rig.state.writes[mode])
        assertEquals(1, rig.transport.writes.size, "the stored value was written back for nothing")
    }

    @Test
    fun `while armed nothing is edited and nothing is sent`() = runTest {
        val rig = shown()
        rig.armed = true

        rig.model.edit(limit, Value.U32(15_000))
        rig.model.editFrame(identity.roles, identity.signs)
        rig.model.setLevel()
        flush()

        assertTrue(rig.state.writes.isEmpty())
        assertTrue(rig.transport.writes.isEmpty())
    }

    /**
     * 3.6a: `CFG_ARMED` is per ROW, not one banner. R4 refuses every config write while armed, so
     * the first refusal is enough to report on all of them, and the lane is asked once.
     */
    @Test
    fun `a board that refuses as armed says so on every queued row and asks the lane once`() = runTest {
        val rig = shown()
        rig.transport.boardArmed = true
        rig.model.edit(mode, Value.U8(1))
        rig.model.edit(limit, Value.U32(15_000))
        rig.model.edit(method, Value.U8(2))

        flush()

        assertEquals(1, rig.transport.writes.size, "the rest of the queue was sent to an armed board")
        val refused = WriteState.Refused(CfgRefusal.ARMED)
        assertEquals(refused, state(rig, mode))
        assertEquals(refused, state(rig, limit))
        assertEquals(refused, state(rig, method))
        assertNull(rig.state.notice, "a per-row refusal was also raised as a banner")
    }

    @Test
    fun `a board that arms mid-drain refuses the rest, each row saying so`() = runTest {
        val rig = shown()
        rig.transport.writeHook = { k, v ->
            if (k == mode) {
                rig.transport.store[board to k] = v
                rig.transport.boardArmed = true
                WriteVerified(v)
            } else {
                null
            }
        }
        rig.model.edit(mode, Value.U8(1))
        rig.model.edit(limit, Value.U32(15_000))

        flush()

        assertEquals(WriteState.Written, state(rig, mode))
        assertEquals(WriteState.Refused(CfgRefusal.ARMED), state(rig, limit))
    }

    @Test
    fun `a mismatched echo is not a success, and the key is read again`() = runTest {
        val rig = shown()
        rig.transport.writeHook = { _, _ -> WriteMismatch(wrote = Value.U8(2), stored = Value.U8(0)) }
        val readsBefore = rig.transport.reads.size

        rig.model.edit(method, Value.U8(2))
        flush()

        assertEquals(WriteState.Mismatch(Value.U8(0)), state(rig, method))
        assertEquals(listOf(board to method), rig.transport.reads.drop(readsBefore))
    }

    /** One field's failure is one row's: the next field is still written. */
    @Test
    fun `an answer outside the wire contract is garbled on its own row and the next field still goes`() = runTest {
        val rig = shown()
        // An undefined status byte (Busy is no longer this case: the model waits out a busy slot).
        rig.transport.writeHook = { k, _ ->
            if (k == mode) Malformed(ConfigResp(k.fieldId, k.index, 0x42, 0, ByteArray(0))) else null
        }
        rig.model.edit(mode, Value.U8(1))
        rig.model.edit(limit, Value.U32(15_000))

        flush()

        assertEquals(WriteState.Garbled, state(rig, mode))
        assertEquals(WriteState.Written, state(rig, limit))
        assertEquals(2, rig.transport.writes.size)
    }

    /** A write that landed with its answer lost is a write: the re-read says so. */
    @Test
    fun `an unanswered write whose value the board turns out to hold reads back as written`() = runTest {
        val rig = shown()
        rig.transport.writeHook = { key, value ->
            rig.transport.store[board to key] = value
            TimedOut
        }
        rig.model.edit(limit, Value.U32(15_000))
        flush()

        assertEquals(WriteState.Written, state(rig, limit))
        assertEquals(Value.U32(15_000), rig.state.values[limit])
    }

    /**
     * The link drop case, where no re-read was possible at the time (`specs/rider-ui.md` 3.4, "Link
     * drop mid-edit"): the read pass on reconnect resolves what landed, and the retry sends only the
     * row that still failed.
     */
    @Test
    fun `a drop mid-drain leaves the rest unsent, and the read pass resolves what landed`() = runTest {
        val rig = shown()
        rig.transport.writeHook = { key, value ->
            if (key == limit) {
                rig.transport.store[board to key] = value
                rig.transport.setAttachedBoard(null)
                TimedOut
            } else {
                null
            }
        }
        rig.model.edit(limit, Value.U32(15_000))
        rig.model.edit(method, Value.U8(1))
        flush()
        assertEquals(WriteState.NotSent, state(rig, method), "a write went out with no board attached")

        rig.transport.writeHook = { _, _ -> null }
        rig.transport.setAttachedBoard(board)
        runCurrent()
        assertEquals(WriteState.Written, state(rig, limit), "the landed write was not resolved")
        assertEquals(WriteState.NotSent, state(rig, method))

        val writesBefore = rig.transport.writes.size
        rig.model.retry(method)
        flush()
        assertEquals(listOf(Triple(board, method, Value.U8(1))), rig.transport.writes.drop(writesBefore))
        assertEquals(WriteState.Written, state(rig, method))
    }

    @Test
    fun `an edit with no board attached reports not sent on its row`() = runTest {
        val rig = shown()
        rig.model.edit(mode, Value.U8(1))
        rig.transport.setAttachedBoard(null)

        flush()

        assertTrue(rig.transport.writes.isEmpty())
        assertEquals(WriteState.NotSent, state(rig, mode))
    }

    @Test
    fun `writes name the attached board`() = runTest {
        val rig = SetupRig(this)
        rig.transport.setAttachedBoard(0x05)
        rig.model.onShown()
        runCurrent()
        rig.model.edit(mode, Value.U8(1))
        flush()

        assertTrue(rig.transport.reads.all { it.first == 0x05 })
        assertEquals(listOf(0x05), rig.transport.writes.map { it.first })
    }

    /**
     * 3.6a: the restart is CONFIRMED by `boot_count` on the wire, not asserted by a button. The boot
     * sequence reads every field, so a restart puts every write this session into effect and leaves
     * no row with anything to report, and the re-read learns what the board now runs.
     */
    @Test
    fun `a boot tag change puts the writes into effect and re-reads the board`() = runTest {
        val rig = shown()
        rig.emitBoot(bootTag = 7)
        runCurrent()
        rig.model.edit(method, Value.U8(2))
        rig.model.edit(name, Value.Str("rover-a"))
        flush()
        assertEquals(WriteState.Written, state(rig, method))
        assertEquals(WriteState.Written, state(rig, name))

        // A restart the app did not ask for, reported by the wire.
        val readsBefore = rig.transport.reads.size
        rig.emitBoot(bootTag = 8)
        runCurrent()

        assertTrue(rig.state.writes.isEmpty(), "a write record survived the restart that applied it")
        // The ride facts first (what the arm control states), then the whole screen.
        assertEquals(
            rideFacts + SetupFields.ALL.map { board to it.key },
            rig.transport.reads.drop(readsBefore),
            "no re-read after the restart",
        )
        assertEquals(Value.U8(2), rig.state.running[method], "the re-read did not learn what the board now runs")
    }

    @Test
    fun `the first boot tag seen is no restart`() = runTest {
        val rig = shown()
        rig.model.edit(name, Value.Str("rover-a"))
        flush()
        rig.emitBoot(bootTag = 3)
        runCurrent()

        assertEquals(WriteState.Written, state(rig, name))
    }

    @Test
    fun `a different board drops the write records`() = runTest {
        val rig = shown()
        rig.model.edit(mode, Value.U8(1))
        flush()
        rig.transport.setAttachedBoard(null)
        rig.transport.setAttachedBoard(0x02)
        runCurrent()

        assertTrue(rig.state.writes.isEmpty())
        assertEquals(SetupNotice.BoardChanged(board), rig.state.notice)
        assertEquals(0x02, rig.state.board)
    }

    // --- The IMU frame: one object with an invariant across its fields ---

    /**
     * 3.6a, the frame's cross-field transient: the eight indices go one write at a time, and the two
     * role indices are written BACK TO BACK, so the window in which the board can hold a duplicate
     * pair is one write wide and nothing else is interleaved into it.
     */
    @Test
    fun `a pose writes the six signs and then the role pair back to back`() = runTest {
        val rig = shown() // every sign and role stored unset, so the whole frame is a change
        val pose = Orientation.Pose(Orientation.Face.STOCK_FORWARD_EDGE_DOWN, Orientation.Heading.STOCK_LEFT)
        val frame = Orientation.frameOf(pose)
        assertEquals(listOf(1, 3), frame.roles)

        rig.model.editFrame(frame.roles, frame.signs)
        assertEquals(pose, rig.state.intendedPose)
        assertTrue(rig.state.frameWriting)
        flush()

        val order = rig.transport.writes.map { it.second }
        assertEquals(signs + roles, order, "the frame was not written signs first, then the role pair")
        assertEquals(1, order.indexOf(roles[1]) - order.indexOf(roles[0]), "the role pair was not back to back")
        assertEquals(frame.roles, rig.state.storedRoles)
        assertEquals(frame.signs, rig.state.storedSigns)
        assertEquals(pose, rig.state.storedPose)
        assertFalse(rig.state.frameWriting)
        assertFalse(rig.state.orientationSettled, "a written frame is not what the board is running yet")
    }

    @Test
    fun `a mirrored frame is refused before any write`() = runTest {
        val rig = shown()
        val mirrored = identity.signs.toMutableList().also { it[0] = -it[0] }

        rig.model.editFrame(identity.roles, mirrored)
        flush()

        assertEquals(SetupNotice.FrameRefused(Orientation.Refusal.ACCEL_MIRRORED), rig.state.notice)
        assertTrue(rig.transport.writes.isEmpty())
    }

    @Test
    fun `a sign or role is never written on its own, only as part of a frame`() = runTest {
        val rig = shown()

        rig.model.edit(signs[0], Value.I32(1))
        rig.model.edit(roles[0], Value.U8(2))
        flush()

        assertTrue(rig.transport.writes.isEmpty(), "a bare sign or role write reached the board")
        assertTrue(rig.state.writes.isEmpty())
    }

    @Test
    fun `a frame whose roles the stored ones already resolve to leaves the roles alone`() = runTest {
        val rig = shown() // roles stored unset, which resolve to the compiled roles
        val stock = Orientation.frameOf(Orientation.STOCK_POSE)

        rig.model.editFrame(stock.roles, stock.signs)
        flush()

        assertTrue(roles.none { it in rig.state.writes }, "unset roles were rewritten as the compiled ones")
        assertEquals(stock.signs, rig.state.storedSigns)
    }

    /**
     * The frame is judged through the roles the board STORES. A sign map that is a reflection under
     * the compiled roles is a proper rotation under the transposition UP = X, PITCH_RATE = Y, so a
     * board holding both is safe, and a flat pose then writes the compiled roles back with its signs.
     */
    @Test
    fun `the stored roles decide whether the stored signs are a rotation`() = runTest {
        val legalUnderXY = listOf(1, 1, -1, 1, 1, -1)
        val rig = SetupRig(this)
        signs.forEachIndexed { i, k -> rig.transport.store[board to k] = Value.I32(legalUnderXY[i]) }
        rig.transport.store[board to roles[0]] = Value.U8(1)
        rig.transport.store[board to roles[1]] = Value.U8(2)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()

        assertEquals(listOf(1, 2), rig.state.storedRoles)
        assertFalse(rig.state.storedFrameUnsafe, "a rotation under the stored roles was judged mirrored")

        rig.model.editFrame(identity.roles, identity.signs)
        flush()

        assertNull(rig.state.notice)
        assertEquals(Orientation.DEFAULT_ROLES, rig.state.storedRoles)
        assertEquals(identity.signs, rig.state.storedSigns)
    }

    /**
     * P1-2 of the Setup audit, under write-on-change: a frame write that stops part way leaves the
     * stored frame mirrored, which the screen warns about until the frame is written whole. No write
     * order avoids that intermediate, so completing it is the only remedy.
     */
    @Test
    fun `a frame write that stops part way leaves a frame the board refuses, and finishing it clears that`() =
        runTest {
            val rig = storing(Orientation.REFERENCE)
            // ax is written and verified; az goes unanswered and the board keeps its old sign.
            rig.transport.writeHook = { key, _ -> if (key == signs[2]) TimedOut else null }
            rig.model.editFrame(identity.roles, identity.signs)
            flush()

            assertEquals(WriteState.Unanswered, state(rig, signs[2]))
            val stored = checkNotNull(rig.state.storedSigns)
            val roleMap = checkNotNull(rig.state.storedRoles)
            assertEquals(Orientation.Refusal.ACCEL_MIRRORED, Orientation.check(stored, roleMap))
            assertTrue(rig.state.storedFrameUnsafe)

            // An edit of another field still writes: it is its own transaction.
            rig.model.edit(mode, Value.U8(1))
            flush()
            assertEquals(WriteState.Written, state(rig, mode))
            assertTrue(rig.state.storedFrameUnsafe, "the stored frame is still one the board refuses")

            rig.transport.writeHook = { _, _ -> null }
            rig.model.editFrame(identity.roles, identity.signs)
            flush()
            assertEquals(identity.signs, rig.state.storedSigns)
            assertFalse(rig.state.storedFrameUnsafe)
        }

    @Test
    fun `a board already holding a mirrored frame is flagged, and an unrelated edit still writes`() = runTest {
        val rig = storing(listOf(-1, 1, 1, -1, 1, -1))
        assertTrue(rig.state.writes.isEmpty())
        assertTrue(rig.state.storedFrameUnsafe)

        rig.model.edit(mode, Value.U8(1))
        flush()

        assertEquals(listOf(Triple(board, mode, Value.U8(1))), rig.transport.writes)
        assertNull(rig.state.notice)
        assertEquals(WriteState.Written, state(rig, mode))
    }

    @Test
    fun `a frame write goes out whole even where an index was never read`() = runTest {
        val rig = SetupRig(this)
        rig.transport.unreadable += signs[1]
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        assertNull(rig.state.storedSigns)

        val frame = Orientation.frameOf(
            Orientation.Pose(Orientation.Face.STOCK_FORWARD_EDGE_DOWN, Orientation.Heading.STOCK_LEFT),
        )
        rig.model.editFrame(frame.roles, frame.signs)
        flush()

        assertEquals(signs + roles, rig.transport.writes.map { it.second })
        assertEquals(frame.signs, rig.state.storedSigns)
    }

    // --- Flows over telemetry ---

    @Test
    fun `set level writes the trim the board is running plus the reading`() = runTest {
        val rig = SetupRig(this)
        rig.transport.store[board to trims[0]] = Value.I16(10)
        rig.transport.store[board to trims[1]] = Value.I16(-20)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        rig.emitAttitude(pitch = 305, roll = -50)

        rig.model.setLevel()
        flush()

        assertEquals(
            listOf(Triple(board, trims[0], Value.I16(315)), Triple(board, trims[1], Value.I16(-70))),
            rig.transport.writes,
        )
    }

    /** The level trim is boot-read, so until the board restarts the running trim is unknown. */
    @Test
    fun `set level refuses while a trim it wrote is not in effect`() = runTest {
        val rig = shown()
        rig.emitAttitude(pitch = 305, roll = -50)
        rig.model.setLevel()
        flush()
        assertEquals(WriteState.Written, state(rig, trims[0]))

        rig.model.setLevel()

        assertEquals(SetupNotice.LevelUnavailable(Blocked.AWAITING_RESTART), rig.state.notice)
    }

    /** The frame is arm-time, so a written orientation blocks set level until the next arm. */
    @Test
    fun `set level refuses while an orientation it wrote waits for the next arm`() = runTest {
        val rig = shown()
        rig.emitAttitude(pitch = 305, roll = -50)
        rig.model.editFrame(identity.roles, identity.signs)
        flush()

        rig.model.setLevel()

        assertEquals(SetupNotice.LevelUnavailable(Blocked.AWAITING_ARM), rig.state.notice)
        assertTrue(trims.none { it in rig.state.writes })
    }

    @Test
    fun `set level with no telemetry says so`() = runTest {
        val rig = shown()
        rig.model.setLevel()
        assertEquals(SetupNotice.LevelUnavailable(Blocked.NO_TELEMETRY), rig.state.notice)
    }

    /**
     * The board ANSWERED and the answer did not decode, which is the state a non-UTF-8
     * `DEVICE_NAME` leaves behind: the board stopped validating `STR` records as text in shrink
     * round 2 and serves whatever bytes it holds, while the mirror's value model carries a
     * `String` (`specs/storage-layer.md`). Reporting that as "Not read" is a false statement about
     * a board that replied, so the two states are separate and a key is in exactly one of them.
     */
    @Test
    fun `a read the board answered and the app could not decode is garbled, not unread`() = runTest {
        val rig = SetupRig(this)
        val name = SetupFields.DEVICE_NAME.key
        rig.transport.garbledReads += name
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()

        assertEquals(setOf(name), rig.state.garbled)
        assertFalse(name in rig.state.unread, "an answered read is not an unread one")
        assertNull(rig.state.values[name])
    }

    /** A later read that does decode clears the garbled mark rather than leaving it standing. */
    @Test
    fun `a garbled key stops being garbled once a read of it decodes`() = runTest {
        val rig = SetupRig(this)
        val name = SetupFields.DEVICE_NAME.key
        rig.transport.garbledReads += name
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        assertEquals(setOf(name), rig.state.garbled)

        rig.transport.garbledReads -= name
        rig.model.refresh()
        flush()

        assertTrue(rig.state.garbled.isEmpty())
        assertEquals(Fields.DEVICE_NAME.default, rig.state.values[name])
    }

    @Test
    fun `a field whose read fails is marked unread, and set level then says the trims are not read`() = runTest {
        val rig = SetupRig(this)
        rig.transport.unreadable += trims[1]
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        assertEquals(setOf(trims[1]), rig.state.unread)
        assertNull(rig.state.values[trims[1]])

        rig.emitAttitude(pitch = 305, roll = -50)
        rig.model.setLevel()

        assertEquals(SetupNotice.LevelUnavailable(Blocked.NOT_READ), rig.state.notice)
        assertTrue(rig.state.writes.isEmpty())
    }

    @Test
    fun `the rotation check judges level and the forward-lean sign from telemetry`() = runTest {
        val rig = shown()
        rig.emitAttitude(pitch = 120, roll = -300)
        rig.model.checkLevel()
        assertEquals(CheckResult.PASS, rig.state.rotationCheck.level)

        rig.emitAttitude(pitch = 0, roll = 17_900) // the upside-down reading a wrong map gives
        rig.model.checkLevel()
        assertEquals(CheckResult.FAIL, rig.state.rotationCheck.level)

        rig.emitAttitude(pitch = -1_500, roll = 0)
        rig.model.checkForwardLean()
        assertEquals(CheckResult.PASS, rig.state.rotationCheck.forwardLean)
        rig.emitAttitude(pitch = 1_500, roll = 0)
        rig.model.checkForwardLean()
        assertEquals(CheckResult.FAIL, rig.state.rotationCheck.forwardLean)
        rig.emitAttitude(pitch = -500, roll = 0)
        rig.model.checkForwardLean()
        assertEquals(CheckResult.INCONCLUSIVE, rig.state.rotationCheck.forwardLean)
    }

    @Test
    fun `the rotation check refuses while a written orientation waits for the next arm`() = runTest {
        val rig = shown()
        rig.emitAttitude(pitch = 0, roll = 0)
        rig.model.editFrame(identity.roles, identity.signs)
        flush()

        rig.model.checkLevel()

        assertEquals(SetupNotice.CheckUnavailable(Blocked.AWAITING_ARM), rig.state.notice)
        assertNull(rig.state.rotationCheck.level)
    }

    /**
     * Two flat poses whose frames are diagonal under the compiled roles: the board component side up
     * facing stock-rear is the identity map, and the stock pose the family's half turn about Z.
     */
    private val identity = Orientation.frameOf(
        Orientation.Pose(Orientation.Face.COMPONENT_UP, Orientation.Heading.STOCK_REAR),
    )

    /**
     * Unset signs are no pose; the compiled reference stored explicitly is the board component side
     * down on this family, and the family yaw stored over unset roles is the stock pose.
     */
    @Test
    fun `an unset board reads as no pose and the stored family yaw as the stock pose`() = runTest {
        val unset = shown()
        assertNull(unset.state.storedPose)
        assertNull(unset.state.intendedPose)

        val reference = storing(Orientation.REFERENCE)
        assertEquals(Orientation.REFERENCE, reference.state.storedSigns)
        assertEquals(Orientation.Face.COMPONENT_DOWN, reference.state.storedPose?.face)

        val yaw = storing(listOf(-1, -1, 1, -1, -1, 1))
        assertEquals(listOf(0, 0), yaw.state.storedRoles)
        assertEquals(Orientation.STOCK_POSE, yaw.state.storedPose)
    }

    /** A rig whose board stores [map] as its sign map, shown and read. */
    private fun TestScope.storing(map: List<Int>): SetupRig = SetupRig(this).also { rig ->
        signs.forEachIndexed { i, k -> rig.transport.store[board to k] = Value.I32(map[i]) }
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
    }
}
