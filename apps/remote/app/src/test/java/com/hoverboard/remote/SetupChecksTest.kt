package com.hoverboard.remote

import com.hoverboard.protocol.checks.Check
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The Setup screen runs the value checks before every write (`specs/config-read-once.md` section
 * 4): the range table and the IMU frame, ported from the CLI in `protocol-kotlin`.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupChecksTest {

    private val board = 0x01

    /**
     * The checks refuse what the row's own editor would take: the floor row takes any i16, and the
     * range table holds it to 0..6000. Nothing is written, and the notice names the field and the
     * reason.
     */
    @Test
    fun `a value the range table refuses is never written and the notice names the field`() = runTest {
        val rig = shownRig(board)
        val floor = SetupFields.BATTERY_FLOOR.key
        rig.model.edit(floor, Value.I16(-1))
        flush()

        assertTrue(rig.transport.writes.isEmpty())
        val refused = rig.state.notice as SetupNotice.CheckRefused
        assertEquals(Check.RANGE, refused.refusal.check)
        assertEquals("machine.battery.floor: -1 is outside 0..6000", refused.refusal.msg)

        rig.model.edit(floor, Value.I16(2_000))
        flush()
        assertEquals(listOf(Triple(board, floor, Value.I16(2_000))), rig.transport.writes)
    }

    /**
     * `imu.model` switches the IMU group on (`shared/config-checks/group.json`, "imu.model alone"):
     * Setup runs the group check on the bus the board stores, so a model with no bus is refused as
     * the CLI refuses it, naming `imu.scl`, and a model with the whole bus is written.
     */
    @Test
    fun `an imu model with no bus stored is refused as the cli refuses it`() = runTest {
        val rig = shownRig(board)
        val model = SetupFields.IMU_MODEL.key
        rig.model.edit(model, Value.U8(Fields.ImuModel.CLONE_2E))
        flush()

        assertTrue(rig.transport.writes.isEmpty())
        val refused = rig.state.notice as SetupNotice.CheckRefused
        assertEquals(Check.GROUP, refused.refusal.check)
        assertEquals(Fields.IMU_SCL_PIN.key(), refused.refusal.key)
    }

    /** Model 0 is the IMU off: the group check passes whatever the bus holds. */
    @Test
    fun `an imu model of none is written with no bus stored`() = runTest {
        val rig = SetupRig(this)
        val model = SetupFields.IMU_MODEL.key
        rig.transport.store[board to model] = Value.U8(Fields.ImuModel.CLONE_2E)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        rig.model.edit(model, Value.U8(Fields.ImuModel.NONE))
        flush()

        assertEquals(listOf(Triple(board, model, Value.U8(Fields.ImuModel.NONE))), rig.transport.writes)
    }

    @Test
    fun `an imu model with the whole bus stored is written`() = runTest {
        val rig = SetupRig(this)
        rig.transport.store[board to Fields.IMU_SCL_PIN.key()] = Value.PinAf(1, 6, 1)
        rig.transport.store[board to Fields.IMU_SDA_PIN.key()] = Value.PinAf(1, 7, 1)
        rig.transport.store[board to Fields.IMU_I2C.key()] = Value.U8(0)
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        val model = SetupFields.IMU_MODEL.key
        rig.model.edit(model, Value.U8(Fields.ImuModel.CLONE_2E))
        flush()

        assertEquals(listOf(Triple(board, model, Value.U8(Fields.ImuModel.CLONE_2E))), rig.transport.writes)
    }

    /** A bus that could not be read leaves the group check unrun, so a model is not written. */
    @Test
    fun `an imu model is not written while the bus is unread`() = runTest {
        val rig = SetupRig(this)
        rig.transport.unreadable += Fields.IMU_I2C.key()
        rig.transport.setAttachedBoard(board)
        rig.model.onShown()
        runCurrent()
        val model = SetupFields.IMU_MODEL.key
        rig.model.edit(model, Value.U8(Fields.ImuModel.CLONE_2E))
        flush()

        assertTrue(rig.transport.writes.isEmpty())
        assertEquals(SetupNotice.BusUnread(model), rig.state.notice)
    }
}
