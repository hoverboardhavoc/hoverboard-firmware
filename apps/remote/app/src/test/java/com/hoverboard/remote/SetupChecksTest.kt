package com.hoverboard.remote

import com.hoverboard.protocol.checks.Check
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
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
}
