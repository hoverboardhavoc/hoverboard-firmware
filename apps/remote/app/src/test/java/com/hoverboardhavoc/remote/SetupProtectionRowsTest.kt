package com.hoverboardhavoc.remote

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboardhavoc.remote.model.SetupField
import com.hoverboardhavoc.remote.model.SetupFields
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/**
 * The optional protections' Setup rows (`specs/optional-protections.md` section 7): each shown for
 * a motor the board drives whose layout has the protection's input. The peak trip needs none; the
 * rail skip needs a sensed pair, the DC-link limit `current.dc_link.pin`, the break switch
 * `gate.break.pin`.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class SetupProtectionRowsTest {

    private val board = 0x02

    /** A rig whose board stores [stored] before it attaches, with the Setup screen shown. */
    private fun TestScope.rig(stored: Map<Key, Value>): SetupRig = SetupRig(this).also {
        for ((k, v) in stored) it.transport.store[board to k] = v
        it.transport.setAttachedBoard(board)
        it.model.onShown()
        runCurrent()
    }

    private fun SetupRig.shown(): List<SetupField> = state.rows.filter { it in SetupFields.PROTECTIONS }

    @Test
    fun aBoardWithNoInputsShowsMotorZerosPeakTripAlone() = runTest {
        assertEquals(listOf(SetupFields.PEAK_TRIP[0]), rig(emptyMap()).shown())
    }

    @Test
    fun eachRowFollowsItsOwnInput() = runTest {
        val inputs = mapOf(
            Fields.MOTOR_PHASE_A.key(0) to Value.PinAdc(0, 0, 0),
            Fields.MOTOR_PHASE_B.key(0) to Value.PinAdc(2, 3, 13),
            Fields.MOTOR_DC_LINK_PIN.key(0) to Value.PinAdc(2, 1, 11),
        )
        assertEquals(
            listOf(SetupFields.PEAK_TRIP[0], SetupFields.RAIL_SKIP[0], SetupFields.DC_LINK_LIMIT[0]),
            rig(inputs).shown(),
        )
        val onePhase = mapOf(
            Fields.MOTOR_PHASE_A.key(0) to Value.PinAdc(0, 0, 0),
            Fields.MOTOR_BREAK_PIN.key(0) to Value.PinAf(1, 12, 0),
        )
        assertEquals(
            listOf(SetupFields.PEAK_TRIP[0], SetupFields.BREAK_ENABLE[0]),
            rig(onePhase).shown(),
            "one phase pin is no sensed pair",
        )
    }

    /** Motor 1's rows show on a board that drives it, and only for the inputs motor 1 stores. */
    @Test
    fun motorOnesRowsNeedMotorOne() = runTest {
        val stored = mapOf(
            SetupModel.MOTOR_ONE to Value.PinAf(2, 6, 0),
            Fields.MOTOR_BREAK_PIN.key(1) to Value.PinAf(0, 6, 0),
        )
        assertEquals(
            listOf(SetupFields.PEAK_TRIP[0], SetupFields.PEAK_TRIP[1], SetupFields.BREAK_ENABLE[1]),
            rig(stored).shown(),
        )
        val noMotorOne = mapOf(Fields.MOTOR_BREAK_PIN.key(1) to Value.PinAf(0, 6, 0))
        assertEquals(listOf(SetupFields.PEAK_TRIP[0]), rig(noMotorOne).shown(), "motor 1's input without motor 1")
    }
}
