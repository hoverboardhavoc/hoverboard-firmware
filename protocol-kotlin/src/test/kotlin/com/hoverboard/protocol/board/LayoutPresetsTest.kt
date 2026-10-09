package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Fields
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The shipped presets, against the validator.
 *
 * "Known-good" has to be a checked claim rather than a label, and it is checkable: each preset
 * declares the part it is for and the `LINK_SET` its board carries, which is everything the pure
 * validator needs. A preset that could not boot therefore cannot ship.
 */
class LayoutPresetsTest {

    private fun verdict(preset: LayoutPreset) =
        validate(preset.fields, preset.part, reservedSet(preset.part, preset.linkSet))

    @Test
    fun everyPresetValidatesOnItsOwnPartAndLinkSet() {
        for (preset in LayoutPresets.ALL) {
            val v = verdict(preset)
            assertNull(v.error, "${preset.name}: ${v.error}")
            assertNotNull(v.selfHold, "${preset.name} latches its own rail")
        }
    }

    @Test
    fun everyPresetDescribesAWholeMotorAndAnImu() {
        // The point of a preset is that it needs no pin-level editing afterwards, so each one
        // carries the groups that make a board drive and balance, not a fragment of them.
        for (preset in LayoutPresets.ALL) {
            val plan = checkNotNull(verdict(preset).plan)
            assertNotNull(plan.motors[0].halls, "${preset.name} halls")
            assertNotNull(plan.motors[0].gates, "${preset.name} gates")
            assertNotNull(plan.motors[0].phaseCurrent, "${preset.name} phase current")
            assertNotNull(plan.imu, "${preset.name} IMU")
            assertEquals(MotorPlan(), plan.motors[1], "${preset.name} has one motor")
        }
    }

    @Test
    fun theBenchPairSensesOnTheBusItsLinkSetFrees() {
        // The standard family's IMU is on I2C0's PB6/PB7, which are also the offroad family's BLE
        // USART: it is the board's own LINK_SET that decides whether those pins are the link's or
        // the IMU's, and this is the staging where they are the IMU's.
        for (preset in listOf(LayoutPresets.BENCH_MASTER, LayoutPresets.BENCH_SLAVE)) {
            val imu = checkNotNull(verdict(preset).plan?.imu)
            assertEquals(0, imu.bus, "${preset.name} I2C0")
            assertEquals(Fields.ImuModel.CLONE_2E, imu.model)
        }
        // And the same pins staged on an offroad board, whose BLE module is on them, are refused.
        val wrongBoard = LayoutPresets.BENCH_SLAVE
        val error = validate(
            wrongBoard.fields,
            wrongBoard.part,
            reservedSet(wrongBoard.part, LayoutPresets.LINK_SET_OFFROAD),
        ).error
        assertEquals(
            BoardErrorKind.ReservedPin(checkNotNull(Pin.byName("PB6"))),
            error?.kind,
            "a preset applied to the wrong board is refused, not silently wrong",
        )
    }

    @Test
    fun theOffroadPairSensesOnI2c1() {
        for (preset in listOf(LayoutPresets.OFFROAD_MASTER, LayoutPresets.OFFROAD_SLAVE)) {
            assertEquals(1, verdict(preset).plan?.imu?.bus, "${preset.name} I2C1")
        }
    }

    @Test
    fun onlyTheMasterPresetsSenseTheBattery() {
        // A slave's PA4 reads a fiction, so staging the pin on one would make it divide its control
        // by a pack voltage that is not there.
        for (preset in listOf(LayoutPresets.BENCH_MASTER, LayoutPresets.OFFROAD_MASTER)) {
            val vbatt = checkNotNull(verdict(preset).plan?.vbatt) { "${preset.name} senses" }
            assertEquals("PA4", vbatt.pin.name, "${preset.name} PA4")
            assertEquals(4, vbatt.channel, "${preset.name} channel 4")
        }
        for (preset in listOf(LayoutPresets.BENCH_SLAVE, LayoutPresets.OFFROAD_SLAVE)) {
            assertNull(verdict(preset).plan?.vbatt, "${preset.name} does not sense")
        }
    }

    @Test
    fun thePhasePinsAreBoardDataAndNotOneConstant() {
        // The three variants' phase-current pairs differ, which is the whole reason these are fields
        // rather than a compiled constant.
        fun pins(preset: LayoutPreset) =
            checkNotNull(verdict(preset).plan?.motors?.get(0)?.phaseCurrent).pins.map { it.name }
        assertEquals(listOf("PB0", "PA0"), pins(LayoutPresets.BENCH_MASTER))
        assertEquals(listOf("PB0", "PB1"), pins(LayoutPresets.BENCH_SLAVE))
        assertEquals(listOf("PA0", "PB0"), pins(LayoutPresets.OFFROAD_MASTER))
    }

    @Test
    fun applyingAPresetKeepsTheFactsAPinMapCannotState() {
        // Drive direction, align offset and counts per amp are not layout facts: the first is a
        // wiring fact, the second is bench-swept and the third belongs to the shunt chain fitted. A
        // preset leaves all three as the board holds them.
        val board = LayoutPresets.BENCH_MASTER.fields.let { f ->
            f.copy(
                motors = f.motors.mapIndexed { i, m ->
                    if (i == 0) m.copy(direction = 1, alignOffset = 3, currentCal = 400) else m
                },
            )
        }
        val applied = LayoutPresets.BENCH_SLAVE.applyTo(board)

        assertEquals(1, applied.motors[0].direction)
        assertEquals(3, applied.motors[0].alignOffset)
        assertEquals(400, applied.motors[0].currentCal)
        // Everything the validator judges comes from the preset.
        assertEquals(LayoutPresets.BENCH_SLAVE.fields.motors[0].phaseB, applied.motors[0].phaseB)
        assertEquals(PIN_ABSENT, applied.vbatt, "the slave preset does not sense")
        // And the delta from the applied layout to the preset's own is only those carried facts.
        val carried = Layout.delta(applied, LayoutPresets.BENCH_SLAVE.fields)
        assertTrue(carried.all { it.boardField == null }, "only carried facts differ: $carried")
    }

    @Test
    fun noPresetClaimsAFactAPinMapCannotState() {
        // The three carried per-motor facts are left at zero in the data itself, not just ignored by
        // [LayoutPreset.applyTo]: a number sitting in a preset reads as a claim about the board, and
        // these are not claims anyone derived.
        val carried = Layout.SLOTS.filter { it.boardField == null }
        for (preset in LayoutPresets.ALL) {
            for (slot in carried) {
                assertEquals(0, slot.of(preset.fields), "${preset.name} ${slot.key}")
            }
        }
    }

    @Test
    fun aPresetIsAWholeLayoutRatherThanAPatch() {
        // Applying one over a board staged differently leaves nothing of the old staging behind in a
        // field the preset does not mention: that is what stops a stale pin surviving into a layout
        // nobody meant to describe.
        val oddBoard = LayoutPresets.OFFROAD_MASTER.fields.copy(buzzer = 0x15, padA = 0x12)
        val applied = LayoutPresets.BENCH_MASTER.applyTo(oddBoard)

        assertEquals(LayoutPresets.BENCH_MASTER.fields.buzzer, applied.buzzer)
        assertEquals(LayoutPresets.BENCH_MASTER.fields.padA, applied.padA)
        assertNull(validate(applied, ChipFamily.F103C8, reservedSet(ChipFamily.F103C8, LayoutPresets.LINK_SET_STANDARD)).error)
    }
}
