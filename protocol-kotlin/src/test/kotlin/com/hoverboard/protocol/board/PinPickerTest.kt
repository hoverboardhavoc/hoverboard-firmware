package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Fields
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The picker's side of the rules: what a client may offer for one slot.
 *
 * Every case here is also a case the validator refuses, which is the point. The two are separate
 * implementations of the same rules, so these tests state the narrowing AND check it against
 * [validate]'s verdict on the same assignment: an offered pin must validate, and a withheld one
 * must be refused for the reason it was withheld.
 */
class PinPickerTest {

    private val chip = ChipFamily.F103C8
    private val reserved = reservedSet(chip, 0)

    private fun slot(field: BoardField, motor: Int? = null) =
        checkNotNull(Layout.forField(FieldRef(field, motor)))

    private val board = BoardFields(
        selfHold = 0x1C,
        vbatt = 0x04,
        buzzer = 0x19,
        ledGreen = 0x13,
        ledOrange = 0x0F,
        ledRed = 0x14,
        padA = 0x0B,
        padB = 0x2F,
    )

    private fun offered(field: BoardField, motor: Int? = null, fields: BoardFields = board) =
        candidatePins(slot(field, motor), fields, chip, reserved).map { it.packed }

    @Test
    fun aClaimedPinIsOfferedToItsOwnSlotAndToNoOther() {
        assertTrue(0x19 in offered(BoardField.BUZZER), "its own value, so the row can show it")
        assertFalse(0x19 in offered(BoardField.LED_GREEN), "the buzzer holds it")
        assertFalse(0x1C in offered(BoardField.LED_GREEN), "the power latch holds it")
    }

    @Test
    fun aReservedPinIsNeverOffered() {
        for (raw in reserved) assertFalse(raw in offered(BoardField.BUZZER), "pin ${raw.toString(16)}")
        // And the validator agrees about why.
        val err = validate(board.copy(buzzer = reserved.first()), chip, reserved).error
        assertEquals(BoardErrorKind.ReservedPin(checkNotNull(Pin.byName("PA13"))), err?.kind)
    }

    @Test
    fun aPinThisPartDoesNotBondIsNeverOffered() {
        assertFalse(0x50 in offered(BoardField.PAD_B), "PF0 is not bonded on the F103C8")
        assertTrue(0x50 in candidatePins(slot(BoardField.PAD_B), board, ChipFamily.F130C8, reservedSet(ChipFamily.F130C8, 0)).map { it.packed })
    }

    @Test
    fun aGateCapablePinIsOfferedOnlyToAGateSlot() {
        for (raw in ChipFamily.GATES_T0_HI + ChipFamily.GATES_T0_LO) {
            assertFalse(raw in offered(BoardField.LED_GREEN), "pin ${raw.toString(16)} drives a gate")
            assertFalse(raw in offered(BoardField.VBATT), "pin ${raw.toString(16)} drives a gate")
        }
        assertEquals(
            ChipFamily.GATES_T0_HI + ChipFamily.GATES_T0_LO,
            offered(BoardField.GATE_HI_A, 0).sorted(),
            "a gate slot offers the part's gate pins and nothing else",
        )
        // The validator's own answer on a withheld assignment.
        val err = validate(board.copy(ledGreen = ChipFamily.GATES_T0_HI[0]), chip, reserved).error
        assertEquals(
            BoardErrorKind.GateCapableMisused(checkNotNull(Pin.byName("PA8"))),
            err?.kind,
        )
    }

    @Test
    fun theBatterySenseAndThePhasePinsOfferAnalogPinsOnly() {
        for (field in listOf(BoardField.VBATT, BoardField.PHASE_A)) {
            val motor = if (field == BoardField.VBATT) null else 0
            val pins = offered(field, motor)
            assertTrue(pins.isNotEmpty())
            for (raw in pins) {
                assertTrue(
                    chip.adcChannel(checkNotNull(Pin.byName(Pin.parse(raw).let { (it as Parsed.Valid).pin.name }))) != null,
                    "$field offered ${raw.toString(16)}, which has no ADC channel",
                )
            }
            assertFalse(0x2D in pins, "PC13 exists but is not analog")
        }
        val err = validate(board.copy(vbatt = 0x2D), chip, reserved).error
        assertEquals(BoardErrorKind.NotAdcCapable(checkNotNull(Pin.byName("PC13"))), err?.kind)
    }

    @Test
    fun theImuSlotsOfferBusPinsAndNarrowToTheStagedHalf() {
        // A board with only its inter-board link configured: both I2C instances' pins are freed, so
        // both are reachable. Which instances a board can actually offer is a LINK_SET question, and
        // the bench mask below is the case where it has one.
        val bothFree = reservedSet(chip, 0b0010)
        fun imu(field: BoardField, fields: BoardFields, res: List<Int> = bothFree) =
            candidatePins(slot(field), fields, chip, res).map { it.packed }

        assertEquals(listOf(0x16, 0x1A), imu(BoardField.IMU_SCL, board), "the two instances' SCL pins")
        assertEquals(listOf(0x17, 0x1B), imu(BoardField.IMU_SDA, board), "and their SDA pins")
        // With one half staged, the other half offers only the pin that pairs with it: a mismatched
        // pair is a refusal, and this is what stops it being reachable.
        assertEquals(listOf(0x17), imu(BoardField.IMU_SDA, board.copy(imuScl = 0x16)))
        assertEquals(listOf(0x1A), imu(BoardField.IMU_SCL, board.copy(imuSda = 0x1B)), "PB10 is PB11's SCL")
        val err = validate(board.copy(imuScl = 0x16, imuSda = 0x1B, imuModel = 2), chip, bothFree).error
        assertEquals(BoardErrorKind.NotI2cPair, err?.kind, "the pair the picker refuses to offer")

        // The bench mask, where bit 2 keeps the USART2 wiring live: I2C1's pins are that port's, so
        // the only bus this board can be offered is I2C0, and the editor cannot stage the other.
        val bench = reservedSet(chip, 0b0110)
        assertEquals(listOf(0x16), imu(BoardField.IMU_SCL, board, bench))
        assertEquals(listOf(0x17), imu(BoardField.IMU_SDA, board, bench))
    }

    @Test
    fun aNonPinSlotOffersNoPins() {
        // The model, the dead time and the current-sense declaration are choices, not pins.
        for (key in listOf(Fields.IMU_MODEL.key(), Fields.MOTOR_DEAD_TIME.key(0), Fields.MOTOR_CURRENT_SENSE.key(0))) {
            val s = checkNotNull(Layout.forKey(key))
            assertFalse(s.isPin)
            assertEquals(emptyList<Pin>(), candidatePins(s, board, chip, reserved))
        }
    }

    @Test
    fun everyOfferedPinValidatesInTheSlotItWasOfferedFor() {
        // The property the whole picker exists for, over every pin slot of a blank-ish board: taking
        // an offer never produces a layout the board would refuse for THAT field.
        for (s in Layout.SLOTS.filter { it.isPin }) {
            for (pin in candidatePins(s, board, chip, reserved)) {
                val staged = s.on(board, pin.packed)
                val err = validate(staged, chip, reserved).error ?: continue
                // What a single offer CANNOT settle is a group: a function made of several pins is
                // all-or-none, so a group under construction is still refused, and that is the only
                // refusal an offer may produce. A per-pin one (reserved, claimed, incapable, absent
                // on this part) would mean the picker offered a pin it had no business offering.
                assertTrue(
                    err.kind in setOf(
                        BoardErrorKind.IncompleteGroup,
                        BoardErrorKind.MissingDeadTime,
                        BoardErrorKind.InvalidGateSet,
                    ),
                    "offering ${pin.name} for ${s.key} produced $err",
                )
            }
        }
    }
}
