package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The Kotlin validator against the Rust validator's own vectors.
 *
 * `RustSourceDriftTest` pins the SHAPE of this mirror (the field set, the error kinds and their
 * `BOARD_OBS` codes, the capability tables, the reserved-set rule) against the Rust source. This
 * file pins the BEHAVIOUR, by running the vectors from `crates/board/src/tests.rs`: the same
 * inputs, asserting the same outcome, field and kind. Each case names the Rust test it came from, so a Rust-side
 * change to one of them is a change to a case here with a name to look up.
 *
 * Both halves are needed. A shape pin cannot catch a mirrored check that fires in the wrong ORDER
 * or names the wrong field, and a vector cannot catch a rule the Rust grew that neither side runs.
 */
class BoardLayoutTest {

    /**
     * The reserved set the firmware passes an unconfigured board: the safe-USART allowlist pins
     * plus SWD. The same vector as `RESERVED` in `crates/board/src/tests.rs`.
     */
    private val reserved = listOf(0x16, 0x17, 0x02, 0x03, 0x1A, 0x1B, 0x0D, 0x0E)

    /** The same vector as that file's `blank_board`: the benign fleet defaults, with vbatt on PA4. */
    private fun blankBoard() = BoardFields(
        selfHold = 0x1C, // PB12
        vbatt = 0x04, // PA4
        buzzer = 0x19, // PB9
        ledGreen = 0x13, // PB3
        ledOrange = 0x0F, // PA15
        ledRed = 0x14, // PB4
        padA = 0x0B, // PA11
        padB = 0x2F, // PC15
        button = PIN_ABSENT,
        imuScl = PIN_ABSENT,
        imuSda = PIN_ABSENT,
        imuModel = 0,
    )

    /** The bench 6-FET preset's motor-0 wiring, as that file's `bench_motor0`. */
    private fun benchMotor0() = MotorFields(
        hallA = 0x2D, // PC13
        hallB = 0x01, // PA1
        hallC = 0x2E, // PC14
        gateHiA = 0x08, // PA8
        gateHiB = 0x09, // PA9
        gateHiC = 0x0A, // PA10
        gateLoA = 0x1D, // PB13
        gateLoB = 0x1E, // PB14
        gateLoC = 0x1F, // PB15
        deadTime = 25,
        direction = 0,
        alignOffset = 3,
        currentSense = 1,
        currentCal = 455,
        phaseA = 0x10, // PB0, ADC channel 8
        phaseB = 0x00, // PA0, ADC channel 0
    )

    /**
 * The same wiring with no phase-current sense: the vector `bench_motor0_no_sense` in
 * `crates/board/src/tests.rs`. It is the vector for the parts where PB0/PA0 are not free to be
 * analog inputs.
 */
    private fun benchMotor0NoSense() =
        benchMotor0().copy(currentSense = 0, phaseA = PIN_ABSENT, phaseB = PIN_ABSENT)

    private fun BoardFields.withMotor(m: Int, mf: MotorFields) =
        copy(motors = motors.mapIndexed { i, old -> if (i == m) mf else old })

    private fun pin(raw: Int): Pin = (Pin.parse(raw) as Parsed.Valid).pin

    private fun plan(fields: BoardFields, chip: ChipFamily = ChipFamily.F103C8, res: List<Int> = reserved): BoardPlan =
        checkNotNull(validate(fields, chip, res).plan) { "expected a valid layout, got ${validate(fields, chip, res).error}" }

    private fun error(fields: BoardFields, chip: ChipFamily = ChipFamily.F103C8, res: List<Int> = reserved): BoardError =
        checkNotNull(validate(fields, chip, res).error) { "expected a refusal, got a valid layout" }

    // --- the encoding: `parse_accepts_the_encoding_and_rejects_the_rest` in
    // crates/board/src/tests.rs -----------------------------------------------------------------

    @Test
    fun parseAcceptsTheEncodingAndRejectsTheRest() {
        for ((port, byte) in listOf(0 to 0x00, 1 to 0x1F, 2 to 0x2D, 3 to 0x30, 5 to 0x5A)) {
            val p = (Pin.parse(byte) as Parsed.Valid).pin
            assertEquals(port, p.port, "port of ${byte.toString(16)}")
            assertEquals(byte and 0x0F, p.pin)
            assertEquals(byte, p.packed)
        }
        assertEquals(Parsed.Absent, Pin.parse(PIN_ABSENT))
        // Port E (4) is not in the encoding, and neither is anything above F.
        for (byte in listOf(0x40, 0x4C, 0x60, 0x7F, 0x90, 0xA1, 0xE0, 0xF0, 0xFE)) {
            assertEquals(Parsed.Invalid, Pin.parse(byte), "byte ${byte.toString(16)} must be invalid")
        }
    }

    @Test
    fun aPinSpellsItselfTheWayAPresetWritesIt() {
        // The client-side spelling the preset contract uses, and its inverse.
        assertEquals("PB12", pin(0x1C).name)
        assertEquals("PA0", pin(0x00).name)
        assertEquals("PF7", pin(0x57).name)
        assertEquals(pin(0x1C), Pin.byName("PB12"))
        assertEquals(pin(0x2E), Pin.byName("PC14"))
        assertNull(Pin.byName("PE0"), "port E is not in the encoding")
        assertNull(Pin.byName("PB16"), "16 is not a pin number")
        assertNull(Pin.byName("nonsense"))
    }

    // --- whole-set validation: `blank_board_validates_to_the_benign_plan` in
    // crates/board/src/tests.rs, and its neighbours ---------------------------------------------

    @Test
    fun blankBoardValidatesToTheBenignPlan() {
        val v = validate(blankBoard(), ChipFamily.F103C8, reserved)
        assertEquals(pin(0x1C), v.selfHold)
        val plan = checkNotNull(v.plan)
        assertEquals(pin(0x04), plan.vbatt?.pin)
        assertEquals(4, plan.vbatt?.channel, "PA4 = ADC channel 4, derived")
        assertEquals(pin(0x19), plan.buzzer)
        assertNull(plan.imu, "no IMU configured on a blank board")
        for (m in plan.motors) assertEquals(MotorPlan(), m, "motor groups absent on a blank board")
    }

    @Test
    fun allAbsentBoardIsAValidEmptyPlan() {
        assertEquals(BoardPlan(), plan(BoardFields()))
    }

    @Test
    fun benchPresetBoardValidatesFully() {
        // Half one: with the FULL allowlist reserved (an unconfigured board), the IMU on PB6/PB7 is
        // refused at the IMU field, because those pins are a live link port's until LINK_SET says
        // otherwise.
        val fields = blankBoard()
            .copy(imuScl = 0x16, imuSda = 0x17, imuModel = 2)
            .withMotor(0, benchMotor0())
        val err = error(fields)
        assertEquals(FieldRef(BoardField.IMU_SCL), err.field)
        assertEquals(BoardErrorKind.ReservedPin(pin(0x16)), err.kind)

        // Half two: with the caller having freed that port per LINK_SET, the same layout validates.
        val freed = reserved.filter { it != 0x16 && it != 0x17 }
        val plan = plan(fields, res = freed)
        assertEquals(0, plan.imu?.bus, "PB6/PB7 = I2C0")
        assertEquals(2, plan.imu?.model)
        assertEquals(0, plan.motors[0].gates?.timer, "TIMER0")
        assertEquals(25, plan.motors[0].gates?.deadTime)
        assertEquals(listOf(8, 0), plan.motors[0].phaseCurrent?.channels, "PB0 = ch8, PA0 = ch0")
    }

    @Test
    fun carriedMotorFactsFlowIntoThePlan() {
        // direction / align offset / counts per amp are carried into the plan unvalidated.
        var fields = blankBoard().withMotor(0, benchMotor0())
        var p = plan(fields)
        assertTrue(!p.motors[0].direction, "0 means Forward")
        assertEquals(3, p.motors[0].alignOffset)
        assertEquals(455, p.motors[0].currentCal)
        assertEquals(listOf(pin(0x10), pin(0x00)), p.motors[0].phaseCurrent?.pins)

        // Nonzero direction maps to Reverse, and the facts are carried raw, not range-checked.
        fields = fields.withMotor(
            0,
            benchMotor0().copy(
                direction = 1,
                alignOffset = 200,
                currentCal = 60_000,
                currentSense = 0,
                phaseA = PIN_ABSENT,
                phaseB = PIN_ABSENT,
            ),
        )
        p = plan(fields)
        assertTrue(p.motors[0].direction, "nonzero means Reverse")
        assertEquals(200, p.motors[0].alignOffset, "carried raw, unvalidated")
        assertEquals(60_000, p.motors[0].currentCal, "carried raw")
        assertNull(p.motors[0].phaseCurrent)
    }

    // --- the set-level checks -------------------------------------------------------------------

    @Test
    fun badEncodingNamesTheField() {
        val err = error(blankBoard().copy(buzzer = 0x40)) // port E
        assertEquals(FieldRef(BoardField.BUZZER), err.field)
        assertEquals(BoardErrorKind.BadEncoding(0x40), err.kind)
    }

    @Test
    fun partialHallGroupIsInvalidNotAbsent() {
        val err = error(blankBoard().withMotor(0, MotorFields(hallA = 0x2D, hallB = 0x01)))
        assertEquals(FieldRef(BoardField.HALL_C, 0), err.field, "the first absent member")
        assertEquals(BoardErrorKind.IncompleteGroup, err.kind)
    }

    @Test
    fun partialGateGroupIsInvalidNotAbsent() {
        val err = error(blankBoard().withMotor(0, benchMotor0().copy(gateLoB = PIN_ABSENT)))
        assertEquals(FieldRef(BoardField.GATE_LO_B, 0), err.field)
        assertEquals(BoardErrorKind.IncompleteGroup, err.kind)
    }

    /**
     * The same vectors as `configured_gate_group_requires_a_dead_time_at_the_floor` in
     * `crates/board/src/tests.rs`: the rule is a FLOOR, so 1..17 ticks are refused where they
     * used to validate, the floor itself and every value the fleet runs are taken, and 0 stays
     * legal with the gates unset.
     */
    @Test
    fun configuredGateGroupRequiresADeadTimeAtTheFloor() {
        for (dtg in listOf(0, 1, DEAD_TIME_MIN_DTG - 1)) {
            val err = error(blankBoard().withMotor(0, benchMotor0().copy(deadTime = dtg)))
            assertEquals(FieldRef(BoardField.DEAD_TIME, 0), err.field, "dtg $dtg")
            assertEquals(BoardErrorKind.DeadTimeBelowFloor, err.kind, "dtg $dtg")
        }
        for (dtg in listOf(DEAD_TIME_MIN_DTG, 25, 28, 32, 255)) {
            val gates = plan(blankBoard().withMotor(0, benchMotor0().copy(deadTime = dtg))).motors[0].gates
            assertEquals(dtg, gates?.deadTime, "dtg $dtg")
        }
        // Halls only: 0 is the right value for a board with no gates claimed.
        val halls = MotorFields(hallA = 0x2D, hallB = 0x01, hallC = 0x2E)
        assertEquals(0, halls.deadTime)
        assertNotNull(plan(blankBoard().withMotor(0, halls)).motors[0].halls)
    }

    @Test
    fun imuGroupIsAllOrNoneIncludingTheModel() {
        val freed = reserved.filter { it != 0x16 && it != 0x17 }
        // The pair with no model: names the model.
        var err = error(blankBoard().copy(imuScl = 0x16, imuSda = 0x17, imuModel = 0), res = freed)
        assertEquals(FieldRef(BoardField.IMU_MODEL), err.field)
        assertEquals(BoardErrorKind.IncompleteGroup, err.kind)
        // A model with no pins: names the first absent pin.
        err = error(blankBoard().copy(imuModel = 2))
        assertEquals(FieldRef(BoardField.IMU_SCL), err.field)
        // One pin only: names the other.
        err = error(blankBoard().copy(imuScl = 0x16, imuModel = 2), res = freed)
        assertEquals(FieldRef(BoardField.IMU_SDA), err.field)
    }

    @Test
    fun duplicatePinNamesTheSecondClaimant() {
        val err = error(blankBoard().copy(ledRed = 0x13)) // PB3, led_green's pin
        assertEquals(FieldRef(BoardField.LED_RED), err.field)
        assertEquals(BoardErrorKind.DuplicatePin(pin(0x13)), err.kind)
    }

    @Test
    fun reservedPinsRefuseEveryField() {
        // The property that makes a stolen link port impossible to express: every field, in turn,
        // on a reserved pin.
        for (raw in reserved) {
            val err = error(blankBoard().copy(buzzer = raw))
            assertEquals(FieldRef(BoardField.BUZZER), err.field, "pin ${raw.toString(16)}")
            assertEquals(BoardErrorKind.ReservedPin(pin(raw)), err.kind, "pin ${raw.toString(16)}")
        }
        // And a motor pin is no different from a singleton one.
        val err = error(blankBoard().withMotor(0, benchMotor0().copy(hallA = 0x02))) // PA2, the link
        assertEquals(FieldRef(BoardField.HALL_A, 0), err.field)
        assertEquals(BoardErrorKind.ReservedPin(pin(0x02)), err.kind)
    }

    @Test
    fun firstFailureWinsInFieldOrder() {
        // A bad encoding on an early field and a reserved hit on a later one: the early one wins.
        val err = error(blankBoard().copy(vbatt = 0x60, padA = 0x02))
        assertEquals(FieldRef(BoardField.VBATT), err.field)
        assertEquals(BoardErrorKind.BadEncoding(0x60), err.kind)
    }

    @Test
    fun capabilityStageRunsAfterTheSetLevelChecks() {
        // A capability failure on an EARLY field (vbatt on a non-ADC pin) against a set-level
        // failure on a LATE one (a duplicate pad). The set-level failure wins.
        val err = error(blankBoard().copy(vbatt = 0x2D, padB = 0x0B))
        assertEquals(FieldRef(BoardField.PAD_B), err.field)
        assertEquals(BoardErrorKind.DuplicatePin(pin(0x0B)), err.kind)
    }

    // --- the power latch ------------------------------------------------------------------------

    @Test
    fun aStagedNonDefaultLatchPinIsBothDrivenAndReserved() {
        val staged = 0x15 // PB5: not the fleet default, not reserved
        val fields = blankBoard().copy(selfHold = staged)
        assertEquals(pin(staged), validate(fields, ChipFamily.F103C8, reserved).selfHold)
        plan(fields)

        // Another field claiming the staged pin is refused, naming that other field.
        val err = error(fields.copy(buzzer = staged))
        assertEquals(FieldRef(BoardField.BUZZER), err.field)
        assertEquals(BoardErrorKind.DuplicatePin(pin(staged)), err.kind)

        // And PB12, which the pre-field firmware compiled in, is an ordinary pin on such a board.
        val movedOn = fields.copy(buzzer = 0x1C)
        val v = validate(movedOn, ChipFamily.F103C8, reserved)
        assertEquals(pin(staged), v.selfHold, "still the staged latch")
        assertEquals(pin(0x1C), v.plan?.buzzer)
    }

    @Test
    fun anAbsentLatchFieldDrivesNothingAndReservesNothing() {
        val v = validate(blankBoard().copy(selfHold = PIN_ABSENT, buzzer = 0x1C), ChipFamily.F103C8, reserved)
        assertNull(v.selfHold)
        assertEquals(pin(0x1C), v.plan?.buzzer)
    }

    @Test
    fun anUnresolvableLatchFieldDrivesNothingAndTheFailureNamesIt() {
        val cases = listOf<Pair<Int, BoardErrorKind>>(
            0x40 to BoardErrorKind.BadEncoding(0x40), // port E: not in the encoding
            0x5F to BoardErrorKind.UnknownPin(pin(0x5F)), // PF15: encodes, absent on this part
            0x0D to BoardErrorKind.ReservedPin(pin(0x0D)), // SWD
        )
        for ((staged, kind) in cases) {
            val v = validate(blankBoard().copy(selfHold = staged), ChipFamily.F103C8, reserved)
            assertNull(v.selfHold, "${staged.toString(16)}: nothing to drive")
            assertEquals(FieldRef(BoardField.SELF_HOLD), v.error?.field, staged.toString(16))
            assertEquals(kind, v.error?.kind, staged.toString(16))
        }
    }

    @Test
    fun aFailureElsewhereStillReportsTheLatch() {
        // The property the split exists for: a board mis-staged in an unrelated field still latches
        // its own rail, so it stays powered and reachable to be corrected.
        val v = validate(blankBoard().copy(ledRed = 0x13), ChipFamily.F103C8, reserved)
        assertEquals(pin(0x1C), v.selfHold, "the latch survives the failure")
        assertEquals(FieldRef(BoardField.LED_RED), v.error?.field)
    }

    @Test
    fun anUnstagedBoardLatchesTheFleetDefault() {
        assertEquals(pin(0x1C), validate(blankBoard(), ChipFamily.F103C8, reserved).selfHold)
        assertEquals(Value.U8(0x1C), Fields.BOARD_SELF_HOLD.default)
    }

    // --- the capability stage -------------------------------------------------------------------

    @Test
    fun benchPresetValidatesOnBothFamiliesItFits() {
        val fields = blankBoard()
            .copy(imuScl = 0x16, imuSda = 0x17, imuModel = 2)
            .withMotor(0, benchMotor0())
        val freed = reserved.filter { it != 0x16 && it != 0x17 }
        for (chip in listOf(ChipFamily.F103C8, ChipFamily.F130C8)) {
            val plan = plan(fields, chip, freed)
            assertEquals(0, plan.imu?.bus, "$chip: PB6/PB7 = I2C0 on both families")
            assertEquals(0, plan.motors[0].gates?.timer, "$chip: TIMER0 on both")
            assertEquals(4, plan.vbatt?.channel, "$chip: PA4 = channel 4 on both")
        }
    }

    @Test
    fun twelveFetMapValidatesOnlyWhereItsTimersExist() {
        val fields = blankBoard()
            .withMotor(0, benchMotor0NoSense())
            .withMotor(
                1,
                MotorFields(
                    hallA = 0x2A, hallB = 0x2B, hallC = 0x2C, // PC10/PC11/PC12
                    gateHiA = 0x26, gateHiB = 0x27, gateHiC = 0x28, // PC6/PC7/PC8
                    gateLoA = 0x07, gateLoB = 0x10, gateLoC = 0x11, // PA7/PB0/PB1
                    deadTime = 32,
                    direction = 1,
                    currentCal = 455,
                ),
            )
        // On the 12-FET part both motors validate, on distinct advanced timers.
        val plan = plan(fields, ChipFamily.F103RC)
        assertEquals(0, plan.motors[0].gates?.timer)
        assertEquals(1, plan.motors[1].gates?.timer, "TIM8, the 12-FET second motor")

        // On a 48-pin part the second motor's pins do not exist: refused at the first such field.
        val err = error(fields, ChipFamily.F103C8)
        assertEquals(FieldRef(BoardField.HALL_A, 1), err.field)
        assertEquals(BoardErrorKind.UnknownPin(pin(0x2A)), err.kind)
    }

    @Test
    fun wrongFamilyPinsAreUnknown() {
        // PF0 exists on the F130, not the F103.
        var fields = blankBoard().copy(padB = 0x50)
        var err = error(fields, ChipFamily.F103C8)
        assertEquals(FieldRef(BoardField.PAD_B), err.field)
        assertEquals(BoardErrorKind.UnknownPin(pin(0x50)), err.kind)
        plan(fields, ChipFamily.F130C8)

        // And PD0 exists on the F103, not the F130.
        fields = blankBoard().copy(padB = 0x30)
        err = error(fields, ChipFamily.F130C8)
        assertEquals(FieldRef(BoardField.PAD_B), err.field)
        assertEquals(BoardErrorKind.UnknownPin(pin(0x30)), err.kind)
        plan(fields, ChipFamily.F103C8)
    }

    @Test
    fun imuOnANonI2cPairIsRefused() {
        // A complete IMU group on existing, unreserved, non-I2C pins: the software bus is not built.
        val err = error(blankBoard().copy(imuScl = 0x05, imuSda = 0x06, imuModel = 1), ChipFamily.F130C8)
        assertEquals(FieldRef(BoardField.IMU_SCL), err.field)
        assertEquals(BoardErrorKind.NotI2cPair, err.kind)
    }

    @Test
    fun scrambledGateSetIsRefused() {
        // The right six pins in an electrically implausible assignment: a low-side pin in a
        // high-side slot and vice versa. Pins exist, no duplicates, but no timer has this shape.
        val fields = blankBoard().withMotor(0, benchMotor0().copy(gateHiC = 0x1D, gateLoA = 0x0A))
        val err = error(fields)
        assertEquals(FieldRef(BoardField.GATE_HI_A, 0), err.field)
        assertEquals(BoardErrorKind.InvalidGateSet, err.kind)
    }

    @Test
    fun gateCapablePinsRefuseNonGateFunctions() {
        // PA8 (TIMER0 CH0) in a LED field: refused even though the pin exists and is unreserved.
        var err = error(blankBoard().copy(ledGreen = 0x08))
        assertEquals(FieldRef(BoardField.LED_GREEN), err.field)
        assertEquals(BoardErrorKind.GateCapableMisused(pin(0x08)), err.kind)
        // And a low-side gate pin in a pad field.
        err = error(blankBoard().copy(padA = 0x1E))
        assertEquals(FieldRef(BoardField.PAD_A), err.field)
        assertEquals(BoardErrorKind.GateCapableMisused(pin(0x1E)), err.kind)
        // The same pins in their own gate slots are of course fine.
        plan(blankBoard().withMotor(0, benchMotor0()))
    }

    @Test
    fun vbattMustBeAdcCapable() {
        val err = error(blankBoard().copy(vbatt = 0x2D)) // PC13: exists, no ADC channel
        assertEquals(FieldRef(BoardField.VBATT), err.field)
        assertEquals(BoardErrorKind.NotAdcCapable(pin(0x2D)), err.kind)
    }

    // --- the phase-current group ----------------------------------------------------------------

    @Test
    fun phaseCurrentGroupIsAllOrNoneWithItsDeclaration() {
        // Declared but not wired: names the first absent pin.
        var mf = benchMotor0().copy(phaseA = PIN_ABSENT, phaseB = PIN_ABSENT)
        var err = error(blankBoard().withMotor(0, mf))
        assertEquals(FieldRef(BoardField.PHASE_A, 0), err.field)
        assertEquals(BoardErrorKind.IncompleteGroup, err.kind)

        // Half-wired: names the missing half.
        mf = mf.copy(phaseA = 0x10)
        err = error(blankBoard().withMotor(0, mf))
        assertEquals(FieldRef(BoardField.PHASE_B, 0), err.field)

        // Wired but NOT declared: names the declaration.
        err = error(blankBoard().withMotor(0, benchMotor0().copy(currentSense = 0)))
        assertEquals(FieldRef(BoardField.CURRENT_SENSE, 0), err.field)
        assertEquals(BoardErrorKind.IncompleteGroup, err.kind)

        // Neither: a motor with no current sense is a valid board state.
        val plan = plan(blankBoard().withMotor(0, benchMotor0NoSense()))
        assertNull(plan.motors[0].phaseCurrent)
        assertNotNull(plan.motors[0].gates, "the gate group still stands")
    }

    @Test
    fun phaseCurrentPinsMustBeAdcCapable() {
        // PB2: bonded, not gate-capable, no ADC channel behind it.
        val err = error(blankBoard().withMotor(0, benchMotor0().copy(phaseB = 0x12)))
        assertEquals(FieldRef(BoardField.PHASE_B, 0), err.field)
        assertEquals(BoardErrorKind.NotAdcCapable(pin(0x12)), err.kind)
    }

    @Test
    fun phaseCurrentPinsCollideWithOtherFieldsLikeAnyPin() {
        // vbatt on PA0, which the bench motor senses phase B on: the second claimant is named.
        val err = error(blankBoard().copy(vbatt = 0x00).withMotor(0, benchMotor0()))
        assertEquals(FieldRef(BoardField.PHASE_B, 0), err.field)
        assertEquals(BoardErrorKind.DuplicatePin(pin(0x00)), err.kind)
    }

    // --- the fleet's own boards, end to end: `one_image_two_boards` in
    // crates/board/src/tests.rs -----------------------------------------------------------------

    @Test
    fun theOffroadBoardGetsI2c1AndValidatesBeforeLinkSetIsWritten() {
        // The classywalk offroad board: IMU 0x68 clone on PB10/PB11, BLE on USART0's PB6/PB7.
        val fields = blankBoard().copy(imuScl = 0x1A, imuSda = 0x1B, imuModel = 2)
        // Configured (LINK_SET bit 1 = the inter-board link, bit 3 = BLE on USART0).
        var plan = plan(fields, ChipFamily.F130C8, reservedSet(ChipFamily.F130C8, 0b1010))
        assertEquals(1, plan.imu?.bus, "IMU on I2C1, the second instance")
        // And UNCONFIGURED, which is the staging trap worth knowing is benign: PB10/PB11 is not
        // reserved on this part, because no USART2 exists to claim it.
        plan = plan(fields, ChipFamily.F130C8, reservedSet(ChipFamily.F130C8, 0))
        assertEquals(1, plan.imu?.bus, "the IMU validates on an unconfigured offroad board")
    }

    @Test
    fun theBenchBoardTakesOneLayoutOnBothOfItsParts() {
        // The bench pair: IMU on I2C0's PB6/PB7, with LINK_SET bit 2 selecting the USART2 BLE
        // wiring and bit 3 CLEAR, which is what frees PB6/PB7 for the IMU.
        val fields = blankBoard().copy(imuScl = 0x16, imuSda = 0x17, imuModel = 2)
        for (chip in listOf(ChipFamily.F103C8, ChipFamily.F130C8)) {
            val plan = plan(fields, chip, reservedSet(chip, 0b0110))
            assertEquals(0, plan.imu?.bus, "$chip: IMU on I2C0")
        }
    }

    // --- the reserved set itself ----------------------------------------------------------------

    @Test
    fun theReservedSetFreesOnlyLinkSetClearedPorts() {
        // Unconfigured: every routable port is a claimant, so the whole allowlist is reserved.
        val f103 = reservedSet(ChipFamily.F103C8, 0)
        assertTrue(f103.containsAll(SWD_PINS), "SWD is always reserved")
        assertTrue(f103.containsAll(listOf(0x02, 0x03)), "the inter-board link")
        assertTrue(f103.containsAll(listOf(0x1A, 0x1B)), "USART2's pins, which this family routes")
        assertTrue(0x16 !in f103, "PB6 reaches no USART on the F10x, so nothing claims it")

        // Configured with bit 3 clear: the PB6/PB7 wiring is freed for the IMU.
        val configured = reservedSet(ChipFamily.F130C8, 0b0110)
        assertTrue(0x16 !in configured && 0x17 !in configured, "the freed port")
        assertTrue(configured.containsAll(listOf(0x02, 0x03)), "the live inter-board link stays held")
    }

    @Test
    fun theReservedSetIgnoresPortsThisChipCannotRoute() {
        // The F1x0 has no USART2 at all, so holding PB10/PB11 for it would block the offroad IMU
        // from the bus it is physically wired to, for a probe that cannot happen.
        val f130 = reservedSet(ChipFamily.F130C8, 0)
        assertTrue(0x1A !in f130 && 0x1B !in f130, "no USART2 on this family")
        assertTrue(f130.containsAll(listOf(0x16, 0x17)), "USART0's pins, which this family routes")
    }

    @Test
    fun exactlyOneBleWiringIsRoutablePerPart() {
        // What makes the two BLE wirings one slot rather than an ambiguity.
        for (chip in ChipFamily.entries) {
            val ble = allowlistFor(chip).filter { it.netPort == NET_PORT_BLE && it.routable }
            assertEquals(1, ble.size, "$chip: expected exactly one routable BLE wiring, got $ble")
            val expected = if (chip.mcu == McuFamily.F1X0) listOf(0x16, 0x17) else listOf(0x1A, 0x1B)
            assertEquals(expected, ble.single().pins, "$chip: the wiring it is built with")
            assertTrue(
                allowlistFor(chip).single { it.netPort == NET_PORT_UART }.routable,
                "$chip: the inter-board link must route",
            )
        }
    }

    // --- the layout as one object ---------------------------------------------------------------

    @Test
    fun aLayoutReadsBackAsTheFieldsItWasBuiltFrom() {
        val fields = blankBoard()
            .copy(imuScl = 0x16, imuSda = 0x17, imuModel = 2)
            .withMotor(0, benchMotor0())
        val stored = Layout.SLOTS.associate { it.key to it.value(it.of(fields)) }
        assertEquals(emptyList<LayoutSlot>(), Layout.missing(stored))
        assertEquals(fields, Layout.fieldsFrom(stored))
    }

    @Test
    fun aPartialReadIsNoLayoutAtAll() {
        val fields = blankBoard()
        val stored = Layout.SLOTS.associate { it.key to it.value(it.of(fields)) } - Fields.PAD_B.key()
        assertNull(Layout.fieldsFrom(stored), "a layout is one object: a gap is not a layout")
        assertEquals(listOf(Fields.PAD_B.key()), Layout.missing(stored).map { it.key })
    }

    @Test
    fun aWrongTypeReadsAsMissingRatherThanAsAValue() {
        val stored = Layout.SLOTS.associate { it.key to Value.I32(0) }
        assertNull(Layout.fieldsFrom(stored))
        assertEquals(Layout.SLOTS.size, Layout.missing(stored).size)
    }

    @Test
    fun theDeltaIsTheWritesAndNothingElse() {
        val from = blankBoard()
        val to = from.copy(buzzer = 0x15).withMotor(1, MotorFields(deadTime = 32))
        assertEquals(
            listOf(Fields.BOARD_BUZZER.key(), Fields.MOTOR_DEAD_TIME.key(1)),
            Layout.writes(from, to).map { it.first },
            "one write per changed slot, in slot order",
        )
        assertEquals(listOf(Value.U8(0x15), Value.U8(32)), Layout.writes(from, to).map { it.second })
        assertEquals(emptyList<Pair<*, *>>(), Layout.writes(from, from), "nothing to write for no change")
    }

    @Test
    fun everySlotMapsBackToTheFieldTheValidatorNames() {
        // The verdict is rendered per field, so every field a refusal can name has to resolve to
        // the slot a client would correct, and the two IMU frame fields deliberately have none.
        val named = BoardField.entries - BoardField.IMU_AXIS_SIGN - BoardField.IMU_AXIS_ROLE
        for (field in named) {
            val motors = if (field.perMotor) (0 until BoardFields.MOTORS).toList() else listOf(null)
            for (m in motors) {
                val slot = checkNotNull(Layout.forField(FieldRef(field, m))) { "$field motor $m has no slot" }
                assertEquals(field.def.key(m ?: 0), slot.key)
            }
        }
        assertNull(Layout.forField(FieldRef(BoardField.IMU_AXIS_SIGN)), "the frame is not a layout field")
    }

    @Test
    fun aRefusalCarriesTheObsCodeTheBoardWouldReport() {
        // The verdict a client renders is the record the firmware writes, by code and detail.
        val err = error(blankBoard().copy(vbatt = 0x2D))
        assertEquals(9, err.kind.obsResult, "NotAdcCapable")
        assertEquals(0x2D, err.kind.obsDetail)
        assertEquals(Fields.BOARD_VBATT.id, err.field.field.id)
    }
}
