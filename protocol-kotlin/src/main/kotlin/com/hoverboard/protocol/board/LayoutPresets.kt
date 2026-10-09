package com.hoverboard.protocol.board

import com.hoverboard.protocol.store.FieldDef
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value

/**
 * A known-good layout for one board variant (`specs/rider-ui.md`, section 3.5, "PRESETS").
 *
 * Applying one is the normal case, and the reason the editor is not thirty pin pickers: nobody
 * enters a board's wiring by hand. A preset is a WHOLE layout rather than a patch, because a layout
 * is one object: staging a handful of fields over whatever a board happened to hold before would
 * leave a pin from the previous staging in a field the preset does not mention.
 *
 * The exception is the three per-motor facts the validator carries without judging (direction, align
 * offset and counts per amp). Those are not layout facts a dump or a pin map can state: the drive
 * direction is a hall-and-phase WIRING fact, the align offset is bench-swept, and the calibration
 * belongs to the shunt chain fitted. [applyTo] therefore takes them from the board rather than from
 * the preset, which is the rule `specs/offroad-pinmap.md`, section 1.3, states for exactly these
 * fields.
 *
 * @param part the MCU this variant is, which the preset states so the verdict needs no guess.
 * @param linkSet the `LINK_SET` mask the variant's board carries. Not staged (the walk owns it) and
 *   not read by any client: it is here so the preset can be validated against the reserved set its
 *   own board computes, which is what makes "known-good" a checked claim rather than a label. It
 *   matters because the two families' BLE wirings are each other's I2C pins.
 * @param note what an operator needs to know before applying it, or null.
 */
data class LayoutPreset(
    val name: String,
    val part: ChipFamily,
    val linkSet: Int,
    val fields: BoardFields,
    val note: String? = null,
) {
    /**
     * This preset's layout for a board that currently holds [stored]: every field the validator
     * judges from the preset, and the carried per-motor facts left as the board has them.
     */
    fun applyTo(stored: BoardFields): BoardFields =
        Layout.SLOTS.filter { it.boardField == null }
            .fold(fields) { acc, slot -> slot.on(acc, slot.of(stored)) }
}

/**
 * The layouts of the boards this fleet actually has.
 *
 * Every value here is copied from the spec that derived it, and the test suite validates each preset
 * against its own part and `LINK_SET`, so a preset that could not boot cannot ship. What the presets
 * deliberately do NOT carry is anything a pin map cannot state: see [LayoutPreset.applyTo].
 */
object LayoutPresets {

    /** A registered pin field's default, which is the fleet-uniform pin for the benign functions. */
    private fun fleetPin(field: FieldDef): Int = (field.default as Value.U8).v

    /**
     * The benign fleet pins every mapped board shares. Taken from the registry rather than written
     * out again: those defaults exist precisely so a blank board behaves like the pre-layout
     * firmware, and a preset that disagreed with them would be a second opinion about one fact.
     */
    private val fleet = BoardFields(
        selfHold = fleetPin(Fields.BOARD_SELF_HOLD),
        buzzer = fleetPin(Fields.BOARD_BUZZER),
        ledGreen = fleetPin(Fields.LED_GREEN),
        ledOrange = fleetPin(Fields.LED_ORANGE),
        ledRed = fleetPin(Fields.LED_RED),
        padA = fleetPin(Fields.PAD_A),
        padB = fleetPin(Fields.PAD_B),
    )

    /**
     * The 6-FET split board's motor 0, the silicon-proven bench map (`specs/board-model.md`, section
     * 4, which carries it as preset data, with DTG 25 from
     * `specs/commutation.md`, the dead-time table).
     *
     * [phaseA] and [phaseB] differ between the two bench boards, which is why the phase pins are
     * board data and not a constant. Their rank ORDER is confirmed on silicon; which physical phase
     * each one senses is not (`specs/motor-integration.md`, the phase-identity rule), and this
     * carries the order the spec writes.
     */
    private fun splitMotor(phaseA: Int, phaseB: Int) = MotorFields(
        hallA = 0x2D, // PC13
        hallB = 0x01, // PA1
        hallC = 0x2E, // PC14
        gateHiA = 0x08, // PA8
        gateHiB = 0x09, // PA9
        gateHiC = 0x0A, // PA10
        gateLoA = 0x1D, // PB13
        gateLoB = 0x1E, // PB14
        gateLoC = 0x1F, // PB15
        deadTime = 25, // DTG 25, ~694 ns, silicon-proven on this board
        currentSense = 1,
        phaseA = phaseA,
        phaseB = phaseB,
    )

    /** The classywalk offroad board's motor 0 (`specs/offroad-pinmap.md`, section 1.1). */
    private val offroadMotor = MotorFields(
        hallA = 0x2D, // PC13
        hallB = 0x01, // PA1
        hallC = 0x2E, // PC14
        gateHiA = 0x08, // PA8
        gateHiB = 0x09, // PA9
        gateHiC = 0x0A, // PA10
        gateLoA = 0x1D, // PB13
        gateLoB = 0x1E, // PB14
        gateLoC = 0x1F, // PB15
        deadTime = 0x1C, // DTCFG 28, ~778 ns at 36 MHz fDTS
        currentSense = 1,
        phaseA = 0x00, // PA0 (ADC_IN0); the opposite order from the bench master's, per the dump
        phaseB = 0x10, // PB0 (ADC_IN8)
    )

    /** The battery sense pin, fleet-uniform at PA4 and staged on MASTERS only. */
    private const val VBATT_MASTER = 0x04

    /**
     * The bench 6-FET split pair. The IMU sits on the standard family's hardware I2C0 (PB6/PB7),
     * which the board's `LINK_SET` frees by carrying its BLE module on the USART2 wiring instead.
     * The fitted part answers `WHO_AM_I` 0x2E, so the model is the clone rather than an MPU-6050.
     */
    val BENCH_MASTER = LayoutPreset(
        name = "6-FET split master (bench)",
        part = ChipFamily.F103C8,
        linkSet = LINK_SET_STANDARD,
        fields = fleet.copy(
            vbatt = VBATT_MASTER,
            imuScl = 0x16, // PB6
            imuSda = 0x17, // PB7
            imuModel = Fields.ImuModel.CLONE_2E,
            motors = listOf(splitMotor(phaseA = 0x10, phaseB = 0x00), MotorFields.ABSENT), // PB0, PA0
        ),
        note = "Battery sense is staged: it reads the pack only on a master.",
    )

    /**
     * The bench slave: the same board wired as the mirror half, so no battery sense (a slave's PA4
     * sits at a fictitious 2.0 V and a slave staged to sense would divide its control by it) and the
     * other phase-current pair.
     */
    val BENCH_SLAVE = LayoutPreset(
        name = "6-FET split slave (bench)",
        part = ChipFamily.F130C8,
        linkSet = LINK_SET_STANDARD,
        fields = fleet.copy(
            imuScl = 0x16, // PB6
            imuSda = 0x17, // PB7
            imuModel = Fields.ImuModel.CLONE_2E,
            motors = listOf(splitMotor(phaseA = 0x10, phaseB = 0x11), MotorFields.ABSENT), // PB0, PB1
        ),
        note = "No battery sense: the slave reads the pack over the link, not from its own PA4.",
    )

    /**
     * The classywalk offroad master (`specs/offroad-pinmap.md`, section 1.1, derived from the
     * boards' own stock dumps). Its IMU is on PB10/PB11, which is I2C1 on this family and the
     * standard family's BLE USART, and its BLE module is on the PB6/PB7 wiring instead.
     */
    val OFFROAD_MASTER = LayoutPreset(
        name = "classywalk offroad master",
        part = ChipFamily.F130C8,
        linkSet = LINK_SET_OFFROAD,
        fields = fleet.copy(
            vbatt = VBATT_MASTER,
            button = 0x0C, // PA12, which is also the role strap
            imuScl = 0x1A, // PB10
            imuSda = 0x1B, // PB11
            imuModel = Fields.ImuModel.CLONE_2E,
            motors = listOf(offroadMotor, MotorFields.ABSENT),
        ),
        note = "The IMU model is the one silicon answered with, not the MPU-6050 the dump implied.",
    )

    /** The offroad slave: the identical pin map, without the battery sense. */
    val OFFROAD_SLAVE = LayoutPreset(
        name = "classywalk offroad slave",
        part = ChipFamily.F130C8,
        linkSet = LINK_SET_OFFROAD,
        fields = OFFROAD_MASTER.fields.copy(vbatt = PIN_ABSENT),
        note = "No battery sense: the slave reads the pack over the link, not from its own PA4.",
    )

    /** Every preset, in the order a picker offers them. */
    val ALL = listOf(BENCH_MASTER, BENCH_SLAVE, OFFROAD_MASTER, OFFROAD_SLAVE)

    /** The mask a standard-family board carries: the inter-board link plus the USART2 BLE wiring. */
    const val LINK_SET_STANDARD = 0b0110

    /** The offroad mask: the inter-board link plus the USART0 (PB6/PB7) BLE wiring. */
    const val LINK_SET_OFFROAD = 0b1010
}
