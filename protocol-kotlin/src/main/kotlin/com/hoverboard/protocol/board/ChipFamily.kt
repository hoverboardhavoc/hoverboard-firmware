package com.hoverboard.protocol.board

/** The MCU family of a part, which decides which safe-USART wirings it can route ([allowlistFor]). */
enum class McuFamily { F10X, F1X0 }

/**
 * The capability answers of each fleet part: [validate]'s chip input, one [Capabilities] per part.
 *
 * **Where these come from, and why they can be trusted.** The firmware does not answer these
 * questions from a table like this one: it asks runtime-hal's descriptor-backed R-CAP queries
 * (`runtime_hal::pincap`), which this module cannot read, because they live in another repository
 * that the Kotlin drift gate deliberately does not check out. What it CAN read is
 * `crates/board/src/tests.rs`, `MockChip`: its tables are the same three parts' answers written
 * out, and its `rcap_agreement` module asserts that the mock and the real queries agree on
 * EVERY encodable pin, on the fleet gate maps and their mutations, on every ordered pin pair, and
 * on `validate`'s whole outcome, per part. This table is the Kotlin mirror of `MockChip`, pinned to
 * it by `RustSourceDriftTest`, so the chain is: Kotlin mirrors `MockChip`, `MockChip` equals the
 * real R-CAP answers.
 *
 * That is also the honest limit of the prediction. A part not listed here has no mirrored answers
 * at all, and the chip is not readable over the wire (`specs/rider-ui.md` (section 3.5), "The chip
 * family is NOT readable over the wire today"), so a client takes it from a preset or an explicit
 * selection and says which part it assumed.
 */
enum class ChipFamily(val label: String, val mcu: McuFamily) : Capabilities {
    /** The bench F103 master and the 6-FET split boards: GD32F103C8, LQFP48. */
    F103C8("GD32F103C8 (6-FET, 48-pin)", McuFamily.F10X),

    /** The bench F130 slave and the offroad pair: GD32F130C8, LQFP48. */
    F130C8("GD32F130C8 (6-FET, 48-pin)", McuFamily.F1X0),

    /** The 12-FET dual-motor mainboard: GD32F103RC, LQFP64, two advanced timers. */
    F103RC("GD32F103RC (12-FET, 64-pin)", McuFamily.F10X),
    ;

    /**
     * Does this pin exist on this part?
     *
     * The fleet's three parts differ in ways a layout can trip over: the F103C8 bonds PD0/PD1 and
     * no port F, the F130C8 bonds PF0/PF1 and PF6/PF7 and no port D, and the LQFP64 F103RC has all
     * of port C.
     */
    override fun pinExists(pin: Pin): Boolean {
        val n = pin.pin
        return when (this) {
            // F103C8 (LQFP48): PA0-15, PB0-15, PC13-15, PD0-1, no port F.
            F103C8 -> when (pin.port) {
                PORT_A, PORT_B -> true
                PORT_C -> n >= PORT_C_FIRST_48PIN
                PORT_D -> n <= 1
                else -> false
            }
            // F130C8 (LQFP48): PA0-15, PB0-15, PC13-15, PF0/PF1 + PF6/PF7, no port D.
            F130C8 -> when (pin.port) {
                PORT_A, PORT_B -> true
                PORT_C -> n >= PORT_C_FIRST_48PIN
                PORT_F -> n == 0 || n == 1 || n == 6 || n == 7
                else -> false
            }
            // F103RC (LQFP64): PA, PB, PC full, PD0-2, no port F.
            F103RC -> when (pin.port) {
                PORT_A, PORT_B, PORT_C -> true
                PORT_D -> n <= 2
                else -> false
            }
        }
    }

    /** Is this pin an advanced-timer channel here, and so refused to every non-gate field? */
    override fun gateCapable(pin: Pin): Boolean {
        val t0 = pin.packed in GATES_T0_HI || pin.packed in GATES_T0_LO
        return when (this) {
            F103C8, F130C8 -> t0
            F103RC -> t0 || pin.packed in GATES_T8_HI || pin.packed in GATES_T8_LO
        }
    }

    /** The advanced timer these six pins drive (0 = TIMER0, 1 = TIMER7/TIM8), or null. */
    override fun gateSet(hi: List<Pin>, lo: List<Pin>): Int? {
        fun matches(wantHi: List<Int>, wantLo: List<Int>) =
            hi.map { it.packed } == wantHi && lo.map { it.packed } == wantLo
        if (matches(GATES_T0_HI, GATES_T0_LO)) return TIMER0 // every fleet part
        if (this == F103RC && matches(GATES_T8_HI, GATES_T8_LO)) return TIMER7 // the 12-FET only
        return null
    }

    /**
     * The ADC channel behind [pin], if any: the standard F1-class analog map, PA0-7 = channels
     * 0-7, PB0-1 = 8-9, PC0-5 = 10-15 (port C analog only where those pins exist, which is the
     * LQFP64 part alone).
     */
    override fun adcChannel(pin: Pin): Int? = when {
        pin.port == PORT_A && pin.pin <= PORT_A_LAST_ANALOG -> pin.pin
        pin.port == PORT_B && pin.pin <= 1 -> PORT_B_FIRST_CHANNEL + pin.pin
        pin.port == PORT_C && pin.pin <= PORT_C_LAST_ANALOG && pinExists(pin) ->
            PORT_C_FIRST_CHANNEL + pin.pin
        else -> null
    }

    /**
     * The hardware-I2C instance this (SCL, SDA) pair forms, in the GD numbering. Identical on both
     * families, datasheet-verified: PB6/PB7 = I2C0 and PB10/PB11 = I2C1 on the F103 AND the F130,
     * so this answer does not read `this`.
     */
    override fun i2cPair(scl: Pin, sda: Pin): Int? = when {
        scl.packed == I2C0_SCL && sda.packed == I2C0_SDA -> 0
        scl.packed == I2C1_SCL && sda.packed == I2C1_SDA -> 1
        else -> null
    }

    companion object {
        /** The 6-FET gate map, both 48-pin families: TIMER0 hi PA8/PA9/PA10, lo PB13/PB14/PB15. */
        val GATES_T0_HI = listOf(0x08, 0x09, 0x0A)
        val GATES_T0_LO = listOf(0x1D, 0x1E, 0x1F)

        /** The 12-FET second motor (F103RC): TIMER7/TIM8 hi PC6/PC7/PC8, lo PA7/PB0/PB1. */
        val GATES_T8_HI = listOf(0x26, 0x27, 0x28)
        val GATES_T8_LO = listOf(0x07, 0x10, 0x11)

        /** The advanced-timer indices [gateSet] derives, as the validator's seam numbers them. */
        const val TIMER0 = 0
        const val TIMER7 = 1

        /** The I2C0 pair, PB6/PB7, and the I2C1 pair, PB10/PB11, packed. */
        const val I2C0_SCL = 0x16
        const val I2C0_SDA = 0x17
        const val I2C1_SCL = 0x1A
        const val I2C1_SDA = 0x1B

        private const val PORT_A = 0
        private const val PORT_B = 1
        private const val PORT_C = 2
        private const val PORT_D = 3
        private const val PORT_F = 5

        /** The lowest port-C pin the 48-pin parts bond (PC13). */
        private const val PORT_C_FIRST_48PIN = 13

        private const val PORT_A_LAST_ANALOG = 7
        private const val PORT_C_LAST_ANALOG = 5
        private const val PORT_B_FIRST_CHANNEL = 8
        private const val PORT_C_FIRST_CHANNEL = 10
    }
}
