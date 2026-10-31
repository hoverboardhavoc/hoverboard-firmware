package com.hoverboard.protocol.board

import com.hoverboard.protocol.linkctl.ChipTag

/**
 * The fleet's parts, as the layout editor names and picks pins on them: each part's label and the
 * pins it bonds ([pinExists]).
 *
 * Which part a board IS comes from the board itself: every `CYCLIC_STATE` carries the tag its boot
 * probe measured (`shared/linkctl/src/lib.rs`, `ChipTag`), and [forTag] turns that tag into the
 * part, or into nothing when the tag names no part this table holds. What a part can ROUTE to a
 * function is not here: the firmware asks the HAL at boot and reports what it could not bring up
 * in `board.status` ([BoardStatus]).
 */
enum class ChipFamily(val label: String) {
    /** The bench F103 master and the 6-FET split boards: GD32F103C8, LQFP48. */
    F103C8("GD32F103C8 (6-FET, 48-pin)"),

    /** The bench F130 slave and the offroad pair: GD32F130C8, LQFP48. */
    F130C8("GD32F130C8 (6-FET, 48-pin)"),

    /** The 12-FET dual-motor mainboard: GD32F103RC, LQFP64, two advanced timers. */
    F103RC("GD32F103RC (12-FET, 64-pin)"),
    ;

    /**
     * Does this pin exist on this part?
     *
     * The fleet's three parts differ in ways a layout can trip over: the F103C8 bonds PD0/PD1 and
     * no port F, the F130C8 bonds PF0/PF1 and PF6/PF7 and no port D, and the LQFP64 F103RC has all
     * of port C.
     */
    fun pinExists(pin: Pin): Boolean {
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

    companion object {
        /**
         * The part a board NAMES on the wire, or null for a tag this module models no part for
         * (`shared/linkctl/src/lib.rs`, `ChipTag`).
         *
         * [ChipTag.Unknown] is that null: the board's boot probe measured a family and an
         * advanced-timer count the firmware's own allocation has no fleet part for, so there is no
         * pin list to pick from. A client says the part is not known rather than offering the
         * nearest part's pins.
         */
        fun forTag(tag: ChipTag): ChipFamily? = when (tag) {
            ChipTag.F103C8 -> F103C8
            ChipTag.F130C8 -> F130C8
            ChipTag.F103RC -> F103RC
            ChipTag.Unknown -> null
        }

        private const val PORT_A = 0
        private const val PORT_B = 1
        private const val PORT_C = 2
        private const val PORT_D = 3
        private const val PORT_F = 5

        /** The lowest port-C pin the 48-pin parts bond (PC13). */
        private const val PORT_C_FIRST_48PIN = 13
    }
}
