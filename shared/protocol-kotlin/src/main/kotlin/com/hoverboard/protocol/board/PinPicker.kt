package com.hoverboard.protocol.board

/**
 * Which pins a client may offer for one layout slot (`specs/remote-ui.md`, section 3.5, "THE PIN
 * GRID"): the pins [chip] bonds, less the SWD pair and the pins another slot already holds.
 *
 * It does not ask what the chip can route to the slot's function (an ADC channel, an I2C pair, a
 * gate timer). The firmware asks the HAL that at boot, per board, and a pin it cannot use is
 * listed in the board's status after the reboot ([BoardStatus]); a table here would be a second,
 * unpinned answer to the same question.
 *
 * Returns an empty list for a slot that holds no pin ([LayoutSlot.isPin] false), which has choices
 * of its own rather than pins. [PIN_ABSENT] is not among the offers: unsetting a function is always
 * available and is not a pin. A slot's own current value is offered, so a row shows what it holds.
 */
fun candidatePins(slot: LayoutSlot, fields: BoardFields, chip: ChipFamily): List<Pin> {
    if (!slot.isPin) return emptyList()
    val taken = Layout.SLOTS
        .filter { it.isPin && it.key != slot.key }
        .mapNotNull { (Pin.parse(it.of(fields)) as? Parsed.Valid)?.pin?.packed }
        .toSet()
    return (0 until PIN_ABSENT)
        .mapNotNull { (Pin.parse(it) as? Parsed.Valid)?.pin }
        .filter { chip.pinExists(it) && it.packed !in SWD_PINS && it.packed !in taken }
}
