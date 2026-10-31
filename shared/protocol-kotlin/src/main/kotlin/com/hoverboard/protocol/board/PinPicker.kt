package com.hoverboard.protocol.board

/**
 * Which pins a client may offer for one layout slot: the pins that are free and capability-eligible
 * for that function, so an invalid layout is hard to EXPRESS rather than merely refused
 * (`specs/rider-ui.md`, section 3.5, "THE PIN GRID").
 *
 * This is deliberately a SECOND implementation of rules [validate] owns, not a shortcut past it. The
 * picker narrows what a user can reach; the validator still judges the result, because the two can
 * disagree (a picker cannot see a whole gate set while a single pin is being chosen) and only one of
 * them is the board's own answer. A client runs both.
 */

/**
 * The pins [slot] may be assigned on [chip], given the layout [fields] as it stands and the
 * [reserved] set, in packed order. [PIN_ABSENT] is not among them: unsetting a function is always
 * available and is not a pin.
 *
 * A pin is offered when all of these hold:
 *
 * - it exists on this part, and is not in the reserved set (the live link ports and SWD);
 * - no OTHER slot in [fields] already claims it (its own current value is offered, so a row shows
 *   what it holds);
 * - it satisfies the slot's capability. A gate slot lists gate-capable pins and every other slot
 *   lists only pins that are NOT gate-capable, which is the denylist rule from the same side; the
 *   battery sense and the two phase-current slots list ADC-capable pins; and an IMU slot lists the
 *   pins that form a hardware-I2C instance, narrowed to the pin that pairs with the other half once
 *   that half is staged.
 *
 * Returns an empty list for a slot that holds no pin ([LayoutSlot.isPin] false), which has choices
 * of its own rather than pins.
 */
fun candidatePins(slot: LayoutSlot, fields: BoardFields, chip: ChipFamily, reserved: List<Int>): List<Pin> {
    if (!slot.isPin) return emptyList()
    val taken = Layout.SLOTS
        .filter { it.isPin && it.key != slot.key }
        .mapNotNull { (Pin.parse(it.of(fields)) as? Parsed.Valid)?.pin?.packed }
        .toSet()
    return allPins()
        .filter { chip.pinExists(it) && it.packed !in reserved && it.packed !in taken }
        .filter { eligible(slot, it, fields, chip) }
}

/** Every pin the encoding defines, in packed order (the domain a picker chooses from). */
private fun allPins(): List<Pin> =
    (0 until PIN_ABSENT).mapNotNull { (Pin.parse(it) as? Parsed.Valid)?.pin }

private fun eligible(slot: LayoutSlot, pin: Pin, fields: BoardFields, chip: ChipFamily): Boolean {
    val field = slot.boardField ?: return false
    if (field.isGate) return chip.gateCapable(pin)
    // The denylist rule, from the picker's side: a gate-capable pin is never offered elsewhere.
    if (chip.gateCapable(pin)) return false
    return when (field) {
        BoardField.VBATT, BoardField.PHASE_A, BoardField.PHASE_B -> chip.adcChannel(pin) != null
        BoardField.IMU_SCL -> pairsWith(chip, pin, fields.imuSda, sclSide = true)
        BoardField.IMU_SDA -> pairsWith(chip, pin, fields.imuScl, sclSide = false)
        else -> true
    }
}

/**
 * Whether [pin] can take its side of the IMU bus: it pairs with the staged other half if there is
 * one, and otherwise with any pin at all (no pair at all means the unbuilt software-I2C bus, which
 * the validator refuses).
 */
private fun pairsWith(chip: ChipFamily, pin: Pin, otherRaw: Int, sclSide: Boolean): Boolean {
    val other = (Pin.parse(otherRaw) as? Parsed.Valid)?.pin
    val partners = if (other != null) listOf(other) else allPins()
    return partners.any { p -> if (sclSide) chip.i2cPair(pin, p) != null else chip.i2cPair(p, pin) != null }
}
