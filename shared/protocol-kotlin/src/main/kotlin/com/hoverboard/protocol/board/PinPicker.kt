package com.hoverboard.protocol.board

import com.hoverboard.protocol.checks.Checks
import com.hoverboard.protocol.checks.Role
import com.hoverboard.protocol.pincap.Chip
import com.hoverboard.protocol.pincap.SWD
import com.hoverboard.protocol.store.Value

/**
 * The values a client may offer for one layout slot (`specs/remote-ui.md` section 3.5, "THE PIN
 * GRID"), taken from [chip]'s table ([com.hoverboard.protocol.pincap]): every pin the chip bonds,
 * less the SWD pair and the pins another function holds, that can carry the slot's function. A
 * plain pin is any such pin; an ADC input carries its pin's channel; an alternate-function pin
 * carries the AF number of each route the table has for its function (0 on a chip that remaps,
 * whose group's remap key selects the route instead).
 *
 * The offer narrows what can be expressed; it does not judge the layout as a whole. Which
 * instance a group uses, and whether the group is complete, are the checks' to answer before the
 * write ([Checks]). A slot that holds no pin, or no pin of a group, offers nothing.
 */
fun candidates(slot: LayoutSlot, fields: BoardFields, chip: Chip): List<Value> {
    val role = Checks.roleOf(slot.key) ?: return emptyList()
    val t = chip.table
    val taken = Checks.GROUPS.flatMap { g -> g.pins.map { it.first.key } }
        .filter { it != slot.key }
        .mapNotNull { Checks.pinAt(fields.values, it) }
        .toSet()
    val free = (0 until PORTS).flatMap { port -> (0 until PINS).map { port to it } }
        .filter { (port, pin) -> t.bonded(port, pin) && (port to pin) !in SWD && (port to pin) !in taken }
    return free.flatMap { (port, pin) ->
        when (role) {
            Role.Plain -> listOf(Value.Pin(port, pin))
            Role.Adc -> listOfNotNull(t.adcChannel(port, pin)?.let { Value.PinAdc(port, pin, it) })
            is Role.Af -> t.routes
                .filter { it.port == port && it.pin == pin && it.func.kind == role.kind && it.func.ch == role.ch }
                .map { Value.PinAf(port, pin, if (chip.remaps) 0 else it.sel) }
                .distinct()
        }
    }
}

/** The ports a pin table describes, `A` to `F`. */
private const val PORTS = 6

/** The pins of one port. */
private const val PINS = 16
