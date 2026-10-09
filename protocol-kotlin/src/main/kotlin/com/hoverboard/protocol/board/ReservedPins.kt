package com.hoverboard.protocol.board

/**
 * The reserved pin set [validate] refuses every layout field (`specs/board-model.md`, check 3): the
 * safe-USART allowlist pins the link layer may claim this boot, plus SWD.
 *
 * The link's own pins are deliberately NOT layout fields. The safe-USART allowlist is a compiled
 * chip-safety fact (`specs/l3.md`, "Pin safety": it is what lets an UNCONFIGURED board probe for a
 * controller before any field can exist), so making it a field would put the safety boundary inside
 * the thing it guards. The reserved set is how a layout is kept off those pins, and mirroring it
 * here is what lets a client refuse a layout that would steal a link port BEFORE writing it.
 */

/** One safe-USART allowlist entry (`board::plumbing`, `AllowlistPort`; the firmware owns the table). */
data class AllowlistPort(
    /** This wiring's bit in the persisted `LINK_SET` mask, unique per entry. */
    val linkSetBit: Int,
    /**
     * The `net` port slot this entry's link occupies when live. NOT unique: the two BLE wirings are
     * alternatives for one board function and share the BLE slot.
     */
    val netPort: Int,
    /** The entry's two pins, packed. */
    val pins: List<Int>,
    /**
     * Can this part route the entry at all? The HAL's per-boot answer in the firmware; here, the
     * family fact behind it ([allowlistFor]). A pair this silicon cannot route claims nothing, so
     * reserving its pins would reserve them against a phantom.
     */
    val routable: Boolean,
)

/** The SWD pins (PA13/PA14), always reserved on every fleet part (`board::plumbing`, `SWD_PINS`). */
val SWD_PINS = listOf(0x0D, 0x0E)

/**
 * The compiled allowlist as the firmware declares it (`crates/firmware/src/main.rs`,
 * `SAFE_LINK_USARTS`): the two BLE wirings and the inter-board link, pins and `LINK_SET` bits only.
 *
 * The first and last entries are the fleet's TWO BLE wirings. The onboard CC2541 hangs off USART2's
 * PB10/PB11 on the standard family and off USART0's PB6/PB7 on the offroad family, and the mirror
 * image is true of the IMU: the pins one board uses for its module are the pins the other uses for
 * its I2C bus. So they are two entries with distinct `LINK_SET` bits and one shared `net` slot, and
 * which one a board brings up is decided by what was staged on it, never by the chip.
 */
private val SAFE_LINK_USARTS = listOf(
    // PB6/PB7 = USART0 on the F1x0: the offroad family's BLE module, and the standard family's IMU bus.
    Triple(3, NET_PORT_BLE, listOf(0x16, 0x17)),
    // PA2/PA3 = USART1: the inter-board link, both boards, both families.
    Triple(NET_PORT_UART, NET_PORT_UART, listOf(0x02, 0x03)),
    // PB10/PB11 = USART2 on the F10x: the standard family's onboard CC2541.
    Triple(NET_PORT_BLE, NET_PORT_BLE, listOf(0x1A, 0x1B)),
)

/** The `net` slot carrying the inter-board link (`PORT_IDX_UART`). */
const val NET_PORT_UART = 1

/** The `net` slot carrying the BLE module (`PORT_IDX_BLE`), whichever wiring reaches it. */
const val NET_PORT_BLE = 2

/**
 * The allowlist as [chip] can route it.
 *
 * Which entries are routable is a per-family fact in the firmware too, but asked of the pin model
 * rather than written down: PA2/PA3 is USART1's default mapping on both families, PB10/PB11 reaches
 * USART2 only on the F10x, and PB6/PB7 reaches USART0 only on the F1x0 (on the F10x it reaches no
 * USART without an AFIO remap the HAL does not implement). `crates/board/src/tests.rs`, `port_routability_agreement`,
 * asserts against runtime-hal that exactly one BLE wiring
 * routes per fleet part and that it is the one each family is built with, which is the fact this
 * mirrors.
 */
fun allowlistFor(chip: ChipFamily): List<AllowlistPort> = SAFE_LINK_USARTS.map { (bit, slot, pins) ->
    val routable = when (slot) {
        NET_PORT_UART -> true
        else -> when (chip.mcu) {
            McuFamily.F10X -> pins == listOf(0x1A, 0x1B) // USART2's pins
            McuFamily.F1X0 -> pins == listOf(0x16, 0x17) // USART0's pins
        }
    }
    AllowlistPort(linkSetBit = bit, netPort = slot, pins = pins, routable = routable)
}

/**
 * Is this allowlist port one the link layer may claim its pins for, given [linkSet]
 * (`board::plumbing`, `claims_pins`)?
 *
 * - `linkSet == 0` means UNCONFIGURED: the board will probe every allowlisted port, so every one is
 *   a claimant. A nonzero mask means configured, and only the ports whose bit IS set are live link
 *   ports; the clear-bit ports are freed for other functions, which is how the standard family's
 *   IMU gets PB6/PB7.
 * - A port this silicon cannot route will never be configured whatever the mask says, so it claims
 *   nothing.
 */
fun claimsPins(port: AllowlistPort, linkSet: Int): Boolean =
    port.routable && (linkSet == 0 || (linkSet and (1 shl port.linkSetBit)) != 0)

/**
 * The reserved set [validate] consumes: SWD, then the pins of every allowlist port that
 * [claimsPins] this boot (`board::plumbing`, `reserved_set`).
 *
 * @param linkSet the board's persisted `LINK_SET` mask, readable over the wire (`store::LINK_SET`).
 */
fun reservedSet(allowlist: List<AllowlistPort>, linkSet: Int): List<Int> =
    SWD_PINS + allowlist.filter { claimsPins(it, linkSet) }.flatMap { it.pins }

/** The reserved set for [chip] under [linkSet], over the compiled allowlist as that part routes it. */
fun reservedSet(chip: ChipFamily, linkSet: Int): List<Int> = reservedSet(allowlistFor(chip), linkSet)
