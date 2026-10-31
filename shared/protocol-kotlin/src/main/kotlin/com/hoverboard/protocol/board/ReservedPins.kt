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
 * `SAFE_LINK_USARTS`): the inter-board link, pins and `LINK_SET` bit only.
 *
 * It held the onboard CC2541's two wirings as well (USART2 on PB10/PB11 under bit 2, USART0 on
 * PB6/PB7 under bit 3) until the firmware stopped driving the module (2026-10-27); those pins are
 * no longer reserved and those bits mean nothing.
 */
private val SAFE_LINK_USARTS = listOf(
    // PA2/PA3 = USART1: the inter-board link, both boards, both families.
    NET_PORT_UART to listOf(0x02, 0x03),
)

/** The `net` slot carrying the inter-board link (`PORT_IDX_UART`), which is also its `LINK_SET` bit. */
const val NET_PORT_UART = 1

/**
 * The allowlist as [chip] can route it.
 *
 * PA2/PA3 is USART1's default mapping on both families, so the inter-board link routes everywhere;
 * `crates/board/src/tests.rs`, `port_routability_agreement`, asserts that against runtime-hal's own
 * pin model.
 */
@Suppress("UNUSED_PARAMETER")
fun allowlistFor(chip: ChipFamily): List<AllowlistPort> = SAFE_LINK_USARTS.map { (bit, pins) ->
    AllowlistPort(linkSetBit = bit, pins = pins, routable = true)
}

/**
 * Is this allowlist port one the link layer may claim its pins for, given [linkSet]
 * (`board::plumbing`, `claims_pins`)?
 *
 * - `linkSet == 0` means UNCONFIGURED: the board will probe every allowlisted port, so every one is
 *   a claimant. A nonzero mask means configured, and only the ports whose bit IS set are live link
 *   ports; the clear-bit ports are freed for other functions.
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
