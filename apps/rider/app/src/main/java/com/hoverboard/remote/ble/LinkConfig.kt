package com.hoverboard.remote.ble

/**
 * Compiled-in link defaults for the rider remote.
 *
 * There are no node ids here. The app's own address and the board's address are **session facts**,
 * not configuration: the app is a transient controller (`specs/l3.md`), so it takes a guest address
 * from `0x80..0xFE` in the board's `NODE_HELLO` reply at first contact and addresses that board by
 * the id the same exchange reported (assigning one if the board reports none). Both are held by
 * [BleHoverboardTransport] for the life of one connection and dropped on disconnect; a compiled-in
 * copy would be a second, wrong answer to a question the wire already answers.
 *
 * The advertised name to scan for is likewise not a field here. It is user-settable and persisted,
 * so it is owned by [LinkSettings] and there is exactly one place to read it from;
 * [DEFAULT_DEVICE_NAME] is the seed [LinkSettings] falls back to when nothing is stored.
 */
object LinkConfig {

    /**
     * The name a board advertises out of the box, and so the name the app looks for until the
     * user changes it.
     *
     * Case matters. The scan filter is an exact string comparison (after trimming the padding
     * the CC2541 adds), and the firmware's own store default for `DEVICE_NAME` is the LOWERCASE
     * `"hoverboard"`. A board left unstaged therefore does NOT match this, by design: the bench
     * procedure stages the name explicitly (specs/ble-session.md in the firmware repo), which is
     * also the only way to tell two boards apart on a scan list.
     */
    const val DEFAULT_DEVICE_NAME: String = "Hoverboard"

    /**
     * How often [CommandPump] re-sends the drive demand.
     *
     * 20 Hz, and the reason is the failure it fixes. `DRIVE_CMD` decays after 200 ms
     * (`crates/linkctl/src/lib.rs:57`) and the link is best-effort with no retransmit, so the
     * cadence decides how many consecutive lost frames it takes to zero the reference. At 10 Hz
     * that number is ONE: a single dropped frame opens a 200 ms gap, the reference decays, and the
     * motor stutters. That is exactly what a 10 Hz build did on the bench, reported as slow and
     * jittery with drop-outs. At 20 Hz it takes three consecutive losses, which is the difference
     * between a link that stutters constantly and one that rides through an ordinary RF dropout.
     *
     * Doubling the rate did not double the traffic, because the `INPUTS` half stopped going out
     * every tick; see [INPUTS_KEEPALIVE_TICKS]. One `DRIVE_CMD` is 12 bytes on the wire, so this is
     * ~240 B/s of the CC2541's ~960 B/s metered UART, against ~230 B/s for the 10 Hz build that
     * sent both payloads every tick. The module's known ceiling is the ~330 B/s at which 30 Hz
     * overran its BLE-to-UART buffer.
     *
     * It lives here rather than in the transport because the ViewModel needs it too, to size the
     * window it holds the link open for after disarming
     * ([com.hoverboard.remote.MainViewModel.disconnect]).
     */
    const val SEND_INTERVAL_MS: Long = 50L

    /**
     * How often the `INPUTS` mirror is re-sent when nothing about it has changed.
     *
     * `INPUTS` does not decay. The firmware stores the remote mirror latest-wins with no age at all
     * (`crates/orchestrator/src/lib.rs:223`), so unlike the drive demand, re-sending an unchanged
     * arm level buys nothing: the board is already holding it. Streaming it every tick was pure
     * cost on a metered link, and that cost came straight out of the drive demand's timing budget.
     *
     * So it is sent on change (repeated [INPUTS_CHANGE_REPEATS] times, because a lost arm frame
     * would otherwise be missed entirely on a link with no retransmit) and then only as a slow
     * keepalive, to re-assert the level if the board and the app ever disagree.
     *
     * This is the GAP between consecutive `INPUTS` sends, in ticks: 10 ticks at 20 Hz is 500 ms,
     * 2 Hz, about 22 B/s. [CommandPump] counts the tick it is deciding, so the number here is the
     * interval itself and not one less than it.
     */
    const val INPUTS_KEEPALIVE_TICKS: Int = 10

    /**
     * How many consecutive ticks a CHANGED `INPUTS` level is repeated on.
     *
     * A level that latches forever is exactly the one that must not be dropped: a lost disarm frame
     * leaves a board armed with nothing scheduled to correct it until the next keepalive. Three
     * back-to-back sends inside 150 ms is cheap insurance against a single RF dropout.
     */
    const val INPUTS_CHANGE_REPEATS: Int = 3
}
