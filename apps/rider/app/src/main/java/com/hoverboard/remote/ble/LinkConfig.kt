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
     * 20 Hz, and the reason is the failure it fixes. The firmware calls the demand stale once
     * `drive_age > DRIVE_TIMEOUT_TICKS` (50 ticks at 250 Hz, `crates/linkctl/src/lib.rs:57`), so
     * strictly at 204 ms of silence, and the link is best-effort with no retransmit: the cadence
     * decides how many consecutive lost frames it takes before the reference starts falling. At
     * 10 Hz that number is ONE, and a single dropped frame was a visible stutter. That is exactly
     * what a 10 Hz build did on the bench, reported as slow and jittery with drop-outs.
     *
     * At 20 Hz it takes four, or three plus a few ms of jitter: three losses at an exact 50 ms
     * cadence put the next frame at 200 ms, which just survives the strict test, and Android's
     * connection-interval quantization makes a few ms of slip routine. So the honest claim is
     * "rides through an ordinary RF dropout", not a guarantee at three.
     *
     * Going stale is also a ramp rather than a zeroing: the firmware feeds (0, 0) into the same
     * rate limiter (`RATE = 480` fixdt = 30 units/tick of its 1000-domain,
     * `crates/control/src/config.rs:157`), so a full demand needs a further ~133 ms, plus the
     * low-pass tail, to reach zero. A brief loss is a sag and a recovery; a stop needs sustained
     * silence.
     *
     * Doubling the rate did not double the traffic, because the `INPUTS` half stopped going out
     * every tick; see [INPUTS_KEEPALIVE_TICKS]. A stream frame is
     * `SOF + len + frag-hdr + PDU + CRC16` = PDU + 5
     * (`com.hoverboard.protocol.l2.StreamFrame`), so `DRIVE_CMD` (5-byte payload, 8-byte PDU) is
     * 13 bytes on the wire and `INPUTS` (4-byte payload, 7-byte PDU) is 12. That is 260 B/s of
     * demand plus ~24 B/s of keepalive, ~284 B/s armed, of the CC2541's ~960 B/s metered UART,
     * against 250 B/s for the 10 Hz build that sent both payloads every tick. The module's known
     * ceiling is the ~360 B/s (30 Hz x 12 B) at which the old INPUTS-only build overran its
     * BLE-to-UART buffer.
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
     * 2 Hz, and at 12 bytes a frame ([SEND_INTERVAL_MS] for the wire arithmetic) about 24 B/s.
     * [CommandPump] counts the tick it is deciding, so the number here is the interval itself and
     * not one less than it.
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
