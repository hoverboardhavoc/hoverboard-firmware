package com.hoverboard.remote.model

import com.hoverboard.protocol.linkctl.CyclicState
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * What each board has said, keyed by the L3 SOURCE ADDRESS its frames arrived under.
 *
 * Two boards' `CYCLIC_STATE` arrive on the one Bluetooth link. The attached board emits its own at
 * 5 Hz, and it re-emits its peer's last fresh state at 2.5 Hz under the PEER's address
 * (`specs/link-control.md`, "The peer's state on the BLE port: the forward"), so what distinguishes
 * them is the `src` byte the L3 header has always carried. Merged into one record they are one
 * board contradicting itself twice a second; keyed by source they are two boards, each saying what
 * it alone can say about itself.
 *
 * ## A board that has gone quiet is dropped, not frozen
 *
 * A source stays in the map only while it keeps speaking. The attached board stops being heard when
 * the phone link stalls; the peer's entry stops when the attached board stops forwarding it, which
 * is what it does the moment it no longer trusts the reading itself (its peer has gone silent).
 * Either way the app has nothing current to say about that board, and the last reading is not an
 * answer: a frozen pack voltage and a frozen lean angle read exactly like live ones.
 *
 * The window is [QUIET_MS], measured from the last frame of any kind from that source on the
 * phone's own monotonic clock. It is set against the SLOWER of the two streams, the 2.5 Hz forward:
 * two missed forwards is 800 ms, plus room for a connection-parameter churn pause, so a board that
 * is merely slow to be heard is never called absent. The faster 5 Hz stream is covered by the same
 * number with five frames to spare.
 *
 * The caller owns the clock and the pump: [cyclic] and [fault] stamp a source as heard, [expire]
 * drops the ones that have gone quiet, and the owner calls it on a timer it already runs. Nothing
 * here reads a clock of its own, which is what makes the window testable without waiting for it.
 */
class TelemetryInbox(private val quietMs: Long = QUIET_MS) {

    private val heardAtMs = mutableMapOf<Int, Long>()
    private val _boards = MutableStateFlow<Map<Int, TelemetryUi>>(emptyMap())

    /** Every board heard from inside the window, by its L3 address. */
    val boards: StateFlow<Map<Int, TelemetryUi>> = _boards.asStateFlow()

    /** Fold one board's `CYCLIC_STATE` in, latest-wins, and stamp it heard at [nowMs]. */
    fun cyclic(src: Int, state: CyclicState, nowMs: Long) {
        put(src, nowMs) { it.merge(state) }
    }

    /**
     * Fold one board's FAULT edge in, and stamp it heard at [nowMs].
     *
     * The edge is kept with that board's cyclic record rather than beside it, so it expires with
     * the stream that would have told the app the fault had cleared.
     */
    fun fault(src: Int, stop: Boolean, code: Int, nowMs: Long) {
        put(src, nowMs) { it.copy(faultStop = stop, faultCode = code) }
    }

    /** Drop every source that has not been heard from for [quietMs]. */
    fun expire(nowMs: Long) {
        val quiet = heardAtMs.filterValues { nowMs - it >= quietMs }.keys
        if (quiet.isEmpty()) return
        heardAtMs.keys.removeAll(quiet)
        _boards.value = _boards.value - quiet
    }

    /** Forget every board: a session that has ended knows nothing about either of them. */
    fun clear() {
        heardAtMs.clear()
        _boards.value = emptyMap()
    }

    private fun put(src: Int, nowMs: Long, fold: (TelemetryUi) -> TelemetryUi) {
        heardAtMs[src] = nowMs
        _boards.value = _boards.value + (src to fold(_boards.value[src] ?: TelemetryUi()))
    }

    companion object {
        /** How long a source may stay silent before the app stops claiming to know its state. */
        const val QUIET_MS = 1_200L
    }
}
