package com.hoverboard.remote

import com.hoverboard.protocol.config.Busy
import kotlinx.coroutines.delay

/** Back-off while another model's operation holds the session's request slot. */
internal const val SLOT_RETRY_MS = 50L

/** Enough retries to outlast one request's whole retransmit budget (five 1 s reply windows). */
internal const val SLOT_RETRIES = 120

/**
 * Run [call] again while it answers [Busy], up to [SLOT_RETRIES] times, and return the first other
 * answer (or the last [Busy]).
 *
 * The config and tune lanes share one request slot per session
 * ([com.hoverboard.protocol.config.ConfigExchange]), and the Setup and Tune models each run one
 * operation at a time, but not in step with each other: Setup reads the ride facts on attach while a
 * shown Tune screen reads its gains. Within a model, one operation in flight is enforced by the
 * model's own lock, so a [Busy] here can only mean the OTHER model holds the slot for one request,
 * and waiting it out is the answer; refusing would drop that model's read for the session.
 */
internal suspend fun <T> awaitSlot(call: suspend () -> T?): T? {
    var r = call()
    var tries = 0
    while (r == Busy && tries++ < SLOT_RETRIES) {
        delay(SLOT_RETRY_MS)
        r = call()
    }
    return r
}
