package com.hoverboard.remote

import com.hoverboard.protocol.config.Busy
import kotlinx.coroutines.delay

/** Back-off while another model's operation holds the session's request slot. */
internal const val SLOT_RETRY_MS = 50L

/** Outlasts one request's whole retransmit budget (five 1 s reply windows), not a whole read pass. */
internal const val SLOT_RETRIES = 120

/**
 * Run [call] again while it answers [Busy], up to [SLOT_RETRIES] times, and return the first other
 * answer (or the last [Busy]).
 *
 * The config and tune lanes share one request slot per session
 * ([com.hoverboard.protocol.config.ConfigExchange]), and the Setup and Tune models each run one
 * operation at a time, but not in step with each other: Setup reads the ride facts on attach while a
 * shown Tune screen reads its gains. Within a model, one operation in flight is enforced by the
 * model's own lock, so a [Busy] here means the OTHER model holds the slot, and waiting is the answer;
 * refusing at once would drop this model's request for the session.
 *
 * The other model holds the slot request by request, not for one request: a full read pass takes it
 * for each of its reads in turn, and any read that is re-sent holds it for a further reply timeout.
 * Its pass can therefore outlast the [SLOT_RETRIES] budget, and a last [Busy] returned from here
 * means only that the slot stayed taken: nothing was sent and the board said nothing. A caller
 * reports it as such (the Tune screen's [TuneNotice.SlotBusy], with a retry), never as a board answer.
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
