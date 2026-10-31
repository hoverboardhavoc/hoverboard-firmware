package com.hoverboard.protocol.config

import com.hoverboard.protocol.l3.ConfigResp
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value

/** How a [TuneClient.read] ended. */
sealed interface TuneReadResult

/** How a [TuneClient.write] ended. */
sealed interface TuneWriteResult

/**
 * The board's STAGED value for the key: its RAM gain shadow, as `TUNE_READ` reported it. Not the
 * flash value (a `CONFIG_READ` of the same key) and not what the balance loop is multiplying now,
 * which converges to this through the firmware's ramp (`specs/rider-ui.md` section 4) and is not
 * readable over the link.
 */
data class StagedValue(val value: Int) : TuneReadResult

/**
 * The write was accepted and the `CONFIG_RESP` echo equals what was written: the board's RAM shadow
 * now holds [staged]. Nothing was persisted, and the running loop has not taken it yet: it ramps
 * toward it.
 */
data class TuneVerified(val staged: Int) : TuneWriteResult

/**
 * The write was accepted (`CFG_OK`) but the echo is not what was written, or carried no `I16` value
 * ([staged] null). The same no-sequence-number caveat as [WriteMismatch] applies; a re-read settles
 * it.
 */
data class TuneMismatch(val wrote: Int, val staged: Int?) : TuneWriteResult

/**
 * The request/response layer over the live tune lane, `TUNE_READ` / `TUNE_WRITE`
 * (`specs/rider-ui.md` section 4): the [ConfigClient]'s shape (one operation in flight, on the SAME
 * [ConfigExchange], to a named board, typed results) over a different contract.
 *
 * - A write lands in the board's RAM shadow only. It never touches flash, so the firmware never
 *   answers it `CFG_ARMED` and it is allowed armed or disarmed.
 * - The allowlist is the two gain fields; any other key is `CFG_UNKNOWN_KEY`. The firmware's seam
 *   range-checks the value and refuses one outside the field's range with `CFG_BAD` (it does not
 *   clamp).
 * - The value is always an `I16`: every live-tunable field is one.
 */
class TuneClient(private val exchange: ConfigExchange) {

    /** Read [key]'s staged value from the board at [target]. */
    suspend fun read(key: Key, target: Int): TuneReadResult =
        when (val o = exchange.request(key, target) { sendTuneRead(target, key) }) {
            ConfigExchange.Outcome.Busy -> Busy
            ConfigExchange.Outcome.TimedOut -> TimedOut
            is ConfigExchange.Outcome.Answered -> {
                val resp = o.resp
                when (resp.status) {
                    Walk.CFG_OK -> resp.i16()?.let { StagedValue(it) } ?: Malformed(resp)
                    else -> CfgRefusal.fromCode(resp.status)?.let { Refused(it) } ?: Malformed(resp)
                }
            }
        }

    /** Stage [value] for [key] on the board at [target] and verify the staged value the response echoes. */
    suspend fun write(key: Key, value: Int, target: Int): TuneWriteResult =
        when (val o = exchange.request(key, target) { sendTuneWrite(target, key, value) }) {
            ConfigExchange.Outcome.Busy -> Busy
            ConfigExchange.Outcome.TimedOut -> TimedOut
            is ConfigExchange.Outcome.Answered -> {
                val resp = o.resp
                when (resp.status) {
                    Walk.CFG_OK -> {
                        val staged = resp.i16()
                        if (staged == value) TuneVerified(staged) else TuneMismatch(wrote = value, staged = staged)
                    }
                    else -> CfgRefusal.fromCode(resp.status)?.let { Refused(it) } ?: Malformed(resp)
                }
            }
        }

    private fun ConfigResp.i16(): Int? = (decodeValue() as? Value.I16)?.v
}
