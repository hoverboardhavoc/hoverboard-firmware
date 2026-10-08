package com.hoverboard.protocol.config

import com.hoverboard.protocol.l3.BleWalkEngine
import com.hoverboard.protocol.l3.ConfigResp
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.l3.Retransmit
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.l3.isBoard
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.delay
import kotlinx.coroutines.sync.Mutex

/**
 * A `CONFIG_RESP` refusal status: every status the firmware defines other than `CFG_OK`
 * (`crates/net/src/walk.rs`; the codes are [Walk]'s, which `RustSourceDriftTest` pins).
 */
enum class CfgRefusal(val code: Int) {
    /** A malformed request (unknown type tag, bad value bytes, truncated PDU). */
    BAD(Walk.CFG_BAD),

    /** No field declares this `field_id`. */
    UNKNOWN_KEY(Walk.CFG_UNKNOWN_KEY),

    /** The value's type did not match the field's registered type. */
    TYPE_MISMATCH(Walk.CFG_TYPE_MISMATCH),

    /** The store write failed (flash full / error). */
    STORE_ERR(Walk.CFG_STORE_ERR),

    /** A write refused because the board is armed. Reads are never refused for this. */
    ARMED(Walk.CFG_ARMED),
    ;

    companion object {
        fun fromCode(code: Int): CfgRefusal? = entries.firstOrNull { it.code == code }
    }
}

/** How a [ConfigClient.read] ended. */
sealed interface ConfigReadResult

/** How a [ConfigClient.write] ended. */
sealed interface ConfigWriteResult

/** The board's stored value for the key, as `CONFIG_RESP` reported it. */
data class ReadValue(val value: Value) : ConfigReadResult

/**
 * The write was accepted and the `CONFIG_RESP` echo of the STORED value equals what was written.
 *
 * That is all it says: the STORED value matched, a statement about flash, never about what a
 * running loop is using. Whether and how fast a running loop follows a stored value is the field's
 * own business (`specs/rider-ui.md` section 3.3: a gain's engaged value converges to the staged
 * one, it does not jump).
 */
data class WriteVerified(val stored: Value) : ConfigWriteResult

/**
 * The write was accepted (`CFG_OK`) but the echoed stored value is not what was written, or the
 * echo carried no decodable value at all ([stored] null). Normally the board's store holds
 * [stored], not [wrote].
 *
 * Not always: the wire has no sequence number, so two consecutive writes of the SAME key to the
 * SAME board, where the first one's reply arrives after the engine's retransmit budget (so it
 * timed out) and a duplicate of that reply lands after the second request was sent, are reported
 * as `WriteMismatch(wrote = second, stored = first)` although the store may hold the second. The
 * error is in the safe direction, a spurious mismatch, which a screen answers with a re-read of
 * the key.
 */
data class WriteMismatch(val wrote: Value, val stored: Value?) : ConfigWriteResult

/** The board answered with a named refusal. */
data class Refused(val refusal: CfgRefusal) :
    ConfigReadResult, ConfigWriteResult, TuneReadResult, TuneWriteResult

/**
 * The board answered with something the wire contract does not define: an unknown status byte, or
 * a `CFG_OK` read whose value does not decode by its own type tag (for the tune lane, does not
 * decode as the `I16` every live-tunable field is).
 */
data class Malformed(val resp: ConfigResp) :
    ConfigReadResult, ConfigWriteResult, TuneReadResult, TuneWriteResult

/**
 * The request went unanswered through the engine's whole retransmit budget. A write that times out
 * may or may not have been applied; a read of the same key settles it.
 */
data object TimedOut : ConfigReadResult, ConfigWriteResult, TuneReadResult, TuneWriteResult

/**
 * Another operation was in flight, so this one was not sent. The caller decides whether to try
 * again; nothing is queued behind its back. "Another operation" includes one on the OTHER client
 * sharing the same [ConfigExchange]: the config and tune lanes share the engine's one outstanding
 * request.
 */
data object Busy : ConfigReadResult, ConfigWriteResult, TuneReadResult, TuneWriteResult

/**
 * The engine's one outstanding request, shared by every client that sends a request answered by
 * `CONFIG_RESP`: [ConfigClient] (`CONFIG_READ` / `CONFIG_WRITE`) and [TuneClient] (`TUNE_READ` /
 * `TUNE_WRITE`).
 *
 * One per session, because the engine arms ONE request for retransmit and both lanes answer with
 * the same opcode: a `CONFIG_READ` and a `TUNE_READ` of the same key to the same board get replies
 * that cannot be told apart. Two clients each with its own lock could have one of each in flight,
 * and each could take the other's answer. So the lock lives here and a second operation on either
 * lane is refused with [Busy].
 *
 * ## It does not turn the engine
 *
 * The engine has exactly one driver per session (in the rider app, `L3Session`'s service loop, the
 * link's single writer): that loop pumps received packets into the engine, which captures each
 * `CONFIG_RESP`, and flushes the engine's outgoing bytes, which carries the clients' requests and
 * the engine's re-sends. This only stages requests, collects responses and asks the engine whether
 * its request is overdue ([BleWalkEngine.serviceRetransmit]), all under [lock], the same lock that
 * driver holds.
 *
 * ## One operation in flight: a second caller is refused, not queued
 *
 * A second call while one is outstanding returns [Busy] at once. Queueing would let a burst of
 * taps become a burst of back-to-back writes, which is the streaming the config path's stack depth
 * forbids (`specs/rider-ui.md` section 1.7: one in flight, apply-on-release, nothing streamed); a
 * refusal leaves that decision with the screen, which knows whether a newer value supersedes it.
 *
 * ## Matching a response to its request
 *
 * The wire carries no sequence number. A response is taken as the answer when it comes FROM the
 * target board and names the requested key; anything else in the inbox (a late duplicate of an
 * earlier exchange) is discarded, and the inbox is drained before each request is sent. A late
 * duplicate for the SAME board and key is indistinguishable from the answer; see [WriteMismatch]
 * for the one case that produces and why it fails safe.
 */
class ConfigExchange(
    private val engine: BleWalkEngine,
    private val lock: Any,
) {
    private val inFlight = Mutex()

    /**
     * Send one request to [target] about [key] (staged by [send] under [lock]) and wait for its
     * response: the response, or [Busy] if another operation holds the slot, or [TimedOut] once the
     * engine reports the retransmit budget exhausted.
     */
    internal suspend fun request(key: Key, target: Int, send: BleWalkEngine.() -> Unit): Outcome {
        require(isBoard(target)) { "target 0x${Integer.toHexString(target)} is not a board address" }
        if (!inFlight.tryLock()) return Outcome.Busy
        try {
            return exchange(key, target, send)?.let { Outcome.Answered(it) } ?: Outcome.TimedOut
        } finally {
            inFlight.unlock()
        }
    }

    /** What [request] came to. */
    internal sealed interface Outcome {
        data class Answered(val resp: ConfigResp) : Outcome
        data object TimedOut : Outcome
        data object Busy : Outcome
    }

    private suspend fun exchange(key: Key, target: Int, send: BleWalkEngine.() -> Unit): ConfigResp? {
        synchronized(lock) {
            while (engine.takeConfigResp() != null) Unit // stale responses from earlier exchanges
            engine.send()
        }
        while (true) {
            synchronized(lock) {
                takeMatching(key, target)?.let { return it }
                if (engine.serviceRetransmit() == Retransmit.EXHAUSTED) return null
            }
            delay(POLL_IDLE_MS)
        }
    }

    /** Drain the inbox up to the first response from [target] naming [key]; discard the rest. */
    private fun takeMatching(key: Key, target: Int): ConfigResp? {
        while (true) {
            val bytes = engine.takeConfigResp() ?: return null
            val pdu = Pdu.decodeOrNull(bytes) ?: continue
            val resp = ConfigResp.parse(pdu) ?: continue
            if (pdu.src == target && resp.fieldId == key.fieldId && resp.index == key.index) return resp
        }
    }

    companion object {
        /** Idle backoff between checks for the response (the session loop's own poll cadence). */
        const val POLL_IDLE_MS = 20L
    }
}

/**
 * The request/response layer over `CONFIG_READ` / `CONFIG_WRITE` (`specs/rider-ui.md` section 2):
 * one operation at a time (on the session's [ConfigExchange], which the tune lane shares), to an
 * explicitly named board, with the engine's own retransmit, the status decoded to a typed result,
 * and every write verified against the stored value its `CONFIG_RESP` echoes.
 */
class ConfigClient(private val exchange: ConfigExchange) {

    /** Read [key] from the board at [target] (a board address from this session's discovery). */
    suspend fun read(key: Key, target: Int): ConfigReadResult =
        when (val o = exchange.request(key, target) { sendConfigRead(target, key) }) {
            ConfigExchange.Outcome.Busy -> Busy
            ConfigExchange.Outcome.TimedOut -> TimedOut
            is ConfigExchange.Outcome.Answered -> {
                val resp = o.resp
                when (resp.status) {
                    Walk.CFG_OK -> resp.decodeValue()?.let { ReadValue(it) } ?: Malformed(resp)
                    else -> CfgRefusal.fromCode(resp.status)?.let { Refused(it) } ?: Malformed(resp)
                }
            }
        }

    /**
     * Write [value] to [key] on the board at [target] and verify the stored value the response
     * echoes. The write is persisted to flash only; see [WriteVerified].
     */
    suspend fun write(key: Key, value: Value, target: Int): ConfigWriteResult =
        when (val o = exchange.request(key, target) { sendConfigWrite(target, key, value) }) {
            ConfigExchange.Outcome.Busy -> Busy
            ConfigExchange.Outcome.TimedOut -> TimedOut
            is ConfigExchange.Outcome.Answered -> {
                val resp = o.resp
                when (resp.status) {
                    Walk.CFG_OK -> {
                        val stored = resp.decodeValue()
                        if (stored == value) WriteVerified(stored) else WriteMismatch(wrote = value, stored = stored)
                    }
                    else -> CfgRefusal.fromCode(resp.status)?.let { Refused(it) } ?: Malformed(resp)
                }
            }
        }
}
