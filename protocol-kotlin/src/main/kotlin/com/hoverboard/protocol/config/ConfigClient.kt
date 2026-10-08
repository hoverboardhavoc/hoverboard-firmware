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
 * That is all it says. No store field applies live (every field is read once at boot), so this is
 * a statement about flash, never about what a running loop is using.
 */
data class WriteVerified(val stored: Value) : ConfigWriteResult

/**
 * The write was accepted (`CFG_OK`) but the echoed stored value is not what was written, or the
 * echo carried no decodable value at all ([stored] null). The board's store holds [stored], not
 * [wrote].
 */
data class WriteMismatch(val wrote: Value, val stored: Value?) : ConfigWriteResult

/** The board answered with a named refusal. */
data class Refused(val refusal: CfgRefusal) : ConfigReadResult, ConfigWriteResult

/**
 * The board answered with something the wire contract does not define: an unknown status byte, or
 * a `CFG_OK` read whose value does not decode by its own type tag.
 */
data class Malformed(val resp: ConfigResp) : ConfigReadResult, ConfigWriteResult

/**
 * The request went unanswered through the engine's whole retransmit budget. A write that times out
 * may or may not have been applied; a read of the same key settles it.
 */
data object TimedOut : ConfigReadResult, ConfigWriteResult

/**
 * Another operation was in flight, so this one was not sent. The caller decides whether to try
 * again; nothing is queued behind its back.
 */
data object Busy : ConfigReadResult, ConfigWriteResult

/**
 * The request/response layer over `CONFIG_READ` / `CONFIG_WRITE` (`specs/rider-ui.md` section 2):
 * one operation at a time, to an explicitly named board, with the [engine]'s own retransmit, the
 * status decoded to a typed result, and every write verified against the stored value its
 * `CONFIG_RESP` echoes.
 *
 * ## It does not turn the engine
 *
 * The engine has exactly one driver per session (in the rider app, `L3Session`'s service loop, the
 * link's single writer): that loop pumps received packets into the engine, which captures each
 * `CONFIG_RESP`, and flushes the engine's outgoing bytes, which carries this client's requests and
 * the engine's re-sends. This client only stages requests, collects responses and asks the engine
 * whether its request is overdue ([BleWalkEngine.serviceRetransmit]), all under [lock], the same
 * lock that driver holds.
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
 * earlier exchange) is discarded, and the inbox is drained before each request is sent.
 */
class ConfigClient(
    private val engine: BleWalkEngine,
    private val lock: Any,
    private val pollIdleMs: Long = DEFAULT_POLL_IDLE_MS,
) {
    private val inFlight = Mutex()

    /** Read [key] from the board at [target] (a board address from this session's discovery). */
    suspend fun read(key: Key, target: Int): ConfigReadResult {
        require(isBoard(target)) { "target 0x${Integer.toHexString(target)} is not a board address" }
        if (!inFlight.tryLock()) return Busy
        try {
            val resp = exchange(key, target) { engine.sendConfigRead(target, key) } ?: return TimedOut
            return when (resp.status) {
                Walk.CFG_OK -> resp.decodeValue()?.let { ReadValue(it) } ?: Malformed(resp)
                else -> CfgRefusal.fromCode(resp.status)?.let { Refused(it) } ?: Malformed(resp)
            }
        } finally {
            inFlight.unlock()
        }
    }

    /**
     * Write [value] to [key] on the board at [target] and verify the stored value the response
     * echoes. The write is persisted to flash only; see [WriteVerified].
     */
    suspend fun write(key: Key, value: Value, target: Int): ConfigWriteResult {
        require(isBoard(target)) { "target 0x${Integer.toHexString(target)} is not a board address" }
        if (!inFlight.tryLock()) return Busy
        try {
            val resp = exchange(key, target) { engine.sendConfigWrite(target, key, value) } ?: return TimedOut
            return when (resp.status) {
                Walk.CFG_OK -> {
                    val stored = resp.decodeValue()
                    if (stored == value) WriteVerified(stored) else WriteMismatch(wrote = value, stored = stored)
                }
                else -> CfgRefusal.fromCode(resp.status)?.let { Refused(it) } ?: Malformed(resp)
            }
        } finally {
            inFlight.unlock()
        }
    }

    /**
     * Send one request (staged by [send] under [lock]) and wait for its response, or null once the
     * engine reports the retransmit budget exhausted.
     */
    private suspend fun exchange(key: Key, target: Int, send: () -> Unit): ConfigResp? {
        synchronized(lock) {
            while (engine.takeConfigResp() != null) Unit // stale responses from earlier exchanges
            send()
        }
        while (true) {
            synchronized(lock) {
                takeMatching(key, target)?.let { return it }
                if (engine.serviceRetransmit() == Retransmit.EXHAUSTED) return null
            }
            delay(pollIdleMs)
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
        const val DEFAULT_POLL_IDLE_MS = 20L
    }
}
