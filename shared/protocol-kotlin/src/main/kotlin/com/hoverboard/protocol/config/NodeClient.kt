package com.hoverboard.protocol.config

import com.hoverboard.protocol.board.BoardStatus
import com.hoverboard.protocol.l3.Identity
import com.hoverboard.protocol.l3.Opcode

/** How a [NodeClient.identify] ended. */
sealed interface IdentifyResult

/** How a [NodeClient.status] ended. */
sealed interface StatusReadResult

/** The node's `IDENTITY`. */
data class Identified(val identity: Identity) : IdentifyResult

/** The node's boot status record: what its last bring-up could not bring up; empty when nothing. */
data class StatusEntries(val entries: List<BoardStatus.Entry>) : StatusReadResult

/** The node answered with a payload the wire contract does not define (an `IDENTITY` of the wrong length). */
data class Unreadable(val payload: ByteArray) : IdentifyResult {
    override fun equals(other: Any?): Boolean = other is Unreadable && payload.contentEquals(other.payload)
    override fun hashCode(): Int = payload.contentHashCode()
}

/**
 * The two facts a node reports about itself, outside the registry (`specs/machine-settings.md`
 * section 6.1): `IDENTIFY` (what it is: class, chip, firmware version, factory unique ID) and
 * `STATUS_READ` (its boot status record). One request at a time on the session's [ConfigExchange],
 * the slot the config and tune lanes share.
 */
class NodeClient(private val exchange: ConfigExchange) {

    /** Ask the node at [target] what it is. */
    suspend fun identify(target: Int): IdentifyResult =
        when (val o = exchange.requestReply(Opcode.Identify, Opcode.Identity, target) { sendIdentify(target) }) {
            ConfigExchange.Asked.Busy -> Busy
            ConfigExchange.Asked.TimedOut -> TimedOut
            is ConfigExchange.Asked.Answered -> Identity.decode(o.value)?.let { Identified(it) } ?: Unreadable(o.value)
        }

    /** Read the node at [target]'s boot status record. A trailing partial entry is ignored. */
    suspend fun status(target: Int): StatusReadResult =
        when (val o = exchange.requestReply(Opcode.StatusRead, Opcode.Status, target) { sendStatusRead(target) }) {
            ConfigExchange.Asked.Busy -> Busy
            ConfigExchange.Asked.TimedOut -> TimedOut
            is ConfigExchange.Asked.Answered -> StatusEntries(BoardStatus.decode(o.value))
        }
}
