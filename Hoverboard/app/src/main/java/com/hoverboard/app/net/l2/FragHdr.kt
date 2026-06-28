package com.hoverboard.app.net.l2

/**
 * The one-byte L2 fragmentation header, a mirror of `crates/link/src/frag.rs`. Bit layout:
 *
 * ```text
 * bit 7      MORE       1 = more fragments of this packet follow, 0 = last (or only) fragment
 * bits 6..4  PID        packet id, 0..7, increments per packet on a link (wraps); groups a set
 * bits 3..0  FRAG_IDX   fragment index within the packet, 0..15
 * ```
 *
 * It carries no addressing (that is L3); the chunk it prefixes is an opaque L3 packet slice.
 */
data class FragHdr(val more: Boolean, val pid: Int, val fragIdx: Int) {

    /** Pack into the single wire byte; over-range pid/frag_idx are masked to their fields. */
    fun encode(): Int {
        val moreBit = if (more) MORE_BIT else 0
        return moreBit or ((pid and MAX_PID) shl PID_SHIFT) or (fragIdx and MAX_FRAG_IDX)
    }

    companion object {
        /** `MORE` bit (bit 7). */
        const val MORE_BIT = 0b1000_0000
        private const val PID_SHIFT = 4

        /** `PID` is 3 bits (0..7); also the wrap mask. */
        const val MAX_PID = 0b0000_0111

        /** `FRAG_IDX` is 4 bits (0..15). */
        const val MAX_FRAG_IDX = 0b0000_1111

        /** A packet is bounded to 16 fragments (`FRAG_IDX` 0..15). */
        const val MAX_FRAGMENTS = 16

        /** Unpack the single wire byte (every byte is a valid header; all fields are fixed-width). */
        fun decode(b: Int): FragHdr =
            FragHdr(
                more = (b and MORE_BIT) != 0,
                pid = (b ushr PID_SHIFT) and MAX_PID,
                fragIdx = b and MAX_FRAG_IDX,
            )
    }
}
