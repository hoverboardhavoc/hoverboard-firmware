package com.hoverboard.app.net.l2

/**
 * One per-link transport that carries opaque L2 frames (`[ frag-hdr ][ chunk ]`), a mirror of
 * `link.rs`'s `Transport`. The datagram transport puts each frame in one transaction as-is; the
 * byte-stream transport (the BLE bridge, the inter-board UART) wraps it in SOF/len/CRC. L2 never sees
 * the difference.
 */
interface Transport {
    /** The largest L2 frame (`frag-hdr` + chunk) this link puts in one frame; usable chunk is this - 1. */
    fun frameCapacity(): Int

    /** Put one L2 frame (`l2.size <= frameCapacity()`) on the wire. */
    fun sendL2Frame(l2: ByteArray)

    /** Pull the next received L2 frame, or null if none is ready. */
    fun recvL2Frame(): ByteArray?
}

/** Reason [Link.send] failed (mirror of `link.rs`'s `SendError`). */
class SendException(message: String) : Exception(message)

/**
 * L2 over one transport (a mirror of `link.rs`'s `Link`): fragments outgoing packets, reassembles
 * incoming ones. The caller hands whole opaque packets (L3 PDUs) and never sees the MTU.
 */
class Link(private val transport: Transport) {
    private var txPid = 0
    private val reasm = Reassembler()

    /** The largest packet this link will carry: 16 fragments x usable-chunk. */
    fun mtuHint(): Int = FragHdr.MAX_FRAGMENTS * (transport.frameCapacity() - 1)

    /** Deliver one opaque packet to the peer, fragmenting to the link's frame capacity. */
    fun send(packet: ByteArray) {
        val chunkCap = transport.frameCapacity() - 1
        try {
            fragment(packet, chunkCap, txPid) { hdr, chunk ->
                val frame = ByteArray(1 + chunk.size)
                frame[0] = hdr.encode().toByte()
                System.arraycopy(chunk, 0, frame, 1, chunk.size)
                transport.sendL2Frame(frame)
            }
        } catch (e: FragException) {
            throw SendException("packet too large: ${e.message}")
        }
        txPid = (txPid + 1) and FragHdr.MAX_PID
    }

    /**
     * Return the next fully reassembled packet, or null. Non-blocking: drains the transport's ready
     * frames and feeds them through reassembly, returning the first completed packet.
     */
    fun pollRecv(): ByteArray? {
        while (true) {
            val frame = transport.recvL2Frame() ?: return null
            if (frame.isEmpty()) continue // a frame with no frag-hdr cannot exist; ignore defensively
            val hdr = frame[0].toInt() and 0xFF
            val chunk = frame.copyOfRange(1, frame.size)
            val pkt = reasm.push(hdr, chunk)
            if (pkt != null) return pkt
        }
    }
}
