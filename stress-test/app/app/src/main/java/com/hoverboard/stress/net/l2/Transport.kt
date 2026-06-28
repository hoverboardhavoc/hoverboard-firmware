package com.hoverboard.stress.net.l2

/**
 * One per-link transport that carries opaque L2 frames (`[ frag-hdr ][ chunk ]`), a mirror of
 * `link.rs`'s `Transport` (extracted from `Hoverboard/.../net/l2/Link.kt`). The byte-stream transport
 * (the BLE bridge) wraps each L2 frame in SOF/len/CRC; the stress runner only needs the byte-stream
 * variant ([BleStreamTransport]), so the L2 `Link` fragment/reassembly layer is not copied here.
 */
interface Transport {
    /** The largest L2 frame (`frag-hdr` + chunk) this link puts in one frame; usable chunk is this - 1. */
    fun frameCapacity(): Int

    /** Put one L2 frame (`l2.size <= frameCapacity()`) on the wire. */
    fun sendL2Frame(l2: ByteArray)

    /** Pull the next received L2 frame, or null if none is ready. */
    fun recvL2Frame(): ByteArray?
}
