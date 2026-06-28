package com.hoverboard.app.net.l2

/**
 * L2 [Transport] over a raw BLE byte stream (the master's onboard CC2541 transparent-UART bridge), a
 * mirror of the firmware's byte-stream `SerialTransport` (`crates/link/src/serial.rs`).
 *
 * Outgoing L2 frames (`[ frag-hdr ][ chunk ]`) are wrapped SOF/len/CRC with [StreamFrame.encode] and
 * queued as a continuous byte stream for the GATT write char (0x1001); incoming notification bytes
 * from the notify char (0x1002) are fed through a resyncing [StreamFramer] back into whole L2 frames.
 *
 * The CC2541 bridge is a **byte stream**: it does NOT preserve frame boundaries either way (it
 * coalesces successive writes and re-chunks the notify stream, e.g. a 20 B echo arriving as 18 + 2).
 * The length-delimited SOF/len/CRC framing tolerates any chunking, so this transport never assumes
 * one notification equals one frame.
 *
 * Synchronous and I/O-free by design: the protocol stepping ([com.hoverboard.app.net.l3.BleWalkEngine])
 * is host-testable through a mock byte-stream loopback, while the BLE coroutine plumbing (drain the
 * outgoing bytes to the write char; feed notification bytes to [onReceive]) lives in the driver.
 */
class BleStreamTransport(
    private val frameCapacity: Int = DEFAULT_FRAME_CAPACITY,
) : Transport {

    private val framer = StreamFramer()
    private val rxFrames = ArrayDeque<ByteArray>()
    private val txStream = ArrayDeque<Byte>()

    override fun frameCapacity(): Int = frameCapacity

    /** Wrap one L2 frame SOF/len/CRC and append its bytes to the outgoing stream. */
    override fun sendL2Frame(l2: ByteArray) {
        for (b in StreamFrame.encode(l2)) txStream.addLast(b)
    }

    /** The next reassembled inbound L2 frame the framer has completed, or null. */
    override fun recvL2Frame(): ByteArray? = rxFrames.removeFirstOrNull()

    /** Feed raw bytes received from the notify char; completed L2 frames become available to [recvL2Frame]. */
    fun onReceive(bytes: ByteArray) = framer.feed(bytes) { rxFrames.addLast(it) }

    /** Drain all pending outgoing stream bytes (to write to the GATT write char), or null if none. */
    fun drainOutgoing(): ByteArray? {
        if (txStream.isEmpty()) return null
        val out = ByteArray(txStream.size)
        var i = 0
        while (txStream.isNotEmpty()) out[i++] = txStream.removeFirst()
        return out
    }

    /** Reset the inbound framer to the hunt state, dropping any partial frame (on (re)connect). */
    fun resetRx() = framer.reset()

    companion object {
        /**
         * The L2 frame capacity (`frag-hdr` + chunk) for the BLE link, mirroring the firmware's
         * `BLE_FRAME_CAP` (`crates/firmware/src/main.rs`): 16 B, so a whole stream frame is
         * `SOF + len + 16 + CRC = 20 B` = one BLE ATT write.
         */
        const val DEFAULT_FRAME_CAPACITY = 16
    }
}
