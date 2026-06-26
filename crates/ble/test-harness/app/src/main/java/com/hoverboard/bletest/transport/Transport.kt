package com.hoverboard.bletest.transport

/**
 * The raw, opaque, bidirectional byte pipe below `link` (`specs/ble.md`, "Transport API (raw byte
 * pipe)"). The transport adds NO framing; framing is the caller's job (the harness uses its own test
 * envelope). The same contract is shared by both centrals (the real Android BLE central and the local
 * loopback fake) so they are interchangeable against the same scorer.
 *
 * Carrier guarantee: ordered, not reliable. Bytes never reorder within a direction, but chunks can be
 * dropped under pressure; end-to-end delivery needs an ack/retransmit layer above this (not here).
 */
interface Transport {
    /** Connect / open the pipe. For BLE: scan by name, connect, discover the write+notify chars, set MTU. */
    fun connect()

    /** Send bytes. The transport may chunk internally; it never adds a length prefix or delimiter. */
    fun send(bytes: ByteArray)

    /** Register the sink for received bytes (BLE: notify-characteristic chunks; loopback: echoed bytes). */
    fun onReceive(sink: (ByteArray) -> Unit)

    /** Close the pipe. */
    fun close()
}
