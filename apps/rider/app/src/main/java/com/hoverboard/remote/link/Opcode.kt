package com.hoverboard.remote.link

/**
 * Frame opcode (byte 2 of the header). Mirrors crates/link/src/opcode.rs.
 *
 * Decode does NOT fail on an unknown opcode byte: a frame with an unrecognized opcode still decodes
 * structurally (CRC still checked) and [fromByte] returns null for it, matching the Rust
 * `Opcode::Unknown(b)` behaviour. A node ignores opcodes outside its caps after the CRC passes.
 */
enum class Opcode(val value: Int) {
    NodeHello(0x01),
    CyclicState(0x10),
    DriveCmd(0x11),
    Telemetry(0x20),
    ConfigRead(0x30),
    ConfigWrite(0x31),
    ConfigResp(0x32),
    Fault(0x40),
    Inputs(0x50),
    Event(0x60);

    companion object {
        /** Map a raw opcode byte to a known [Opcode], or null if this build does not recognize it. */
        fun fromByte(b: Int): Opcode? = entries.firstOrNull { it.value == (b and 0xFF) }
    }
}
