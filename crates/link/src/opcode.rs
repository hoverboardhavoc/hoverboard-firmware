//! Frame opcodes.
//!
//! The opcode is byte 2 of the header. Decode does NOT fail on an unknown opcode: a node ignores
//! opcodes outside its caps AFTER the CRC passes (per nodes-and-links.md), so unknown values map to
//! `Opcode::Unknown(u8)` and the frame still decodes.

/// Frame opcode. Known variants carry a fixed `#[repr(u8)]` wire value; any other byte becomes
/// `Unknown(u8)` so decode tolerates opcodes this build does not know.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Opcode {
    NodeHello = 0x01,
    CyclicState = 0x10,
    DriveCmd = 0x11,
    Telemetry = 0x20,
    ConfigRead = 0x30,
    ConfigWrite = 0x31,
    ConfigResp = 0x32,
    Fault = 0x40,
    Inputs = 0x50,
    Event = 0x60,
    /// An opcode byte this build does not recognise. Carries the raw value so it round-trips.
    Unknown(u8),
}

impl Opcode {
    /// Map a raw opcode byte to an `Opcode`. Unknown values become `Unknown(v)` rather than failing.
    pub fn from_u8(v: u8) -> Opcode {
        match v {
            0x01 => Opcode::NodeHello,
            0x10 => Opcode::CyclicState,
            0x11 => Opcode::DriveCmd,
            0x20 => Opcode::Telemetry,
            0x30 => Opcode::ConfigRead,
            0x31 => Opcode::ConfigWrite,
            0x32 => Opcode::ConfigResp,
            0x40 => Opcode::Fault,
            0x50 => Opcode::Inputs,
            0x60 => Opcode::Event,
            other => Opcode::Unknown(other),
        }
    }

    /// The raw wire byte for this opcode.
    pub fn to_u8(self) -> u8 {
        match self {
            Opcode::NodeHello => 0x01,
            Opcode::CyclicState => 0x10,
            Opcode::DriveCmd => 0x11,
            Opcode::Telemetry => 0x20,
            Opcode::ConfigRead => 0x30,
            Opcode::ConfigWrite => 0x31,
            Opcode::ConfigResp => 0x32,
            Opcode::Fault => 0x40,
            Opcode::Inputs => 0x50,
            Opcode::Event => 0x60,
            Opcode::Unknown(v) => v,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Opcode;

    #[test]
    fn known_round_trip() {
        for op in [
            Opcode::NodeHello,
            Opcode::CyclicState,
            Opcode::DriveCmd,
            Opcode::Telemetry,
            Opcode::ConfigRead,
            Opcode::ConfigWrite,
            Opcode::ConfigResp,
            Opcode::Fault,
            Opcode::Inputs,
            Opcode::Event,
        ] {
            assert_eq!(Opcode::from_u8(op.to_u8()), op);
        }
    }

    #[test]
    fn unknown_preserves_byte() {
        assert_eq!(Opcode::from_u8(0x7F), Opcode::Unknown(0x7F));
        assert_eq!(Opcode::Unknown(0x7F).to_u8(), 0x7F);
    }
}
