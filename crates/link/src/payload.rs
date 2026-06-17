//! Payload codecs.
//!
//! All multi-byte fields are little-endian, scaled integers, no floats (matches the no-FPU math
//! basis). Decoders read the committed leading fields and IGNORE any trailing bytes, so the
//! trailing-field extensibility the spec commits actually holds: a future build can append fields
//! and an old build still decodes the committed prefix.
//!
//! Two flavours:
//! - fully committed (`CyclicState`, `DriveCmd`): both encode and decode.
//! - committed-in-shape (`NodeHello`, `Telemetry`, `ConfigRead`, `Fault`, `Inputs`): decode now;
//!   encode for the fixed-shape ones too. The borrow-carrying ones (`ConfigWrite`, `ConfigResp`,
//!   `Event`) decode and return the trailing variable-length slice.

/// Reasons a payload decode can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PayloadError {
    /// Fewer bytes than the committed leading fields require.
    TooShort,
    /// The supplied encode buffer is too small.
    OutTooSmall,
}

#[inline]
fn rd_i16(b: &[u8], off: usize) -> i16 {
    i16::from_le_bytes([b[off], b[off + 1]])
}
#[inline]
fn rd_u16(b: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([b[off], b[off + 1]])
}

// --- CyclicState (fully committed) -------------------------------------------------------------

/// Cyclic exchange payload. `status` bits: rider-present, stationary, balance-engaged,
/// fault-pending.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CyclicState {
    pub pitch: i16,
    pub roll: i16,
    pub wheel_speed: i16,
    pub status: u8,
}

impl CyclicState {
    /// On-wire length of the committed fields.
    pub const LEN: usize = 7;

    /// Encode into `out`, returning the byte count.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0..2].copy_from_slice(&self.pitch.to_le_bytes());
        out[2..4].copy_from_slice(&self.roll.to_le_bytes());
        out[4..6].copy_from_slice(&self.wheel_speed.to_le_bytes());
        out[6] = self.status;
        Self::LEN
    }

    /// Decode the committed prefix; ignore trailing bytes.
    pub fn decode(b: &[u8]) -> Result<CyclicState, PayloadError> {
        if b.len() < Self::LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(CyclicState {
            pitch: rd_i16(b, 0),
            roll: rd_i16(b, 2),
            wheel_speed: rd_i16(b, 4),
            status: b[6],
        })
    }
}

// --- DriveCmd (fully committed) ----------------------------------------------------------------

/// Drive command payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriveCmd {
    pub kind: u8,
    pub value: i16,
    pub steer: i16,
}

impl DriveCmd {
    pub const LEN: usize = 5;

    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0] = self.kind;
        out[1..3].copy_from_slice(&self.value.to_le_bytes());
        out[3..5].copy_from_slice(&self.steer.to_le_bytes());
        Self::LEN
    }

    pub fn decode(b: &[u8]) -> Result<DriveCmd, PayloadError> {
        if b.len() < Self::LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(DriveCmd { kind: b[0], value: rd_i16(b, 1), steer: rd_i16(b, 3) })
    }
}

// --- NodeHello (committed in shape) ------------------------------------------------------------

/// Topology announcement. `caps` is the `ItemSet` bitmask of produced+consumed items.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeHello {
    pub node_id: u8,
    pub role: u8,
    pub motor_count: u8,
    pub proto_ver: u8,
    pub fw_ver: u16,
    pub caps: u16,
}

impl NodeHello {
    pub const LEN: usize = 8;

    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0] = self.node_id;
        out[1] = self.role;
        out[2] = self.motor_count;
        out[3] = self.proto_ver;
        out[4..6].copy_from_slice(&self.fw_ver.to_le_bytes());
        out[6..8].copy_from_slice(&self.caps.to_le_bytes());
        Self::LEN
    }

    pub fn decode(b: &[u8]) -> Result<NodeHello, PayloadError> {
        if b.len() < Self::LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(NodeHello {
            node_id: b[0],
            role: b[1],
            motor_count: b[2],
            proto_ver: b[3],
            fw_ver: rd_u16(b, 4),
            caps: rd_u16(b, 6),
        })
    }
}

// --- Telemetry (committed in shape) ------------------------------------------------------------

/// Per-motor telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Telemetry {
    pub motor_index: u8,
    pub battery_mv: u16,
    pub current_ca: i16,
    pub speed: i16,
    pub fault_code: u8,
    pub flags: u8,
}

impl Telemetry {
    pub const LEN: usize = 9;

    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0] = self.motor_index;
        out[1..3].copy_from_slice(&self.battery_mv.to_le_bytes());
        out[3..5].copy_from_slice(&self.current_ca.to_le_bytes());
        out[5..7].copy_from_slice(&self.speed.to_le_bytes());
        out[7] = self.fault_code;
        out[8] = self.flags;
        Self::LEN
    }

    pub fn decode(b: &[u8]) -> Result<Telemetry, PayloadError> {
        if b.len() < Self::LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(Telemetry {
            motor_index: b[0],
            battery_mv: rd_u16(b, 1),
            current_ca: rd_i16(b, 3),
            speed: rd_i16(b, 5),
            fault_code: b[7],
            flags: b[8],
        })
    }
}

// --- ConfigRead (committed in shape) -----------------------------------------------------------

/// Read a config register by key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigRead {
    pub key: u16,
}

impl ConfigRead {
    pub const LEN: usize = 2;

    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0..2].copy_from_slice(&self.key.to_le_bytes());
        Self::LEN
    }

    pub fn decode(b: &[u8]) -> Result<ConfigRead, PayloadError> {
        if b.len() < Self::LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(ConfigRead { key: rd_u16(b, 0) })
    }
}

// --- Fault (committed in shape) ----------------------------------------------------------------

/// A fault report / command. `action` carries e.g. stop-all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fault {
    pub fault_code: u8,
    pub action: u8,
}

impl Fault {
    pub const LEN: usize = 2;

    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0] = self.fault_code;
        out[1] = self.action;
        Self::LEN
    }

    pub fn decode(b: &[u8]) -> Result<Fault, PayloadError> {
        if b.len() < Self::LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(Fault { fault_code: b[0], action: b[1] })
    }
}

// --- Inputs (committed in shape) ---------------------------------------------------------------

/// Rider input snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inputs {
    pub throttle: i16,
    pub buttons: u8,
    pub rider: u8,
}

impl Inputs {
    pub const LEN: usize = 4;

    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0..2].copy_from_slice(&self.throttle.to_le_bytes());
        out[2] = self.buttons;
        out[3] = self.rider;
        Self::LEN
    }

    pub fn decode(b: &[u8]) -> Result<Inputs, PayloadError> {
        if b.len() < Self::LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(Inputs { throttle: rd_i16(b, 0), buttons: b[2], rider: b[3] })
    }
}

// --- Borrow-carrying payloads (decode returns the trailing slice) -------------------------------

/// Write a config register: a key and a variable-length value blob.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigWrite<'a> {
    pub key: u16,
    pub value: &'a [u8],
}

impl<'a> ConfigWrite<'a> {
    /// Committed leading length (the key); `value` is everything after it.
    pub const HEAD_LEN: usize = 2;

    pub fn decode(b: &'a [u8]) -> Result<ConfigWrite<'a>, PayloadError> {
        if b.len() < Self::HEAD_LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(ConfigWrite { key: rd_u16(b, 0), value: &b[Self::HEAD_LEN..] })
    }
}

/// Config read/write response: a key, a status byte, and a variable-length value blob.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigResp<'a> {
    pub key: u16,
    pub status: u8,
    pub value: &'a [u8],
}

impl<'a> ConfigResp<'a> {
    pub const HEAD_LEN: usize = 3;

    pub fn decode(b: &'a [u8]) -> Result<ConfigResp<'a>, PayloadError> {
        if b.len() < Self::HEAD_LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(ConfigResp { key: rd_u16(b, 0), status: b[2], value: &b[Self::HEAD_LEN..] })
    }
}

/// A generic event: an id and a variable-length argument blob.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Event<'a> {
    pub event_id: u8,
    pub args: &'a [u8],
}

impl<'a> Event<'a> {
    pub const HEAD_LEN: usize = 1;

    pub fn decode(b: &'a [u8]) -> Result<Event<'a>, PayloadError> {
        if b.len() < Self::HEAD_LEN {
            return Err(PayloadError::TooShort);
        }
        Ok(Event { event_id: b[0], args: &b[Self::HEAD_LEN..] })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cyclic_state_round_trip() {
        let s = CyclicState { pitch: -1234, roll: 567, wheel_speed: -8, status: 0b1011 };
        let mut buf = [0u8; 16];
        let n = s.encode(&mut buf);
        assert_eq!(n, CyclicState::LEN);
        assert_eq!(CyclicState::decode(&buf[..n]).unwrap(), s);
    }

    #[test]
    fn drive_cmd_round_trip() {
        let c = DriveCmd { kind: 2, value: -300, steer: 75 };
        let mut buf = [0u8; 16];
        let n = c.encode(&mut buf);
        assert_eq!(DriveCmd::decode(&buf[..n]).unwrap(), c);
    }

    #[test]
    fn node_hello_round_trip() {
        let h = NodeHello {
            node_id: 7,
            role: 1,
            motor_count: 2,
            proto_ver: 1,
            fw_ver: 0x0102,
            caps: 0x002D,
        };
        let mut buf = [0u8; 16];
        let n = h.encode(&mut buf);
        assert_eq!(NodeHello::decode(&buf[..n]).unwrap(), h);
    }

    #[test]
    fn telemetry_round_trip() {
        let t = Telemetry {
            motor_index: 1,
            battery_mv: 36000,
            current_ca: -250,
            speed: 4096,
            fault_code: 0,
            flags: 0x80,
        };
        let mut buf = [0u8; 16];
        let n = t.encode(&mut buf);
        assert_eq!(Telemetry::decode(&buf[..n]).unwrap(), t);
    }

    #[test]
    fn config_read_inputs_fault_round_trip() {
        let cr = ConfigRead { key: 0xBEEF };
        let mut buf = [0u8; 8];
        let n = cr.encode(&mut buf);
        assert_eq!(ConfigRead::decode(&buf[..n]).unwrap(), cr);

        let inp = Inputs { throttle: -1000, buttons: 0x0F, rider: 1 };
        let n = inp.encode(&mut buf);
        assert_eq!(Inputs::decode(&buf[..n]).unwrap(), inp);

        let f = Fault { fault_code: 5, action: 1 };
        let n = f.encode(&mut buf);
        assert_eq!(Fault::decode(&buf[..n]).unwrap(), f);
    }

    #[test]
    fn trailing_bytes_ignored() {
        // NodeHello with 4 extra trailing bytes: committed fields decode, extras ignored.
        let h = NodeHello {
            node_id: 9, role: 0, motor_count: 1, proto_ver: 1, fw_ver: 0x1234, caps: 0x000A,
        };
        let mut buf = [0u8; 16];
        let n = h.encode(&mut buf);
        buf[n] = 0xAA;
        buf[n + 1] = 0xBB;
        buf[n + 2] = 0xCC;
        buf[n + 3] = 0xDD;
        assert_eq!(NodeHello::decode(&buf[..n + 4]).unwrap(), h);

        // Telemetry likewise.
        let t = Telemetry {
            motor_index: 0, battery_mv: 100, current_ca: 1, speed: 2, fault_code: 0, flags: 0,
        };
        let n = t.encode(&mut buf);
        assert_eq!(Telemetry::decode(&buf[..n + 3]).unwrap(), t);
    }

    #[test]
    fn borrow_carrying_decode() {
        let bytes = [0x34, 0x12, 0xDE, 0xAD, 0xBE]; // key=0x1234, value=[DE AD BE]
        let cw = ConfigWrite::decode(&bytes).unwrap();
        assert_eq!(cw.key, 0x1234);
        assert_eq!(cw.value, &[0xDE, 0xAD, 0xBE]);

        let resp_bytes = [0x34, 0x12, 0x00, 0x01, 0x02]; // key, status=0, value=[01 02]
        let cr = ConfigResp::decode(&resp_bytes).unwrap();
        assert_eq!(cr.key, 0x1234);
        assert_eq!(cr.status, 0);
        assert_eq!(cr.value, &[0x01, 0x02]);

        let ev_bytes = [0x07, 0xAA, 0xBB]; // event 7, args [AA BB]
        let ev = Event::decode(&ev_bytes).unwrap();
        assert_eq!(ev.event_id, 7);
        assert_eq!(ev.args, &[0xAA, 0xBB]);
    }

    #[test]
    fn short_payloads_rejected() {
        assert_eq!(CyclicState::decode(&[0, 1, 2]), Err(PayloadError::TooShort));
        assert_eq!(ConfigWrite::decode(&[0]), Err(PayloadError::TooShort));
    }
}
