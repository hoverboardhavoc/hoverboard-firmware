//! Shared CRC-16/MODBUS helper.
//!
//! One CRC implementation that the config-store records and the link frames both use, so a
//! checksum computed anywhere is byte-for-byte identical. This wraps the exact `crc` crate config
//! that runtime-hal's `parse.rs::payload_crc` uses (`crc::Crc::<u16>::new(&crc::CRC_16_MODBUS)`):
//! reflected poly 0xA001, init 0xFFFF, no final xor; little-endian on the wire.
//!
//! Two forms are provided:
//! - [`modbus`], the one-shot over a contiguous slice.
//! - [`Crc16`], the incremental form for the store's header-then-value records and the link framer,
//!   which feed the CRC in pieces. Both produce identical results for the same byte sequence.

use crc::{Crc, Digest, CRC_16_MODBUS};

/// The shared CRC-16/MODBUS algorithm instance. Both [`modbus`] and [`Crc16`] use this, so the two
/// forms cannot drift from each other.
const CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

/// CRC-16/MODBUS over `bytes`. Identical to runtime-hal's `payload_crc`.
#[inline]
pub fn modbus(bytes: &[u8]) -> u16 {
    CRC.checksum(bytes)
}

/// Incremental CRC-16/MODBUS, for callers that build the input in pieces (the config store's
/// header-then-value records, the link stream framer). `Crc16::new()` then any number of
/// [`update`](Crc16::update) calls then [`finish`](Crc16::finish) yields the same value as
/// [`modbus`] over the concatenated input.
pub struct Crc16 {
    digest: Digest<'static, u16>,
}

impl Crc16 {
    /// A fresh CRC accumulator seeded with the MODBUS init value (0xFFFF).
    #[inline]
    pub fn new() -> Self {
        Self {
            digest: CRC.digest(),
        }
    }

    /// Feed more bytes into the running CRC.
    #[inline]
    pub fn update(&mut self, bytes: &[u8]) {
        self.digest.update(bytes);
    }

    /// Consume the accumulator and return the final CRC-16/MODBUS value (no final xor).
    #[inline]
    pub fn finish(self) -> u16 {
        self.digest.finalize()
    }
}

impl Default for Crc16 {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::{modbus, Crc16};

    // The published CRC-16/MODBUS check value: 0x4B37 for the ASCII string "123456789". This pins
    // our checksum byte-for-byte to runtime-hal's `payload_crc` (same `crc` config).
    #[test]
    fn golden_check_value() {
        assert_eq!(modbus(b"123456789"), 0x4B37);
    }

    // A second frozen pair, to catch a config drift (a different init or reflect setting) that the
    // check value alone might miss. Computed once with crc::CRC_16_MODBUS.
    #[test]
    fn golden_frozen_pair() {
        // 0x01 0x02 0x03 0x04 0x05 -> 0xBB2A (CRC-16/MODBUS)
        assert_eq!(modbus(&[0x01, 0x02, 0x03, 0x04, 0x05]), 0xBB2A);
    }

    // Empty input is the init value (0xFFFF), no bytes consumed.
    #[test]
    fn empty_is_init() {
        assert_eq!(modbus(&[]), 0xFFFF);
    }

    // A frame-sized vector (a plausible link header: SOF, ver, opcode, src, dst, len).
    #[test]
    fn golden_frame_header() {
        // 0xAA 0x01 0x10 0x00 0x01 0x04 -> 0x4221 (CRC-16/MODBUS)
        assert_eq!(modbus(&[0xAA, 0x01, 0x10, 0x00, 0x01, 0x04]), 0x4221);
    }

    // The incremental form must agree with the one-shot for the same byte sequence.
    #[test]
    fn incremental_matches_oneshot() {
        let mut c = Crc16::new();
        c.update(b"1234");
        c.update(b"5678");
        c.update(b"9");
        let got = c.finish();
        assert_eq!(got, modbus(b"123456789"));
        assert_eq!(got, 0x4B37);
    }

    // A fresh incremental accumulator with no updates returns the init value, matching empty input.
    #[test]
    fn incremental_empty_is_init() {
        let c = Crc16::new();
        let got = c.finish();
        assert_eq!(got, modbus(&[]));
        assert_eq!(got, 0xFFFF);
    }

    // Default mirrors new().
    #[test]
    fn incremental_default_matches_new() {
        let mut a = Crc16::new();
        let mut b = Crc16::default();
        a.update(&[0x01, 0x02, 0x03, 0x04, 0x05]);
        b.update(&[0x01, 0x02, 0x03, 0x04, 0x05]);
        assert_eq!(a.finish(), b.finish());
    }
}
