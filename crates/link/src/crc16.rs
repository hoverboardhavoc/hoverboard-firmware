//! Shared CRC-16/MODBUS helper (Phase 0).
//!
//! One CRC implementation that the link frame, the board-definition blob, and (by agreement) the
//! MCU descriptor all use, so a checksum computed anywhere is byte-for-byte identical. This wraps
//! the exact `crc` crate config `runtime-hal/src/parse.rs::payload_crc` uses
//! (`crc::Crc::<u16>::new(&crc::CRC_16_MODBUS)`): reflected poly 0xA001, init 0xFFFF, no final xor.

/// CRC-16/MODBUS over `bytes`. Identical to runtime-hal's `payload_crc`.
#[inline]
pub fn modbus(bytes: &[u8]) -> u16 {
    crc::Crc::<u16>::new(&crc::CRC_16_MODBUS).checksum(bytes)
}

#[cfg(test)]
mod tests {
    use super::modbus;

    // Phase 0 golden vector. CRC-16/MODBUS has a published check value of 0x4B37 for the ASCII
    // string "123456789". runtime-hal computes the same value with the same `crc` config
    // (crc::CRC_16_MODBUS), so this single constant pins our checksum byte-for-byte to runtime-hal's
    // `payload_crc`. (runtime-hal's parse tests compute the CRC inline rather than freezing a
    // literal, so we freeze the standard check value here.)
    #[test]
    fn golden_check_value() {
        assert_eq!(modbus(b"123456789"), 0x4B37);
    }

    // A second frozen pair, computed once with crc::CRC_16_MODBUS, to guard against a config drift
    // (e.g. a different init or reflect setting) that the check value alone might not catch.
    #[test]
    fn golden_frozen_pair() {
        // bytes: 0x01 0x02 0x03 0x04 0x05 -> 0xBB2A (CRC-16/MODBUS)
        assert_eq!(modbus(&[0x01, 0x02, 0x03, 0x04, 0x05]), 0xBB2A);
    }

    // Empty input is the init value (0xFFFF) reflected through with no bytes consumed.
    #[test]
    fn empty_is_init() {
        assert_eq!(modbus(&[]), 0xFFFF);
    }
}
