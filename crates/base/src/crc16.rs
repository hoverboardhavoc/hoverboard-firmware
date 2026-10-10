//! Shared CRC-16/MODBUS helper.
//!
//! One CRC implementation that the config-store records and the link frames both use, so a
//! checksum computed anywhere is byte-for-byte identical. It computes the exact algorithm
//! runtime-hal's `parse.rs::payload_crc` computes (`crc::Crc::<u16>::new(&crc::CRC_16_MODBUS)`):
//! reflected poly 0xA001, init 0xFFFF, no final xor; little-endian on the wire.
//!
//! Two forms are provided:
//! - [`modbus`], the one-shot over a contiguous slice.
//! - [`Crc16`], the incremental form for the store's header-then-value records and the link framer,
//!   which feed the CRC in pieces. Both produce identical results for the same byte sequence.
//!
//! **Why this is hand-written rather than the `crc` crate.** The crate's default implementation is
//! table-driven over a 256-entry `[u16; 256]`, which is 512 B of rodata in an image whose span is
//! the nearer wall (`specs/decision-flash-budget.md`). Its bitwise `NoTable` form drops the table
//! but costs eight shift-xor steps per byte, and this CRC sits in the link framer, inside the
//! cooperative main loop whose collapse the bounded drain exists to prevent. The nibble form below
//! is the middle: a 16-entry (32 B) table, two lookups per byte, and the same four golden vectors
//! the crate's config was pinned against. `base` therefore carries no CRC dependency at all.

/// The reflected CRC-16/MODBUS polynomial: 0xA001 is the bit reversal of 0x8005, which is what
/// makes the shift-right form below compute the reflected algorithm (`refin`/`refout` both true)
/// without reversing anything per byte.
const POLY: u16 = 0xA001;

/// The nibble table: `NIBBLE[i]` is the reflected remainder of the four bits of `i`, 32 B of
/// rodata. Derived from [`POLY`] at compile time rather than written out, so the table cannot
/// drift from the polynomial it is a table of.
const NIBBLE: [u16; 16] = {
    let mut table = [0u16; 16];
    let mut i = 0usize;
    while i < 16 {
        let mut v = i as u16;
        let mut bit = 0;
        while bit < 4 {
            v = if v & 1 != 0 { (v >> 1) ^ POLY } else { v >> 1 };
            bit += 1;
        }
        table[i] = v;
        i += 1;
    }
    table
};

/// Fold one byte into a running remainder, low nibble first (the reflected bit order).
#[inline]
const fn feed(crc: u16, byte: u8) -> u16 {
    let crc = (crc >> 4) ^ NIBBLE[((crc ^ byte as u16) & 0x0F) as usize];
    (crc >> 4) ^ NIBBLE[((crc ^ (byte as u16 >> 4)) & 0x0F) as usize]
}

/// Fold a whole slice into a running remainder: the ONE copy of the loop in the image.
///
/// `#[inline(never)]` for the same reason `fixed::div` carries it (`specs/decision-flash-budget.md`):
/// `base` is compiled at opt-level 3 in the shipping profile, so left inlinable this loop is
/// unrolled and vectorised separately at every CRC site in the link framer, the store's records and
/// the walk payloads. One shared body is the whole saving; the call costs a handful of cycles and
/// leaves the per-byte work (two table lookups) unchanged.
///
/// It belongs in the GD32F1x0's zero-wait first 32 KiB, and `crates/firmware/memory.x` puts it
/// there by NAME (`*(.text.*4base5crc16*)`), not by a `link_section` attribute here. Before this
/// was outlined, the link framer's CRC was inlined into `link` and that script's
/// `*(.text.*_4link*)` anchor placed it in the window; a shared body above the boundary would
/// refetch the per-byte loop at 2 wait states with no prefetch or cache, every iteration.
/// `NIBBLE` needs no placement: the same script puts all of `.rodata` in that window anyway.
///
/// The attribute is the mechanism that script PREFERS, because it travels with the item instead of
/// going stale on a rename, and it is what `fixed::div` carries. It cannot be used here: `base` is
/// linked by the bench and emulator images too (`crates/store-test`, `crates/l2-uart-bench`, ...),
/// whose own `memory.x` files declare no `.hotcode` output section, so the attribute makes
/// `.hotcode` an orphan section in each of them. That still LINKS, and the store-test image then
/// fails under the emulator with every scenario reading a zero result word.
#[inline(never)]
fn fold(mut crc: u16, bytes: &[u8]) -> u16 {
    for &b in bytes {
        crc = feed(crc, b);
    }
    crc
}

/// The MODBUS init value: the remainder a fresh accumulator starts from, and the value an empty
/// input hashes to (there is no final xor).
const INIT: u16 = 0xFFFF;

/// CRC-16/MODBUS over `bytes`. Identical to runtime-hal's `payload_crc`.
#[inline]
pub fn modbus(bytes: &[u8]) -> u16 {
    fold(INIT, bytes)
}

/// Incremental CRC-16/MODBUS, for callers that build the input in pieces (the config store's
/// header-then-value records, the link stream framer). `Crc16::new()` then any number of
/// [`update`](Crc16::update) calls then [`finish`](Crc16::finish) yields the same value as
/// [`modbus`] over the concatenated input.
pub struct Crc16 {
    crc: u16,
}

impl Crc16 {
    /// A fresh CRC accumulator seeded with the MODBUS init value (0xFFFF).
    #[inline]
    pub fn new() -> Self {
        Self { crc: INIT }
    }

    /// Feed more bytes into the running CRC.
    #[inline]
    pub fn update(&mut self, bytes: &[u8]) {
        self.crc = fold(self.crc, bytes);
    }

    /// Consume the accumulator and return the final CRC-16/MODBUS value (no final xor).
    #[inline]
    pub fn finish(self) -> u16 {
        self.crc
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

    // The nibble table IS the polynomial: every entry is the four-bit reflected remainder of its
    // index, recomputed here bit by bit. A mistyped or mis-derived table would still pass a
    // one-shot vector by accident far more easily than it passes all sixteen entries.
    #[test]
    fn nibble_table_is_the_polynomial() {
        for (i, &entry) in super::NIBBLE.iter().enumerate() {
            let mut v = i as u16;
            for _ in 0..4 {
                v = if v & 1 != 0 {
                    (v >> 1) ^ super::POLY
                } else {
                    v >> 1
                };
            }
            assert_eq!(entry, v, "NIBBLE[{i}]");
        }
    }

    // Every single byte agrees with the bitwise definition of the algorithm (eight shift-xor steps
    // from the init value), so the two-lookups-per-byte folding is pinned over the whole input
    // alphabet rather than at the four golden vectors alone.
    #[test]
    fn every_byte_matches_the_bitwise_form() {
        for b in 0u16..=255 {
            let mut v = 0xFFFFu16 ^ b;
            for _ in 0..8 {
                v = if v & 1 != 0 {
                    (v >> 1) ^ super::POLY
                } else {
                    v >> 1
                };
            }
            assert_eq!(modbus(&[b as u8]), v, "byte {b:#04x}");
        }
    }
}
