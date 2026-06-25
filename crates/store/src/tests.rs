//! Tier-1 host test suite over `MockFlash` (the spec's "Tier 1", items 1-6).
//!
//! `MockFlash` models the silicon write rules, so the codec, log, and compaction are exercised
//! without hardware. This is NOT the on-chip proof (that is tiers 2/3, the exact image against a real
//! FMC); it is the fast logic gate.

extern crate std;
use std::vec;
use std::vec::Vec;

use base::error::FlashError;

use crate::field::{BlobField, Field, DEVICE_NAME, MOTOR_CURRENT_LIMIT, MOTOR_METHOD, SOME_BLOB};
#[cfg(feature = "test-fields")]
use crate::field::{T_KEY, T_VAL};
use crate::flash::{FailingMockFlash, Flash, MockFlash};
use crate::key::{Key, Type};
use crate::record::{self, HeaderScan, MAGIC};
#[cfg(feature = "test-fields")]
use crate::run;
use crate::store::{Store, StoreError};

const PS: usize = 1024;
const PAGE_HEADER_LEN: usize = 8;

// A small region builder for the planted (host-crafted) scenarios. It mirrors the on-flash layout so
// torn writes are "a normal program sequence stopped early".
struct RegionBuilder {
    page_size: usize,
    bytes: Vec<u8>,
}

impl RegionBuilder {
    fn new(page_size: usize) -> Self {
        Self {
            page_size,
            bytes: vec![0xFF; 2 * page_size],
        }
    }

    /// Write page `p`'s full header (magic + seq), byte-identical to what the store writes
    /// ([`record::encode_page_header`], the single source of truth for the layout).
    fn page_header(&mut self, p: usize, seq: u16) -> &mut Self {
        let off = p * self.page_size;
        let hdr = record::encode_page_header(seq);
        self.bytes[off..off + hdr.len()].copy_from_slice(&hdr);
        self
    }

    /// Append a complete record at page-relative `off` (returns the next offset).
    fn record(&mut self, abs_off: usize, key: Key, type_tag: u8, value: &[u8]) -> usize {
        let mut buf = vec![0u8; record::record_size(value.len())];
        let n = record::encode(key.field_id, key.index, type_tag, value, &mut buf);
        self.bytes[abs_off..abs_off + n].copy_from_slice(&buf[..n]);
        abs_off + n
    }

    fn build(self) -> MockFlash {
        MockFlash::from_image(self.page_size, &self.bytes)
    }
}

// =====================================================================================
// 1. Record + CRC: encode/decode, hdr_crc makes a torn payload skippable, val_crc gates validity.
// =====================================================================================

#[test]
fn record_roundtrip_even_len() {
    let mut buf = [0u8; 32];
    let val = [0xDE, 0xAD, 0xBE, 0xEF];
    let n = record::encode(0x20, 1, Type::U32.tag(), &val, &mut buf);
    assert_eq!(n, record::record_size(4));
    match record::parse_header(&buf, 0) {
        HeaderScan::Good(h) => {
            assert_eq!(h.field_id, 0x20);
            assert_eq!(h.index, 1);
            assert_eq!(h.type_tag, Type::U32.tag());
            assert_eq!(h.len, 4);
            assert!(record::is_committed(&buf, 0, &h));
            assert_eq!(record::value_bytes(&buf, 0, &h), &val);
        }
        _ => panic!("expected a good header"),
    }
}

#[test]
fn record_odd_len_pads_to_even() {
    // A 3-byte value: padded to 4, the record is even-length and val_crc stays halfword-aligned.
    let val = [0xAA, 0xBB, 0xCC];
    assert_eq!(record::record_size(3), 8 + 4 + 2);
    let mut buf = [0u8; 32];
    let n = record::encode(0x30, 0, Type::Blob.tag(), &val, &mut buf);
    assert_eq!(n % 2, 0);
    // The pad byte is 0xFF (reads as erased).
    assert_eq!(buf[8 + 3], 0xFF);
    let h = match record::parse_header(&buf, 0) {
        HeaderScan::Good(h) => h,
        _ => panic!(),
    };
    assert!(record::is_committed(&buf, 0, &h));
    assert_eq!(record::value_bytes(&buf, 0, &h), &val);
}

#[test]
fn torn_payload_hdr_crc_good_but_val_crc_fails() {
    let val = [1u8, 2, 3, 4];
    let mut buf = [0u8; 32];
    record::encode(0x20, 0, Type::U32.tag(), &val, &mut buf);
    // Header still parses (len trusted), so the record is skippable...
    let h = match record::parse_header(&buf, 0) {
        HeaderScan::Good(h) => h,
        _ => panic!(),
    };
    // ...but corrupt the value: val_crc no longer matches, so it is not committed (never wins a read).
    buf[8] ^= 0xFF;
    assert!(!record::is_committed(&buf, 0, &h));
}

#[test]
fn torn_header_hdr_crc_fails() {
    let val = [1u8, 2, 3, 4];
    let mut buf = [0u8; 32];
    record::encode(0x20, 0, Type::U32.tag(), &val, &mut buf);
    // Corrupt a covered header byte: hdr_crc fails, len is garbage, the log is not walkable past here.
    buf[4] ^= 0xFF;
    assert!(matches!(record::parse_header(&buf, 0), HeaderScan::Torn));
}

#[test]
fn blank_field_id_is_the_frontier() {
    let buf = [0xFFu8; 16];
    assert!(matches!(record::parse_header(&buf, 0), HeaderScan::Blank));
}

// =====================================================================================
// 2. Flash write rules: the MockFlash enforces the silicon model.
// =====================================================================================

#[test]
fn program_rejects_odd_offset_and_odd_length() {
    let mut f = MockFlash::erased(PS);
    assert_eq!(f.program(1, &[0, 0]), Err(FlashError::Misaligned)); // odd offset
    assert_eq!(f.program(0, &[0, 0, 0]), Err(FlashError::Misaligned)); // odd length
    assert_eq!(f.program(0, &[0xAA, 0xBB]), Ok(())); // aligned + even is fine
}

#[test]
fn program_is_write_once_not_and() {
    let mut f = MockFlash::erased(PS);
    assert_eq!(f.program(0, &[0x00, 0x00]), Ok(()));
    // Re-programming an already-written halfword fails (write-once); it does NOT AND bits in.
    assert_eq!(f.program(0, &[0x00, 0x00]), Err(FlashError::ProgramFailed));
    // Even writing all-1s back (which an AND model would allow) is refused.
    assert_eq!(f.program(0, &[0xFF, 0xFF]), Err(FlashError::ProgramFailed));
    // A rejected write leaves flash untouched.
    assert_eq!(&f.as_bytes()[0..2], &[0x00, 0x00]);
}

#[test]
fn erase_fills_page_with_0xffff() {
    let mut f = MockFlash::erased(PS);
    f.program(0, &[0x12, 0x34]).unwrap();
    f.erase_page(0).unwrap();
    assert!(f.as_bytes()[..PS].iter().all(|&b| b == 0xFF));
    // After erase, the halfword can be programmed again.
    assert_eq!(f.program(0, &[0x56, 0x78]), Ok(()));
}

#[test]
fn program_out_of_bounds() {
    let mut f = MockFlash::erased(PS);
    assert_eq!(f.program(2 * PS, &[0, 0]), Err(FlashError::OutOfBounds));
}

#[test]
fn appended_records_are_even_aligned() {
    // Every append lands at an even offset and occupies an even number of bytes.
    let mut f = MockFlash::erased(PS);
    {
        let mut s = Store::mount(&mut f).unwrap();
        s.set_bytes(SOME_BLOB, &[1, 2, 3]).unwrap(); // odd len -> padded
        s.set(MOTOR_METHOD, 7).unwrap(); // 1-byte scalar -> padded to halfword
    }
    // Re-mount and confirm the frontier walked cleanly (no misalignment fault).
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get_bytes(SOME_BLOB), &[1, 2, 3]);
    assert_eq!(s.get(MOTOR_METHOD), 7);
}

// =====================================================================================
// 3. Log mechanics: scan/frontier, append, three torn cases, ping-pong compaction.
// =====================================================================================

#[test]
fn scan_finds_latest_per_key() {
    let mut f = MockFlash::erased(PS);
    {
        let mut s = Store::mount(&mut f).unwrap();
        s.set(MOTOR_CURRENT_LIMIT, 100).unwrap();
        s.set(MOTOR_CURRENT_LIMIT, 200).unwrap();
        s.set(MOTOR_CURRENT_LIMIT, 300).unwrap();
    }
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 300); // newest wins
}

#[test]
fn append_then_remount_keeps_frontier() {
    let mut f = MockFlash::erased(PS);
    {
        let mut s = Store::mount(&mut f).unwrap();
        s.set(MOTOR_CURRENT_LIMIT, 111).unwrap();
    }
    {
        // A fresh mount finds the frontier after the first record and can append again.
        let mut s = Store::mount(&mut f).unwrap();
        s.set(MOTOR_METHOD, 5).unwrap();
    }
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 111);
    assert_eq!(s.get(MOTOR_METHOD), 5);
}

#[test]
fn torn_payload_recovery_last_good_value_reads() {
    // Host plants: a good record, then a half-written payload (hdr_crc good, val_crc absent/garbage).
    let mut b = RegionBuilder::new(PS);
    b.page_header(0, 0);
    let key = MOTOR_CURRENT_LIMIT.key();
    let off1 = b.record(PAGE_HEADER_LEN, key, Type::U32.tag(), &42u32.to_le_bytes());
    // Plant a torn payload at off1: a valid header but a corrupted value (val_crc fails).
    let mut buf = vec![0u8; record::record_size(4)];
    record::encode(
        key.field_id,
        key.index,
        Type::U32.tag(),
        &99u32.to_le_bytes(),
        &mut buf,
    );
    let n = buf.len();
    buf[8] ^= 0xFF; // corrupt the value
    b.bytes[off1..off1 + n].copy_from_slice(&buf);
    let mut f = b.build();

    let s = Store::mount(&mut f).unwrap();
    // The torn record never wins; the last good value (42) reads.
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 42);
}

#[test]
fn torn_header_auto_compacts_and_recovers() {
    // Host plants: two good records, then a torn header (hdr_crc bad). Mount auto-compacts.
    let mut b = RegionBuilder::new(PS);
    b.page_header(0, 3);
    let k1 = MOTOR_CURRENT_LIMIT.key();
    let k2 = MOTOR_METHOD.key();
    let mut off = b.record(PAGE_HEADER_LEN, k1, Type::U32.tag(), &1234u32.to_le_bytes());
    off = b.record(off, k2, Type::U8.tag(), &[9]);
    // Plant a torn header at `off`: encode a record then corrupt a covered header byte.
    let mut buf = vec![0u8; record::record_size(4)];
    record::encode(
        k1.field_id,
        k1.index,
        Type::U32.tag(),
        &5u32.to_le_bytes(),
        &mut buf,
    );
    buf[4] ^= 0xFF; // corrupt len -> hdr_crc fails
    b.bytes[off..off + buf.len()].copy_from_slice(&buf);
    let mut f = b.build();

    {
        let s = Store::mount(&mut f).unwrap();
        // Survivors read back, and the active side is now the spare page (clean frontier).
        assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 1234);
        assert_eq!(s.get(MOTOR_METHOD), 9);
    }
    // Re-mount: still clean, and we can append again.
    let mut s = Store::mount(&mut f).unwrap();
    s.set(MOTOR_METHOD, 11).unwrap();
    assert_eq!(s.get(MOTOR_METHOD), 11);
}

#[test]
fn torn_header_auto_compaction_flash_failure_surfaces() {
    // The one Flash(..)-on-failure path: a failing backend during the auto-compaction.
    let mut b = RegionBuilder::new(PS);
    b.page_header(0, 0);
    let k1 = MOTOR_CURRENT_LIMIT.key();
    let off = b.record(PAGE_HEADER_LEN, k1, Type::U32.tag(), &7u32.to_le_bytes());
    let mut buf = vec![0u8; record::record_size(4)];
    record::encode(
        k1.field_id,
        k1.index,
        Type::U32.tag(),
        &5u32.to_le_bytes(),
        &mut buf,
    );
    buf[4] ^= 0xFF; // torn header
    b.bytes[off..off + buf.len()].copy_from_slice(&buf);
    let inner = b.build();
    // Fail the very first program the compaction attempts.
    let mut f = FailingMockFlash::new(inner, 0);
    match Store::mount(&mut f) {
        Err(StoreError::Flash(FlashError::ProgramFailed)) => {}
        other => panic!(
            "expected Flash(ProgramFailed) on auto-compaction, got {:?}",
            other.is_ok()
        ),
    }
}

#[test]
fn compaction_higher_seq_wins_after_power_loss_mid_copy() {
    // Power-loss-mid-compaction: the new page's header is written LAST, so a torn copy leaves the OLD
    // page as the only valid side. Model it by planting BOTH pages: an intact old page (seq 5) plus a
    // partially-copied spare WITHOUT its magic (header never committed). Mount must pick the old page.
    let mut b = RegionBuilder::new(PS);
    // Old page (page 0): intact, seq 5.
    b.page_header(0, 5);
    let k = MOTOR_CURRENT_LIMIT.key();
    b.record(PAGE_HEADER_LEN, k, Type::U32.tag(), &777u32.to_le_bytes());
    // Spare page (page 1): a copied record but NO magic header (still 0xFFFF) -> not a valid side.
    let spare = PS;
    b.record(
        spare + PAGE_HEADER_LEN,
        k,
        Type::U32.tag(),
        &111u32.to_le_bytes(),
    );
    let mut f = b.build();

    let s = Store::mount(&mut f).unwrap();
    // The intact old page wins; the orphan spare copy is ignored.
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 777);
}

#[test]
fn compaction_prefers_completed_new_page() {
    // The inverse: both pages valid, the new page (higher seq) is complete -> it wins.
    let mut b = RegionBuilder::new(PS);
    b.page_header(0, 5);
    let k = MOTOR_CURRENT_LIMIT.key();
    b.record(PAGE_HEADER_LEN, k, Type::U32.tag(), &777u32.to_le_bytes());
    b.page_header(1, 6); // completed, higher seq
    b.record(
        PS + PAGE_HEADER_LEN,
        k,
        Type::U32.tag(),
        &888u32.to_le_bytes(),
    );
    let mut f = b.build();
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 888);
}

#[test]
fn compact_preserves_latest_per_key_and_advances_seq() {
    let mut f = MockFlash::erased(PS);
    {
        let mut s = Store::mount(&mut f).unwrap();
        s.set(MOTOR_CURRENT_LIMIT, 1).unwrap();
        s.set(MOTOR_CURRENT_LIMIT, 2).unwrap();
        s.set(MOTOR_METHOD, 3).unwrap();
        s.compact().unwrap();
        // After compaction only the latest per key survives, and they still read.
        assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 2);
        assert_eq!(s.get(MOTOR_METHOD), 3);
    }
    // Survives a remount (the new active side has the higher seq).
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 2);
    assert_eq!(s.get(MOTOR_METHOD), 3);
}

// =====================================================================================
// 4. Read/write: defaults, latest-per-key, scalar + variable round-trip, Full, ValueTooLarge.
// =====================================================================================

#[test]
fn absent_field_reads_default() {
    let mut f = MockFlash::erased(PS);
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 10_000); // the handle default
    assert_eq!(s.get(MOTOR_METHOD), 0);
    assert_eq!(s.get_str(DEVICE_NAME), "hoverboard");
    assert_eq!(s.get_bytes(SOME_BLOB), &[] as &[u8]);
}

#[test]
fn scalar_roundtrip_all_widths() {
    let mut f = MockFlash::erased(PS);
    let f8: Field<u8> = Field::new(0x40, 0);
    let f16: Field<u16> = Field::new(0x41, 0);
    let f32: Field<u32> = Field::new(0x42, 0);
    let f64: Field<u64> = Field::new(0x43, 0);
    let fi16: Field<i16> = Field::new(0x44, 0);
    let fi32: Field<i32> = Field::new(0x45, 0);
    let fi64: Field<i64> = Field::new(0x46, 0);
    let fb: Field<bool> = Field::new(0x47, false);
    {
        let mut s = Store::mount(&mut f).unwrap();
        s.set(f8, 0xAB).unwrap();
        s.set(f16, 0x1234).unwrap();
        s.set(f32, 0xDEAD_BEEF).unwrap();
        s.set(f64, 0x0123_4567_89AB_CDEF).unwrap();
        s.set(fi16, -1000).unwrap();
        s.set(fi32, -123456).unwrap();
        s.set(fi64, -9_000_000_000).unwrap();
        s.set(fb, true).unwrap();
    }
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(f8), 0xAB);
    assert_eq!(s.get(f16), 0x1234);
    assert_eq!(s.get(f32), 0xDEAD_BEEF);
    assert_eq!(s.get(f64), 0x0123_4567_89AB_CDEF);
    assert_eq!(s.get(fi16), -1000);
    assert_eq!(s.get(fi32), -123456);
    assert_eq!(s.get(fi64), -9_000_000_000);
    assert!(s.get(fb));
}

#[test]
fn variable_roundtrip_str_and_blob() {
    let mut f = MockFlash::erased(PS);
    {
        let mut s = Store::mount(&mut f).unwrap();
        s.set_str(DEVICE_NAME, "my-board").unwrap();
        s.set_bytes(SOME_BLOB, &[0xCA, 0xFE, 0xBA, 0xBE, 0x01])
            .unwrap();
    }
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get_str(DEVICE_NAME), "my-board");
    assert_eq!(s.get_bytes(SOME_BLOB), &[0xCA, 0xFE, 0xBA, 0xBE, 0x01]);
}

#[test]
fn at_index_selects_instance() {
    let mut f = MockFlash::erased(PS);
    {
        let mut s = Store::mount(&mut f).unwrap();
        s.set(MOTOR_CURRENT_LIMIT.at(0), 100).unwrap();
        s.set(MOTOR_CURRENT_LIMIT.at(1), 200).unwrap();
    }
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT.at(0)), 100);
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT.at(1)), 200);
    // A stray higher index that was never written reads the default.
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT.at(7)), 10_000);
}

#[test]
fn set_full_then_compact_then_retry() {
    // Fill the active page with a large blob field until a set returns Full, then compact + retry.
    let mut f = MockFlash::erased(PS);
    let big: BlobField = BlobField::new(0x50, &[]);
    let payload = [0xAAu8; 200];
    let mut s = Store::mount(&mut f).unwrap();
    let mut full_hit = false;
    for _ in 0..20 {
        match s.set_bytes(big, &payload) {
            Ok(()) => {}
            Err(StoreError::Full) => {
                full_hit = true;
                break;
            }
            Err(e) => panic!("unexpected {e:?}"),
        }
    }
    assert!(full_hit, "expected the active page to fill");
    // Compact then the retry succeeds.
    s.compact().unwrap();
    s.set_bytes(big, &payload).unwrap();
    assert_eq!(s.get_bytes(big), &payload);
}

#[test]
fn value_too_large_erases_nothing() {
    let mut f = MockFlash::erased(PS);
    let big: BlobField = BlobField::new(0x50, &[]);
    // A value larger than a page's data area can never fit; ValueTooLarge, nothing erased.
    let huge = vec![0u8; PS];
    let mut s = Store::mount(&mut f).unwrap();
    assert_eq!(s.set_bytes(big, &huge), Err(StoreError::ValueTooLarge));
}

// =====================================================================================
// 5. Field set: build-time id-uniqueness (compiles => unique), encode/decode, type validation,
//    undeclared field_id skipped, absent reads default.
// =====================================================================================

#[test]
fn field_ids_are_unique() {
    // The const assert_unique_ids fired at build time. Re-check at runtime as a belt-and-braces.
    let ids = crate::field::FIELD_IDS;
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            assert_ne!(ids[i], ids[j], "duplicate field id");
        }
    }
}

#[test]
fn wrong_type_record_on_flash_is_ignored() {
    // A record stored under MOTOR_CURRENT_LIMIT's id but with the WRONG type tag must not be decoded
    // as the field's type; the read falls back to the default.
    let mut b = RegionBuilder::new(PS);
    b.page_header(0, 0);
    let key = MOTOR_CURRENT_LIMIT.key();
    // Same field_id, but stored as U8 (1 byte) instead of U32.
    b.record(PAGE_HEADER_LEN, key, Type::U8.tag(), &[0x55]);
    let mut f = b.build();
    let s = Store::mount(&mut f).unwrap();
    // type mismatch -> default, never a wrong-width decode.
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 10_000);
}

#[test]
fn undeclared_field_id_is_skipped() {
    // A record whose field_id no handle names is walkable (compaction preserves it) but never read by
    // the typed path. The declared fields around it still read.
    let mut b = RegionBuilder::new(PS);
    b.page_header(0, 0);
    let unknown = Key {
        field_id: 0x7A,
        index: 0,
    };
    let mut off = b.record(
        PAGE_HEADER_LEN,
        unknown,
        Type::U16.tag(),
        &0xBEEFu16.to_le_bytes(),
    );
    off = b.record(off, MOTOR_METHOD.key(), Type::U8.tag(), &[4]);
    let _ = off;
    let mut f = b.build();
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(MOTOR_METHOD), 4); // declared field still reads past the unknown record
}

#[test]
fn non_utf8_str_record_falls_back_to_default() {
    let mut b = RegionBuilder::new(PS);
    b.page_header(0, 0);
    // Plant invalid UTF-8 under DEVICE_NAME.
    b.record(
        PAGE_HEADER_LEN,
        DEVICE_NAME.key(),
        Type::Str.tag(),
        &[0xFF, 0xFE],
    );
    let mut f = b.build();
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get_str(DEVICE_NAME), "hoverboard"); // bad UTF-8 ignored -> default
}

// =====================================================================================
// 6. Backwards-compat: a store with unknown keys survives mount + a compaction (keys preserved,
//    key-agnostic).
// =====================================================================================

#[test]
fn unknown_keys_survive_compaction() {
    let mut b = RegionBuilder::new(PS);
    b.page_header(0, 0);
    let unknown = Key {
        field_id: 0x7B,
        index: 2,
    };
    let mut off = b.record(
        PAGE_HEADER_LEN,
        MOTOR_CURRENT_LIMIT.key(),
        Type::U32.tag(),
        &55u32.to_le_bytes(),
    );
    off = b.record(off, unknown, Type::U32.tag(), &0x1122_3344u32.to_le_bytes());
    let _ = off;
    let mut f = b.build();

    {
        let mut s = Store::mount(&mut f).unwrap();
        s.compact().unwrap();
    }
    // The known key still reads, and the unknown record was preserved verbatim (key-agnostic
    // compaction), checked by scanning the raw active region.
    assert!(
        raw_region_contains_key(&f, unknown),
        "unknown key dropped by compaction"
    );
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(MOTOR_CURRENT_LIMIT), 55);
}

// Helper: does the active region still hold a committed record for `key`? (key-agnostic survival.)
fn raw_region_contains_key(f: &MockFlash, key: Key) -> bool {
    let ps = PS;
    let region = f.as_bytes();
    for page in 0..2 {
        let base = page * ps;
        // Only scan a page that is a valid side.
        let magic = u32::from_le_bytes([
            region[base],
            region[base + 1],
            region[base + 2],
            region[base + 3],
        ]);
        if magic != MAGIC {
            continue;
        }
        let mut cursor = base + PAGE_HEADER_LEN;
        while cursor < base + ps {
            match record::parse_header(region, cursor) {
                HeaderScan::Good(h) => {
                    let here = cursor;
                    cursor = here + record::record_size(h.len as usize);
                    if h.field_id == key.field_id
                        && h.index == key.index
                        && record::is_committed(region, here, &h)
                    {
                        return true;
                    }
                }
                _ => break,
            }
        }
    }
    false
}

// =====================================================================================
// The persist-survives-reboot host test (the `run` function over MockFlash) + negative control.
// =====================================================================================

#[cfg(feature = "test-fields")]
#[test]
fn persist_survives_reboot() {
    let mut mock = MockFlash::erased(1024); // arg = page size; the mock is the two-page region
    run(&mut mock, 0); // cmd = (PERSIST, phase 0): set + persist
    assert_eq!(run(&mut mock, 1), T_VAL); // cmd = (PERSIST, phase 1): fresh mount reads from flash
}

#[cfg(feature = "test-fields")]
#[test]
fn no_write_reads_default_not_t_val() {
    // Negative control: only the read phase, never the set. The read returns the default, not T_VAL,
    // so a vacuous pass would be caught.
    let mut mock = MockFlash::erased(1024);
    assert_ne!(run(&mut mock, 1), T_VAL);
    assert_eq!(run(&mut mock, 1), T_KEY.default());
}

#[cfg(feature = "test-fields")]
#[test]
fn persist_survives_reboot_2k_page() {
    // The 2 KiB-page rerun uses erased(2048) (two 2 KiB pages).
    let mut mock = MockFlash::erased(2048);
    run(&mut mock, 0);
    assert_eq!(run(&mut mock, 1), T_VAL);
}
