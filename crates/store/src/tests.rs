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
    assert_eq!(s.get_text(DEVICE_NAME), b"Hoverboard");
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
    assert_eq!(s.get_text(DEVICE_NAME), b"my-board");
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
fn non_utf8_str_record_reads_back_as_its_bytes() {
    // REWRITTEN at shrink round 2 (`specs/decision-flash-budget.md`, item 4), and it is the one
    // behaviour change of that item: this test used to be
    // `non_utf8_str_record_falls_back_to_default`, because the board validated a `STR` record as
    // UTF-8 and ignored one that failed. It no longer validates, so the record reads back as the
    // bytes it holds. The malformed-record rule is otherwise unchanged: a wrong-width fixed type,
    // an absent record and a wrong-type record all still read as the field's default (the tests
    // above and below pin those).
    let mut b = RegionBuilder::new(PS);
    b.page_header(0, 0);
    // Plant bytes that are not valid UTF-8 under DEVICE_NAME.
    b.record(
        PAGE_HEADER_LEN,
        DEVICE_NAME.key(),
        Type::Str.tag(),
        &[0xFF, 0xFE],
    );
    let mut f = b.build();
    let s = Store::mount(&mut f).unwrap();
    assert_eq!(s.get_text(DEVICE_NAME), &[0xFF, 0xFE]);
    // The dynamic face agrees with the typed one, rather than one of them keeping the old rule.
    assert_eq!(
        s.get_value(DEVICE_NAME.key()).unwrap(),
        crate::value::Value::Str(&[0xFF, 0xFE])
    );
    // A wrong-TYPE record is still ignored: the default comes back. Only the charset check went.
    let mut b2 = RegionBuilder::new(PS);
    b2.page_header(0, 0);
    b2.record(
        PAGE_HEADER_LEN,
        DEVICE_NAME.key(),
        Type::Blob.tag(),
        &[0xFF, 0xFE],
    );
    let mut f2 = b2.build();
    let s2 = Store::mount(&mut f2).unwrap();
    assert_eq!(s2.get_text(DEVICE_NAME), b"Hoverboard");
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

// ---------------------------------------------------------------------------------------------------
// The dynamic Key/Value path (get_value / set_value over the registry): the deferred Layer-3 CONFIG_*
// face, un-deferred here. Exercised against MockFlash exactly like the typed path.
// ---------------------------------------------------------------------------------------------------
mod dynamic {
    use super::*;
    use crate::store::DynError;
    use crate::value::Value;

    #[test]
    fn set_value_then_get_value_round_trips_a_scalar() {
        let mut f = MockFlash::erased(PS);
        let mut s = Store::mount(&mut f).unwrap();
        let key = MOTOR_CURRENT_LIMIT.key();
        s.set_value(key, Value::U32(15_000)).unwrap();
        assert_eq!(s.get_value(key).unwrap(), Value::U32(15_000));
    }

    #[test]
    fn get_value_of_an_absent_key_returns_the_registered_default() {
        let mut f = MockFlash::erased(PS);
        let s = Store::mount(&mut f).unwrap();
        // Never written: the registry default comes back (10_000 for MOTOR_CURRENT_LIMIT).
        assert_eq!(
            s.get_value(MOTOR_CURRENT_LIMIT.key()).unwrap(),
            Value::U32(10_000)
        );
        assert_eq!(
            s.get_value(DEVICE_NAME.key()).unwrap(),
            Value::Str(b"Hoverboard")
        );
    }

    #[test]
    fn set_value_then_get_value_round_trips_a_string() {
        let mut f = MockFlash::erased(PS);
        let mut s = Store::mount(&mut f).unwrap();
        let key = DEVICE_NAME.key();
        s.set_value(key, Value::Str(b"board-7")).unwrap();
        assert_eq!(s.get_value(key).unwrap(), Value::Str(b"board-7"));
    }

    #[test]
    fn type_mismatch_is_rejected_and_writes_nothing() {
        let mut f = MockFlash::erased(PS);
        let mut s = Store::mount(&mut f).unwrap();
        let key = MOTOR_CURRENT_LIMIT.key(); // a U32 field
        assert_eq!(s.set_value(key, Value::U16(5)), Err(DynError::TypeMismatch));
        assert_eq!(
            s.set_value(key, Value::Str(b"x")),
            Err(DynError::TypeMismatch)
        );
        // The field is untouched: still the default.
        assert_eq!(s.get_value(key).unwrap(), Value::U32(10_000));
    }

    #[test]
    fn unknown_key_is_rejected_on_both_get_and_set() {
        let mut f = MockFlash::erased(PS);
        let mut s = Store::mount(&mut f).unwrap();
        let bogus = Key {
            field_id: 0x99,
            index: 0,
        };
        assert_eq!(s.get_value(bogus), Err(DynError::UnknownKey));
        assert_eq!(s.set_value(bogus, Value::U8(1)), Err(DynError::UnknownKey));
    }

    #[test]
    fn a_dynamic_write_persists_across_a_cold_remount() {
        // The "node_address persist + survive reboot" shape (slice 4 uses this for ASSIGN): write via
        // the dynamic path, drop the store, cold-mount a fresh one, and read it back from flash.
        let mut f = MockFlash::erased(PS);
        let key = MOTOR_METHOD.key(); // a U8 field
        {
            let mut s = Store::mount(&mut f).unwrap();
            s.set_value(key, Value::U8(3)).unwrap();
        }
        let s = Store::mount(&mut f).unwrap();
        assert_eq!(s.get_value(key).unwrap(), Value::U8(3));
    }

    /// The `CONTROL_GAIN_*` defaults are the control crate's compiled gain constants, index for
    /// index, so a board that has never been tuned runs exactly what it ran when these were
    /// constants (`specs/rider-ui.md` section 4, "Defaults equal today's constants").
    ///
    /// The two crates cannot share the declaration: `control` is pure cascade math and does not
    /// depend on `store`, and `store` must not depend on `control` (a config store that needs the
    /// control loop to compile is the wrong shape). So the values are written twice and pinned
    /// here, through a dev-dependency that reaches no shipped build. The field IDs are pinned the
    /// same way: `control::GAIN_FIELD_A/B` are what the live tune seam matches on.
    #[test]
    fn the_gain_field_defaults_are_the_control_crates_compiled_profiles() {
        use crate::field::{CONTROL_GAIN_A, CONTROL_GAIN_B};
        assert_eq!(CONTROL_GAIN_A.id(), control::GAIN_FIELD_A);
        assert_eq!(CONTROL_GAIN_B.id(), control::GAIN_FIELD_B);
        assert_eq!(CONTROL_GAIN_A.len(), control::GAINS_PER_PROFILE);
        assert_eq!(CONTROL_GAIN_B.len(), control::GAINS_PER_PROFILE);

        let a = control::RUN_PROFILE_A;
        let b = control::PROFILE_B;
        for (field, triple) in [(CONTROL_GAIN_A, a), (CONTROL_GAIN_B, b)] {
            let want = [triple.kp, triple.bk, triple.pr];
            for (index, w) in want.iter().enumerate() {
                assert_eq!(
                    i32::from(field.at(index as u8).default()),
                    *w,
                    "{:#04x} index {index} default drifted from the control constant",
                    field.id()
                );
            }
        }

        // And every default is inside the seam's own default range, so a fresh board's values
        // survive the clamp the boot seam puts them through unchanged.
        for (index, hi) in control::DEFAULT_GAIN_MAX.iter().enumerate() {
            for field in [CONTROL_GAIN_A, CONTROL_GAIN_B] {
                let d = field.at(index as u8).default();
                assert!(
                    d >= control::GAIN_MIN && d <= *hi,
                    "{:#04x} index {index} default {d} is outside its own range",
                    field.id()
                );
            }
        }
    }

    /// The `CONTROL_GAIN_MAX` defaults are `control::DEFAULT_GAIN_MAX`, index for index, so a
    /// board that never staged the maxima enforces exactly the old compiled range table
    /// (`specs/rider-ui.md` section 4, "Ranges"). Pinned here for the reason the gain defaults are:
    /// `control` does not depend on `store`.
    #[test]
    fn the_gain_max_defaults_are_the_control_crates_default_maxima() {
        use crate::field::CONTROL_GAIN_MAX;
        assert_eq!(CONTROL_GAIN_MAX.id(), 0x74);
        assert_eq!(CONTROL_GAIN_MAX.len(), control::GAINS_PER_PROFILE);
        for (index, want) in control::DEFAULT_GAIN_MAX.iter().enumerate() {
            assert_eq!(CONTROL_GAIN_MAX.at(index as u8).default(), *want);
        }
    }

    /// The `CONTROL_GAIN_MAX` boot seam end to end: a maximum written below a stored gain clamps
    /// that gain when the next boot builds the shadow from the store, and the lane then refuses
    /// above it. Boot-read, NOT on the tune lane.
    #[test]
    fn a_stored_gain_max_bounds_the_shadow_built_at_the_next_boot() {
        use crate::field::{CONTROL_GAIN_A, CONTROL_GAIN_B, CONTROL_GAIN_MAX};
        let mut f = MockFlash::erased(PS);
        {
            let mut s = Store::mount(&mut f).unwrap();
            s.set_value(CONTROL_GAIN_MAX.at(0).key(), Value::I16(5000))
                .unwrap();
        }
        let s = Store::mount(&mut f).unwrap();
        let read = |field: crate::IndexedField<i16, 3>| {
            [s.get(field.at(0)), s.get(field.at(1)), s.get(field.at(2))]
        };
        let mut g = control::GainShadow::of_stored(
            [read(CONTROL_GAIN_A), read(CONTROL_GAIN_B)],
            read(CONTROL_GAIN_MAX),
        );
        assert_eq!(g.a().kp, 5000, "6000 clamps to the stored maximum");
        assert_eq!(g.b().kp, 3000, "under it: unchanged");
        assert_eq!(
            g.set(control::GAIN_FIELD_A, 0, 5001),
            Err(control::TuneError::OutOfRange)
        );
        assert_eq!(g.set(control::GAIN_FIELD_A, 0, 5000), Ok(()));
        for index in 0..3 {
            assert_eq!(
                g.set(CONTROL_GAIN_MAX.id(), index, 0),
                Err(control::TuneError::UnknownKey),
                "not on the tune lane"
            );
        }
    }

    /// The `CONTROL_DRIVE_LEAN` defaults are the control crate's unstaged `DriveLean` (lean_max 0 =
    /// disabled, lean_slew 4), and both survive the boot seam's clamps unchanged, so a board that
    /// was never staged runs exactly the pre-(h) loop (`specs/control.md` (h)). Pinned here for the
    /// same reason the gain defaults are: `control` does not depend on `store`.
    #[test]
    fn the_drive_lean_defaults_are_the_control_crates_unstaged_seam() {
        use crate::field::CONTROL_DRIVE_LEAN;
        assert_eq!(CONTROL_DRIVE_LEAN.id(), 0x73);
        assert_eq!(CONTROL_DRIVE_LEAN.len(), 2);
        let (max, slew) = (
            CONTROL_DRIVE_LEAN.at(0).default(),
            CONTROL_DRIVE_LEAN.at(1).default(),
        );
        let unstaged = control::DriveLean::default();
        assert_eq!(max, unstaged.lean_max());
        assert_eq!(slew, unstaged.lean_slew());
        assert_eq!(
            control::DriveLean::new(max, slew),
            unstaged,
            "the seam clamps neither"
        );
        assert_eq!(max, 0, "disabled by default");
    }

    /// The IMU frame seam, end to end: the three fields as the app writes them (the dynamic
    /// `CONFIG_WRITE` path), read back the way the firmware's bring-up reads them (typed, per
    /// index), staged into `imu::Config`, and decoded. A board on edge (UP = chip X, PITCH_RATE =
    /// chip Z) must put the gravity it reads on chip X onto body Z (`specs/imu.md`,
    /// `IMU_AXIS_ROLE`). An unstaged store yields the compiled frame, and a staged repeated role
    /// is refused.
    #[test]
    fn the_imu_frame_fields_round_trip_into_the_staged_config() {
        use crate::field::{IMU_AXIS_ROLE, IMU_AXIS_SIGN, IMU_GYRO_BIAS};
        let read = |s: &Store<'_, MockFlash>| {
            let mut sign = [0i32; 6];
            for (i, v) in sign.iter_mut().enumerate() {
                *v = s.get(IMU_AXIS_SIGN.at(i as u8));
            }
            let bias = [0u8, 1, 2].map(|i| s.get(IMU_GYRO_BIAS.at(i)));
            let roles = [0u8, 1].map(|i| s.get(IMU_AXIS_ROLE.at(i)));
            imu::Config::staged(sign, bias, roles)
        };
        let key = |f: u8, i: u8| Key {
            field_id: f,
            index: i,
        };

        let mut f = MockFlash::erased(PS);
        let mut s = Store::mount(&mut f).unwrap();
        assert_eq!(IMU_AXIS_ROLE.id(), 0x68);
        // Nothing staged: the compiled frame, the identity roles.
        let unstaged = read(&s).unwrap();
        assert_eq!(unstaged.roles, imu::DEFAULT_ROLES);
        assert_eq!(unstaged.sign, imu::Config::default().sign);

        // Stage an on-edge frame: all-positive signs (a cyclic permutation needs product +1).
        for i in 0..6 {
            s.set_value(key(0x65, i), Value::I32(1)).unwrap();
        }
        s.set_value(key(0x61, 0), Value::I32(5)).unwrap();
        s.set_value(key(0x68, 0), Value::U8(1)).unwrap();
        s.set_value(key(0x68, 1), Value::U8(3)).unwrap();
        let cfg = read(&s).unwrap();
        assert_eq!(cfg.roles, [1, 3]);
        assert_eq!(cfg.sign, [1; 6]);
        assert_eq!(cfg.gyro_bias, [5, 0, 0]);
        let mut buf = [0u8; imu::BURST_LEN];
        buf[0..2].copy_from_slice(&8192i16.to_be_bytes()); // +1 g on chip X
        buf[8..10].copy_from_slice(&105i16.to_be_bytes()); // a rate on chip X
        let sample = imu::Imu::new(imu::MPU6050, cfg).decode(&buf);
        assert_eq!(sample.accel_raw, [0, 0, 8192], "gravity on body Z");
        assert_eq!(
            sample.gyro_raw,
            [0, 0, 100],
            "the chip-X bias rides to body Z"
        );

        // A repeated role is refused, not staged.
        s.set_value(key(0x68, 1), Value::U8(1)).unwrap();
        assert_eq!(read(&s).unwrap_err(), imu::FrameError::Roles);
    }

    /// The `CONTROL_RIDER_REQUIRED` seam end to end (`specs/control.md` (i)): an erased board reads
    /// the default 1 and the control dispatch's boot seam keeps the rider gate; a `CONFIG_WRITE`
    /// of 0 (the dynamic path) survives a cold remount, and the next boot's typed read waives it.
    /// The field is a boot-read machine-type setting, NOT a live tunable: the tune lane's
    /// allowlist (the control crate's `GainShadow`) refuses its key.
    #[test]
    fn the_rider_requirement_round_trips_from_the_store_into_the_control_dispatch() {
        use crate::field::CONTROL_RIDER_REQUIRED;
        assert_eq!(CONTROL_RIDER_REQUIRED.id(), 0x23);
        let mut f = MockFlash::erased(PS);
        {
            let s = Store::mount(&mut f).unwrap();
            assert_eq!(s.get(CONTROL_RIDER_REQUIRED), 1, "default = required");
            let d = control::ControlDispatch::new(
                1,
                true,
                s.get(CONTROL_RIDER_REQUIRED),
                s.get(crate::field::CONTROL_BATTERY_FLOOR),
            );
            assert!(
                d.rider_required(),
                "an unconfigured board keeps the rider gate"
            );
        }
        {
            let mut s = Store::mount(&mut f).unwrap();
            s.set_value(CONTROL_RIDER_REQUIRED.key(), Value::U8(0))
                .unwrap();
        }
        let s = Store::mount(&mut f).unwrap();
        assert_eq!(s.get(CONTROL_RIDER_REQUIRED), 0);
        let d = control::ControlDispatch::new(
            1,
            true,
            s.get(CONTROL_RIDER_REQUIRED),
            s.get(crate::field::CONTROL_BATTERY_FLOOR),
        );
        assert!(!d.rider_required(), "the next boot waives it");

        let mut g = control::GainShadow::default();
        for index in 0..3 {
            assert_eq!(
                g.set(CONTROL_RIDER_REQUIRED.id(), index, 0),
                Err(control::TuneError::UnknownKey),
                "not on the tune lane"
            );
            assert_eq!(g.get(CONTROL_RIDER_REQUIRED.id(), index), None);
        }
    }

    /// The `CONTROL_BATTERY_FLOOR` seam end to end (`specs/sensing-and-safety.md`, "The low-battery
    /// floor"): an erased board reads 2400 cV and the dispatch refuses 2399; a `CONFIG_WRITE` of 0
    /// survives a cold remount and the next boot engages at any known word. Boot-read, NOT on the
    /// tune lane.
    #[test]
    fn the_battery_floor_round_trips_from_the_store_into_the_control_dispatch() {
        use crate::field::{CONTROL_BATTERY_FLOOR, CONTROL_RIDER_REQUIRED};
        assert_eq!(CONTROL_BATTERY_FLOOR.id(), 0x24);
        let mut f = MockFlash::erased(PS);
        {
            let s = Store::mount(&mut f).unwrap();
            assert_eq!(s.get(CONTROL_BATTERY_FLOOR), 2400, "default");
            let d = control::ControlDispatch::new(
                1,
                true,
                s.get(CONTROL_RIDER_REQUIRED),
                s.get(CONTROL_BATTERY_FLOOR),
            );
            assert!(!d.battery_ok(2399));
            assert!(d.battery_ok(2400));
        }
        {
            let mut s = Store::mount(&mut f).unwrap();
            s.set_value(CONTROL_BATTERY_FLOOR.key(), Value::I16(0))
                .unwrap();
        }
        let s = Store::mount(&mut f).unwrap();
        assert_eq!(s.get(CONTROL_BATTERY_FLOOR), 0);
        let d = control::ControlDispatch::new(
            1,
            true,
            s.get(CONTROL_RIDER_REQUIRED),
            s.get(CONTROL_BATTERY_FLOOR),
        );
        assert!(d.battery_ok(1), "no floor: any known word");
        assert!(!d.battery_ok(0), "UNKNOWN still refuses");

        let mut g = control::GainShadow::default();
        assert_eq!(
            g.set(CONTROL_BATTERY_FLOOR.id(), 0, 0),
            Err(control::TuneError::UnknownKey)
        );
    }

    #[test]
    fn registry_is_enumerable_and_every_field_round_trips_its_default() {
        // Enumerate the registry and confirm each field's dynamic get (absent) equals its default - the
        // schema-less "render any field generically" property.
        let mut f = MockFlash::erased(PS);
        let s = Store::mount(&mut f).unwrap();
        for d in &crate::field::REGISTRY {
            // The entry's OWN key: an index family declared with `IndexedField` contributes one
            // entry per index, and each must read back that index's default.
            let key = Key {
                field_id: d.field_id,
                index: d.index,
            };
            let got = s.get_value(key).unwrap();
            assert_eq!(got.kind(), d.kind);
            assert_eq!(got, d.default);
        }
    }

    // There is deliberately NO host test here for "REGISTRY is borrowed rather than rebuilt", and the
    // absence is the finding. One was written and it was worthless: it compared the `Str` default's
    // pointer from `REGISTRY` against the one `lookup` returned, but `Value::Str` points at the
    // `"Hoverboard"` literal's bytes in `.rodata`, NOT into the table, so a copied `FieldDef` carries the same
    // pointer bit-for-bit and the assertion holds whether the table is borrowed or rebuilt. It passed
    // with the by-value registry restored (audit round 1, 2026-08-13). Its companion assertion, that
    // `addr_of!(REGISTRY)` equals itself, is a tautology of every static.
    //
    // The property is about how much STACK a call consumes, and stack consumption is not observable
    // from inside safe Rust on the host: the frame is gone by the time any code the test controls
    // runs. So the ONLY gate on this is tier 2, where the emulator owns memory and can measure the
    // real image's stack pointer excursion directly:
    // `dynamic_config_write_costs_no_extra_stack_chip1k` in crates/emulator-runner. A green host tick
    // that catches nothing is worse than no host test, because it stops the next person looking.

    #[cfg(feature = "test-fields")]
    #[test]
    fn the_dynamic_scenario_round_trips_on_the_host_too() {
        // The tier-1 half of the DYN_VALUE scenario the emulator runs on silicon-shaped hardware:
        // `set_value` persists and a COLD MOUNT reads it back through `get_value`. Same `run` entry,
        // same cmd packing, MockFlash instead of FmcFlash.
        let mut f = MockFlash::erased(PS);
        assert_eq!(run(&mut f, crate::DYN_VALUE << 16), 0);
        assert_eq!(run(&mut f, (crate::DYN_VALUE << 16) | 1), T_VAL);
    }
}
