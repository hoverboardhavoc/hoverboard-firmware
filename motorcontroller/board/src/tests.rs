//! Host tests: the pin decode, the dead-time floor and the status record.

extern crate std;

use super::status::{decode, Entry, Reason, StatusRecord, ENTRY_LEN, MAGIC, MAX_ENTRIES};
use super::*;
use std::vec::Vec;

#[test]
fn parse_accepts_the_encoding_and_rejects_the_rest() {
    // Valid ports A/B/C/D/F, all pin numbers.
    for (port, byte) in [(0u8, 0x00u8), (1, 0x1F), (2, 0x2D), (3, 0x30), (5, 0x5A)] {
        let p = match Pin::parse(byte) {
            Parsed::Valid(p) => p,
            other => panic!("byte {byte:#04x}: expected Valid, got {other:?}"),
        };
        assert_eq!(p.port(), port);
        assert_eq!(p.pin(), byte & 0x0F);
        assert_eq!(p.packed(), byte);
    }
    // 0xFF is absent, not an error.
    assert_eq!(Pin::parse(ABSENT), Parsed::Absent);
    // Port E (4) does not exist in the encoding; neither does anything above F (6..=14; 15 with
    // a non-0xF low nibble is also port 15 = invalid).
    for byte in [0x40u8, 0x4C, 0x60, 0x7F, 0x90, 0xA1, 0xE0, 0xF0, 0xFE] {
        assert_eq!(
            Pin::parse(byte),
            Parsed::Invalid,
            "byte {byte:#04x} must be encoding-invalid"
        );
    }
}

#[test]
fn the_dead_time_floor_is_500_ns_at_the_gate_clock() {
    // 500 ns at 72 MHz through the /2 generator is 18 ticks, below every value the fleet runs
    // (25, 28, 32) and above the shoot-through range.
    assert_eq!(DEAD_TIME_MIN_DTG, 18);
}

#[test]
fn an_empty_record_has_its_magic_no_latch_and_no_entries() {
    let r = StatusRecord::new();
    assert_eq!((r.magic, r.self_hold, r.count), (MAGIC, ABSENT, 0));
    assert!(r.is_empty());
    assert_eq!(r.entries(), &[] as &[u8]);
    // The layout an SWD read decodes: magic, latch, count, then the entries from byte 6.
    assert_eq!(core::mem::size_of::<StatusRecord>(), 64);
    assert_eq!(core::mem::offset_of!(StatusRecord, self_hold), 4);
    assert_eq!(core::mem::offset_of!(StatusRecord, count), 5);
    assert_eq!(core::mem::offset_of!(StatusRecord, entries), 6);
}

#[test]
fn entries_encode_as_four_bytes_each_in_record_order() {
    let mut r = StatusRecord::new();
    r.record(0x56, 1, Reason::NotRouted, 0x16);
    r.record(0x4D, 0, Reason::DeadTime, 10);
    assert!(!r.is_empty());
    assert_eq!(r.entries(), &[0x56, 1, 3, 0x16, 0x4D, 0, 5, 10]);
    let back: Vec<Entry> = decode(r.entries()).collect();
    assert_eq!(
        back,
        [
            Entry {
                field: 0x56,
                index: 1,
                reason: 3,
                detail: 0x16
            },
            Entry {
                field: 0x4D,
                index: 0,
                reason: 5,
                detail: 10
            },
        ]
    );
}

#[test]
fn the_record_holds_fourteen_entries_and_counts_the_rest() {
    let mut r = StatusRecord::new();
    for i in 0..20u8 {
        r.record(0x40 + i, 0, Reason::NotAPin, i);
    }
    assert_eq!(r.count, 20, "the count says how many there were");
    assert_eq!(r.entries().len(), MAX_ENTRIES * ENTRY_LEN);
    let kept: Vec<u8> = decode(r.entries()).map(|e| e.detail).collect();
    assert_eq!(
        kept,
        (0..14).collect::<Vec<u8>>(),
        "the first fourteen are kept"
    );
    // 56 B: the value fits one CONFIG_RESP.
    assert_eq!(r.entries().len(), 56);
}

#[test]
fn every_reason_round_trips_through_its_byte() {
    use Reason::*;
    for r in [
        NotAPin, Incomplete, NotRouted, SwdPin, DeadTime, NotUp, ImuFrame,
    ] {
        assert_eq!(Reason::from_u8(r as u8), Some(r));
    }
    assert_eq!(Reason::from_u8(0), None);
    assert_eq!(Reason::from_u8(8), None);
}

#[test]
fn a_trailing_partial_entry_is_ignored() {
    assert_eq!(decode(&[1, 2, 3, 4, 5, 6]).count(), 1);
}
