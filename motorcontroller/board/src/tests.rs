//! Host tests: the dead-time floor and the status record.

extern crate std;

use super::status::{decode, Entry, Reason, StatusRecord, ENTRY_LEN, MAGIC, MAX_ENTRIES, NO_PORT};
use super::*;
use std::vec::Vec;

#[test]
fn the_dead_time_floor_is_500_ns_at_the_gate_clock() {
    // 500 ns at 72 MHz through the /2 generator is 18 ticks, below every value the fleet runs
    // (25, 28, 32) and above the shoot-through range.
    assert_eq!(DEAD_TIME_MIN_DTG, 18);
}

#[test]
fn an_empty_record_has_its_magic_no_latch_and_no_entries() {
    let r = StatusRecord::new();
    assert_eq!((r.magic, r.self_hold_port, r.count), (MAGIC, NO_PORT, 0));
    assert!(r.is_empty());
    assert_eq!(r.entries(), &[] as &[u8]);
    // The layout an SWD read decodes: magic, the latch's port and pin, count, then the entries
    // from byte 7.
    assert_eq!(core::mem::size_of::<StatusRecord>(), 64);
    assert_eq!(core::mem::offset_of!(StatusRecord, self_hold_port), 4);
    assert_eq!(core::mem::offset_of!(StatusRecord, self_hold_pin), 5);
    assert_eq!(core::mem::offset_of!(StatusRecord, count), 6);
    assert_eq!(core::mem::offset_of!(StatusRecord, entries), 7);
}

#[test]
fn entries_encode_as_four_bytes_each_in_record_order() {
    let mut r = StatusRecord::new();
    r.record(0x0F, 1, Reason::NotUp, 3);
    r.record(0x11, 0, Reason::NotUp, 0);
    assert!(!r.is_empty());
    assert_eq!(r.entries(), &[0x0F, 1, 6, 3, 0x11, 0, 6, 0]);
    let back: Vec<Entry> = decode(r.entries()).collect();
    assert_eq!(
        back,
        [
            Entry {
                field: 0x0F,
                index: 1,
                reason: 6,
                detail: 3
            },
            Entry {
                field: 0x11,
                index: 0,
                reason: 6,
                detail: 0
            },
        ]
    );
}

#[test]
fn the_record_holds_fourteen_entries_and_counts_the_rest() {
    let mut r = StatusRecord::new();
    for i in 0..20u8 {
        r.record(0x40 + i, 0, Reason::NotUp, i);
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
    assert_eq!(Reason::from_u8(Reason::NotUp as u8), Some(Reason::NotUp));
    assert_eq!(Reason::NotUp as u8, 6);
    for b in (0..=255u8).filter(|b| *b != 6) {
        assert_eq!(Reason::from_u8(b), None);
    }
}

#[test]
fn a_trailing_partial_entry_is_ignored() {
    assert_eq!(decode(&[1, 2, 3, 4, 5, 6]).count(), 1);
}
