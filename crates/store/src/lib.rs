//! Layer 1: the flash key-value config store.
//!
//! A firmware-owned, wear-aware flash store of flat `field_id`/`index`/`type`/`value` records, with
//! one 16-bit key namespace shared by the flash store, a RAM live-table, and (later) the link register
//! protocol. Built logic-first against a mock flash so it is fully host-tested; the thin FMC backend
//! (`FmcFlash`, in the `store-test` crate) is wired to hardware last.
//!
//! Layout:
//! - [`params`]: the wire vocabulary ([`Key`] / [`Type`] / [`Value`]).
//! - [`registry`]: the field dictionary ([`FieldDef`] / [`REGISTRY`] / [`Persist`]).
//! - [`record`]: the on-flash record codec (two CRCs, the padded walk).
//! - [`flash`]: the region-relative [`Flash`] trait and the host [`MockFlash`].
//! - [`store`]: the log-structured [`Store`], the ping-pong region, and [`run_phase`].
//!
//! `no_std`, depends only on `base` (CRC, errors). The host test harness needs `std`.

#![no_std]
#[cfg(any(test, feature = "std"))]
extern crate std;

pub mod flash;
pub mod params;
pub mod record;
pub mod registry;
pub mod store;

pub use flash::Flash;
pub use params::{Key, Type, Value};
pub use registry::{FieldDef, Persist, REGISTRY};
pub use store::{run_phase, Store, T_KEY, T_VAL};

#[cfg(any(test, feature = "std"))]
pub use flash::MockFlash;

#[cfg(test)]
mod tests {
    use crate::flash::MockFlash;
    use crate::params::{Key, Value};
    use crate::registry::T_FIELD_ID;
    use crate::store::{run_phase, Store, T_KEY, T_VAL};

    // The two page sizes the tier-1 suite runs at: two 1 KiB pages, two 2 KiB pages.
    const PAGE_SIZES: [usize; 2] = [1024, 2048];

    /// The pass case: phase 0 sets + persists; a fresh (cold) mount in phase 1 reads it back from
    /// flash, not RAM, so output == T_VAL.
    #[test]
    fn persist_survives_reboot() {
        for &ps in &PAGE_SIZES {
            let mut mock = MockFlash::erased(ps); // arg = page size; the mock is the two-page region
            run_phase(&mut mock, 0); // set + persist
            assert_eq!(
                run_phase(&mut mock, 1),
                T_VAL,
                "page_size {ps}: a persisted value must survive the cold mount"
            );
        }
    }

    /// The negative control: set the value LIVE but skip persist; after the cold mount the read
    /// returns the default (!= T_VAL), so the assertion catches the missing persistence. This proves
    /// the test verifies flash persistence, not a RAM round-trip.
    #[test]
    fn no_persist() {
        for &ps in &PAGE_SIZES {
            let mut mock = MockFlash::erased(ps);
            // Set live but do NOT persist (skip run_phase(0)'s persist; do it by hand).
            {
                let mut store = Store::mount(&mut mock);
                store.set_live(T_KEY, Value::U32(T_VAL));
                // no persist
            }
            // Cold mount + read: the default (U32(0)), NOT T_VAL.
            let read = run_phase(&mut mock, 1);
            assert_ne!(
                read, T_VAL,
                "page_size {ps}: without persist the value must NOT survive the reboot"
            );
            assert_eq!(read, 0, "the absent key reads its registry default (0)");
        }
    }

    /// A direct round-trip of set_live + persist + cold-mount get for a non-test field, both sizes.
    #[test]
    fn set_persist_get_round_trips() {
        for &ps in &PAGE_SIZES {
            let mut mock = MockFlash::erased(ps);
            let key = Key::new(0x02, 0); // motor.current_limit (U16)
            {
                let mut store = Store::mount(&mut mock);
                store.set_live(key, Value::U16(1234));
                store.persist(&mut mock).unwrap();
            }
            let store = Store::mount(&mut mock);
            assert_eq!(store.get(key), Value::U16(1234));
        }
    }

    /// Latest-per-key wins on replay: two persists of the same key read back the second value.
    #[test]
    fn latest_per_key_wins() {
        let mut mock = MockFlash::erased(1024);
        let key = Key::new(0x04, 0); // features (U32)
        {
            let mut store = Store::mount(&mut mock);
            store.set_live(key, Value::U32(0x1111_1111));
            store.persist(&mut mock).unwrap();
            store.set_live(key, Value::U32(0x2222_2222));
            store.persist(&mut mock).unwrap();
        }
        let store = Store::mount(&mut mock);
        assert_eq!(store.get(key), Value::U32(0x2222_2222));
    }

    /// An absent key reads its registry default.
    #[test]
    fn absent_key_reads_default() {
        let mut mock = MockFlash::erased(1024);
        let store = Store::mount(&mut mock);
        // motor.current_limit default is 5000.
        assert_eq!(store.get(Key::new(0x02, 0)), Value::U16(5000));
        // The test field default is U32(0).
        assert_eq!(store.get(Key::new(T_FIELD_ID, 0)), Value::U32(0));
    }

    /// Compaction triggers when the page fills, preserves the latest value per key, and survives a
    /// cold mount (higher seq wins). Fill the page by persisting many distinct keys/indices.
    #[test]
    fn compaction_preserves_keys() {
        for &ps in &PAGE_SIZES {
            let mut mock = MockFlash::erased(ps);
            let key = Key::new(0x04, 0); // features (U32), 6-byte hdr + 4 val + 2 crc = 12 bytes/record
            {
                let mut store = Store::mount(&mut mock);
                // Persist the same key many times: each is a new append. With (page_size - 8) / 12
                // records fitting, well over that count forces at least one compaction.
                let iters = (ps / 12) as u32 + 50;
                for i in 0..iters {
                    store.set_live(key, Value::U32(i));
                    store.persist(&mut mock).unwrap();
                }
            }
            // The latest value survives the compaction + cold mount.
            let store = Store::mount(&mut mock);
            let expected = (ps / 12) as u32 + 50 - 1;
            assert_eq!(
                store.get(key),
                Value::U32(expected),
                "page_size {ps}: compaction must keep the latest value"
            );
        }
    }

    /// Unknown keys survive a mount and a compaction (backwards-compat: compaction is key-agnostic).
    /// We hand-build an unknown-field record (id 0x77, not in REGISTRY) of a KNOWN type (U32) directly
    /// into a freshly-formatted active page, mount (the scan walks past it and caches it), then force a
    /// compaction and confirm the unknown key's latest value is preserved verbatim.
    #[test]
    fn unknown_key_survives_compaction() {
        use crate::flash::Flash;
        use crate::params::Type;
        use crate::record::build;

        let unknown = Key::new(0x77, 0);
        for &ps in &PAGE_SIZES {
            let mut mock = MockFlash::erased(ps);
            // Format side 0 by hand: magic + seq=1, then one unknown-field U32 record right after.
            let mut hdr = [0xFFu8; 8];
            hdr[0..4].copy_from_slice(&0x5453_5648u32.to_le_bytes()); // PAGE_MAGIC "HVST"
            hdr[4..6].copy_from_slice(&1u16.to_le_bytes());
            mock.program(0, &hdr).unwrap();
            let mut rec = [0u8; 32];
            let n = build(
                unknown.field_id,
                unknown.index,
                Type::U32,
                &0xABCD_1234u32.to_le_bytes(),
                &mut rec,
            );
            mock.program(8, &rec[..n]).unwrap();

            // Mount: the scan walks past the unknown record and caches it (known type, unknown id).
            {
                let store = Store::mount(&mut mock);
                assert_eq!(
                    store.get(unknown),
                    Value::U32(0xABCD_1234),
                    "page_size {ps}: an unknown-id record is still read"
                );
            }
            // Force a compaction by filling with a KNOWN key, then confirm the unknown key survived.
            {
                let mut store = Store::mount(&mut mock);
                for i in 0..((ps / 12) as u32 + 50) {
                    store.set_live(Key::new(0x04, 0), Value::U32(i));
                    store.persist(&mut mock).unwrap();
                }
            }
            let store = Store::mount(&mut mock);
            assert_eq!(
                store.get(unknown),
                Value::U32(0xABCD_1234),
                "page_size {ps}: compaction is key-agnostic, the unknown key is preserved"
            );
        }
    }

    /// A torn (half-written) record is skipped on mount and the last good value reads. We persist a
    /// good value, then hand-corrupt a freshly appended record's value (val_crc no longer matches), and
    /// confirm the cold mount ignores the torn record and keeps the good value.
    #[test]
    fn torn_write_is_skipped() {
        use crate::flash::Flash;
        use crate::params::Type;
        use crate::record::{build, record_span, HEADER_LEN};

        let key = Key::new(0x04, 0); // features (U32)
        let mut mock = MockFlash::erased(1024);
        // Persist a good value and learn where the frontier is by mounting after.
        {
            let mut store = Store::mount(&mut mock);
            store.set_live(key, Value::U32(0x600D_600D));
            store.persist(&mut mock).unwrap();
        }
        // The good record sits at offset 8 (one 12-byte U32 record), so the frontier is at 8 + 12 = 20.
        let frontier = 8 + record_span(4);
        // Build a SECOND record for the same key but corrupt its value byte so val_crc fails (a torn
        // write: header intact + walkable, value not committed).
        let mut rec = [0u8; 32];
        let n = build(
            key.field_id,
            key.index,
            Type::U32,
            &0xBADB_AD00u32.to_le_bytes(),
            &mut rec,
        );
        rec[HEADER_LEN] ^= 0xFF; // corrupt the value: val_crc no longer matches the stored crc
        mock.program(frontier, &rec[..n]).unwrap();

        // Cold mount: the torn second record is skipped (val_crc gate), so the good value reads.
        let store = Store::mount(&mut mock);
        assert_eq!(
            store.get(key),
            Value::U32(0x600D_600D),
            "a torn record must not supersede the last good value"
        );
    }

    /// Write-once is enforced by the mock: re-programming a written halfword fails (not AND). This
    /// pins that the mock models silicon write-once, the property the codec/log depend on.
    #[test]
    fn write_once_is_enforced() {
        use crate::flash::Flash;
        let mut mock = MockFlash::erased(1024);
        mock.program(0, &[0x12, 0x34]).unwrap();
        // Re-programming the same halfword (now non-0xFFFF) is REFUSED.
        assert!(mock.program(0, &[0x56, 0x78]).is_err());
        // The original content is unchanged (no AND).
        let mut buf = [0u8; 2];
        mock.read(0, &mut buf).unwrap();
        assert_eq!(buf, [0x12, 0x34]);
        // A misaligned / odd-length program is rejected too.
        assert!(mock.program(1, &[0xAA, 0xBB]).is_err());
        assert!(mock.program(8, &[0xAA]).is_err());
    }
}
