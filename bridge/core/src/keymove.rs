//! The one-off key move (`specs/machine-settings.md` section 8): bench-only code for the fleet's
//! move to the renumbered field registry, reverted once all four bridges have confirmed it.
//!
//! A bridge's store is not erased in the move: it holds the Wi-Fi credentials that keep it
//! reachable. So at every boot, after the store mounts and before Wi-Fi starts, the bridge copies
//! its four keys from their old ids to their new ones ([`copy`]), and once the running slot is
//! `Valid` it tombstones the old records ([`clear`]): in the new map `0x10..0x13` are
//! `links.<i>.rx`, `imu.scl`, `imu.sda` and `imu.model`, and a stale string there would read as a
//! pin. A `PendingVerify` boot leaves them, so a bridge that is never confirmed rolls back to an
//! old image that still finds its name and credentials.

use store::{lookup_key, DynError, Flash, Key, Store, StoreError, Stored, Value, ERASED_TAG};

/// The four keys by their old ids, and the fields they moved to: `device.name`, `wifi.ssid`,
/// `wifi.pass`, `wifi.tx_power`.
const MOVES: [(u8, Key); 4] = [
    (0x10, store::DEVICE_NAME.key()),
    (0x11, store::WIFI_SSID.key()),
    (0x12, store::WIFI_PASS.key()),
    (0x13, store::WIFI_TX_POWER.key()),
];

/// The longest value the copy carries: what one `CONFIG_WRITE` could have stored.
const VALUE_MAX: usize = net::walk::CONFIG_WRITE_VALUE_MAX;

/// One line of the move's console log: the ids, never the value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// The value at old id `0` was written at new id `1` (`moved 0x11 -> 0x03`).
    Moved(u8, u8),
    /// The record at old id `0` was tombstoned (`cleared 0x11`).
    Cleared(u8),
}

/// The newest committed record at the old id `old`, read raw (`Store::for_each_in` does not
/// consult the registry), when it holds a value of `to`'s registered type: the value's bytes and
/// their length. A tombstone, a record of another type, or none: `None`.
fn old_value<F: Flash>(s: &Store<F>, old: u8, to: Key) -> Option<([u8; VALUE_MAX], usize)> {
    let tag = lookup_key(to)?.kind.tag();
    let mut found = None;
    s.for_each_in(
        &[Key {
            field_id: old,
            index: 0,
        }],
        |_, t, bytes| {
            found = (t == tag && bytes.len() <= VALUE_MAX).then(|| {
                let mut b = [0u8; VALUE_MAX];
                b[..bytes.len()].copy_from_slice(bytes);
                (b, bytes.len())
            });
        },
    );
    found
}

/// Write `value` at `key`, compacting once on a full page as every store writer does.
fn write<F: Flash>(s: &mut Store<F>, key: Key, value: Value) -> Result<(), DynError> {
    match s.set_value(key, value) {
        Err(DynError::Store(StoreError::Full)) => {
            s.compact()?;
            s.set_value(key, value)
        }
        r => r,
    }
}

/// Copy each key found at its old id to its new id, overwriting what the new id holds. A new id
/// that already holds the same value is left as it is, so a repeated copy writes nothing.
pub fn copy<F: Flash>(s: &mut Store<F>, log: &mut impl FnMut(Step)) {
    for (old, to) in MOVES {
        let Some(def) = lookup_key(to) else { continue };
        let Some((buf, n)) = old_value(s, old, to) else {
            continue;
        };
        let Some(value) = Value::decode(def.kind, &buf[..n]) else {
            continue;
        };
        if matches!(s.get_value(to), Ok((v, Stored::Yes)) if v == value) {
            continue;
        }
        if write(s, to, value).is_ok() {
            log(Step::Moved(old, to.field_id));
        }
    }
}

/// Whether any of the four old records is still there (not tombstoned).
pub fn left<F: Flash>(s: &Store<F>) -> bool {
    MOVES
        .iter()
        .any(|&(old, to)| old_value(s, old, to).is_some())
}

/// Tombstone each old record still there. Only once the running slot is `Valid`: in the
/// `ESP_UPDATE_CONFIRM` handler after it marks the slot, and at a boot that finds the slot `Valid`
/// with old records left (a confirm whose tombstones were interrupted).
pub fn clear<F: Flash>(s: &mut Store<F>, log: &mut impl FnMut(Step)) {
    for (old, to) in MOVES {
        if old_value(s, old, to).is_none() {
            continue;
        }
        let key = Key {
            field_id: old,
            index: 0,
        };
        let r = match s.erase_value(key) {
            Err(DynError::Store(StoreError::Full)) => s
                .compact()
                .map_err(DynError::Store)
                .and_then(|()| s.erase_value(key)),
            r => r,
        };
        if r.is_ok() {
            log(Step::Cleared(old));
        }
    }
}

/// The boot half: copy, then, when the running slot is not waiting for confirmation, clear what
/// is left.
pub fn at_boot<F: Flash>(s: &mut Store<F>, pending: bool, log: &mut impl FnMut(Step)) {
    copy(s, log);
    if !pending && left(s) {
        clear(s, log);
    }
}

// A record whose newest entry is a tombstone is not a value to copy: `for_each_in` hands the
// tombstone's tag, which no registered type uses.
const _: () = assert!(ERASED_TAG != store::Type::Str.tag() && ERASED_TAG != store::Type::U8.tag());

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::sim::RamFlash;
    use std::vec::Vec;

    /// A bridge store as the current image wrote it: the name, the network and the power at the
    /// old ids, through handles at those ids (the typed path does not consult the registry).
    fn old_store() -> RamFlash {
        let mut f = RamFlash::erased(4096);
        {
            let mut s = Store::mount(&mut f).unwrap();
            s.set_str(store::StrField::new(0x10, ""), "rover-l")
                .unwrap();
            s.set_str(store::StrField::new(0x11, ""), "home").unwrap();
            s.set_str(store::StrField::new(0x12, ""), "secret-pass")
                .unwrap();
            s.set(store::Field::<u8>::new(0x13, 0), 40).unwrap();
        }
        f
    }

    fn run(f: &mut RamFlash, pending: bool) -> Vec<Step> {
        let mut s = Store::mount(f).unwrap();
        let mut steps = Vec::new();
        at_boot(&mut s, pending, &mut |st| steps.push(st));
        steps
    }

    fn values(f: &mut RamFlash) -> (Vec<u8>, Vec<u8>, Vec<u8>, u8) {
        let s = Store::mount(f).unwrap();
        (
            s.get_text(store::DEVICE_NAME).to_vec(),
            s.get_text(store::WIFI_SSID).to_vec(),
            s.get_text(store::WIFI_PASS).to_vec(),
            s.get(store::WIFI_TX_POWER),
        )
    }

    /// `specs/machine-settings.md` section 9, test 8: the four values come out at the new ids; a
    /// `PendingVerify` boot leaves the old records, a `Valid` one (or the confirm) tombstones
    /// them, and a second run changes nothing.
    #[test]
    fn the_keys_move_and_the_old_records_go_once_the_slot_is_valid() {
        let mut f = old_store();
        // The first boot of the new image waits for its confirm: copy, keep the old records.
        let steps = run(&mut f, true);
        assert_eq!(
            steps,
            [
                Step::Moved(0x10, 0x02),
                Step::Moved(0x11, 0x03),
                Step::Moved(0x12, 0x04),
                Step::Moved(0x13, 0x05),
            ]
        );
        assert_eq!(
            values(&mut f),
            (
                b"rover-l".to_vec(),
                b"home".to_vec(),
                b"secret-pass".to_vec(),
                40
            )
        );
        assert!(left(&Store::mount(&mut f).unwrap()));
        // Another unconfirmed boot copies nothing new and still keeps them.
        assert_eq!(run(&mut f, true), []);
        assert!(left(&Store::mount(&mut f).unwrap()));
        // The confirm tombstones them.
        {
            let mut s = Store::mount(&mut f).unwrap();
            let mut steps = Vec::new();
            clear(&mut s, &mut |st| steps.push(st));
            assert_eq!(
                steps,
                [
                    Step::Cleared(0x10),
                    Step::Cleared(0x11),
                    Step::Cleared(0x12),
                    Step::Cleared(0x13),
                ]
            );
        }
        assert!(!left(&Store::mount(&mut f).unwrap()));
        // The new ids keep the values; the old ids read their new fields' defaults.
        assert_eq!(
            values(&mut f),
            (
                b"rover-l".to_vec(),
                b"home".to_vec(),
                b"secret-pass".to_vec(),
                40
            )
        );
        {
            let s = Store::mount(&mut f).unwrap();
            assert_eq!(s.get(store::LINK_RX.at(0)), store::PIN_ABSENT);
            assert_eq!(s.get(store::IMU_SCL_PIN), store::PIN_ABSENT);
            assert_eq!(s.get(store::IMU_MODEL), 0);
        }
        // A second run, confirmed or not, changes nothing.
        let before = f.bytes.clone();
        assert_eq!(run(&mut f, false), []);
        assert_eq!(run(&mut f, true), []);
        assert!(f.bytes == before);
    }

    /// A boot that finds the slot already `Valid` with old records left (a confirm whose
    /// tombstones were interrupted) clears them.
    #[test]
    fn a_valid_boot_with_old_records_left_clears_them() {
        let mut f = old_store();
        let steps = run(&mut f, false);
        assert_eq!(steps.len(), 8, "four moved, four cleared: {steps:?}");
        assert!(!left(&Store::mount(&mut f).unwrap()));
        assert_eq!(values(&mut f).0, b"rover-l".to_vec());
    }

    /// A store with nothing at the old ids (a fresh install of the new image) moves nothing.
    #[test]
    fn nothing_at_the_old_ids_moves_nothing() {
        let mut f = RamFlash::erased(4096);
        assert_eq!(run(&mut f, false), []);
        assert_eq!(run(&mut f, true), []);
    }
}
