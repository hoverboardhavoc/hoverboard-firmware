//! The field registry: the store's field dictionary.
//!
//! One row per `field_id`, holding only what the store reads: the storage [`Type`], the default
//! [`Value`], and the [`Persist`] class. Lookup is a `binary_search` on `field_id` (the table is
//! sorted by id, asserted `const`). An unknown id is skipped, not fatal (newer firmware writes keys
//! older consumers ignore).
//!
//! The registry deliberately carries NO hardware pins, no `Sem`/range/name, and no `arity`: those are
//! the compile-time board model or a codegen data file, not embedded facts. It is a curated, minimal
//! set of genuine tunables plus the reserved store-test field.

use crate::params::{Type, Value};

/// The persistence class of a field.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Persist {
    /// Apply-live in RAM, flush to flash when disarmed (gains, board config, node address, odometry).
    Persistent,
    /// RAM only, never written; resets to default on boot (a guest session address, runtime state).
    Volatile,
}

/// One registry row: the store's contract for a `field_id`.
#[derive(Clone, Copy, Debug)]
pub struct FieldDef {
    /// The field id (matches `Key.field_id`).
    pub id: u8,
    /// The storage type; the store sizes and validates a record against it.
    pub kind: Type,
    /// The value an absent key reads (same for every index).
    pub default: Value<'static>,
    /// The persistence class.
    pub persist: Persist,
}

/// The reserved store-test field id. A `Persistent` `U32`, used by `run_phase` and the tiers' images.
pub const T_FIELD_ID: u8 = 0xFE;

/// The field dictionary, ONE row per `field_id`, sorted by id (the `const` assertion below enforces
/// uniqueness + sort order). A deliberately minimal, curated set; the only Rust consumers are the
/// store and the firmware.
pub const REGISTRY: &[FieldDef] = &[
    // motor.method: which commutation/control method (an enum stored as U8). Singleton-ish per motor
    // index. Default 0 (the baseline method).
    FieldDef {
        id: 0x01,
        kind: Type::U8,
        default: Value::U8(0),
        persist: Persist::Persistent,
    },
    // motor.current_limit: the per-motor current limit in milliamps (U16). Default a conservative cap.
    FieldDef {
        id: 0x02,
        kind: Type::U16,
        default: Value::U16(5000),
        persist: Persist::Persistent,
    },
    // odometry: lifetime hall-tick distance (U64), accumulates in RAM, flushes on disarm.
    FieldDef {
        id: 0x03,
        kind: Type::U64,
        default: Value::U64(0),
        persist: Persist::Persistent,
    },
    // features: a bitset of enabled feature flags (U32).
    FieldDef {
        id: 0x04,
        kind: Type::U32,
        default: Value::U32(0),
        persist: Persist::Persistent,
    },
    // board.name: a human-set board name (STR, variable, disarmed-only direct append, read on demand).
    FieldDef {
        id: 0x05,
        kind: Type::Str,
        default: Value::Bytes(b""),
        persist: Persist::Persistent,
    },
    // The reserved store-test field (T_KEY): a Persistent U32 the test images set + persist + read.
    FieldDef {
        id: T_FIELD_ID,
        kind: Type::U32,
        default: Value::U32(0),
        persist: Persist::Persistent,
    },
];

// A const uniqueness + sort assertion over REGISTRY ids: a duplicate or an out-of-order id is a build
// error (the binary_search lookup depends on the sort). Evaluated at compile time.
const _: () = {
    let mut i = 1;
    while i < REGISTRY.len() {
        // strictly increasing => sorted AND unique in one check.
        if REGISTRY[i].id <= REGISTRY[i - 1].id {
            panic!("REGISTRY must be sorted by id and have no duplicate id");
        }
        i += 1;
    }
};

/// Look up a field by id (a `binary_search` on the sorted table). An unknown id returns `None`: an
/// unknown key is skipped, not fatal.
#[inline]
pub fn lookup(field_id: u8) -> Option<&'static FieldDef> {
    REGISTRY
        .binary_search_by_key(&field_id, |d| d.id)
        .ok()
        .map(|i| &REGISTRY[i])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookup_finds_known_and_misses_unknown() {
        assert_eq!(lookup(0x01).map(|d| d.id), Some(0x01));
        assert_eq!(lookup(T_FIELD_ID).map(|d| d.id), Some(T_FIELD_ID));
        // An unknown id is None (skipped, not fatal).
        assert!(lookup(0x77).is_none());
    }

    #[test]
    fn test_field_is_persistent_u32() {
        let d = lookup(T_FIELD_ID).unwrap();
        assert_eq!(d.kind, Type::U32);
        assert_eq!(d.persist, Persist::Persistent);
    }

    #[test]
    fn registry_ids_are_sorted_and_unique() {
        // The const assertion guarantees this at build; re-check at runtime for a clear failure site.
        for w in REGISTRY.windows(2) {
            assert!(w[0].id < w[1].id, "ids must strictly increase");
        }
    }
}
