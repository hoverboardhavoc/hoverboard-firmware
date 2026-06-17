//! Named data items and the `ItemSet` bitmask.
//!
//! `DataItem` names the link's data items (attitude, wheel speed, drive command, inputs, telemetry,
//! fault). `ItemSet` is a small `u16` bitmask over them. The same bitmask is reused as the
//! `NodeHello.caps` value, so config and node both build on it; it lives here because both will use
//! it.

/// A named data item carried over a link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataItem {
    Attitude,
    WheelSpeed,
    DriveCmd,
    Inputs,
    Telemetry,
    Fault,
    /// The cyclic-state status byte (rider-present, stationary, balance-engaged, fault-pending bits).
    /// Carried in the `CyclicState` frame alongside attitude and wheel speed; bound independently so a
    /// node can advertise it in produce/consume.
    Status,
}

impl DataItem {
    /// The single-bit mask for this item within an `ItemSet`.
    #[inline]
    fn bit(self) -> u16 {
        let shift = match self {
            DataItem::Attitude => 0,
            DataItem::WheelSpeed => 1,
            DataItem::DriveCmd => 2,
            DataItem::Inputs => 3,
            DataItem::Telemetry => 4,
            DataItem::Fault => 5,
            DataItem::Status => 6,
        };
        1u16 << shift
    }
}

/// A bitmask over `DataItem`. Reused as `NodeHello.caps`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ItemSet(pub u16);

impl ItemSet {
    /// An empty set.
    #[inline]
    pub const fn empty() -> ItemSet {
        ItemSet(0)
    }

    /// Build directly from a raw `u16` mask (e.g. a decoded `caps` field).
    #[inline]
    pub const fn from_bits(bits: u16) -> ItemSet {
        ItemSet(bits)
    }

    /// The raw `u16` mask.
    #[inline]
    pub const fn bits(self) -> u16 {
        self.0
    }

    /// Add `item` to the set.
    #[inline]
    pub fn insert(&mut self, item: DataItem) {
        self.0 |= item.bit();
    }

    /// True if `item` is present.
    #[inline]
    pub fn contains(self, item: DataItem) -> bool {
        self.0 & item.bit() != 0
    }

    /// The union of two sets.
    #[inline]
    pub fn union(self, other: ItemSet) -> ItemSet {
        ItemSet(self.0 | other.0)
    }
}

#[cfg(test)]
mod tests {
    use super::{DataItem, ItemSet};

    #[test]
    fn insert_contains() {
        let mut s = ItemSet::empty();
        assert!(!s.contains(DataItem::Attitude));
        s.insert(DataItem::Attitude);
        s.insert(DataItem::Fault);
        assert!(s.contains(DataItem::Attitude));
        assert!(s.contains(DataItem::Fault));
        assert!(!s.contains(DataItem::WheelSpeed));
    }

    #[test]
    fn union_combines() {
        let mut a = ItemSet::empty();
        a.insert(DataItem::Attitude);
        let mut b = ItemSet::empty();
        b.insert(DataItem::Telemetry);
        let u = a.union(b);
        assert!(u.contains(DataItem::Attitude));
        assert!(u.contains(DataItem::Telemetry));
    }

    #[test]
    fn bits_round_trip() {
        let mut s = ItemSet::empty();
        s.insert(DataItem::DriveCmd);
        s.insert(DataItem::Inputs);
        assert_eq!(ItemSet::from_bits(s.bits()), s);
    }
}
