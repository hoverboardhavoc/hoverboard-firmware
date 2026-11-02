//! The boot status record (`specs/configured-links.md` section 5): which configured peripheral did
//! not come up. The configuration itself is checked before it is written
//! (`specs/config-read-once.md` section 4), so the one reason left is a runtime fact.
//!
//! One record in RAM, written during boot and fixed after it. The firmware places it at the
//! `BOARD_OBS` symbol, where an SWD read finds it, and serves its entries over the link in the
//! `STATUS` reply to `STATUS_READ` (`specs/machine-settings.md` section 6.1). No entries means
//! everything configured came up. The host tools decode the same bytes with [`decode`].

/// The record's magic, `"BSTS"` little-endian: a read of `BOARD_OBS` that does not start with it
/// is not a record this image wrote.
pub const MAGIC: u32 = 0x5354_5342;

/// The most entries the record holds. Fourteen four-byte entries are 56 B, which with the
/// `CONFIG_RESP` header is one reply (`specs/configured-links.md` section 5).
pub const MAX_ENTRIES: usize = 14;

/// The bytes of one entry: the field id that names the function, the index, the [`Reason`], and
/// the reason's detail byte.
pub const ENTRY_LEN: usize = 4;

/// Why a function is absent: byte 2 of an entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Reason {
    /// The peripheral did not come up: a HAL constructor refused it, or the IMU did not answer its
    /// probe. Detail: the HAL's error code (`DescriptorError::code()`), or 0 for a probe that did
    /// not answer. The number is the one this reason had beside the configuration reasons that went
    /// (1 to 5 and 7), so a reader that decodes it needs no renumbering.
    NotUp = 6,
}

impl Reason {
    /// The reason a byte names, or `None` for a byte no variant has.
    pub const fn from_u8(b: u8) -> Option<Reason> {
        match b {
            6 => Some(Reason::NotUp),
            _ => None,
        }
    }
}

/// The `self_hold_port` of a boot that drove no power latch.
pub const NO_PORT: u8 = 0xFF;

/// The record, as it sits in RAM at `BOARD_OBS`: the magic, the power-latch pin this boot drove as
/// its port and pin ([`NO_PORT`] = none), the entry count, then the entries. `count` says how many
/// entries were recorded, which can exceed [`MAX_ENTRIES`]: entries past the fourteenth are
/// dropped and only counted.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StatusRecord {
    /// [`MAGIC`].
    pub magic: u32,
    /// The port of the power-latch pin this boot drove high (`0` = A), or [`NO_PORT`].
    pub self_hold_port: u8,
    /// Its pin number.
    pub self_hold_pin: u8,
    /// How many entries were recorded (saturating at 255).
    pub count: u8,
    /// The first [`MAX_ENTRIES`] entries, [`ENTRY_LEN`] bytes each.
    pub entries: [u8; MAX_ENTRIES * ENTRY_LEN],
}

impl Default for StatusRecord {
    fn default() -> Self {
        Self::new()
    }
}

impl StatusRecord {
    /// An empty record: nothing recorded, no latch driven.
    pub const fn new() -> Self {
        StatusRecord {
            magic: MAGIC,
            self_hold_port: NO_PORT,
            self_hold_pin: 0,
            count: 0,
            entries: [0; MAX_ENTRIES * ENTRY_LEN],
        }
    }

    /// Add one entry: the function named by `field` (its group's first field id, or its own) at
    /// `index` is absent for `reason`, with `detail`.
    pub fn record(&mut self, field: u8, index: u8, reason: Reason, detail: u8) {
        if let Some(e) = self
            .entries
            .as_chunks_mut::<ENTRY_LEN>()
            .0
            .get_mut(self.count as usize)
        {
            *e = [field, index, reason as u8, detail];
        }
        self.count = self.count.saturating_add(1);
    }

    /// Whether anything was recorded.
    pub const fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// The recorded entries as bytes, [`ENTRY_LEN`] per entry: what `STATUS` carries.
    pub fn entries(&self) -> &[u8] {
        let n = (self.count as usize).min(MAX_ENTRIES);
        self.entries.get(..n * ENTRY_LEN).unwrap_or(&[])
    }
}

/// One decoded entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Entry {
    /// The field id that names the function.
    pub field: u8,
    /// The index (the motor or the link; 0 for single fields).
    pub index: u8,
    /// The reason byte; [`Reason::from_u8`] names it.
    pub reason: u8,
    /// The reason's detail byte.
    pub detail: u8,
}

/// Decode a `STATUS` payload into its entries. A trailing partial entry is ignored.
pub fn decode(bytes: &[u8]) -> impl Iterator<Item = Entry> + '_ {
    bytes
        .as_chunks::<ENTRY_LEN>()
        .0
        .iter()
        .map(|&[field, index, reason, detail]| Entry {
            field,
            index,
            reason,
            detail,
        })
}
