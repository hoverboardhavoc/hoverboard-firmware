// SPDX-License-Identifier: GPL-3.0-or-later
//! The field-registry fingerprint the mailbox header publishes at
//! [`layout::REGISTRY_HASH`](crate::layout::REGISTRY_HASH).
//!
//! **What it answers.** A host tool carries its own COMPILED copy of the store field registry, and
//! before this word there was no way to tell "the board does not have this field" from "this tool
//! is older than the image". A `swd-mailbox-config` binary predating field `0x67` reported
//! `FAIL: unknown field id 0x67 (not in the store registry)` against a board that holds the field:
//! the tool blamed the board for its own staleness. `MAGIC` / `VERSION` cannot answer it, because
//! they describe the RING LAYOUT and have not moved since the crate was written.
//!
//! **One source, two readers.** [`REGISTRY_HASH`] is folded at compile time from
//! [`store::REGISTRY`], the same table [`store::lookup`] answers from. The firmware writes it into
//! the header at boot ([`Mailbox::init_header`](crate::Mailbox::init_header)); the host reads the
//! word at attach and compares it against the constant ITS build folded from ITS copy of the same
//! table. Two builds of the same source agree by construction, and the only way they differ is the
//! thing the word exists to report. Nothing is hand-maintained, which is the defect this fixes
//! rather than reproduces.
//!
//! **What goes into it, and what does not.** Each row contributes its `field_id`, its `index` and
//! its type tag: exactly the facts a tool and an image must agree on to exchange a `CONFIG_WRITE`
//! (which fields exist, which indices they declare, and how a value is encoded for them). The
//! row's DEFAULT is excluded. A default is not part of that agreement (a tool never asserts a
//! board's default; it reads the board's value), and including it would fire the warning on every
//! tuning change, which is how a warning stops being read. The row count is folded in at the end so
//! a registry that only lost or gained rows cannot collide.
//!
//! FNV-1a over those bytes: a few lines of `const fn`, no dependency, and it costs the image
//! nothing at runtime because the whole fold happens at compile time.

use store::{FieldEntry, REGISTRY};

/// This build's fingerprint of [`store::REGISTRY`]. The firmware publishes it in the mailbox
/// header; a host tool compares the word it reads against this constant.
pub const REGISTRY_HASH: u32 = registry_hash(&REGISTRY);

const FNV_OFFSET: u32 = 0x811c_9dc5;
const FNV_PRIME: u32 = 0x0100_0193;

const fn fold(h: u32, b: u8) -> u32 {
    (h ^ b as u32).wrapping_mul(FNV_PRIME)
}

/// Fold a registry into its fingerprint: `field_id`, `index` and type tag per row, then the row
/// count. Takes the table as an argument so a test can fold a deliberately-altered one and show
/// that the value moves.
pub const fn registry_hash(entries: &[FieldEntry]) -> u32 {
    let mut h = FNV_OFFSET;
    let mut i = 0;
    while i < entries.len() {
        let e = entries[i];
        h = fold(h, e.field_id);
        h = fold(h, e.index);
        h = fold(h, e.kind.tag());
        i += 1;
    }
    let n = entries.len();
    fold(fold(h, n as u8), (n >> 8) as u8)
}
