//! SWD RAM-mailbox L2 transport (`specs/swd-mailbox.md`): an L2 byte-stream link whose carrier is a
//! fixed RAM mailbox, read and written over SWD MEM-AP **while the core runs** - not a UART, not BLE.
//!
//! This crate is the HAL-free Tier-1 core:
//! - [`Mailbox`] - the header + two single-producer/single-consumer byte rings at a fixed RAM base,
//!   with the SPSC produce/consume discipline (payload written **first**, then the `head` commit, with
//!   a real `DMB` barrier on the firmware producer; all accesses `volatile`).
//! - [`MailboxSerial`] - an `embedded-io` `Read`/`Write`/`ReadReady` over the two rings, one endpoint
//!   per role ([`Role::Firmware`] drains `h2t` and fills `t2h`; [`Role::Bridge`] is the mirror). Wrap
//!   it in `link`'s [`SerialTransport`](link::SerialTransport) and the existing `StreamFramer` carries
//!   `l2.md` frames over the rings unchanged.
//! - [`EpochWatch`] - the firmware-side epoch poll: on a bumped `epoch` it flushes the inbound ring and
//!   the caller resets the framer, so a stale partial frame from a previous bridge session is dropped.
//! - [`Bridge`] - the host-side attach: validate, bump `epoch`, discard stale outbound, await the
//!   firmware flush ack.
//!
//! Framing is **not** reinvented here: it is `link`'s `StreamFramer` over [`SerialTransport`], reused
//! verbatim (the mailbox is just another byte-stream carrier).
//!
//! [`SerialTransport`]: link::SerialTransport

#![no_std]
// The host test harness needs std (Box-backed mock RAM, Vec); the crate itself is no_std.
#[cfg(test)]
extern crate std;

mod serial;

pub use serial::MailboxSerial;

// ---------------------------------------------------------------------------------------------------
// Layout (`specs/swd-mailbox.md`, "Memory layout"). A header of `u32` words, then the two ring data
// buffers. All fields little-endian, word-aligned (MEM-AP is word access). Free-running byte counters:
// the slot is `index & (cap - 1)`, `used = head - tail`, `free = cap - used`; the `u32` difference
// wraps cleanly.
// ---------------------------------------------------------------------------------------------------

/// The fixed, version-stable mailbox base address: the bottom of the smallest part's 8 KB SRAM, valid
/// on every target. The firmware reserves `[BASE, BASE + REGION_LEN)` by starting its linked RAM above
/// it (a reserved-front carve, the `store-test` reserved-region idiom) and writes the header there; the
/// host bridge reads it cold over MEM-AP. A fixed ABI constant, not an `nm`-found symbol.
pub const MAILBOX_BASE: u32 = 0x2000_0000;

/// `"MBX1"` little-endian - the bridge reads this first to confirm the block.
pub const MAGIC: u32 = u32::from_le_bytes(*b"MBX1");
/// Layout version (currently 1; grow backward-compatibly).
pub const VERSION: u32 = 1;
/// Ring capacity in bytes. A power of two so `index & (cap - 1)` is the slot; 256 per the spec.
pub const RING_CAP: u32 = 256;

// Header word byte offsets.
const O_MAGIC: usize = 0;
const O_VERSION: usize = 4;
const O_EPOCH: usize = 8;
const O_EPOCH_ACK: usize = 12;
const O_H2T_OFF: usize = 16;
const O_H2T_CAP: usize = 20;
const O_H2T_HEAD: usize = 24;
const O_H2T_TAIL: usize = 28;
const O_T2H_OFF: usize = 32;
const O_T2H_CAP: usize = 36;
const O_T2H_HEAD: usize = 40;
const O_T2H_TAIL: usize = 44;

/// Header size in bytes (12 `u32` words).
pub const HEADER_LEN: usize = 48;

/// The header field byte offsets - the on-wire ABI. The pointer-based [`Mailbox`] uses the private
/// `O_*` consts internally; these are the **public** view a host bridge needs to address the same
/// fields over MEM-AP (it accesses the mailbox by `read32`/`write32` at `base + offset`, not a local
/// pointer, so it cannot use [`Mailbox`]). The const-asserts below keep the two in lockstep.
pub mod layout {
    /// `magic` ("MBX1").
    pub const MAGIC: usize = 0;
    /// `version`.
    pub const VERSION: usize = 4;
    /// `epoch` (the bridge bumps it on attach).
    pub const EPOCH: usize = 8;
    /// `epoch_ack` (the firmware writes it back after flushing).
    pub const EPOCH_ACK: usize = 12;
    /// `h2t_off` (host->target ring data offset).
    pub const H2T_OFF: usize = 16;
    /// `h2t_cap`.
    pub const H2T_CAP: usize = 20;
    /// `h2t_head` (producer = bridge).
    pub const H2T_HEAD: usize = 24;
    /// `h2t_tail` (consumer = firmware).
    pub const H2T_TAIL: usize = 28;
    /// `t2h_off` (target->host ring data offset).
    pub const T2H_OFF: usize = 32;
    /// `t2h_cap`.
    pub const T2H_CAP: usize = 36;
    /// `t2h_head` (producer = firmware).
    pub const T2H_HEAD: usize = 40;
    /// `t2h_tail` (consumer = bridge).
    pub const T2H_TAIL: usize = 44;
}

// The public ABI offsets must match the private ones the pointer `Mailbox` uses.
const _: () = {
    assert!(layout::MAGIC == O_MAGIC);
    assert!(layout::VERSION == O_VERSION);
    assert!(layout::EPOCH == O_EPOCH);
    assert!(layout::EPOCH_ACK == O_EPOCH_ACK);
    assert!(layout::H2T_OFF == O_H2T_OFF);
    assert!(layout::H2T_CAP == O_H2T_CAP);
    assert!(layout::H2T_HEAD == O_H2T_HEAD);
    assert!(layout::H2T_TAIL == O_H2T_TAIL);
    assert!(layout::T2H_OFF == O_T2H_OFF);
    assert!(layout::T2H_CAP == O_T2H_CAP);
    assert!(layout::T2H_HEAD == O_T2H_HEAD);
    assert!(layout::T2H_TAIL == O_T2H_TAIL);
};

/// Offset of the `h2t` (host -> target) ring data buffer from the base.
pub const H2T_DATA_OFF: usize = HEADER_LEN;
/// Offset of the `t2h` (target -> host) ring data buffer from the base.
pub const T2H_DATA_OFF: usize = HEADER_LEN + RING_CAP as usize;
/// Total bytes the mailbox region occupies (header + both ring buffers).
pub const REGION_LEN: usize = HEADER_LEN + 2 * RING_CAP as usize;

/// The L2 frame capacity the mailbox advertises. It is **below the ring size** so a whole stream frame
/// (`frame_capacity` + [`STREAM_OVERHEAD`]) always fits the 256-byte ring at once and the cooperative
/// firmware producer never needs partial-write backpressure (`specs/swd-mailbox.md`, "What the rings
/// carry"). Config / L3 frames are tiny, so 128 is generous and still leaves room for a second frame.
pub const FRAME_CAPACITY: usize = 128;

/// Bytes a [`SerialTransport`](link::SerialTransport) stream frame adds around an L2 frame: `SOF` +
/// `len` + 2-byte CRC.
pub const STREAM_OVERHEAD: usize = link::framer::STREAM_HEADER_LEN + link::framer::STREAM_CRC_LEN;

// A whole stream frame must fit the ring at once (the spec's frame_capacity-below-ring invariant).
const _: () = assert!(FRAME_CAPACITY + STREAM_OVERHEAD <= RING_CAP as usize);
// The slot index `head & (cap - 1)` requires a power-of-two capacity.
const _: () = assert!(RING_CAP.is_power_of_two());

/// One SPSC ring's fixed location in the header + data area.
#[derive(Clone, Copy)]
pub(crate) struct RingRef {
    /// Data-buffer offset from the base.
    off: usize,
    /// Capacity in bytes (power of two).
    cap: u32,
    /// Header offset of the producer's `head` word.
    head_off: usize,
    /// Header offset of the consumer's `tail` word.
    tail_off: usize,
}

pub(crate) const H2T: RingRef = RingRef {
    off: H2T_DATA_OFF,
    cap: RING_CAP,
    head_off: O_H2T_HEAD,
    tail_off: O_H2T_TAIL,
};
pub(crate) const T2H: RingRef = RingRef {
    off: T2H_DATA_OFF,
    cap: RING_CAP,
    head_off: O_T2H_HEAD,
    tail_off: O_T2H_TAIL,
};

/// The barrier issued between the payload writes and the index commit.
#[derive(Clone, Copy)]
pub(crate) enum Commit {
    /// A real `DMB` (the firmware side): a **second bus master** (the debug AP) reads the ring, and the
    /// Cortex-M3 write path buffers SRAM stores, so a hardware barrier (not a compiler fence) is what
    /// guarantees the data has drained before the new `head` is visible (`specs/swd-mailbox.md`, "Ring
    /// discipline").
    Hardware,
    /// A compiler fence (the bridge side): MEM-AP transfers are issued in order, so its ordering is
    /// automatic; only the compiler must be kept from reordering the local stores.
    Compiler,
}

impl Commit {
    #[inline(always)]
    fn barrier(self) {
        match self {
            Commit::Hardware => dmb(),
            Commit::Compiler => {
                core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst)
            }
        }
    }
}

/// A real data memory barrier on the chip; a host-equivalent `fence` off it (the host has no second
/// bus master, so a `SeqCst` fence is a faithful stand-in for the test).
#[inline(always)]
fn dmb() {
    #[cfg(target_arch = "arm")]
    cortex_m::asm::dmb();
    #[cfg(not(target_arch = "arm"))]
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}

/// Which end of the link an endpoint is, and therefore which ring it produces / consumes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Role {
    /// The board: consumes the inbound `h2t` ring, produces the outbound `t2h` ring, and commits with
    /// a real `DMB`.
    Firmware,
    /// The host/debugger: the mirror - consumes `t2h`, produces `h2t`, commits with a compiler fence.
    Bridge,
}

impl Role {
    pub(crate) fn inbound(self) -> RingRef {
        match self {
            Role::Firmware => H2T,
            Role::Bridge => T2H,
        }
    }
    pub(crate) fn outbound(self) -> RingRef {
        match self {
            Role::Firmware => T2H,
            Role::Bridge => H2T,
        }
    }
    pub(crate) fn commit(self) -> Commit {
        match self {
            Role::Firmware => Commit::Hardware,
            Role::Bridge => Commit::Compiler,
        }
    }
}

/// A handle to the mailbox at a fixed RAM base. All header and data accesses are `volatile` (the
/// compiler re-reads `head`/`tail` each poll and never hoists or elides them - the repo's fixed-RAM
/// idiom). `Copy`, because it is just a base pointer: the firmware endpoint and the epoch watcher hold
/// independent handles onto the same shared region (that sharing is the whole point).
#[derive(Clone, Copy)]
pub struct Mailbox {
    base: *mut u8,
}

impl Mailbox {
    /// Wrap a fixed base address as a mailbox.
    ///
    /// # Safety
    /// `base` must be a valid, 4-byte-aligned pointer to at least [`REGION_LEN`] bytes that live for
    /// as long as this handle (and any [`MailboxSerial`] / [`EpochWatch`] / [`Bridge`] derived from
    /// it) is used. On silicon it is the linker-fixed mailbox base; in a host test it is a 4-aligned
    /// backing buffer.
    pub unsafe fn from_raw(base: *mut u8) -> Self {
        Mailbox { base }
    }

    #[inline]
    fn read_word(&self, off: usize) -> u32 {
        // SAFETY: `off` is a header word offset within the region, 4-byte aligned by construction.
        unsafe { (self.base.add(off) as *const u32).read_volatile() }
    }

    #[inline]
    fn write_word(&self, off: usize, v: u32) {
        // SAFETY: as `read_word`.
        unsafe { (self.base.add(off) as *mut u32).write_volatile(v) }
    }

    /// `magic`, read first by the bridge to confirm the block.
    pub fn magic(&self) -> u32 {
        self.read_word(O_MAGIC)
    }
    /// The layout `version`.
    pub fn version(&self) -> u32 {
        self.read_word(O_VERSION)
    }
    /// The session `epoch` (the bridge bumps it on attach).
    pub fn epoch(&self) -> u32 {
        self.read_word(O_EPOCH)
    }

    /// The `epoch_ack` the firmware has written back (it equals `epoch` once the firmware has observed
    /// and flushed that session). The bridge spins until `epoch_ack == epoch` before producing.
    pub fn epoch_ack(&self) -> u32 {
        self.read_word(O_EPOCH_ACK)
    }

    /// `magic` + `version` both match this build's ABI.
    pub fn is_valid(&self) -> bool {
        self.magic() == MAGIC && self.version() == VERSION
    }

    /// Firmware boot initialization. A mailbox region is neither `.data` (copied) nor `.bss` (zeroed),
    /// so its contents are indeterminate at reset: the firmware must write `magic` / `version` / the
    /// `*_off` / `*_cap` fields and **zero the four ring indices** (and `epoch` / `epoch_ack`) before
    /// any bridge attaches (`specs/swd-mailbox.md`, "The firmware initializes the header at boot").
    pub fn init_header(&self) {
        self.write_word(O_MAGIC, MAGIC);
        self.write_word(O_VERSION, VERSION);
        self.write_word(O_EPOCH, 0);
        self.write_word(O_EPOCH_ACK, 0);
        self.write_word(O_H2T_OFF, H2T_DATA_OFF as u32);
        self.write_word(O_H2T_CAP, RING_CAP);
        self.write_word(O_H2T_HEAD, 0);
        self.write_word(O_H2T_TAIL, 0);
        self.write_word(O_T2H_OFF, T2H_DATA_OFF as u32);
        self.write_word(O_T2H_CAP, RING_CAP);
        self.write_word(O_T2H_HEAD, 0);
        self.write_word(O_T2H_TAIL, 0);
    }

    /// Bytes currently queued in a ring: `head - tail` (the `u32` difference wraps cleanly).
    pub(crate) fn used(&self, r: RingRef) -> u32 {
        self.read_word(r.head_off)
            .wrapping_sub(self.read_word(r.tail_off))
    }

    /// Free space in a ring: `cap - used`. (The producer computes free inline; this is the named
    /// accessor the ring tests assert against.)
    #[cfg(test)]
    pub(crate) fn free(&self, r: RingRef) -> u32 {
        r.cap - self.used(r)
    }

    /// Producer: write the payload bytes **first**, then the barrier, then advance `head` (the commit).
    /// A reader that sees the new `head` is guaranteed the bytes under it are there. Writes at most the
    /// free space and returns the count written (a short or zero return is partial-write backpressure;
    /// the cooperative producer keeps a whole frame within the ring so it never bites here).
    pub(crate) fn produce(&self, r: RingRef, src: &[u8], commit: Commit) -> usize {
        let mask = r.cap - 1;
        let tail = self.read_word(r.tail_off);
        let mut head = self.read_word(r.head_off);
        let mut free = r.cap - head.wrapping_sub(tail);
        let mut written = 0;
        for &b in src {
            if free == 0 {
                break;
            }
            let slot = (head & mask) as usize;
            // SAFETY: `slot < cap` and `r.off + slot` is within the region's data buffer.
            unsafe { self.base.add(r.off + slot).write_volatile(b) };
            head = head.wrapping_add(1);
            free -= 1;
            written += 1;
        }
        commit.barrier();
        self.write_word(r.head_off, head); // the commit, after the payload + barrier
        written
    }

    /// Consumer: read up to `dst.len()` bytes (bounded by `used`), then the barrier, then advance
    /// `tail` to release the space. Returns the count read.
    pub(crate) fn consume(&self, r: RingRef, dst: &mut [u8], commit: Commit) -> usize {
        let mask = r.cap - 1;
        let head = self.read_word(r.head_off);
        let mut tail = self.read_word(r.tail_off);
        let avail = head.wrapping_sub(tail) as usize;
        let n = core::cmp::min(dst.len(), avail);
        for d in dst.iter_mut().take(n) {
            let slot = (tail & mask) as usize;
            // SAFETY: `slot < cap` and `r.off + slot` is within the region's data buffer.
            *d = unsafe { self.base.add(r.off + slot).read_volatile() };
            tail = tail.wrapping_add(1);
        }
        commit.barrier(); // reads complete before tail release (the producer must not overwrite them)
        self.write_word(r.tail_off, tail);
        n
    }

    /// Flush a ring **as its consumer**: `tail := head`, dropping everything unread. Only the consumer
    /// writes `tail`, so this never violates the SPSC "don't write the other side's index" rule.
    pub(crate) fn flush_consumer(&self, r: RingRef) {
        self.write_word(r.tail_off, self.read_word(r.head_off));
    }

    #[cfg(test)]
    pub(crate) fn head(&self, r: RingRef) -> u32 {
        self.read_word(r.head_off)
    }

    #[cfg(test)]
    pub(crate) fn tail(&self, r: RingRef) -> u32 {
        self.read_word(r.tail_off)
    }

    /// The committed data byte at free-running `index` (its slot is `index & (cap - 1)`).
    #[cfg(test)]
    pub(crate) fn data_byte(&self, r: RingRef, index: u32) -> u8 {
        let slot = (index & (r.cap - 1)) as usize;
        // SAFETY: `slot < cap`, within the ring's data buffer.
        unsafe { self.base.add(r.off + slot).read_volatile() }
    }
}

/// The firmware-side epoch poll. The firmware polls [`EpochWatch::poll`] from its scheduler; on a
/// changed `epoch` (a new bridge session) it flushes the inbound `h2t` ring (`h2t_tail := h2t_head`)
/// and returns `true`. The caller then resets the byte-stream framer
/// ([`SerialTransport::reset`](link::SerialTransport::reset)) and calls [`EpochWatch::ack`] to write
/// `epoch_ack := epoch` - in that order, per `specs/swd-mailbox.md` ("Attach + session flush"): flush
/// the ring, reset the framer, *then* acknowledge, so the bridge (which waits for `epoch_ack == epoch`)
/// only starts producing once the firmware is fully reset. The framer resync is a second line of defence.
pub struct EpochWatch {
    mb: Mailbox,
    last_epoch: u32,
}

impl EpochWatch {
    /// Start watching from the current `epoch`.
    pub fn new(mb: Mailbox) -> Self {
        EpochWatch {
            mb,
            last_epoch: mb.epoch(),
        }
    }

    /// Poll for a new bridge session. On a changed `epoch`, flush the inbound ring and return `true`
    /// (the caller then resets its framer and calls [`EpochWatch::ack`]). Otherwise return `false`.
    pub fn poll(&mut self) -> bool {
        let e = self.mb.epoch();
        if e != self.last_epoch {
            self.last_epoch = e;
            self.mb.flush_consumer(H2T); // firmware is the h2t consumer
            true
        } else {
            false
        }
    }

    /// Acknowledge the observed-and-flushed session: write `epoch_ack := epoch`. Called **after** the
    /// framer is reset, so the bridge's `epoch_ack == epoch` wait is satisfied only once the firmware
    /// is fully ready for the new session's first frame.
    pub fn ack(&self) {
        self.mb.write_word(O_EPOCH_ACK, self.last_epoch);
    }
}

/// Why [`Bridge::attach`] declined.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttachError {
    /// `magic` / `version` did not match: not our firmware, not running, or the wrong address. The
    /// bridge reports and does **not** write.
    Invalid,
}

/// The host-side attach (`specs/swd-mailbox.md`, "Attach + session flush"). Validates the header, bumps
/// `epoch` to mark a new session, and discards any stale **outbound** by setting `t2h_tail := t2h_head`
/// (the bridge's own consumer index). The caller then awaits the firmware's flush ack
/// ([`Bridge::flush_acked`]) before producing its first inbound frame, so the firmware's `h2t` flush
/// cannot eat that first frame (the attach race).
pub struct Bridge {
    mb: Mailbox,
    /// The epoch this session bumped to; the firmware acks by writing `epoch_ack := session_epoch`.
    session_epoch: u32,
}

impl Bridge {
    /// Validate and start a fresh session. Returns [`AttachError::Invalid`] without writing on a
    /// `magic`/`version` mismatch.
    pub fn attach(mb: Mailbox) -> Result<Self, AttachError> {
        if !mb.is_valid() {
            return Err(AttachError::Invalid);
        }
        let session_epoch = mb.epoch().wrapping_add(1);
        mb.write_word(O_EPOCH, session_epoch); // bump epoch: a new session
        mb.flush_consumer(T2H); // discard stale outbound (the bridge is the t2h consumer)
        Ok(Bridge { mb, session_epoch })
    }

    /// True once the firmware has acked this session (`epoch_ack == epoch`). The bridge spins on this
    /// before producing its first inbound frame. **Not** `h2t_tail == h2t_head`: an already-empty ring
    /// reads "flushed" before the firmware has even observed the new epoch, re-opening the attach race;
    /// `epoch_ack` is written only *after* the firmware flushes, so it is unambiguous.
    pub fn flush_acked(&self) -> bool {
        self.mb.epoch_ack() == self.session_epoch
    }

    /// The epoch this session bumped to.
    pub fn session_epoch(&self) -> u32 {
        self.session_epoch
    }

    /// The mailbox handle (for building this bridge's [`MailboxSerial`]).
    pub fn mailbox(&self) -> Mailbox {
        self.mb
    }
}

#[cfg(test)]
mod tests;
