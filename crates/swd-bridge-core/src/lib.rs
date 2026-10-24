// SPDX-License-Identifier: GPL-3.0-or-later
//! The bridge end of the SWD RAM mailbox (`specs/swd-mailbox.md`), `no_std`.
//!
//! The firmware owns the mailbox in RAM and services it by polling; this crate is the **other end**,
//! reading and writing the same rings through a [`MemAp`] (an SWD memory access port while the core
//! runs, or an in-RAM buffer in tests). It cannot use the firmware's pointer-based
//! `swd_mailbox::Mailbox` (the target's RAM is not in this side's address space), so it implements
//! the bridge side of the SPSC ring discipline over the [`MemAp`] seam:
//!
//! - the bridge is the **`h2t` producer** ([`HostMailbox::produce`]) and the **`t2h` consumer**
//!   ([`HostMailbox::consume`]), the mirror of the firmware `MailboxSerial`;
//! - it implements the **attach + epoch handshake** ([`HostMailbox::attach`]): validate `magic`/
//!   `version`, bump `epoch`, discard stale outbound (`t2h_tail := t2h_head`, the bridge's own index),
//!   then wait `epoch_ack == epoch` before producing (`specs/swd-mailbox.md`, "Attach race").
//!
//! [`BridgeSerial`] wraps it as an `embedded-io` serial (drain `t2h` on `Read`, fill `h2t` on
//! `Write`), so `link::SerialTransport` runs the same `StreamFramer` over it.
//!
//! Two users: `crates/swd-bridge` (the host tools, over OpenOCD's TCL RPC) and `apps/esp-tool` (the
//! ESP32 board tool, over its own SWD engine). Each brings its backend's error type through
//! [`MemAp::Error`] and its own way of waiting through [`HostMailbox::wait_flush_ack`]'s `pause`.
#![no_std]

use core::fmt;

use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write};
use swd_mailbox::{layout, MAGIC, RING_CAP, VERSION};

/// The mailbox region base address on the target (the fixed ABI base).
pub use swd_mailbox::MAILBOX_BASE;

/// A word/byte memory accessor over the SWD memory access port (or, in tests, in-RAM). All accesses
/// are **while-running**: no core halt. Addresses are absolute target addresses.
pub trait MemAp {
    /// What the backend fails with. `Clone` so a refused write can latch a copy
    /// ([`BridgeSerial::take_write_error`]).
    type Error: fmt::Debug + fmt::Display + Clone;
    /// Read one little-endian 32-bit word at `addr`.
    fn read32(&mut self, addr: u32) -> Result<u32, Self::Error>;
    /// Write one little-endian 32-bit word at `addr`.
    fn write32(&mut self, addr: u32, val: u32) -> Result<(), Self::Error>;
    /// Read `out.len()` bytes starting at `addr`.
    fn read(&mut self, addr: u32, out: &mut [u8]) -> Result<(), Self::Error>;
    /// Write `data` starting at `addr`.
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), Self::Error>;
}

/// What the bridge can fail with. `E` is the [`MemAp`] backend's error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BridgeError<E> {
    /// `magic` / `version` did not match: not our firmware, not running, or the wrong address.
    Invalid {
        /// The `magic` word read.
        magic: u32,
        /// The `version` word read.
        version: u32,
    },
    /// The firmware did not write `epoch_ack == epoch` within the attach budget.
    AckTimeout,
    /// The memory access backend failed.
    MemAp(E),
    /// The `h2t` ring had no room for a whole frame: the board is not draining its mailbox
    /// (`specs/swd-mailbox.md`, "Backpressure", requirement 6).
    ///
    /// The bridge end of the contract the firmware end has: never `Ok(0)` for a non-empty buffer,
    /// because `write_all` panics on exactly that. A halted or wedged core stops advancing
    /// `h2t_tail`, and a tool that panicked with "write() returned Ok(0)" reported nothing about why.
    NotDraining {
        /// Bytes unread in `h2t` **after** the refusal. On the ordinary path nothing was committed,
        /// so it is the occupancy the free check saw; on the premise-violation path a short prefix
        /// went in and the ring is full, which is [`RING_CAP`] by construction.
        used: u32,
        /// Bytes this refused write needed to place.
        needed: usize,
    },
    /// **A second bridge attached to this mailbox while this one owned it**: `epoch` has moved away
    /// from the value [`HostMailbox::attach`] set.
    ///
    /// A bridge attaching bumps `epoch`, and the firmware answers by FLUSHING the inbound ring
    /// (`h2t_tail := h2t_head`), so a second attach discards everything this session had queued. The
    /// requirement is one session OWNER per run (`specs/swd-mailbox.md`, "One session owner per
    /// run"). Sequential one-shots are unaffected: only an OVERLAP moves the epoch under a live owner.
    SessionTaken {
        /// The epoch this session attached as.
        ours: u32,
        /// The epoch found in the header, which the second attacher wrote.
        found: u32,
    },
    /// **A second producer wrote the `h2t` ring**: `h2t_head` moved between this session's writes,
    /// and only a producer writes `h2t_head` (the firmware advances `h2t_tail` alone). The companion
    /// to [`SessionTaken`](BridgeError::SessionTaken), for a writer that pokes the ring without
    /// attaching.
    SecondProducer {
        /// `h2t_head` as this session left it.
        expected: u32,
        /// `h2t_head` as it reads now.
        found: u32,
    },
}

impl<E> BridgeError<E> {
    /// The same error with the backend error mapped through `f` (a host tool folding a typed backend
    /// error into its own message type, for one).
    pub fn map_mem<F>(self, f: impl FnOnce(E) -> F) -> BridgeError<F> {
        match self {
            BridgeError::Invalid { magic, version } => BridgeError::Invalid { magic, version },
            BridgeError::AckTimeout => BridgeError::AckTimeout,
            BridgeError::MemAp(e) => BridgeError::MemAp(f(e)),
            BridgeError::NotDraining { used, needed } => BridgeError::NotDraining { used, needed },
            BridgeError::SessionTaken { ours, found } => BridgeError::SessionTaken { ours, found },
            BridgeError::SecondProducer { expected, found } => {
                BridgeError::SecondProducer { expected, found }
            }
        }
    }
}

impl<E: fmt::Display> fmt::Display for BridgeError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BridgeError::Invalid { magic, version } => write!(
                f,
                "mailbox header invalid (magic={magic:#010x}, version={version}): not our firmware, \
                 not running, or wrong address"
            ),
            BridgeError::AckTimeout => write!(f, "timed out waiting for epoch_ack == epoch"),
            BridgeError::MemAp(e) => write!(f, "MEM-AP error: {e}"),
            BridgeError::NotDraining { used, needed } => write!(
                f,
                "the board is not draining its mailbox: {used} of {RING_CAP} bytes unread in h2t, \
                 no room for this {needed}-byte frame (a halted or wedged core, or a firmware with \
                 no mailbox poll-site)"
            ),
            BridgeError::SessionTaken { ours, found } => write!(
                f,
                "a second mailbox session attached (epoch {ours} -> {found}): THIS run is the one \
                 losing, because the firmware flushed the inbound ring for the new session and \
                 everything this run had queued went with it. Run one mailbox owner at a time \
                 (swd-mailbox-session holds an arm and a demand together); sequential one-shots \
                 are fine, overlapping ones are not"
            ),
            BridgeError::SecondProducer { expected, found } => write!(
                f,
                "a second mailbox producer wrote the h2t ring (head {expected} -> {found} between \
                 this run's writes): two processes are feeding one mailbox and the frames of both \
                 are being lost. Run one mailbox owner at a time (swd-mailbox-session)"
            ),
        }
    }
}

impl<E: fmt::Debug + fmt::Display> core::error::Error for BridgeError<E> {}

// So `BridgeSerial` can be an `embedded-io` serial that `link::SerialTransport` drives.
impl<E: fmt::Debug> embedded_io::Error for BridgeError<E> {
    fn kind(&self) -> ErrorKind {
        match self {
            // The kind the `Write` contract prescribes for "not able to accept more bytes".
            BridgeError::NotDraining { .. } => ErrorKind::WriteZero,
            _ => ErrorKind::Other,
        }
    }
}

/// What this session owns of the mailbox, from [`HostMailbox::attach`] onward: the epoch it bumped
/// to and `h2t_head` as it last left it. Both are what a SECOND owner moves.
#[derive(Clone, Copy)]
struct Session {
    epoch: u32,
    expect_h2t_head: u32,
}

/// The bridge-side view of the mailbox over a [`MemAp`]. Holds the session it attached as.
pub struct HostMailbox<M: MemAp> {
    mem: M,
    base: u32,
    session: Option<Session>,
}

/// This module's result type over a backend `M`.
pub type Res<T, M> = Result<T, BridgeError<<M as MemAp>::Error>>;

impl<M: MemAp> HostMailbox<M> {
    /// A bridge over `mem`, with the mailbox at `base` (use [`MAILBOX_BASE`] on silicon).
    pub fn new(mem: M, base: u32) -> Self {
        HostMailbox {
            mem,
            base,
            session: None,
        }
    }

    /// Borrow the underlying accessor (e.g. for a second raw read in a test/validation).
    pub fn mem(&mut self) -> &mut M {
        &mut self.mem
    }

    /// Give the accessor back.
    pub fn into_mem(self) -> M {
        self.mem
    }

    fn rd(&mut self, off: usize) -> Res<u32, M> {
        self.mem
            .read32(self.base + off as u32)
            .map_err(BridgeError::MemAp)
    }
    fn wr(&mut self, off: usize, val: u32) -> Res<(), M> {
        self.mem
            .write32(self.base + off as u32, val)
            .map_err(BridgeError::MemAp)
    }

    /// `magic`.
    pub fn magic(&mut self) -> Res<u32, M> {
        self.rd(layout::MAGIC)
    }
    /// `version`.
    pub fn version(&mut self) -> Res<u32, M> {
        self.rd(layout::VERSION)
    }
    /// `epoch`.
    pub fn epoch(&mut self) -> Res<u32, M> {
        self.rd(layout::EPOCH)
    }
    /// `epoch_ack`.
    pub fn epoch_ack(&mut self) -> Res<u32, M> {
        self.rd(layout::EPOCH_ACK)
    }

    /// The session this bridge bumped `epoch` to (after [`attach`](HostMailbox::attach)); 0 before
    /// it has attached, which is the value the firmware's `init_header` leaves.
    pub fn session_epoch(&self) -> u32 {
        self.session.map_or(0, |s| s.epoch)
    }

    /// Whether this bridge has attached a session.
    pub fn attached(&self) -> bool {
        self.session.is_some()
    }

    /// Forget the session without touching the target: the board has reset or been reflashed, so
    /// the header it attached to is gone and the next step is a fresh [`attach`](Self::attach).
    pub fn detach(&mut self) {
        self.session = None;
    }

    /// The image's field-registry fingerprint, published in the header
    /// ([`swd_mailbox::REGISTRY_HASH`]).
    pub fn registry_hash(&mut self) -> Res<u32, M> {
        self.rd(layout::REGISTRY_HASH)
    }

    /// **Is this bridge still the mailbox's only session?** Reads `epoch` and compares it against
    /// the one [`attach`](HostMailbox::attach) bumped to; a difference is a second attach
    /// ([`BridgeError::SessionTaken`]). Before attach there is no session to lose, and this is
    /// `Ok(())`.
    pub fn check_session(&mut self) -> Res<(), M> {
        let Some(session) = self.session else {
            return Ok(());
        };
        let found = self.epoch()?;
        if found != session.epoch {
            return Err(BridgeError::SessionTaken {
                ours: session.epoch,
                found,
            });
        }
        Ok(())
    }

    /// Validate `magic` + `version` against this build's ABI.
    pub fn validate(&mut self) -> Res<(), M> {
        let magic = self.magic()?;
        let version = self.version()?;
        if magic != MAGIC || version != VERSION {
            return Err(BridgeError::Invalid { magic, version });
        }
        Ok(())
    }

    /// Attach a fresh session (`specs/swd-mailbox.md`, "Attach + session flush"): validate the
    /// header, **bump `epoch`**, and discard stale outbound by setting `t2h_tail := t2h_head` (the
    /// bridge's own consumer index). Does **not** wait for the ack; call
    /// [`wait_flush_ack`](HostMailbox::wait_flush_ack) (or poll [`flush_acked`](Self::flush_acked))
    /// before producing the first inbound frame. It also takes the two readings a second owner would
    /// move, `epoch` and `h2t_head`, for [`check_session`](Self::check_session) and
    /// [`produce`](Self::produce).
    pub fn attach(&mut self) -> Res<(), M> {
        self.validate()?;
        let new_epoch = self.epoch()?.wrapping_add(1);
        self.wr(layout::EPOCH, new_epoch)?;
        let t2h_head = self.rd(layout::T2H_HEAD)?;
        self.wr(layout::T2H_TAIL, t2h_head)?;
        let expect_h2t_head = self.rd(layout::H2T_HEAD)?;
        self.session = Some(Session {
            epoch: new_epoch,
            expect_h2t_head,
        });
        Ok(())
    }

    /// True once the firmware has acked this session (`epoch_ack == epoch`).
    pub fn flush_acked(&mut self) -> Res<bool, M> {
        Ok(self.epoch_ack()? == self.session_epoch())
    }

    /// Poll [`flush_acked`](Self::flush_acked) up to `attempts` times, calling `pause` between
    /// polls (the caller's delay: a thread sleep on a host, a busy wait or a timer on a target).
    /// `Err(AckTimeout)` if the firmware never acks, e.g. it has no mailbox poll-site.
    pub fn wait_flush_ack(&mut self, attempts: u32, mut pause: impl FnMut()) -> Res<(), M> {
        for _ in 0..attempts {
            if self.flush_acked()? {
                return Ok(());
            }
            pause();
        }
        Err(BridgeError::AckTimeout)
    }

    // ---- the h2t producer side (bridge -> firmware) ----

    /// `h2t_head` (the bridge's producer index).
    pub fn h2t_head(&mut self) -> Res<u32, M> {
        self.rd(layout::H2T_HEAD)
    }
    /// `h2t_tail` (the firmware's consumer index).
    pub fn h2t_tail(&mut self) -> Res<u32, M> {
        self.rd(layout::H2T_TAIL)
    }
    /// Bytes the firmware has not yet drained from `h2t`.
    pub fn h2t_used(&mut self) -> Res<u32, M> {
        Ok(self.h2t_head()?.wrapping_sub(self.h2t_tail()?))
    }

    /// Room left in `h2t`: `cap - used`. What [`BridgeSerial`]'s all-or-nothing write checks before
    /// it produces a byte.
    pub fn h2t_free(&mut self) -> Res<u32, M> {
        Ok(RING_CAP - self.h2t_used()?)
    }

    /// Produce up to `src.len()` bytes into the `h2t` ring (write the payload **first**, then advance
    /// `h2t_head`; memory access port transfers are issued in order, so no barrier is needed).
    /// Returns the count written (bounded by free space).
    ///
    /// A short or zero return is byte-stream partial-write backpressure, the right answer for a ring
    /// of bytes. Frames are kept whole one layer up: [`BridgeSerial`]'s write refuses rather than
    /// place a prefix. A caller that uses this directly must check the count itself.
    ///
    /// This end is the ring's only producer by contract, so the `h2t_head` read here is the one this
    /// session committed last; a different value means somebody else wrote the ring
    /// ([`BridgeError::SecondProducer`]), and nothing is produced on top of it.
    pub fn produce(&mut self, src: &[u8]) -> Res<usize, M> {
        let cap = RING_CAP;
        let head = self.h2t_head()?;
        if let Some(session) = self.session {
            if head != session.expect_h2t_head {
                return Err(BridgeError::SecondProducer {
                    expected: session.expect_h2t_head,
                    found: head,
                });
            }
        }
        let tail = self.h2t_tail()?;
        let free = cap - head.wrapping_sub(tail);
        let n = (src.len() as u32).min(free) as usize;
        self.write_ring(swd_mailbox::H2T_DATA_OFF as u32, head, &src[..n])?;
        let committed = head.wrapping_add(n as u32);
        self.wr(layout::H2T_HEAD, committed)?; // the commit, after the payload
        if let Some(session) = &mut self.session {
            session.expect_h2t_head = committed;
        }
        Ok(n)
    }

    // ---- the t2h consumer side (firmware -> bridge) ----

    /// `t2h_head` (the firmware's producer index).
    pub fn t2h_head(&mut self) -> Res<u32, M> {
        self.rd(layout::T2H_HEAD)
    }
    /// `t2h_tail` (the bridge's consumer index).
    pub fn t2h_tail(&mut self) -> Res<u32, M> {
        self.rd(layout::T2H_TAIL)
    }
    /// Bytes the firmware has produced into `t2h` and the bridge has not yet drained.
    pub fn t2h_used(&mut self) -> Res<u32, M> {
        Ok(self.t2h_head()?.wrapping_sub(self.t2h_tail()?))
    }

    /// Consume up to `dst.len()` bytes from the `t2h` ring (read, then advance `t2h_tail`). Returns
    /// the count read.
    pub fn consume(&mut self, dst: &mut [u8]) -> Res<usize, M> {
        let head = self.t2h_head()?;
        let tail = self.t2h_tail()?;
        let avail = head.wrapping_sub(tail);
        let n = (dst.len() as u32).min(avail) as usize;
        self.read_ring(swd_mailbox::T2H_DATA_OFF as u32, tail, &mut dst[..n])?;
        self.wr(layout::T2H_TAIL, tail.wrapping_add(n as u32))?;
        Ok(n)
    }

    /// Write `data` into a ring's data buffer starting at free-running `index`, handling the wrap at
    /// `cap` (one access for the run to the buffer end, a second for the wrapped remainder).
    fn write_ring(&mut self, data_off: u32, index: u32, data: &[u8]) -> Res<(), M> {
        if data.is_empty() {
            return Ok(());
        }
        let start = index & (RING_CAP - 1);
        let first = ((RING_CAP - start) as usize).min(data.len());
        self.mem
            .write(self.base + data_off + start, &data[..first])
            .map_err(BridgeError::MemAp)?;
        if first < data.len() {
            self.mem
                .write(self.base + data_off, &data[first..])
                .map_err(BridgeError::MemAp)?;
        }
        Ok(())
    }

    /// Read `out.len()` bytes from a ring's data buffer starting at free-running `index`, handling
    /// the wrap.
    fn read_ring(&mut self, data_off: u32, index: u32, out: &mut [u8]) -> Res<(), M> {
        if out.is_empty() {
            return Ok(());
        }
        let start = index & (RING_CAP - 1);
        let first = ((RING_CAP - start) as usize).min(out.len());
        self.mem
            .read(self.base + data_off + start, &mut out[..first])
            .map_err(BridgeError::MemAp)?;
        if first < out.len() {
            self.mem
                .read(self.base + data_off, &mut out[first..])
                .map_err(BridgeError::MemAp)?;
        }
        Ok(())
    }
}

/// The bridge end as an `embedded-io` serial: `Read` drains `t2h`, `Write` fills `h2t`, `ReadReady`
/// reports `t2h` occupancy. Wrap it in `link::SerialTransport` to carry `l2.md` frames: the mirror
/// of the firmware `swd_mailbox::MailboxSerial::bridge`.
pub struct BridgeSerial<M: MemAp> {
    mb: HostMailbox<M>,
    /// The first write error since the latch was last taken
    /// ([`take_write_error`](BridgeSerial::take_write_error)).
    write_error: Option<BridgeError<M::Error>>,
}

impl<M: MemAp> BridgeSerial<M> {
    /// Wrap an attached [`HostMailbox`] as a serial. Takes anything that converts into one, so a
    /// host tool's wrapper around the mailbox passes straight through.
    pub fn new(mb: impl Into<HostMailbox<M>>) -> Self {
        BridgeSerial {
            mb: mb.into(),
            write_error: None,
        }
    }
    /// Borrow the inner mailbox (indices / attach).
    pub fn mailbox(&mut self) -> &mut HostMailbox<M> {
        &mut self.mb
    }

    /// Take the latched write error, if a write has failed since this was last called.
    ///
    /// The latch exists because the layer above throws the error away: `SerialTransport::
    /// send_l2_frame` drops the frame on a write error, which is right for a board (L2 is
    /// best-effort) and wrong for a tool, where the dropped frame IS the command. It latches the
    /// FIRST error of a run, any backend failure included, not just [`BridgeError::NotDraining`].
    pub fn take_write_error(&mut self) -> Option<BridgeError<M::Error>> {
        self.write_error.take()
    }
}

impl<M: MemAp> ErrorType for BridgeSerial<M> {
    type Error = BridgeError<M::Error>;
}

impl<M: MemAp> Read for BridgeSerial<M> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.mb.consume(buf)
    }
}

impl<M: MemAp> Write for BridgeSerial<M> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // All-or-nothing and never `Ok(0)` (`specs/swd-mailbox.md`, "Backpressure", requirement 6):
        // the free space is checked before any byte is produced, so a refused frame leaves `h2t` as
        // it was. Check-then-produce holds by the SPSC discipline: this end is the only `h2t`
        // producer and the firmware only advances `h2t_tail`, so free space can grow under the check
        // but never shrink. The session check costs one index read per frame and reports a second
        // attach at this run's next send rather than as a later timeout.
        let result = match self.mb.check_session().and_then(|()| self.mb.h2t_free()) {
            Ok(free) if (free as usize) < buf.len() => Err(BridgeError::NotDraining {
                used: RING_CAP - free,
                needed: buf.len(),
            }),
            // `free >= buf.len()`, so this SHOULD place the whole buffer, and the count is checked
            // rather than assumed: `produce` re-reads the indices, so the never-`Ok(0)` property
            // holds even when the single-producer premise does not. The decision is
            // `swd_mailbox::placed_whole`, the one the firmware endpoint takes too.
            Ok(_) => match self.mb.produce(buf) {
                Ok(n) => {
                    swd_mailbox::placed_whole(n, buf.len()).map_err(|_| BridgeError::NotDraining {
                        used: RING_CAP,
                        needed: buf.len(),
                    })
                }
                Err(e) => Err(e),
            },
            Err(e) => Err(e),
        };
        if let Err(e) = &result {
            self.write_error.get_or_insert(e.clone());
        }
        result
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(()) // a produced byte is committed by the head write in `produce`
    }
}

impl<M: MemAp> ReadReady for BridgeSerial<M> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.mb.t2h_used()? > 0)
    }
}

#[cfg(test)]
mod tests;
