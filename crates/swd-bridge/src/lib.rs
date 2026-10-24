// SPDX-License-Identifier: GPL-3.0-or-later
//! Host-side SWD RAM-mailbox bridge (`specs/swd-mailbox.md`, "Host").
//!
//! The firmware owns the mailbox in RAM and services it by polling; this library is the **other end**,
//! reading and writing the same rings over the SWD **MEM-AP while the core runs, no halt** (the proven
//! `LINK_OBS` / openocd `read_memory`/`write_memory` background-AHB path). It cannot use the firmware's
//! pointer-based `swd_mailbox::Mailbox` (the silicon RAM is not in the host's address space), so it
//! re-implements the bridge side of the SPSC ring discipline over a [`MemAp`] abstraction:
//!
//! - the bridge is the **`h2t` producer** ([`HostMailbox::produce`]) and the **`t2h` consumer**
//!   ([`HostMailbox::consume`]) - the mirror of the firmware `MailboxSerial`;
//! - it implements the **attach + epoch handshake** ([`HostMailbox::attach`]): validate `magic`/
//!   `version`, bump `epoch`, discard stale outbound (`t2h_tail := t2h_head`, the bridge's own index),
//!   then wait `epoch_ack == epoch` before producing (`epoch_ack` is written only after the firmware
//!   flushes, so it is unambiguous, `specs/swd-mailbox.md` "Attach race").
//!
//! [`BridgeSerial`] wraps it as an `embedded-io` serial (drain `t2h` on `Read`, fill `h2t` on `Write`),
//! so `link::SerialTransport` runs the same `StreamFramer` over it - one L2 code path, the mailbox is
//! just another byte-stream carrier.
//!
//! [`MemAp`] is the seam: an in-RAM [`MockMemAp`] backs the host unit tests (shared with the firmware
//! `Mailbox` over one buffer, so the test exercises both ends), and [`openocd::OpenOcdTcl`] is the
//! silicon backend (MEM-AP via openocd's TCL RPC, read/write-while-running).

pub mod config;
pub mod openocd;
pub mod walk;

use std::fmt;

use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write};
use swd_mailbox::{layout, MAGIC, RING_CAP, VERSION};

use crate::config::RegistryAgreement;

/// The mailbox region base address on the target (the fixed ABI base).
pub use swd_mailbox::MAILBOX_BASE;

/// A word/byte memory accessor over the SWD MEM-AP (or, in tests, in-RAM). All accesses are
/// **while-running** - no core halt. Addresses are absolute target addresses.
pub trait MemAp {
    /// Read one little-endian 32-bit word at `addr`.
    fn read32(&mut self, addr: u32) -> Result<u32, BridgeError>;
    /// Write one little-endian 32-bit word at `addr`.
    fn write32(&mut self, addr: u32, val: u32) -> Result<(), BridgeError>;
    /// Read `out.len()` bytes starting at `addr`.
    fn read(&mut self, addr: u32, out: &mut [u8]) -> Result<(), BridgeError>;
    /// Write `data` starting at `addr`.
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), BridgeError>;
}

/// What the bridge can fail with.
///
/// `Clone` so a refused write can latch a copy of the error it returns: `link::SerialTransport`
/// drops a frame on a write error and reports nothing (L2 is best-effort), so the only way the tool
/// ever learns the cause is a latch on the serial ([`BridgeSerial::take_write_error`]).
#[derive(Debug, Clone)]
pub enum BridgeError {
    /// `magic` / `version` did not match: not our firmware, not running, or the wrong address.
    Invalid {
        /// The `magic` word read.
        magic: u32,
        /// The `version` word read.
        version: u32,
    },
    /// The firmware did not write `epoch_ack == epoch` within the attach budget.
    AckTimeout,
    /// The MEM-AP backend (openocd / probe) failed.
    MemAp(String),
    /// The `h2t` ring had no room for a whole frame: the board is not draining its mailbox
    /// (`specs/swd-mailbox.md`, "Backpressure", requirement 6).
    ///
    /// The host end of the same contract the firmware end has: never `Ok(0)` for a non-empty buffer,
    /// because `write_all` panics on exactly that. The board is not at risk here, the bench session
    /// is: a halted or wedged core stops advancing `h2t_tail`, and a tool that panicked with
    /// "write() returned Ok(0)" reported nothing about why.
    NotDraining {
        /// Bytes unread in `h2t` **after** the refusal, so it is what a bench `mdw` of the header
        /// reads next rather than a figure that no longer holds. On the ordinary path nothing was
        /// committed, so it is the occupancy the free check saw; on the premise-violation path a
        /// short prefix went in and the ring is full, which is [`RING_CAP`] by construction.
        used: u32,
        /// Bytes this refused write needed to place.
        needed: usize,
    },
    /// **A second process attached to this mailbox while this run owned it**: `epoch` has moved
    /// away from the value [`HostMailbox::attach`] set (`todo.md` section 4).
    ///
    /// The harm is not the head race. A bridge attaching bumps `epoch`, and the firmware answers
    /// by FLUSHING the inbound ring (`h2t_tail := h2t_head`) and waiting for `epoch_ack`, so a
    /// second attach DISCARDS everything this run had queued. That is deterministic, not a race,
    /// and serialising the two writers would not have changed it: the requirement is one session
    /// OWNER per run, which is what `swd-mailbox-session` is.
    ///
    /// Sequential one-shots are unaffected, and always were: one attach, finish, exit, next. Only
    /// an OVERLAP moves the epoch under a live owner.
    SessionTaken {
        /// The epoch this run attached as.
        ours: u32,
        /// The epoch found in the header, which the second attacher wrote.
        found: u32,
    },
    /// **A second process produced into the `h2t` ring**: `h2t_head` moved between this run's
    /// writes, and only a producer writes `h2t_head` (the firmware advances `h2t_tail` alone).
    ///
    /// The companion to [`SessionTaken`](BridgeError::SessionTaken), for the process that pokes
    /// the ring without attaching. The frames of both producers interleave into one byte stream
    /// and the firmware's framer resyncs past the wreckage, which is why the symptom otherwise
    /// surfaces as somebody else's timeout.
    SecondProducer {
        /// `h2t_head` as this run left it.
        expected: u32,
        /// `h2t_head` as it reads now.
        found: u32,
    },
}

impl fmt::Display for BridgeError {
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

impl std::error::Error for BridgeError {}

// So `BridgeSerial` can be an `embedded-io` serial that `link::SerialTransport` drives.
impl embedded_io::Error for BridgeError {
    fn kind(&self) -> ErrorKind {
        match self {
            // The kind the `Write` contract prescribes for "not able to accept more bytes".
            BridgeError::NotDraining { .. } => ErrorKind::WriteZero,
            _ => ErrorKind::Other,
        }
    }
}

/// What this run owns of the mailbox, from [`HostMailbox::attach`] onward: the epoch it bumped to
/// and `h2t_head` as it last left it. Both are what a SECOND owner moves, and they are one value
/// because they begin together at attach and mean nothing before it (a `HostMailbox` that has not
/// attached owns no session and has no expectation of the head).
#[derive(Clone, Copy)]
struct Session {
    /// The epoch this run bumped to. `epoch` moving away from it means a second bridge attached.
    epoch: u32,
    /// `h2t_head` as this run left it. It moving under this run means a second producer wrote the
    /// ring (the firmware only ever advances `h2t_tail`).
    expect_h2t_head: u32,
}

/// The bridge-side view of the mailbox over a [`MemAp`]. Holds the session it attached as.
pub struct HostMailbox<M: MemAp> {
    mem: M,
    base: u32,
    session: Option<Session>,
}

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

    fn rd(&mut self, off: usize) -> Result<u32, BridgeError> {
        self.mem.read32(self.base + off as u32)
    }
    fn wr(&mut self, off: usize, val: u32) -> Result<(), BridgeError> {
        self.mem.write32(self.base + off as u32, val)
    }

    /// `magic`.
    pub fn magic(&mut self) -> Result<u32, BridgeError> {
        self.rd(layout::MAGIC)
    }
    /// `version`.
    pub fn version(&mut self) -> Result<u32, BridgeError> {
        self.rd(layout::VERSION)
    }
    /// `epoch`.
    pub fn epoch(&mut self) -> Result<u32, BridgeError> {
        self.rd(layout::EPOCH)
    }
    /// `epoch_ack`.
    pub fn epoch_ack(&mut self) -> Result<u32, BridgeError> {
        self.rd(layout::EPOCH_ACK)
    }

    /// The session this bridge bumped `epoch` to (after [`attach`](HostMailbox::attach)); 0 before
    /// it has attached, which is the value the firmware's `init_header` leaves.
    pub fn session_epoch(&self) -> u32 {
        self.session.map_or(0, |s| s.epoch)
    }

    /// The image's field-registry fingerprint, published in the header
    /// ([`swd_mailbox::REGISTRY_HASH`]).
    pub fn registry_hash(&mut self) -> Result<u32, BridgeError> {
        self.rd(layout::REGISTRY_HASH)
    }

    /// Read the image's registry fingerprint and pair it with this tool's own, so the tool can
    /// tell a field the board does not have from a tool older than the board
    /// ([`RegistryAgreement`]). A pure header read: no epoch bump, no flush, nothing another
    /// owner's session would notice, so it is safe to call before attaching (or instead of it).
    pub fn registry_agreement(&mut self) -> Result<RegistryAgreement, BridgeError> {
        Ok(RegistryAgreement::new(self.registry_hash()?))
    }

    /// **Is this run still the mailbox's only session?** Reads `epoch` and compares it against the
    /// one [`attach`](HostMailbox::attach) bumped to; a difference is a second attach, which has
    /// already cost this run everything it had queued ([`BridgeError::SessionTaken`]).
    ///
    /// Before attach there is no session to lose, and this is `Ok(())`.
    pub fn check_session(&mut self) -> Result<(), BridgeError> {
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
    pub fn validate(&mut self) -> Result<(), BridgeError> {
        let magic = self.magic()?;
        let version = self.version()?;
        if magic != MAGIC || version != VERSION {
            return Err(BridgeError::Invalid { magic, version });
        }
        Ok(())
    }

    /// Attach a fresh session (`specs/swd-mailbox.md`, "Attach + session flush"): validate the header,
    /// **bump `epoch`**, and discard stale outbound by setting `t2h_tail := t2h_head` (the bridge's own
    /// consumer index, so this does not rewrite a firmware index). Does **not** wait for the ack; call
    /// [`wait_flush_ack`](HostMailbox::wait_flush_ack) before producing the first inbound frame.
    /// It also takes the two readings a second owner would move, `epoch` and `h2t_head`, so that
    /// [`check_session`](HostMailbox::check_session) and [`produce`](HostMailbox::produce) can
    /// report one the moment it happens instead of letting it surface as this run's timeout.
    pub fn attach(&mut self) -> Result<(), BridgeError> {
        self.validate()?;
        let new_epoch = self.epoch()?.wrapping_add(1);
        self.wr(layout::EPOCH, new_epoch)?;
        // Discard any stale outbound: t2h_tail := t2h_head (bridge is the t2h consumer).
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
    pub fn flush_acked(&mut self) -> Result<bool, BridgeError> {
        Ok(self.epoch_ack()? == self.session_epoch())
    }

    /// Poll `flush_acked` up to `attempts` times (caller sleeps between). `Err(AckTimeout)` if the
    /// firmware never acks - e.g. it has no mailbox poll-site yet (before Tier-2 step 3).
    pub fn wait_flush_ack(&mut self, attempts: u32) -> Result<(), BridgeError> {
        for _ in 0..attempts {
            if self.flush_acked()? {
                return Ok(());
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        Err(BridgeError::AckTimeout)
    }

    // ---- the h2t producer side (bridge -> firmware) ----

    /// `h2t_head` (the bridge's producer index).
    pub fn h2t_head(&mut self) -> Result<u32, BridgeError> {
        self.rd(layout::H2T_HEAD)
    }
    /// `h2t_tail` (the firmware's consumer index).
    pub fn h2t_tail(&mut self) -> Result<u32, BridgeError> {
        self.rd(layout::H2T_TAIL)
    }
    /// Bytes the firmware has not yet drained from `h2t`.
    pub fn h2t_used(&mut self) -> Result<u32, BridgeError> {
        Ok(self.h2t_head()?.wrapping_sub(self.h2t_tail()?))
    }

    /// Room left in `h2t`: `cap - used`. What [`BridgeSerial`]'s all-or-nothing write checks before
    /// it produces a byte.
    pub fn h2t_free(&mut self) -> Result<u32, BridgeError> {
        Ok(RING_CAP - self.h2t_used()?)
    }

    /// Produce up to `src.len()` bytes into the `h2t` ring (write the payload **first**, then advance
    /// `h2t_head` - MEM-AP transfers are issued in order, so no `DMB` is needed). Returns the count
    /// written (bounded by free space).
    ///
    /// A short or zero return is byte-stream partial-write backpressure, the right answer for a ring
    /// of bytes, and it stays (the firmware's `swd_mailbox::Mailbox::produce` is the same). Frames
    /// are kept whole one layer up: [`BridgeSerial`]'s write refuses rather than place a prefix. A
    /// caller that uses this directly must check the count itself.
    /// It costs no extra MEM-AP traffic to notice a SECOND producer here, so it is noticed: this
    /// end is the ring's only producer by contract, the firmware advances `h2t_tail` alone, and
    /// the `h2t_head` this read returns is therefore the one this run committed last. A different
    /// value means somebody else wrote the ring ([`BridgeError::SecondProducer`]), and nothing is
    /// produced on top of it.
    pub fn produce(&mut self, src: &[u8]) -> Result<usize, BridgeError> {
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
    pub fn t2h_head(&mut self) -> Result<u32, BridgeError> {
        self.rd(layout::T2H_HEAD)
    }
    /// `t2h_tail` (the bridge's consumer index).
    pub fn t2h_tail(&mut self) -> Result<u32, BridgeError> {
        self.rd(layout::T2H_TAIL)
    }
    /// Bytes the firmware has produced into `t2h` and the bridge has not yet drained.
    pub fn t2h_used(&mut self) -> Result<u32, BridgeError> {
        Ok(self.t2h_head()?.wrapping_sub(self.t2h_tail()?))
    }

    /// Consume up to `dst.len()` bytes from the `t2h` ring (read, then advance `t2h_tail`). Returns the
    /// count read.
    pub fn consume(&mut self, dst: &mut [u8]) -> Result<usize, BridgeError> {
        let head = self.t2h_head()?;
        let tail = self.t2h_tail()?;
        let avail = head.wrapping_sub(tail);
        let n = (dst.len() as u32).min(avail) as usize;
        self.read_ring(swd_mailbox::T2H_DATA_OFF as u32, tail, &mut dst[..n])?;
        self.wr(layout::T2H_TAIL, tail.wrapping_add(n as u32))?;
        Ok(n)
    }

    /// Write `data` into a ring's data buffer starting at free-running `index`, handling the wrap at
    /// `cap` (one MEM-AP write for the run to the buffer end, a second for the wrapped remainder).
    fn write_ring(&mut self, data_off: u32, index: u32, data: &[u8]) -> Result<(), BridgeError> {
        if data.is_empty() {
            return Ok(());
        }
        let cap = RING_CAP;
        let mask = cap - 1;
        let start = index & mask;
        let first = ((cap - start) as usize).min(data.len());
        self.mem
            .write(self.base + data_off + start, &data[..first])?;
        if first < data.len() {
            self.mem.write(self.base + data_off, &data[first..])?;
        }
        Ok(())
    }

    /// Read `out.len()` bytes from a ring's data buffer starting at free-running `index`, handling wrap.
    fn read_ring(&mut self, data_off: u32, index: u32, out: &mut [u8]) -> Result<(), BridgeError> {
        if out.is_empty() {
            return Ok(());
        }
        let cap = RING_CAP;
        let mask = cap - 1;
        let start = index & mask;
        let first = ((cap - start) as usize).min(out.len());
        self.mem
            .read(self.base + data_off + start, &mut out[..first])?;
        if first < out.len() {
            let rest = out.len() - first;
            self.mem
                .read(self.base + data_off, &mut out[first..first + rest])?;
        }
        Ok(())
    }
}

/// The bridge end as an `embedded-io` serial: `Read` drains `t2h`, `Write` fills `h2t`, `ReadReady`
/// reports `t2h` occupancy. Wrap it in [`link::SerialTransport`] to carry `l2.md` frames - the mirror
/// of the firmware `swd_mailbox::MailboxSerial::bridge`.
pub struct BridgeSerial<M: MemAp> {
    mb: HostMailbox<M>,
    /// The first write error since the latch was last taken
    /// ([`take_write_error`](BridgeSerial::take_write_error)).
    write_error: Option<BridgeError>,
}

impl<M: MemAp> BridgeSerial<M> {
    /// Wrap an attached [`HostMailbox`] as a serial.
    pub fn new(mb: HostMailbox<M>) -> Self {
        BridgeSerial {
            mb,
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
    /// best-effort, a higher layer retransmits) and wrong for a bench tool, where the dropped frame
    /// IS the command the operator typed. So the error is kept here and the caller
    /// (`walk::WalkDriver::send`) reads it back after every send and fails with it, rather than
    /// letting it become a walk timeout with no stated cause.
    ///
    /// It latches the FIRST error of a run and any MEM-AP failure too, not just
    /// [`BridgeError::NotDraining`]: a probe that dies mid-frame was equally invisible before.
    pub fn take_write_error(&mut self) -> Option<BridgeError> {
        self.write_error.take()
    }
}

impl<M: MemAp> ErrorType for BridgeSerial<M> {
    type Error = BridgeError;
}

impl<M: MemAp> Read for BridgeSerial<M> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.mb.consume(buf)
    }
}

impl<M: MemAp> Write for BridgeSerial<M> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // All-or-nothing and never `Ok(0)`, the firmware endpoint's rule at this end of the same ring
        // (`specs/swd-mailbox.md`, "Backpressure", requirement 6): the free space is checked before
        // any byte is produced, so a refused frame leaves `h2t` byte-for-byte as it was instead of a
        // prefix the firmware's framer would have to resync past. Check-then-produce is sound by the
        // SPSC discipline: this end is the only `h2t` producer and the firmware only ever advances
        // `h2t_tail`, so free space can grow under the check but never shrink.
        //
        // The check costs two MEM-AP index reads per frame on top of the two `produce` makes, over
        // the openocd TCL round trip, and it is kept: what it buys is that a refusal commits nothing,
        // and the traffic here is a handful of frames per command and tens per second in the
        // streaming tools, so four index reads a frame is not what bounds them. Folding the two
        // pairs into one read would mean a `produce` variant taking indices from its caller, which is
        // more API for a latency nobody has measured as a problem.
        // Is this run still the only session? One index read per frame, on the path where the
        // answer is load-bearing: a second attach has already made the firmware throw this run's
        // queued frames away, and without this the loss resurfaces minutes later as a walk or
        // keepalive timeout against a board that was behaving correctly (`todo.md` section 4, the
        // 2026-10-08 session).
        //
        // On the WRITE path and not the read poll, which is a cost decision rather than a
        // coverage one: `recv` spins as fast as the MEM-AP allows and a check there would be
        // permanent extra traffic, while every tool that holds a link re-sends on a timer (the
        // walk retransmits at 3 s, the INPUTS keepalive at its own period, the config exchange
        // retransmits at 3 s), so a stolen session is reported within one of those intervals
        // anyway, through the same latch the caller already reads after each send.
        let result = match self.mb.check_session().and_then(|()| self.mb.h2t_free()) {
            Ok(free) if (free as usize) < buf.len() => Err(BridgeError::NotDraining {
                used: RING_CAP - free,
                needed: buf.len(),
            }),
            // `free >= buf.len()`, so this SHOULD place the whole buffer - and the count is checked
            // rather than assumed. `produce` re-reads `h2t_head`/`h2t_tail` and recomputes free for
            // itself, so the never-`Ok(0)` property is structural here: the free check above is an
            // optimisation (it is what keeps a refusal all-or-nothing), and this is what holds the
            // contract when the premise the check rests on does not.
            //
            // That premise is single-producer. A second producer is now DETECTED at this end
            // (the session check above, and `produce`'s expectation of `h2t_head`), which is a
            // detection and not a lock: it reports the overlap after the fact, so the count
            // check below is still what holds the `Write` contract while it is happening.
            //
            // The decision is `swd_mailbox::placed_whole`, the one the firmware endpoint takes too,
            // so the two ends of the ring cannot drift apart on the property that matters. The ring
            // is FULL whenever it refuses: `produce` stops only when free space runs out, so the
            // reported occupancy is the whole ring, with no further MEM-AP read.
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

/// An in-RAM [`MemAp`] over a host buffer, for tests. It addresses `buf[addr]` (so a `HostMailbox` with
/// `base = 0` and the firmware `swd_mailbox::Mailbox::from_raw(buf_ptr)` share one backing - the test
/// drives both ends of the SPSC mailbox over the same memory).
pub struct MockMemAp {
    ptr: *mut u8,
    len: usize,
}

impl MockMemAp {
    /// # Safety
    /// `ptr` must point to at least `len` bytes that outlive this accessor.
    pub unsafe fn new(ptr: *mut u8, len: usize) -> Self {
        MockMemAp { ptr, len }
    }

    fn slice(&mut self, addr: u32, n: usize) -> Result<&mut [u8], BridgeError> {
        let start = addr as usize;
        if start + n > self.len {
            return Err(BridgeError::MemAp(format!(
                "mock access {start}..{} past len {}",
                start + n,
                self.len
            )));
        }
        // SAFETY: bounds checked above; the buffer outlives self by the `new` contract.
        Ok(unsafe { std::slice::from_raw_parts_mut(self.ptr.add(start), n) })
    }
}

impl MemAp for MockMemAp {
    fn read32(&mut self, addr: u32) -> Result<u32, BridgeError> {
        let s = self.slice(addr, 4)?;
        Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn write32(&mut self, addr: u32, val: u32) -> Result<(), BridgeError> {
        let s = self.slice(addr, 4)?;
        s.copy_from_slice(&val.to_le_bytes());
        Ok(())
    }
    fn read(&mut self, addr: u32, out: &mut [u8]) -> Result<(), BridgeError> {
        let n = out.len();
        out.copy_from_slice(self.slice(addr, n)?);
        Ok(())
    }
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), BridgeError> {
        let n = data.len();
        self.slice(addr, n)?.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests;
