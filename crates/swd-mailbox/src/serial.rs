//! [`MailboxSerial`]: an `embedded-io` serial over the two mailbox rings.
//!
//! It implements `Read` by draining the **inbound** ring, `Write` by appending to the **outbound**
//! ring, and `ReadReady` from the inbound ring's `used` count. Which ring is inbound vs outbound is the
//! endpoint's [`Role`]. Wrapping it in `link`'s [`SerialTransport`](link::SerialTransport) runs the
//! existing `StreamFramer` over the rings, so the SWD mailbox is just another byte-stream L2 link.

use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write};

use crate::{Mailbox, Role};

/// Why a [`MailboxSerial`] write placed nothing (`specs/swd-mailbox.md`, "Backpressure: a full ring
/// drops a frame, it does not panic the board").
///
/// **This type exists because `Infallible` was a broken trait contract.** `embedded_io::Write`
/// forbids answering `Ok(0)` for a non-empty buffer, and `write_all` **panics** on exactly that. A
/// serial that can block honours the contract by blocking on the first byte; this one cannot, because
/// the outbound ring's consumer is a debugger that may never attach, so its only contract-respecting
/// answer is an error. While the error type was uninhabited the error was impossible to report and the
/// panic was inevitable: a `panic-halt` board stopped feeding the watchdog and the IWDG reset it on
/// the 29th nine-byte frame into an undrained ring (the F130 slave's every-28th-walk warm reset).
///
/// [`Read`] shares this type (`ErrorType` is one type per endpoint) but never produces a value: RAM
/// reads cannot fail, and an empty inbound ring is `Ok(0)`, not an error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MailboxError {
    /// The outbound ring had no room for the whole of this buffer. Normally nothing was produced and
    /// the ring is byte-for-byte as it was (the free check runs first); if a second producer took the
    /// space after that check, a short prefix may have been committed, which the receiver's framer
    /// resyncs past. Either way the frame is lost and the caller is told, which is the property that
    /// matters.
    RingFull,
    /// Nothing has reached this endpoint this boot (no inbound byte consumed, no `epoch` bump), so
    /// a frame put on the outbound ring would only age there. The emission is dropped at the source
    /// instead ([`MailboxSerial::outbound_has_consumer`], which holds the rule and its limits).
    NotAttached,
}

impl embedded_io::Error for MailboxError {
    fn kind(&self) -> ErrorKind {
        match self {
            // The kind the `Write` contract prescribes for "not able to accept more bytes".
            MailboxError::RingFull => ErrorKind::WriteZero,
            // Not a full buffer: the ring is empty and staying that way until a bridge attaches,
            // which is what `NotConnected` says ("not connected yet").
            MailboxError::NotAttached => ErrorKind::NotConnected,
        }
    }
}

/// One end of the mailbox link as an `embedded-io` serial.
pub struct MailboxSerial {
    mb: Mailbox,
    role: Role,
    /// Writes refused since boot, saturating: one per frame dropped on the outbound ring
    /// ([`MailboxSerial::refused_writes`]). A `u16` rather than a `u32` because it fits the padding
    /// after `role` and so costs this endpoint no bytes at all.
    refused_writes: u16,
    /// Whether this endpoint has ever consumed a byte from its inbound ring
    /// ([`MailboxSerial::outbound_has_consumer`]). Shares the padding after `role`, so it is free
    /// too.
    inbound_seen: bool,
}

impl MailboxSerial {
    /// The board endpoint: drains `h2t`, fills `t2h`, commits with a real `DMB`.
    pub fn firmware(mb: Mailbox) -> Self {
        MailboxSerial {
            mb,
            role: Role::Firmware,
            refused_writes: 0,
            inbound_seen: false,
        }
    }

    /// The host/debugger endpoint: drains `t2h`, fills `h2t`, commits with a compiler fence.
    pub fn bridge(mb: Mailbox) -> Self {
        MailboxSerial {
            mb,
            role: Role::Bridge,
            refused_writes: 0,
            inbound_seen: false,
        }
    }

    /// This endpoint's role.
    pub fn role(&self) -> Role {
        self.role
    }

    /// The underlying mailbox handle.
    pub fn mailbox(&self) -> Mailbox {
        self.mb
    }

    /// Writes this endpoint refused since boot, saturating at `u16::MAX`.
    ///
    /// **One refusal is one whole dropped frame**, because `SerialTransport` hands each frame to
    /// `write_all`, whose first `write` either places all of it or returns an error. On this port a
    /// frame is also one whole PDU, and that holds on a BOUND rather than on the all-or-nothing
    /// rule: the mailbox chunk cap is `FRAME_CAPACITY - 1` = 127 bytes and the largest PDU the
    /// responder emits is `net::walk::MAX_PDU` = 64, so nothing the mailbox port carries fragments.
    /// Raise `MAX_PDU` past the chunk cap and this counts frames while an emission can cost several,
    /// so a reader comparing it against walks would need that factor.
    ///
    /// The firmware samples it into the SWD-readable observable block next to the other links' loss
    /// counters, which is what makes "my walk reply never came" diagnosable
    /// (`specs/swd-mailbox.md`, "Backpressure", requirement 4).
    ///
    /// **Reading it.** Both refusal causes land here ([`MailboxError`]), and the two are told apart
    /// by the ring, not by `epoch`: nothing ever clears `epoch`, so a non-zero one means a bridge
    /// attached at SOME point this boot and not that one is attached now.
    ///
    /// - A climbing count with `t2h_used` (`t2h_head - t2h_tail`) at or near [`crate::RING_CAP`] is
    ///   a real backlog: something reached this board and is no longer draining its replies.
    /// - A climbing count with `t2h` EMPTY is the at-the-source drop of a board nothing has reached
    ///   this boot ([`MailboxSerial::outbound_has_consumer`]), which is the expected reading on a
    ///   slave and says nothing is wrong.
    ///
    /// Both words are in the same header the reader is already reading, so this costs no second
    /// mechanism.
    pub fn refused_writes(&self) -> u16 {
        self.refused_writes
    }

    /// Whether anything has reached this endpoint, the gate on producing into the outbound ring at
    /// all (`specs/swd-mailbox.md`, "Backpressure", requirement 5).
    ///
    /// **This gate is a politeness, not the safety property.** What keeps the board alive is
    /// requirements 1 to 3, the fallible all-or-nothing write, and those hold whether or not this
    /// answer is right. What it buys is that a board nothing has ever talked to does not spend its
    /// ring on replies nobody will read. It is also NOT what protects a bridge attaching later from
    /// stale bytes: [`Bridge::attach`](crate::Bridge::attach) flushes the outbound ring itself
    /// (`t2h_tail := t2h_head`) as its own consumer, so a late attacher is covered there. Do not
    /// strengthen this gate on the belief that staleness depends on it.
    ///
    /// - **Firmware.** Two pieces of evidence, either sufficient:
    ///   - **An inbound byte consumed this boot**, which is proof of a live peer: something is
    ///     producing into `h2t`, and the thing that produces into `h2t` is the thing that drains
    ///     `t2h`.
    ///   - **`epoch != 0`**, which covers the bridge that has attached but not yet written.
    ///
    ///   `epoch` alone is NOT enough, and that is the trap worth naming: `init_header` zeroes it on
    ///   every boot, so a board that resets MID-SESSION (a brown-out, a bench power cycle, an
    ///   unrelated watchdog reset) comes back still being fed by a host that never re-attaches. On
    ///   `epoch` alone it would drain those frames, act on them, and refuse every reply as
    ///   "unattached", turning a recoverable reset into a tool timeout with no stated cause, which
    ///   is the failure requirement 6 exists to abolish. Consumption is what sees that peer.
    /// - **Bridge.** Always true: `h2t`'s consumer is the firmware, which is running by
    ///   construction, since the bridge validated the header that firmware wrote and bumped the
    ///   epoch before it produces anything.
    pub fn outbound_has_consumer(&self) -> bool {
        match self.role {
            // The cheap fact first: once a byte has arrived the volatile `epoch` read is skipped.
            Role::Firmware => self.inbound_seen || self.mb.epoch() != 0,
            Role::Bridge => true,
        }
    }
}

impl ErrorType for MailboxSerial {
    type Error = MailboxError;
}

impl Read for MailboxSerial {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Non-blocking by design: returns 0 when the inbound ring is empty. The cooperative caller
        // (`SerialTransport`) gates `read` on `read_ready`, so a 0 is never mistaken for EOF.
        let n = self
            .mb
            .consume(self.role.inbound(), buf, self.role.commit());
        // A byte that came OUT of the inbound ring is the proof a peer is producing into it, and so
        // the evidence the outbound gate runs on (`outbound_has_consumer`). Recorded here because
        // this is the only place inbound bytes are taken; a zero read says nothing either way.
        self.inbound_seen |= n > 0;
        Ok(n)
    }
}

impl Write for MailboxSerial {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if !self.outbound_has_consumer() {
            self.refused_writes = self.refused_writes.saturating_add(1);
            return Err(MailboxError::NotAttached);
        }
        let ring = self.role.outbound();
        // All-or-nothing (`specs/swd-mailbox.md`, "Backpressure", requirement 2): the free space is
        // checked BEFORE any byte is produced, so a refused frame leaves the ring byte-for-byte as it
        // was instead of a malformed prefix for the next bridge to resync past. `produce`'s
        // partial-write behaviour is correct for a byte stream and is untouched; this rule is the
        // transport layer's.
        //
        // Check-then-produce is sound without a lock, by the SPSC discipline: this endpoint is the
        // ring's only producer and the consumer only ever advances `tail`, so free space can grow
        // under the check but never shrink.
        if (self.mb.free(ring) as usize) < buf.len() {
            self.refused_writes = self.refused_writes.saturating_add(1);
            return Err(MailboxError::RingFull);
        }
        // The never-`Ok(0)` property is STRUCTURAL, not argued: `produce` re-reads `head`/`tail` and
        // recomputes free for itself, so its count is checked rather than assumed. The free check
        // above is an optimisation on this path (it is what keeps a refusal all-or-nothing); this is
        // what makes the contract hold even when the premise the check rests on does not.
        //
        // That premise is single-producer, and nothing enforces it (`todo.md` section 4; a second
        // host process violated it on this bench on 2026-10-08). If a second producer takes the
        // space between the check and the commit, the worst case is now a short frame in the ring,
        // a malformed prefix the receiver's framer resyncs past, instead of an `Ok(0)` that panics
        // the board.
        //
        // The interleaving itself is pinned at the OTHER end of the same ring, where the `MemAp`
        // seam can inject it (`swd-bridge`'s `a_short_produce_is_an_error_rather_than_an_ok_zero`);
        // this pointer `Mailbox` reads RAM directly, so there is no seam to inject one here.
        let n = self.mb.produce(ring, buf, self.role.commit());
        if n < buf.len() {
            self.refused_writes = self.refused_writes.saturating_add(1);
            return Err(MailboxError::RingFull);
        }
        Ok(n)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // The "wire" is RAM: a written byte is already committed by `produce`'s `head` store.
        Ok(())
    }
}

impl ReadReady for MailboxSerial {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.mb.used(self.role.inbound()) > 0)
    }
}
