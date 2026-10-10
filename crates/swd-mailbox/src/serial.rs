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
    /// The outbound ring has no room for the rest of this buffer. The write is all-or-nothing, so
    /// nothing was produced and the ring is byte-for-byte as it was.
    RingFull,
    /// No consumer for the outbound ring: no bridge has attached this boot, so a frame put there
    /// would only age until one did. The emission is dropped at the source instead
    /// ([`MailboxSerial::outbound_has_consumer`]).
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
}

impl MailboxSerial {
    /// The board endpoint: drains `h2t`, fills `t2h`, commits with a real `DMB`.
    pub fn firmware(mb: Mailbox) -> Self {
        MailboxSerial {
            mb,
            role: Role::Firmware,
            refused_writes: 0,
        }
    }

    /// The host/debugger endpoint: drains `t2h`, fills `h2t`, commits with a compiler fence.
    pub fn bridge(mb: Mailbox) -> Self {
        MailboxSerial {
            mb,
            role: Role::Bridge,
            refused_writes: 0,
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
    /// With the all-or-nothing rule a refusal is one **whole dropped frame**: `SerialTransport`
    /// hands a frame to `write_all`, whose first `write` either places all of it or places none, so
    /// this counts frames and not bytes. The firmware samples it into the SWD-readable observable
    /// block next to the other links' loss counters, which is what makes "my walk reply never came"
    /// diagnosable (`specs/swd-mailbox.md`, "Backpressure", requirement 4).
    ///
    /// Both refusal causes land here ([`MailboxError`]), and a reader tells them apart from the
    /// mailbox header it is already reading: a zero `epoch` means no bridge ever attached this boot,
    /// so every refusal is the at-the-source drop of an unattached board; a non-zero `epoch` with a
    /// stepping counter means a bridge is attached and not draining.
    pub fn refused_writes(&self) -> u16 {
        self.refused_writes
    }

    /// Whether this endpoint's outbound ring has a consumer, the gate on producing into it at all.
    ///
    /// One rule with a role-specific answer, and both answers come from the attach handshake
    /// (`specs/swd-mailbox.md`, "Attach + session flush"):
    ///
    /// - **Firmware.** `t2h`'s consumer is a bridge. The firmware zeroes `epoch` in `init_header`
    ///   every boot and an attaching bridge bumps it, so a non-zero `epoch` means "a bridge has
    ///   attached this boot" (it could only read zero again after 2^32 attaches without a reset).
    ///   An unattached board therefore drops its mailbox emissions at the source rather than filling
    ///   a ring nobody reads, so a bridge attaching later does not first read 28 ancient probe
    ///   hellos. This is the outbound half of the attach flush, which covers `h2t` only.
    /// - **Bridge.** `h2t`'s consumer is the firmware, which is running by construction: the bridge
    ///   validated the header that firmware wrote and bumped the epoch before it produces anything.
    pub fn outbound_has_consumer(&self) -> bool {
        match self.role {
            Role::Firmware => self.mb.epoch() != 0,
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
        Ok(self
            .mb
            .consume(self.role.inbound(), buf, self.role.commit()))
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
        // `free >= buf.len()`, so this places the whole buffer and returns `buf.len()`.
        Ok(self.mb.produce(ring, buf, self.role.commit()))
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
