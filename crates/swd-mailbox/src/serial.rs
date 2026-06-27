//! [`MailboxSerial`]: an `embedded-io` serial over the two mailbox rings.
//!
//! It implements `Read` by draining the **inbound** ring, `Write` by appending to the **outbound**
//! ring, and `ReadReady` from the inbound ring's `used` count. Which ring is inbound vs outbound is the
//! endpoint's [`Role`]. Wrapping it in `link`'s [`SerialTransport`](link::SerialTransport) runs the
//! existing `StreamFramer` over the rings, so the SWD mailbox is just another byte-stream L2 link.

use embedded_io::{ErrorType, Read, ReadReady, Write};

use crate::{Mailbox, Role};

/// One end of the mailbox link as an `embedded-io` serial.
pub struct MailboxSerial {
    mb: Mailbox,
    role: Role,
}

impl MailboxSerial {
    /// The board endpoint: drains `h2t`, fills `t2h`, commits with a real `DMB`.
    pub fn firmware(mb: Mailbox) -> Self {
        MailboxSerial {
            mb,
            role: Role::Firmware,
        }
    }

    /// The host/debugger endpoint: drains `t2h`, fills `h2t`, commits with a compiler fence.
    pub fn bridge(mb: Mailbox) -> Self {
        MailboxSerial {
            mb,
            role: Role::Bridge,
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
}

impl ErrorType for MailboxSerial {
    // RAM access cannot fail; the `embedded-io` `Error` impl for `Infallible` makes this a valid
    // error type that never produces a value.
    type Error = core::convert::Infallible;
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
        Ok(self
            .mb
            .produce(self.role.outbound(), buf, self.role.commit()))
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
