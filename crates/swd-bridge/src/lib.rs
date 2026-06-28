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

pub mod openocd;
pub mod walk;

use std::fmt;

use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write};
use swd_mailbox::{layout, MAGIC, RING_CAP, VERSION};

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
#[derive(Debug)]
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
        }
    }
}

impl std::error::Error for BridgeError {}

// So `BridgeSerial` can be an `embedded-io` serial that `link::SerialTransport` drives.
impl embedded_io::Error for BridgeError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// The bridge-side view of the mailbox over a [`MemAp`]. Holds the session epoch it bumped to.
pub struct HostMailbox<M: MemAp> {
    mem: M,
    base: u32,
    session_epoch: u32,
}

impl<M: MemAp> HostMailbox<M> {
    /// A bridge over `mem`, with the mailbox at `base` (use [`MAILBOX_BASE`] on silicon).
    pub fn new(mem: M, base: u32) -> Self {
        HostMailbox {
            mem,
            base,
            session_epoch: 0,
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

    /// The session this bridge bumped `epoch` to (after [`attach`](HostMailbox::attach)).
    pub fn session_epoch(&self) -> u32 {
        self.session_epoch
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
    pub fn attach(&mut self) -> Result<(), BridgeError> {
        self.validate()?;
        let new_epoch = self.epoch()?.wrapping_add(1);
        self.wr(layout::EPOCH, new_epoch)?;
        self.session_epoch = new_epoch;
        // Discard any stale outbound: t2h_tail := t2h_head (bridge is the t2h consumer).
        let t2h_head = self.rd(layout::T2H_HEAD)?;
        self.wr(layout::T2H_TAIL, t2h_head)?;
        Ok(())
    }

    /// True once the firmware has acked this session (`epoch_ack == epoch`).
    pub fn flush_acked(&mut self) -> Result<bool, BridgeError> {
        Ok(self.epoch_ack()? == self.session_epoch)
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

    /// Produce up to `src.len()` bytes into the `h2t` ring (write the payload **first**, then advance
    /// `h2t_head` - MEM-AP transfers are issued in order, so no `DMB` is needed). Returns the count
    /// written (bounded by free space).
    pub fn produce(&mut self, src: &[u8]) -> Result<usize, BridgeError> {
        let cap = RING_CAP;
        let head = self.h2t_head()?;
        let tail = self.h2t_tail()?;
        let free = cap - head.wrapping_sub(tail);
        let n = (src.len() as u32).min(free) as usize;
        self.write_ring(swd_mailbox::H2T_DATA_OFF as u32, head, &src[..n])?;
        self.wr(layout::H2T_HEAD, head.wrapping_add(n as u32))?; // the commit, after the payload
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
}

impl<M: MemAp> BridgeSerial<M> {
    /// Wrap an attached [`HostMailbox`] as a serial.
    pub fn new(mb: HostMailbox<M>) -> Self {
        BridgeSerial { mb }
    }
    /// Borrow the inner mailbox (indices / attach).
    pub fn mailbox(&mut self) -> &mut HostMailbox<M> {
        &mut self.mb
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
        self.mb.produce(buf)
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
