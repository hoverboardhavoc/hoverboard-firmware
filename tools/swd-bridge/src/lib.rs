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
//!
//! The ring logic and the attach handshake live in the `no_std` `swd-bridge-core` crate, which the
//! hoverboard bridge (`bridge/`) runs over its own SWD engine; this crate re-exports it and adds
//! the OpenOCD backend, the thread sleep in the flush-ack wait and the host tools.

pub mod config;
pub mod openocd;
pub mod walk;

use std::ops::{Deref, DerefMut};

use crate::config::RegistryAgreement;

/// The bridge end as an `embedded-io` serial (`swd_bridge_core::BridgeSerial`), over a host backend.
pub use swd_bridge_core::BridgeSerial;
/// The mailbox region base address on the target (the fixed ABI base).
pub use swd_mailbox::MAILBOX_BASE;

/// What the bridge can fail with on the host: the core error with the backend's message as text
/// (OpenOCD's replies, a socket error).
pub type BridgeError = swd_bridge_core::BridgeError<String>;

/// A host memory access backend: a `swd_bridge_core::MemAp` whose error is a message. OpenOCD's TCL
/// RPC ([`openocd::OpenOcdTcl`]) and the in-RAM [`MockMemAp`] are the two.
///
/// It repeats the four accesses with the host's [`BridgeError`] so the tools call them on a concrete
/// backend (`host.mem().read32(..)`) with this trait alone in scope, as they did before the ring
/// logic moved into `swd-bridge-core`. Generic code bounded on this trait calls the core trait's
/// methods by path instead, where both would be in scope.
pub trait MemAp: swd_bridge_core::MemAp<Error = String> {
    /// Read one little-endian 32-bit word at `addr`.
    fn read32(&mut self, addr: u32) -> Result<u32, BridgeError> {
        swd_bridge_core::MemAp::read32(self, addr).map_err(BridgeError::MemAp)
    }
    /// Write one little-endian 32-bit word at `addr`.
    fn write32(&mut self, addr: u32, val: u32) -> Result<(), BridgeError> {
        swd_bridge_core::MemAp::write32(self, addr, val).map_err(BridgeError::MemAp)
    }
    /// Read `out.len()` bytes starting at `addr`.
    fn read(&mut self, addr: u32, out: &mut [u8]) -> Result<(), BridgeError> {
        swd_bridge_core::MemAp::read(self, addr, out).map_err(BridgeError::MemAp)
    }
    /// Write `data` starting at `addr`.
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), BridgeError> {
        swd_bridge_core::MemAp::write(self, addr, data).map_err(BridgeError::MemAp)
    }
}

impl<T: swd_bridge_core::MemAp<Error = String>> MemAp for T {}

/// The bridge-side view of the mailbox (`swd_bridge_core::HostMailbox`), with the two things only
/// a host has: a thread to sleep in while the firmware acks, and the registry agreement's wording.
/// Every ring and attach method is the core's, reached through `Deref`.
pub struct HostMailbox<M: swd_bridge_core::MemAp>(swd_bridge_core::HostMailbox<M>);

impl<M: swd_bridge_core::MemAp> HostMailbox<M> {
    /// A bridge over `mem`, with the mailbox at `base` (use [`MAILBOX_BASE`] on silicon).
    pub fn new(mem: M, base: u32) -> Self {
        HostMailbox(swd_bridge_core::HostMailbox::new(mem, base))
    }

    /// Poll `flush_acked` up to `attempts` times, 10 ms apart. `Err(AckTimeout)` if the firmware
    /// never acks, e.g. it has no mailbox poll-site.
    pub fn wait_flush_ack(&mut self, attempts: u32) -> swd_bridge_core::Res<(), M> {
        self.0.wait_flush_ack(attempts, || {
            std::thread::sleep(std::time::Duration::from_millis(10))
        })
    }

    /// Read the image's registry fingerprint and pair it with this tool's own, so the tool can
    /// tell a field the board does not have from a tool older than the board
    /// ([`RegistryAgreement`]). A pure header read: no epoch bump, no flush, nothing another
    /// owner's session would notice, so it is safe to call before attaching (or instead of it).
    pub fn registry_agreement(&mut self) -> swd_bridge_core::Res<RegistryAgreement, M> {
        Ok(RegistryAgreement::new(self.0.registry_hash()?))
    }
}

impl<M: swd_bridge_core::MemAp> Deref for HostMailbox<M> {
    type Target = swd_bridge_core::HostMailbox<M>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<M: swd_bridge_core::MemAp> DerefMut for HostMailbox<M> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<M: swd_bridge_core::MemAp> From<HostMailbox<M>> for swd_bridge_core::HostMailbox<M> {
    fn from(h: HostMailbox<M>) -> Self {
        h.0
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

    fn slice(&mut self, addr: u32, n: usize) -> Result<&mut [u8], String> {
        let start = addr as usize;
        if start + n > self.len {
            return Err(format!(
                "mock access {start}..{} past len {}",
                start + n,
                self.len
            ));
        }
        // SAFETY: bounds checked above; the buffer outlives self by the `new` contract.
        Ok(unsafe { std::slice::from_raw_parts_mut(self.ptr.add(start), n) })
    }
}

impl swd_bridge_core::MemAp for MockMemAp {
    type Error = String;
    fn read32(&mut self, addr: u32) -> Result<u32, String> {
        let s = self.slice(addr, 4)?;
        Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn write32(&mut self, addr: u32, val: u32) -> Result<(), String> {
        let s = self.slice(addr, 4)?;
        s.copy_from_slice(&val.to_le_bytes());
        Ok(())
    }
    fn read(&mut self, addr: u32, out: &mut [u8]) -> Result<(), String> {
        let n = out.len();
        out.copy_from_slice(self.slice(addr, n)?);
        Ok(())
    }
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), String> {
        let n = data.len();
        self.slice(addr, n)?.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests;
