//! The ESP's own slot update (`specs/esp-tool.md`, "Installing and updating the ESP", "Later
//! updates"): `ESP_UPDATE_CHUNK`s with the board flash's window and acknowledgements, written into
//! the inactive app slot as they arrive, a sector at a time (the image is far larger than RAM).
//!
//! When the last byte is in, the image's SHA-256 is checked twice, over the bytes as they arrived
//! and over the slot read back, and the slot's app descriptor must carry the version
//! `ESP_UPDATE_BEGIN` announced. Only then is the slot made the next boot's, marked new; the
//! service then has the firmware reset. The bootloader boots a new slot pending verification and
//! falls back to the other slot at the next reset unless `ESP_UPDATE_CONFIRM` marked it valid.
//!
//! The flash and the boot selection are the firmware's ([`Platform`]); here is the order.

use espsvc::{app_version, Status, StreamRx, APP_HEAD};
use sha2::{Digest, Sha256};

use crate::svc::Platform;

/// The flash erase unit: the image is written one sector at a time.
pub const SECTOR: usize = 4096;
const _: () = assert!(APP_HEAD <= SECTOR && APP_HEAD.is_multiple_of(4));

/// What one `ESP_UPDATE_CHUNK` comes to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// No update is running (a late chunk of a finished or refused one): no answer.
    Ignore,
    /// Answer `ESP_UPDATE_ACK` with this offset.
    Ack(u32),
    /// The last chunk: answer `ESP_UPDATE_ACK` with the length, then `RESULT` with this outcome.
    /// `Ok` means the slot is the next boot's.
    Done(u32, Result<(), Status>),
}

/// An update in progress.
pub struct Update {
    rx: StreamRx,
    sha: [u8; 32],
    version: [u8; 3],
    hash: Sha256,
    /// The sector being filled, and how much of it is.
    sector: [u8; SECTOR],
    fill: usize,
    /// The slot is the next boot's: duplicates of the last chunk are acknowledged, nothing more.
    activated: bool,
}

impl Default for Update {
    fn default() -> Self {
        Self::new()
    }
}

impl Update {
    /// No update.
    pub fn new() -> Self {
        Update {
            rx: StreamRx::new(),
            sha: [0; 32],
            version: [0; 3],
            hash: Sha256::new(),
            sector: [0xFF; SECTOR],
            fill: 0,
            activated: false,
        }
    }

    /// `ESP_UPDATE_BEGIN`: start writing an image of `len` bytes into the inactive slot, dropping
    /// any update before it.
    pub fn begin<P: Platform>(
        &mut self,
        p: &mut P,
        len: u32,
        sha: [u8; 32],
        version: [u8; 3],
    ) -> Result<(), Status> {
        *self = Update::new();
        self.rx.begin(len).map_err(|_| Status::BadRequest)?;
        if let Err(e) = p.update_begin(len) {
            self.rx.reset();
            return Err(e);
        }
        self.sha = sha;
        self.version = version;
        Ok(())
    }

    /// `ESP_UPDATE_CHUNK`. `Err` answers `RESULT` (a malformed chunk; the update goes on).
    pub fn chunk<P: Platform>(
        &mut self,
        p: &mut P,
        offset: u32,
        data: &[u8],
    ) -> Result<Step, Status> {
        if self.rx.is_empty() {
            return Ok(Step::Ignore);
        }
        if self.activated {
            return Ok(Step::Ack(self.rx.len()));
        }
        if !self
            .rx
            .chunk(offset, data)
            .map_err(|_| Status::BadRequest)?
        {
            return Ok(Step::Ack(self.rx.contiguous()));
        }
        self.hash.update(data);
        let r = self.take(p, offset, data).and_then(|()| {
            if self.rx.complete() {
                self.finish(p)
            } else {
                Ok(())
            }
        });
        match r {
            Err(e) => {
                let len = self.rx.len();
                self.rx.reset();
                Ok(Step::Done(len, Err(e)))
            }
            Ok(()) if self.activated => Ok(Step::Done(self.rx.len(), Ok(()))),
            Ok(()) => Ok(Step::Ack(self.rx.contiguous())),
        }
    }

    /// Put the next bytes, at `offset`, into the sector buffer, writing each sector as it fills
    /// and the last one at the end.
    fn take<P: Platform>(&mut self, p: &mut P, offset: u32, data: &[u8]) -> Result<(), Status> {
        let mut at = offset as usize;
        let mut rest = data;
        while !rest.is_empty() {
            let n = rest.len().min(SECTOR - self.fill);
            self.sector[self.fill..self.fill + n].copy_from_slice(&rest[..n]);
            self.fill += n;
            at += n;
            rest = &rest[n..];
            if self.fill == SECTOR || at == self.rx.len() as usize {
                let base = (at - self.fill) as u32;
                p.update_write(base, &self.sector[..self.fill])?;
                self.fill = 0;
            }
        }
        Ok(())
    }

    /// Every byte is written: check it, then make the slot the next boot's.
    fn finish<P: Platform>(&mut self, p: &mut P) -> Result<(), Status> {
        let arrived: [u8; 32] = core::mem::take(&mut self.hash).finalize().into();
        if arrived != self.sha {
            return Err(Status::ShaMismatch);
        }
        // The slot read back, a sector at a time through the now idle sector buffer. Reads are
        // whole words; the slot is larger than the image, so the last one may run past it.
        let len = self.rx.len() as usize;
        let mut hash = Sha256::new();
        let mut off = 0;
        while off < len {
            let n = (len - off).min(SECTOR);
            p.update_read(off as u32, &mut self.sector[..n.next_multiple_of(4)])?;
            if off == 0 && app_version(&self.sector[..APP_HEAD.min(n)]) != Some(self.version) {
                return Err(Status::BadImage);
            }
            hash.update(&self.sector[..n]);
            off += n;
        }
        let written: [u8; 32] = hash.finalize().into();
        if written != self.sha {
            return Err(Status::VerifyMismatch);
        }
        p.update_activate()?;
        self.activated = true;
        Ok(())
    }
}
