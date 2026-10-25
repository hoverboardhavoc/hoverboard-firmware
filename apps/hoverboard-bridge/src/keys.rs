//! The bridge's store (`specs/hoverboard-bridge.md`, "Its store"): the `store` crate the boards use, over the
//! `keys` partition. Its first two 4 KiB sectors are the store's two pages. The C3 does not
//! memory-map that partition, so the 8 KiB region is mirrored in RAM: it is read into the mirror
//! at boot, [`Flash::as_bytes`] returns the mirror, and `program` and `erase_page` write through
//! with `esp-storage` and then update the mirror.
//!
//! A region that cannot be read, or a store that does not mount on it, leaves the bridge with an
//! erased mirror and no write-through ([`KeysFlash::ram_only`]): it runs unassigned with the
//! default name, `ESP_INFO` reports the store as not mounted, and nothing written survives a reset.

use base::error::FlashError;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::partitions::{read_partition_table, PARTITION_TABLE_MAX_LEN};
use esp_storage::FlashStorage;
use store::Flash;

/// The partition's label in `partitions.csv`.
const LABEL: &str = "keys";
/// One store page: one flash sector.
pub const PAGE: usize = 4096;
/// The store's region: two pages.
pub const REGION: usize = 2 * PAGE;
/// The most bytes one write-through moves at a time.
const CHUNK: usize = 256;

/// The `keys` region, mirrored in RAM.
pub struct KeysFlash {
    flash: FlashStorage<'static>,
    mirror: &'static mut [u8; REGION],
    /// The partition's flash offset; `None` once the region runs from RAM alone.
    base: Option<u32>,
}

impl KeysFlash {
    /// Find the `keys` partition in the partition table and read its region into `mirror`. A
    /// table without it, a partition smaller than the region, or a failed read leaves it in RAM.
    pub fn load(
        mut flash: FlashStorage<'static>,
        table: &mut [u8; PARTITION_TABLE_MAX_LEN],
        mirror: &'static mut [u8; REGION],
    ) -> Self {
        let base = read_partition_table(&mut flash, table).ok().and_then(|pt| {
            pt.iter()
                .find(|p| p.label_as_str() == LABEL && p.len() as usize >= REGION)
                .map(|p| p.offset())
        });
        let mut keys = KeysFlash {
            flash,
            mirror,
            base,
        };
        match keys.base {
            Some(at) if keys.flash.read(at, &mut keys.mirror[..]).is_ok() => {}
            _ => keys.ram_only(),
        }
        keys
    }

    /// Run from an erased mirror with no write-through: the store did not mount on the region.
    pub fn ram_only(&mut self) {
        self.base = None;
        self.mirror.fill(0xFF);
    }

    /// Whether writes reach the partition.
    pub fn persists(&self) -> bool {
        self.base.is_some()
    }
}

impl Flash for KeysFlash {
    fn page_size(&self) -> usize {
        PAGE
    }

    fn as_bytes(&self) -> &[u8] {
        &self.mirror[..]
    }

    fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
        let start = page * PAGE;
        if start + PAGE > REGION {
            return Err(FlashError::OutOfBounds);
        }
        if let Some(base) = self.base {
            let at = base + start as u32;
            self.flash
                .erase(at, at + PAGE as u32)
                .map_err(|_| FlashError::EraseFailed)?;
        }
        self.mirror[start..start + PAGE].fill(0xFF);
        Ok(())
    }

    /// Halfword-aligned and write-once, the rules the store is written to. The flash writes whole
    /// words, so each write-through covers the words the halfwords fall in, taking the bytes
    /// around them from the mirror: those are either erased or already programmed with the same
    /// value, and writing a byte's own value again changes nothing.
    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
        if !off.is_multiple_of(2) || !bytes.len().is_multiple_of(2) {
            return Err(FlashError::Misaligned);
        }
        let end = off + bytes.len();
        if end > REGION {
            return Err(FlashError::OutOfBounds);
        }
        if (off..end)
            .zip(bytes)
            .any(|(i, &b)| self.mirror[i] != 0xFF && self.mirror[i] != b)
        {
            return Err(FlashError::ProgramFailed);
        }
        if let Some(base) = self.base {
            let words_start = off & !3;
            let words_end = (end + 3) & !3;
            let mut at = words_start;
            while at < words_end {
                let n = (words_end - at).min(CHUNK);
                let mut buf = [0u8; CHUNK];
                buf[..n].copy_from_slice(&self.mirror[at..at + n]);
                for i in at..at + n {
                    if (off..end).contains(&i) {
                        buf[i - at] = bytes[i - off];
                    }
                }
                self.flash
                    .write(base + at as u32, &buf[..n])
                    .map_err(|_| FlashError::ProgramFailed)?;
                at += n;
            }
        }
        self.mirror[off..end].copy_from_slice(bytes);
        Ok(())
    }
}
