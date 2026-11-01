//! The remembered bridge in flash (`specs/handheld-remote.md` section 3.2): the first 4 KB sector
//! of the `nvs` data partition, read and written with `esp-storage`. esp-radio opens no NVS (its
//! NVS hooks are stubs that return failure, `esp-radio/src/wifi/os_adapter/mod.rs`, and its misc
//! NVS is a RAM array, `common_adapter.rs`), so nothing else in the image uses the sector. The
//! record's format is `handheld_core::pick`'s.
//!
//! The view runs on the second core, so the flash is opened with `multicore_auto_park`: a write
//! parks the second core for its length. Writes happen only on the Connect screen (a tap or the
//! not-a-bridge rule), never while connected to a machine.

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::partitions::{
    read_partition_table, DataPartitionSubType, PartitionType, PARTITION_TABLE_MAX_LEN,
};
use esp_println::println;
use esp_storage::FlashStorage;

use handheld_core::pick::{self, Pick, RECORD_MAX};

/// One flash sector.
const SECTOR: u32 = 4096;

/// The record's sector.
pub struct PickFlash {
    flash: FlashStorage<'static>,
    at: u32,
}

impl PickFlash {
    /// Find the `nvs` data partition in the partition table. `None` when the table has none: the
    /// pick then lives in RAM until the next boot.
    pub fn open(mut flash: FlashStorage<'static>) -> Option<PickFlash> {
        let mut table = [0u8; PARTITION_TABLE_MAX_LEN];
        let at = read_partition_table(&mut flash, &mut table)
            .ok()?
            .iter()
            .find(|p| {
                p.partition_type() == PartitionType::Data(DataPartitionSubType::Nvs)
                    && p.len() >= SECTOR
            })?
            .offset();
        println!("handheld: the pick is kept at {:#x}", at);
        Some(PickFlash { flash, at })
    }

    /// The pick the sector holds, if it holds a valid record.
    pub fn load(&mut self) -> Option<Pick> {
        let mut r = [0u8; RECORD_MAX];
        self.flash.read(self.at, &mut r).ok()?;
        pick::decode(&r)
    }

    /// Erase the sector and write `p`'s record.
    pub fn store(&mut self, p: &Pick) {
        let ok = self.flash.erase(self.at, self.at + SECTOR).is_ok()
            && self.flash.write(self.at, &pick::encode(p)).is_ok();
        println!(
            "handheld: pick {} {}",
            pick::addr_text(&p.addr),
            if ok { "written" } else { "write FAILED" }
        );
    }

    /// Erase the sector: no pick.
    pub fn erase(&mut self) {
        let ok = self.flash.erase(self.at, self.at + SECTOR).is_ok();
        println!(
            "handheld: pick {}",
            if ok { "erased" } else { "erase FAILED" }
        );
    }
}
