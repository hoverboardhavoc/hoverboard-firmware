//! The bridge itself, as the command service asks for it (`hoverboard_bridge_core::svc::Platform`): the Wi-Fi
//! station's state, and its own flash and boot selection for its updates (`specs/hoverboard-bridge.md`,
//! "Installing and updating the bridge").
//!
//! The partition table (`partitions.csv`) has two app slots, `ota_0` and `ota_1`, and `otadata`,
//! which the ESP-IDF second-stage bootloader reads to choose the slot, built with rollback
//! (`bootloader/`). An update writes the slot not running, then selects it marked new; the
//! bootloader boots it pending verification, and at the next reset aborts it and runs the other
//! slot unless [`Platform::confirm`] marked it valid.
//!
//! The otadata entries are read and written with `esp-bootloader-esp-idf`'s `Ota`. Its
//! `OtaUpdater::next_partition` is not used: with otadata blank (a first install, running `ota_0`)
//! it computes the slot from the factory subtype with an underflow and names `ota_0`, the slot
//! running. The target here is the app slot that is not the booted one, and a blank otadata is
//! first pointed at the running slot so the sequence numbers start from a selected slot.

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::ota::OtaImageState;
use esp_bootloader_esp_idf::ota_updater::OtaUpdater;
use esp_bootloader_esp_idf::partitions::{
    read_partition_table, AppPartitionSubType, PartitionType, PARTITION_TABLE_MAX_LEN,
};
use esp_storage::FlashStorage;
use espsvc::{Status, WifiState, SLOT_NONE};

use hoverboard_bridge_core::svc::{Platform, Running};
use hoverboard_bridge_core::update::SECTOR;

/// The slot an update writes.
#[derive(Clone, Copy)]
struct Target {
    /// Its partition's flash offset and size.
    offset: u32,
    size: u32,
    sub: AppPartitionSubType,
}

/// The firmware's platform.
pub struct EspPlatform {
    flash: FlashStorage<'static>,
    table: &'static mut [u8; PARTITION_TABLE_MAX_LEN],
    target: Option<Target>,
}

fn slot_number(sub: AppPartitionSubType) -> Option<u8> {
    match sub {
        AppPartitionSubType::Ota0 => Some(0),
        AppPartitionSubType::Ota1 => Some(1),
        _ => None,
    }
}

impl EspPlatform {
    /// The platform over the ESP's flash. The station's state comes from [`crate::wifi::state`].
    pub fn new(
        flash: FlashStorage<'static>,
        table: &'static mut [u8; PARTITION_TABLE_MAX_LEN],
    ) -> Self {
        EspPlatform {
            flash,
            table,
            target: None,
        }
    }

    /// The app slot the bootloader loaded this image from.
    fn booted(&mut self) -> Option<AppPartitionSubType> {
        let pt = read_partition_table(&mut self.flash, self.table).ok()?;
        match pt.booted_partition().ok()??.partition_type() {
            PartitionType::App(sub) => Some(sub),
            _ => None,
        }
    }

    /// The slot otadata selects and its state; `None` for either when otadata is blank.
    fn selected(&mut self) -> Option<(AppPartitionSubType, OtaImageState)> {
        let mut ota = OtaUpdater::new(&mut self.flash, self.table).ok()?;
        let sub = ota.selected_partition().ok()?;
        let state = ota.current_ota_state().ok()?;
        Some((sub, state))
    }

    fn pending(&mut self) -> bool {
        let booted = self.booted();
        booted.is_some()
            && matches!(self.selected(), Some((s, OtaImageState::PendingVerify)) if Some(s) == booted)
    }
}

impl Platform for EspPlatform {
    fn wifi(&self) -> (WifiState, [u8; 4]) {
        crate::wifi::state()
    }

    fn running(&mut self) -> Running {
        let slot = self.booted().and_then(slot_number).unwrap_or(SLOT_NONE);
        Running {
            slot,
            pending: self.pending(),
        }
    }

    fn update_begin(&mut self, len: u32) -> Result<(), Status> {
        self.target = None;
        let sub = match self.booted() {
            Some(AppPartitionSubType::Ota0) => AppPartitionSubType::Ota1,
            Some(AppPartitionSubType::Ota1) => AppPartitionSubType::Ota0,
            _ => return Err(Status::FlashError),
        };
        let pt =
            read_partition_table(&mut self.flash, self.table).map_err(|_| Status::FlashError)?;
        let part = pt
            .find_partition(PartitionType::App(sub))
            .ok()
            .flatten()
            .ok_or(Status::FlashError)?;
        let target = Target {
            offset: part.offset(),
            size: part.len(),
            sub,
        };
        if len > target.size {
            return Err(Status::TooLarge);
        }
        self.target = Some(target);
        Ok(())
    }

    fn update_erase(&mut self, offset: u32) -> Result<(), Status> {
        let t = self.target.ok_or(Status::BadRequest)?;
        if !(offset as usize).is_multiple_of(SECTOR) || offset >= t.size {
            return Err(Status::BadRequest);
        }
        let at = t.offset + offset;
        self.flash
            .erase(at, at + SECTOR as u32)
            .map_err(|_| Status::FlashError)
    }

    fn update_write(&mut self, offset: u32, data: &[u8]) -> Result<(), Status> {
        let t = self.target.ok_or(Status::BadRequest)?;
        if !(offset as usize).is_multiple_of(SECTOR) || data.len() > SECTOR || offset >= t.size {
            return Err(Status::BadRequest);
        }
        let at = t.offset + offset;
        let fail = |_| Status::FlashError;
        // Whole words; the last word of an image whose length is not one is padded erased.
        let whole = data.len() / 4 * 4;
        self.flash.write(at, &data[..whole]).map_err(fail)?;
        if whole < data.len() {
            let mut word = [0xFF; 4];
            word[..data.len() - whole].copy_from_slice(&data[whole..]);
            self.flash.write(at + whole as u32, &word).map_err(fail)?;
        }
        Ok(())
    }

    fn update_read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Status> {
        let t = self.target.ok_or(Status::BadRequest)?;
        if offset + buf.len() as u32 > t.size {
            return Err(Status::BadRequest);
        }
        self.flash
            .read(t.offset + offset, buf)
            .map_err(|_| Status::FlashError)
    }

    fn update_activate(&mut self) -> Result<(), Status> {
        let t = self.target.ok_or(Status::BadRequest)?;
        let booted = self.booted().ok_or(Status::FlashError)?;
        let fail = |_| Status::FlashError;
        let mut ota = OtaUpdater::new(&mut self.flash, self.table).map_err(fail)?;
        let mut data = ota.ota_data().map_err(fail)?;
        if data.current_app_partition().map_err(fail)? == AppPartitionSubType::Factory {
            data.set_current_app_partition(booted).map_err(fail)?;
        }
        data.set_current_app_partition(t.sub).map_err(fail)?;
        data.set_current_ota_state(OtaImageState::New)
            .map_err(fail)?;
        Ok(())
    }

    fn confirm(&mut self) -> Result<(), Status> {
        if !self.pending() {
            return Ok(());
        }
        let fail = |_| Status::FlashError;
        let mut ota = OtaUpdater::new(&mut self.flash, self.table).map_err(fail)?;
        ota.set_current_ota_state(OtaImageState::Valid)
            .map_err(fail)
    }
}
