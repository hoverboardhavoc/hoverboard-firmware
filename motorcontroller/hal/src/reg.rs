// SPDX-License-Identifier: GPL-3.0-or-later
//! Volatile register access: one word or half-word at `base + offset`.
//!
//! Every peripheral model reaches its registers through these, so a register access is a single
//! width-exact volatile load or store (the golden traces compare widths strictly, section 8.1 of
//! specs/hal.md). The host build compiles them but nothing on the host calls them: the host tests
//! cover only the pure parts.

/// A 32-bit memory-mapped register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reg32 {
    addr: u32,
}

impl Reg32 {
    /// The register at `base + offset`.
    #[inline(always)]
    pub const fn new(base: u32, offset: u32) -> Self {
        Self {
            addr: base + offset,
        }
    }

    /// The register's absolute address.
    #[inline(always)]
    pub const fn addr(&self) -> u32 {
        self.addr
    }

    /// One volatile 32-bit load.
    #[inline(always)]
    pub fn read(&self) -> u32 {
        // SAFETY: the address is a word-aligned peripheral register of the compiled chip, named by
        // the model that constructed it.
        unsafe { core::ptr::read_volatile(self.addr as usize as *const u32) }
    }

    /// One volatile 32-bit store.
    #[inline(always)]
    pub fn write(&self, value: u32) {
        // SAFETY: as `read`.
        unsafe { core::ptr::write_volatile(self.addr as usize as *mut u32, value) }
    }

    /// Read, replace the bits in `mask` with `value & mask`, write back.
    #[inline(always)]
    pub fn modify(&self, mask: u32, value: u32) {
        let v = self.read();
        self.write((v & !mask) | (value & mask));
    }

    /// Set the bits in `bits` (a read-modify-write).
    #[inline(always)]
    pub fn set_bits(&self, bits: u32) {
        self.modify(bits, bits);
    }

    /// Clear the bits in `bits` (a read-modify-write).
    #[inline(always)]
    pub fn clear_bits(&self, bits: u32) {
        self.modify(bits, 0);
    }
}

/// A 16-bit memory-mapped register (the FWDGT key and reload registers take half-word stores on
/// some parts; the FMC programs half-words).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reg16 {
    addr: u32,
}

impl Reg16 {
    /// The register at `base + offset`.
    #[inline(always)]
    pub const fn new(base: u32, offset: u32) -> Self {
        Self {
            addr: base + offset,
        }
    }

    /// One volatile 16-bit load.
    #[inline(always)]
    pub fn read(&self) -> u16 {
        // SAFETY: a half-word-aligned register or flash half-word of the compiled chip.
        unsafe { core::ptr::read_volatile(self.addr as usize as *const u16) }
    }

    /// One volatile 16-bit store.
    #[inline(always)]
    pub fn write(&self, value: u16) {
        // SAFETY: as `read`.
        unsafe { core::ptr::write_volatile(self.addr as usize as *mut u16, value) }
    }
}
