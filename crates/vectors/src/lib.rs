//! The flash interrupt vector table for every thumbv7m image in this workspace.
//!
//! # Why this crate exists
//!
//! cortex-m-rt ships a generic `__INTERRUPTS` of **240** entries (the ARMv7-M architectural
//! maximum), 960 B of flash. Neither GD32 family in this fleet has anywhere near that many
//! interrupts: runtime-hal's IRQ model sizes the union of both families at
//! [`runtime_hal::irq::MAX_IRQS`] entries (F1x0's highest external IRQ, `CAN1_SCE` = 73, which also
//! covers F10x's highest of 59). Supplying a table of that length instead reclaims the difference.
//!
//! Switching cortex-m-rt's default off is done with its `device` feature, and Cargo unifies
//! features across the **one shared cortex-m-rt build** for the whole workspace. So the choice is
//! not per-binary: the moment `device` is on, *every* thumbv7m binary here (dummy-test, store-test,
//! l2-uart-bench, ble-loopback-test, wdg-bench, imu-bench, firmware) loses the default table and
//! must get one from somewhere. This crate is that somewhere -- one table, one length, one owner,
//! depended on by all of them, rather than a copy per binary or a release-only feature that would
//! make the built image differ from the `cargo build` image.
//!
//! # Why every slot is `DefaultHandler`
//!
//! This flash table is consulted only in the window between reset and runtime-hal's `VTOR` flip
//! onto its **RAM** vector table, which is what actually dispatches (it is specialized per detected
//! family, because the F1x0 grouped and F10x separate IRQ layouts put the same peripheral at
//! different slots; see `runtime_hal::irq`). No interrupt is unmasked before that flip, so no IRQ
//! slot here can be taken. The entries exist to occupy the architectural table positions and to
//! give a defined landing point if one ever were.
//!
//! # Linking
//!
//! `__INTERRUPTS` is referenced by cortex-m-rt's `link.x` (`EXTERN(__INTERRUPTS)`) and placed by
//! its `.vector_table.interrupts` section, so it is pulled in by the linker rather than by any Rust
//! call. Binaries therefore depend on this crate for its symbols only, and mark that with
//! `use vectors as _;` so the rlib reaches the link line.

#![no_std]

/// A vector-table entry.
///
/// The Cortex-M ABI wants an array of function pointers, but reserved slots in a device's table
/// hold a plain word, so the entry is the usual union of the two (the same shape svd2rust-generated
/// device crates emit). Every entry this crate builds uses the handler arm.
#[cfg(target_arch = "arm")]
#[derive(Clone, Copy)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    _reserved: u32,
}

#[cfg(target_arch = "arm")]
extern "C" {
    fn DefaultHandler();
}

/// The device interrupt portion of the flash vector table, `runtime_hal::irq::MAX_IRQS` entries
/// long instead of cortex-m-rt's generic 240.
///
/// The length is read from runtime-hal's IRQ model rather than restated, so the table cannot drift
/// out of agreement with the RAM table built from the same constant.
#[cfg(target_arch = "arm")]
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [Vector; runtime_hal::irq::MAX_IRQS] = [Vector {
    handler: DefaultHandler,
}; runtime_hal::irq::MAX_IRQS];
