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

/// The flash alignment `.text` must keep, in bytes.
///
/// **This is a measured constraint, not a tidiness preference.** `.text` starts immediately after
/// the vector table, so shortening the table moves every function in the image, and these parts run
/// 2 flash wait states with no cache or prefetch, which makes a hot path's position relative to the
/// flash fetch line load-bearing. Shortening the table to exactly `MAX_IRQS` cost the F130 slave
/// real control ticks. Measured on silicon, 2026-07-26, F130 slave, settled 30 s windows, against
/// the same-session baseline (which holds `control:tick` 1.0000 with the tick/control offset
/// constant at 1):
///
/// | table | `.text` base | span vs baseline | F130 `control:tick` |
/// |---|---|---|---|
/// | 240 entries (cortex-m-rt default) | `0x0800_0400` | - | 1.0000 exact |
/// | 74 (`MAX_IRQS`, unpadded) | `0x0800_0168` | -664 B | **0.9960** (offset +5 per 1,260) |
/// | 96 (128-aligned) | `0x0800_0180` | -640 B | **0.9997** (offset +4 per 15,103) |
/// | 128 (512-aligned) | `0x0800_0200` | -512 B | 1.0000 exact (offset constant at 1) |
///
/// The F103 master held 1.0000 throughout, which is exactly why the both-family rule exists. 512 is
/// the smallest boundary tested that reproduces the baseline timing on BOTH families; 128 measurably
/// does not, so the relevant granularity is larger than one fetch line (the reclaim is bought back
/// down from 664 B to 512 B, and that is the price of the gate). Lowering this constant re-opens a
/// silicon question: re-measure both families before changing it.
#[cfg(target_arch = "arm")]
const TEXT_ALIGN: usize = 512;

/// Entries in the interrupt table: `runtime_hal::irq::MAX_IRQS`, rounded up so the whole
/// `.vector_table` section (the 16 system-exception slots plus these) is a multiple of
/// [`TEXT_ALIGN`]. The extra entries over `MAX_IRQS` are unreachable padding, and over-provisioning
/// the IRQ table is already this project's rule.
#[cfg(target_arch = "arm")]
const ENTRIES: usize = {
    let per = TEXT_ALIGN / 4; // table entries per alignment unit
    let total = runtime_hal::irq::SYSTEM_VECTORS + runtime_hal::irq::MAX_IRQS;
    total.next_multiple_of(per) - runtime_hal::irq::SYSTEM_VECTORS
};

/// The device interrupt portion of the flash vector table: [`ENTRIES`] long instead of
/// cortex-m-rt's generic 240.
///
/// The length is derived from runtime-hal's IRQ model rather than restated, so the table cannot
/// drift out of agreement with the RAM table built from the same constant.
#[cfg(target_arch = "arm")]
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [Vector; ENTRIES] = [Vector {
    handler: DefaultHandler,
}; ENTRIES];
