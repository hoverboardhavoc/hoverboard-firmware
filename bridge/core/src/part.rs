//! The part table (`specs/bridge-chip-support.md`, section 4): every part the bridge identifies,
//! flashes, erases the store of and clears the protection of is one row, and every constant that
//! depends on the part (the store address, the image limit, the RAM loader's ring and stack, the
//! debug hold) derives from its row here. A part in no row is refused
//! ([`espsvc::Status::UnsupportedPart`]).

use espsvc::{FAMILY_F10X, FAMILY_F1X0, FAMILY_MM32SPIN05};

/// Main flash base on every row.
pub const FLASH_BASE: u32 = store::geometry::FLASH_BASE;

/// The debug block a part's core has, which says where its debug hold is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Debug {
    /// A GD32 (Cortex-M3): `DBG_CTL0` at `0xE004_2004`.
    Gd32,
    /// The MM32SPIN05 (Cortex-M0): `DBGMCU_CR` at `0x4001_3404`.
    Mm32,
}

/// A debug hold: the register and the bits that freeze the free watchdog and timer 0 while the
/// core is halted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hold {
    /// The register.
    pub addr: u32,
    /// The bits to set.
    pub bits: u32,
}

impl Debug {
    /// The hold: bit 8 the free watchdog, bit 10 timer 0, on both (GD32 `FWDGT_HOLD` and
    /// `TIMER0_HOLD`; MindMotion `reg_dbg.h` `DBG_IWDG_STOP` and `DBG_TIM1_STOP`).
    pub const fn hold(self) -> Hold {
        Hold {
            addr: match self {
                Debug::Gd32 => 0xE004_2004,
                Debug::Mm32 => 0x4001_3404,
            },
            bits: 1 << 8 | 1 << 10,
        }
    }
}

/// The core, as `CPUID`'s part number gives it, and on a Cortex-M0 the ID register value that
/// names the part (`specs/bridge-chip-support.md`, section 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Core {
    /// A Cortex-M3 (part number `0xC23`): a GD32, told apart by its UID and flash size registers.
    M3,
    /// A Cortex-M0 (part number `0xC20`) whose ID register at [`M0_ID`] reads `id`.
    M0 {
        /// The ID register value.
        id: u32,
    },
}

/// `CPUID`'s part number for a Cortex-M0.
pub const PARTNO_M0: u32 = 0xC20;
/// `CPUID`'s part number for a Cortex-M3.
pub const PARTNO_M3: u32 = 0xC23;
/// The ID register on a Cortex-M0 part (the MM32's `DBGMCU_IDCODE`).
pub const M0_ID: u32 = 0x4001_3400;

/// `CPUID`'s part number field.
pub const fn partno(cpuid: u32) -> u32 {
    (cpuid >> 4) & 0xFFF
}

/// One supported part.
#[derive(Debug, PartialEq, Eq)]
pub struct Part {
    /// The part's name, for a terminal.
    pub name: &'static str,
    /// The core, and on a Cortex-M0 its ID.
    pub core: Core,
    /// `TargetId::family`.
    pub family: u8,
    /// Main flash, KiB.
    pub flash_kib: u16,
    /// The erase page, bytes.
    pub page: u32,
    /// SRAM, KiB.
    pub sram_kib: u16,
    /// The RAM loader's ring, bytes.
    pub ring: u32,
    /// The debug block, which gives the debug hold.
    pub debug: Debug,
}

/// The GD32F103 with 64 KiB of flash (the rover's mainboards).
pub const GD32F103_64: Part = Part {
    name: "GD32F103 (64 KiB)",
    core: Core::M3,
    family: FAMILY_F10X,
    flash_kib: 64,
    page: 1024,
    sram_kib: 20,
    ring: 4096,
    debug: Debug::Gd32,
};

/// The GD32F130C8 (the offroad's mainboards).
pub const GD32F130C8: Part = Part {
    name: "GD32F130C8",
    core: Core::M3,
    family: FAMILY_F1X0,
    flash_kib: 64,
    page: 1024,
    sram_kib: 8,
    ring: 4096,
    debug: Debug::Gd32,
};

/// The GD32F130C6 (the sideboards), from its datasheet; the flash size register tells it from
/// the C8.
pub const GD32F130C6: Part = Part {
    name: "GD32F130C6",
    core: Core::M3,
    family: FAMILY_F1X0,
    flash_kib: 32,
    page: 1024,
    sram_kib: 4,
    ring: 2048,
    debug: Debug::Gd32,
};

/// The MM32SPIN05 (the Gen2.4.2 mainboards). Its 32 KiB was measured on the bench
/// (`specs/bench-work/mm32/README.md`); it has no flash size register.
pub const MM32SPIN05: Part = Part {
    name: "MM32SPIN05",
    core: Core::M0 { id: 0xCC44_60B1 },
    family: FAMILY_MM32SPIN05,
    flash_kib: 32,
    page: 1024,
    sram_kib: 4,
    ring: 2048,
    debug: Debug::Mm32,
};

/// Every supported part.
pub const PARTS: [&Part; 4] = [&GD32F103_64, &GD32F130C8, &GD32F130C6, &MM32SPIN05];

/// The SRAM base.
pub const SRAM_BASE: u32 = 0x2000_0000;
/// The loader's stack top is capped here: the 20 KiB F103 uses the F130C8's layout.
pub const STACK_CAP: u32 = 0x2000_2000;

/// The row for a family and flash size, as `TARGET_ID` reports them.
pub fn lookup(family: u8, flash_kib: u16) -> Option<&'static Part> {
    PARTS
        .iter()
        .copied()
        .find(|p| p.family == family && p.flash_kib == flash_kib)
}

/// The Cortex-M0 row whose ID register reads `id`. Its flash size comes from the row: the MM32
/// has no flash size register.
pub fn m0(id: u32) -> Option<&'static Part> {
    PARTS.iter().copied().find(|p| p.core == Core::M0 { id })
}

/// The debug block of every Cortex-M3 row. `debug_hold` sets it on an M3 before it knows which
/// GD32 it is; a const check below keeps every M3 row on it.
pub const M3_DEBUG: Debug = Debug::Gd32;

const _: () = {
    let mut i = 0;
    while i < PARTS.len() {
        if let Core::M3 = PARTS[i].core {
            assert!(PARTS[i].debug as u8 == M3_DEBUG as u8);
        }
        i += 1;
    }
};

impl Part {
    /// The store's first byte: the store crate's placement rule (`shared/store/src/geometry.rs`),
    /// the one the firmware mounts its store with.
    pub const fn store_start(&self) -> u32 {
        store::geometry::store_base(self.flash_kib as u32 * 1024, self.page)
    }

    /// The store region's length: two pages.
    pub const fn store_len(&self) -> u32 {
        store::geometry::region_len(self.page as usize) as u32
    }

    /// The largest image: everything below the store.
    pub const fn max_image(&self) -> usize {
        (self.store_start() - FLASH_BASE) as usize
    }

    /// The loader's stack top: the end of the row's SRAM, capped at [`STACK_CAP`].
    pub const fn stack_top(&self) -> u32 {
        let end = SRAM_BASE + self.sram_kib as u32 * 1024;
        if end < STACK_CAP {
            end
        } else {
            STACK_CAP
        }
    }

    /// The debug hold.
    pub const fn hold(&self) -> Hold {
        self.debug.hold()
    }
}

/// The largest image any row takes: the size of the bridge's image buffer.
pub const MAX_IMAGE: usize = {
    let mut m = 0;
    let mut i = 0;
    while i < PARTS.len() {
        if PARTS[i].max_image() > m {
            m = PARTS[i].max_image();
        }
        i += 1;
    }
    m
};

/// The lowest store address of any row: where a GDB session's flash ends when the part is not
/// known (`crate::gdb`).
pub const MIN_STORE_START: u32 = {
    let mut m = u32::MAX;
    let mut i = 0;
    while i < PARTS.len() {
        if PARTS[i].store_start() < m {
            m = PARTS[i].store_start();
        }
        i += 1;
    }
    m
};
