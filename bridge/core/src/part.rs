//! The part table (`specs/bridge-chip-support.md`, section 4): every part the bridge identifies,
//! flashes, erases the store of and clears the protection of is one row, and every constant that
//! depends on the part (the store address, the erase page, the image limit, the RAM loader's ring
//! and stack, the debug hold, the advanced timers the armed check reads) derives from its row
//! here. A part in no row is refused ([`espsvc::Status::UnsupportedPart`]).

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

/// A debug hold: the register and the bits that freeze the free watchdog and the advanced timers
/// while the core is halted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hold {
    /// The register.
    pub addr: u32,
    /// The bits to set.
    pub bits: u32,
}

impl Debug {
    /// The hold: bit 8 the free watchdog, bit 10 timer 0, on both (GD32 `FWDGT_HOLD` and
    /// `TIMER0_HOLD`; MindMotion `reg_dbg.h` `DBG_IWDG_STOP` and `DBG_TIM1_STOP`). On a GD32 also
    /// bit 17, `TIMER7_HOLD` on the GD32F10x (User Manual Rev2.6 section 10.4.2), which freezes the
    /// GD32F103RC's second bridge; it is `I2C2_HOLD` on the GD32F1x0 (User Manual Rev3.6 section
    /// 9.4.2), a peripheral neither the F103C8 nor the F130 has, so it is inert there. Every GD32
    /// row takes it because [`crate::target::debug_hold`] sets the hold before it knows which GD32
    /// is on the line.
    pub const fn hold(self) -> Hold {
        match self {
            Debug::Gd32 => Hold {
                addr: 0xE004_2004,
                bits: 1 << 8 | 1 << 10 | 1 << 17,
            },
            Debug::Mm32 => Hold {
                addr: 0x4001_3404,
                bits: 1 << 8 | 1 << 10,
            },
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

/// Timer 0's `CCHP`, whose bit 15 (`POEN`, the outputs enabled) the armed check reads and the
/// disarm clears. The MM32's TIM1 `BDTR` is at the same address with `MOE` at the same bit
/// (MindMotion `reg_tim.h`).
pub const TIMER0_CCHP: u32 = 0x4001_2C44;
/// Timer 7's `CCHP` on a high-density GD32F10x (timer 7 at `0x4001_3400`). Read only on a row that
/// lists it: on the other parts the address is another peripheral or nothing (`specs/hal.md` 8.4,
/// the stalled-bus incident).
pub const TIMER7_CCHP: u32 = 0x4001_3444;
/// The advanced timers of a part with timer 0 alone.
pub const TIMER0_ONLY: &[u32] = &[TIMER0_CCHP];

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
    /// The `CCHP` of each advanced timer the part has, timer 0's first: `TARGET_ID.armed` is any
    /// of their `POEN` bits set, and the disarm clears every one.
    pub timers: &'static [u32],
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
    timers: TIMER0_ONLY,
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
    timers: TIMER0_ONLY,
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
    timers: TIMER0_ONLY,
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
    timers: TIMER0_ONLY,
};

/// The GD32F103RC (the 12-FET, `specs/twelve-fet.md` section 3.4): 256 KiB in 2 KiB pages and
/// 48 KiB of SRAM (GD32F103xx Datasheet Rev2.14, Table 2-2; User Manual Rev2.6 section 2.2), and
/// a second advanced timer, timer 7, for its second bridge. Its flash size register reads 256;
/// its image limit is the image buffer's, not everything below its store.
pub const GD32F103RC: Part = Part {
    name: "GD32F103RC",
    core: Core::M3,
    family: FAMILY_F10X,
    flash_kib: 256,
    page: 2048,
    sram_kib: 48,
    ring: 4096,
    debug: Debug::Gd32,
    timers: &[TIMER0_CCHP, TIMER7_CCHP],
};

/// Every supported part.
pub const PARTS: [&Part; 5] = [
    &GD32F103_64,
    &GD32F130C8,
    &GD32F130C6,
    &MM32SPIN05,
    &GD32F103RC,
];

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

    /// The largest image: everything below the store, capped at the bridge's image buffer
    /// ([`IMAGE_BUFFER`]).
    pub const fn max_image(&self) -> usize {
        let below_store = (self.store_start() - FLASH_BASE) as usize;
        if below_store < IMAGE_BUFFER {
            below_store
        } else {
            IMAGE_BUFFER
        }
    }

    /// The first byte past the largest image: where a GDB session's flash ends.
    pub const fn image_end(&self) -> u32 {
        FLASH_BASE + self.max_image() as u32
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

/// The bridge's image buffer, in its RAM: 63,488 B, the 64 KiB rows' limit. A row's image limit
/// is capped at it ([`Part::max_image`]), so a larger part does not grow it: the ESP32-C3 also
/// runs BLE and Wi-Fi in its RAM (`specs/twelve-fet.md` section 3.4).
pub const IMAGE_BUFFER: usize = 63_488;

/// The smallest image limit of any row: where a GDB session's flash ends when the part is not
/// known (`crate::gdb`).
pub const MIN_IMAGE: usize = {
    let mut m = usize::MAX;
    let mut i = 0;
    while i < PARTS.len() {
        if PARTS[i].max_image() < m {
            m = PARTS[i].max_image();
        }
        i += 1;
    }
    m
};

/// The smallest erase page of any row: a GDB session's flash block when the part is not known.
pub const MIN_PAGE: u32 = {
    let mut m = u32::MAX;
    let mut i = 0;
    while i < PARTS.len() {
        if PARTS[i].page < m {
            m = PARTS[i].page;
        }
        i += 1;
    }
    m
};

/// Every row lists timer 0 first: the armed check and the disarm of a part in no row read timer 0
/// alone, as every row's first.
const _: () = {
    let mut i = 0;
    while i < PARTS.len() {
        assert!(PARTS[i].timers[0] == TIMER0_CCHP);
        i += 1;
    }
};
