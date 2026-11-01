// SPDX-License-Identifier: GPL-3.0-or-later
//! `fmc`: `hal::fmc::Fmc` on silicon (specs/hal.md section 8.4).
//!
//! Scratch pages: the two pages below the store's two (on a 64 KiB part `0x0800_F000` and
//! `0x0800_F400`), above any validator image and outside the store, which the bridge's flash path
//! also never reaches. Erases both, programs 32 bytes across their boundary, reads them back,
//! re-programs a written half-word (must be refused), tries out-of-range and misaligned addresses
//! (must be refused), and erases both again (must read back erased). Leaves both pages erased.
//!
//! Result words (error codes: 0 ok, 1 BadArg, 2 NotErased, 3 Closed, 4 WriteProtect,
//! 5 ProgramError, 6 Timeout): 2 = scratch base, 3 = page size, 4 = erase codes (page0 | page1 <<
//! 8), 5 = erased-before-program check (1 = both pages read 0xFFFF), 6 = program code, 7 = bytes
//! that read back wrong, 8 = re-program code (want nonzero), 9 = cell after the refused re-program
//! (want the first program's half-word), 10 = out-of-range code (want 1), 11 = misaligned code (want
//! 1), 12 = final erase codes, 13 = final erased check (1 = both pages erased), 14 = verdict (1 = all
//! as wanted).
#![no_std]
#![no_main]

#[cfg(target_os = "none")]
mod image {
    use cortex_m_rt::entry;
    use hal::error::FmcError;
    use hal::fmc::Fmc;
    use hal::Chip;
    use hal_bench::{finish, put};
    use panic_halt as _;

    fn code(r: Result<(), FmcError>) -> u32 {
        match r {
            Ok(()) => 0,
            Err(FmcError::BadArg) => 1,
            Err(FmcError::NotErased) => 2,
            Err(FmcError::Closed) => 3,
            Err(FmcError::WriteProtect) => 4,
            Err(FmcError::ProgramError) => 5,
            Err(FmcError::Timeout) => 6,
        }
    }

    fn erased(base: u32, len: u32) -> bool {
        (0..len)
            .step_by(4)
            // SAFETY: inside main flash, read-mapped.
            .all(|o| unsafe { core::ptr::read_volatile((base + o) as *const u32) } == u32::MAX)
    }

    #[entry]
    fn main() -> ! {
        let Some(chip) = Chip::take() else { finish(2) };
        let mut fmc = Fmc::new(&chip);
        let page = fmc.page_size();
        let base = 0x0800_0000 + fmc.flash_size_bytes() - 4 * page;
        put(2, base);
        put(3, page);

        let e0 = code(fmc.erase_page(base));
        let e1 = code(fmc.erase_page(base + page));
        put(4, e0 | e1 << 8);
        let clean = erased(base, 2 * page);
        put(5, u32::from(clean));

        let at = base + page - 16;
        let mut data = [0u8; 32];
        for (i, b) in data.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(37).wrapping_add(5);
        }
        let p = code(fmc.program(at, &data));
        put(6, p);
        let wrong = data
            .iter()
            .enumerate()
            // SAFETY: the bytes just programmed, inside main flash.
            .filter(
                |(i, b)| unsafe { core::ptr::read_volatile((at + *i as u32) as *const u8) } != **b,
            )
            .count() as u32;
        put(7, wrong);

        let again = code(fmc.program(at, &[0x00, 0x00]));
        put(8, again);
        // SAFETY: as above.
        let cell = u32::from(unsafe { core::ptr::read_volatile(at as *const u16) });
        put(9, cell);

        let range = code(fmc.program(0x0800_0000 + fmc.flash_size_bytes(), &[0, 0]));
        put(10, range);
        let odd = code(fmc.program(base + 1, &[0, 0]));
        put(11, odd);

        let f0 = code(fmc.erase_page(base));
        let f1 = code(fmc.erase_page(base + page));
        put(12, f0 | f1 << 8);
        let clean_after = erased(base, 2 * page);
        put(13, u32::from(clean_after));

        let first = u32::from(data[0]) | u32::from(data[1]) << 8;
        let ok = e0 == 0
            && e1 == 0
            && clean
            && p == 0
            && wrong == 0
            && again != 0
            && cell == first
            && range == 1
            && odd == 1
            && f0 == 0
            && f1 == 0
            && clean_after;
        put(14, u32::from(ok));
        finish(2)
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
