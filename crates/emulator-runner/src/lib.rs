//! Run a dummy firmware image under the Unicorn emulator at the GD32 memory map and read the result.
//!
//! Unicorn is CPU + memory only (no NVIC, SysTick, or async exception entry), which is exactly what
//! the polled dummy needs. We map flash at `0x0800_0000` and RAM at `0x2000_0000`, load the image's
//! loadable segments into flash at their physical addresses, take SP/PC from the vector table, write
//! the host input to `CMD_ADDR`, run until the firmware writes `RESULT_ADDR` (a memory-write hook
//! stops the run there), and read the [`TestResult`] back. An instruction-count cap is the hang
//! backstop: an image that never writes `RESULT_ADDR` hits the cap and returns "not ready".
//!
//! The release ELF drops its symbol table, so nothing here resolves symbols: addresses come from the
//! shared [`RESULT_ADDR`] / [`CMD_ADDR`] constants and SP/PC from the vector table.
//!
//! KNOWN unicorn gotcha (2.x): `Mode::MCLASS` rejects every Thumb instruction as invalid, so we use
//! `Arch::ARM` + `Mode::THUMB` and set the Thumb bit (`addr | 1`) on the `emu_start` begin address.
//!
//! The Unicorn executor is behind the `unicorn` feature (it links a cmake-built native lib). Without
//! it the crate is an empty host library, so a plain `cargo check` needs no native toolchain.

#![cfg(feature = "unicorn")]

use std::path::Path;

use object::{Object, ObjectSegment};
use test_shared::{dummy, TestResult, CMD_ADDR, RESULT_ADDR, RESULT_READY};
use unicorn_engine::{Arch, HookType, Mode, Prot, RegisterARM, Unicorn};

/// GD32 flash base. The vector table lives at the start of this region.
const FLASH_BASE: u64 = 0x0800_0000;
/// Flash window size to map (generous; the dummy image is a few KiB).
const FLASH_SIZE: u64 = 256 * 1024;
/// GD32 RAM base.
const RAM_BASE: u64 = 0x2000_0000;
/// RAM window size to map. Covers the full 8 KiB part plus the reserved tail (RESULT/CMD words).
const RAM_SIZE: u64 = 8 * 1024;

/// Hang backstop: max instructions before we give up and call the run "not ready". The dummy images
/// reach their write in a handful of instructions plus cortex-m-rt startup; this is comfortably
/// above that and well below anything that would make the test slow.
const MAX_INSTRUCTIONS: usize = 2_000_000;

/// Read 4 bytes little-endian from a slice.
fn read_u32_le(buf: &[u8]) -> u32 {
    u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]])
}

/// Load `image` into a fresh Unicorn instance, deliver `input`, run to the result write (or the
/// instruction cap), and read back the published [`TestResult`].
///
/// `ready` not reaching [`RESULT_READY`] means the run hung or faulted before publishing, which the
/// caller treats as a caught failure (the `dummy-hang` case).
pub fn run_image(image: &Path, input: u32) -> std::io::Result<TestResult> {
    let bytes = std::fs::read(image)?;
    let file = object::File::parse(&*bytes)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e.to_string()))?;

    let mut emu = Unicorn::new(Arch::ARM, Mode::THUMB).expect("unicorn init");

    emu.mem_map(FLASH_BASE, FLASH_SIZE, Prot::ALL)
        .expect("map flash");
    emu.mem_map(RAM_BASE, RAM_SIZE, Prot::ALL).expect("map ram");

    // Load every loadable segment into flash at its physical address. The release ELF has no symbol
    // table, but its program headers (segments) carry the bytes and their target addresses.
    for seg in file.segments() {
        let addr = seg.address();
        let data = seg
            .data()
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e.to_string()))?;
        if data.is_empty() {
            continue;
        }
        emu.mem_write(addr, data).expect("load segment");
    }

    // The Cortex-M vector table is at the start of flash: word 0 = initial SP, word 1 = reset vector
    // (PC, with the Thumb bit set). Read them from the bytes we just loaded into flash.
    let mut vt = [0u8; 8];
    emu.mem_read(FLASH_BASE, &mut vt)
        .expect("read vector table");
    let sp = read_u32_le(&vt[0..4]);
    let reset = read_u32_le(&vt[4..8]);

    emu.reg_write(RegisterARM::SP, sp as u64).expect("set sp");

    // Deliver the host input to the reserved RAM tail before running, mirroring SWD/silicon. It is
    // outside the linked RAM region, so startup .bss clear does not touch it.
    emu.mem_write(CMD_ADDR as u64, &input.to_le_bytes())
        .expect("write input");

    // Stop the run as soon as the firmware writes the result word. We hook writes to RESULT_ADDR
    // (the `ready` word, written LAST) and stop there; the value has been applied to RAM by the time
    // we read it back. We read the result back unconditionally afterwards, so the hook only needs to
    // end the run, not record anything: a hung image never triggers it and hits the instruction cap
    // instead, leaving `ready` unwritten.
    emu.add_mem_hook(
        HookType::MEM_WRITE,
        RESULT_ADDR as u64,
        RESULT_ADDR as u64 + 3,
        move |uc, _mem_type, _addr, _size, _value| {
            // emu_stop ends the run after the current block; the write itself still lands.
            let _ = uc.emu_stop();
            true
        },
    )
    .expect("add result-write hook");

    // Run from the reset vector with the Thumb bit set. The instruction-count cap is the hang
    // backstop: if the image never writes RESULT_ADDR, emu_start returns when the cap is reached.
    let begin = (reset as u64) | 1;
    let _ = emu.emu_start(begin, FLASH_BASE + FLASH_SIZE, 0, MAX_INSTRUCTIONS);

    // Read the result back regardless. If the image hung, `ready` is still whatever was in RAM
    // (zero, since we never wrote it), so `produced_correct_output` sees it as not ready.
    let mut result = [0u8; 8];
    emu.mem_read(RESULT_ADDR as u64, &mut result)
        .expect("read result");

    Ok(TestResult {
        ready: read_u32_le(&result[0..4]),
        output: read_u32_le(&result[4..8]),
    })
}

/// The host-side judgment: run the image and decide whether it produced the correct output. The
/// expected value comes from the shared [`dummy`], so there is no duplicated magic number and no
/// circular self-check. Returns false when the run hung (no `ready`) or returned the wrong output.
pub fn produced_correct_output(image: &Path, input: u32) -> bool {
    match run_image(image, input) {
        Ok(r) => r.ready == RESULT_READY && r.output == dummy(input),
        Err(_) => false,
    }
}
