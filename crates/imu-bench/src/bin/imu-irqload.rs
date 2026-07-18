//! Controlled ISR-load repro for the GD32 I2C master-receive race (the 2026-07-18 `imu_live=0`
//! root cause; see runtime-hal `src/i2c.rs` module docs).
//!
//! The integrated firmware's 14-byte IMU burst wedged the I2C block on silicon under SysTick +
//! DMA interrupt load, while the identical burst runs error-free in the interrupt-free
//! `imu-bench` image. The manual's Solution A receive "requires the software's quick response":
//! an interrupt landing between the byte-N-1 read and the ACKEN-clear/STOP lets the last byte be
//! ACKed and the STOP fall mid-byte, wedging the block (CTL0 START|STOP pending, I2CBSY stuck
//! with MASTER dropped). This image is the controlled stimulus:
//!
//! - **SysTick INTERRUPT** at ~10 kHz, each handler burning ~40 us (~40% CPU in ISR), so a
//!   ~1.6 ms burst takes ~16 interruptions and the race window is hit within seconds.
//! - **Main loop**: back-to-back 14-byte burst reads (no pacing; maximum exposure), counting
//!   successes and errors, snapshotting the I2C registers at the FIRST error.
//!
//! Built twice, against the PRE-fix HAL (Solution A, no recovery: expect `samples` to freeze
//! and `errs` to climb once wedged, with the wedge signature in the register snapshot) and the
//! FIXED HAL (Solution B + recovery: expect `samples` climbing indefinitely, `errs` = 0). That
//! A/B is the proof the fix addresses the trigger, not just the happy path.
//!
//! OBS block `IRQLOAD_OBS` (`#[repr(C)]`, offsets from the symbol base):
//! | off | field | meaning |
//! |---|---|---|
//! | 0x00 | `magic` | `0x4C51_5249` ("IRQL") once the loop is running (or with a fatal `err`) |
//! | 0x04 | `err` | fatal bring-up error (1 detect, 2 clock, 3 gpio, 4 i2c, 5 probe, 7 init), 0 ok |
//! | 0x08 | `samples` | successful 14-byte bursts |
//! | 0x0C | `errs` | failed bursts (running count) |
//! | 0x10 | `first_err_stat0` | I2C STAT0 at the first failure |
//! | 0x14 | `first_err_stat1` | I2C STAT1 at the first failure |
//! | 0x18 | `first_err_ctl0` | I2C CTL0 at the first failure |
//! | 0x1C | `ticks` | SysTick ISR entries (proves the load is live) |
//!
//! Busy-spins on fatal error, never `wfi` (GD32 SWD-lockout rule). Touches only PB6/PB7.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use core::sync::atomic::{AtomicU32, Ordering};

    use cortex_m::asm::nop;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m_rt::{entry, exception};
    use panic_halt as _;

    use embedded_hal::i2c::I2c as _;
    use imu::{Config, Imu, CLONE_2E};
    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::{detect_chip, I2c, I2cMode, PeriphLabel};
    use test_shared::obs_store;

    /// The production 72 MHz tree (matching the integrated image's clock regime).
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    /// I2C bus rate: 100 kHz standard mode (the failing configuration).
    const I2C_HZ: u32 = 100_000;
    /// SysTick interrupt rate: ~5 kHz. With the 120 us burn below this is ~60% CPU in the ISR,
    /// and the expected rate of an ISR entering the sub-us Solution-A fatal gap is ~1 per second
    /// of bursting (gap ~0.5 us x 5 kHz x ~400 bursts/s).
    const IRQ_HZ: u32 = 5_000;
    /// Cycles burned per handler: ~120 us at 72 MHz, LONGER than one I2C byte time at 100 kHz
    /// (~90 us). Only a preemption longer than the byte-N margin can push the Solution-A
    /// ACKEN-clear past the last byte's ACK slot; a first A/B round with a 40 us burn (10 kHz)
    /// confirmed a sub-byte-time preemption is harmless (66k bursts, 0 errors, pre-fix driver).
    const IRQ_BURN_CYCLES: u32 = 72 * 120;

    /// The I2C0 base for the first-error register snapshot (the shared family base; reading it
    /// raw here keeps the snapshot independent of the driver under test).
    const I2C0_BASE: u32 = 0x4000_5400;

    #[repr(C)]
    pub struct Obs {
        magic: u32,
        err: u32,
        samples: u32,
        errs: u32,
        first_err_stat0: u32,
        first_err_stat1: u32,
        first_err_ctl0: u32,
        ticks: u32,
    }

    const _: () = {
        assert!(core::mem::offset_of!(Obs, samples) == 0x08);
        assert!(core::mem::offset_of!(Obs, errs) == 0x0C);
        assert!(core::mem::offset_of!(Obs, first_err_stat0) == 0x10);
        assert!(core::mem::offset_of!(Obs, ticks) == 0x1C);
        assert!(core::mem::size_of::<Obs>() == 0x20);
    };

    const MAGIC: u32 = 0x4C51_5249; // "IRQL"

    #[no_mangle]
    static mut IRQLOAD_OBS: Obs = Obs {
        magic: 0,
        err: 0,
        samples: 0,
        errs: 0,
        first_err_stat0: 0,
        first_err_stat1: 0,
        first_err_ctl0: 0,
        ticks: 0,
    };

    /// ISR-side tick counter (the one ISR/thread crossing; the OBS mirror is thread-written).
    static TICKS: AtomicU32 = AtomicU32::new(0);

    /// The SysTick interrupt: bump the counter and burn ~40 us, so the polled I2C loop in the
    /// main thread is regularly preempted for about half a byte time.
    #[exception]
    fn SysTick() {
        TICKS.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::delay(IRQ_BURN_CYCLES);
    }

    fn fail(code: u32) -> ! {
        obs_store!(IRQLOAD_OBS, err, code);
        obs_store!(IRQLOAD_OBS, magic, MAGIC);
        loop {
            nop();
        }
    }

    #[entry]
    fn main() -> ! {
        let mut cp = cortex_m::Peripherals::take().unwrap();

        let chip = match detect_chip() {
            Ok(c) => c,
            Err(_) => fail(1),
        };
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            fail(2);
        }
        let gpiob = match chip.gpiob() {
            Ok(p) => p.split(),
            Err(_) => fail(3),
        };
        let mut i2c = match I2c::new(
            &chip,
            &CLOCK,
            PeriphLabel::I2c0,
            (gpiob.pb6, gpiob.pb7),
            I2cMode::standard(I2C_HZ),
        ) {
            Ok(b) => b,
            Err(_) => fail(4),
        };

        // Probe + init BEFORE the interrupt load starts (the integrated image's boot does the
        // same; the race under test is the cyclic burst, not the bring-up).
        let mut sensor = Imu::new(CLONE_2E, Config::default());
        if sensor.probe(&mut i2c).is_err() {
            fail(5);
        }
        if sensor.init(&mut i2c).is_err() {
            fail(7);
        }
        cortex_m::asm::delay((CLOCK.sysclk_hz / 1000) * 100); // post-init settle

        // Start the interrupt load: SysTick at IRQ_HZ with the burning handler.
        cp.SYST.set_clock_source(SystClkSource::Core);
        cp.SYST.set_reload(CLOCK.sysclk_hz / IRQ_HZ - 1);
        cp.SYST.clear_current();
        cp.SYST.enable_interrupt();
        cp.SYST.enable_counter();

        obs_store!(IRQLOAD_OBS, magic, MAGIC);

        // Back-to-back bursts, forever.
        let mut samples: u32 = 0;
        let mut errs: u32 = 0;
        let mut buf = [0u8; imu::BURST_LEN];
        loop {
            match i2c.write_read(imu::ADDR, &[0x3B], &mut buf) {
                Ok(()) => {
                    samples = samples.wrapping_add(1);
                    obs_store!(IRQLOAD_OBS, samples, samples);
                }
                Err(_) => {
                    if errs == 0 {
                        // First failure: snapshot the raw I2C registers (the wedge signature).
                        let rd = |off: u32| unsafe {
                            core::ptr::read_volatile((I2C0_BASE + off) as *const u32)
                        };
                        obs_store!(IRQLOAD_OBS, first_err_stat0, rd(0x14));
                        obs_store!(IRQLOAD_OBS, first_err_stat1, rd(0x18));
                        obs_store!(IRQLOAD_OBS, first_err_ctl0, rd(0x00));
                    }
                    errs = errs.wrapping_add(1);
                    obs_store!(IRQLOAD_OBS, errs, errs);
                }
            }
            obs_store!(IRQLOAD_OBS, ticks, TICKS.load(Ordering::Relaxed));
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
