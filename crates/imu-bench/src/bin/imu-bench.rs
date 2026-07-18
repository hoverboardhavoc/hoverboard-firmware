//! IMU + attitude on-silicon validator (`specs/imu.md` "Silicon validation" +
//! `specs/silicon-queue.md` section 2).
//!
//! Target: the bench F130 slave with the clone IMU (`imu::CLONE_2E`, WHO_AM_I `0x2E`) on hardware
//! **I2C0 PB6/PB7** (AF1 on the F1x0; the silicon-proven mux). The image, in order:
//!
//! 1. `detect_chip`, bring up the production 72 MHz tree (`ClockConfig::REFERENCE_72M_IRC8M`).
//! 2. Bring up I2C0 on PB6/PB7 at 400 kHz fast mode (the production rate, `specs/imu.md`; at
//!    72 MHz APB1 = 36 MHz, so I2CCLK = 0x24 / RT = 0x0B / CKCFG = 0x801E per `timing_for`).
//! 3. `Imu::probe` as `CLONE_2E` (records WHO_AM_I + probe verdict; an identity mismatch records
//!    the value the device actually returned).
//! 4. `Imu::init` (the 5-write config sequence, issued back-to-back per the spec's timing delta),
//!    then read the five config registers back and compare against `imu::CONFIG_WRITES` (the
//!    silicon-queue "assumed-standard config map end to end" check).
//! 5. Loop at 250 Hz (SysTick free-running 4 ms wrap pacing): 14-byte burst read, feed the Mahony
//!    attitude filter per `specs/attitude.md` "How it is driven" (gyro rad/s + sign-applied accel
//!    counts, one `update` per tick), publish everything to `IMU_BENCH_OBS`.
//! 6. Bias-cal capture (`specs/imu.md` "Silicon validation"): after a 50-sample warm-up, average
//!    the zero-bias gyro counts over 256 still samples, feed the captured bias back into the imu
//!    `Config` (the spec's feedback step), then average the next 256 post-bias samples as the
//!    resting-rate record (should be ~0). The still flag is recorded independently (first sample
//!    index where it asserted); the bench operator keeps the board still from boot.
//!
//! Read the result over SWD: `nm` the `IMU_BENCH_OBS` symbol, dump the struct (field offsets are
//! documented on [`firmware::Obs`]). Failure states busy-spin with a distinct `err` code (and the
//! magic set, so the reader can trust the block), NEVER `wfi` (a bare `wfi` with `DBG_CTL0 = 0`
//! locks GD32 SWD re-attach).
//!
//! No motor-adjacent pin is touched: the image configures only PB6/PB7 (I2C0), never PA9/PA10 or
//! any FET gate pin.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m::peripheral::SYST;
    use cortex_m_rt::entry;
    use panic_halt as _;

    use attitude::Mahony;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::I2c as _;
    use imu::{Config, Error, Fix, Imu, CLONE_2E, CONFIG_WRITES};
    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::{detect_chip, Delay, I2c, I2cMode, PeriphLabel};
    use test_shared::obs_store;

    // --- parameters -----------------------------------------------------------------------------

    /// The production 72 MHz tree (IRC8M -> PLL), so the bench validates the IMU in the shipping
    /// clock regime (I2C timing computed from APB1 = 36 MHz, SysTick pacing from 72 MHz).
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    /// I2C bus rate: 400 kHz fast mode (`specs/imu.md`, "Hardware I2C first": the production
    /// rate; clone-validated here 2026-07-18, sustained 251/s bursts with zero errors).
    const I2C_HZ: u32 = 400_000;
    /// The sample/tick rate: the spec'd 250 Hz control tick.
    const TICK_HZ: u32 = 250;
    /// Post-`init` settling pause before the first burst read (the caller-owned pause the spec
    /// names; the bench probe needed no more than tens of ms, 100 ms is comfortable).
    const SETTLE_MS: u32 = 100;
    /// Warm-up samples discarded before the bias capture starts (first reads after wake).
    const WARMUP_SAMPLES: u32 = 50;
    /// Bias-capture window length (power of two so the average is a shift): ~1 s at 250 Hz.
    const CAPTURE_SAMPLES: u32 = 256;
    /// Post-bias resting-rate window length (same size, directly comparable to the capture).
    const POST_SAMPLES: u32 = 256;

    // --- the SWD-readable result block ----------------------------------------------------------

    /// The one SWD-readable observation block. `#[repr(C)]`, every field offset from the
    /// `IMU_BENCH_OBS` symbol base pinned by the compile-time asserts below.
    ///
    /// | offset | field | meaning |
    /// |---|---|---|
    /// | `0x00` | `magic` | `0x494D_5542` ("IMUB") once the block is live (set after bring-up, or together with a fatal `err`) |
    /// | `0x04` | `err` | 0 = ok; nonzero = fatal bring-up error, busy-spinning (codes on the field) |
    /// | `0x05` | `probe_ok` | 1 = `Imu::probe` passed as `CLONE_2E` |
    /// | `0x06` | `who_am_i` | WHO_AM_I readback (expected `0x2E`; on an identity mismatch, the value the device returned) |
    /// | `0x07` | `readback_ok` | 1 = all five config registers read back the written values |
    /// | `0x08` | `readback[5]` | config-register readback bytes, in `CONFIG_WRITES` order (PWR_MGMT_1, SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG) |
    /// | `0x0D` | `phase` | 0 = warm-up, 1 = bias capture, 2 = post-bias window, 3 = steady state |
    /// | `0x0E` | `still` | latest `Sample.still` flag (1 = asserted) |
    /// | `0x0F` | `bias_applied` | 1 = the captured gyro bias has been fed back into the imu `Config` |
    /// | `0x10` | `samples` | running burst-read sample counter (increments per successful 250 Hz read) |
    /// | `0x14` | `read_errs` | bus errors during the cyclic read (sample skipped, loop continues) |
    /// | `0x18` | `accel_raw[3]` | latest sign-applied accel counts, `i16` x/y/z (`Sample.accel_raw`) |
    /// | `0x1E` | `gyro_raw[3]` | latest bias-corrected gyro counts, `i16` x/y/z (`Sample.gyro_raw`) |
    /// | `0x24` | `pitch_q16` | latest Mahony pitch, degrees as I16F16 raw bits (`value = pitch_q16 / 65536.0`) |
    /// | `0x28` | `roll_q16` | latest Mahony roll, degrees as I16F16 raw bits (`value = roll_q16 / 65536.0`) |
    /// | `0x2C` | `still_count` | latest `Imu::still_count` (consecutive-identical counter, asserts > 50) |
    /// | `0x2E` | `temp_centi` | latest temperature, centidegrees C |
    /// | `0x30` | `still_first` | sample index (1-based) where `still` FIRST asserted; 0 = never yet |
    /// | `0x34` | `gyro_bias[3]` | captured zero-rate gyro bias, `i32` x/y/z counts (valid once `bias_applied` = 1) |
    /// | `0x40` | `post_rest[3]` | post-bias resting rates: mean bias-corrected gyro counts over the post window, `i32` x/y/z (valid once `phase` >= 3; expect ~0) |
    ///
    /// Total size `0x4C` (76) bytes.
    #[repr(C)]
    pub struct Obs {
        /// `0x494D_5542` ("IMUB").
        magic: u32,
        /// Fatal bring-up error, busy-spinning: 0 ok, 1 detect_chip failed, 2 clock tree failed,
        /// 3 GPIOB split failed, 4 I2C bring-up failed, 5 probe bus error, 6 probe identity
        /// mismatch (`who_am_i` holds the returned value), 7 init write failed, 8 config readback
        /// bus error.
        err: u8,
        /// `Imu::probe` passed as `CLONE_2E`.
        probe_ok: u8,
        /// WHO_AM_I readback.
        who_am_i: u8,
        /// All five config registers read back the written values.
        readback_ok: u8,
        /// Config-register readback bytes, `CONFIG_WRITES` order.
        readback: [u8; 5],
        /// 0 warm-up, 1 capture, 2 post-bias window, 3 steady.
        phase: u8,
        /// Latest still flag.
        still: u8,
        /// Captured bias fed back into the imu `Config`.
        bias_applied: u8,
        /// Successful burst reads.
        samples: u32,
        /// Bus errors during the cyclic read.
        read_errs: u32,
        /// Latest sign-applied accel counts.
        accel_raw: [i16; 3],
        /// Latest bias-corrected gyro counts.
        gyro_raw: [i16; 3],
        /// Latest pitch, degrees, I16F16 raw bits.
        pitch_q16: i32,
        /// Latest roll, degrees, I16F16 raw bits.
        roll_q16: i32,
        /// Latest consecutive-identical still counter.
        still_count: u16,
        /// Latest temperature, centidegrees C.
        temp_centi: i16,
        /// 1-based sample index of the first still assertion; 0 = never.
        still_first: u32,
        /// Captured zero-rate gyro bias, counts.
        gyro_bias: [i32; 3],
        /// Post-bias mean resting rates, counts.
        post_rest: [i32; 3],
    }

    // Pin every offset the bench reader depends on (the doc table above).
    const _: () = {
        assert!(core::mem::offset_of!(Obs, magic) == 0x00);
        assert!(core::mem::offset_of!(Obs, err) == 0x04);
        assert!(core::mem::offset_of!(Obs, probe_ok) == 0x05);
        assert!(core::mem::offset_of!(Obs, who_am_i) == 0x06);
        assert!(core::mem::offset_of!(Obs, readback_ok) == 0x07);
        assert!(core::mem::offset_of!(Obs, readback) == 0x08);
        assert!(core::mem::offset_of!(Obs, phase) == 0x0D);
        assert!(core::mem::offset_of!(Obs, still) == 0x0E);
        assert!(core::mem::offset_of!(Obs, bias_applied) == 0x0F);
        assert!(core::mem::offset_of!(Obs, samples) == 0x10);
        assert!(core::mem::offset_of!(Obs, read_errs) == 0x14);
        assert!(core::mem::offset_of!(Obs, accel_raw) == 0x18);
        assert!(core::mem::offset_of!(Obs, gyro_raw) == 0x1E);
        assert!(core::mem::offset_of!(Obs, pitch_q16) == 0x24);
        assert!(core::mem::offset_of!(Obs, roll_q16) == 0x28);
        assert!(core::mem::offset_of!(Obs, still_count) == 0x2C);
        assert!(core::mem::offset_of!(Obs, temp_centi) == 0x2E);
        assert!(core::mem::offset_of!(Obs, still_first) == 0x30);
        assert!(core::mem::offset_of!(Obs, gyro_bias) == 0x34);
        assert!(core::mem::offset_of!(Obs, post_rest) == 0x40);
        assert!(core::mem::size_of::<Obs>() == 0x4C);
    };

    const MAGIC: u32 = 0x494D_5542; // "IMUB"

    /// The single SWD-readable result instance (read by `nm IMU_BENCH_OBS`).
    #[no_mangle]
    static mut IMU_BENCH_OBS: Obs = Obs {
        magic: 0,
        err: 0,
        probe_ok: 0,
        who_am_i: 0,
        readback_ok: 0,
        readback: [0; 5],
        phase: 0,
        still: 0,
        bias_applied: 0,
        samples: 0,
        read_errs: 0,
        accel_raw: [0; 3],
        gyro_raw: [0; 3],
        pitch_q16: 0,
        roll_q16: 0,
        still_count: 0,
        temp_centi: 0,
        still_first: 0,
        gyro_bias: [0; 3],
        post_rest: [0; 3],
    };

    // --- entry ----------------------------------------------------------------------------------

    #[entry]
    fn main() -> ! {
        // Claim SysTick for the settling delay + tick pacing (the application owns the one
        // Peripherals::take(), same as ble-loopback).
        let cp = cortex_m::Peripherals::take().unwrap();

        let chip = match detect_chip() {
            Ok(c) => c,
            Err(_) => fail(1),
        };

        // The production 72 MHz tree, before any peripheral bring-up (I2C timing and SysTick
        // pacing are both derived from it).
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            fail(2);
        }

        // GPIOB carries the IMU bus pins PB6 (SCL) / PB7 (SDA). Split once to configure them.
        let gpiob = match chip.gpiob() {
            Ok(p) => p.split(),
            Err(_) => fail(3),
        };

        // Hardware I2C0 on PB6/PB7 (AF1 on the F1x0, the silicon-proven mux), 400 kHz fast
        // mode. `I2c::new` consumes the named pins, enables the peripheral clock, configures the
        // pins AF open-drain, and programs the SPL-faithful timing from APB1 = 36 MHz.
        let mut i2c = match I2c::new(
            &chip,
            &CLOCK,
            PeriphLabel::I2c0,
            (gpiob.pb6, gpiob.pb7),
            I2cMode::fast(I2C_HZ, runtime_hal::i2c::FastDuty::Two),
        ) {
            Ok(b) => b,
            Err(_) => fail(4),
        };

        // The driver under test: the CLONE_2E model with the reference Config (stock sign map,
        // zero gyro bias; the bias is captured below and fed back).
        let mut sensor = Imu::new(CLONE_2E, Config::default());

        // Probe: WHO_AM_I identity check as CLONE_2E. Record the readback either way (on a
        // mismatch the Identity error carries what the device returned).
        match sensor.probe(&mut i2c) {
            Ok(()) => {
                obs_store!(IMU_BENCH_OBS, who_am_i, CLONE_2E.who_am_i);
                obs_store!(IMU_BENCH_OBS, probe_ok, 1);
            }
            Err(Error::Identity { got, .. }) => {
                obs_store!(IMU_BENCH_OBS, who_am_i, got);
                fail(6);
            }
            Err(Error::Bus(_)) => fail(5),
        }

        // Init: the 5-write config sequence, back-to-back (the spec's timing delta under test).
        if sensor.init(&mut i2c).is_err() {
            fail(7);
        }

        // Readback: each config register individually, compared against the written value (the
        // silicon-queue "config map end to end" check). Registers come from CONFIG_WRITES, the
        // byte contract's one owner.
        let mut readback = [0u8; 5];
        let mut readback_ok = true;
        for (i, (reg, val)) in CONFIG_WRITES.iter().enumerate() {
            let mut b = [0u8; 1];
            if i2c.write_read(imu::ADDR, &[*reg], &mut b).is_err() {
                fail(8);
            }
            readback[i] = b[0];
            if b[0] != *val {
                readback_ok = false;
            }
        }
        obs_store!(IMU_BENCH_OBS, readback, readback);
        obs_store!(IMU_BENCH_OBS, readback_ok, readback_ok as u8);

        // Post-init settling pause (caller-owned per the spec), then the block is live.
        let mut delay = Delay::new(cp.SYST, CLOCK.sysclk_hz);
        delay.delay_ms(SETTLE_MS);
        obs_store!(IMU_BENCH_OBS, magic, MAGIC);

        // Repurpose SysTick as the free-running 250 Hz pacing timer: reload = sysclk/250 - 1
        // (287_999 at 72 MHz, fits the 24-bit field), poll COUNTFLAG per tick. No interrupt.
        let mut syst = delay.free();
        syst.set_reload(CLOCK.sysclk_hz / TICK_HZ - 1);
        syst.clear_current();
        syst.enable_counter();

        run(&mut i2c, &mut sensor, &mut syst);
    }

    /// The 250 Hz loop: burst read -> attitude update -> OBS publish, with the bias-cal capture
    /// state machine (warm-up -> capture -> feed back -> post window -> steady).
    fn run(i2c: &mut I2c, sensor: &mut Imu, syst: &mut SYST) -> ! {
        // The Mahony filter, reference attitude config (Kp = 1.0, +1 signs: the imu front-end
        // already applied the board sign map), driven once per tick per specs/attitude.md.
        let mut mahony = Mahony::new(attitude::Config::default());

        let mut samples: u32 = 0;
        let mut read_errs: u32 = 0;
        let mut still_first: u32 = 0;
        // Bias capture accumulators (zero-bias gyro counts), then the post-bias window's.
        let mut cap_sum = [0i32; 3];
        let mut post_sum = [0i32; 3];

        loop {
            // 4 ms tick edge (COUNTFLAG read-clears; a long body just slips one period).
            while !syst.has_wrapped() {
                nop();
            }

            let sample = match sensor.read(i2c) {
                Ok(s) => s,
                Err(_) => {
                    read_errs = read_errs.wrapping_add(1);
                    obs_store!(IMU_BENCH_OBS, read_errs, read_errs);
                    continue; // skip the tick; stale-buffer policy is not this bench's concern
                }
            };
            samples += 1;

            // Attitude wiring per specs/attitude.md "How it is driven": gyro in rad/s (Fix),
            // accel as sign-applied counts (Fix, direction-only), one update per tick.
            let accel = [
                Fix::from_num(sample.accel_raw[0]),
                Fix::from_num(sample.accel_raw[1]),
                Fix::from_num(sample.accel_raw[2]),
            ];
            let out = mahony.update(sample.gyro, accel);

            // Publish the tick.
            obs_store!(IMU_BENCH_OBS, samples, samples);
            obs_store!(IMU_BENCH_OBS, accel_raw, sample.accel_raw);
            obs_store!(IMU_BENCH_OBS, gyro_raw, sample.gyro_raw);
            obs_store!(IMU_BENCH_OBS, pitch_q16, out.pitch_deg.to_bits());
            obs_store!(IMU_BENCH_OBS, roll_q16, out.roll_deg.to_bits());
            obs_store!(IMU_BENCH_OBS, temp_centi, sample.temp_centi_degc as i16);
            obs_store!(IMU_BENCH_OBS, still, sample.still as u8);
            obs_store!(IMU_BENCH_OBS, still_count, sensor.still_count());
            if sample.still && still_first == 0 {
                still_first = samples;
                obs_store!(IMU_BENCH_OBS, still_first, still_first);
            }

            // Bias-cal state machine, by sample index (the operator keeps the board still from
            // boot; the still flag is recorded independently above).
            if samples == WARMUP_SAMPLES {
                obs_store!(IMU_BENCH_OBS, phase, 1);
            } else if samples > WARMUP_SAMPLES && samples <= WARMUP_SAMPLES + CAPTURE_SAMPLES {
                // Capture window: accumulate the zero-bias gyro counts.
                for (acc, &g) in cap_sum.iter_mut().zip(sample.gyro_raw.iter()) {
                    *acc += g as i32;
                }
                if samples == WARMUP_SAMPLES + CAPTURE_SAMPLES {
                    // Mean zero-rate counts = the captured bias; feed it back into the Config
                    // (the spec's feedback step). Truncating division: sub-count precision is
                    // below the sensor noise floor.
                    let bias = [
                        cap_sum[0] / CAPTURE_SAMPLES as i32,
                        cap_sum[1] / CAPTURE_SAMPLES as i32,
                        cap_sum[2] / CAPTURE_SAMPLES as i32,
                    ];
                    sensor.set_config(Config {
                        gyro_bias: bias,
                        ..Config::default()
                    });
                    obs_store!(IMU_BENCH_OBS, gyro_bias, bias);
                    obs_store!(IMU_BENCH_OBS, bias_applied, 1);
                    obs_store!(IMU_BENCH_OBS, phase, 2);
                }
            } else if samples > WARMUP_SAMPLES + CAPTURE_SAMPLES
                && samples <= WARMUP_SAMPLES + CAPTURE_SAMPLES + POST_SAMPLES
            {
                // Post-bias window: the now-bias-corrected counts; their mean is the resting rate.
                for (acc, &g) in post_sum.iter_mut().zip(sample.gyro_raw.iter()) {
                    *acc += g as i32;
                }
                if samples == WARMUP_SAMPLES + CAPTURE_SAMPLES + POST_SAMPLES {
                    let rest = [
                        post_sum[0] / POST_SAMPLES as i32,
                        post_sum[1] / POST_SAMPLES as i32,
                        post_sum[2] / POST_SAMPLES as i32,
                    ];
                    obs_store!(IMU_BENCH_OBS, post_rest, rest);
                    obs_store!(IMU_BENCH_OBS, phase, 3);
                }
            }
        }
    }

    /// Record a fatal bring-up error (with the magic, so the reader trusts the block) and
    /// busy-spin forever. NEVER `wfi` (GD32 SWD-lockout rule).
    fn fail(code: u8) -> ! {
        obs_store!(IMU_BENCH_OBS, err, code);
        obs_store!(IMU_BENCH_OBS, magic, MAGIC);
        loop {
            nop();
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
