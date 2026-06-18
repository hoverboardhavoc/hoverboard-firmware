//! MPU-6050-class IMU front-end.
//!
//! Owns the device contract for a single MPU-6050-class 6-axis sensor at 7-bit I2C address `0x68`:
//! the boot configuration writes, the cyclic 14-byte burst read at 250 Hz, the per-axis sign map,
//! gyro-bias subtraction (no accel bias), full-scale scaling into engineering units, and the IIR
//! pre-filter coefficients. It produces, once per control tick, a calibrated gyro rate vector
//! (rad/s), a sign-applied acceleration vector (direction-only counts), a temperature reading, and
//! the bias-corrected raw words the attitude filter ([`crates/attitude`]) consumes in the same tick.
//!
//! Uses the reference constants from the reference design; every numeric
//! constant is preserved verbatim from `todo/imu.md` (the normative spec). See `spec/core.md`
//! ("Math: fixed-point Q", the no-FPU constraint) for the Q-format basis.
//!
//! Transport: the driver is generic over the `embedded-hal` 1.0 `i2c::I2c` trait, so it works with
//! either runtime-hal's hardware-I2C peripheral or a bit-banged software-I2C shim exposing the same
//! trait (spec section 2, options A and B). The per-edge waveform and concrete pins/bus are
//! firmware-wiring concerns resolved against the `McuDescriptor`, not this crate's.
//!
//! No-FPU adaptation: the scaling and bias math run in fixed-point Q. The gyro rad/s output is
//! `I32F32`, matching the attitude filter's body Q so the sample feeds it without a reconversion.
//!
//! `no_std`; host tests in `#[cfg(test)]` link `std` via the host target and mock the I2C bus.

#![no_std]

use embedded_hal::i2c::I2c;
use fixed::types::I32F32;

// ---------------------------------------------------------------------------------------------
// Device constants (FIXED, identical across all boards). Spec sections 1, 4, 5, 6, 12.
// ---------------------------------------------------------------------------------------------

/// 7-bit I2C device address (spec section 1). `embedded-hal`'s `I2c` takes the 7-bit address and
/// forms the read/write framing itself, so the `(0x68 << 1)` / `(0x68 << 1) | 1` bus bytes from
/// spec section 4 are produced by the bus layer, not here.
pub const ADDR: u8 = 0x68;

// Configuration registers (spec section 5).
const REG_SMPLRT_DIV: u8 = 0x19;
const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_PWR_MGMT_1: u8 = 0x6B;

/// First data register of the 14-byte sensor block, ACCEL_XOUT_H (spec section 6.1).
const REG_ACCEL_XOUT_H: u8 = 0x3B;

/// Length of the cyclic burst read (spec section 6.1).
pub const BURST_LEN: usize = 14;

/// The boot configuration writes, in order, as `(register, value)` pairs. FIXED constants, written
/// once at boot (spec section 5):
///
/// | Register | Name | Value | Effect |
/// |---|---|---|---|
/// | `0x6B` | PWR_MGMT_1   | `0x00` | wake device, internal 8 MHz oscillator |
/// | `0x19` | SMPLRT_DIV   | `0x00` | sample-rate divider = 0 |
/// | `0x1A` | CONFIG       | `0x00` | DLPF setting 0 |
/// | `0x1B` | GYRO_CONFIG  | `0x08` | gyro full scale = +-500 deg/s |
/// | `0x1C` | ACCEL_CONFIG | `0x08` | accel full scale = +-4 g |
///
/// PWR_MGMT_1 is first (the wake step); the device needs a settling delay after it. The exact delay
/// is a firmware-timing concern (the caller may pause between writes); the byte contract is here.
pub const CONFIG_WRITES: [(u8, u8); 5] = [
    (REG_PWR_MGMT_1, 0x00),
    (REG_SMPLRT_DIV, 0x00),
    (REG_CONFIG, 0x00),
    (REG_GYRO_CONFIG, 0x08),
    (REG_ACCEL_CONFIG, 0x08),
];

// ---------------------------------------------------------------------------------------------
// Scaling constants. Spec sections 5, 7. Stated as exact reals so the Q form matches.
// ---------------------------------------------------------------------------------------------

/// Gyro count -> rad/s scale: `(500/32768) * (pi/180) = 0.000266316114` (spec section 5 / 7.2,
/// source float `0x398BA058`). This encodes the exact 65.536 LSB/(deg/s) full-scale count math, not
/// the datasheet-rounded 65.5 (which drifts ~0.055%). Applied to the bias-corrected gyro count.
/// Identical to the attitude filter's `GYRO_SCALE`, so the rate feeds it directly.
pub const GYRO_SCALE: f64 = 0.000_266_316_114;

/// Accel full-scale sensitivity at +-4 g: `8192 LSB/g` (spec section 7.3). The accel vector is
/// normalized to a unit gravity direction downstream, so this absolute scale affects only
/// intermediate values; the single shared scale across the three axes is what must be exact. The
/// attitude filter consumes sign-applied counts directly (direction only), so `read` publishes the
/// sign-applied counts; [`Sample::accel_g`] applies this divisor for callers that want g units.
pub const ACCEL_LSB_PER_G: i32 = 8192;

/// Temperature transfer offset and divisor: `temp_centi_degC = (raw + 12420) / 340` (spec section
/// 7.5), the integer form of the datasheet `T(degC) = raw/340 + 36.53` expressed in centidegrees.
pub const TEMP_OFFSET: i32 = 12420;
pub const TEMP_DIV: i32 = 340;

/// Symmetric saturation guard for the bias-corrected stored words: `[-32767, +32767]` (spec section
/// 7.4 uses a symmetric guard rather than the full `-32768`).
pub const CLAMP_MAX: i32 = 32767;
pub const CLAMP_MIN: i32 = -32767;

// ---------------------------------------------------------------------------------------------
// IIR pre-filter coefficients (spec section 10). Single-pole `new = a*sample + (1-a)*prev`.
// Preserved verbatim; placement (which axis/signal) is owned by the attitude spec, so these are
// provided as a reusable filter the front-end can apply per the assembly wiring.
// ---------------------------------------------------------------------------------------------

/// Fast pre-filter pair `0.02 / 0.98` (spec section 10).
pub const IIR_FAST_NEW: f64 = 0.02;
pub const IIR_FAST_OLD: f64 = 0.98;
/// Slow pre-filter pair `0.01 / 0.99` (spec section 10).
pub const IIR_SLOW_NEW: f64 = 0.01;
pub const IIR_SLOW_OLD: f64 = 0.99;

/// Still-detection (spec section 9): counter cap and the threshold above which the still flag
/// asserts.
pub const STILL_COUNTER_CAP: u16 = 0xFFFE;
pub const STILL_THRESHOLD: u16 = 50;

// ---------------------------------------------------------------------------------------------
// Q-format body type. Matches the attitude filter's `Fix` (I32F32): 32 fractional bits hold the
// gyro scale 0.000266316114 to ~4e-7 relative error, 31 integer bits hold any bias-corrected
// count * scale (<< 1) without overflow.
// ---------------------------------------------------------------------------------------------

/// Scaling Q type. Identical to `attitude::Fix`.
pub type Fix = I32F32;

// ---------------------------------------------------------------------------------------------
// Configuration / board data (per-board, from the board definition or calibration page). Spec
// section 7.1, 7.2, 12. Defaults are the reference values.
// ---------------------------------------------------------------------------------------------

/// Per-axis sign map and gyro-bias offsets: the calibration contract this front-end shares with the
/// attitude filter (spec sections 7.1, 7.2, 12). Board/config data, not fixed firmware constants:
/// a differently mounted sensor or a different unit carries a different map / different biases.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Per-axis sign map `(AX, AY, AZ, GX, GY, GZ)`. **Load-bearing for attitude convergence**: it
    /// must match what the attitude filter expects, or the complementary filter's accel error
    /// feedback pushes the gyro integration the wrong way and the estimate diverges (spec 7.1).
    /// Order: `[ax, ay, az, gx, gy, gz]`.
    pub sign: [i32; 6],
    /// Per-axis zero-rate gyro bias offsets, in raw counts, subtracted after the sign is applied
    /// (`corrected = sign*raw - bias`). Per-board, nonzero, from cal-page idx3/4/5
    /// (`flash_config` `hw[3..5]`). Order: `[gx, gy, gz]` (spec 7.2). No accel bias exists (7.3).
    pub gyro_bias: [i32; 3],
}

impl Default for Config {
    /// The reference defaults. Sign map `(-1, +1, -1, -1, +1, -1)`: a 180-degree
    /// rotation about Y (determinant +1) applied to both accel and gyro (spec section 7.1). The
    /// gyro bias defaults to zero here; the real per-board nonzero offsets come from the cal page
    /// and are not a fixed firmware constant, so the in-code default is the neutral 0.
    fn default() -> Self {
        Config {
            sign: [-1, 1, -1, -1, 1, -1],
            gyro_bias: [0, 0, 0],
        }
    }
}

// ---------------------------------------------------------------------------------------------
// IIR pre-filter (spec section 10). One single-pole channel: `y <- new_w * x + old_w * y`.
// ---------------------------------------------------------------------------------------------

/// First-order IIR pre-filter for one signal (spec section 10). `y <- new_w * x + old_w * y`; the
/// first sample primes the state to itself so there is no startup ramp from zero.
#[derive(Clone, Copy, Debug)]
pub struct Iir {
    new_w: Fix,
    old_w: Fix,
    y: Fix,
    primed: bool,
}

impl Iir {
    /// Build a pre-filter from its two coefficients (e.g. [`IIR_FAST_NEW`] / [`IIR_FAST_OLD`]).
    pub fn new(new_w: f64, old_w: f64) -> Self {
        Iir {
            new_w: Fix::from_num(new_w),
            old_w: Fix::from_num(old_w),
            y: Fix::ZERO,
            primed: false,
        }
    }

    /// The fast `0.02 / 0.98` pre-filter (spec section 10).
    pub fn fast() -> Self {
        Iir::new(IIR_FAST_NEW, IIR_FAST_OLD)
    }

    /// The slow `0.01 / 0.99` pre-filter (spec section 10).
    pub fn slow() -> Self {
        Iir::new(IIR_SLOW_NEW, IIR_SLOW_OLD)
    }

    /// Push a sample and return the smoothed output. The first call primes `y` to the sample.
    pub fn step(&mut self, x: Fix) -> Fix {
        if !self.primed {
            self.y = x;
            self.primed = true;
        } else {
            self.y = self.new_w * x + self.old_w * self.y;
        }
        self.y
    }

    /// Current filter output without pushing a new sample.
    pub fn value(&self) -> Fix {
        self.y
    }
}

// ---------------------------------------------------------------------------------------------
// Sample: the published per-tick output (spec section 8).
// ---------------------------------------------------------------------------------------------

/// One calibrated sample, published per control tick (spec section 8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Calibrated gyro rate, 3 axes, rad/s (sign + bias applied, scaled by [`GYRO_SCALE`]). Feeds
    /// the attitude filter directly. Order `[x, y, z]`.
    pub gyro: [Fix; 3],
    /// Bias-corrected, sign-applied, clamped gyro counts retained for the attitude filter and the
    /// still-detection (spec section 8). Order `[x, y, z]`.
    pub gyro_raw: [i16; 3],
    /// Sign-applied, clamped acceleration counts (no bias). Direction-only; the attitude filter
    /// normalizes to a unit gravity vector. Order `[x, y, z]`.
    pub accel_raw: [i16; 3],
    /// Temperature in centidegrees Celsius (spec section 7.5). Telemetry only.
    pub temp_centi_degc: i32,
    /// Still-detection flag: asserted when more than [`STILL_THRESHOLD`] consecutive samples were
    /// bit-exactly identical (spec section 9).
    pub still: bool,
}

impl Sample {
    /// Acceleration in g per axis (sign-applied counts / [`ACCEL_LSB_PER_G`]). The control path uses
    /// the direction-only counts in [`Sample::accel_raw`]; this is offered for callers wanting g.
    pub fn accel_g(&self) -> [Fix; 3] {
        let div = Fix::from_num(ACCEL_LSB_PER_G);
        [
            Fix::from_num(self.accel_raw[0]) / div,
            Fix::from_num(self.accel_raw[1]) / div,
            Fix::from_num(self.accel_raw[2]) / div,
        ]
    }
}

// ---------------------------------------------------------------------------------------------
// The driver.
// ---------------------------------------------------------------------------------------------

/// Error from a bus transaction. Wraps the underlying `embedded-hal` I2C error.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Error<E> {
    /// A configuration write or burst read failed on the bus.
    Bus(E),
}

/// Reassemble a big-endian signed 16-bit word from two bytes (spec section 6.1).
#[inline]
fn be_i16(hi: u8, lo: u8) -> i16 {
    (((hi as u16) << 8) | (lo as u16)) as i16
}

/// Apply the sign then clamp to the symmetric `[-32767, +32767]` guard (spec section 7.4).
#[inline]
fn sign_clamp(sign: i32, raw: i16) -> i16 {
    let v = sign * (raw as i32);
    v.clamp(CLAMP_MIN, CLAMP_MAX) as i16
}

/// MPU-6050-class IMU front-end. Generic over the `embedded-hal` 1.0 I2C bus; holds the per-board
/// calibration config, the still-detection state, and the one-tick corrected-sample history.
pub struct Mpu6050 {
    cfg: Config,
    gyro_scale: Fix,
    /// Previous tick's six corrected words `[ax, ay, az, gx, gy, gz]`, for still-detection (spec 9).
    prev_words: Option<[i16; 6]>,
    /// Consecutive-identical-sample counter (spec section 9, saturates at [`STILL_COUNTER_CAP`]).
    still_count: u16,
}

impl Default for Mpu6050 {
    fn default() -> Self {
        Mpu6050::new(Config::default())
    }
}

impl Mpu6050 {
    /// Build the front-end with a per-board calibration config (sign map + gyro bias).
    pub fn new(cfg: Config) -> Self {
        Mpu6050 {
            cfg,
            gyro_scale: Fix::from_num(GYRO_SCALE),
            prev_words: None,
            still_count: 0,
        }
    }

    /// Replace the calibration config (e.g. after loading the cal page at boot).
    pub fn set_config(&mut self, cfg: Config) {
        self.cfg = cfg;
    }

    /// The active config.
    pub fn config(&self) -> &Config {
        &self.cfg
    }

    /// Boot configuration: write the FIXED register map (spec section 5) once, in order. Each write
    /// is a single-byte register write `[register, value]`. The PWR_MGMT_1 wake is first; the
    /// caller inserts the device settling delay (a firmware-timing concern) before the cyclic reads.
    pub fn init<I, E>(&mut self, i2c: &mut I) -> Result<(), Error<E>>
    where
        I: I2c<Error = E>,
    {
        for (reg, val) in CONFIG_WRITES {
            i2c.write(ADDR, &[reg, val]).map_err(Error::Bus)?;
        }
        Ok(())
    }

    /// Cyclic burst read (per 250 Hz tick): read 14 bytes from `0x3B`, decode the six big-endian
    /// signed words plus temperature, apply the per-axis sign, subtract the gyro bias (no accel
    /// bias), clamp, scale the gyro to rad/s, run still-detection, and return the [`Sample`].
    ///
    /// Spec section 6.3 (stale-data): a bus failure is surfaced as [`Error`]. The real control
    /// orchestrator may ignore it and reprocess the stale buffer; that policy is the caller's, so
    /// this returns the error rather than silently reusing the previous bytes.
    pub fn read<I, E>(&mut self, i2c: &mut I) -> Result<Sample, Error<E>>
    where
        I: I2c<Error = E>,
    {
        let mut buf = [0u8; BURST_LEN];
        i2c.write_read(ADDR, &[REG_ACCEL_XOUT_H], &mut buf)
            .map_err(Error::Bus)?;
        Ok(self.decode(&buf))
    }

    /// Decode a 14-byte burst buffer into a [`Sample`] (the pure math half of [`Self::read`]).
    /// Exposed so callers running the spec-6.3 stale-data path can reprocess a retained buffer, and
    /// so the host tests can hand-compute against scripted bytes.
    pub fn decode(&mut self, buf: &[u8; BURST_LEN]) -> Sample {
        // Big-endian signed words (spec section 6.1).
        let ax = be_i16(buf[0], buf[1]);
        let ay = be_i16(buf[2], buf[3]);
        let az = be_i16(buf[4], buf[5]);
        let temp = be_i16(buf[6], buf[7]);
        let gx = be_i16(buf[8], buf[9]);
        let gy = be_i16(buf[10], buf[11]);
        let gz = be_i16(buf[12], buf[13]);

        // Accel: sign then clamp, no bias (spec section 7.3).
        let acc = [
            sign_clamp(self.cfg.sign[0], ax),
            sign_clamp(self.cfg.sign[1], ay),
            sign_clamp(self.cfg.sign[2], az),
        ];

        // Gyro: sign, then subtract bias, then clamp (spec section 7.2).
        let raw_g = [gx, gy, gz];
        let mut gyro_raw = [0i16; 3];
        for i in 0..3 {
            let signed = self.cfg.sign[3 + i] * (raw_g[i] as i32);
            let corrected = signed - self.cfg.gyro_bias[i];
            gyro_raw[i] = corrected.clamp(CLAMP_MIN, CLAMP_MAX) as i16;
        }

        // Gyro scale to rad/s (spec section 7.2): bias-corrected count * 0.000266316114.
        let gyro = [
            Fix::from_num(gyro_raw[0]) * self.gyro_scale,
            Fix::from_num(gyro_raw[1]) * self.gyro_scale,
            Fix::from_num(gyro_raw[2]) * self.gyro_scale,
        ];

        // Temperature (spec section 7.5): centidegrees.
        let temp_centi_degc = ((temp as i32) + TEMP_OFFSET) / TEMP_DIV;

        // Still-detection (spec section 9): bit-exact equality of all six corrected words against
        // the previous tick. No per-axis magnitude windows.
        let words = [acc[0], acc[1], acc[2], gyro_raw[0], gyro_raw[1], gyro_raw[2]];
        match self.prev_words {
            Some(prev) if prev == words => {
                if self.still_count < STILL_COUNTER_CAP {
                    self.still_count += 1;
                }
            }
            _ => self.still_count = 0,
        }
        self.prev_words = Some(words);
        let still = self.still_count > STILL_THRESHOLD;

        Sample {
            gyro,
            gyro_raw,
            accel_raw: acc,
            temp_centi_degc,
            still,
        }
    }

    /// Current still-detection counter (spec section 9). Exposed for telemetry / tests.
    pub fn still_count(&self) -> u16 {
        self.still_count
    }
}

#[cfg(test)]
mod tests;
