// SPDX-License-Identifier: GPL-3.0-or-later
//! MPU-6050-class IMU front-end (`specs/imu.md`).
//!
//! Owns the device contract for a single MPU-6050-class 6-axis sensor at 7-bit I2C address `0x68`:
//! the boot configuration writes, the cyclic 14-byte burst read at 250 Hz, the per-axis sign map,
//! gyro-bias subtraction (no accel bias), full-scale scaling into engineering units, and the IIR
//! pre-filter. It produces, once per control tick, a calibrated gyro rate vector (rad/s), a
//! sign-applied acceleration vector (direction-only counts), a temperature reading, and the
//! bias-corrected raw words the attitude filter (`crates/attitude`) consumes in the same tick.
//!
//! Every numeric constant is preserved exactly from the recovered stock behavioral contract (the
//! spec's source 1); which IMU model is fitted is a [`Model`] descriptor (the spec's model table).
//!
//! Transport: the driver is generic over the `embedded-hal` 1.0 `i2c::I2c` trait per call, so it
//! works with either runtime-hal's hardware-I2C peripheral (the silicon-proven default path) or a
//! future bit-banged software-I2C master exposing the same trait (a board-model variant, not
//! built). The concrete pins/bus instance are firmware wiring, not this crate's.
//!
//! No-FPU adaptation: the scaling and bias math run in fixed-point Q. The gyro rad/s output is
//! [`Fix`] (`base::Fix`, I32F32), matching the attitude filter's body Q so the sample feeds it
//! without a reconversion.
//!
//! `no_std`; host tests in `#[cfg(test)]` link `std` via the host target and mock the I2C bus.

#![no_std]

use base::lane::Lane;
use embedded_hal::i2c::I2c;

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

/// Device identity register (spec "Models"): [`Imu::probe`] reads it and compares against the
/// model's expected value.
const REG_WHO_AM_I: u8 = 0x75;

/// One supported IMU model (`specs/imu.md`, "Models"). A descriptor carries only what differs
/// between supported models; everything the entries share (address, config sequence, burst layout,
/// scales) is the MPU-6050-class contract in this crate, and a field is added here only when a
/// supported model actually differs in it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Model {
    /// Expected readback of the WHO_AM_I register (`0x75`).
    pub who_am_i: u8,
}

/// A genuine MPU-6050 (WHO_AM_I reads its own bus address, `0x68`).
pub const MPU6050: Model = Model { who_am_i: 0x68 };

/// The bench F130's MPU-6050 clone: standard register map and address, but WHO_AM_I reads `0x2E`
/// (silicon-observed 2026-06-18, hardware I2C0 PB6/PB7). Config sequence and burst layout are
/// assumed MPU-6050-standard until the bench confirms (`specs/silicon-queue.md`).
pub const CLONE_2E: Model = Model { who_am_i: 0x2E };

/// The registered `imu.model` index lookup (`specs/imu.md`, the field table: the numbering is
/// owned by this crate so the registry field stays a plain scalar): `0` = no IMU fitted,
/// `1` = [`MPU6050`], `2` = [`CLONE_2E`]; anything else is unknown (`None`, the caller's
/// fail-soft). First consumer: the firmware's plan-gated IMU bring-up (`specs/integration.md`).
pub fn model_from_index(index: u8) -> Option<Model> {
    match index {
        1 => Some(MPU6050),
        2 => Some(CLONE_2E),
        _ => None,
    }
}

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

/// Scaling Q type: `base::Fix` (I32F32), identical to `attitude::Fix`.
pub type Fix = base::fixed::Fix;

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
    /// The axis ROLES `[UP, PITCH_RATE]`: which CHIP axis plays each body role, `1 = X`, `2 = Y`,
    /// `3 = Z` (`specs/imu.md`, `IMU_AXIS_ROLE`). The third role, FORWARD (body X), is the remaining
    /// chip axis. [`Imu::decode`] applies them as a permutation of the conditioned words into BODY
    /// order (x-forward, y = the pitch axis, z-up) after sign, bias and clamp, which all stay per
    /// chip axis. [`DEFAULT_ROLES`] (UP = Z, PITCH_RATE = Y) is the identity, the stock flat mount.
    pub roles: [u8; 2],
}

/// The compiled axis roles `[UP, PITCH_RATE]` = `[Z, Y]`: the stock flat mount, under which the
/// body order IS the chip order (the identity permutation). The fallback for an unset (`0`) role.
pub const DEFAULT_ROLES: [u8; 2] = [3, 2];

/// Why [`Config::staged`] refused a frame (`specs/imu.md`, `IMU_AXIS_ROLE`, "Validation").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// A role is outside `1..=3`, or the two roles name the same chip axis.
    Roles,
    /// The accel triple read through the roles is a reflection (determinant -1), or a sign is not
    /// `+-1`.
    Accel,
    /// The gyro triple read through the roles is a reflection, or a sign is not `+-1`.
    Gyro,
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
            roles: DEFAULT_ROLES,
        }
    }
}

impl Config {
    /// Build a config from per-board STAGED values (the store's `IMU_AXIS_SIGN`, `IMU_GYRO_BIAS`
    /// and `IMU_AXIS_ROLE`), applying the unset rule, and REFUSE a frame that is not a rotation.
    ///
    /// A staged sign of **0 means "not configured"** and falls back to that index of the reference
    /// map in [`Config::default`], so a board with nothing staged behaves exactly as it did before
    /// the field existed, and a board with a real mount overrides it per axis. 0 is not a valid
    /// sign, which is what lets one all-indices default express six different per-axis defaults.
    /// A staged role of 0 likewise falls back to that index of [`DEFAULT_ROLES`].
    ///
    /// The rule lives here rather than in the firmware's bring-up because it is this type's own
    /// contract: `sign` and `roles` are the config's fields, so what an absent value means for them
    /// is the config's to say, and saying it here is what makes it host-testable.
    ///
    /// The resolved frame must pass [`Config::frame_is_rotation`] for BOTH triples, or this returns
    /// the [`FrameError`] and the caller does not bring the IMU up: a wrong role is worse than a
    /// wrong sign (the machine balances about the wrong axis and nothing in the loop can tell), so
    /// it is a boot refusal, not only a host-test property.
    pub fn staged(sign: [i32; 6], gyro_bias: [i32; 3], roles: [u8; 2]) -> Result<Self, FrameError> {
        let mut cfg = Config {
            gyro_bias,
            ..Config::default()
        };
        for (dst, staged) in cfg.sign.iter_mut().zip(sign.iter()) {
            if *staged != 0 {
                *dst = *staged;
            }
        }
        for (dst, staged) in cfg.roles.iter_mut().zip(roles.iter()) {
            if *staged != 0 {
                *dst = *staged;
            }
        }
        let s = cfg.sign;
        if body_order(cfg.roles).is_none() {
            Err(FrameError::Roles)
        } else if !Self::frame_is_rotation(cfg.roles, [s[0], s[1], s[2]]) {
            Err(FrameError::Accel)
        } else if !Self::frame_is_rotation(cfg.roles, [s[3], s[4], s[5]]) {
            Err(FrameError::Gyro)
        } else {
            Ok(cfg)
        }
    }

    /// Whether a three-axis sign triple is a proper ROTATION (determinant +1) rather than a
    /// reflection.
    ///
    /// Only the mounts reachable by physically rotating the chip are legal. A per-axis flip with
    /// determinant -1 (e.g. negating only the up axis to make a level board read positive) is a
    /// left-handed frame: the fusion's accel-error cross product then pushes the gyro integration
    /// the wrong way about an axis, which is a diverging estimate rather than an obviously wrong
    /// one. For a diagonal map the determinant is just the product of the three signs.
    pub fn triple_is_rotation(triple: [i32; 3]) -> bool {
        Self::frame_is_rotation(DEFAULT_ROLES, triple)
    }

    /// Whether a sign triple (chip axes) read through the axis `roles` is a proper ROTATION: the
    /// frame `body = P * S * chip` has `det(P * S) = sgn(P) * (s0 * s1 * s2) = +1`.
    ///
    /// Requires each role in `1..=3`, the two roles DISTINCT, each sign `+-1`, and the sign product
    /// equal to the permutation's parity (`+1` for the identity and the two cyclic permutations,
    /// `-1` for the three transpositions). The roles are the RESOLVED ones (no `0`): the unset rule
    /// is [`Config::staged`]'s. With [`DEFAULT_ROLES`] it is [`Config::triple_is_rotation`].
    pub fn frame_is_rotation(roles: [u8; 2], triple: [i32; 3]) -> bool {
        let Some(order) = body_order(roles) else {
            return false;
        };
        // Parity of the permutation body[i] <- chip[order[i]]. A permutation of three is even (the
        // identity or a 3-cycle) exactly when UP's chip axis is the cyclic successor of
        // PITCH_RATE's (identity: Y then Z); otherwise it is one of the three transpositions.
        let parity = if order[2].index() == (order[1].index() + 1) % 3 {
            1
        } else {
            -1
        };
        triple.iter().all(|s| *s == 1 || *s == -1) && triple[0] * triple[1] * triple[2] == parity
    }
}

/// The body-order permutation the roles select: `order[i]` is the CHIP axis (0-based) that becomes
/// body axis `i` (`0 = FORWARD`, `1 = PITCH_RATE`, `2 = UP`), or `None` if a role is outside `1..=3`
/// or the two roles are equal. FORWARD is the remaining axis (the three indices sum to 3).
///
/// This is where a role byte becomes a [`Lane`]: validated ONCE, here, so the per-sample decode
/// indexes by type instead of by checked byte.
fn body_order(roles: [u8; 2]) -> Option<[Lane; 3]> {
    let [up, pitch] = roles;
    if !(1..=3).contains(&up) || !(1..=3).contains(&pitch) || up == pitch {
        return None;
    }
    let (up, pitch) = (up - 1, pitch - 1);
    Some([
        Lane::from_index(3 - up - pitch)?,
        Lane::from_index(pitch)?,
        Lane::from_index(up)?,
    ])
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
    /// the attitude filter directly. BODY order `[forward, pitch, up]` (the roles' permutation).
    pub gyro: [Fix; 3],
    /// Bias-corrected, sign-applied, clamped gyro counts retained for the attitude filter and the
    /// still-detection (spec section 8). BODY order `[forward, pitch, up]`.
    pub gyro_raw: [i16; 3],
    /// Sign-applied, clamped acceleration counts (no bias). Direction-only; the attitude filter
    /// normalizes to a unit gravity vector. BODY order `[forward, pitch, up]`.
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

/// Driver error.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Error<E> {
    /// A transaction failed on the bus. Wraps the underlying `embedded-hal` I2C error.
    Bus(E),
    /// [`Imu::probe`]: the WHO_AM_I readback does not match the configured [`Model`] (a mis-set
    /// model must fail loud, never silently mis-scale).
    Identity { expected: u8, got: u8 },
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

/// MPU-6050-class IMU front-end. Generic over the `embedded-hal` 1.0 I2C bus per call; holds the
/// configured [`Model`], the per-board calibration config, the still-detection state, and the
/// one-tick corrected-sample history.
pub struct Imu {
    model: Model,
    cfg: Config,
    /// The body-order permutation `cfg.roles` selects (`order[i]` = the chip axis that becomes body
    /// axis `i`), resolved once when the config is installed rather than per sample.
    ///
    /// A [`Lane`] rather than a `u8`: the decode runs in the 250 Hz control task, and a `u8`
    /// selector indexing a three-element array emits `panic_bounds_check` there even though
    /// [`body_order`] validated the roles, because the type still admits a 4. A panic in that task
    /// stops the watchdog feed, so an armed bridge holds its last duties for the whole 500 ms IWDG
    /// window (`specs/panic-free.md`).
    order: [Lane; 3],
    gyro_scale: Fix,
    /// Previous tick's six corrected words `[ax, ay, az, gx, gy, gz]`, for still-detection (spec 9).
    prev_words: Option<[i16; 6]>,
    /// Consecutive-identical-sample counter (spec section 9, saturates at [`STILL_COUNTER_CAP`]).
    still_count: u16,
}

impl Imu {
    /// Build the front-end for a [`Model`] with a per-board calibration config (sign map + gyro
    /// bias). Which model a board carries is board configuration, never runtime-guessed.
    pub fn new(model: Model, cfg: Config) -> Self {
        Imu {
            model,
            order: order_of(&cfg),
            cfg,
            gyro_scale: Fix::from_num(GYRO_SCALE),
            prev_words: None,
            still_count: 0,
        }
    }

    /// Identity check (spec "Models"): read WHO_AM_I (`0x75`) and compare against the model's
    /// expected value. Separate from [`Imu::init`]; bring-up order (probe-then-init vs init-blind)
    /// is the caller's policy.
    pub fn probe<I, E>(&mut self, i2c: &mut I) -> Result<(), Error<E>>
    where
        I: I2c<Error = E>,
    {
        let mut got = [0u8; 1];
        i2c.write_read(ADDR, &[REG_WHO_AM_I], &mut got)
            .map_err(Error::Bus)?;
        if got[0] != self.model.who_am_i {
            return Err(Error::Identity {
                expected: self.model.who_am_i,
                got: got[0],
            });
        }
        Ok(())
    }

    /// Replace the calibration config (e.g. after loading the cal page at boot).
    pub fn set_config(&mut self, cfg: Config) {
        self.order = order_of(&cfg);
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

        // Accel: sign then clamp, no bias (spec section 7.3). Chip order.
        let acc_chip = [
            sign_clamp(self.cfg.sign[0], ax),
            sign_clamp(self.cfg.sign[1], ay),
            sign_clamp(self.cfg.sign[2], az),
        ];

        // Gyro: sign, then subtract bias, then clamp (spec section 7.2).
        let raw_g = [gx, gy, gz];
        let mut gyro_chip = [0i16; 3];
        for i in 0..3 {
            let signed = self.cfg.sign[3 + i] * (raw_g[i] as i32);
            let corrected = signed - self.cfg.gyro_bias[i];
            gyro_chip[i] = corrected.clamp(CLAMP_MIN, CLAMP_MAX) as i16;
        }

        // The axis roles: permute the conditioned words from chip order into BODY order
        // (`specs/imu.md`, `IMU_AXIS_ROLE`). Sign, bias and clamp above stay per CHIP axis (the
        // bias is captured in chip axes); everything below, and every consumer, is body-frame.
        let mut acc = [0i16; 3];
        let mut gyro_raw = [0i16; 3];
        for (i, lane) in self.order.iter().enumerate() {
            acc[i] = lane.pick(&acc_chip);
            gyro_raw[i] = lane.pick(&gyro_chip);
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
        let words = [
            acc[0],
            acc[1],
            acc[2],
            gyro_raw[0],
            gyro_raw[1],
            gyro_raw[2],
        ];
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

/// The body-order permutation a config's roles select. A config that passed [`Config::staged`] always
/// has one; a `Config` built by hand with an invalid role pair (no validation runs on that path, as
/// for a hand-built sign map) decodes in chip order, the [`DEFAULT_ROLES`] identity.
fn order_of(cfg: &Config) -> [Lane; 3] {
    body_order(cfg.roles).unwrap_or(Lane::IDENTITY)
}

#[cfg(test)]
mod tests;
