//! Mahony complementary quaternion attitude filter, fixed-point Q.
//!
//! Fuses a conditioned 3-axis angular-rate (gyro, rad/s) vector and a 3-axis acceleration
//! (direction-only) vector into a unit-quaternion orientation estimate, then extracts the three
//! published Euler angles the balance controller reads: a fused-quaternion pitch and two
//! accelerometer inclinations (roll, heading). This is the proportional form of a Mahony filter:
//! proportional gain `Kp` plus fixed per-axis gyro-bias offsets, no running integral accumulator.
//!
//! The reference numeric constants are preserved exactly. See `todo/attitude.md` (the behavioral
//! spec, normative) and `spec/core.md` ("Math: fixed-point Q", the no-FPU constraint).
//!
//! No-FPU adaptation: the reference design computes the filter body in single-precision float and
//! the Euler trig in double. This project bans software float from the 250 Hz loop, so the whole
//! body runs in `I32F32` fixed-point and the trig uses the `cordic` crate (asin / sqrt). The decimal
//! constants below are reproduced in Q; the IEEE-754 bit patterns are provenance only, not
//! reproduced.
//!
//! `no_std`; host tests in `#[cfg(test)]` link `std` via the host target.

#![no_std]

use cordic::{asin, sqrt};
use fixed::types::{I16F16, I32F32};

/// Body-math Q type. 32 fractional bits hold the gyro scale 0.000266316114 to ~4e-7 relative error
/// (an `I16F16` would carry ~3% error on that constant alone, a systematic integrated gain error,
/// so the wider fraction is load-bearing). 31 integer bits hold every body quantity (rates,
/// quaternion derivatives) and the cordic pi/e constants.
pub type Fix = I32F32;

/// Output Q type. Degree values are in +/-90, the output IIR coefficients are 0.1-class; 16
/// fractional bits are ample for a smoothed display/control angle.
pub type Out = I16F16;

// ---------------------------------------------------------------------------------------------
// Fixed numeric constants (spec sections 3, 5, 6, 10). Reproduced as decimals in Q, not bit
// patterns. `const fn from_num` is not available, so these are built once via the helpers below.
// ---------------------------------------------------------------------------------------------

/// Gyro count -> rad/s scale: (500/32768) * (pi/180) = 0.000266316114 (spec section 3, source float
/// `0x398BA058`). Applied to bias-subtracted raw 16-bit gyro counts.
pub const GYRO_SCALE: f64 = 0.000_266_316_114;

/// Integration half-step h = (1/2) * dt = 0.5/250 = 0.002 (spec section 5 step 5, source float
/// `0x3B03126F`). The 1/2 of q-dot = (1/2) q (x) w is folded into h, so the quaternion derivative
/// carries no extra 1/2.
pub const HALF_STEP: f64 = 0.002;

/// Rad-to-deg magnitude 57.29578399658203 = 180/pi (spec section 6, source float `0x42652EE2`
/// widened to double). Pitch and roll use the negative scale (`0xC04CA5DC40000000`), heading the
/// positive (`0x404CA5DC40000000`); the sign is carried on the channel, not here.
pub const RAD_TO_DEG: f64 = 57.295_783_996_582_03;

/// Pre-filter (IMU front-end) IIR coefficients, reproduced so the filter sees the same conditioned
/// input (spec section 3). Primary channel `y <- 0.02*new + 0.98*old`, slow channel
/// `0.01*new + 0.99*old`.
pub const IIR_FAST_NEW: f64 = 0.02;
pub const IIR_FAST_OLD: f64 = 0.98;
pub const IIR_SLOW_NEW: f64 = 0.01;
pub const IIR_SLOW_OLD: f64 = 0.99;

/// Output IIR pair (spec section 6.2, section 10): `out <- a*new + b*prev`. `a = 0.1`
/// (`0x3FB999999999999A`), `b = 0.8999997615814209` (`0x3FECCCCC4CCCCCCD`, NOT exactly 0.9). Pitch
/// uses 0.1 / 0.9 in the source; roll uses 0.1 / 0.8999997615814209. Both reproduced as the source
/// has them.
pub const OUT_IIR_NEW: f64 = 0.1;
pub const OUT_IIR_PREV_PITCH: f64 = 0.9;
pub const OUT_IIR_PREV_ROLL: f64 = 0.899_999_761_581_420_9;

/// Per-board / per-unit calibration and tuning the caller supplies. These are configuration, not
/// fixed math constants (spec sections 4, 10): the reference unit uses `Kp = 1.0`, gyro-bias
/// offsets `= 0.0`, and the natural (x, y, z) axis order with all signs `+1`, but a different
/// board/mount changes them, so they are parameters here, not baked in.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Mahony proportional gain `Kp` (spec slot [7]; reference 1.0). Error scale into the rate.
    pub kp: Fix,
    /// Per-axis Mahony gyro-bias offsets (bx, by, bz) added to the rate (spec slots [4..6];
    /// reference 0.0). Distinct from the per-unit IMU-read bias subtraction.
    pub gyro_bias: [Fix; 3],
    /// Per-axis sign map applied to the raw gyro counts before scaling (spec section 3 / 10;
    /// reference +1, +1, +1). Board sensor-mounting data, load-bearing for correctness.
    pub gyro_sign: [i32; 3],
    /// Per-axis sign map applied to the raw accel counts before use (reference +1, +1, +1).
    pub accel_sign: [i32; 3],
    /// Pitch level-trim, degrees, subtracted from the smoothed pitch before publish (cal idx 6,
    /// centidegrees / 100). The stock roll/fore-aft channel has no trim (= 0).
    pub pitch_trim_deg: Out,
    /// Heading (ay-channel) trim, degrees, subtracted from the heading inclination (cal idx 7).
    pub heading_trim_deg: Out,
}

impl Default for Config {
    /// The reference defaults (spec section 10): Kp = 1.0, zero bias, +1 signs,
    /// zero trims.
    fn default() -> Self {
        Config {
            kp: Fix::from_num(1.0),
            gyro_bias: [Fix::ZERO; 3],
            gyro_sign: [1, 1, 1],
            accel_sign: [1, 1, 1],
            pitch_trim_deg: Out::ZERO,
            heading_trim_deg: Out::ZERO,
        }
    }
}

/// First-order IIR pre-filter state for one channel (`y <- new_w * x + old_w * y`). The IMU
/// front-end owns these in the real system (spec section 3); they are provided here so the filter
/// sees the same conditioned input and the host test can exercise them.
#[derive(Clone, Copy, Debug)]
pub struct Iir {
    new_w: Fix,
    old_w: Fix,
    y: Fix,
    primed: bool,
}

impl Iir {
    /// Build an IIR with the given new/old weights, unprimed (first sample passes through).
    pub fn new(new_w: f64, old_w: f64) -> Self {
        Iir {
            new_w: Fix::from_num(new_w),
            old_w: Fix::from_num(old_w),
            y: Fix::ZERO,
            primed: false,
        }
    }

    /// Fast primary channel: 0.02 / 0.98 (spec section 3).
    pub fn fast() -> Self {
        Iir::new(IIR_FAST_NEW, IIR_FAST_OLD)
    }

    /// Slow secondary channel: 0.01 / 0.99 (spec section 3).
    pub fn slow() -> Self {
        Iir::new(IIR_SLOW_NEW, IIR_SLOW_OLD)
    }

    /// Advance one sample. The first sample initializes the state (no startup ramp from zero).
    pub fn step(&mut self, x: Fix) -> Fix {
        if !self.primed {
            self.y = x;
            self.primed = true;
        } else {
            self.y = self.new_w * x + self.old_w * self.y;
        }
        self.y
    }

    /// Current filtered value.
    pub fn value(&self) -> Fix {
        self.y
    }
}

/// Published attitude outputs (spec section 6). All angles in degrees.
#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    /// Unit quaternion (q0, q1, q2, q3).
    pub q: [Fix; 4],
    /// Fused-quaternion pitch, degrees, after the 0.1/0.9 output IIR and level trim.
    pub pitch_deg: Out,
    /// Accel body-X inclination ("roll" / fore-aft), degrees, after the 0.1/0.8999997615814209 IIR.
    pub roll_deg: Out,
    /// Accel body-Y inclination (heading), degrees, after trim. Unused by the stock balance loop.
    pub heading_deg: Out,
}

/// The Mahony filter: persistent quaternion + output-IIR state, plus the supplied config.
#[derive(Clone, Copy, Debug)]
pub struct Mahony {
    /// Orientation quaternion q = (q0, q1, q2, q3); identity at boot, renormalized every call.
    q: [Fix; 4],
    /// Output-IIR history for pitch and roll (heading is published without an IIR in the stock
    /// path; only pitch and roll are low-pass filtered per spec section 6.2).
    pitch_prev: Out,
    roll_prev: Out,
    pitch_primed: bool,
    roll_primed: bool,
    cfg: Config,
    // Cached Q constants (built once, not per-call, to keep the hot path free of f64).
    half_step: Fix,
    gyro_scale: Fix,
    out_iir_new: Out,
    out_iir_prev_pitch: Out,
    out_iir_prev_roll: Out,
    rad_to_deg: Fix,
}

impl Mahony {
    /// Construct from a config, quaternion at identity (1, 0, 0, 0) per spec section 4.
    pub fn new(cfg: Config) -> Self {
        Mahony {
            q: [Fix::from_num(1.0), Fix::ZERO, Fix::ZERO, Fix::ZERO],
            pitch_prev: Out::ZERO,
            roll_prev: Out::ZERO,
            pitch_primed: false,
            roll_primed: false,
            cfg,
            half_step: Fix::from_num(HALF_STEP),
            gyro_scale: Fix::from_num(GYRO_SCALE),
            out_iir_new: Out::from_num(OUT_IIR_NEW),
            out_iir_prev_pitch: Out::from_num(OUT_IIR_PREV_PITCH),
            out_iir_prev_roll: Out::from_num(OUT_IIR_PREV_ROLL),
            rad_to_deg: Fix::from_num(RAD_TO_DEG),
        }
    }

    /// Current quaternion state.
    pub fn quaternion(&self) -> [Fix; 4] {
        self.q
    }

    /// Replace the config (board/tuning data) without disturbing the quaternion state.
    pub fn set_config(&mut self, cfg: Config) {
        self.cfg = cfg;
    }

    /// Scale a raw 16-bit gyro count to rad/s with the sign map applied (spec section 3). The IMU
    /// front-end normally does the bias subtraction; this applies the reference sign and the
    /// 0.000266316114 scale.
    pub fn gyro_to_rad(&self, axis: usize, raw: i32) -> Fix {
        let signed = Fix::from_num(self.cfg.gyro_sign[axis] * raw);
        signed * self.gyro_scale
    }

    /// Apply the accel sign map to a raw count (no unit scale; only direction is used, spec
    /// section 3).
    pub fn accel_signed(&self, axis: usize, raw: i32) -> Fix {
        Fix::from_num(self.cfg.accel_sign[axis] * raw)
    }

    /// One full filter step from already-conditioned inputs: `gyro` in rad/s (scaled + sign-applied,
    /// e.g. via [`Self::gyro_to_rad`]), `accel` as sign-applied counts (direction only). Runs the
    /// Mahony body, renormalizes the quaternion, extracts and IIR-smooths the three Euler angles,
    /// and returns the published [`Output`]. This is the per-250-Hz-tick entry point (spec section
    /// 2, 5, 6).
    pub fn update(&mut self, gyro: [Fix; 3], accel: [Fix; 3]) -> Output {
        let (gx, gy, gz) = (gyro[0], gyro[1], gyro[2]);
        let (q0, q1, q2, q3) = (self.q[0], self.q[1], self.q[2], self.q[3]);

        // --- Step 1: accel as a usable direction. magnitude = sqrt(ax^2 + ay^2 + az^2). ---
        // Raw sum-of-squares can reach 3 * 32768^2 = 3.2e9, which overflows I32F32's integer range
        // (~2.1e9). Right-shift each component by 1 before squaring: the /2 cancels in the unit
        // vector (direction only), and 14-bit accel is far more precision than the direction needs.
        let ax_h = accel[0] >> 1;
        let ay_h = accel[1] >> 1;
        let az_h = accel[2] >> 1;
        let mag2 = ax_h * ax_h + ay_h * ay_h + az_h * az_h;

        // ahat is the normalized accel unit vector; (ax, ay, az) are the conditioned components used
        // for the accel inclinations in step 6 (spec section 6.1 reads offsets 0 and 1).
        let (ahat, have_accel) = if mag2 == Fix::ZERO {
            ([Fix::ZERO; 3], false)
        } else {
            let mag = sqrt(mag2);
            (
                [ax_h / mag, ay_h / mag, az_h / mag],
                true,
            )
        };

        // --- Steps 2-4: gravity estimate, direction error, proportional feedback into the rate. ---
        // wx/wy/wz default to gyro + bias only; the accel correction is added when usable.
        let mut wx = gx + self.cfg.gyro_bias[0];
        let mut wy = gy + self.cfg.gyro_bias[1];
        let mut wz = gz + self.cfg.gyro_bias[2];

        if have_accel {
            // Step 2: estimated body-frame gravity (the q rotation-matrix "down" column). The factor
            // 2 is a 1-bit left shift in the source (scalbnf(x, 1)); here it is `<< 1`.
            let vx = (q1 * q3 - q0 * q2) << 1;
            let vy = (q0 * q1 + q2 * q3) << 1;
            // vz = q0^2 - q1^2 - q2^2 + q3^2, computed as (q0^2 - q1^2 - q2^2) then + q3^2.
            let vz = (q0 * q0 - q1 * q1 - q2 * q2) + q3 * q3;

            // Step 3: direction error = ahat x v (cross product).
            let ex = ahat[1] * vz - ahat[2] * vy;
            let ey = ahat[2] * vx - ahat[0] * vz;
            let ez = ahat[0] * vy - ahat[1] * vx;

            // Step 4: proportional feedback. wi = Kp * ei + gi + bi (no integral accumulation).
            wx += self.cfg.kp * ex;
            wy += self.cfg.kp * ey;
            wz += self.cfg.kp * ez;
        }

        // --- Step 5: quaternion derivative and fixed-half-step integration. ---
        // q-dot = q (x) (0, wx, wy, wz), factored. The 1/2 is folded into HALF_STEP.
        let dq0 = -q1 * wx - q2 * wy - q3 * wz;
        let dq1 = q0 * wx + q2 * wz - q3 * wy;
        let dq2 = q0 * wy - q1 * wz + q3 * wx;
        let dq3 = q0 * wz + q1 * wy - q2 * wx;

        let h = self.half_step;
        let mut nq = [
            q0 + h * dq0,
            q1 + h * dq1,
            q2 + h * dq2,
            q3 + h * dq3,
        ];

        // --- Step 6: renormalize to unit length. On a zero norm, keep the last good quaternion. ---
        let norm2 = nq[0] * nq[0] + nq[1] * nq[1] + nq[2] * nq[2] + nq[3] * nq[3];
        if norm2 != Fix::ZERO {
            let norm = sqrt(norm2);
            if norm != Fix::ZERO {
                nq = [nq[0] / norm, nq[1] / norm, nq[2] / norm, nq[3] / norm];
                self.q = nq;
            }
        }
        // else: leave self.q unchanged (fall back to last good quaternion).

        // --- Euler extraction (spec section 6.1). ---
        let (nq0, nq1, nq2, nq3) = (self.q[0], self.q[1], self.q[2], self.q[3]);

        // Pitch: fused-quaternion angle asin(2*(q1*q3 - q0*q2)) * (-57.29578). Clamp the asin arg to
        // [-1, +1] so the poles give a finite result.
        let pitch_arg = clamp_unit((nq1 * nq3 - nq0 * nq2) << 1);
        let pitch_rad = asin(pitch_arg);
        let pitch_deg_raw = -(pitch_rad * self.rad_to_deg);

        // Roll: accel body-X inclination -asin(ax/|a|) * 57.2957840, i.e. asin(ax/|a|) * (-scale).
        // ahat[0] is already ax/|a| (offset 0). Clamp.
        // Heading: accel body-Y inclination +asin(ay/|a|) * 57.2957840 (offset 1). Clamp.
        let (roll_deg_raw, heading_deg_raw) = if have_accel {
            let roll = -(asin(clamp_unit(ahat[0])) * self.rad_to_deg);
            let heading = asin(clamp_unit(ahat[1])) * self.rad_to_deg;
            (roll, heading)
        } else {
            // No accel direction this tick: hold the inclinations at zero contribution.
            (Fix::ZERO, Fix::ZERO)
        };

        // Narrow degree values from the body type to the output type.
        let pitch_new = to_out(pitch_deg_raw);
        let roll_new = to_out(roll_deg_raw);
        let heading_new = to_out(heading_deg_raw);

        // --- Step 6.2: output IIR smoothing and trims. ---
        // Pitch: 0.1*new + 0.9*prev, then subtract the level trim. Roll: 0.1*new + 0.899...*prev,
        // no trim (stock roll channel has no trim). Heading: trim subtraction, no IIR (stock).
        let pitch_smoothed = if self.pitch_primed {
            self.out_iir_new * pitch_new + self.out_iir_prev_pitch * self.pitch_prev
        } else {
            self.pitch_primed = true;
            pitch_new
        };
        self.pitch_prev = pitch_smoothed;

        let roll_smoothed = if self.roll_primed {
            self.out_iir_new * roll_new + self.out_iir_prev_roll * self.roll_prev
        } else {
            self.roll_primed = true;
            roll_new
        };
        self.roll_prev = roll_smoothed;

        let pitch_pub = pitch_smoothed - self.cfg.pitch_trim_deg;
        let roll_pub = roll_smoothed; // no stock trim on the roll/fore-aft channel
        let heading_pub = heading_new - self.cfg.heading_trim_deg;

        Output {
            q: self.q,
            pitch_deg: pitch_pub,
            roll_deg: roll_pub,
            heading_deg: heading_pub,
        }
    }
}

/// Clamp an asin argument to the closed domain [-1, +1] so +/-90 deg gives a finite result, never
/// NaN (spec sections 5, 6). cordic asin is only valid on [-1, 1].
fn clamp_unit(x: Fix) -> Fix {
    let one = Fix::from_num(1.0);
    if x > one {
        one
    } else if x < -one {
        -one
    } else {
        x
    }
}

/// Narrow a body-type degree value (I32F32) to the output type (I16F16), saturating. Degrees are in
/// +/-90 so this never actually saturates in normal operation.
fn to_out(x: Fix) -> Out {
    Out::saturating_from_num(x)
}

#[cfg(test)]
mod tests;
