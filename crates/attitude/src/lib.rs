//! Mahony complementary quaternion attitude filter, fixed-point Q (`specs/attitude.md`).
//!
//! Fuses a conditioned 3-axis angular-rate (gyro, rad/s) vector and a 3-axis acceleration
//! (direction-only) vector into a unit-quaternion orientation estimate, then publishes **pitch and
//! roll, both extracted from that fused quaternion** (Tait-Bryan ZYX). This is the proportional
//! form of a Mahony filter: proportional gain `Kp` plus fixed per-axis gyro-bias offsets, no
//! running integral accumulator.
//!
//! Pitch is preserved bit-for-bit against the recovered design. Roll is the spec's deliberate
//! upgrade: the stock firmware published roll as an accelerometer body-X inclination (and a second
//! accel "heading" inclination that nothing consumed); this filter publishes the fused-quaternion
//! ZYX roll `atan2(vy, vz)` instead and carries **no heading channel** (see the spec, "Divergence
//! from stock" and "Yaw").
//!
//! The recovered numeric constants are preserved exactly. The shared device fact `GYRO_SCALE` is
//! owned by `crates/imu` (this crate's input layer); the pre-filter `Iir` lives there too.
//!
//! No-FPU adaptation: the reference design computes the filter body in single-precision float and
//! the Euler trig in double. This project bans software float from the 250 Hz loop, so the whole
//! body runs in `I32F32` fixed-point and the trig uses `cordic` (asin / atan2 / sqrt, via `base`).
//! The decimal constants below are reproduced in Q; the IEEE-754 bit patterns are provenance only.
//!
//! `no_std`; host tests in `#[cfg(test)]` link `std` via the host target.

#![no_std]

// `atan2` is NOT taken from cordic: the local one below is bit-exact with it and a quarter the
// flash (it hoists the divide and the CORDIC kernel out of the four quadrant arms). Every `Fix`
// division on this path goes through `base::fixed::div`, the image's single division body, rather
// than a `/` that would expand a fresh 128-bit software divide inline at each site.
use base::fixed::cordic::{asin, atan, sqrt};
use base::fixed::div;

/// Body-math Q type (`base::Fix`, I32F32). 32 fractional bits hold the gyro scale 0.000266316114
/// to ~4e-7 relative error (an `I16F16` would carry ~3% error on that constant alone, a systematic
/// integrated gain error, so the wider fraction is load-bearing). 31 integer bits hold every body
/// quantity (rates, quaternion derivatives) and the cordic pi/e constants.
pub type Fix = base::fixed::Fix;

/// Output Q type (`base::Out`, I16F16). Degree values are in +/-90, the output IIR coefficients
/// are 0.1-class; 16 fractional bits are ample for a smoothed display/control angle.
pub type Out = base::fixed::Out;

// ---------------------------------------------------------------------------------------------
// Fixed numeric constants (spec sections 3, 5, 6, 10). Reproduced as decimals in Q, not bit
// patterns. `const fn from_num` is not available, so these are built once via the helpers below.
// ---------------------------------------------------------------------------------------------

/// Integration half-step h = (1/2) * dt = 0.5/250 = 0.002 (spec section 5 step 5, source float
/// `0x3B03126F`). The 1/2 of q-dot = (1/2) q (x) w is folded into h, so the quaternion derivative
/// carries no extra 1/2.
pub const HALF_STEP: f64 = 0.002;

/// The [`Mahony::update_dt`] clamp: at most 25 nominal ticks (100 ms, the supervision-window
/// scale) integrate as one step; a staler gap integrates as if 100 ms (the accel correction
/// re-converges the remainder rather than one huge gyro step overshooting).
pub const MAX_DT_TICKS: u32 = 25;

/// Rad-to-deg magnitude 57.29578399658203 = 180/pi (spec "Outputs", source float `0x42652EE2`
/// widened to double). Pitch and the fused roll both use the negative scale
/// (`0xC04CA5DC40000000`, the recovered channel sign convention); the sign is carried on the
/// channel, not here.
pub const RAD_TO_DEG: f64 = 57.295_783_996_582_03;

/// Output IIR pair (spec "Output IIR and level trims"): `out <- a*new + b*prev`. `a = 0.1`
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
    /// centidegrees / 100).
    pub pitch_trim_deg: Out,
    /// Roll level-trim, degrees, subtracted from the smoothed fused roll before publish. The stock
    /// roll channel had no trim (a different, accel-inclination quantity); the fused roll gets its
    /// own per-unit trim, default 0.0 (spec "Open questions" -> reuse-IIR + 0.0 trim).
    pub roll_trim_deg: Out,
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
            roll_trim_deg: Out::ZERO,
        }
    }
}

impl Config {
    /// The reference config with this board's staged LEVEL TRIMS applied: `[pitch, roll]` in
    /// CENTIDEGREES, the unit the store field carries (`store::ATTITUDE_LEVEL_TRIM`, indices 0/1)
    /// and the unit stock's per-board cal page used.
    ///
    /// The trim is per-board mounting calibration, so it comes from the store rather than from a
    /// compiled constant, exactly as the IMU's sign map and gyro bias do (`imu::Config::staged`,
    /// the same idiom one layer up). `0` means untrimmed and is the registry default, so a board
    /// with nothing staged gets `Config::default()` unchanged.
    ///
    /// The other fields stay at their reference values deliberately: the same stock recovery that
    /// shows the trims differing per board shows `Kp` and the fusion gyro bias identical on both
    /// halves, so those are firmware-fixed and have no field to read.
    pub fn staged(level_trim_centideg: [i16; 2]) -> Self {
        let deg = |centi: i16| Out::from_num(centi) / Out::from_num(100);
        Config {
            pitch_trim_deg: deg(level_trim_centideg[0]),
            roll_trim_deg: deg(level_trim_centideg[1]),
            ..Config::default()
        }
    }
}

/// Published attitude outputs (spec "Outputs"). All angles in degrees. No yaw / heading field:
/// a 6-axis IMU cannot observe yaw, and the stock accel "heading" inclination is removed (spec
/// "Yaw" / "Divergence from stock").
#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    /// Unit quaternion (q0, q1, q2, q3).
    pub q: [Fix; 4],
    /// Fused-quaternion ZYX pitch, degrees, after the 0.1/0.9 output IIR and level trim
    /// (bit-exact with the recovered design).
    pub pitch_deg: Out,
    /// Fused-quaternion ZYX roll, degrees, after the 0.1/0.8999997615814209 output IIR and level
    /// trim (the spec's deliberate upgrade from the stock accel inclination).
    pub roll_deg: Out,
}

/// The Mahony filter: persistent quaternion + output-IIR state, plus the supplied config.
#[derive(Clone, Copy, Debug)]
pub struct Mahony {
    /// Orientation quaternion q = (q0, q1, q2, q3); identity at boot, renormalized every call.
    q: [Fix; 4],
    /// Output-IIR history for the two published channels (spec "Output IIR and level trims").
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
            gyro_scale: Fix::from_num(imu::GYRO_SCALE),
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

    /// Scale a raw 16-bit gyro count to rad/s with the sign map applied. The IMU front-end
    /// normally does the bias subtraction and scaling ([`imu::GYRO_SCALE`] is the constant's
    /// single owner); this applies the filter's own sign map and the same scale, for callers
    /// driving the filter from raw counts (tests, diagnostics).
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
    /// Mahony body (spec steps 1-6), renormalizes the quaternion, extracts and IIR-smooths pitch
    /// and roll from the updated quaternion, applies the trims, and returns the published
    /// [`Output`]. This is the per-250-Hz-tick entry point.
    pub fn update(&mut self, gyro: [Fix; 3], accel: [Fix; 3]) -> Output {
        self.update_dt(gyro, accel, 1)
    }

    /// [`Mahony::update`] with an EXPLICIT elapsed time of `dt_ticks` nominal 250 Hz ticks (the
    /// dt-honest entry, round-4 defect B): the quaternion integration step scales by the ticks
    /// that actually elapsed since the previous run, so an overloaded scheduler (control running
    /// late) integrates the gyro over the real interval instead of silently assuming 4 ms. The
    /// audit-measured symptom of the dishonest dt was pitch oscillating +/-35-70 deg under flood
    /// (real period 5-40 ms) and drifting ~-0.47 deg/s at 194.9 Hz. `dt_ticks` is clamped to
    /// [1, MAX_DT_TICKS]: 0 is degenerate (a run implies time passed) and beyond 100 ms the
    /// sample is stale enough that a larger single step only amplifies its error (the supervision
    /// window scale; the accel term re-converges the residual).
    pub fn update_dt(&mut self, gyro: [Fix; 3], accel: [Fix; 3], dt_ticks: u32) -> Output {
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

        // ahat is the normalized accel unit vector (direction only; the /2 pre-shift cancels).
        let (ahat, have_accel) = if mag2 == Fix::ZERO {
            ([Fix::ZERO; 3], false)
        } else {
            let mag = sqrt(mag2);
            ([div(ax_h, mag), div(ay_h, mag), div(az_h, mag)], true)
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

        let h = self.half_step * Fix::from_num(dt_ticks.clamp(1, MAX_DT_TICKS));
        let mut nq = [q0 + h * dq0, q1 + h * dq1, q2 + h * dq2, q3 + h * dq3];

        // --- Step 6: renormalize to unit length. On a zero norm, keep the last good quaternion. ---
        let norm2 = nq[0] * nq[0] + nq[1] * nq[1] + nq[2] * nq[2] + nq[3] * nq[3];
        if norm2 != Fix::ZERO {
            let norm = sqrt(norm2);
            if norm != Fix::ZERO {
                nq = [
                    div(nq[0], norm),
                    div(nq[1], norm),
                    div(nq[2], norm),
                    div(nq[3], norm),
                ];
                self.q = nq;
            }
        }
        // else: leave self.q unchanged (fall back to last good quaternion).

        // --- Euler extraction (spec "Outputs"): both angles from the fused quaternion AFTER
        // step 6 (the renormalized state; step 2's gravity column was pre-update). Tait-Bryan ZYX.
        let (nq0, nq1, nq2, nq3) = (self.q[0], self.q[1], self.q[2], self.q[3]);

        // Pitch: asin(2*(q1*q3 - q0*q2)) * (-57.29578). Clamp the asin arg to [-1, +1] so the
        // poles give a finite result. Bit-exact with the recovered design.
        let pitch_arg = clamp_unit((nq1 * nq3 - nq0 * nq2) << 1);
        let pitch_rad = asin(pitch_arg);
        let pitch_deg_raw = -(pitch_rad * self.rad_to_deg);

        // Roll: the ZYX roll atan2(vy, vz) on the step-2 gravity-column formulas evaluated on the
        // updated quaternion, with the recovered negative channel scale (the spec's deliberate
        // upgrade from the stock accel inclination). Extracted every tick, accel or not: the
        // quaternion is always defined. atan2 needs no clamp; guard only the (0, 0) pole (exactly
        // +/-90 deg pitch), where the f64 reference's atan2(0, 0) is 0.
        let vy_e = (nq0 * nq1 + nq2 * nq3) << 1;
        let vz_e = (nq0 * nq0 - nq1 * nq1 - nq2 * nq2) + nq3 * nq3;
        let roll_rad = if vy_e == Fix::ZERO && vz_e == Fix::ZERO {
            Fix::ZERO
        } else {
            atan2(vy_e, vz_e)
        };
        let roll_deg_raw = -(roll_rad * self.rad_to_deg);

        // Narrow degree values from the body type to the output type.
        let pitch_new = to_out(pitch_deg_raw);
        let roll_new = to_out(roll_deg_raw);

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
        let roll_pub = roll_smoothed - self.cfg.roll_trim_deg;

        Output {
            q: self.q,
            pitch_deg: pitch_pub,
            roll_deg: roll_pub,
        }
    }
}

/// Clamp an asin argument to the closed domain [-1, +1] so +/-90 deg gives a finite result, never
/// NaN (spec "Outputs"). cordic asin is only valid on [-1, 1]; atan2 (the roll path) needs none.
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

/// `atan2(y, x)` over [`Fix`], **bit-exact** with `cordic::atan2` and one quarter of its flash cost.
///
/// `cordic::atan2` writes the division and the `atan` call **inside each of its four quadrant match
/// arms**. At `I32F32` each `atan` is a 32-iteration CORDIC kernel and each `/` is a ~730 B inlined
/// 128-bit software division (see [`base::fixed::div`]), so the upstream shape puts **four static
/// copies of both** in the image while exactly one ever executes. Measured cost of that duplication
/// on the `717efe0` image: 2,680 B of division plus 552 B of collapsed `atan` bodies; hoisting is
/// worth **-3,200 B** of flashed span (`specs/wide-division-menu.md`, item S2). It is also the one
/// group the shared-body lever cannot reach on its own, because those divisions live in an upstream
/// crate that cannot be routed through our helper. The upstream crate is not patched or forked; only
/// the call site moves here.
///
/// Cycles are unchanged: the same single division and the same single `atan` execute, and only the
/// three copies that never ran are gone.
///
/// **Why the hoist is bit-exact**, arm by arm, against the upstream body
/// (`cordic-0.1.5/src/lib.rs:125`). The two zero guards are reproduced verbatim. Past them, write
/// `q = y / x`:
///
/// - Fixed-point division truncates **toward zero**, and truncation toward zero is odd-symmetric, so
///   `(-y) / x` and `y / (-x)` both equal `-(y / x)` on every bit. That is what lets the divide move
///   above the match: upstream's `-y / x` (arm 2) and `y / -x` (arm 3) are `-q` exactly.
/// - Arm `(x>=0, y>0)`: `atan(y/x)` = `atan(q)`, and `q > 0`.
/// - Arm `(x>=0, y<0)`: `-atan(-y/x)` = `-atan(-q)`, and `-q > 0`.
/// - Arm `(x<0, y>0)`: `pi - atan(y/-x)` = `pi - atan(-q)`, and `-q > 0`.
/// - Arm `(x<0, y<0)`: `atan(y/x) - pi` = `atan(q) - pi`, and `q > 0`.
///
/// So every arm's `atan` argument is `q` when the signs agree and `-q` when they differ, always
/// positive, which is exactly the selection below; the per-arm `pi` fix-up is then pure addition on
/// the shared result. `Fix::PI` is the same constant upstream uses (`CordicNumber::pi()` returns the
/// `fixed` type's own `PI`), so the fix-up is bit-identical too.
///
/// The 400k-sample property test in `tests` is the oracle, with `cordic::atan2` itself as reference.
fn atan2(y: Fix, x: Fix) -> Fix {
    // The two zero guards, verbatim from upstream: no division happens on either path.
    if x == Fix::ZERO {
        return if y < Fix::ZERO {
            -Fix::FRAC_PI_2
        } else {
            Fix::FRAC_PI_2
        };
    }
    if y == Fix::ZERO {
        return if x >= Fix::ZERO { Fix::ZERO } else { Fix::PI };
    }

    let x_neg = x < Fix::ZERO;
    let y_neg = y < Fix::ZERO;

    // ONE division and ONE CORDIC kernel for all four quadrants.
    let q = div(y, x);
    let a = atan(if x_neg == y_neg { q } else { -q });

    match (x_neg, y_neg) {
        (false, false) => a,
        (false, true) => -a,
        (true, false) => Fix::PI - a,
        (true, true) => a - Fix::PI,
    }
}

#[cfg(test)]
mod tests;
