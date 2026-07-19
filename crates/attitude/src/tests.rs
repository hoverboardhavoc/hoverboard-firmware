//! Host tests for the fixed-point Mahony filter.
//!
//! The required validation (`specs/attitude.md`, "Build / test plan") is: the fixed-point
//! implementation tracks a double-precision reference fed the identical input stream, within a
//! documented tolerance band that absorbs the float-to-Q quantization. The reference below
//! recomputes every spec formula in f64 (gyro scale, h = 0.002, the cross-product/quaternion
//! structure, renormalization, the pitch `asin` and fused-roll `atan2` extractions from the
//! post-step-6 quaternion, the 0.1/0.9 and 0.1/0.8999997615814209 output IIRs, the trims). Tests
//! may use `std`/`f64`; the library itself is `no_std` fixed-point.
//!
//! Ported from the archived pre-reset suite (`archive/accumulated-build`), with the reference and
//! the orientation tests upgraded from the stock accel-inclination roll (+ heading channel) to the
//! spec's fused-quaternion roll (no heading).

use super::*;
use imu::GYRO_SCALE;

// ---------------------------------------------------------------------------------------------
// Double-precision reference (the spec formulas, recomputed in f64).
// ---------------------------------------------------------------------------------------------

struct RefConfig {
    kp: f64,
    gyro_bias: [f64; 3],
    gyro_sign: [f64; 3],
    pitch_trim_deg: f64,
    roll_trim_deg: f64,
}

impl Default for RefConfig {
    fn default() -> Self {
        RefConfig {
            kp: 1.0,
            gyro_bias: [0.0; 3],
            gyro_sign: [1.0; 3],
            pitch_trim_deg: 0.0,
            roll_trim_deg: 0.0,
        }
    }
}

struct RefMahony {
    q: [f64; 4],
    pitch_prev: f64,
    roll_prev: f64,
    pitch_primed: bool,
    roll_primed: bool,
    cfg: RefConfig,
}

impl RefMahony {
    fn new(cfg: RefConfig) -> Self {
        RefMahony {
            q: [1.0, 0.0, 0.0, 0.0],
            pitch_prev: 0.0,
            roll_prev: 0.0,
            pitch_primed: false,
            roll_primed: false,
            cfg,
        }
    }

    fn gyro_to_rad(&self, axis: usize, raw: i32) -> f64 {
        self.cfg.gyro_sign[axis] * (raw as f64) * GYRO_SCALE
    }

    /// One step in f64, matching the spec operation order exactly. Returns (pitch, roll) degrees.
    fn update(&mut self, gyro: [f64; 3], accel: [f64; 3]) -> (f64, f64) {
        let [gx, gy, gz] = gyro;
        let [q0, q1, q2, q3] = self.q;

        // Step 1: accel direction (with the same /2 pre-shift the fixed path uses, which cancels).
        let ah = [accel[0] * 0.5, accel[1] * 0.5, accel[2] * 0.5];
        let mag2 = ah[0] * ah[0] + ah[1] * ah[1] + ah[2] * ah[2];
        let (ahat, have_accel) = if mag2 == 0.0 {
            ([0.0; 3], false)
        } else {
            let mag = mag2.sqrt();
            ([ah[0] / mag, ah[1] / mag, ah[2] / mag], true)
        };

        let mut wx = gx + self.cfg.gyro_bias[0];
        let mut wy = gy + self.cfg.gyro_bias[1];
        let mut wz = gz + self.cfg.gyro_bias[2];

        if have_accel {
            // Step 2: estimated body-frame gravity (pre-update quaternion).
            let vx = 2.0 * (q1 * q3 - q0 * q2);
            let vy = 2.0 * (q0 * q1 + q2 * q3);
            let vz = (q0 * q0 - q1 * q1 - q2 * q2) + q3 * q3;

            // Step 3: direction error = ahat x v.
            let ex = ahat[1] * vz - ahat[2] * vy;
            let ey = ahat[2] * vx - ahat[0] * vz;
            let ez = ahat[0] * vy - ahat[1] * vx;

            // Step 4: proportional feedback.
            wx += self.cfg.kp * ex;
            wy += self.cfg.kp * ey;
            wz += self.cfg.kp * ez;
        }

        // Step 5: quaternion derivative + fixed-half-step integration.
        let dq0 = -q1 * wx - q2 * wy - q3 * wz;
        let dq1 = q0 * wx + q2 * wz - q3 * wy;
        let dq2 = q0 * wy - q1 * wz + q3 * wx;
        let dq3 = q0 * wz + q1 * wy - q2 * wx;

        let h = HALF_STEP;
        let mut nq = [q0 + h * dq0, q1 + h * dq1, q2 + h * dq2, q3 + h * dq3];

        // Step 6: renormalize.
        let norm2 = nq[0] * nq[0] + nq[1] * nq[1] + nq[2] * nq[2] + nq[3] * nq[3];
        if norm2 != 0.0 {
            let norm = norm2.sqrt();
            if norm != 0.0 {
                nq = [nq[0] / norm, nq[1] / norm, nq[2] / norm, nq[3] / norm];
                self.q = nq;
            }
        }

        // Extraction: BOTH angles from the post-step-6 quaternion (Tait-Bryan ZYX), every tick.
        let [nq0, nq1, nq2, nq3] = self.q;

        let pitch_arg = clamp_f64(2.0 * (nq1 * nq3 - nq0 * nq2));
        let pitch_deg_raw = -(pitch_arg.asin() * RAD_TO_DEG);

        let vy_e = 2.0 * (nq0 * nq1 + nq2 * nq3);
        let vz_e = (nq0 * nq0 - nq1 * nq1 - nq2 * nq2) + nq3 * nq3;
        // f64 atan2(0, 0) is 0, matching the fixed path's explicit (0, 0) guard.
        let roll_deg_raw = -(vy_e.atan2(vz_e) * RAD_TO_DEG);

        // Output IIRs (0.1/0.9 pitch, 0.1/0.8999997615814209 roll), then the trims.
        let pitch_smoothed = if self.pitch_primed {
            OUT_IIR_NEW * pitch_deg_raw + OUT_IIR_PREV_PITCH * self.pitch_prev
        } else {
            self.pitch_primed = true;
            pitch_deg_raw
        };
        self.pitch_prev = pitch_smoothed;

        let roll_smoothed = if self.roll_primed {
            OUT_IIR_NEW * roll_deg_raw + OUT_IIR_PREV_ROLL * self.roll_prev
        } else {
            self.roll_primed = true;
            roll_deg_raw
        };
        self.roll_prev = roll_smoothed;

        (
            pitch_smoothed - self.cfg.pitch_trim_deg,
            roll_smoothed - self.cfg.roll_trim_deg,
        )
    }
}

fn clamp_f64(x: f64) -> f64 {
    x.clamp(-1.0, 1.0)
}

// ---------------------------------------------------------------------------------------------
// Tolerances. The dominant error source is cordic asin/atan2 (~0.01 rad absolute), which at the
// 57.29578 deg/rad scale is up to ~0.6 deg on a single extracted angle; the output IIR then
// attenuates per-step error. Q quantization of the body (I32F32) is ~1e-7, negligible beside the
// trig error. These bands are sized to absorb the cordic trig error, not to hide a formula
// mismatch (a sign or order error blows past them by tens of degrees).
// ---------------------------------------------------------------------------------------------

/// Degree tolerance for a published angle (dominated by cordic trig error through the 57.29578
/// scale). Shared by pitch (asin) and roll (atan2); `cordic_atan2_stays_inside_the_band` pins
/// that atan2 does not need a wider roll-only band.
const DEG_TOL: f64 = 0.8;
/// Quaternion-component tolerance (body Q quantization + accumulated trig-free integration error).
const Q_TOL: f64 = 1e-3;

fn out_f64(m: &Mahony) -> [f64; 4] {
    let q = m.quaternion();
    [
        q[0].to_num::<f64>(),
        q[1].to_num::<f64>(),
        q[2].to_num::<f64>(),
        q[3].to_num::<f64>(),
    ]
}

/// Accel triple leaning `deg` degrees from +Z toward the given axis (0 = +X, 1 = +Y), at a 1 g
/// 16384-count magnitude.
fn tilted_accel(deg: f64, axis: usize) -> [Fix; 3] {
    let s = (deg.to_radians().sin() * 16384.0).round();
    let c = (deg.to_radians().cos() * 16384.0).round();
    let mut a = [Fix::ZERO, Fix::ZERO, Fix::from_num(c)];
    a[axis] = Fix::from_num(s);
    a
}

fn tilted_accel_ref(deg: f64, axis: usize) -> [f64; 3] {
    let s = (deg.to_radians().sin() * 16384.0).round();
    let c = (deg.to_radians().cos() * 16384.0).round();
    let mut a = [0.0, 0.0, c];
    a[axis] = s;
    a
}

// ---------------------------------------------------------------------------------------------
// Constant-reproduction tests: the Q constants match the decimal targets within the format's
// resolution (spec: reproduce decimals, not bit patterns).
// ---------------------------------------------------------------------------------------------

#[test]
fn constants_reproduced_in_q() {
    // I32F32 resolution is 2^-32 ~ 2.3e-10; these constants land well inside that.
    assert!((Fix::from_num(GYRO_SCALE).to_num::<f64>() - GYRO_SCALE).abs() < 1e-9);
    assert!((Fix::from_num(HALF_STEP).to_num::<f64>() - HALF_STEP).abs() < 1e-9);
    assert!((Fix::from_num(RAD_TO_DEG).to_num::<f64>() - RAD_TO_DEG).abs() < 1e-8);
    // Output IIR coefficients in I16F16 (resolution 2^-16 ~ 1.5e-5).
    assert!((Out::from_num(OUT_IIR_NEW).to_num::<f64>() - OUT_IIR_NEW).abs() < 1e-4);
    assert!((Out::from_num(OUT_IIR_PREV_PITCH).to_num::<f64>() - OUT_IIR_PREV_PITCH).abs() < 1e-4);
    assert!((Out::from_num(OUT_IIR_PREV_ROLL).to_num::<f64>() - OUT_IIR_PREV_ROLL).abs() < 1e-4);
    // The roll prev coefficient is specifically NOT 0.9: 0.8999997615814209.
    assert!((OUT_IIR_PREV_ROLL - 0.9).abs() > 1e-7);
}

#[test]
fn gyro_scale_is_full_scale_relation() {
    // 0.000266316114 == (500/32768) * (pi/180); the constant's single owner is crates/imu.
    let expect = (500.0 / 32768.0) * (core::f64::consts::PI / 180.0);
    assert!((GYRO_SCALE - expect).abs() < 1e-9);
}

#[test]
fn half_step_relation() {
    // h = 0.5 * dt, dt = 1/250 = 0.004.
    let dt = 1.0 / 250.0;
    assert!((HALF_STEP - 0.5 * dt).abs() < 1e-12);
    assert!((dt - 0.004).abs() < 1e-12);
}

#[test]
fn cordic_atan2_stays_inside_the_band() {
    // Spec open question: the cordic atan2 error through the degree scale must stay inside
    // DEG_TOL, else roll needs a wider stated band. Sweep directions around the circle.
    for k in 0..72 {
        let ang = (k as f64) * 5.0_f64.to_radians();
        let (y, x) = (ang.sin(), ang.cos());
        let got: f64 = atan2(Fix::from_num(y), Fix::from_num(x)).to_num();
        let err_deg = (got - y.atan2(x)).abs() * RAD_TO_DEG;
        assert!(err_deg < DEG_TOL, "atan2 err {err_deg} deg at {k}*5deg");
    }
}

// ---------------------------------------------------------------------------------------------
// Still sensor (gravity only) converges to level.
// ---------------------------------------------------------------------------------------------

#[test]
fn still_sensor_converges_to_level() {
    // Gravity along +Z (az positive), no rotation. From identity the estimate should stay level:
    // pitch and roll near 0, quaternion near identity.
    let mut m = Mahony::new(Config::default());
    let accel = [Fix::ZERO, Fix::ZERO, Fix::from_num(8192)]; // +Z gravity, arbitrary magnitude
    let gyro = [Fix::ZERO; 3];

    let mut last = Output::default();
    for _ in 0..1000 {
        last = m.update(gyro, accel);
    }

    assert!(
        last.pitch_deg.to_num::<f64>().abs() < DEG_TOL,
        "pitch {}",
        last.pitch_deg
    );
    assert!(
        last.roll_deg.to_num::<f64>().abs() < DEG_TOL,
        "roll {}",
        last.roll_deg
    );

    let q = out_f64(&m);
    assert!((q[0].abs() - 1.0).abs() < Q_TOL, "q0 {}", q[0]);
    assert!(q[1].abs() < Q_TOL && q[2].abs() < Q_TOL && q[3].abs() < Q_TOL);
}

// ---------------------------------------------------------------------------------------------
// Known orientations (spec test 4): a held tilt about each tilt axis walks the matching fused
// angle to the true tilt with the recovered (negative-scale) sign, and the other angle stays 0.
// ---------------------------------------------------------------------------------------------

#[test]
fn pitched_sensor_fused_pitch_matches_reference_and_sign() {
    // Gravity leaning toward +X = a tilt about the pitch (Y) axis. Fused pitch walks to -20 deg
    // (the negative channel scale); fused roll stays ~0. Matches the f64 reference.
    let deg = 20.0_f64;
    let accel = tilted_accel(deg, 0);
    let accel_ref = tilted_accel_ref(deg, 0);

    let mut m = Mahony::new(Config::default());
    let mut r = RefMahony::new(RefConfig::default());
    let mut last = Output::default();
    let mut rp = (0.0, 0.0);
    for _ in 0..2000 {
        last = m.update([Fix::ZERO; 3], accel);
        rp = r.update([0.0; 3], accel_ref);
    }

    let pitch = last.pitch_deg.to_num::<f64>();
    assert!(
        (pitch - (-deg)).abs() < DEG_TOL,
        "pitch {pitch} expected {}",
        -deg
    );
    assert!((pitch - rp.0).abs() < DEG_TOL, "pitch {pitch} ref {}", rp.0);
    let roll = last.roll_deg.to_num::<f64>();
    assert!(roll.abs() < DEG_TOL, "roll {roll} expected ~0");
    assert!((roll - rp.1).abs() < DEG_TOL, "roll {roll} ref {}", rp.1);
}

#[test]
fn rolled_sensor_fused_roll_matches_reference_and_sign() {
    // Gravity leaning toward +Y = a tilt about the roll (X) axis. The fused ZYX roll
    // atan2(vy, vz) walks to the true tilt with the recovered negative scale: -20 deg. Fused
    // pitch stays ~0. This pins the spec's sign/convention obligation for the upgraded roll.
    let deg = 20.0_f64;
    let accel = tilted_accel(deg, 1);
    let accel_ref = tilted_accel_ref(deg, 1);

    let mut m = Mahony::new(Config::default());
    let mut r = RefMahony::new(RefConfig::default());
    let mut last = Output::default();
    let mut rp = (0.0, 0.0);
    for _ in 0..2000 {
        last = m.update([Fix::ZERO; 3], accel);
        rp = r.update([0.0; 3], accel_ref);
    }

    let roll = last.roll_deg.to_num::<f64>();
    assert!(
        (roll - (-deg)).abs() < DEG_TOL,
        "roll {roll} expected {}",
        -deg
    );
    assert!((roll - rp.1).abs() < DEG_TOL, "roll {roll} ref {}", rp.1);
    let pitch = last.pitch_deg.to_num::<f64>();
    assert!(pitch.abs() < DEG_TOL, "pitch {pitch} expected ~0");
    assert!((pitch - rp.0).abs() < DEG_TOL, "pitch {pitch} ref {}", rp.0);
}

// ---------------------------------------------------------------------------------------------
// Known gyro rate integrates to the expected angle over N ticks (gyro-only, no accel).
// ---------------------------------------------------------------------------------------------

#[test]
fn gyro_rate_integrates_to_expected_angle() {
    // Pure rotation about body-Y at a fixed rate, accel zero (so no correction): the quaternion
    // integrates the rate. Check the quaternion against the f64 reference (no trig -> tight band).
    let rate_rad_s = 1.0_f64; // 1 rad/s about Y
    let raw = (rate_rad_s / GYRO_SCALE).round() as i32;

    let mut m = Mahony::new(Config::default());
    let mut r = RefMahony::new(RefConfig::default());

    let n = 250; // 1 second at 250 Hz
    for _ in 0..n {
        let gy_fix = m.gyro_to_rad(1, raw);
        let gy_ref = r.gyro_to_rad(1, raw);
        let _ = m.update([Fix::ZERO, gy_fix, Fix::ZERO], [Fix::ZERO; 3]);
        let _ = r.update([0.0, gy_ref, 0.0], [0.0; 3]);
    }

    let qf = out_f64(&m);
    for (i, &qr) in r.q.iter().enumerate() {
        assert!(
            (qf[i] - qr).abs() < Q_TOL,
            "q[{}] fix {} ref {}",
            i,
            qf[i],
            qr
        );
    }

    // Sanity: a 1 rad/s rotation for 1 s about Y rotates ~1 rad = ~57.3 deg. The half-angle
    // quaternion q2 ~ sin(0.5) = 0.479. Confirm we actually rotated.
    assert!(qf[2].abs() > 0.4, "expected rotation, q2 = {}", qf[2]);
}

// ---------------------------------------------------------------------------------------------
// Renormalization keeps the quaternion unit.
// ---------------------------------------------------------------------------------------------

#[test]
fn renormalization_keeps_unit_quaternion() {
    // Drive with a vigorous changing gyro and accel; the norm must stay ~1 every tick.
    let mut m = Mahony::new(Config::default());
    for k in 0..3000_i32 {
        let raw = ((k % 200) - 100) * 30;
        let gx = m.gyro_to_rad(0, raw);
        let gy = m.gyro_to_rad(1, -raw);
        let gz = m.gyro_to_rad(2, raw / 2);
        let accel = [
            Fix::from_num(((k % 17) - 8) * 1000),
            Fix::from_num(((k % 13) - 6) * 1000),
            Fix::from_num(8000),
        ];
        m.update([gx, gy, gz], accel);

        let q = out_f64(&m);
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((norm - 1.0).abs() < 1e-3, "norm {} at k {}", norm, k);
    }
}

// ---------------------------------------------------------------------------------------------
// Output IIR smooths as designed (0.1 / 0.9-class blend) on both channels.
// ---------------------------------------------------------------------------------------------

#[test]
fn output_iir_smooths_a_one_tick_step() {
    // With the fused extraction, the raw angle can only jump when the QUATERNION jumps, so step it
    // with one large single-tick gyro impulse about X (test-only rate; the filter takes rad/s
    // directly). One integration step rotates ~2*h*|w| rad; the published roll must move only
    // ~10% of that raw jump on the first sample (the 0.1 new-weight), then converge back.
    let mut m = Mahony::new(Config::default());
    let mut r = RefMahony::new(RefConfig::default());

    // Settle level so both IIRs are primed near 0.
    let level = [Fix::ZERO, Fix::ZERO, Fix::from_num(16384)];
    let level_ref = [0.0, 0.0, 16384.0];
    for _ in 0..200 {
        m.update([Fix::ZERO; 3], level);
        r.update([0.0; 3], level_ref);
    }

    // One-tick impulse: w = 50 rad/s about X rotates ~2*0.002*50 = 0.2 rad ~ 11.5 deg this tick
    // (gyro-only tick so the accel correction does not fight the impulse).
    let w = 50.0;
    let of = m.update([Fix::from_num(w), Fix::ZERO, Fix::ZERO], [Fix::ZERO; 3]);
    let (rp, rr) = r.update([w, 0.0, 0.0], [0.0; 3]);

    // The raw quaternion roll jumped ~11.5 deg; the published (IIR'd) roll must be ~10% of it.
    let roll = of.roll_deg.to_num::<f64>();
    assert!((roll - rr).abs() < DEG_TOL, "roll {roll} ref {rr}");
    assert!(roll.abs() > 0.4, "roll should move: {roll}");
    assert!(
        roll.abs() < 4.0,
        "IIR failed to smooth: {roll} (raw jump ~11.5)"
    );
    let pitch = of.pitch_deg.to_num::<f64>();
    assert!((pitch - rp).abs() < DEG_TOL);

    // Impulse gone, gravity restored: it converges back toward level over many ticks.
    let mut last = of;
    for _ in 0..2000 {
        last = m.update([Fix::ZERO; 3], level);
        r.update([0.0; 3], level_ref);
    }
    assert!(last.roll_deg.to_num::<f64>().abs() < DEG_TOL);
}

// ---------------------------------------------------------------------------------------------
// Axis-sign correctness: flipping a gyro sign inverts the integrated rotation.
// ---------------------------------------------------------------------------------------------

#[test]
fn gyro_sign_inverts_rotation() {
    let raw = (0.5 / GYRO_SCALE).round() as i32; // 0.5 rad/s about Y

    let cfg_pos = Config {
        gyro_sign: [1, 1, 1],
        ..Config::default()
    };
    let cfg_neg = Config {
        gyro_sign: [1, -1, 1], // flip Y
        ..Config::default()
    };

    let mut mp = Mahony::new(cfg_pos);
    let mut mn = Mahony::new(cfg_neg);

    for _ in 0..250 {
        let gp = mp.gyro_to_rad(1, raw);
        let gn = mn.gyro_to_rad(1, raw);
        mp.update([Fix::ZERO, gp, Fix::ZERO], [Fix::ZERO; 3]);
        mn.update([Fix::ZERO, gn, Fix::ZERO], [Fix::ZERO; 3]);
    }

    let qp = out_f64(&mp);
    let qn = out_f64(&mn);
    // The Y-rotation component q2 must have opposite sign.
    assert!(
        qp[2] * qn[2] < 0.0,
        "signs did not invert: {} {}",
        qp[2],
        qn[2]
    );
    assert!(
        (qp[2] + qn[2]).abs() < Q_TOL,
        "magnitudes differ: {} {}",
        qp[2],
        qn[2]
    );
}

// ---------------------------------------------------------------------------------------------
// The core spec validation: fixed pitch AND roll track the f64 reference over a combined stream.
// ---------------------------------------------------------------------------------------------

#[test]
fn pitch_and_roll_match_reference_over_stream() {
    // Drive a combined gyro + accel stream (a slow lean about Y with gravity present, so the
    // angles are genuine fused angles) and compare the fixed pitch/roll to the f64 reference
    // every tick.
    let mut m = Mahony::new(Config::default());
    let mut r = RefMahony::new(RefConfig::default());

    for k in 0..1500_i32 {
        let raw_gy = if k < 400 {
            (0.3 / GYRO_SCALE).round() as i32
        } else {
            0
        };
        // Tilt the accel vector along +X over time to mimic the physical lean.
        let frac = ((k.min(400) as f64) / 400.0) * 25.0_f64.to_radians();
        let ax = (frac.sin() * 16384.0).round();
        let az = (frac.cos() * 16384.0).round();

        let gy_fix = m.gyro_to_rad(1, raw_gy);
        let gy_ref = r.gyro_to_rad(1, raw_gy);

        let accel_fix = [Fix::from_num(ax), Fix::ZERO, Fix::from_num(az)];
        let accel_ref = [ax, 0.0, az];

        let of = m.update([Fix::ZERO, gy_fix, Fix::ZERO], accel_fix);
        let (rp, rr) = r.update([0.0, gy_ref, 0.0], accel_ref);

        if k > 100 {
            assert!(
                (of.pitch_deg.to_num::<f64>() - rp).abs() < DEG_TOL,
                "pitch fix {} ref {} at k {}",
                of.pitch_deg,
                rp,
                k
            );
            assert!(
                (of.roll_deg.to_num::<f64>() - rr).abs() < DEG_TOL,
                "roll fix {} ref {} at k {}",
                of.roll_deg,
                rr,
                k
            );
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Edge case: zero accel vector integrates gyro only; both angles still extract (no NaN).
// ---------------------------------------------------------------------------------------------

#[test]
fn zero_accel_integrates_gyro_only() {
    // A pure X rotation with NO accel: the quaternion integrates gyro alone, and the fused roll
    // is still extracted from it (the extraction has no accel dependence). 0.2 rad/s for 1 s
    // ~ 11.46 deg of body-X roll; the published roll (negative scale, IIR-lagged) tracks the
    // reference.
    let mut m = Mahony::new(Config::default());
    let mut r = RefMahony::new(RefConfig::default());
    let raw = (0.2 / GYRO_SCALE).round() as i32;
    let mut of = Output::default();
    let mut rr = (0.0, 0.0);
    for _ in 0..250 {
        let gx = m.gyro_to_rad(0, raw);
        of = m.update([gx, Fix::ZERO, Fix::ZERO], [Fix::ZERO; 3]);
        rr = r.update([r.gyro_to_rad(0, raw), 0.0, 0.0], [0.0; 3]);

        let q = out_f64(&m);
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((norm - 1.0).abs() < 1e-3);
    }
    // X rotation accumulated...
    let q = out_f64(&m);
    assert!(q[1].abs() > 0.05, "expected X rotation, q1 {}", q[1]);
    // ...and the roll extraction saw it, matching the reference, with the negative channel scale.
    let roll = of.roll_deg.to_num::<f64>();
    assert!((roll - rr.1).abs() < DEG_TOL, "roll {roll} ref {}", rr.1);
    assert!(roll < -5.0, "roll should be well negative by 1 s: {roll}");
}

// ---------------------------------------------------------------------------------------------
// Pole behavior: tilt through 90 deg; asin clamp + the atan2 (0,0) guard give finite results.
// ---------------------------------------------------------------------------------------------

#[test]
fn pole_behavior_no_blowup() {
    // ax = |a| exactly -> the estimate converges toward pitch = -90 deg (the asin clamp's edge,
    // and the roll atan2's (0, 0) pole). Everything must stay finite; pitch approaches -90.
    let accel = [Fix::from_num(16384), Fix::ZERO, Fix::ZERO];
    let mut m = Mahony::new(Config::default());
    let mut last = Output::default();
    for _ in 0..4000 {
        last = m.update([Fix::ZERO; 3], accel);
        let q = out_f64(&m);
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((norm - 1.0).abs() < 1e-3);
    }
    let pitch = last.pitch_deg.to_num::<f64>();
    let roll = last.roll_deg.to_num::<f64>();
    assert!((-91.0..=-80.0).contains(&pitch), "pitch at pole {pitch}");
    assert!(
        (-180.5..=180.5).contains(&roll),
        "roll at pole must stay finite/in range: {roll}"
    );
}

// ---------------------------------------------------------------------------------------------
// Convergence from a wrong start (identity) to a fixed tilt.
// ---------------------------------------------------------------------------------------------

#[test]
fn converges_from_wrong_start() {
    // Hold a fixed +15 deg tilt about Y (accel leans along +X). The fused pitch should walk from 0
    // toward the tilt and stay; the fused roll stays ~0. Compare to the f64 reference at the end.
    let deg = 15.0_f64;
    let accel_fix = tilted_accel(deg, 0);
    let accel_ref = tilted_accel_ref(deg, 0);

    let mut m = Mahony::new(Config::default());
    let mut r = RefMahony::new(RefConfig::default());

    let mut of = Output::default();
    let mut rp = (0.0, 0.0);
    for _ in 0..4000 {
        of = m.update([Fix::ZERO; 3], accel_fix);
        rp = r.update([0.0, 0.0, 0.0], accel_ref);
    }
    // Pitch converged to -15 deg and matches the reference; roll stays level.
    assert!(
        (of.pitch_deg.to_num::<f64>() - rp.0).abs() < DEG_TOL,
        "pitch {} ref {}",
        of.pitch_deg,
        rp.0
    );
    assert!(
        (of.pitch_deg.to_num::<f64>() - (-deg)).abs() < DEG_TOL,
        "pitch {}",
        of.pitch_deg
    );
    assert!(
        (of.roll_deg.to_num::<f64>() - rp.1).abs() < DEG_TOL,
        "roll {} ref {}",
        of.roll_deg,
        rp.1
    );
}

// ---------------------------------------------------------------------------------------------
// Level trims subtract from the published angles.
// ---------------------------------------------------------------------------------------------

#[test]
fn trims_subtract_from_published_angles() {
    let cfg = Config {
        pitch_trim_deg: Out::from_num(1.5),
        roll_trim_deg: Out::from_num(-2.0),
        ..Config::default()
    };
    let mut trimmed = Mahony::new(cfg);
    let mut plain = Mahony::new(Config::default());

    let accel = [Fix::ZERO, Fix::ZERO, Fix::from_num(16384)];
    let mut ot = Output::default();
    let mut op = Output::default();
    for _ in 0..500 {
        ot = trimmed.update([Fix::ZERO; 3], accel);
        op = plain.update([Fix::ZERO; 3], accel);
    }
    let dp = (op.pitch_deg - ot.pitch_deg).to_num::<f64>();
    let dr = (op.roll_deg - ot.roll_deg).to_num::<f64>();
    assert!((dp - 1.5).abs() < 1e-3, "pitch trim delta {dp}");
    assert!((dr - (-2.0)).abs() < 1e-3, "roll trim delta {dr}");
}

// ---------------------------------------------------------------------------------------------
// Pre-filter IIR parity (spec test 7): driven through imu::Iir, the single owner, so the attitude
// tests condition input exactly as the assembled system does.
// ---------------------------------------------------------------------------------------------

#[test]
fn prefilter_iir_tracks_and_uses_exact_coeffs() {
    use imu::{Iir, IIR_FAST_NEW, IIR_FAST_OLD};

    let mut fast = Iir::fast();
    let target = Fix::from_num(100.0);

    // First sample primes to the input.
    assert_eq!(fast.step(target), target);

    // Reset and compare step-by-step against the f64 IIR with the exact coefficients.
    let mut fast = Iir::fast();
    let mut yref = 0.0_f64;
    let mut primed = false;
    for k in 0..500 {
        let x = (k as f64) * 0.5;
        let yf = fast.step(Fix::from_num(x)).to_num::<f64>();
        if !primed {
            yref = x;
            primed = true;
        } else {
            yref = IIR_FAST_NEW * x + IIR_FAST_OLD * yref;
        }
        assert!(
            (yf - yref).abs() < 1e-3,
            "fast iir {} ref {} at k {}",
            yf,
            yref,
            k
        );
    }

    // Slow channel tracks a step more slowly than fast: fresh filters, primed at 0, then driven
    // toward a constant target. After equal driving the fast channel is closer (larger new_w).
    let mut fast = Iir::fast();
    let mut slow = Iir::slow();
    let zero = Fix::ZERO;
    fast.step(zero); // prime both at 0
    slow.step(zero);
    for _ in 0..50 {
        fast.step(target);
        slow.step(target);
    }
    let fdist = (fast.value().to_num::<f64>() - 100.0).abs();
    let sdist = (slow.value().to_num::<f64>() - 100.0).abs();
    assert!(
        fdist < sdist,
        "fast should be closer to target than slow: {} {}",
        fdist,
        sdist
    );
}

// --- dt honesty (round-4 defect B): the mechanism CHECK the audit asked for --------------------
//
// The check came back with a DISCONFIRMATION worth pinning: on a STILL board with the bench-
// captured bias, the settling point is dt-INDEPENDENT (at equilibrium the bias term and the Kp
// error term scale with the step identically, so the assumed dt cancels; both filters settle at
// the same ~0.2 deg). Therefore the audited +/-35-70 deg wander CANNOT be produced by the
// dt-mismatch + unstaged-bias pair alone; a further driver is required (leading candidate: torn
// burst samples while the I2C read is SCL-stretched across IMU sample updates under CPU
// starvation, which only occurs in the starved regimes where the wander was seen). What dt
// honesty DOES buy, and what these tests pin: (a) the clamps, (b) a large attitude error
// re-converges in the true wall-clock time (the honest filter takes real-sized steps per run,
// the dishonest one crawls at 1/real_ticks of the correct speed per wall second).

/// One simulated still-board tick: gravity-only accel, gyro = raw bias counts (a still board's
/// zero-rate output IS the bias), fed through the sign map exactly as the imu crate would.
fn still_inputs(bias_counts: [i32; 3]) -> ([Fix; 3], [Fix; 3]) {
    let gyro = [
        Fix::from_num(bias_counts[0] as f64 * GYRO_SCALE),
        Fix::from_num(bias_counts[1] as f64 * GYRO_SCALE),
        Fix::from_num(bias_counts[2] as f64 * GYRO_SCALE),
    ];
    // Flat board: gravity along +Z in counts (8192 = 1 g at the imu crate's +-4 g scale).
    let accel = [Fix::ZERO, Fix::ZERO, Fix::from_num(8192)];
    (gyro, accel)
}

#[test]
fn still_board_equilibrium_is_dt_independent_the_disconfirmation() {
    let bias = [48, 13, -88]; // the 2026-07-18 bench capture, raw counts
    let (gyro, accel) = still_inputs(bias);
    let real_ticks = 10; // control running at 25 Hz: the audited flooded regime

    let mut dishonest = Mahony::new(Config::default());
    let mut honest = Mahony::new(Config::default());
    let mut pitch_dishonest = 0.0f64;
    let mut pitch_honest = 0.0f64;
    for _ in 0..2_000 {
        pitch_dishonest = dishonest.update(gyro, accel).pitch_deg.to_num::<f64>();
        pitch_honest = honest
            .update_dt(gyro, accel, real_ticks)
            .pitch_deg
            .to_num::<f64>();
    }
    // Both settle near truth, and NEAR EACH OTHER: the equilibrium is dt-independent, so the
    // audited large wander needs a driver beyond dt + bias (see the module comment).
    assert!(pitch_honest.abs() < 1.5, "honest equilibrium near truth");
    assert!(
        (pitch_dishonest - pitch_honest).abs() < 0.5,
        "equilibria agree (dt cancels at the still-board fixed point): {pitch_dishonest} vs {pitch_honest}"
    );
}

#[test]
fn dt_honest_reconverges_a_large_error_in_true_wall_time() {
    // Start both filters with a LARGE attitude error (feed a tilted gravity long enough to
    // converge there, then flip the board flat) and count RUNS to re-converge at 25 Hz real
    // rate. The honest filter takes real-sized steps, so it needs ~real_ticks-fold FEWER runs
    // (= the same wall time as a healthy 250 Hz filter); the dishonest one crawls.
    let bias = [48, 13, -88];
    let (gyro, flat) = still_inputs(bias);
    let tilted = [Fix::from_num(4096), Fix::ZERO, Fix::from_num(7094)]; // ~30 deg
    let real_ticks = 10;

    let mut honest = Mahony::new(Config::default());
    let mut dishonest = Mahony::new(Config::default());
    for _ in 0..4_000 {
        let _ = honest.update_dt(gyro, tilted, real_ticks);
        let _ = dishonest.update(gyro, tilted);
    }

    let runs_to_flat = |m: &mut Mahony, dt: u32| -> u32 {
        for run in 1..20_000u32 {
            let p = m.update_dt(gyro, flat, dt).pitch_deg.to_num::<f64>();
            if p.abs() < 2.0 {
                return run;
            }
        }
        20_000
    };
    let honest_runs = runs_to_flat(&mut honest, real_ticks);
    let dishonest_runs = runs_to_flat(&mut dishonest, 1);
    assert!(
        dishonest_runs > 4 * honest_runs,
        "honest dt re-converges in ~1/real_ticks the runs (same wall time as at-rate): honest {honest_runs} vs dishonest {dishonest_runs}"
    );
}

#[test]
fn update_dt_clamps_degenerate_and_stale_gaps() {
    let (gyro, accel) = still_inputs([48, 13, -88]);
    let mut a = Mahony::new(Config::default());
    let mut b = Mahony::new(Config::default());
    // dt = 0 behaves as 1 (a run implies time passed).
    let pa = a.update_dt(gyro, accel, 0).pitch_deg.to_num::<f64>();
    let pb = b.update_dt(gyro, accel, 1).pitch_deg.to_num::<f64>();
    assert!((pa - pb).abs() < 1e-9, "dt=0 clamps to 1");
    // dt beyond MAX_DT_TICKS behaves as MAX_DT_TICKS.
    let mut c = Mahony::new(Config::default());
    let mut d = Mahony::new(Config::default());
    let pc = c.update_dt(gyro, accel, 10_000).pitch_deg.to_num::<f64>();
    let pd = d
        .update_dt(gyro, accel, MAX_DT_TICKS)
        .pitch_deg
        .to_num::<f64>();
    assert!((pc - pd).abs() < 1e-9, "dt clamps to MAX_DT_TICKS");
}
