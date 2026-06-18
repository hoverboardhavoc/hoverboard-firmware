//! Host tests for the fixed-point Mahony filter.
//!
//! The required validation (spec section 7) is: the fixed-point implementation tracks a
//! double-precision reference fed the identical input stream, within a documented tolerance band
//! that absorbs the float-to-Q quantization. The reference below recomputes every spec formula in
//! f64 (gyro scale, h = 0.002, the cross-product/quaternion structure, renormalization, the
//! section-6 degree conversion and signs, the 0.1/0.9 and 0.1/0.8999997615814209 output IIR). Tests
//! may use `std`/`f64`; the library itself is `no_std` fixed-point.

use super::*;

// ---------------------------------------------------------------------------------------------
// Double-precision reference (the spec formulas, recomputed in f64).
// ---------------------------------------------------------------------------------------------

struct RefConfig {
    kp: f64,
    gyro_bias: [f64; 3],
    gyro_sign: [f64; 3],
    pitch_trim_deg: f64,
    heading_trim_deg: f64,
}

impl Default for RefConfig {
    fn default() -> Self {
        RefConfig {
            kp: 1.0,
            gyro_bias: [0.0; 3],
            gyro_sign: [1.0; 3],
            pitch_trim_deg: 0.0,
            heading_trim_deg: 0.0,
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

    /// One step in f64, matching the spec operation order exactly.
    fn update(&mut self, gyro: [f64; 3], accel: [f64; 3]) -> (f64, f64, f64) {
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
            let vx = 2.0 * (q1 * q3 - q0 * q2);
            let vy = 2.0 * (q0 * q1 + q2 * q3);
            let vz = (q0 * q0 - q1 * q1 - q2 * q2) + q3 * q3;

            let ex = ahat[1] * vz - ahat[2] * vy;
            let ey = ahat[2] * vx - ahat[0] * vz;
            let ez = ahat[0] * vy - ahat[1] * vx;

            wx += self.cfg.kp * ex;
            wy += self.cfg.kp * ey;
            wz += self.cfg.kp * ez;
        }

        let dq0 = -q1 * wx - q2 * wy - q3 * wz;
        let dq1 = q0 * wx + q2 * wz - q3 * wy;
        let dq2 = q0 * wy - q1 * wz + q3 * wx;
        let dq3 = q0 * wz + q1 * wy - q2 * wx;

        let h = HALF_STEP;
        let mut nq = [q0 + h * dq0, q1 + h * dq1, q2 + h * dq2, q3 + h * dq3];

        let norm2 = nq[0] * nq[0] + nq[1] * nq[1] + nq[2] * nq[2] + nq[3] * nq[3];
        if norm2 != 0.0 {
            let norm = norm2.sqrt();
            if norm != 0.0 {
                nq = [nq[0] / norm, nq[1] / norm, nq[2] / norm, nq[3] / norm];
                self.q = nq;
            }
        }

        let [nq0, nq1, nq2, nq3] = self.q;

        let pitch_arg = clamp_f64(2.0 * (nq1 * nq3 - nq0 * nq2));
        let pitch_deg_raw = -(pitch_arg.asin() * RAD_TO_DEG);

        let (roll_deg_raw, heading_deg_raw) = if have_accel {
            let roll = -(clamp_f64(ahat[0]).asin() * RAD_TO_DEG);
            let heading = clamp_f64(ahat[1]).asin() * RAD_TO_DEG;
            (roll, heading)
        } else {
            (0.0, 0.0)
        };

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
            roll_smoothed,
            heading_deg_raw - self.cfg.heading_trim_deg,
        )
    }
}

fn clamp_f64(x: f64) -> f64 {
    if x > 1.0 {
        1.0
    } else if x < -1.0 {
        -1.0
    } else {
        x
    }
}

// ---------------------------------------------------------------------------------------------
// Tolerances. The dominant error source is cordic asin (~0.01 rad absolute per its own test suite),
// which at the 57.29578 deg/rad scale is up to ~0.6 deg on a single extracted angle. The output IIR
// then attenuates per-step error. Q quantization of the body (I32F32) is ~1e-7, negligible beside
// the trig error. These bands are sized to absorb the cordic asin error, not to hide a formula
// mismatch (a sign or order error blows past them by tens of degrees).
// ---------------------------------------------------------------------------------------------

/// Degree tolerance for a published angle (dominated by cordic asin error through the 57.29578
/// scale).
const DEG_TOL: f64 = 0.8;
/// Quaternion-component tolerance (body Q quantization + accumulated asin-free integration error).
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

// ---------------------------------------------------------------------------------------------
// Constant-reproduction tests: the Q constants match the decimal targets within the format's
// resolution (spec section 5.1: reproduce decimals, not bit patterns).
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
    // The b roll coefficient is specifically NOT 0.9 (spec section 6.2): 0.8999997615814209.
    assert!((OUT_IIR_PREV_ROLL - 0.9).abs() > 1e-7);
}

#[test]
fn gyro_scale_is_full_scale_relation() {
    // 0.000266316114 == (500/32768) * (pi/180) (spec section 3).
    let expect = (500.0 / 32768.0) * (core::f64::consts::PI / 180.0);
    assert!((GYRO_SCALE - expect).abs() < 1e-9);
}

#[test]
fn half_step_relation() {
    // h = 0.5 * dt, dt = 1/250 = 0.004 (spec section 2/5).
    let dt = 1.0 / 250.0;
    assert!((HALF_STEP - 0.5 * dt).abs() < 1e-12);
    assert!((dt - 0.004).abs() < 1e-12);
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

    // ax = ay = 0 -> roll and heading inclinations are 0. Pitch fused angle stays 0.
    assert!(last.pitch_deg.to_num::<f64>().abs() < DEG_TOL, "pitch {}", last.pitch_deg);
    assert!(last.roll_deg.to_num::<f64>().abs() < DEG_TOL, "roll {}", last.roll_deg);
    assert!(last.heading_deg.to_num::<f64>().abs() < DEG_TOL, "heading {}", last.heading_deg);

    let q = out_f64(&m);
    assert!((q[0].abs() - 1.0).abs() < Q_TOL, "q0 {}", q[0]);
    assert!(q[1].abs() < Q_TOL && q[2].abs() < Q_TOL && q[3].abs() < Q_TOL);
}

// ---------------------------------------------------------------------------------------------
// Tilted still sensor: roll/heading accel inclinations match asin of the tilt.
// ---------------------------------------------------------------------------------------------

#[test]
fn tilted_sensor_accel_inclinations_match_reference() {
    // Tilt so ax/|a| = sin(20 deg). Roll (body-X inclination) should read -20 deg, heading 0.
    let deg = 20.0_f64;
    let ax = (deg.to_radians().sin() * 16384.0).round();
    let az = (deg.to_radians().cos() * 16384.0).round();
    let accel = [Fix::from_num(ax), Fix::ZERO, Fix::from_num(az)];
    let gyro = [Fix::ZERO; 3];

    let mut m = Mahony::new(Config::default());
    let mut last = Output::default();
    for _ in 0..2000 {
        last = m.update(gyro, accel);
    }

    // Roll formula is -asin(ax/|a|) * 57.29578 -> approximately -20 deg.
    assert!(
        (last.roll_deg.to_num::<f64>() - (-deg)).abs() < DEG_TOL,
        "roll {} expected {}",
        last.roll_deg,
        -deg
    );
    assert!(last.heading_deg.to_num::<f64>().abs() < DEG_TOL, "heading {}", last.heading_deg);
}

// ---------------------------------------------------------------------------------------------
// Known gyro rate integrates to the expected angle over N ticks (gyro-only, no accel).
// ---------------------------------------------------------------------------------------------

#[test]
fn gyro_rate_integrates_to_expected_angle() {
    // Pure rotation about body-Y at a fixed rate, accel zero (so no correction): the quaternion
    // integrates the rate. After t seconds the rotation angle is rate * t. With small-angle pitch
    // tracking 2*(q1q3 - q0q2), check the quaternion magnitude of the rotation against f64 reference.
    let rate_rad_s = 1.0_f64; // 1 rad/s about Y
    // Convert to the raw count that yields this rate, then back through gyro_to_rad for parity.
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

    // Compare the fixed quaternion to the f64 reference quaternion (no asin involved -> tight band).
    let qf = out_f64(&m);
    for i in 0..4 {
        assert!(
            (qf[i] - r.q[i]).abs() < Q_TOL,
            "q[{}] fix {} ref {}",
            i,
            qf[i],
            r.q[i]
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
// Output IIR smooths as designed (0.1 / 0.9-class blend).
// ---------------------------------------------------------------------------------------------

#[test]
fn output_iir_smooths() {
    // Step the accel tilt from level to 30 deg in one tick and confirm the published roll does NOT
    // jump the full way: the 0.1/0.899... IIR limits the first step to ~10% of the change.
    let deg = 30.0_f64;
    let ax = (deg.to_radians().sin() * 16384.0).round();
    let az = (deg.to_radians().cos() * 16384.0).round();

    let mut m = Mahony::new(Config::default());
    // Prime at level for a while so prev settles near 0.
    let level = [Fix::ZERO, Fix::ZERO, Fix::from_num(16384)];
    for _ in 0..50 {
        m.update([Fix::ZERO; 3], level);
    }
    let before = m.update([Fix::ZERO; 3], level).roll_deg.to_num::<f64>();
    assert!(before.abs() < DEG_TOL);

    // First tilted sample: roll target is -30 deg, but blended output should be ~ -3 deg (10%).
    let tilted = [Fix::from_num(ax), Fix::ZERO, Fix::from_num(az)];
    let first = m.update([Fix::ZERO; 3], tilted).roll_deg.to_num::<f64>();
    // Expected ~ 0.1 * (-30) + 0.9 * 0 = -3. Must be far from the full -30.
    assert!(first < -1.0 && first > -8.0, "first blended roll {}", first);
    assert!(first.abs() < 8.0, "IIR failed to smooth: {}", first);

    // Many samples later it converges to ~ -30.
    let mut last = 0.0;
    for _ in 0..400 {
        last = m.update([Fix::ZERO; 3], tilted).roll_deg.to_num::<f64>();
    }
    assert!((last - (-deg)).abs() < DEG_TOL, "converged roll {}", last);
}

// ---------------------------------------------------------------------------------------------
// Axis-sign correctness: flipping a gyro sign inverts the integrated rotation.
// ---------------------------------------------------------------------------------------------

#[test]
fn gyro_sign_inverts_rotation() {
    let raw = (0.5 / GYRO_SCALE).round() as i32; // 0.5 rad/s about Y

    let mut cfg_pos = Config::default();
    cfg_pos.gyro_sign = [1, 1, 1];
    let mut cfg_neg = Config::default();
    cfg_neg.gyro_sign = [1, -1, 1]; // flip Y

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
    assert!(qp[2] * qn[2] < 0.0, "signs did not invert: {} {}", qp[2], qn[2]);
    assert!((qp[2] + qn[2]).abs() < Q_TOL, "magnitudes differ: {} {}", qp[2], qn[2]);
}

// ---------------------------------------------------------------------------------------------
// Pitch sign convention: forward lean yields negative pitch (the -57.29578 scale, spec section 6).
// ---------------------------------------------------------------------------------------------

#[test]
fn pitch_sign_matches_reference_over_stream() {
    // Drive a combined gyro + accel stream and compare the fixed pitch/roll/heading to the f64
    // reference every tick. This is the core spec validation: Q matches the f64 reference within
    // tolerance (spec section 7).
    let mut m = Mahony::new(Config::default());
    let mut r = RefMahony::new(RefConfig::default());

    // A slow tilt about Y while gravity is present, so pitch is a genuine fused angle.
    for k in 0..1500_i32 {
        let raw_gy = if k < 400 { (0.3 / GYRO_SCALE).round() as i32 } else { 0 };
        // Tilt the accel vector along +X over time to mimic the physical lean.
        let frac = ((k.min(400) as f64) / 400.0) * 25.0_f64.to_radians();
        let ax = (frac.sin() * 16384.0).round();
        let az = (frac.cos() * 16384.0).round();

        let gy_fix = m.gyro_to_rad(1, raw_gy);
        let gy_ref = r.gyro_to_rad(1, raw_gy);

        let accel_fix = [Fix::from_num(ax), Fix::ZERO, Fix::from_num(az)];
        let accel_ref = [ax, 0.0, az];

        let of = m.update([Fix::ZERO, gy_fix, Fix::ZERO], accel_fix);
        let (rp, rr, rh) = r.update([0.0, gy_ref, 0.0], accel_ref);

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
            assert!(
                (of.heading_deg.to_num::<f64>() - rh).abs() < DEG_TOL,
                "heading fix {} ref {} at k {}",
                of.heading_deg,
                rh,
                k
            );
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Edge case: zero accel vector integrates gyro only, no NaN / no divide-by-zero.
// ---------------------------------------------------------------------------------------------

#[test]
fn zero_accel_integrates_gyro_only() {
    let mut m = Mahony::new(Config::default());
    let raw = (0.2 / GYRO_SCALE).round() as i32;
    for _ in 0..250 {
        let gx = m.gyro_to_rad(0, raw);
        let out = m.update([gx, Fix::ZERO, Fix::ZERO], [Fix::ZERO; 3]);
        // No accel -> roll/heading inclinations are exactly 0 contribution; nothing blows up.
        let q = out_f64(&m);
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((norm - 1.0).abs() < 1e-3);
        let _ = out;
    }
    // X rotation accumulated.
    let q = out_f64(&m);
    assert!(q[1].abs() > 0.05, "expected X rotation, q1 {}", q[1]);
}

// ---------------------------------------------------------------------------------------------
// Pole behavior: tilt through 90 deg, asin clamp gives a finite result, no blow-up.
// ---------------------------------------------------------------------------------------------

#[test]
fn pole_behavior_no_blowup() {
    // ax = |a| exactly -> ax/|a| = 1 -> asin clamps to pi/2 -> roll = -90 deg, finite.
    let accel = [Fix::from_num(16384), Fix::ZERO, Fix::ZERO];
    let mut m = Mahony::new(Config::default());
    let mut last = Output::default();
    for _ in 0..500 {
        last = m.update([Fix::ZERO; 3], accel);
    }
    let roll = last.roll_deg.to_num::<f64>();
    assert!(roll.is_finite());
    assert!((roll - (-90.0)).abs() < DEG_TOL, "roll at pole {}", roll);
}

// ---------------------------------------------------------------------------------------------
// Convergence from a wrong start (identity) to a fixed tilt.
// ---------------------------------------------------------------------------------------------

#[test]
fn converges_from_wrong_start() {
    // Hold a fixed +15 deg tilt about Y (accel leans along +X). The fused pitch should walk from 0
    // toward the tilt and stay. Compare to the f64 reference at the end.
    let deg = 15.0_f64;
    let ax = (deg.to_radians().sin() * 16384.0).round();
    let az = (deg.to_radians().cos() * 16384.0).round();
    let accel_fix = [Fix::from_num(ax), Fix::ZERO, Fix::from_num(az)];
    let accel_ref = [ax, 0.0, az];

    let mut m = Mahony::new(Config::default());
    let mut r = RefMahony::new(RefConfig::default());

    let mut of = Output::default();
    let mut rp = (0.0, 0.0, 0.0);
    for _ in 0..4000 {
        of = m.update([Fix::ZERO; 3], accel_fix);
        rp = r.update([0.0, 0.0, 0.0], accel_ref);
    }
    // Pitch converged and matches the reference.
    assert!((of.pitch_deg.to_num::<f64>() - rp.0).abs() < DEG_TOL, "pitch {} ref {}", of.pitch_deg, rp.0);
    assert!((of.roll_deg.to_num::<f64>() - rp.1).abs() < DEG_TOL, "roll {} ref {}", of.roll_deg, rp.1);
    // Roll is the accel body-X inclination = -15 deg.
    assert!((of.roll_deg.to_num::<f64>() - (-deg)).abs() < DEG_TOL, "roll {}", of.roll_deg);
}

// ---------------------------------------------------------------------------------------------
// Pre-filter IIR: coefficients applied, value tracks toward input.
// ---------------------------------------------------------------------------------------------

#[test]
fn prefilter_iir_tracks_and_uses_exact_coeffs() {
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
        assert!((yf - yref).abs() < 1e-3, "fast iir {} ref {} at k {}", yf, yref, k);
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
    assert!(fdist < sdist, "fast should be closer to target than slow: {} {}", fdist, sdist);
}
