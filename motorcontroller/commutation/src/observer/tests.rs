//! Host tests of the flux observer (`specs/flux-observer.md`, "Host tests"): the `atan2` and the
//! square root, the observer on a plant with a turning rotor, and the verdict and its table.

extern crate std;
use std::println;
use std::vec::Vec;

use super::*;
use crate::foc::{foc_step, svpwm, DutyOrder, FocState, PhaseOffsets};
use crate::{Direction, MotorOutput, PhaseCmd};

// ---------------------------------------------------------------------------------------------
// atan2
// ---------------------------------------------------------------------------------------------

/// Degrees per angle unit.
const DEG: f64 = 360.0 / 65_536.0;

/// The true angle of the integer point `(x, y)` in angle units, `[0, 65536)`.
fn true_units(y: i64, x: i64) -> f64 {
    let a = (y as f64).atan2(x as f64);
    (a.rem_euclid(2.0 * core::f64::consts::PI)) / (2.0 * core::f64::consts::PI) * 65_536.0
}

/// **All 65,536 directions at three radii, against `f64::atan2`**: the worst error under 0.35
/// degrees, and the output never steps backwards as the direction turns (each step's wrapped
/// difference is non-negative). The third radius is past `2^17`, where the shared shift applies.
#[test]
fn atan2_tracks_f64_over_every_direction_at_three_radii() {
    for r in [1_000.0f64, 100_000.0, 3_000_000.0] {
        let mut worst = 0.0f64;
        let mut prev: Option<u16> = None;
        for k in 0..65_536u32 {
            let a = k as f64 / 65_536.0 * 2.0 * core::f64::consts::PI;
            let (x, y) = ((r * a.cos()).round() as i64, (r * a.sin()).round() as i64);
            let got = atan2_u16(y as i32, x as i32);
            let want = true_units(y, x);
            let err = ((got as f64 - want + 32_768.0).rem_euclid(65_536.0) - 32_768.0) * DEG;
            worst = worst.max(err.abs());
            if let Some(p) = prev {
                assert!(
                    (got.wrapping_sub(p) as i16) >= 0,
                    "radius {r}, direction {k}: {got} after {p}"
                );
            }
            prev = Some(got);
        }
        println!("atan2 at radius {r}: worst error {worst:.3} degrees");
        assert!(worst < 0.35, "radius {r}: worst error {worst} degrees");
    }
}

/// The axes and the diagonals are exact, at a small and a large radius, and the origin is 0.
#[test]
fn atan2_is_exact_at_the_eight_octant_boundaries() {
    assert_eq!(atan2_u16(0, 0), 0);
    for r in [1i32, 7, 1 << 16, 1 << 20, i32::MAX / 2] {
        let cases = [
            ((0, r), 0u16),
            ((r, r), 8192),
            ((r, 0), 16384),
            ((r, -r), 24576),
            ((0, -r), 32768),
            ((-r, -r), 40960),
            ((-r, 0), 49152),
            ((-r, r), 57344),
        ];
        for ((y, x), want) in cases {
            assert_eq!(atan2_u16(y, x), want, "({x}, {y})");
        }
    }
    // The extremes fold without overflow.
    assert_eq!(atan2_u16(0, i32::MIN), 32768);
    assert_eq!(atan2_u16(i32::MIN, 0), 49152);
}

/// Components above `2^17` take the shared right shift, which leaves the angle that of the same
/// pair halved.
#[test]
fn atan2_above_the_fold_reads_the_halved_pair() {
    let mut seed = 0x1234_5678u32;
    for _ in 0..10_000 {
        let (a, b) = (xorshift(&mut seed), xorshift(&mut seed));
        // Halves in [2^16, 2^17), so the doubled pair sits in [2^17, 2^18).
        let x = ((a % 65_536) as i32 + 65_536) * if a & 1 == 0 { 1 } else { -1 };
        let y = (b % 131_072) as i32 * if b & 2 == 0 { 1 } else { -1 };
        assert_eq!(atan2_u16(2 * y, 2 * x), atan2_u16(y, x), "({x}, {y})");
    }
}

fn xorshift(s: &mut u32) -> u32 {
    *s ^= *s << 13;
    *s ^= *s >> 17;
    *s ^= *s << 5;
    *s
}

fn xorshift64(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

// ---------------------------------------------------------------------------------------------
// isqrt
// ---------------------------------------------------------------------------------------------

/// `r` is the floor square root of `v`: `r^2 <= v < (r + 1)^2`, in `u128`, and within one of
/// `f64::sqrt`.
fn check_isqrt(v: u64) {
    let r = isqrt_u64(v) as u128;
    assert!(r * r <= v as u128, "isqrt({v}) = {r} too large");
    assert!((r + 1) * (r + 1) > v as u128, "isqrt({v}) = {r} too small");
    assert!(
        ((v as f64).sqrt() - r as f64).abs() <= 1.0,
        "isqrt({v}) = {r}"
    );
}

/// The edges, every square and square less one over the `u64` range at a stride, and 100,000
/// random values.
#[test]
fn isqrt_is_the_exact_floor() {
    assert_eq!(isqrt_u64(0), 0);
    assert_eq!(isqrt_u64(1), 1);
    assert_eq!(isqrt_u64(2), 1);
    assert_eq!(isqrt_u64(u64::MAX), u32::MAX);
    for v in [
        0u64,
        1,
        2,
        3,
        4,
        u64::MAX,
        u64::MAX - 1,
        1 << 62,
        (1 << 62) - 1,
    ] {
        check_isqrt(v);
    }
    let mut k = 1u64;
    while k <= u32::MAX as u64 {
        check_isqrt(k * k);
        check_isqrt(k * k - 1);
        check_isqrt(k * k + 1);
        k = k * 3 / 2 + 1;
    }
    check_isqrt((u32::MAX as u64) * (u32::MAX as u64));
    let mut seed = 0x9E37_79B9_7F4A_7C15u64;
    for _ in 0..100_000 {
        let v = xorshift64(&mut seed);
        check_isqrt(v >> (v % 64));
    }
}

// ---------------------------------------------------------------------------------------------
// The observer on a plant
// ---------------------------------------------------------------------------------------------

/// The plant of `specs/foc-torque.md` section 9 (`crate::tests`' `PLANT_*`): 0.24 ohm, 0.5 mH,
/// 31 V, 155 counts per amp.
const PLANT_R: f64 = 0.24;
const PLANT_L: f64 = 0.5e-3;
const PLANT_VBUS: f64 = 31.0;
const PLANT_CAL_A: f64 = 155.0;
const PLANT_T: f64 = 1.0 / 16_000.0;
/// The plant's flux linkage, volt-seconds (21,000 microvolt-seconds, `specs/flux-observer.md`).
const PLANT_FLUX: f64 = 0.021;
/// The dead time of the plant with one, as a fraction of the period (the staged 28 at
/// `DEAD_TIME_DTS_DIV` 2, 56 of the period's 4,500 ticks).
const PLANT_DEAD: f64 = 56.0 / 4500.0;
/// Sub-steps a period, so the back-EMF turns with the rotor inside a period.
const SUB: usize = 16;

/// A wye-connected three-phase R-L winding with a permanent-magnet flux on a rotor turning at a
/// constant electrical rate, read by the two sensed phases at the top of each centre-aligned
/// period and driven by the arm's duties from the period's end (the compares are preloaded and
/// update there), so the interval between two samples runs half on the duties before and half on
/// the ones the first sample produced, in the textbook frame: phase `k` at `k * 120`
/// degrees, the magnet's flux on phase `k` `lambda cos(theta_d - k 120)`.
struct Plant {
    i: [f64; 3],
    applied: [u16; 3],
    r: f64,
    dead: f64,
    /// The rotor's d axis, radians.
    theta_d: f64,
    omega: f64,
}

impl Plant {
    fn new(hz: f64, r: f64, dead: f64) -> Self {
        Plant {
            i: [0.0; 3],
            applied: [crate::MID_RAIL; 3],
            r,
            dead,
            theta_d: 0.3,
            omega: 2.0 * core::f64::consts::PI * hz,
        }
    }

    fn sense(&self) -> (i16, i16) {
        let c = |a: f64| (a * PLANT_CAL_A).round().clamp(-32767.0, 32767.0) as i16;
        (c(self.i[0]), c(self.i[1]))
    }

    /// The q axis in angle units: a quarter turn ahead of the magnet's flux.
    fn theta_q(&self) -> u16 {
        let u = (self.theta_d + core::f64::consts::FRAC_PI_2) / (2.0 * core::f64::consts::PI);
        (u.rem_euclid(1.0) * 65_536.0) as u32 as u16
    }

    fn step(&mut self, next: MotorOutput) {
        let dt = PLANT_T / SUB as f64;
        for s in 0..SUB {
            if s == SUB / 2 {
                self.applied = next.phases.map(|p| match p {
                    PhaseCmd::Drive(d) => d,
                    PhaseCmd::Float => crate::MID_RAIL,
                });
            }
            let v: [f64; 3] = core::array::from_fn(|k| {
                self.applied[k] as f64 / crate::ARR as f64 * PLANT_VBUS
                    - self.i[k].signum() * self.dead * PLANT_VBUS
            });
            let n = (v[0] + v[1] + v[2]) / 3.0;
            for (k, (i, v)) in self.i.iter_mut().zip(v).enumerate() {
                let ph = self.theta_d - k as f64 * 2.0 * core::f64::consts::PI / 3.0;
                let e = -self.omega * PLANT_FLUX * ph.sin();
                *i += dt / PLANT_L * (v - n - e - self.r * *i);
            }
            self.theta_d += self.omega * dt;
        }
    }
}

/// `kv` at the plant's bus.
fn plant_kv() -> i32 {
    (PLANT_VBUS * 1000.0 * 65_536.0 / KV_DEN as f64).round() as i32
}

/// The default parameters on the plant's calibration and dead time, with the lead table zeroed so
/// the window's mean is the raw error. The plant's q axis leads its flux by a quarter turn
/// (`Plant::theta_q`), which is the `Reverse` motor's convention on the bench (the frame note).
fn plant_params(dt_comp: bool) -> ObserverParams {
    ObserverParams {
        lead: [0; BINS],
        ..ObserverParams::derive(
            [240, 500, 0, 0],
            [2730, 1820, 64, 243, 0, 10, dt_comp as i16],
            PLANT_CAL_A as u16,
            56,
            Direction::Reverse,
        )
    }
}

/// One run: the arm at `iq_ref` on a rotor turning at `hz`, the observer beside it in the ISR's
/// order (this period's current, the voltage the period that just ended applied, this period's
/// `theta`), for `settle` periods and then `windows` closed windows.
fn run_observer(
    hz: f64,
    iq_ref: i32,
    plant_r: f64,
    dead: f64,
    p: &ObserverParams,
    settle: usize,
    windows: usize,
) -> Vec<ObserverWindow> {
    let mut st = FocState::new(
        PhaseOffsets::try_new(0x7FB8, 0x7DAE).unwrap(),
        DutyOrder::DIRECT,
        28_500,
    );
    let mut plant = Plant::new(hz, plant_r, dead);
    let mut obs = FluxObserver::new();
    let kv = plant_kv();
    let mut out = Vec::new();
    for n in 0..settle + windows * WINDOW as usize {
        let i = plant.sense();
        let theta = plant.theta_q();
        let prev = st.applied();
        let next = foc_step(&mut st, theta, i, iq_ref, true, false);
        obs.step(
            p,
            &ObserverInput {
                v: (prev.alpha, prev.beta),
                i,
                theta,
                kv,
                speed: 0,
                hall_fault: false,
                drive: true,
            },
        );
        plant.step(next);
        if (n + 1) % WINDOW as usize == 0 {
            let w = obs.close(p);
            if n >= settle {
                out.push(w);
            }
        }
    }
    out
}

/// The leak's phase lead at `hz` electrical, degrees, at shift 10.
fn leak_lead_deg(hz: f64) -> f64 {
    let fc = 16_000.0 / (2.0 * core::f64::consts::PI * 1024.0);
    (fc / hz).atan().to_degrees()
}

/// Five leak time constants at shift 10.
const SETTLE: usize = 5 * 1024;

/// One voltage count on the plant's bus, against the observer's scale: the SVPWM's own gain (the
/// largest leg excursion from the mean at alpha 32,767) over `V_bus / 56,754`. Printed, because
/// the flux magnitude carries it one for one.
#[test]
fn the_voltage_count_is_the_observers_scale() {
    let s = svpwm(32767, 0);
    let d = [s.base, s.c1, s.c2].map(|d| d as f64);
    let mean = (d[0] + d[1] + d[2]) / 3.0;
    let peak = d.iter().map(|x| (x - mean).abs()).fold(0.0, f64::max);
    let volts = peak / crate::ARR as f64 * PLANT_VBUS / 32767.0;
    let ratio = volts / (PLANT_VBUS / KV_DEN as f64);
    println!("one voltage count is {ratio:.4} of the observer's V_bus / 56754");
    assert!((ratio - 1.0).abs() < 0.01, "{ratio}");
}

/// **The observer on the plant** (`specs/flux-observer.md`, "Host tests"): at 20, 50, 100 and 150
/// Hz electrical, after five leak time constants, the angle error's window mean is the leak's
/// phase lead within 1 degree and its spread under 1 degree; the flux magnitude is within 3
/// percent of the set linkage from 50 Hz up.
#[test]
fn the_observer_tracks_the_plants_rotor_at_four_speeds() {
    let p = plant_params(true);
    for hz in [20.0, 50.0, 100.0, 150.0] {
        let ws = run_observer(hz, 300, PLANT_R, PLANT_DEAD, &p, SETTLE, 8);
        let lead = leak_lead_deg(hz);
        for w in &ws {
            let mean = w.mean as f64 * DEG;
            let spread = w.spread as f64 * DEG;
            let flux = w.flux as f64;
            println!(
                "{hz:5.0} Hz: mean {mean:6.2} deg (lead {lead:5.2}), spread {spread:5.2} deg, \
                 flux {flux:6.0} uV.s"
            );
            assert!(w.judged);
            assert!(
                (mean - lead).abs() < 1.0,
                "{hz} Hz: mean {mean}, lead {lead}"
            );
            assert!(spread < 1.0, "{hz} Hz: spread {spread}");
            if hz >= 50.0 {
                let rel = flux / (PLANT_FLUX * 1e6) - 1.0;
                assert!(rel.abs() < 0.03, "{hz} Hz: flux {flux}");
            }
        }
    }
}

/// **A wrong `R`** (the plant's doubled against the observer's), read at 50 Hz over a sweep of
/// `iq_ref`: printed, and pinned to what the arithmetic says it is. With the current on the q
/// axis, the drop the observer does not subtract, `dR i`, integrates into a vector a quarter turn
/// behind the current, along the magnet's flux (d), so it moves the flux magnitude, up by
/// `dR |i| / omega`, and leaves the angle where it was.
#[test]
fn a_wrong_r_reads_as_a_flux_magnitude_that_grows_with_current() {
    let p = plant_params(true);
    let hz = 50.0;
    let omega = 2.0 * core::f64::consts::PI * hz;
    let predicted =
        |iq_ref: i32| PLANT_FLUX * 1e6 + PLANT_R * (iq_ref as f64 / PLANT_CAL_A) / omega * 1e6;
    let mut flux = Vec::new();
    let mut mean = Vec::new();
    for iq_ref in [0, 600, 1200, 2400] {
        let ws = run_observer(hz, iq_ref, 2.0 * PLANT_R, PLANT_DEAD, &p, SETTLE, 4);
        let w = ws[ws.len() - 1];
        println!(
            "R doubled, {hz} Hz, iq_ref {iq_ref}: mean {:6.2} deg, spread {:5.2} deg, flux {} uV.s \
             (dR i / omega predicts {:.0})",
            w.mean as f64 * DEG,
            w.spread as f64 * DEG,
            w.flux,
            predicted(iq_ref)
        );
        flux.push(w.flux as f64);
        mean.push(w.mean as f64 * DEG);
    }
    // The magnitude grows with the current, monotonically, by the predicted amount within 5
    // percent of the linkage.
    for k in 1..flux.len() {
        assert!(flux[k] > flux[k - 1], "{flux:?}");
    }
    assert!(
        (flux[3] - predicted(2400)).abs() < 0.05 * PLANT_FLUX * 1e6,
        "{flux:?}"
    );
    // The angle stays inside a quarter of the default band at every current.
    for m in &mean {
        assert!((m - leak_lead_deg(hz)).abs() < 15.0 / 4.0, "{mean:?}");
    }
}

/// **A wrong `L`** (the observer's doubled against the plant's), at 50 Hz: the inductive flux the
/// observer subtracts in excess lies along the current, on q, perpendicular to the magnet's, so it
/// turns the angle by `atan(dL |i| / lambda)` against the rotation: the error mean moves away from
/// the lead as `iq_ref` grows and leaves the default band at the plant's full current.
#[test]
fn a_wrong_l_reads_as_an_angle_error_that_grows_with_current() {
    let p = ObserverParams {
        lead: [0; BINS],
        ..ObserverParams::derive(
            [240, 1000, 0, 0],
            [2730, 1820, 64, 243, 0, 10, 1],
            PLANT_CAL_A as u16,
            56,
            Direction::Reverse,
        )
    };
    let hz = 50.0;
    let mut mean = Vec::new();
    // 2,450 counts is the fleet's full scale (`firmware::motor::full_scale_counts`).
    for iq_ref in [0, 600, 1200, 2450] {
        let ws = run_observer(hz, iq_ref, PLANT_R, PLANT_DEAD, &p, SETTLE, 4);
        let w = ws[ws.len() - 1];
        let amps = iq_ref as f64 / PLANT_CAL_A;
        let predicted = leak_lead_deg(hz) - (500e-6 * amps / PLANT_FLUX).atan().to_degrees();
        println!(
            "L doubled, {hz} Hz, iq_ref {iq_ref}: mean {:6.2} deg (predicted {predicted:6.2}), \
             tracked {}",
            w.mean as f64 * DEG,
            w.tracked
        );
        mean.push((w.mean as f64 * DEG, w.tracked));
    }
    for k in 1..mean.len() {
        assert!(mean[k].0 < mean[k - 1].0, "{mean:?}");
    }
    assert!(mean[0].1, "{mean:?}");
    assert!(!mean[3].1 && mean[3].0 < -15.0, "{mean:?}");
}

/// **The dead-time compensation** against a plant with dead time, at 20 Hz: the error's spread
/// over a revolution, and the flux magnitude's departure from the linkage, are both larger with
/// `dt_comp` off than on.
#[test]
fn the_dead_time_compensation_reduces_the_error_at_20_hz() {
    let hz = 20.0;
    // A revolution at 20 Hz is 800 periods: thirteen windows.
    let run = |comp: bool| {
        let ws = run_observer(
            hz,
            600,
            PLANT_R,
            PLANT_DEAD,
            &plant_params(comp),
            SETTLE,
            13,
        );
        let worst = ws.iter().map(|w| w.spread).max().unwrap() as f64 * DEG;
        let mean_dev = ws
            .iter()
            .map(|w| (w.mean as f64 * DEG - leak_lead_deg(hz)).abs())
            .fold(0.0, f64::max);
        let flux = ws.iter().map(|w| w.flux as f64).sum::<f64>() / ws.len() as f64;
        (worst, mean_dev, (flux / (PLANT_FLUX * 1e6) - 1.0).abs())
    };
    let on = run(true);
    let off = run(false);
    println!("20 Hz with dead time: dt_comp on (spread, |mean - lead|, |flux error|) = {on:?}");
    println!("20 Hz with dead time: dt_comp off                                     = {off:?}");
    assert!(off.0 + off.1 > on.0 + on.1, "on {on:?}, off {off:?}");
    assert!(off.2 > on.2, "on {on:?}, off {off:?}");
}

/// A suspended period (no bus scale, or the bridge floated) integrates nothing, and a window with
/// one is not judged.
#[test]
fn a_suspended_period_leaves_the_window_unjudged() {
    let p = plant_params(true);
    let mut obs = FluxObserver::new();
    let x = ObserverInput {
        v: (1000, 0),
        i: (0, 0),
        theta: 0,
        kv: plant_kv(),
        speed: 5,
        hall_fault: false,
        drive: true,
    };
    for n in 0..WINDOW {
        let suspended = if n == 10 {
            ObserverInput { kv: 0, ..x }
        } else if n == 20 {
            ObserverInput { drive: false, ..x }
        } else {
            x
        };
        let before = obs.psi();
        obs.step(&p, &suspended);
        if n == 10 || n == 20 {
            assert_eq!(obs.psi(), before);
        }
    }
    let w = obs.close(&p);
    assert!(!w.judged && !w.tracked);
    assert_eq!(
        (w.mean, w.spread, w.flux),
        (0, 0, 0),
        "a short window publishes nothing"
    );
    assert_eq!(w.angle, ANGLE_90, "but its angles");
    // A window the observer did not run in reads all zero.
    assert_eq!(obs.close(&p), ObserverWindow::default());
}

// ---------------------------------------------------------------------------------------------
// The verdict
// ---------------------------------------------------------------------------------------------

/// A window of 64 periods whose observer angle sits at `errs[n]` units from `theta` (cycled):
/// the observer is driven to the error by choosing the window's `theta` against the angle it
/// returns. Returns the closed window.
fn window_of(p: &ObserverParams, errs: &[i16], hall_fault_at: Option<usize>) -> ObserverWindow {
    let mut obs = FluxObserver::new();
    for n in 0..WINDOW as usize {
        // A zero-voltage, zero-current period leaves the flux at zero, whose angle is a quarter
        // turn (the origin's 0 plus the d-to-q quarter); theta is placed to make the error.
        let angle = ANGLE_90;
        let theta = angle.wrapping_sub(errs[n % errs.len()] as u16);
        obs.step(
            p,
            &ObserverInput {
                v: (0, 0),
                i: (0, 0),
                theta,
                kv: 1000,
                speed: 0,
                hall_fault: hall_fault_at == Some(n),
                drive: true,
            },
        );
    }
    obs.close(p)
}

fn verdict_params() -> ObserverParams {
    ObserverParams::derive(
        [240, 500, 0, 0],
        [2730, 1820, 64, 243, 0, 10, 0],
        455,
        56,
        Direction::Reverse,
    )
}

/// The mean of errors either side of the half-turn wraps: errors at +179 and -179 degrees average
/// to the half-turn, not to zero.
#[test]
fn the_error_mean_wraps_across_the_half_turn() {
    let p = verdict_params();
    // +-179 degrees.
    let e = (179.0 / DEG) as i16;
    let w = window_of(&p, &[e, -e], None);
    // The per-period errors are e and -e as i16, so the sum is zero: the mean of a wrapped pair
    // straddling the half-turn is 0 and its spread is the full excursion, which fails the band.
    assert_eq!(w.mean, 0);
    assert!(w.spread >= e - 1);
    assert!(!w.tracked);
    // A pair on one side of zero: the mean is their midpoint.
    let w = window_of(&p, &[100, 300], None);
    assert_eq!(w.mean, 200);
    assert!(w.tracked);
    // The wrap of a single period's error: an angle a hair past the half-turn reads negative.
    let w = window_of(&p, &[i16::MIN + 5], None);
    assert_eq!(w.mean, i16::MIN + 5);
    // Across the angles' zero: an observer at 100 against a theta of 65,436 is 200 ahead, not
    // 65,336 behind (the offset moves the observer's quarter turn to 100).
    let p = ObserverParams {
        offset: ANGLE_90 - 100,
        ..verdict_params()
    };
    let mut obs = FluxObserver::new();
    for _ in 0..WINDOW {
        obs.step(
            &p,
            &ObserverInput {
                v: (0, 0),
                i: (0, 0),
                theta: 65_436,
                kv: 1000,
                speed: 0,
                hall_fault: false,
                drive: true,
            },
        );
    }
    assert_eq!(obs.close(&p).mean, 200);
}

/// A window with a hall-fault period is judged by nothing: not tracked, whatever its error.
#[test]
fn a_window_with_a_hall_fault_period_is_not_tracked() {
    let p = verdict_params();
    assert!(window_of(&p, &[0], None).tracked);
    let w = window_of(&p, &[0], Some(17));
    assert!(!w.judged && !w.tracked);
}

/// The spread is the larger excursion from the mean on either side: errors on both sides of the
/// mean, and errors all on one side.
#[test]
fn the_spread_is_the_larger_excursion_from_the_mean() {
    let p = verdict_params();
    // Both sides: -100 and +300 around a mean of 100 -> max(300 - 100, 100 + 100) = 200.
    let w = window_of(&p, &[-100, 300], None);
    assert_eq!((w.mean, w.spread), (100, 200));
    // One side: 63 periods at 0 and one at 640 -> mean 10, spread 630.
    let mut errs = [0i16; 64];
    errs[63] = 640;
    let w = window_of(&p, &errs, None);
    assert_eq!((w.mean, w.spread), (10, 630));
    // The spread saturates at i16 rather than wrapping.
    let w = window_of(&p, &[i16::MIN + 1, i16::MAX], None);
    assert_eq!(w.spread, i16::MAX);
    // The bands: inside both tracks, a mean or a spread past its band does not.
    assert!(window_of(&p, &[2730], None).tracked);
    assert!(!window_of(&p, &[2731], None).tracked);
    assert!(window_of(&p, &[-1820, 1820], None).tracked);
    assert!(!window_of(&p, &[-1821, 1821], None).tracked);
}

/// The leak's lead, taken off each period's error by the sign of the speed, against the table the
/// parameters carry: at bin 1 and shift 10 it is the spec's 16.7 degrees.
#[test]
fn the_lead_table_is_the_leaks_phase_lead_at_each_bins_speed() {
    let t = lead_table(10);
    assert_eq!(t[0], 0);
    let f_c = 16_000.0 / (2.0 * core::f64::consts::PI * 1024.0);
    for (k, &e) in t.iter().enumerate().skip(1) {
        let f = k as f64 * 16_000.0 / 1920.0;
        let want = (f_c / f).atan().to_degrees();
        assert!(
            (e as f64 * DEG - want).abs() < 0.35,
            "bin {k}: {}",
            e as f64 * DEG
        );
    }
    assert!(
        (t[1] as f64 * DEG - 16.6).abs() < 0.3,
        "{}",
        t[1] as f64 * DEG
    );
    // The parameters carry the table of their own shift.
    let p = ObserverParams::derive(
        [240, 500, 0, 0],
        [2730, 1820, 64, 243, 0, 12, 1],
        455,
        56,
        Direction::Reverse,
    );
    assert_eq!(p.lead, lead_table(12));
    // Subtracted by the sign of the speed: an error equal to the lead at +3 edges reads 0, and
    // the mirror at -3.
    let lead = p.lead[3];
    for (speed, err) in [(3, lead), (-3, 0u16.wrapping_sub(lead))] {
        let mut obs = FluxObserver::new();
        for _ in 0..WINDOW {
            obs.step(
                &p,
                &ObserverInput {
                    v: (0, 0),
                    i: (0, 0),
                    theta: ANGLE_90.wrapping_sub(err),
                    kv: 1000,
                    speed,
                    hall_fault: false,
                    drive: true,
                },
            );
        }
        assert_eq!(obs.close(&p).mean, 0, "speed {speed}");
    }
}

/// The parameters' clamps and scales.
#[test]
fn the_parameters_are_clamped_and_pre_scaled() {
    let p = ObserverParams::derive(
        [240, 500, 0, 0],
        [2730, 1820, 64, 243, 0, 10, 1],
        160,
        56,
        Direction::Forward,
    );
    // The quarter turn by direction (the frame note): back on a Forward motor, forward on a
    // Reverse one.
    assert_eq!(p.quarter, 0u16.wrapping_sub(crate::foc::ANGLE_90));
    assert_eq!(p.r_q, 240 * 65_536 / 160);
    assert_eq!(p.l_q, 16 * 500 * 65_536 / 160);
    // 56 ticks of the 4,500-tick period against the voltage scale: 706, so a 36 V bus's kv of
    // 41,570 drops 448 mV a phase.
    assert_eq!(p.dt_q, 706);
    assert_eq!((41_570 * p.dt_q) >> 16, 447);
    assert_eq!(
        (
            p.leak_shift,
            p.dt_comp,
            p.band,
            p.spread,
            p.min_windows,
            p.ratio
        ),
        (10, true, 2730, 1820, 64, 243)
    );
    let q = ObserverParams::derive(
        [-5, -5, 0, 0],
        [-1, -1, 0, 300, -4, 2, 0],
        0,
        0,
        Direction::Reverse,
    );
    assert_eq!(q.quarter, crate::foc::ANGLE_90);
    assert_eq!((q.r_q, q.l_q, q.dt_q), (0, 0, 0));
    assert_eq!(
        (
            q.leak_shift,
            q.dt_comp,
            q.band,
            q.spread,
            q.min_windows,
            q.ratio
        ),
        (6, false, 0, 0, 1, 256)
    );
    assert_eq!(q.offset, (-4i16) as u16);
    assert_eq!(
        ObserverParams::derive([0; 4], [0, 0, 0, 0, 0, 40, 0], 455, 0, Direction::Reverse)
            .leak_shift,
        14
    );
}

/// The bus scale: a local sense's affine form against the battery word's conversion, a borrowed
/// word, the suspended scale, and the pack.
#[test]
fn the_bus_scale_converts_the_period_count_and_a_borrowed_word() {
    // The fleet calibration (25,200 uV per count, -5 cV) at a count reading 31.00 V.
    let s = BusScale::local(25_200, -5);
    let raw = ((3100 + 5) * 10_000 / 25_200) as u32;
    let want = (3100.0 * 10.0 * 65_536.0 / KV_DEN as f64) as i32;
    let kv = bus_kv(raw, s.pack());
    assert!((kv - want).abs() < want / 500, "{kv} vs {want}");
    // A borrowed word of 31.00 V.
    let kv = bus_kv(1234, BusScale::word(3100).pack());
    assert!((kv - want).abs() <= 2, "{kv} vs {want}");
    // Unknown and stale suspend.
    assert_eq!(bus_kv(4095, BusScale::NONE.pack()), 0);
    assert_eq!(bus_kv(4095, BusScale::word(0).pack()), 0);
    // A huge word clamps at the scale's ceiling rather than overflowing the multiply.
    assert_eq!(bus_kv(0, BusScale::word(i16::MAX).pack()), KV_MAX - 1);
    for w in [s, BusScale::word(3100), BusScale::NONE] {
        assert_eq!(BusScale::unpack(w.pack()), w);
    }
}

// ---------------------------------------------------------------------------------------------
// The table and the lock bin
// ---------------------------------------------------------------------------------------------

/// A table built from `(seen, tracked)` per bin.
fn table(rows: &[(usize, u16, u16)]) -> [u32; BINS] {
    let mut t = [0u32; BINS];
    for &(k, s, tr) in rows {
        t[k] = s as u32 | (tr as u32) << 16;
    }
    t
}

fn lock(t: &[u32; BINS]) -> u8 {
    lock_bin(|k| t[k], 64, 243)
}

/// The counts saturate rather than wrap, and a clear empties the table.
#[test]
fn the_tables_counts_saturate_and_clear() {
    let t = TrackTable::new();
    t.credit(3, true);
    t.credit(3, false);
    t.credit(19, true); // an out-of-range bin folds, never panics
    assert_eq!((seen(t.word(3)), tracked(t.word(3))), (3, 2));
    assert_eq!(credit(0xFFFF_FFFF, true), 0xFFFF_FFFF);
    assert_eq!(credit(0x0000_FFFF, true), 0x0001_FFFF);
    t.clear();
    assert!((0..BINS).all(|k| t.word(k) == 0));
    assert_eq!(t.lock_bin(64, 243), LOCK_NONE);
}

/// The lock bin on hand-built tables: none, all, a gap in the middle, a bin under `min_windows`,
/// an unmeasured bin 0 below passing bins.
#[test]
fn the_lock_bin_on_hand_built_tables() {
    // None: an empty table, and a table of failing bins.
    assert_eq!(lock(&[0; BINS]), LOCK_NONE);
    assert_eq!(lock(&table(&[(5, 100, 10), (9, 100, 50)])), LOCK_NONE);
    // All passing from bin 1: bin 0 (rest) never locks even when it passes too.
    let all: Vec<(usize, u16, u16)> = (0..BINS).map(|k| (k, 100, 100)).collect();
    assert_eq!(lock(&table(&all)), 1);
    // A failing bin in the middle: the lock is above it.
    let mut gap = all.clone();
    gap[7] = (7, 100, 50);
    assert_eq!(lock(&table(&gap)), 8);
    // A bin under min_windows neither passes nor blocks: bins 3..15 pass, bin 6 has 63 windows.
    let mut thin: Vec<(usize, u16, u16)> = (3..BINS).map(|k| (k, 100, 100)).collect();
    thin[3] = (6, 63, 0);
    assert_eq!(lock(&table(&thin)), 3);
    // An unmeasured bin 0 below passing bins locks at the lowest passing bin, not at 0.
    assert_eq!(lock(&table(&[(0, 10, 0), (4, 64, 64), (5, 64, 64)])), 4);
    // The ratio's edge: 243/256 of 256 windows is 242.0..; 243 tracked passes, 242 fails.
    assert_eq!(lock(&table(&[(2, 256, 243)])), 2);
    assert_eq!(lock(&table(&[(2, 256, 242)])), LOCK_NONE);
}

/// The trace byte: 255 until measured, then tracked over seen in 0.4 percent steps.
#[test]
fn the_trace_byte_is_the_ratio_in_quarter_percent_steps() {
    assert_eq!(track_byte(0, 64), 255);
    assert_eq!(track_byte(63 | 63 << 16, 64), 255);
    assert_eq!(track_byte(64 | 64 << 16, 64), 250);
    assert_eq!(track_byte(100 | 50 << 16, 64), 125);
    assert_eq!(track_byte(100, 64), 0);
}

/// The reset: a cleared table, then credits count from zero.
#[test]
fn the_reset_empties_the_table_and_the_lock() {
    let t = TrackTable::new();
    for _ in 0..64 {
        t.credit(4, true);
    }
    assert_eq!(t.lock_bin(64, 243), 4);
    t.clear();
    assert_eq!(t.lock_bin(64, 243), LOCK_NONE);
    t.credit(4, true);
    assert_eq!(t.word(4), 1 | 1 << 16);
}

/// The source angle is the angle the last step computed, word 52's high half, less the offset,
/// with no step of its own: reading it does not move the observer.
#[test]
fn the_source_angle_is_the_last_steps_angle_less_the_offset() {
    let mut p = plant_params(true);
    let mut obs = FluxObserver::new();
    let mut last = 0u16;
    for n in 0..200i32 {
        let x = ObserverInput {
            v: ((n * 37 % 4000 - 2000) as i16, (n * 91 % 4000 - 2000) as i16),
            i: (100, -50),
            theta: 0,
            kv: 40_000,
            speed: 3,
            hall_fault: false,
            drive: true,
        };
        assert_eq!(
            obs.source_angle(&p),
            last,
            "the angle at the start of period {n}"
        );
        last = obs.step(&p, &x);
    }
    let before = obs;
    p.offset = 5_461;
    assert_eq!(obs.source_angle(&p), last.wrapping_sub(5_461));
    assert_eq!(obs, before, "reading it changes nothing");
    // Suspended: the last integrated angle holds.
    obs.step(
        &p,
        &ObserverInput {
            v: (0, 0),
            i: (0, 0),
            theta: 0,
            kv: 0,
            speed: 0,
            hall_fault: false,
            drive: true,
        },
    );
    assert_eq!(obs.source_angle(&p), last.wrapping_sub(5_461));
}

/// The three published words pack as `specs/flux-observer.md` lays them out.
#[test]
fn the_published_words_pack_as_the_spec_lays_them_out() {
    let w = ObserverWindow {
        theta: 0x1234,
        angle: 0xABCD,
        mean: -2,
        spread: 300,
        flux: 21_000,
        judged: true,
        tracked: true,
    };
    assert_eq!(pack_angles(&w), 0xABCD_1234);
    assert_eq!(pack_track(&w), 300 << 16 | 0xFFFE);
    assert_eq!(pack_flux(&w, 7, false, false), (7 | 1 << 8) << 16 | 21_000);
    assert_eq!(
        pack_flux(&ObserverWindow::default(), LOCK_NONE, false, true),
        (255 | 1 << 10) << 16
    );
    // Bit 9: the window's last period drove from the observer (S4's source button).
    assert_eq!(
        pack_flux(&w, 7, true, false),
        (7 | 1 << 8 | 1 << 9) << 16 | 21_000
    );
    assert_eq!(TRACK_SOURCE, 1 << 9);
}

// ---------------------------------------------------------------------------------------------
// The circle form (S4)
// ---------------------------------------------------------------------------------------------

/// The circle form's parameters on the plant: the flux linkage word `flux_uvs`, the shift `shift`,
/// and S1's other defaults with the dead-time compensation on.
fn circle_params(flux_uvs: i16, shift: i16) -> ObserverParams {
    ObserverParams::derive(
        [240, 500, flux_uvs, 0],
        [2730, 1820, 64, 243, 0, shift, 1],
        PLANT_CAL_A as u16,
        56,
        Direction::Reverse,
    )
}

/// One period of a circle run, as [`run_circle`] records it.
#[derive(Clone, Copy, Debug)]
struct CirclePeriod {
    /// The observer's angle less the plant's q axis, degrees, wrapped to a half turn.
    err: f64,
    /// The magnet flux's magnitude, microvolt-seconds.
    mag: f64,
}

/// As [`run_observer`], per period, for `periods` periods at `hz` and `iq_ref`; `kick` runs on the
/// observer after `at` periods (a state perturbation).
fn run_circle(
    hz: f64,
    iq_ref: i32,
    p: &ObserverParams,
    periods: usize,
    at: usize,
    kick: impl Fn(&mut FluxObserver),
) -> Vec<CirclePeriod> {
    let mut st = FocState::new(
        PhaseOffsets::try_new(0x7FB8, 0x7DAE).unwrap(),
        DutyOrder::DIRECT,
        28_500,
    );
    let mut plant = Plant::new(hz, PLANT_R, PLANT_DEAD);
    let mut obs = FluxObserver::new();
    let kv = plant_kv();
    let mut out = Vec::new();
    for n in 0..periods {
        if n == at {
            kick(&mut obs);
        }
        let i = plant.sense();
        let theta = plant.theta_q();
        let prev = st.applied();
        let next = foc_step(&mut st, theta, i, iq_ref, true, false);
        let angle = obs.step(
            p,
            &ObserverInput {
                v: (prev.alpha, prev.beta),
                i,
                theta,
                kv,
                speed: 0,
                hall_fault: false,
                drive: true,
            },
        );
        let c = clarke(i.0, i.1);
        let la = ((c.alpha as i64 * p.l_q as i64) >> 16) as f64;
        let lb = ((c.beta as i64 * p.l_q as i64) >> 16) as f64;
        let (a, b) = obs.psi();
        let mag = ((a as f64 - la).powi(2) + (b as f64 - lb).powi(2)).sqrt() / 16.0;
        let err = (angle.wrapping_sub(theta) as i16) as f64 * DEG;
        out.push(CirclePeriod { err, mag });
        plant.step(next);
    }
    out
}

/// Milliseconds from period `from` to the last period whose `bad` holds (0 when none does): the
/// time the run took to settle for good, since every later period is good.
fn settled_ms(r: &[CirclePeriod], from: usize, bad: impl Fn(&CirclePeriod) -> bool) -> f64 {
    let last = r
        .iter()
        .enumerate()
        .skip(from)
        .filter(|(_, x)| bad(x))
        .map(|(n, _)| n + 1)
        .last()
        .unwrap_or(from);
    (last - from) as f64 / 16.0
}

/// A second of periods: long enough that a run settled in its first quarter has stayed settled.
const SECOND: usize = 16_000;

/// The circle form's plant speeds, and how long each takes to settle from a zero state at shift
/// 6, with a margin over the plant's figures (19, 11 and 13 ms from 50 Hz up, 149 ms at 20 Hz):
/// the correction is radial, so an offset of the flux centre is removed only as the rotor turns it
/// through the radial direction, which is slow at 20 Hz.
const CIRCLE_SWEEP: [(f64, f64); 4] = [(20.0, 175.0), (50.0, 25.0), (100.0, 25.0), (150.0, 25.0)];

/// **The circle form from a zero state** (`specs/flux-observer.md`, "S4: the driving observer",
/// host tests), `flux_uvs` the plant's 21,000 at shift 6, at each speed of the plant's sweep: the
/// magnitude reaches the circle (within 3 percent, for good) and the angle error settles under 2
/// degrees with no lead (every period after), inside [`CIRCLE_SWEEP`]'s times.
#[test]
fn the_circle_form_settles_on_the_rotor_with_no_lead() {
    let p = circle_params(21_000, 6);
    for (hz, limit) in CIRCLE_SWEEP {
        let r = run_circle(hz, 600, &p, SECOND, usize::MAX, |_| {});
        let mag = settled_ms(&r, 0, |x| (x.mag / (PLANT_FLUX * 1e6) - 1.0).abs() >= 0.03);
        let angle = settled_ms(&r, 0, |x| x.err.abs() >= 2.0);
        let tail = r[SECOND / 2..]
            .iter()
            .map(|x| x.err.abs())
            .fold(0.0, f64::max);
        println!(
            "circle, {hz:5.0} Hz from zero: magnitude in 3 percent after {mag:5.1} ms, error \
             under 2 degrees after {angle:5.1} ms, the last half second's worst {tail:4.2} deg"
        );
        assert!(mag <= limit, "{hz} Hz: magnitude {mag} ms");
        assert!(angle <= limit, "{hz} Hz: angle {angle} ms");
    }
}

/// **The circle form from a state 30 degrees off**: the settled flux turned by 30 degrees, which
/// leaves its magnitude on the circle and its centre off the origin; the error decays under 2
/// degrees for good inside [`CIRCLE_SWEEP`]'s times (the plant's: 18, 12 and 11 ms from 50 Hz up,
/// 103 ms at 20 Hz).
#[test]
fn the_circle_form_recovers_from_a_state_30_degrees_off() {
    let p = circle_params(21_000, 6);
    let at = SECOND / 2;
    for (hz, limit) in CIRCLE_SWEEP {
        let r = run_circle(hz, 600, &p, at + SECOND / 2, at, |o| {
            let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
            let (a, b) = (o.psi_a as f64, o.psi_b as f64);
            o.psi_a = (a * c - b * s).round() as i32;
            o.psi_b = (a * s + b * c).round() as i32;
        });
        let kicked = r[at].err;
        let t = settled_ms(&r, at, |x| x.err.abs() >= 2.0);
        println!(
            "circle, {hz:5.0} Hz: kicked to {kicked:6.1} deg, under 2 degrees after {t:5.1} ms"
        );
        assert!(
            kicked.abs() > 25.0,
            "{hz} Hz: the kick moved the angle {kicked}"
        );
        assert!(t <= limit, "{hz} Hz: {t} ms");
    }
}

/// **A wrong `flux_uvs`** (10 percent low and high against the plant's 21,000): the error mean
/// takes the sign of the word's error (ahead with the word low, behind with it high), shrinks as
/// the speed rises, and moves under half a degree with the current. This is the plant's answer,
/// not the spec's prediction (which had it move with current and not with speed): the voltage
/// model holds the magnitude near the true linkage at speed, and the correction, a fixed fraction
/// a period, turns the angle by less of a revolution the faster the rotor turns.
#[test]
fn a_wrong_flux_linkage_reads_as_an_error_that_falls_with_speed() {
    let mean_err = |flux_uvs: i16, hz: f64, iq_ref: i32| {
        let r = run_circle(
            hz,
            iq_ref,
            &circle_params(flux_uvs, 6),
            SECOND,
            usize::MAX,
            |_| {},
        );
        let w = &r[SECOND * 3 / 4..];
        let m = w.iter().map(|x| x.err).sum::<f64>() / w.len() as f64;
        let mag = w.iter().map(|x| x.mag).sum::<f64>() / w.len() as f64;
        println!(
            "flux_uvs {flux_uvs}, {hz:4.0} Hz, iq_ref {iq_ref:4}: error mean {m:6.2} deg, \
             magnitude {mag:6.0} uV.s"
        );
        m
    };
    for (flux_uvs, sign) in [(18_900i16, 1.0), (23_100, -1.0)] {
        let by_speed: Vec<f64> = [50.0, 100.0]
            .iter()
            .map(|&hz| mean_err(flux_uvs, hz, 1200) * sign)
            .collect();
        assert!(by_speed[0] > 2.0, "{flux_uvs}: {by_speed:?}");
        assert!(by_speed[1] > 1.0, "{flux_uvs}: {by_speed:?}");
        assert!(by_speed[1] < by_speed[0] * 0.75, "{flux_uvs}: {by_speed:?}");
        let by_current: Vec<f64> = [600, 1200, 2400]
            .iter()
            .map(|&iq| mean_err(flux_uvs, 50.0, iq))
            .collect();
        let spread = by_current.iter().fold(f64::MIN, |a, &b| a.max(b))
            - by_current.iter().fold(f64::MAX, |a, &b| a.min(b));
        assert!(spread < 0.5, "{flux_uvs}: {by_current:?}");
    }
    // The right word reads no error at the same points.
    for hz in [50.0, 100.0] {
        assert!(mean_err(21_000, hz, 1200).abs() < 0.5);
    }
}

/// **The lead table**: all zero with the circle (no leak to lead), S1's with the leak; and the
/// circle form's verdict error and source angle carry none at any speed.
#[test]
fn the_lead_table_is_zero_with_the_circle_and_s1s_with_the_leak() {
    assert_eq!(circle_params(21_000, 10).lead, [0; BINS]);
    assert_eq!(circle_params(0, 10).lead, lead_table(10));
    assert_eq!(circle_params(0, 12).lead, lead_table(12));
    let p = circle_params(21_000, 6);
    let mut obs = FluxObserver::new();
    for n in 0..200i32 {
        let angle = obs.step(
            &p,
            &ObserverInput {
                v: ((n * 37 % 4000 - 2000) as i16, (n * 91 % 4000 - 2000) as i16),
                i: (100, -50),
                theta: 0,
                kv: 40_000,
                speed: 5,
                hall_fault: false,
                drive: true,
            },
        );
        assert_eq!(obs.lead, 0);
        assert_eq!(obs.source_angle(&p), angle);
    }
}

/// **The install**: `lam2` and `circle_k` from the word, the shift clamped as S1's, 0 for a zero
/// or negative word (the leaky form, every other parameter as S1 derives it); and the gain is
/// the one the spec's time constant asks for: at rest with no voltage, a magnitude 1 percent over
/// the circle falls to `1/e` of its excess in `2^shift / 2` periods.
#[test]
fn the_install_computes_the_circle_gain_and_clamps_the_shift() {
    let lam = (21_000 * 16) >> 8;
    assert_eq!(lam, 1_312);
    let p = circle_params(21_000, 6);
    assert_eq!((p.lam2, p.circle_k), (lam * lam, (1 << 29) / (lam * lam)));
    assert_eq!(p.circle_k, 311);
    let q = circle_params(21_000, 40);
    assert_eq!((q.leak_shift, q.lam2, q.circle_k), (14, p.lam2, p.circle_k));
    assert_eq!(circle_params(21_000, 2).leak_shift, 6);
    // The largest word and the smallest.
    let big = circle_params(i16::MAX, 14);
    assert_eq!((big.lam2, big.circle_k), (2_047 * 2_047, 128));
    let small = circle_params(1, 6);
    assert_eq!((small.lam2, small.circle_k), (1, 1 << 29));
    for w in [0, -21_000] {
        let s1 = circle_params(w, 10);
        assert_eq!((s1.lam2, s1.circle_k), (0, 0));
        assert_eq!(s1, plant_params_with_lead());
    }
    // The decay at rest.
    // Two shifts where the correction near the circle is many LSB a period: at the top of the
    // range it is under one (`pm e / 2^14`), and the floor of the shift leaves a one-LSB inward
    // pull, as S1's leak does at the same shift.
    for shift in [6i16, 8] {
        let p = circle_params(21_000, shift);
        let lsb = PLANT_FLUX * 1e6 * 16.0;
        let mut obs = FluxObserver::new();
        obs.psi_a = (1.01 * lsb) as i32;
        let x = ObserverInput {
            v: (0, 0),
            i: (0, 0),
            theta: 0,
            kv: plant_kv(),
            speed: 0,
            hall_fault: false,
            drive: true,
        };
        let tau = 1usize << (shift - 1);
        for _ in 0..tau {
            obs.step(&p, &x);
        }
        let excess = (obs.psi().0 as f64 / lsb - 1.0) / 0.01;
        println!("shift {shift}: the excess after {tau} periods is {excess:.3} of itself");
        assert!((excess - (-1f64).exp()).abs() < 0.03, "{excess}");
        assert_eq!(obs.psi().1, 0, "the correction is radial");
    }
}

/// S1's parameters on the plant at shift 10, the lead table kept.
fn plant_params_with_lead() -> ObserverParams {
    ObserverParams::derive(
        [240, 500, 0, 0],
        [2730, 1820, 64, 243, 0, 10, 1],
        PLANT_CAL_A as u16,
        56,
        Direction::Reverse,
    )
}
