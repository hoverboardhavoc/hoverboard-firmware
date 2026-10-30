//! Host tests for the state-feedback law (`specs/state-feedback.md` section 11, control (sf)): the
//! arithmetic against its closed forms, the reference and the fold, the ramp, the signs, and a
//! plant model of the rover (section 7's linearized A and B) closed through the integer law.

use super::*;
use crate::config::sf::{NOMINAL_SCALE, Q, RAMP_CAPS};
use std::vec::Vec;

/// Section 7's start gains, `[gp, gr, gx, gv]`.
const START: [i16; 4] = [3491, 4002, 197, 926];
/// Section 7's reference row and clamp.
const START_CFG: SfConfig = SfConfig {
    gains: START,
    speed_max: 80,
    speed_slew: 41,
    pos_clamp: 45,
};
/// The nominal pack the torque scale is quoted at (assumption A1), the divisor torque mode pins.
const NOMINAL: i16 = NOMINAL_SCALE;

fn run_input() -> SfInputs {
    SfInputs {
        scale: NOMINAL,
        in_run: true,
        role_right: true,
        ..SfInputs::default()
    }
}

// ----- The law's arithmetic -----

#[test]
fn each_term_alone_is_its_closed_form() {
    let g = [1234, 2345, 345, 456];
    for s in [
        -87_266i64, -20_001, -4097, -101, -1, 0, 1, 99, 4096, 18_000, 87_266,
    ] {
        let t = terms(g, s.clamp(-32768, 32767) as i16, 0, 0, 0);
        assert_eq!(t, [1234 * s.clamp(-32768, 32767) / 100, 0, 0, 0]);
        let t = terms(g, 0, s as i32, 0, 0);
        assert_eq!(t, [0, 2345 * s / 1000, 0, 0]);
        let t = terms(g, 0, 0, s as i32 * 101, 0);
        assert_eq!(t, [0, 0, 345 * s * 101 / 20480, 0]);
        let t = terms(g, 0, 0, 0, s as i32);
        assert_eq!(t, [0, 0, 0, 456 * s / 256]);
    }
    // Truncation is toward zero, once per term: -1 * 99 / 100 is 0, not -1.
    assert_eq!(terms([99, 0, 0, 0], -1, 0, 0, 0), [0, 0, 0, 0]);
    assert_eq!(terms([199, 0, 0, 0], -1, 0, 0, 0), [-1, 0, 0, 0]);
}

#[test]
fn the_pass_records_each_term_as_a_word_on_its_own() {
    // The observation's four terms (section 8): each through `word` alone, so their sum can
    // differ from the output word only by the per-term truncations and the clamp.
    let mut st = SfState::default();
    let inp = SfInputs {
        pitch_word: 250,
        pitch_rate: -3000,
        wheel_rate_q12: 4096,
        ..run_input()
    };
    let cfg = SfConfig {
        speed_max: 0,
        ..START_CFG
    };
    let w = sf_step(&cfg, &inp, &mut st);
    let t = terms(
        st.live,
        inp.pitch_word,
        inp.pitch_rate,
        st.pos_err,
        -st.v_filt,
    );
    assert_eq!(st.terms, core::array::from_fn(|i| word(t[i], NOMINAL)));
    assert_eq!(w, word(t.iter().sum(), NOMINAL));
    let sum: i32 = st.terms.iter().map(|&x| x as i32).sum();
    assert!((sum - w as i32).abs() <= 4, "{sum} against {w}");
    // An unknown battery word zeroes every term as it zeroes the word.
    sf_step(&cfg, &SfInputs { scale: 0, ..inp }, &mut st);
    assert_eq!(st.terms, [0; 4]);
}

#[test]
fn the_position_clamp_seam_clamps_into_zero_to_its_ceiling() {
    assert_eq!(POS_CLAMP_MAX, 450);
    for (stored, want) in [
        (i16::MIN, 0),
        (-1, 0),
        (0, 0),
        (45, 45),
        (450, 450),
        (451, 450),
    ] {
        assert_eq!(pos_clamp(stored), want, "{stored}");
    }
    assert_eq!(pos_clamp(i16::MAX), 450);
}

#[test]
fn the_word_is_numer_times_3900_over_scale_and_zero_on_an_unknown_battery() {
    assert_eq!(word(1000, 2960), (1000 * 3900 / 2960) as i16);
    assert_eq!(word(-1000, 2960), -(1000 * 3900 / 2960) as i16);
    assert_eq!(word(1_000_000, 0), 0, "scale 0 is UNKNOWN");
    assert_eq!(word(-1_000_000, 0), 0);
}

#[test]
fn the_output_clamp_binds_at_full_scale() {
    // 28500 * 2960 / 3900 = 21630.8: under it the word is the quotient, over it (21,700 asks for
    // 28,591) the clamp.
    assert_eq!(word(21_630, NOMINAL), 28_498);
    assert_eq!(word(-21_630, NOMINAL), -28_498);
    assert_eq!(word(21_700, NOMINAL), 28_500);
    assert_eq!(word(-21_700, NOMINAL), -28_500);
    assert_eq!(word(i64::MAX / 3900, NOMINAL), 28_500);
    assert_eq!(word(-(i64::MAX / 3900), NOMINAL), -28_500);
}

#[test]
fn the_products_at_their_bounds_do_not_overflow() {
    let g = [i16::MAX; 4];
    let t = terms(g, i16::MIN, i32::MIN, -(i16::MAX as i32) * 20480, -32 * Q);
    assert!(t.iter().all(|&v| v < 0));
    assert_eq!(word(t.iter().sum(), 1), -28_500);
}

// ----- Signs (section 2) -----

#[test]
fn a_forward_lean_gives_a_forward_word() {
    let mut st = SfState::default();
    let w = sf_step(
        &START_CFG,
        &SfInputs {
            pitch_word: -500,
            ..run_input()
        },
        &mut st,
    );
    assert!(w < 0, "forward lean, forward torque: {w}");
    let w = sf_step(
        &START_CFG,
        &SfInputs {
            pitch_rate: -10_000,
            ..run_input()
        },
        &mut SfState::default(),
    );
    assert!(w < 0, "forward pitch rate, forward torque: {w}");
}

#[test]
fn a_forward_push_gives_negative_position_and_speed_terms() {
    // The right wheel counts DOWN going forward; one edge forward on each wheel and one count of
    // forward speed, in the counters' own senses.
    let cfg = SfConfig {
        speed_slew: 41,
        pos_clamp: 45,
        gains: START,
        ..SfConfig::default()
    };
    let mut st = SfState::default();
    sf_step(&cfg, &SfInputs { ..run_input() }, &mut st);
    let pushed = SfInputs {
        wheel_pos: -1,
        peer_pos: 1,
        wheel_rate_q12: -Q,
        ..run_input()
    };
    sf_step(&cfg, &pushed, &mut st);
    assert!(st.pos_err < 0, "x grew, so x_ref - x is negative");
    assert!(st.v_filt > 0, "forward speed reads positive");
    let t = terms(START, 0, 0, st.pos_err, st.v_ref - st.v_filt);
    assert!(t[2] < 0 && t[3] < 0, "term_x and term_v negative: {t:?}");
    assert!(
        sf_step(&cfg, &pushed, &mut st) < 0,
        "a forward word: forward torque tips the body back"
    );
}

// ----- The reference, the fold, the filter -----

#[test]
fn the_reference_slews_by_exactly_speed_slew_and_stops_at_the_target() {
    let cfg = SfConfig {
        speed_max: 80,
        speed_slew: 41,
        ..SfConfig::default()
    };
    let mut st = SfState::default();
    let target = 32767 * 80 / 128;
    let mut prev = 0;
    for _ in 0..600 {
        sf_step(
            &cfg,
            &SfInputs {
                value: 32767,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!(st.v_ref - prev, (target - prev).min(41));
        prev = st.v_ref;
    }
    assert_eq!(st.v_ref, target, "stops at the target");
    // Back to centre: it ramps down at the same slew, which is also the stale-link stop.
    sf_step(&cfg, &run_input(), &mut st);
    assert_eq!(st.v_ref, target - 41);
}

#[test]
fn the_position_error_accumulates_the_reference_less_the_fold_and_clamps_both_ways() {
    let cfg = SfConfig {
        speed_max: 80,
        speed_slew: 4096,
        pos_clamp: 45,
        ..SfConfig::default()
    };
    let mut st = SfState::default();
    // A reference speed with the wheels still: P grows by v_ref per tick.
    sf_step(
        &cfg,
        &SfInputs {
            value: 1600,
            ..run_input()
        },
        &mut st,
    );
    let v = 1600 * 80 / 128;
    assert_eq!((st.v_ref, st.pos_err), (v, v));
    // Then one edge forward on each wheel (right counts down, left up): dX2 = 2, P -= 20480.
    sf_step(
        &cfg,
        &SfInputs {
            value: 1600,
            wheel_pos: -1,
            peer_pos: 1,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!(st.pos_err, 2 * v - 2 * 10240);
    // Pushed far forward, the error clamps at -pos_clamp edges.
    let mut pos = 0;
    for _ in 0..100 {
        pos += 3;
        sf_step(
            &cfg,
            &SfInputs {
                wheel_pos: -pos,
                peer_pos: pos,
                ..run_input()
            },
            &mut st,
        );
    }
    assert_eq!(st.pos_err, -45 * 20480);
    // And far back, at +pos_clamp.
    for _ in 0..100 {
        pos -= 3;
        sf_step(
            &cfg,
            &SfInputs {
                wheel_pos: -pos,
                peer_pos: pos,
                ..run_input()
            },
            &mut st,
        );
    }
    assert_eq!(st.pos_err, 45 * 20480);
}

#[test]
fn pos_clamp_zero_pins_the_position_error_at_zero() {
    let cfg = SfConfig {
        speed_max: 80,
        speed_slew: 41,
        ..SfConfig::default()
    };
    let mut st = SfState::default();
    for i in 0..200 {
        sf_step(
            &cfg,
            &SfInputs {
                value: 32767,
                wheel_pos: -i,
                peer_pos: i,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!(st.pos_err, 0);
    }
}

#[test]
fn a_turn_in_place_does_not_move_the_position() {
    // Both wheels turning the same physical way is the machine spinning: forward on one wheel,
    // backward on the other, so the mean, and the fold, is zero.
    let cfg = SfConfig {
        pos_clamp: 45,
        ..SfConfig::default()
    };
    let mut st = SfState::default();
    for i in 0..70 {
        sf_step(
            &cfg,
            &SfInputs {
                wheel_pos: -i,
                peer_pos: -i,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!(st.pos_err, 0, "tick {i}");
    }
}

#[test]
fn the_peer_goes_through_the_opposite_mirror_on_either_side() {
    for right in [true, false] {
        let mut st = SfState::default();
        let cfg = SfConfig {
            pos_clamp: 45,
            ..SfConfig::default()
        };
        sf_step(
            &cfg,
            &SfInputs {
                role_right: right,
                ..run_input()
            },
            &mut st,
        );
        // One edge forward on the PEER only: its counter reads +1 if it is the left wheel.
        let peer_fwd = if right { 1 } else { -1 };
        sf_step(
            &cfg,
            &SfInputs {
                role_right: right,
                peer_pos: peer_fwd,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!(
            st.pos_err, -10240,
            "half an edge of mean, forward, right={right}"
        );
    }
}

#[test]
fn the_fold_folds_deltas_across_the_i32_wrap_and_a_quiet_peer_adds_nothing() {
    let cfg = SfConfig {
        pos_clamp: 45,
        ..SfConfig::default()
    };
    let mut st = SfState::default();
    let quiet = i32::MAX - 1;
    // Not RUN: the latches follow, nothing accumulates, so the disarmed travel is not folded.
    sf_step(
        &cfg,
        &SfInputs {
            wheel_pos: i32::MIN + 1,
            peer_pos: quiet,
            in_run: false,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!(st.pos_err, 0);
    // First RUN tick: one tick's motion, across the wrap (the right wheel counts down going
    // forward: MIN+1 -> MAX is two edges forward), with no new peer word.
    sf_step(
        &cfg,
        &SfInputs {
            wheel_pos: i32::MAX,
            peer_pos: quiet,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!(st.pos_err, -2 * 10240);
}

#[test]
fn every_reference_cell_is_zero_outside_run() {
    let mut st = SfState::default();
    for _ in 0..50 {
        sf_step(
            &START_CFG,
            &SfInputs {
                value: 32767,
                wheel_pos: 3,
                ..run_input()
            },
            &mut st,
        );
    }
    assert!(st.v_ref != 0 && st.pos_err != 0);
    sf_step(
        &START_CFG,
        &SfInputs {
            value: 32767,
            in_run: false,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!((st.v_ref, st.pos_err), (0, 0));
}

#[test]
fn the_speed_error_clamps() {
    let mut st = SfState {
        v_filt: 100 * Q,
        ..SfState::default()
    };
    // At the clamp, term_v is gv * 32 * 4096 / 256, and the filter's own term does not exceed it.
    let w = sf_step(
        &SfConfig {
            gains: [0, 0, 0, 10],
            ..SfConfig::default()
        },
        &SfInputs {
            wheel_rate_q12: -100 * Q,
            scale: 3900,
            // Outside RUN the live gains are the shadow's, so gv is 10 on this pass.
            in_run: false,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!(w as i64, -10 * 32 * 4096 / 256);
}

#[test]
fn the_filter_steps_by_a_quarter_of_the_difference() {
    let mut st = SfState::default();
    let cfg = SfConfig::default();
    let mut prev = 0;
    for _ in 0..40 {
        // Right wheel: -1000 in the counter is +1000 forward.
        sf_step(
            &cfg,
            &SfInputs {
                wheel_rate_q12: -1000,
                in_run: false,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!(st.v_filt, prev + (1000 - prev) / 4);
        prev = st.v_filt;
    }
    // It stalls within FILT_DIV - 1 of the input, the truncation's floor.
    assert!(1000 - st.v_filt < 4);
}

// ----- The live gain ramp (section 6) -----

#[test]
fn the_ramp_steps_each_gain_by_its_cap_and_lands_exactly() {
    let mut live = [0i16; 4];
    let mut passes = [0u32; 4];
    for n in 1..=20_000u32 {
        let next = ramp(live, START);
        for i in 0..4 {
            let step = (next[i] - live[i]) as i32;
            assert!(step.abs() <= RAMP_CAPS[i] as i32);
            if next[i] != START[i] {
                assert_eq!(step, RAMP_CAPS[i] as i32, "a full cap while short of it");
            } else if live[i] != START[i] {
                passes[i] = n;
            }
        }
        live = next;
    }
    assert_eq!(live, START, "every target reached exactly");
    // Section 6's convergence times at 250 Hz: gp 7 s, gr 16 s, gx 0.8 s, gv 3.7 s.
    assert_eq!(passes, [1746, 4002, 197, 926]);
    // And down again.
    let mut live = START;
    for _ in 0..5000 {
        live = ramp(live, [0; 4]);
    }
    assert_eq!(live, [0; 4]);
}

#[test]
fn a_ramped_pass_moves_the_word_under_the_slew_limit_and_an_unramped_one_does_not() {
    // Section 6's bounds: the engage window's pitch, 147 deg/s, a 45-edge error, one count, at a
    // battery word of 2400.
    let at = |g: [i16; 4]| {
        let t = terms(g, 2499, 25_600, 45 * 20480, Q);
        word(t.iter().sum(), 2400) as i32
    };
    let mut live = [0i16; 4];
    let mut worst = 0;
    while live != START {
        let next = ramp(live, START);
        worst = worst.max((at(next) - at(live)).abs());
        live = next;
    }
    assert!(worst < 250, "per-pass torque delta {worst}");
    assert!(worst > 200, "and the bound is near-tight: {worst}");
    // The negative control: the shadow applied in one pass.
    assert!((at(START) - at([0; 4])).abs() > 250);
}

#[test]
fn outside_run_the_live_gains_are_the_shadows() {
    let mut st = SfState::default();
    sf_step(
        &START_CFG,
        &SfInputs {
            in_run: false,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!(st.live, START);
    // In RUN a tuning step ramps.
    let tuned = SfConfig {
        gains: [3591, 4002, 197, 926],
        ..START_CFG
    };
    sf_step(&tuned, &run_input(), &mut st);
    assert_eq!(st.live, [3493, 4002, 197, 926]);
}

// ----- The plant rig -----

/// The rover as section 7 models it (`tools/lqr-gains.py` holds the same numbers), integrated with
/// RK4 inside each 4 ms tick, read through an 8 ms sensor delay, and driven by the integer law.
/// The leader is the right wheel; the peer's word arrives every other tick (125 Hz). The wheel rate
/// is the ideal one (the true speed, truncated to the word), not the period ISR's edge-held one.
struct Rover {
    /// `[theta, theta', x, x']`, physics signs (forward positive).
    z: [f64; 4],
    hist: Vec<[f64; 4]>,
    st: SfState,
    tick: u32,
    /// Peak |torque| applied, N.m, and the peak |word| that asked for it.
    peak_tau: f64,
    peak_word: i16,
    /// A constant error on the pitch the attitude reports, centidegrees.
    pitch_err: i16,
}

const MB: f64 = 20.98;
const COM: f64 = 0.1627;
const IB: f64 = 1.74;
const MW: f64 = 5.54;
const IW: f64 = 0.008;
const R: f64 = 0.0825;
const DELAY: usize = 2;

fn edge() -> f64 {
    2.0 * core::f64::consts::PI * R / 90.0
}

fn count() -> f64 {
    50.0 * edge()
}

impl Rover {
    fn new() -> Self {
        Rover {
            z: [0.0; 4],
            hist: std::vec![[0.0; 4]; DELAY + 1],
            st: SfState::default(),
            tick: 0,
            peak_tau: 0.0,
            peak_word: 0,
            pitch_err: 0,
        }
    }

    fn deriv(z: [f64; 4], tau: f64, force: f64) -> [f64; 4] {
        let a = MB + MW + 2.0 * IW / (R * R);
        let b = MB * COM;
        let d = MB * 9.81 * COM;
        let det = a * IB - b * b;
        let f1 = tau / R + force;
        let f2 = d * z[0] - tau;
        [
            z[1],
            (a * f2 - b * f1) / det,
            z[3],
            (IB * f1 - b * f2) / det,
        ]
    }

    fn inputs(&self, cfg_in_run: bool, value: i16) -> SfInputs {
        let m = self.hist[self.hist.len() - 1 - DELAY];
        let x_edges = (m[2] / edge()).floor() as i32;
        // The peer's word lands on even ticks only; between them the leader holds the last one.
        let peer_tick = self.hist.len() - 1 - DELAY - (self.tick as usize % 2);
        let peer = (self.hist[peer_tick][2] / edge()).floor() as i32;
        SfInputs {
            pitch_word: (-(m[0].to_degrees() * 100.0)) as i16 + self.pitch_err,
            pitch_rate: (-(m[1] * 1e4)) as i32,
            wheel_rate_q12: (-(m[3] / count() * Q as f64)) as i32,
            wheel_pos: -x_edges,
            peer_pos: peer,
            role_right: true,
            value,
            scale: NOMINAL,
            in_run: cfg_in_run,
        }
    }

    /// One tick: read, the law, the torque the word asks for, the plant. Returns the word.
    fn tick(&mut self, cfg: &SfConfig, value: i16, force: f64) -> i16 {
        let inp = self.inputs(true, value);
        let w = sf_step(cfg, &inp, &mut self.st);
        // Assumption A1: a numerator count is 15.2 / 28500 * 3900 / 2960 N.m, and the word is the
        // numerator times 3900 / scale; torque is negative-word forward.
        let tau = -(w as f64) * NOMINAL as f64 / 3900.0 * (15.2 / 28500.0 * 3900.0 / 2960.0);
        self.peak_tau = self.peak_tau.max(tau.abs());
        self.peak_word = self.peak_word.max(w.saturating_abs());
        let h = 0.004 / 20.0;
        for _ in 0..20 {
            let add = |z: [f64; 4], k: [f64; 4], s: f64| core::array::from_fn(|i| z[i] + s * k[i]);
            let k1 = Self::deriv(self.z, tau, force);
            let k2 = Self::deriv(add(self.z, k1, h / 2.0), tau, force);
            let k3 = Self::deriv(add(self.z, k2, h / 2.0), tau, force);
            let k4 = Self::deriv(add(self.z, k3, h), tau, force);
            self.z = core::array::from_fn(|i| {
                self.z[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
            });
        }
        self.hist.push(self.z);
        self.tick += 1;
        w
    }

    /// Engaged at rest: one not-RUN pass latches the positions, then `ticks` of RUN.
    fn engaged(cfg: &SfConfig, ticks: u32) -> Self {
        let mut r = Rover::new();
        let inp = r.inputs(false, 0);
        sf_step(cfg, &inp, &mut r.st);
        for _ in 0..ticks {
            r.tick(cfg, 0, 0.0);
        }
        r
    }
}

#[test]
fn a_one_count_push_stops_inside_a_second_and_comes_back() {
    let mut r = Rover::engaged(&START_CFG, 50);
    r.peak_tau = 0.0;
    r.peak_word = 0;
    r.z[3] = count();
    let mut stop = None;
    let mut travel = 0.0;
    let mut peak_pitch: f64 = 0.0;
    for t in 1..=1000u32 {
        r.tick(&START_CFG, 0, 0.0);
        peak_pitch = peak_pitch.max(r.z[0].abs().to_degrees());
        if stop.is_none() && r.z[3] <= 0.0 {
            stop = Some(t as f64 / 250.0);
            travel = r.z[2];
        }
    }
    let stop = stop.expect("the push must stop");
    std::println!(
        "one-count push: stops in {stop:.3} s after {travel:.3} m, peak torque {:.2} N.m (word {}), peak pitch {peak_pitch:.2} deg",
        r.peak_tau, r.peak_word
    );
    assert!(stop < 1.0, "stops in {stop} s");
    assert!(travel < 0.2, "stops within 0.2 m: {travel}");
    // Section 7's continuous-law simulation peaks at 10.9 N.m on the push's first tick (the speed
    // term at one count). Here the 16 ms speed filter spreads that step while the pitch term
    // answers, and the rig peaks at 6.7 N.m, word 12,611 at the nominal pack (gate 3 reads under
    // 12,000).
    assert!(r.peak_tau < 8.0, "peak torque {}", r.peak_tau);
    // Back toward the start, and resting near it after 4 s.
    assert!(r.z[2].abs() < 0.03, "back near the start: {}", r.z[2]);
    assert!(r.z[3].abs() < 0.1 * count(), "at rest: {}", r.z[3]);
}

#[test]
fn a_held_push_released_returns_without_overshooting_the_start() {
    let mut r = Rover::engaged(&START_CFG, 50);
    for _ in 0..500 {
        r.tick(&START_CFG, 0, 20.0);
    }
    let held = r.z[2];
    assert!(held > 0.1 && held < 45.0 * edge(), "held at {held} m");
    let mut least = f64::MAX;
    for _ in 0..1500 {
        r.tick(&START_CFG, 0, 0.0);
        least = least.min(r.z[2]);
    }
    std::println!("20 N held 2 s: {held:.3} m, least after release {least:.4} m");
    assert!(least > -edge(), "no overshoot past the start: {least}");
    assert!(r.z[2].abs() < 0.01, "back within 1 cm: {}", r.z[2]);
}

#[test]
fn a_held_push_past_the_clamp_comes_back_only_the_clamp() {
    let mut r = Rover::engaged(&START_CFG, 50);
    for _ in 0..750 {
        r.tick(&START_CFG, 0, 40.0);
    }
    let held = r.z[2];
    assert!(held > 45.0 * edge(), "pushed past the clamp: {held} m");
    for _ in 0..1500 {
        r.tick(&START_CFG, 0, 0.0);
    }
    let back = held - r.z[2];
    assert!(
        (back - 45.0 * edge()).abs() < 2.0 * edge(),
        "came back {back} m against a {} m clamp",
        45.0 * edge()
    );
}

#[test]
fn a_stick_step_reaches_ninety_percent_inside_the_slew_time_plus_a_second() {
    let mut r = Rover::engaged(&START_CFG, 50);
    let target = 80.0 / 16.0 * count();
    let slew_ticks = (32767 * 80 / 128) / 41;
    let mut reached = None;
    for t in 1..=1500u32 {
        r.tick(&START_CFG, 32767, 0.0);
        if reached.is_none() && r.z[3] >= 0.9 * target {
            reached = Some(t);
        }
    }
    let reached = reached.expect("reaches the speed");
    std::println!("stick step: 90 % at {reached} ticks against a {slew_ticks}-tick slew");
    assert!(reached <= slew_ticks as u32 + 250);
    assert!((r.z[3] - target).abs() < 0.5 * count(), "holds {}", r.z[3]);
}

#[test]
fn a_standing_pitch_error_parks_at_its_closed_form_offset() {
    let mut r = Rover::engaged(&START_CFG, 0);
    r.pitch_err = 100;
    for _ in 0..2500 {
        r.tick(&START_CFG, 0, 0.0);
    }
    // gp * p_err / (100 * gx) edges; the error reads as a backward lean, which the position term
    // must cancel, so x_ref - x is negative: the machine parks forward.
    let want = -(3491.0 * 100.0) / (100.0 * 197.0);
    let got = r.st.pos_err as f64 / 20480.0;
    assert!(
        (got - want).abs() < 1.0,
        "parked {got} edges, {want} expected"
    );
    assert!(r.z[3].abs() < 0.01, "and at rest: {}", r.z[3]);
    assert!(r.z[0].abs() < 1e-3, "upright: {}", r.z[0]);
}
