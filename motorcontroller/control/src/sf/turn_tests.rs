//! Host tests for the differential half of the balance law (`specs/heading-hold.md` section 9,
//! control (sf, differential)): the three terms against their closed forms, the signs, the
//! reference, the heading error and its clamp, the ramp, the split's identity, the common half
//! left untouched, and a yaw plant of the rover closed through the integer law.

use super::*;
use crate::config::sf::{NOMINAL_SCALE, Q};
use crate::config::turn::{GAIN_MAX, RAMP_CAPS as TURN_CAPS};
use crate::wheel_split;
use std::vec::Vec;

/// Section 7's starting set: `[gh, ghv, gff] = [50, 60, 0]`, `[rate_max, rate_slew] = [20, 41]`,
/// `[turn_max, head_clamp] = [12000, 45]`.
const START: TurnConfig = TurnConfig {
    gains: [50, 60, 0],
    rate_max: 20,
    rate_slew: 41,
    turn_max: 12000,
    head_clamp: 45,
};
const NOMINAL: i16 = NOMINAL_SCALE;

fn cfg(turn: TurnConfig) -> SfConfig {
    SfConfig {
        turn,
        ..SfConfig::default()
    }
}

/// A RUN pass on the leader, the right wheel, at the nominal pack.
fn run_input() -> SfInputs {
    SfInputs {
        scale: NOMINAL,
        in_run: true,
        role_right: true,
        leader: true,
        ..SfInputs::default()
    }
}

/// A state whose live turn gains already stand at `tc`'s, as they do after any pass outside RUN,
/// so a RUN pass multiplies the staged gains and not a ramp from zero.
fn primed(tc: TurnConfig) -> SfState {
    SfState {
        turn_live: tc.gains,
        ..SfState::default()
    }
}

// ----- The arithmetic -----

#[test]
fn each_differential_term_alone_is_its_closed_form() {
    let g = [123, 2345, 3456];
    for s in [
        -131_072i64,
        -20_481,
        -4097,
        -257,
        -1,
        0,
        1,
        255,
        4096,
        20_480,
        131_072,
    ] {
        let x = s as i32;
        assert_eq!(turn_terms(g, x, 0, 0), [123 * s / 20480, 0, 0]);
        assert_eq!(turn_terms(g, 0, x, 0), [0, 2345 * s / 256, 0]);
        assert_eq!(turn_terms(g, 0, 0, x), [0, 0, 3456 * s / 256]);
    }
    // One truncation toward zero per term: -255 * 1 / 256 is 0, not -1.
    assert_eq!(turn_terms([0, 1, 1], 0, -255, -255), [0, 0, 0]);
    assert_eq!(turn_terms([1, 0, 0], -20479, 0, 0), [0, 0, 0]);
}

#[test]
fn the_differential_is_the_sum_through_the_common_normalization_and_clamps_at_turn_max() {
    // A held heading error alone: v_href 0, the rate filter at rest.
    let tc = TurnConfig {
        gains: [100, 0, 0],
        head_clamp: 450,
        turn_max: 28500,
        ..START
    };
    let mut st = primed(tc);
    // The left (peer) wheel one edge back, the right (own) one forward: a left turn of one edge
    // of heading, so H falls by 1 and HE = +20480. The wheels then stay where they are.
    sf_step(&cfg(tc), &run_input(), &mut st);
    let held = SfInputs {
        wheel_pos: -1,
        peer_pos: -1,
        ..run_input()
    };
    let (_, d) = sf_step(&cfg(tc), &held, &mut st);
    assert_eq!(st.head_err, 20480);
    assert_eq!(d, (100 * 3900 / NOMINAL as i64) as i32);
    // Clamped at +-turn_max.
    let clamped = TurnConfig { turn_max: 50, ..tc };
    let (_, d) = sf_step(&cfg(clamped), &held, &mut st);
    assert_eq!(d, 50);
    st.head_err = -20480;
    let (_, d) = sf_step(&cfg(clamped), &held, &mut st);
    assert_eq!(d, -50);
    // turn_max 0 is no differential whatever the gains.
    let off = TurnConfig { turn_max: 0, ..tc };
    st.head_err = 20480 * 40;
    let (_, d) = sf_step(&cfg(off), &held, &mut st);
    assert_eq!(d, 0);
    // An UNKNOWN battery word is 0, as the common half's word is.
    st.head_err = 20480 * 40;
    let (_, d) = sf_step(&cfg(tc), &SfInputs { scale: 0, ..held }, &mut st);
    assert_eq!(d, 0);
    assert_eq!(st.turn_terms, [0, 0]);
}

#[test]
fn the_terms_are_recorded_as_words_on_their_own() {
    let tc = TurnConfig {
        gains: [200, 300, 0],
        ..START
    };
    let mut st = SfState {
        head_err: 10 * 20480,
        vh_filt: 2 * Q,
        ..primed(tc)
    };
    sf_step(&cfg(tc), &run_input(), &mut st);
    let e_hv = st.v_href - st.vh_filt;
    let t = turn_terms(tc.gains, st.head_err, e_hv, st.v_href);
    assert_eq!(st.turn_terms, [word(t[0], NOMINAL), word(t[1], NOMINAL)]);
}

// ----- Signs (section 2) -----

#[test]
fn stick_right_gives_a_positive_feedforward_on_the_first_tick_and_positive_errors_after() {
    let tc = TurnConfig {
        gains: [50, 60, 60],
        ..START
    };
    let mut st = primed(tc);
    let right = SfInputs {
        steer: 32767,
        ..run_input()
    };
    let (_, d) = sf_step(&cfg(tc), &right, &mut st);
    assert_eq!(st.v_href, 41);
    let ff = 60 * 41 / 256;
    assert!(ff > 0);
    assert!(d > 0, "a right turn asked for: {d}");
    // The wheels still: H_ref runs ahead, both error terms go positive.
    for _ in 0..50 {
        sf_step(&cfg(tc), &right, &mut st);
    }
    assert!(st.head_err > 0);
    assert!(
        st.turn_terms[0] > 0 && st.turn_terms[1] > 0,
        "{:?}",
        st.turn_terms
    );
}

#[test]
fn a_held_left_wheel_with_the_right_rolling_on_asks_for_a_right_turn() {
    // Leader on the right: its counter reads DOWN going forward. The peer (left) is held still.
    let mut st = SfState::default();
    let c = cfg(START);
    sf_step(&c, &run_input(), &mut st);
    let mut d = 0;
    for i in 1..=10 {
        (_, d) = sf_step(
            &c,
            &SfInputs {
                wheel_pos: -i,
                ..run_input()
            },
            &mut st,
        );
    }
    assert!(
        st.head_err > 0,
        "the right wheel ahead is a left turn, so HE > 0"
    );
    assert!(d > 0, "positive: less forward on the free right wheel: {d}");
    // The common torque is untouched by it: the sum of the split is 2T at any T.
    for t in [-9000i16, 0, 9000] {
        let (own, peer) = wheel_split(t, d, true);
        assert_eq!(own as i32 + peer as i32, 2 * t as i32);
        assert!(own > peer, "the right wheel more positive (less forward)");
    }
}

#[test]
fn a_disarmed_hand_turn_to_the_right_reads_a_negative_rate_term() {
    // Right turn in place: left wheel forward, right wheel back. The right wheel's counter reads
    // UP going backward; the left wheel's reads UP going forward.
    let mut st = SfState::default();
    let c = cfg(START);
    let turning = SfInputs {
        in_run: false,
        wheel_rate_q12: Q,
        peer_rate_q12: Q,
        ..run_input()
    };
    let mut d = 0;
    for _ in 0..40 {
        (_, d) = sf_step(&c, &turning, &mut st);
    }
    assert!(
        st.vh_filt > 0,
        "a right turn reads positive: {}",
        st.vh_filt
    );
    assert!(
        st.turn_terms[1] < 0,
        "term_hv opposes the hand: {:?}",
        st.turn_terms
    );
    assert_eq!((st.v_href, st.head_err), (0, 0));
    assert_eq!(d, 0, "outside RUN nothing is applied");
}

#[test]
fn the_heading_and_its_rate_follow_the_side_on_either_wall() {
    for right in [true, false] {
        let mut st = SfState::default();
        let c = cfg(TurnConfig {
            head_clamp: 450,
            ..START
        });
        let base = SfInputs {
            role_right: right,
            ..run_input()
        };
        sf_step(&c, &base, &mut st);
        // One edge FORWARD on the left wheel only (a right turn): the left counter reads +1,
        // whichever board carries it.
        let left_fwd = if right {
            SfInputs {
                peer_pos: 1,
                peer_rate_q12: Q,
                ..base
            }
        } else {
            SfInputs {
                wheel_pos: 1,
                wheel_rate_q12: Q,
                ..base
            }
        };
        sf_step(&c, &left_fwd, &mut st);
        assert_eq!(
            st.head_err, -10240,
            "half an edge of heading, right={right}"
        );
        assert_eq!(
            st.vh_filt,
            (Q / 2) / 4,
            "half a count's quarter, right={right}"
        );
    }
}

// ----- The feedforward -----

#[test]
fn the_feedforward_is_gff_times_the_reference_and_zero_at_gff_zero_or_outside_run() {
    let ff_only = TurnConfig {
        gains: [0, 0, 100],
        rate_max: 80,
        rate_slew: i16::MAX,
        ..START
    };
    let mut st = primed(ff_only);
    let (_, d) = sf_step(
        &cfg(ff_only),
        &SfInputs {
            steer: 16000,
            ..run_input()
        },
        &mut st,
    );
    let v = 16000 * 80 / 128;
    assert_eq!(st.v_href, v);
    assert_eq!(d as i64, (100 * v as i64 / 256) * 3900 / NOMINAL as i64);
    // gff 0: no term.
    let none = TurnConfig {
        gains: [0, 0, 0],
        ..ff_only
    };
    let (_, d) = sf_step(
        &cfg(none),
        &SfInputs {
            steer: 16000,
            ..run_input()
        },
        &mut primed(none),
    );
    assert_eq!(d, 0);
    // Outside RUN: v_href is 0 and so is the term.
    let (_, d) = sf_step(
        &cfg(ff_only),
        &SfInputs {
            steer: 16000,
            in_run: false,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!((st.v_href, d), (0, 0));
}

// ----- The reference, the heading error, the filter -----

#[test]
fn the_reference_slews_by_exactly_rate_slew_and_stops_at_the_target() {
    let tc = TurnConfig {
        rate_max: 80,
        rate_slew: 41,
        ..START
    };
    let mut st = SfState::default();
    let target = 32767 * 80 / 128;
    let mut prev = 0;
    for _ in 0..600 {
        sf_step(
            &cfg(tc),
            &SfInputs {
                steer: 32767,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!(st.v_href - prev, (target - prev).min(41));
        prev = st.v_href;
    }
    assert_eq!(st.v_href, target);
    // A stale link arrives as steer 0: the same slew is the stop ramp.
    sf_step(&cfg(tc), &run_input(), &mut st);
    assert_eq!(st.v_href, target - 41);
    // And left, the mirror.
    let mut st = SfState::default();
    sf_step(
        &cfg(tc),
        &SfInputs {
            steer: -32767,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!(st.v_href, -41);
}

#[test]
fn the_heading_error_accumulates_the_reference_less_the_fold_and_clamps_both_ways() {
    let tc = TurnConfig {
        rate_max: 80,
        rate_slew: 4096,
        head_clamp: 45,
        ..START
    };
    let mut st = SfState::default();
    sf_step(
        &cfg(tc),
        &SfInputs {
            steer: 1600,
            ..run_input()
        },
        &mut st,
    );
    let v = 1600 * 80 / 128;
    assert_eq!((st.v_href, st.head_err), (v, v));
    // One edge of right turn on each wheel: the left (peer) forward reads +1, the right (own)
    // backward reads +1. dH2 = 2, HE -= 20480.
    sf_step(
        &cfg(tc),
        &SfInputs {
            steer: 1600,
            wheel_pos: 1,
            peer_pos: 1,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!(st.head_err, 2 * v - 2 * 10240);
    // Turned far right: clamps at -head_clamp edges.
    let mut pos = 1;
    for _ in 0..100 {
        pos += 3;
        sf_step(
            &cfg(tc),
            &SfInputs {
                wheel_pos: pos,
                peer_pos: pos,
                ..run_input()
            },
            &mut st,
        );
    }
    assert_eq!(st.head_err, -45 * 20480);
    // Far left: +head_clamp.
    for _ in 0..100 {
        pos -= 3;
        sf_step(
            &cfg(tc),
            &SfInputs {
                wheel_pos: pos,
                peer_pos: pos,
                ..run_input()
            },
            &mut st,
        );
    }
    assert_eq!(st.head_err, 45 * 20480);
}

#[test]
fn head_clamp_zero_pins_the_heading_error_at_zero() {
    let tc = TurnConfig {
        head_clamp: 0,
        ..START
    };
    let mut st = SfState::default();
    for i in 0..200 {
        sf_step(
            &cfg(tc),
            &SfInputs {
                steer: 32767,
                wheel_pos: i,
                peer_pos: i,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!(st.head_err, 0);
    }
}

#[test]
fn straight_motion_does_not_move_the_heading() {
    let mut st = SfState::default();
    for i in 0..100 {
        // Forward on both: the right wheel counts down, the left up.
        sf_step(
            &cfg(START),
            &SfInputs {
                wheel_pos: -i,
                peer_pos: i,
                wheel_rate_q12: -Q,
                peer_rate_q12: Q,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!((st.head_err, st.vh_filt), (0, 0), "tick {i}");
    }
}

#[test]
fn the_rate_error_clamps() {
    let tc = TurnConfig {
        gains: [0, 10, 0],
        turn_max: 28500,
        ..START
    };
    let mut st = SfState {
        vh_filt: -100 * Q,
        ..primed(tc)
    };
    // Held at -100 counts: the filter stays put, the error clamps at +32 counts.
    sf_step(
        &cfg(tc),
        &SfInputs {
            // Left wheel 100 counts back, right 100 forward (its counter reads down forward).
            wheel_rate_q12: -100 * Q,
            peer_rate_q12: -100 * Q,
            scale: 3900,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!(st.vh_filt, -100 * Q);
    assert_eq!(st.turn_terms[1] as i64, 10 * 32 * 4096 / 256);
}

#[test]
fn the_heading_rate_filter_steps_by_a_quarter_of_the_difference() {
    let mut st = SfState::default();
    let mut prev = 0;
    for _ in 0..40 {
        // Left wheel forward at 1000, right wheel back at 1000: v_h = 1000.
        sf_step(
            &cfg(START),
            &SfInputs {
                wheel_rate_q12: 1000,
                peer_rate_q12: 1000,
                in_run: false,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!(st.vh_filt, prev + (1000 - prev) / 4);
        prev = st.vh_filt;
    }
    assert!(1000 - st.vh_filt < 4);
}

// ----- The producer and RUN gates -----

#[test]
fn the_pivoting_producer_forms_no_differential_and_its_cells_stay_zero() {
    let mut st = SfState::default();
    for i in 0..50 {
        let (_, d) = sf_step(
            &cfg(START),
            &SfInputs {
                leader: false,
                steer: 32767,
                wheel_pos: i,
                peer_pos: i,
                wheel_rate_q12: Q,
                peer_rate_q12: Q,
                ..run_input()
            },
            &mut st,
        );
        assert_eq!(d, 0);
        assert_eq!((st.v_href, st.head_err, st.turn_terms), (0, 0, [0, 0]));
    }
}

#[test]
fn every_differential_cell_is_zero_outside_run() {
    let mut st = SfState::default();
    for i in 0..50 {
        sf_step(
            &cfg(START),
            &SfInputs {
                steer: 32767,
                wheel_pos: i,
                ..run_input()
            },
            &mut st,
        );
    }
    assert!(st.v_href != 0 && st.head_err != 0);
    let (_, d) = sf_step(
        &cfg(START),
        &SfInputs {
            steer: 32767,
            in_run: false,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!((st.v_href, st.head_err, d), (0, 0, 0));
}

#[test]
fn the_common_half_is_byte_identical_with_the_difference_added() {
    // The same input stream through the law with the differential off and at a staged, saturating
    // configuration: the word and every common-half cell agree on every tick.
    let common = SfConfig {
        gains: [3491, 4002, 197, 926],
        speed_max: 80,
        speed_slew: 41,
        pos_clamp: 45,
        turn: TurnConfig::OFF,
    };
    let staged = SfConfig {
        turn: TurnConfig {
            gains: [2000, 10000, 10000],
            rate_max: 512,
            rate_slew: 4096,
            turn_max: 28500,
            head_clamp: 450,
        },
        ..common
    };
    let (mut a, mut b) = (SfState::default(), SfState::default());
    let mut seed: u32 = 0x1234_5678;
    let mut rnd = || {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (seed >> 8) as i32
    };
    let (mut own, mut peer) = (0i32, 0i32);
    for tick in 0..2000 {
        own = own.wrapping_add(rnd() % 5 - 2);
        peer = peer.wrapping_add(rnd() % 5 - 2);
        let inp = SfInputs {
            pitch_word: (rnd() % 2000) as i16,
            pitch_rate: rnd() % 20_000,
            wheel_rate_q12: rnd() % 40_000,
            peer_rate_q12: rnd() % 40_000,
            wheel_pos: own,
            peer_pos: peer,
            value: (rnd() % 65_535 - 32_767) as i16,
            steer: (rnd() % 65_535 - 32_767) as i16,
            in_run: tick % 300 > 20,
            ..run_input()
        };
        let (wa, _) = sf_step(&common, &inp, &mut a);
        let (wb, _) = sf_step(&staged, &inp, &mut b);
        assert_eq!(wa, wb, "tick {tick}");
        assert_eq!(
            (a.v_filt, a.v_ref, a.pos_err, a.live, a.terms),
            (b.v_filt, b.v_ref, b.pos_err, b.live, b.terms),
            "tick {tick}"
        );
    }
}

// ----- The ramp (section 6) -----

#[test]
fn the_turn_ramp_steps_each_gain_by_its_cap_and_lands_exactly() {
    let target = START.gains;
    let mut live = [0i16; 3];
    for _ in 0..100 {
        let next = turn_ramp(live, target);
        for i in 0..3 {
            let step = (next[i] - live[i]) as i32;
            if live[i] != target[i] {
                assert_eq!(step.abs(), TURN_CAPS[i] as i32);
            } else {
                assert_eq!(step, 0);
            }
        }
        live = next;
    }
    assert_eq!(live, target);
    // In RUN a tuning step ramps; outside RUN the live gains are the shadow's.
    let mut st = SfState::default();
    sf_step(
        &cfg(START),
        &SfInputs {
            in_run: false,
            ..run_input()
        },
        &mut st,
    );
    assert_eq!(st.turn_live, START.gains);
    let tuned = TurnConfig {
        gains: [150, 0, 30],
        ..START
    };
    sf_step(&cfg(tuned), &run_input(), &mut st);
    assert_eq!(st.turn_live, [51, 59, 1]);
}

#[test]
fn a_ramped_pass_moves_the_differential_under_the_slew_limit_and_an_unramped_one_does_not() {
    // Section 6's bound: a 45-edge heading error, one count of rate error and one count of
    // reference rate, at a battery word of 2400.
    let at = |g: [i16; 3]| {
        let t = turn_terms(g, 45 * 20480, Q, Q);
        word(t.iter().sum(), 2400) as i32
    };
    let mut live = [0i16; 3];
    let mut worst = 0;
    while live != GAIN_MAX {
        let next = turn_ramp(live, GAIN_MAX);
        worst = worst.max((at(next) - at(live)).abs());
        live = next;
    }
    assert!(worst < 250, "per-pass differential delta {worst}");
    assert!(worst >= 125, "and section 6's 125 is reached: {worst}");
    // The negative control: the start set applied in one pass.
    assert!((at(START.gains) - at([0; 3])).abs() > 250);
}

// ----- The split -----

#[test]
fn a_saturated_differential_keeps_the_common_torque_through_the_split() {
    let tc = TurnConfig {
        gains: [2000, 0, 0],
        turn_max: 28500,
        head_clamp: 450,
        ..START
    };
    let mut st = SfState {
        head_err: 450 * 20480,
        ..primed(tc)
    };
    let (_, d) = sf_step(&cfg(tc), &run_input(), &mut st);
    assert_eq!(d, 28500, "saturated at turn_max");
    for t in [-28_500i16, -20_000, -1, 0, 1, 20_000, 28_500] {
        for right in [true, false] {
            let (own, peer) = wheel_split(t, d, right);
            assert_eq!(own as i32 + peer as i32, 2 * t as i32, "t {t}");
        }
    }
}

// ----- The yaw plant (section 7) -----

/// The rover in yaw alone, as section 7 models it: 1.0 kg.m2 about the vertical, one numerator
/// count of differential `7.0e-4` N.m on each motor opposite ways on a 0.0825 m wheel and a
/// 0.6 m track. Integrated with RK4 inside each 4 ms tick, read through an 8 ms sensor delay; the
/// leader is the right wheel and the peer's words arrive every other tick (125 Hz). The rates are
/// the ideal ones, truncated to the word.
struct Yaw {
    /// `[psi, psi']`, rad, right turn positive.
    z: [f64; 2],
    hist: Vec<[f64; 2]>,
    st: SfState,
    tick: u32,
}

const J: f64 = 1.0;
const R: f64 = 0.0825;
const TRACK: f64 = 0.6;
/// N.m per numerator count per motor (assumption A1 at the nominal pack).
const C: f64 = 7.0e-4;
const DELAY: usize = 2;

fn edge() -> f64 {
    2.0 * core::f64::consts::PI * R / 90.0
}

fn count() -> f64 {
    50.0 * edge()
}

/// Heading in edges of section 2's unit, from the yaw angle.
fn h_edges(psi: f64) -> f64 {
    psi * TRACK / 2.0 / edge()
}

impl Yaw {
    fn new() -> Self {
        Yaw {
            z: [0.0; 2],
            hist: std::vec![[0.0; 2]; DELAY + 1],
            st: SfState::default(),
            tick: 0,
        }
    }

    fn inputs(&self, in_run: bool) -> SfInputs {
        let at = |k: usize| self.hist[self.hist.len() - 1 - k];
        let m = at(DELAY);
        let p = at(DELAY + (self.tick as usize % 2));
        // Left travel +psi*T/2, right travel -psi*T/2. The right wheel's counter reads down going
        // forward, the left's up.
        let right_travel = -m[0] * TRACK / 2.0;
        let left_travel = p[0] * TRACK / 2.0;
        let right_rate = -m[1] * TRACK / 2.0;
        let left_rate = p[1] * TRACK / 2.0;
        SfInputs {
            wheel_pos: -(right_travel / edge()).floor() as i32,
            peer_pos: (left_travel / edge()).floor() as i32,
            wheel_rate_q12: (-(right_rate / count() * Q as f64)) as i32,
            peer_rate_q12: (left_rate / count() * Q as f64) as i32,
            in_run,
            ..run_input()
        }
    }

    /// One tick under `drag`, an equivalent differential in numerator counts (positive yaws right).
    fn tick(&mut self, cfg: &SfConfig, drag: f64) {
        let inp = self.inputs(true);
        let (_, d) = sf_step(cfg, &inp, &mut self.st);
        assert!(d.abs() <= cfg.turn.turn_max as i32);
        // The word back to numerator counts, then to a yaw moment.
        let numer = d as f64 * NOMINAL as f64 / 3900.0;
        let moment = (numer + drag) * C / R * TRACK;
        let h = 0.004 / 20.0;
        for _ in 0..20 {
            let f = |z: [f64; 2]| [z[1], moment / J];
            let add = |z: [f64; 2], k: [f64; 2], s: f64| [z[0] + s * k[0], z[1] + s * k[1]];
            let k1 = f(self.z);
            let k2 = f(add(self.z, k1, h / 2.0));
            let k3 = f(add(self.z, k2, h / 2.0));
            let k4 = f(add(self.z, k3, h));
            self.z = core::array::from_fn(|i| {
                self.z[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
            });
        }
        self.hist.push(self.z);
        self.tick += 1;
    }

    fn engaged(cfg: &SfConfig) -> Self {
        let mut y = Yaw::new();
        let inp = y.inputs(false);
        sf_step(cfg, &inp, &mut y.st);
        for _ in 0..50 {
            y.tick(cfg, 0.0);
        }
        y
    }
}

#[test]
fn a_quarter_turn_heading_step_settles_inside_one_and_a_half_seconds_without_overshoot() {
    // The start set's gains; the clamp opened to 90 edges so the quarter turn (82 edges) is a
    // step the error sees whole rather than one the 45-edge clamp cuts.
    let c = cfg(TurnConfig {
        head_clamp: 90,
        ..START
    });
    let mut y = Yaw::engaged(&c);
    let step = core::f64::consts::FRAC_PI_2;
    y.z[0] += step;
    let start = h_edges(step);
    let mut overshoot: f64 = 0.0;
    let mut settled_at = None;
    for t in 1..=1000u32 {
        y.tick(&c, 0.0);
        let h = h_edges(y.z[0]);
        overshoot = overshoot.max(-h);
        if h.abs() > 0.05 * start {
            settled_at = None;
        } else if settled_at.is_none() {
            settled_at = Some(t);
        }
    }
    let settled = settled_at.expect("settles") as f64 / 250.0;
    std::println!(
        "quarter-turn step ({start:.1} edges): inside 5 % at {settled:.3} s, overshoot {overshoot:.2} edges"
    );
    assert!(settled < 1.5, "settles in {settled} s");
    assert!(overshoot < 0.1 * start, "overshoot {overshoot} edges");
}

#[test]
fn a_held_drag_parks_at_the_residual_and_releases_back_without_overshoot() {
    // Section 7's residual example: gh 100, so 3,000 counts park 30 edges off; ghv at the
    // model's critical 120.
    let c = cfg(TurnConfig {
        gains: [100, 120, 0],
        ..START
    });
    let mut y = Yaw::engaged(&c);
    // A left-yawing drag (the right wheel held), held 4 s.
    for _ in 0..1000 {
        y.tick(&c, -3000.0);
    }
    let parked = h_edges(y.z[0]);
    let want = -3000.0 / 100.0;
    std::println!("3,000-count drag: parked {parked:.2} edges against {want}");
    assert!((parked - want).abs() < 0.1 * want.abs(), "parked {parked}");
    let mut most: f64 = f64::MIN;
    for _ in 0..1000 {
        y.tick(&c, 0.0);
        most = most.max(h_edges(y.z[0]));
    }
    std::println!(
        "released: back to {:.2} edges, most {most:.2}",
        h_edges(y.z[0])
    );
    assert!(h_edges(y.z[0]).abs() < 2.0, "back: {}", h_edges(y.z[0]));
    assert!(most < 2.0, "no overshoot past the heading: {most}");
}
