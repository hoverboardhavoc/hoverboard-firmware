//! The throttle mode's input conditioning (`specs/control.md` (b)): EFeru-parity
//! rate-limit -> low-pass -> speed/steer mixer, never touching the IMU. This is the phase's
//! one NEW construction (nothing to recover: the stock firmware has no throttle mode) and the
//! phase's ONLY EFeru-oracle surface: the three core functions are ports of EFeru's
//! `rateLimiter16` / `filtLowPass32` / `mixerFcn` (`reference/efferu-hoverboard/Src/util.c`
//! @ `a0751d589fd43d8975eda3683fac21a44bbfe8fa`, lines 1642-1723), keeping EFeru's fixdt
//! shapes verbatim per spec (f) (i16/i32/i64 integer arithmetic with EFeru's own shift
//! semantics, incl. its FLOORING `>>` on negatives: these are EFeru's integer shapes, not
//! stock float->int conversions, so the d2iz clause does not apply). The committed fixtures
//! replay the gitignored `reference/efferu-oracle/throttle_harness.c` oracle's observed
//! behavior.
//!
//! # The frame adapters (this port's own seams, named)
//!
//! Spec (b) fixes the frames: inputs arrive on the +-32767 frame scale and the mode's output
//! lands on the +-28500 word the shared output stage envelopes. EFeru's pipeline is built for
//! its +-1000 command domain (`INPUT_MIN/MAX`, util.c:271-272; the fixdt(1,16,4) rate-limiter
//! state cannot even hold +-32767). So the port brackets the verbatim EFeru core with two
//! exact-rational integer scale seams, OURS not EFeru's:
//! - in: `cmd = frame * 1000 / 32767` (truncating; +-32767 -> +-1000 exactly at the rails);
//! - out: `reference = cmd * 57 / 2` (truncating; +-1000 -> +-28500 exactly at the rails).
//!
//! The shared output stage's envelope (cap 28500) remains the +-28500 clamp of record.

use crate::config::throttle as cfgc;

/// The conditioning constants (EFeru's adopted defaults from `config::throttle`; a tunable
/// config surface per spec (b), EFeru fixdt shapes documented per constant).
#[derive(Clone, Copy, Debug)]
pub struct ThrottleConfig {
    /// Rate-limiter step, fixdt(1,16,4) (480 = 30.0 units/tick).
    pub rate: i16,
    /// Low-pass coefficient, fixdt(0,16,16) (6553 = 0.1).
    pub filter: u16,
    /// Mixer speed coefficient, fixdt(1,16,14) (16384 = 1.0).
    pub speed_coeff: i16,
    /// Mixer steer coefficient, fixdt(1,16,14) (8192 = 0.5).
    pub steer_coeff: i16,
}

impl Default for ThrottleConfig {
    fn default() -> Self {
        Self {
            rate: cfgc::RATE,
            filter: cfgc::FILTER_COEF,
            speed_coeff: cfgc::SPEED_COEFFICIENT,
            steer_coeff: cfgc::STEER_COEFFICIENT,
        }
    }
}

/// The per-tick conditioning state (EFeru's four fixdt carries, main.c:155-158).
#[derive(Clone, Copy, Debug, Default)]
pub struct ThrottleState {
    /// Steer rate-limiter carry, fixdt(1,16,4).
    pub steer_rate_fixdt: i16,
    /// Speed rate-limiter carry, fixdt(1,16,4).
    pub speed_rate_fixdt: i16,
    /// Steer low-pass carry, fixdt(1,32,16).
    pub steer_fixdt: i32,
    /// Speed low-pass carry, fixdt(1,32,16).
    pub speed_fixdt: i32,
}

/// One tick's conditioned outputs: the EFeru-domain commands (+-1000) and the +-28500
/// references the shared output stage envelopes. The left/right split is EFeru's mixer
/// (`speed +- steer`); which side feeds THIS board's engagement machine is the caller's role
/// fact.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ThrottleOutput {
    /// Right command in the EFeru +-1000 domain (`speed - steer` side).
    pub cmd_right: i16,
    /// Left command in the EFeru +-1000 domain (`speed + steer` side).
    pub cmd_left: i16,
    /// Right reference on the +-28500 word scale.
    pub ref_right: i32,
    /// Left reference on the +-28500 word scale.
    pub ref_left: i32,
}

/// EFeru `rateLimiter16` (util.c:1682-1698), bit-shape-exact: `y` is fixdt(1,16,4); the delta
/// `(u << 4) - y` (computed in i32, assigned back through the i16 wrap exactly as the C int ->
/// int16_t conversion does) is clamped to +-rate and accumulated.
pub fn rate_limiter16(u: i16, rate: i16, y: &mut i16) {
    let mut q0 = (((u as i32) << 4).wrapping_sub(*y as i32)) as i16;
    if q0 > rate {
        q0 = rate;
    } else if q0 < -rate {
        q0 = -rate;
    }
    *y = q0.wrapping_add(*y);
}

/// EFeru `filtLowPass32` (util.c:1658-1663), bit-shape-exact: `y` is fixdt(1,32,16);
/// `tmp = ((i64)((u << 4) - (y >> 12)) * coef) >> 4` (EFeru's arithmetic shifts: the `>> 12`
/// and `>> 4` FLOOR on negatives, pinned by the oracle's asymmetric negative fixture), clamped
/// to the i32 range, accumulated with the C wrap semantics.
pub fn filt_low_pass32(u: i32, coef: u16, y: &mut i32) {
    let inner = (u.wrapping_shl(4)).wrapping_sub(*y >> 12) as i64;
    let mut tmp = (inner * coef as i64) >> 4;
    tmp = tmp.clamp(i32::MIN as i64, i32::MAX as i64);
    *y = (tmp as i32).wrapping_add(*y);
}

/// EFeru `mixerFcn` (util.c:1706-1723), bit-shape-exact: inputs are the fixdt(1,16,4) values
/// (`speed << 4`, `steer << 4` at the call seam, main.c:350); the coefficient products are
/// `>> 14` with the C int -> int16_t wrap, the sums clamp to the i16 range, the `>> 4`
/// conversions FLOOR (EFeru's shape), and the commands clamp to the +-1000 input domain.
/// Returns `(cmd_right, cmd_left)` = (`speed - steer`, `speed + steer`).
pub fn mixer_fcn(rtu_speed: i16, rtu_steer: i16, speed_coeff: i16, steer_coeff: i16) -> (i16, i16) {
    let prod_speed = (((rtu_speed as i32) * (speed_coeff as i32)) >> 14) as i16;
    let prod_steer = (((rtu_steer as i32) * (steer_coeff as i32)) >> 14) as i16;

    let mut tmp = (prod_speed as i32) - (prod_steer as i32);
    tmp = tmp.clamp(-32768, 32767);
    let mut cmd_right = (tmp >> 4) as i16;
    cmd_right = cmd_right.clamp(-cfgc::CMD_LIMIT, cfgc::CMD_LIMIT);

    let mut tmp = (prod_speed as i32) + (prod_steer as i32);
    tmp = tmp.clamp(-32768, 32767);
    let mut cmd_left = (tmp >> 4) as i16;
    cmd_left = cmd_left.clamp(-cfgc::CMD_LIMIT, cfgc::CMD_LIMIT);

    (cmd_right, cmd_left)
}

/// One conditioning tick (the EFeru main-loop order, main.c:318-321 + 350, bracketed by the
/// port's frame adapters): frame-in scale, rate limit, low-pass, integer readback (`>> 16`),
/// mix, frame-out scale. Never touches the IMU; the outputs feed the shared engagement
/// machine's mirror, whose envelope is the +-28500 clamp of record.
pub fn throttle_tick(
    cfg: &ThrottleConfig,
    speed_in: i16,
    steer_in: i16,
    st: &mut ThrottleState,
) -> ThrottleOutput {
    // Frame in: +-32767 -> the EFeru +-1000 command domain (exact at the rails, truncating).
    let speed_cmd = ((speed_in as i32) * (cfgc::CMD_LIMIT as i32) / cfgc::FRAME_IN_MAX) as i16;
    let steer_cmd = ((steer_in as i32) * (cfgc::CMD_LIMIT as i32) / cfgc::FRAME_IN_MAX) as i16;

    // The EFeru pipeline verbatim (rate limit -> low-pass -> >>16 readback -> mixer).
    rate_limiter16(steer_cmd, cfg.rate, &mut st.steer_rate_fixdt);
    rate_limiter16(speed_cmd, cfg.rate, &mut st.speed_rate_fixdt);
    filt_low_pass32(
        (st.steer_rate_fixdt >> 4) as i32,
        cfg.filter,
        &mut st.steer_fixdt,
    );
    filt_low_pass32(
        (st.speed_rate_fixdt >> 4) as i32,
        cfg.filter,
        &mut st.speed_fixdt,
    );
    let steer = (st.steer_fixdt >> 16) as i16;
    let speed = (st.speed_fixdt >> 16) as i16;
    let (cmd_right, cmd_left) = mixer_fcn(
        speed.wrapping_shl(4),
        steer.wrapping_shl(4),
        cfg.speed_coeff,
        cfg.steer_coeff,
    );

    // Frame out: +-1000 -> the +-28500 word (exact at the rails, truncating).
    ThrottleOutput {
        cmd_right,
        cmd_left,
        ref_right: (cmd_right as i32) * cfgc::REF_SCALE_NUM / cfgc::REF_SCALE_DEN,
        ref_left: (cmd_left as i32) * cfgc::REF_SCALE_NUM / cfgc::REF_SCALE_DEN,
    }
}
