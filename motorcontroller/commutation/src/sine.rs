//! The sinusoidal (open-loop) arm, recovered from the archive.
//!
//! Per control period: take the electrical angle interpolated from the halls by the shared
//! [`crate::foc::RotorFrontEnd`], look up sine values from the 256-entry Q15 quarter-wave table
//! (via [`crate::foc::lookup_sincos`]), scale by the voltage demand, and produce three
//! 120-degree-shifted phase duties on the 0..ARR scale. No current loop, no injected ADC.
//!
//! This is the same modulation as FOC's inverse path but with the demand applied OPEN-LOOP (a
//! voltage amplitude) instead of through the current PI. It reuses the FOC angle front-end and
//! the FOC sine table, so there is no second table or second hall path to maintain. Pure sine
//! (the decided divergence from EFeru's saddle tables; `specs/commutation.md`, "Sine arm").
//!
//! The neutral is centered: each phase sits at mid-rail (ARR/2) and swings about it by
//! `amplitude * sin(theta + phase_offset)`. A zero demand leaves all three at mid-rail (all
//! phases DRIVEN, centered: drive-free but stiff, unlike six-step's coast).

use crate::foc::lookup_sincos;
use crate::{MotorOutput, PhaseCmd, ARR, MID_RAIL};

/// 120 degrees in 16-bit angle units (65536 / 3 = 21845.33, truncated). Phase B trails A by this;
/// phase C leads A by this (equivalently trails by 240 deg).
pub const PHASE_120: u16 = 0x5555; // 21845

/// Scale a signed Q15 sine value by the (unsigned) duty amplitude to a duty OFFSET about
/// mid-rail. `sin_q15` is in [-32767, +32767]; `amp` is the peak duty swing in counts
/// (<= MID_RAIL). Returns the signed offset `round(sin_q15 * amp / 32767)`.
#[inline]
fn sine_offset(sin_q15: i16, amp: u16) -> i32 {
    // amp <= MID_RAIL (1125), sin_q15 fits i16, so the product fits i32 comfortably.
    let prod = sin_q15 as i32 * amp as i32;
    // Round-to-nearest divide by 32767 (toward-zero bias handled symmetrically).
    let bias = if prod >= 0 { 32767 / 2 } else { -(32767 / 2) };
    (prod + bias) / 32767
}

/// Map the signed drive demand to the peak duty amplitude about mid-rail, saturated to
/// [`MID_RAIL`] so a phase never leaves 0..ARR (the demand is a voltage/throttle amplitude on
/// roughly the +-32767 frame scale).
#[inline]
pub fn demand_to_amplitude(demand: i32) -> u16 {
    let mag = demand.unsigned_abs();
    let scaled = (mag as u64 * MID_RAIL as u64) / 32767u64;
    if scaled > MID_RAIL as u64 {
        MID_RAIL
    } else {
        scaled as u16
    }
}

/// One sinusoidal control step. `theta` is the interpolated 16-bit electrical angle from the
/// shared front-end; `demand` is the signed voltage/throttle demand. Produces three
/// 120-degree-shifted phase duties centered on mid-rail and scaled by the demand. A negative
/// demand reverses the field (negates the carrier), which spins the motor the other way. A zero
/// demand -> all three at mid-rail. No current loop, no ADC.
#[inline]
pub fn sine_step(theta: u16, demand: i32) -> MotorOutput {
    let amp = demand_to_amplitude(demand);
    let sign: i32 = if demand < 0 { -1 } else { 1 };

    // Phase A at theta, B at theta-120, C at theta+120 (lookup_sincos returns (sin, cos); the
    // sine component is the phase carrier).
    let (sa, _) = lookup_sincos(theta);
    let (sb, _) = lookup_sincos(theta.wrapping_sub(PHASE_120));
    let (sc, _) = lookup_sincos(theta.wrapping_add(PHASE_120));

    let oa = sign * sine_offset(sa, amp);
    let ob = sign * sine_offset(sb, amp);
    let oc = sign * sine_offset(sc, amp);

    let to_duty = |off: i32| -> PhaseCmd {
        PhaseCmd::Drive((MID_RAIL as i32 + off).clamp(0, ARR as i32) as u16)
    };
    MotorOutput {
        phases: [to_duty(oa), to_duty(ob), to_duty(oc)],
    }
}
