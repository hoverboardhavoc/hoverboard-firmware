//! Speed/steer loop (Section 5 / `FUN_080013e0`) plus the Section-5.1 setpoint helper.
//!
//! REBUILT TO THE BINARY (slice-4 audit ruling; `specs/control.md` (d) "Speed/steer loop" is
//! the folded contract, whose source note records how stock section 5 and the archive both
//! mis-described this routine). Three float-carried states, Q-modeled per spec (f), with the
//! EABI-d2iz conversion at CONSUMPTION only ([`q_to_int_d2iz`]):
//!
//! 1. the pp BLEND, a one-pole IIR run EVERY tick (`0.4*x + 0.6*prev`, float carry @+0x38);
//! 2. the leaky INTEGRATOR (`*0.9996 [+-1.2]`, float carry @+0x20), gated to RUN sub-state 3
//!    (the CELL is zeroed otherwise) and branched by the opposing-signs predicate over
//!    (s1, s2) vs the +-W/5 deadband, under the gate byte;
//! 3. the CORRECTION sum `f2iz(blend) + trim + f2iz(acc)` (i16 arithmetic), every tick: this
//!    IS section 3.2's `pp` producer, NOT forced to zero outside RUN;
//! 4. the DIRECTION/brake word, float-typed: +-30.0 band (in-band -> 0), opposing wheel-speed
//!    signs accumulate via float add; the cell is zeroed outside RUN sub-state 3.
//!
//! The fractional coefficients (0.4/0.6, 0.9996, 1.2) are "(float in original)" and reproduced
//! in Q at their use sites (flagged); integer contract constants live in `config::speed`.
//! There is NO peer-speed blend in this loop (the spec's re-attribution to the FSM sub-2
//! reference blend).

use crate::config::speed as speedc;
use crate::helpers::q_to_int_d2iz;
use base::fixed::Fix;

/// Persistent speed-loop state: the binary's float cells, Q-modeled (spec (f); the Q-vs-single
/// carry-width bound is the (f) analytic clause).
#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedState {
    /// @+0x38: the pp-blend one-pole carry (float in original -> Q). Updated EVERY tick.
    pub blend: Fix,
    /// @+0x20: the leaky-integrator carry (float in original -> Q). The CELL is zeroed outside
    /// RUN sub-state 3; the carry keeps fraction (an int cell either locks up below
    /// |acc| ~1250 or over-decays, the slice-4 finding).
    pub acc: Fix,
    /// @+0x34: the direction/brake word (float in original -> Q). Zeroed outside RUN
    /// sub-state 3.
    pub direction: Fix,
    /// @-0x4c: the correction sum (`pp`), i16 as in the binary's short arithmetic. Updated
    /// EVERY tick.
    pub correction: i16,
}

/// Inputs to the speed/steer loop (parameterized per the folded spec (d): gate byte, W, s1/s2,
/// the blend input, trim, wheel speeds).
#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedInputs {
    /// The blend input `x` (the published pitch float in the binary; float -> Q).
    pub blend_input: Fix,
    /// The trim field added into the correction sum (@-0x5c; section 3.2's `field@0x24`).
    pub trim: i16,
    /// The gate byte (@-0x7e): the integrator's add/subtract branches require it nonzero.
    pub gate: bool,
    /// `s1` (@-0x46): the first signed input of the opposing-signs predicate.
    pub s1: i16,
    /// `s2` (@-0x44): the second signed input of the opposing-signs predicate.
    pub s2: i16,
    /// `W` (@-0x66), unsigned halfword; the deadband half-width is `thr = W/5` (unsigned).
    pub window: u16,
    /// Wheel-speed short A (@-0x42): opposing signs vs B select the direction accumulate path.
    pub wheel_a: i16,
    /// Wheel-speed short B (@-0x40).
    pub wheel_b: i16,
    /// The direction accumulate step (the opposing-signs float add; the decompiler drops the
    /// i2f operand's source, so the added value is a parameterized input; float -> Q).
    pub dir_step: Fix,
    /// The out-of-band POSITIVE direction result (`direction > +30`): the decompile's
    /// argument-dropped float call makes the assigned value unreadable, so it is parameterized
    /// (stock section 14: the branch/sign details are read structurally).
    pub dir_out_pos: Fix,
    /// The out-of-band NEGATIVE direction result (`direction < -30`), parameterized as above.
    pub dir_out_neg: Fix,
    /// True when system mode == RUN (3) AND run sub-state == 3.
    pub run_active: bool,
}

/// One speed/steer-loop tick (the folded spec (d) order). Mutates the persistent state.
pub fn speed_loop(inp: &SpeedInputs, st: &mut SpeedState) {
    // (1) The pp blend: EVERY tick, unconditionally (amendment A1; the PID needs pp every
    // tick). blend = 0.4*x + 0.6*blend_prev. FLAGGED: floats in original
    // (0x3FD99999_9999999A / 0x3FE33333_33333333) -> Q.
    st.blend = Fix::from_num(0.4) * inp.blend_input + Fix::from_num(0.6) * st.blend;
    // One d2iz consumption of the carry feeds the correction sum (the binary's single f2iz of
    // the narrowed single; short store).
    let blend_i = q_to_int_d2iz(st.blend) as i16;

    // (2) The leaky integrator: RUN sub-state 3 only; the CELL is zeroed otherwise.
    if inp.run_active {
        // FLAGGED: 0.9996 (0x3FEFFCB9_23A29C78) and 1.2 (0x3FF33333_33333333) floats -> Q.
        let decayed = st.acc * Fix::from_num(0.9996);
        let thr = (inp.window / speedc::DEADBAND_DIVISOR) as i32;
        let (s1, s2) = (inp.s1 as i32, inp.s2 as i32);
        st.acc = if inp.gate && s1 > thr && s2 < -thr {
            decayed + Fix::from_num(1.2) // Add: opposing signs, s1 high side
        } else if inp.gate && s1 < -thr && s2 > thr {
            decayed - Fix::from_num(1.2) // Subtract: opposing signs, s1 low side
        } else {
            decayed // in-band / same-sign / gate off: decay only
        };
    } else {
        st.acc = Fix::ZERO;
    }

    // (3) The correction sum (pp): every tick, i16 arithmetic as in the binary's short adds.
    // Outside RUN the zeroed acc cell contributes 0, so the sum degrades to blend + trim.
    let acc_i = q_to_int_d2iz(st.acc) as i16;
    st.correction = blend_i.wrapping_add(inp.trim).wrapping_add(acc_i);

    // (4) The direction/brake word: RUN sub-state 3 only, zeroed otherwise. Opposing
    // wheel-speed signs accumulate via float add; else the +-30.0 band: in-band -> 0,
    // out-of-band -> the parameterized results (see the SpeedInputs docs).
    if inp.run_active {
        let opposing = (inp.wheel_a > 0 && inp.wheel_b < 0) || (inp.wheel_a < 0 && inp.wheel_b > 0);
        if opposing {
            st.direction += inp.dir_step;
        } else if st.direction > Fix::from_num(speedc::DIRECTION_BAND) {
            st.direction = inp.dir_out_pos;
        } else if st.direction < Fix::from_num(-speedc::DIRECTION_BAND) {
            st.direction = inp.dir_out_neg;
        } else {
            st.direction = Fix::ZERO;
        }
    } else {
        st.direction = Fix::ZERO;
    }
}

/// Section 5.1 (`FUN_08004c2c`), pure integer: per axis `sp = (u16)measured - 2*target`,
/// SATURATED to `+0x7FFF` / `-0x7FFF` (the 0x8001 halfword; NEVER -0x8000) at the +-0x8000
/// thresholds; the two setpoints are the binary's packed halfword pair. (The helper's target
/// pointers are `0x4001243C`/`0x40012440`; peripheral mapping unverified, recorded in the
/// spec's source note.)
pub fn speed_setpoint(measured: [u16; 2], target: [i32; 2]) -> [i16; 2] {
    [
        axis_setpoint(measured[0], target[0]),
        axis_setpoint(measured[1], target[1]),
    ]
}

/// One axis of [`speed_setpoint`].
fn axis_setpoint(measured: u16, target: i32) -> i16 {
    let v = (measured as i32) - 2 * target;
    if v >= speedc::SETPOINT_THRESHOLD {
        speedc::SETPOINT_SAT
    } else if v <= -speedc::SETPOINT_THRESHOLD {
        -speedc::SETPOINT_SAT
    } else {
        v as i16
    }
}
