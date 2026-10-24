//! The outer speed loop (`specs/control.md` (n)): a slow PI on wheel speed whose output is the
//! commanded lean, sitting over the unchanged pitch PD.
//!
//! Without it the pitch loop is the only loop, so a commanded lean is an ACCELERATION command: a
//! wheeled inverted pendulum held at lean `L` accelerates its base at `g tan L` for as long as the
//! lean is held, a slope's steady torque can only be answered by a steady pitch error that leans
//! into the hill, and a reversed stick asks for the opposite acceleration rather than the opposite
//! speed.
//!
//! The loop meets the cascade at [`DriveLean`](crate::DriveLean)'s commanded lean
//! (`specs/control.md` (h)), which is already a bounded, slewed equilibrium pitch in centidegrees
//! converted into the shaper's `off` units through the live `kp`. When the loop is live the STICK
//! commands speed and this is the lean's producer; when it is not ([`OuterSpeed::live`] is false at
//! `speed_kp == 0`) the stick commands lean directly and nothing here runs.
//!
//! This is OURS, not stock. The recovered leaky integrator in [`speed`](crate::speed) is a
//! different thing at a different seam: it has no speed reference, it ramps at a fixed rate while
//! the two wheels turn in OPPOSING directions (a deck twisted underfoot), and its output moves the
//! pitch error rather than the commanded lean. (n) records why it keeps no producer.
//!
//! Units, fixed by the measurement. The speed is the period ISR's signed hall EDGE COUNT per
//! 20.06 ms window ([`crate::speed`]'s consumer notes and `specs/link-control.md`), FORWARD
//! POSITIVE after [`wheel_forward`]. One count works out at 0.287 m/s on a 165 mm hub with 15 pole
//! pairs, which is an ASSUMPTION about the fleet's wheels and not a measurement: what a board
//! actually turns is `store::MOTOR_POLE_PAIRS` and `store::WHEEL_DIAMETER_MM`, both of which
//! default to unset. The firmware does not know the figure and does not need it: everything here
//! is counts, and the filter state and the error carry [`cfg::Q`], 1/4096 of one.

use crate::config::outer as cfg;
use crate::helpers::clamp_sym;

/// This board's wheel speed in FORWARD-POSITIVE counts, from the block's wheel-speed word and the
/// board's `motor.direction` mirror.
///
/// # Why the mirror is `motor.direction`'s and not a field of its own
///
/// The six-step decode drives sector `(HALL_TO_SECTOR[code] + offset + half) % 6` with `half = 3`
/// for `Direction::Reverse`, and a negative demand flips the effective direction
/// (`crates/commutation/src/sixstep.rs`), while the rotor-side counter increments on `FWD_NEXT`,
/// which is the increasing-sector neighbour. So on a board whose staged direction and align offset
/// make the motor commutate at all, a POSITIVE demand turns the wheel the way the counter counts UP
/// when `motor.direction = 0` and DOWN when it is 1. A NEGATIVE demand drives a balancing board's
/// wheel forward (`specs/control.md` (a), measured twice on silicon), so forward motion reads
/// negative on a `motor.direction = 0` board and positive on a `motor.direction = 1` board.
///
/// The derivation is an inference from the commutation front end, not a measurement, and a board
/// whose hall wiring is mirrored relative to its phase wiring would break it. A sign error here is
/// positive feedback in the outer loop, so (n)'s first bench gate is a hand push with the board
/// DISARMED, reading `motor_speed` (`CTRL_OBS` word 24), and it passes before any energised one
/// runs.
#[inline]
pub const fn wheel_forward(word: i16, reverse: bool) -> i32 {
    if reverse {
        word as i32
    } else {
        -(word as i32)
    }
}

/// The loop's live configuration: the three live-tunable words of the `CONTROL_SPEED_LOOP` row
/// (`store::CONTROL_SPEED_LOOP`, 0x76), as [`GainShadow`](crate::GainShadow) row 2 holds them.
///
/// Built fresh from the shadow on every pass, because the row is on the live tune lane and a write
/// takes effect on the next tick (the engagement machine's gain RAMP does not apply: it bounds a
/// torque delta derived from the PID's own divisors, and these words move a commanded lean, whose
/// rate `lean_slew` already bounds).
///
/// The range is the shadow's and only the shadow's: it clamps on the way in at boot and REFUSES an
/// out-of-range tune write, against the compiled [`cfg::SPEED_GAIN_MAX`]. That is what makes the
/// `i32` bounds in [`Self::step`] facts rather than hopes, which the const assertions below pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct OuterSpeed {
    /// Index 0, `speed_kp`: centidegrees of commanded lean per count of speed error. A lean of one
    /// degree accelerates the body at `g tan 1 deg` = 0.171 m/s^2 and one count is 0.287 m/s on the
    /// assumed 165 mm / 15-pole-pair hub (see the module header), so 100 closes the speed loop with
    /// a time constant of about 1.7 s. The gain itself is in counts and holds whatever the wheel
    /// turns out to be; only the seconds move with it.
    kp: i32,
    /// Index 1, `speed_ki`: centidegrees of lean per count-SECOND of speed error. Its state is a
    /// displacement, so it also sets how far the machine rolls before it parks on a slope: a 5.7
    /// degree lean costs `570 / ki` count-seconds of drift.
    ki: i32,
    /// Index 2, `speed_max`: SIXTEENTHS of a count at full stick, so 16 is one count, which on the
    /// assumed hub is 0.287 m/s and makes 80 about walking pace. Zero is legal and useful: the
    /// machine station-keeps and the stick does nothing.
    speed_max: i32,
}

// The two products of [`OuterSpeed::step`] that could overflow, pinned against the compiled maxima
// and the error clamp that bound them. 128 counts is 36 m/s, so the clamp never binds on a machine;
// it is here for exactly this.
const _: () = assert!(
    (cfg::SPEED_GAIN_MAX[0] as i64) * (cfg::ERR_CLAMP as i64) <= i32::MAX as i64,
    "speed_kp * e must fit i32"
);
const _: () = assert!(
    (cfg::SPEED_GAIN_MAX[1] as i64) * (cfg::ERR_CLAMP as i64) <= i32::MAX as i64,
    "speed_ki * e must fit i32"
);

impl OuterSpeed {
    /// The configuration one [`GainShadow`](crate::GainShadow) speed row names. No clamp here: the
    /// shadow is the single owner of the range (see the type doc).
    pub const fn of_row(row: [i16; crate::config::GAINS_PER_PROFILE]) -> Self {
        OuterSpeed {
            kp: row[0] as i32,
            ki: row[1] as i32,
            speed_max: row[2] as i32,
        }
    }

    /// Whether the loop runs this pass. `speed_kp == 0` is the whole off switch, and with it off
    /// the image is byte-identical to the one before (n): the stick commands lean through (h) and
    /// the carries stay at zero. An integral-only configuration is unreachable by construction,
    /// deliberately, because a loop with no proportional term wanders.
    #[inline]
    pub const fn live(&self) -> bool {
        self.kp != 0
    }

    /// The target speed this stick position asks for, in [`cfg::Q`] units, FORWARD POSITIVE.
    /// `value * speed_max / 128` is `value / 32767 * speed_max / 16 * 4096` to within the
    /// truncation, and `32767 * 32767` fits `i32`, so the whole thing is one multiply and one
    /// divide with no widening.
    #[inline]
    pub const fn target(&self, value: i16) -> i32 {
        (value as i32 * self.speed_max) / cfg::STICK_DIV
    }

    /// One pass. `v` is this board's wheel speed in forward-positive counts ([`wheel_forward`]),
    /// `value` the effective `DRIVE_CMD.value` (already neutral once stale, so a dropped link asks
    /// for zero SPEED and the machine stops and holds), `in_run` whether the engagement machine is
    /// in RUN sub-state 3 this pass, and `lean_max` the commanded-lean authority
    /// ([`DriveLean::lean_max`](crate::DriveLean::lean_max)). Returns the commanded lean in
    /// centidegrees, forward positive, for [`DriveLean::step_lean`](crate::DriveLean::step_lean) to
    /// bound and slew.
    ///
    /// # The two state rules
    ///
    /// **The filter runs every tick, in every sub-state**, because it is a measurement and not a
    /// controller state, so it is converged at the moment the machine engages. Its time constant is
    /// [`cfg::FILT_DIV`] ticks, 128 ms, about 6.4 of the 20.06 ms measurement windows: enough to
    /// resolve roughly 1/6 of a count out of a one-count quantiser (the information about sub-count
    /// speed is in the ratio of 0s to 1s across successive windows) and to put the window-boundary
    /// dither about 20-fold down at the output.
    ///
    /// **The integrator accumulates only in RUN and the cell is zeroed on every other tick**, which
    /// is the recovered integrator's own discipline (`specs/control.md` (d) item 2) and answers
    /// every reset the loop needs with one rule: a disarm, a fall through the tilt cutout's latch,
    /// the fallen dwell while that latch holds, and the soft start are all not-RUN, so no new reset
    /// edge exists to get wrong. `ki == 0` clears it too (the `base::pi` rule), which is a live
    /// control for dumping a wound integrator without disarming.
    ///
    /// The accumulator's bound IS the output bound, so saturation and windup are one limit: the
    /// integrator alone can reach full authority and no further, and unwinding takes as long as
    /// winding did.
    pub fn step(
        &self,
        v: i32,
        value: i16,
        in_run: bool,
        lean_max: i16,
        st: &mut OuterSpeedState,
    ) -> i32 {
        st.filt += (v * cfg::Q - st.filt) / cfg::FILT_DIV;
        let e = clamp_sym(self.target(value) - st.filt, cfg::ERR_CLAMP);
        st.acc = if in_run && self.ki != 0 {
            clamp_sym(
                st.acc + self.ki * e / cfg::TICK_HZ,
                lean_max as i32 * cfg::Q,
            )
        } else {
            0
        };
        self.kp * e / cfg::Q + st.acc / cfg::Q
    }
}

/// The loop's two carries. Not reset on the OFF pass and not reset with the cascade's other
/// carries: the integrator is zeroed by [`OuterSpeed::step`]'s own RUN rule every tick it is not
/// running, and the filter is a measurement that should be converged before the machine engages.
/// It IS replaced wholesale at an arm that changed the active producer, with the records beside it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OuterSpeedState {
    /// The filtered wheel speed in [`cfg::Q`] units, forward positive.
    pub filt: i32,
    /// The integrator, in [`cfg::Q`] units of CENTIDEGREES (the output's unit times [`cfg::Q`], so the
    /// bound is `lean_max * cfg::Q` and the term is `acc / cfg::Q`).
    pub acc: i32,
}
