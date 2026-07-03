//! The six-step (trapezoidal, hall-driven block commutation) arm, the DEFAULT method.
//!
//! Six-step needs NO current sensing: a hall code plus a voltage (duty) demand is enough to turn
//! the motor. It is the low-risk first-motion path, selectable before phase-current sensing is
//! validated on a board.
//!
//! **The contract is the silicon-proven runtime-hal example choreography**
//! (`specs/commutation.md`, "Six-step arm"; `runtime-hal/examples/crates/control`), superseding
//! the archived mid-rail arm: per valid sector exactly **one Pwm source** (`Drive(duty)`), **one
//! sink** (`Drive(0)`: complementary low side on), and **one floating phase** (`Float`, true
//! high-Z); an invalid hall code (0/7) coasts (all-`Float`); zero demand coasts too (no drive
//! without demand; hold/brake semantics are a future deliberate feature, spec "Open questions").
//! `Direction::Reverse` is +3 through the six-state sequence (source and sink swap, same float);
//! the bench-swept **align offset** (0..5 sector rotation) absorbs hall-to-phase alignment. A
//! negative demand flips the effective direction (the sign is the drive direction, the magnitude
//! the duty).
//!
//! The decode (states, hall->sector table, direction/offset composition) is the example's,
//! expressed as pure data; the consistency test ties it to the shared front-end's `BASE_ANGLE`
//! anchors (the drive vector advances +60 deg per forward hall step).

use crate::{MotorOutput, PhaseCmd, ARR};

/// Per-phase drive action for one commutation state (decode vocabulary, duty-free; the example's
/// `PhaseDrive`). [`sixstep_step`] maps it onto [`PhaseCmd`]: `Pwm -> Drive(duty)`,
/// `Sink -> Drive(0)`, `Float -> Float`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhaseDrive {
    /// Source: high-side chops at the step duty.
    Pwm,
    /// Sink: the current return path (compare 0, complementary low side on).
    Sink,
    /// Floating: the phase is electrically off.
    Float,
}

/// Rotation direction. Reversing swaps each state's source and sink phases (a half-turn, +3,
/// through the six-state sequence), keeping the same floating phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// The forward electrical sequence.
    Forward,
    /// The reverse sequence (source and sink swapped).
    Reverse,
}

/// The six commutation states in electrical order (the example's proven table). State `i` and
/// state `i + 3` are mirror images (source/sink swapped, same float). Phase order is `[A, B, C]`.
pub const STATES: [[PhaseDrive; 3]; 6] = {
    use PhaseDrive::{Float, Pwm, Sink};
    [
        [Pwm, Sink, Float],
        [Pwm, Float, Sink],
        [Float, Pwm, Sink],
        [Sink, Pwm, Float],
        [Sink, Float, Pwm],
        [Float, Sink, Pwm],
    ]
};

/// Hall 3-bit code (0..7) -> commutation sector (0..5), or [`INVALID`] for the two fault codes
/// (the example's canonical 120-degree ordering, `code = h_a | h_b << 1 | h_c << 2`). Ascending
/// sector follows the shared front-end's forward code order 1 -> 3 -> 2 -> 6 -> 4 -> 5, so an
/// advancing forward rotor advances the drive state by one (+60 deg) per hall step.
pub const HALL_TO_SECTOR: [u8; 8] = [INVALID, 0, 2, 1, 4, 5, 3, INVALID];

/// Sentinel for an invalid hall code in [`HALL_TO_SECTOR`].
const INVALID: u8 = 0xFF;

/// The 6-step hall decode: direction + the motor-specific alignment offset (the example's
/// `SixStep`, verbatim). The offset is found empirically on the bench (sweep 0..5) and baked per
/// the validated-tuning rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SixStep {
    direction: Direction,
    offset: u8,
}

impl SixStep {
    /// A decoder for `direction` with the motor alignment `offset` (taken mod 6).
    #[inline]
    pub const fn new(direction: Direction, offset: u8) -> Self {
        Self {
            direction,
            offset: offset % 6,
        }
    }

    /// The configured direction.
    #[inline]
    pub const fn direction(&self) -> Direction {
        self.direction
    }

    /// The configured alignment offset (0..5).
    #[inline]
    pub const fn offset(&self) -> u8 {
        self.offset
    }

    /// Decode a 3-bit hall `code` into the per-phase commutation pattern, or `None` if the code
    /// is a sensor fault (0 or 7). Only the low three bits of `code` are used.
    #[inline]
    pub fn pattern(&self, code: u8) -> Option<[PhaseDrive; 3]> {
        let sector = HALL_TO_SECTOR[(code & 0x7) as usize];
        if sector == INVALID {
            return None;
        }
        let half = match self.direction {
            Direction::Forward => 0,
            Direction::Reverse => 3,
        };
        let index = ((sector + self.offset + half) % 6) as usize;
        Some(STATES[index])
    }

    /// True if `code` is one of the six valid hall codes (not a sensor-fault 0 / 7).
    #[inline]
    pub fn is_valid_code(code: u8) -> bool {
        HALL_TO_SECTOR[(code & 0x7) as usize] != INVALID
    }
}

/// The per-motor six-step records: just the decode config (direction + align offset). No current
/// loop and no rotor state of its own; all rotor information comes from the shared front-end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SixStepState {
    /// The hall decode (direction + bench-swept align offset).
    pub decode: SixStep,
}

impl SixStepState {
    /// Fresh records for a decode config.
    pub const fn new(decode: SixStep) -> Self {
        Self { decode }
    }
}

/// Scale the (signed) drive demand to the source phase's duty: `|demand| * ARR / 32767`,
/// saturated at [`ARR`] (the demand is a voltage/throttle amplitude on the +-32767 frame scale;
/// the sign is the direction and is handled by the caller's direction flip).
#[inline]
pub fn demand_to_duty(demand: i32) -> u16 {
    let mag = demand.unsigned_abs();
    let scaled = (mag as u64 * ARR as u64) / 32767u64;
    if scaled > ARR as u64 {
        ARR
    } else {
        scaled as u16
    }
}

/// The all-float coast output (no phase driven; the bridge, if armed, floats).
pub const COAST: MotorOutput = MotorOutput {
    phases: [PhaseCmd::Float; 3],
};

/// One six-step control step. `code` is the debounced hall code (0..7) from the shared front-end;
/// `demand` is the signed voltage/throttle demand. Zero demand or an invalid code coasts
/// (all-`Float`); otherwise the decoded source phase drives at the scaled duty, the sink at 0,
/// and the third floats. A negative demand flips the effective direction.
#[inline]
pub fn sixstep_step(state: &SixStepState, code: u8, demand: i32) -> MotorOutput {
    if demand == 0 {
        return COAST;
    }
    // The demand's sign flips the configured direction (Reverse = +3 = source/sink swap).
    let decode = if demand < 0 {
        let flipped = match state.decode.direction() {
            Direction::Forward => Direction::Reverse,
            Direction::Reverse => Direction::Forward,
        };
        SixStep::new(flipped, state.decode.offset())
    } else {
        state.decode
    };

    match decode.pattern(code) {
        None => COAST, // sensor fault: coast (the front-end's dwell fault latches separately)
        Some(pattern) => {
            let duty = demand_to_duty(demand);
            let mut phases = [PhaseCmd::Float; 3];
            for (i, drive) in pattern.iter().enumerate() {
                phases[i] = match drive {
                    PhaseDrive::Pwm => PhaseCmd::Drive(duty),
                    PhaseDrive::Sink => PhaseCmd::Drive(0),
                    PhaseDrive::Float => PhaseCmd::Float,
                };
            }
            MotorOutput { phases }
        }
    }
}
