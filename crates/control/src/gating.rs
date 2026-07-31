//! The engagement machine's gating/pickup input: the **conditioned up-axis accelerometer
//! channel** (`specs/control.md` (c), "The gating/pickup cell's producer").
//!
//! The cell the engagement machine gates on ([`FsmInputs::gating_field`](crate::FsmInputs)) had
//! been carried as "producer unknown" since the machine was rebuilt to the binary. It is
//! recovered: stock `0x20000204` is the IMU record's conditioned-accel triple + 4
//! (`0x2000019c + 0x68`), read `ldrsh` at both FSM sites (`FUN_08003be8` @ `0x08003cbe` engage,
//! `0x08003ebe` pickup). It is not a demand, a torque or a speed setpoint: it is the
//! **low-pass-filtered accelerometer Z axis in raw counts**, written by the 4 ms IMU task
//! `FUN_08000ccc` as
//!
//! ```text
//!   state = 0.02 * sign_mapped(az) + 0.98 * state      (doubles in the original)
//!   cell  = d2iz(state)                                (truncate toward zero, stored s16)
//! ```
//!
//! with no bias subtraction, no deadband, no clamp and no `abs()` on this path (the clamp in the
//! same function applies to the RAW mirror at struct+0x70, not to the conditioned triple). The
//! same 0.02/0.98 working-channel IIR is the one `Declassyfied/spec/attitude.md` records for the
//! accel/gyro channel.
//!
//! # What the thresholds mean, once the scale is known
//!
//! Stock configures `ACCEL_CONFIG = 0x08` = +-4 g, so a signed 16-bit count is **8192 LSB per g**.
//! Therefore:
//!
//! - engage, `> 500` ([`GATING_THRESHOLD`](crate::config::fsm::GATING_THRESHOLD)): more than
//!   **0.061 g** of gravity on the deck's up-axis, i.e. the deck is within ~86.5 deg of
//!   horizontal and the right way up. It is an ORIENTATION gate, not a demand gate.
//! - pickup, `< 0` for more than 20 ticks (~80 ms): the up-axis has gone negative, i.e. the deck
//!   is inverted.
//! - the dead band between the two edges is 0..500 counts; there is no second threshold constant.
//!   The IIR itself (tau ~0.2 s at the 4 ms tick) is the anti-chatter filter.
//!
//! The axis reading is corroborated inside the same stock function: its still/level classifier
//! compares this triple against 6144 (0.75 g on the up axis) and 2048/3072 (0.25/0.375 g on the
//! other two), and the FSM's pre-gate byte is that classifier's "lying on its side" flag.

use crate::helpers::q_to_int_d2iz;
use base::fixed::Fix;

/// The conditioning carry behind the gating/pickup cell (a balance producer record, held by the
/// layer that runs the IMU tick; the same shape as [`IirCarry`](crate::IirCarry)).
///
/// It starts at zero, which is the stock cell's own cold-boot value: the count climbs into the
/// engage band over the IIR's first few ticks (from a level 1 g sample, 4 ticks / 16 ms), so a
/// board cannot engage on the very first sample after reset.
#[derive(Clone, Copy, Debug, Default)]
pub struct GatingFilter {
    /// The one-pole state, in raw accel counts (Q; a flagged fractional per spec (f)).
    pub state: Fix,
}

impl GatingFilter {
    /// One conditioning step over the **sign-applied** up-axis accel count, returning the cell's
    /// new value.
    ///
    /// `up_axis` is the raw count with the board's accel sign map already applied (the sign map
    /// is the attitude config's, the canonical owner of that wiring; stock's own `-raw_az` is
    /// that same map for its mount). The caller writes the result into the block's gating row.
    pub fn tick(&mut self, up_axis: Fix) -> i16 {
        // FLAGGED: 0.02 / 0.98 doubles in the original -> Q (spec (f)).
        self.state = Fix::from_num(0.02) * up_axis + Fix::from_num(0.98) * self.state;
        // d2iz then the halfword store: truncate toward zero, narrow to s16 (the binary's
        // `strh`). A conditioned accel count cannot leave the s16 range the sample came from.
        q_to_int_d2iz(self.state) as i16
    }
}
