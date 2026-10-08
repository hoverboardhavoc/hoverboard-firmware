//! The battery word (`specs/sensing-and-safety.md`, "The battery word"): the calibration seam, the
//! counts-to-centivolts conversion, the 250 Hz one-pole filter of the local sense, and the source
//! rule that picks the board's ONE battery word per tick.
//!
//! The word is centivolts, `i16`, and **0 = UNKNOWN**. Its consumers are the balance PID's `scale`
//! divisor (0 makes the raw output 0), the balance-mode engage gate, and the cyclic/telemetry
//! payload, which carries the EFFECTIVE word so a non-sensing board relays what it was told.
//!
//! The acquisition is the firmware's: the period ISR stores the injected group's third rank,
//! already reduced to the 12-bit right-aligned count, into one atomic word, and the 250 Hz task
//! hands that count to [`OrchestratorState::vbatt_raw`](crate::OrchestratorState::vbatt_raw)
//! before each pass (the `motor_fault` pattern). Everything from the count onward is here, so it
//! is host-tested end to end.

use linkctl::CyclicState;

/// The calibration's slope seam, microvolts of rail per 12-bit count (`board.vbatt_cal` index 0).
pub const SLOPE_MIN: i16 = 10_000;
/// See [`SLOPE_MIN`].
pub const SLOPE_MAX: i16 = 32_000;
/// The calibration's offset seam, centivolts (`board.vbatt_cal` index 1).
pub const OFFSET_MIN: i16 = -500;
/// See [`OFFSET_MIN`].
pub const OFFSET_MAX: i16 = 500;

/// The validated battery-sense calibration (`store::BOARD_VBATT_CAL`), clamped at its one seam.
/// Read at boot; a `CONFIG_WRITE` applies at the next boot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VbattCal {
    slope: i16,
    offset: i16,
}

impl VbattCal {
    /// Build from the two stored words, each clamped into its seam range (slope 10000..32000
    /// microvolts per count, offset -500..500 centivolts), so a hand-poked out-of-range flash value
    /// cannot reach the conversion.
    pub fn new(slope: i16, offset: i16) -> Self {
        VbattCal {
            slope: slope.clamp(SLOPE_MIN, SLOPE_MAX),
            offset: offset.clamp(OFFSET_MIN, OFFSET_MAX),
        }
    }

    /// The clamped slope, microvolts per count.
    pub fn slope(&self) -> i16 {
        self.slope
    }

    /// The clamped offset, centivolts.
    pub fn offset(&self) -> i16 {
        self.offset
    }

    /// Counts to centivolts: `clamp(raw12 * slope / 10_000 + offset, 1, i16::MAX)`, i32 math,
    /// truncating toward zero. `raw12` is the 12-bit right-aligned count (the calibration's unit;
    /// the producer reduces the left-aligned datum once, at the read). The lower clamp of 1 keeps
    /// a valid reading from reading as UNKNOWN.
    pub fn convert(&self, raw12: u16) -> i16 {
        let cv = (raw12 as i32 * self.slope as i32) / 10_000 + self.offset as i32;
        cv.clamp(1, i16::MAX as i32) as i16
    }
}

/// The local sense of a board whose plan carries `board.vbatt`: its calibration and the 250 Hz
/// one-pole filter `filt += (cv - filt) >> 4` (16 ticks = 64 ms), primed to the first sample.
#[derive(Clone, Copy, Debug)]
pub struct LocalSense {
    cal: VbattCal,
    /// The filtered word; 0 until the first conversion primes it (a converted value is >= 1).
    filt: i16,
}

impl LocalSense {
    /// A sense that has seen no conversion yet (its word is UNKNOWN).
    pub fn new(cal: VbattCal) -> Self {
        LocalSense { cal, filt: 0 }
    }

    /// One 250 Hz step over the latest 12-bit count; returns the filtered word.
    ///
    /// A count of 0 is "no conversion yet": the ISR's word starts at 0 and only a converting
    /// injected group ever writes it, so a sensing board whose motor was never brought up keeps
    /// it at 0 and its word UNKNOWN, rather than converting the 0 into the offset-clamped 1. A
    /// zero count after priming leaves the word where it was.
    pub fn step(&mut self, raw12: u16) -> i16 {
        if raw12 != 0 {
            let cv = self.cal.convert(raw12) as i32;
            self.filt = if self.filt == 0 {
                cv as i16
            } else {
                let f = self.filt as i32;
                (f + ((cv - f) >> 4)) as i16
            };
        }
        self.filt
    }

    /// The current filtered word (0 = no conversion yet).
    pub fn word(&self) -> i16 {
        self.filt
    }
}

/// The source rule: exactly one battery source per tick, in order.
///
/// 1. `local` is `Some` when the plan carries `board.vbatt`: its filtered word, whatever it is
///    (0 while no conversion has arrived). A sensing board never takes its peer's word, so a
///    master ignores its slave's, whose PA4 is not a battery sense.
/// 2. Otherwise the peer's cyclic word, if a peer mirror is present and fresh and the word is
///    nonzero.
/// 3. Otherwise 0, UNKNOWN.
///
/// `peer_fresh` is the existing mirror staleness (`specs/link-control.md`, "Supervision"): the
/// caller passes `!comms_loss`, which with `peer` present means a cyclic arrived inside the
/// timeout.
pub fn battery_source(local: Option<i16>, peer: Option<CyclicState>, peer_fresh: bool) -> i16 {
    match (local, peer) {
        (Some(w), _) => w,
        (None, Some(p)) if peer_fresh && p.battery != 0 => p.battery.min(i16::MAX as u16) as i16,
        _ => 0,
    }
}
