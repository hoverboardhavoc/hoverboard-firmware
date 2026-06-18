//! Foot-pad sensors (rider-on-pad), a 16 ms task.
//!
//! The two foot-pad sensors are plain digital GPIO inputs, polarity **active-high** (pin high = foot
//! on), the OPPOSITE of the active-low buttons. Each pad runs a small debouncer with the inverse
//! asymmetry of the buttons: "foot off" is recognized only after the pin reads low on **2
//! consecutive** samples (~32 ms make for off), and a single high sample returns to "foot on"
//! immediately. Fast to engage, debounced to release.
//!
//! The two debounced states merge into a **2-bit local pad field** (bit 0 = pad A, bit 1 = pad B;
//! set bit = foot on).
//!
//! This field DRIVES THE GAIN SCHEDULE; it is NOT an on/off enable. With no rider the balance loop
//! is still active at low-authority standby gains `{50, 20, 0}`; as the rider mounts and both pads
//! assert, the working balance coefficients step to the RUN set `{6000, 2000, 40}` (~120x more
//! proportional authority). The gain step itself lives in `crates/control` (the standby seed set and
//! Profile-A); this crate produces only the rider/pad FIELD that selects the step. Reference pins
//! `PA11` (pad A) / `PC15` (pad B) are board-definition references the caller supplies as already-
//! sampled high/low levels.

/// Bit 0 of the 2-bit pad field: pad A (foot on when set).
pub const PAD_A_BIT: u8 = 0b01;
/// Bit 1 of the 2-bit pad field: pad B (foot on when set).
pub const PAD_B_BIT: u8 = 0b10;

/// The merged 2-bit local pad field. `0` = no rider, `0b11` = both pads asserted (full rider).
pub type PadField = u8;

/// One foot-pad debouncer: active-high, 2-low-samples-off / 1-high-on.
#[derive(Clone, Copy, Debug)]
struct PadSensor {
    /// True = "foot on" (engaged). Starts off.
    on: bool,
    /// Count of consecutive low samples while engaged (off recognized at 2).
    low_run: u8,
}

impl PadSensor {
    const fn new() -> Self {
        Self {
            on: false,
            low_run: 0,
        }
    }

    /// Advance one 16 ms call. `high` is the already-sampled active-high level (pin high = foot on).
    /// A single high sample re-engages immediately and resets the low run; "foot off" needs 2
    /// consecutive low samples. Returns the debounced "foot on" state.
    fn update(&mut self, high: bool) -> bool {
        if high {
            // Single high sample engages immediately (fast to engage).
            self.on = true;
            self.low_run = 0;
        } else {
            // Low sample: only after 2 consecutive lows is "foot off" recognized.
            if self.low_run < 2 {
                self.low_run += 1;
            }
            if self.low_run >= 2 {
                self.on = false;
            }
        }
        self.on
    }
}

/// The pair of foot-pad debouncers feeding the 2-bit pad field.
#[derive(Clone, Copy, Debug)]
pub struct PadBank {
    pad_a: PadSensor,
    pad_b: PadSensor,
}

impl PadBank {
    /// A fresh pad bank, both pads off.
    pub const fn new() -> Self {
        Self {
            pad_a: PadSensor::new(),
            pad_b: PadSensor::new(),
        }
    }

    /// Advance both pads one 16 ms call from their already-sampled active-high levels. Returns the
    /// merged 2-bit pad field.
    pub fn update(&mut self, pad_a_high: bool, pad_b_high: bool) -> PadField {
        let a = self.pad_a.update(pad_a_high);
        let b = self.pad_b.update(pad_b_high);
        self.field_from(a, b)
    }

    /// The current merged 2-bit pad field without advancing.
    pub fn field(&self) -> PadField {
        self.field_from(self.pad_a.on, self.pad_b.on)
    }

    fn field_from(&self, a: bool, b: bool) -> PadField {
        let mut f: PadField = 0;
        if a {
            f |= PAD_A_BIT;
        }
        if b {
            f |= PAD_B_BIT;
        }
        f
    }
}

impl Default for PadBank {
    fn default() -> Self {
        Self::new()
    }
}
