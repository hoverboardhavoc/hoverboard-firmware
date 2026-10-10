//! Discrete-input debounce (16 ms task): the three-phase per-line state machine and the packed
//! flags byte.
//!
//! Each debounced line is **active-low** (asserted reads logic 0; lines idle high through pull-ups)
//! and owns a tiny record: a phase in {0, 1, 2} and a stable-pressed boolean. The asymmetry is
//! load-bearing: a press needs the line asserted on TWO consecutive 16 ms calls (0 -> 1 -> 2); a
//! release is ONE-sample (de-assert in phase 1 or 2 drops the flag immediately, no extra
//! release-confirmation interval). stable-pressed is a LEVEL signal, not an edge; consumers derive
//! edges.
//!
//! The machine is replicated per line. The count of lines is config (the caller picks it, up to
//! [`MAX_LINES`]); which pin / polarity each line maps to is board data resolved by the caller, who
//! supplies an already-sampled `asserted` bit per line.

/// Maximum number of debounced lines a single [`LineBank`] holds. The packed flags byte gives one
/// bit per line, so the bank caps at 8 lines. The actual count is config (`<= MAX_LINES`).
pub const MAX_LINES: usize = 8;

/// The debounce phase for one line (the recovered discrete-input debounce contract).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DebouncePhase {
    /// Phase 0: released / idle.
    #[default]
    Idle,
    /// Phase 1: candidate (asserted once, not yet pressed).
    Candidate,
    /// Phase 2: held / confirmed.
    Held,
}

/// One debounced line's record: a phase plus the level stable-pressed flag.
#[derive(Clone, Copy, Debug, Default)]
pub struct DebounceLine {
    phase: DebouncePhase,
    stable_pressed: bool,
}

impl DebounceLine {
    /// A fresh line: phase 0 (idle), not pressed.
    pub const fn new() -> Self {
        Self {
            phase: DebouncePhase::Idle,
            stable_pressed: false,
        }
    }

    /// Advance the line one 16 ms call. `asserted` is the already-sampled active-low result
    /// (`asserted == (line reads logic 0)`); the caller does the pin read and polarity mapping.
    ///
    /// Returns the updated stable-pressed level. The transitions are EXACTLY:
    /// - Phase 0 (Idle):      asserted -> Candidate; else clear stable-pressed (stay Idle).
    /// - Phase 1 (Candidate): asserted -> stable-pressed = true, Held; else -> Idle (spike rejected).
    /// - Phase 2 (Held):      asserted -> stay (stable-pressed stays true); else -> Idle, clear
    ///   stable-pressed (release confirmed, one-sample).
    pub fn update(&mut self, asserted: bool) -> bool {
        match self.phase {
            DebouncePhase::Idle => {
                if asserted {
                    // Candidate: not yet pressed, needs one more consecutive assert.
                    self.phase = DebouncePhase::Candidate;
                } else {
                    self.stable_pressed = false;
                }
            }
            DebouncePhase::Candidate => {
                if asserted {
                    // Second consecutive assert: the press is confirmed.
                    self.stable_pressed = true;
                    self.phase = DebouncePhase::Held;
                } else {
                    // Single-sample glitch: reject, return to idle. The flag never rose.
                    self.phase = DebouncePhase::Idle;
                }
            }
            DebouncePhase::Held => {
                if !asserted {
                    // One-sample release: drop the flag and return to idle.
                    self.phase = DebouncePhase::Idle;
                    self.stable_pressed = false;
                }
            }
        }
        self.stable_pressed
    }

    /// The current stable-pressed level (true for the whole hold, false otherwise).
    #[inline]
    pub fn pressed(&self) -> bool {
        self.stable_pressed
    }

    /// The current phase (mostly for tests / introspection).
    #[inline]
    pub fn phase(&self) -> DebouncePhase {
        self.phase
    }
}

/// A bank of `N` debounced lines plus the packed flags byte.
///
/// `N` IS the line count: the array and the count are one fact, so no index can exceed the lines
/// the bank holds and no count can disagree with the array. That is not a style choice. The bank
/// previously held `[DebounceLine; MAX_LINES]` beside a `count: usize` clamped in `new`, and
/// `update`'s `self.lines[i]` therefore carried a `panic_bounds_check` into the 16 ms input task:
/// the clamp lived in a constructor rather than in the type, so nothing downstream could see it.
/// `panic-halt` spins where a panic fires, and the lines this bank debounces are the power button
/// and the foot pads, whose level steps the balance gain schedule (`specs/panic-free.md`; see
/// [`crate::pad`] for what a stuck pad field means).
///
/// Compile-time, because every construction in this firmware is a literal (the orchestrator's
/// one-line power button; three and four in the host tests). A board model that resolved the count
/// at RUNTIME would want a bounded vector here instead, with the same property: the length and the
/// storage as one thing.
#[derive(Clone, Copy, Debug)]
pub struct LineBank<const N: usize> {
    lines: [DebounceLine; N],
}

impl<const N: usize> LineBank<N> {
    /// A bank of `N` lines, all idle. `N > MAX_LINES` does not build: the packed flags byte has one
    /// bit per line and there is no ninth bit to report.
    pub const fn new() -> Self {
        const { assert!(N <= MAX_LINES, "a LineBank holds at most MAX_LINES lines") };
        Self {
            lines: [DebounceLine::new(); N],
        }
    }

    /// The configured line count.
    #[inline]
    pub fn count(&self) -> usize {
        N
    }

    /// Advance every line one 16 ms call from a bitfield of sampled active-low results. Bit `i`
    /// set means line `i` is `asserted` (reads logic 0); the caller has already done the read and
    /// the active-low mapping. Bits at or above `N` are ignored.
    pub fn update(&mut self, asserted_bits: u8) {
        // Iterate the lines, never an index: the trip count is `N`, a constant `<= MAX_LINES`, so
        // the shift amount is in range and there is no bound left to check.
        for (i, line) in self.lines.iter_mut().enumerate() {
            line.update((asserted_bits >> i) & 1 != 0);
        }
    }

    /// Borrow line `i`, or `None` if this bank has no such line.
    #[inline]
    pub fn line(&self, i: usize) -> Option<&DebounceLine> {
        self.lines.get(i)
    }

    /// Line `i`'s stable-pressed level (false for out-of-range indices).
    #[inline]
    pub fn pressed(&self, i: usize) -> bool {
        self.lines.get(i).is_some_and(|l| l.pressed())
    }

    /// The packed flags byte: one bit per line, **bit set = line idle/released**, bit clear = line
    /// asserted/pressed (inverted sense, matching the published byte). Bits at or above `N` are
    /// reported as idle (set), since an absent line is never pressed.
    pub fn flags_byte(&self) -> u8 {
        // Every bit starts idle (set); a pressed line clears its own.
        let mut b = u8::MAX;
        for (i, line) in self.lines.iter().enumerate() {
            if line.pressed() {
                b &= !(1 << i);
            }
        }
        b
    }
}

impl<const N: usize> Default for LineBank<N> {
    fn default() -> Self {
        Self::new()
    }
}
