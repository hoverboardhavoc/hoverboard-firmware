//! Combined button + brake/secondary flag and two-key (power / mode) combo detection (16 ms task).
//!
//! Both are computed each 16 ms call AFTER the per-line debounce, from the debounced stable-pressed
//! states. They add no extra debounce: they inherit the two-call press confirmation and one-call
//! release of their member lines, and are LEVEL signals that coexist with the individual flags.
//!
//! Combo-pair membership (which two lines form the power combo, which form the mode combo) is
//! board-definition config the caller supplies; this module takes the line indices.

use crate::debounce::LineBank;

/// The combined button + brake/secondary sub-flag state.
///
/// Two sub-flags `A` and `B` record the CONTEXT a button press happened in: `A` for "button held
/// while brake/secondary inactive", `B` for "button held while brake/secondary active". The combined
/// flag is `A OR B`, asserted whenever the debounced button is held regardless of brake state.
#[derive(Clone, Copy, Debug, Default)]
pub struct ComboState {
    /// Sub-flag A: press registered with brake/secondary INACTIVE.
    pub a: bool,
    /// Sub-flag B: press registered with brake/secondary ACTIVE.
    pub b: bool,
}

impl ComboState {
    /// A fresh state, both sub-flags clear.
    pub const fn new() -> Self {
        Self { a: false, b: false }
    }

    /// The combined button flag = `A OR B`.
    #[inline]
    pub fn combined(&self) -> bool {
        self.a || self.b
    }
}

/// Update the combined button + brake/secondary sub-flags from the debounced button and the
/// brake/secondary condition. Call each 16 ms after debounce.
///
/// - button held + brake inactive -> set A.
/// - button held + brake active   -> set B.
/// - button not held + brake inactive -> clear A.
/// - button not held + brake active   -> clear B.
///
/// Note the slot cleared when the button is not held depends on the CURRENT brake state, exactly as
/// specified. Returns the combined flag (`A OR B`).
pub fn combined_button(state: &mut ComboState, button_pressed: bool, brake_active: bool) -> bool {
    if button_pressed {
        if !brake_active {
            state.a = true;
        } else {
            state.b = true;
        }
    } else if !brake_active {
        state.a = false;
    } else {
        state.b = false;
    }
    state.combined()
}

/// A two-key combo: the two member line indices into a [`LineBank`]. Membership is board data.
#[derive(Clone, Copy, Debug)]
pub struct ComboPair {
    /// First member line index.
    pub a: usize,
    /// Second member line index.
    pub b: usize,
}

impl ComboPair {
    /// A combo over lines `a` and `b`.
    pub const fn new(a: usize, b: usize) -> Self {
        Self { a, b }
    }

    /// True exactly while BOTH member lines report stable-pressed simultaneously. Level signal;
    /// inherits the members' two-call press / one-call release with no extra debounce.
    #[inline]
    pub fn active(&self, bank: &LineBank) -> bool {
        bank.pressed(self.a) && bank.pressed(self.b)
    }
}

/// The set of defined combos (at least two: a power combo and a mode combo). The memberships are
/// board-definition config the caller fills in.
#[derive(Clone, Copy, Debug)]
pub struct ComboSet {
    /// The power combo pair.
    pub power: ComboPair,
    /// The mode combo pair.
    pub mode: ComboPair,
}

impl ComboSet {
    /// A combo set with the given power and mode pairs.
    pub const fn new(power: ComboPair, mode: ComboPair) -> Self {
        Self { power, mode }
    }

    /// Evaluate both combos against the current debounced bank. Call each 16 ms after debounce.
    pub fn evaluate(&self, bank: &LineBank) -> ComboFlags {
        ComboFlags {
            power: self.power.active(bank),
            mode: self.mode.active(bank),
        }
    }
}

/// The evaluated combo flags for one 16 ms call (level signals).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ComboFlags {
    /// Power combo active (both power-combo members held).
    pub power: bool,
    /// Mode combo active (both mode-combo members held).
    pub mode: bool,
}
