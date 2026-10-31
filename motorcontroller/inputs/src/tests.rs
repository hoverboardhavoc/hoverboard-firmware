//! Host tests for the inputs subsystem: the debounce/combo/pad logic exercised with square-wave and
//! glitch patterns to pin the load-bearing asymmetries.

use crate::combo::{combined_button, ComboPair, ComboSet, ComboState};
use crate::debounce::{DebouncePhase, LineBank};
use crate::pad::{PadBank, PAD_A_BIT, PAD_B_BIT};

// --- Debounce: two-call press, one-call release ---------------------------------------------

#[test]
fn press_needs_two_consecutive_asserts() {
    let mut bank = LineBank::<1>::new();
    // First assert: phase 0 -> 1 (candidate), NOT yet pressed.
    bank.update(0b1);
    assert!(!bank.pressed(0), "one assert must not press");
    assert_eq!(bank.line(0).unwrap().phase(), DebouncePhase::Candidate);
    // Second consecutive assert: phase 1 -> 2 (held), pressed rises.
    bank.update(0b1);
    assert!(bank.pressed(0), "two consecutive asserts press");
    assert_eq!(bank.line(0).unwrap().phase(), DebouncePhase::Held);
}

#[test]
fn release_is_one_sample() {
    let mut bank = LineBank::<1>::new();
    bank.update(0b1);
    bank.update(0b1);
    assert!(bank.pressed(0));
    // Single de-assert drops the flag immediately (no release-confirm interval).
    bank.update(0b0);
    assert!(!bank.pressed(0), "release within one call");
    assert_eq!(bank.line(0).unwrap().phase(), DebouncePhase::Idle);
}

#[test]
fn single_call_glitch_never_presses() {
    let mut bank = LineBank::<1>::new();
    // A lone assert followed by de-assert is a spike rejected in phase 1.
    bank.update(0b1);
    assert!(!bank.pressed(0));
    bank.update(0b0);
    assert!(!bank.pressed(0), "single-call glitch never sets the flag");
    assert_eq!(bank.line(0).unwrap().phase(), DebouncePhase::Idle);
}

#[test]
fn square_wave_flag_rises_after_two_and_drops_after_one() {
    let mut bank = LineBank::<1>::new();
    // Square wave: assert for 3 calls, de-assert for 3, repeated. The flag should rise on the 2nd
    // consecutive assert and drop on the 1st de-assert.
    let mut history = std::vec::Vec::new();
    for cycle in 0..3 {
        for k in 0..3 {
            bank.update(0b1);
            history.push((cycle, "hi", k, bank.pressed(0)));
        }
        for k in 0..3 {
            bank.update(0b0);
            history.push((cycle, "lo", k, bank.pressed(0)));
        }
    }
    // Within every high run: not pressed at k=0, pressed from k=1 onward.
    for &(_, phase, k, pressed) in &history {
        match (phase, k) {
            ("hi", 0) => assert!(!pressed, "k0 high: not yet pressed"),
            ("hi", _) => assert!(pressed, "k>=1 high: pressed"),
            ("lo", _) => assert!(!pressed, "any low: released"),
            _ => {}
        }
    }
}

#[test]
fn packed_flags_byte_set_means_idle() {
    // 3 lines: press line 1 only. Bit set = idle/released, bit clear = pressed.
    let mut bank = LineBank::<3>::new();
    bank.update(0b010);
    bank.update(0b010);
    assert!(bank.pressed(1));
    let b = bank.flags_byte();
    // line1 pressed -> bit1 clear; lines 0,2 idle -> bits 0,2 set; lines 3..7 absent -> set.
    assert_eq!(b & 1, 1, "line0 idle -> bit set");
    assert_eq!(b & 0b010, 0, "line1 pressed -> bit clear");
    assert_eq!(b & 0b100, 0b100, "line2 idle -> bit set");
    assert_eq!(b, 0b1111_1101);
}

// --- Combined button + brake/secondary -------------------------------------------------------

#[test]
fn combined_button_a_b_assignment() {
    let mut st = ComboState::new();
    // Held, brake inactive -> A set, combined true.
    assert!(combined_button(&mut st, true, false));
    assert!(st.a && !st.b);
    // Held, brake active -> B set too.
    assert!(combined_button(&mut st, true, true));
    assert!(st.a && st.b);
    // Not held, brake inactive -> clears A only.
    assert!(combined_button(&mut st, false, false));
    assert!(!st.a && st.b, "release with brake inactive clears A only");
    // Not held, brake active -> clears B.
    assert!(!combined_button(&mut st, false, true));
    assert!(!st.a && !st.b);
}

// --- Two-key combos --------------------------------------------------------------------------

#[test]
fn combo_only_when_both_members_held() {
    let mut bank = LineBank::<4>::new();
    // Power combo = lines (0,1), mode combo = lines (2,3).
    let combos = ComboSet::new(ComboPair::new(0, 1), ComboPair::new(2, 3));

    // Hold only line 0 (two calls). Power combo must be false (line 1 not held).
    bank.update(0b0001);
    bank.update(0b0001);
    assert!(bank.pressed(0) && !bank.pressed(1));
    assert!(!combos.evaluate(&bank).power, "one member is not a combo");

    // Now also hold line 1 (two calls), keeping line 0 held.
    bank.update(0b0011);
    bank.update(0b0011);
    assert!(bank.pressed(0) && bank.pressed(1));
    let f = combos.evaluate(&bank);
    assert!(f.power, "both power members held -> power combo");
    assert!(!f.mode, "mode members not held");

    // Release line 1 (one call): combo drops with its member.
    bank.update(0b0001);
    assert!(
        !combos.evaluate(&bank).power,
        "combo drops when a member releases"
    );
}

// --- Foot-pad debounce -----------------------------------------------------------------------

#[test]
fn pad_engages_on_one_high_off_on_two_low() {
    let mut pads = PadBank::new();
    // Single high on pad A engages immediately.
    let f = pads.update(true, false);
    assert_eq!(f & PAD_A_BIT, PAD_A_BIT, "1 high engages pad A");
    assert_eq!(f & PAD_B_BIT, 0);
    // One low sample does NOT disengage yet.
    let f = pads.update(false, false);
    assert_eq!(f & PAD_A_BIT, PAD_A_BIT, "1 low keeps pad A on");
    // Second consecutive low disengages.
    let f = pads.update(false, false);
    assert_eq!(f & PAD_A_BIT, 0, "2 lows disengage pad A");
}

#[test]
fn pad_high_resets_low_run() {
    let mut pads = PadBank::new();
    pads.update(true, true); // both on
    pads.update(false, false); // one low each (low_run = 1)
                               // A high sample re-engages and must reset the low run, so a single later low cannot disengage.
    pads.update(true, true);
    pads.update(false, false); // low_run back to 1
    let f = pads.field();
    assert_eq!(
        f,
        PAD_A_BIT | PAD_B_BIT,
        "high resets the low run; 1 low does not drop"
    );
}

#[test]
fn pad_counters_read_the_debounce_asymmetry() {
    // The bench gate's two numbers (`specs/silicon-queue.md`, "The foot pad on silicon"): a
    // complete press and release of ONE pad reads 2 changes and 1 held, because the engage follows
    // the pin on the same pass and the release holds through the first low. A symmetric debouncer
    // would read 2 and 2, which is what makes the pair a proof of the asymmetry and not just of
    // movement.
    let mut pads = PadBank::new();
    assert_eq!(
        (pads.changes(), pads.held()),
        (0, 0),
        "a fresh bank is quiet"
    );

    pads.update(true, false); // engage: field changes, state follows the pin
    assert_eq!(
        (pads.changes(), pads.held()),
        (1, 0),
        "an engage holds nothing"
    );
    pads.update(true, false); // steady on
    assert_eq!((pads.changes(), pads.held()), (1, 0));
    pads.update(false, false); // first low: state holds against the pin
    assert_eq!((pads.changes(), pads.held()), (1, 1));
    pads.update(false, false); // second low: state follows, field changes
    assert_eq!((pads.changes(), pads.held()), (2, 1));
    pads.update(false, false); // steady off
    assert_eq!((pads.changes(), pads.held()), (2, 1));
}

#[test]
fn an_absent_pad_never_steps_the_counters() {
    // An unconfigured pad is sampled as its idle level forever (the caller masks it), so the
    // padless half of the gate reads a hard zero rather than a small number: it is what tells a
    // staged-absent pin from a floating one, which is the whole point of staging it absent
    // (`specs/board-model.md`, the `pad.a` row, which defaults both pads ABSENT).
    let mut pads = PadBank::new();
    for _ in 0..200 {
        pads.update(false, false);
    }
    assert_eq!((pads.changes(), pads.held()), (0, 0));
    assert_eq!(pads.field(), 0);
}

#[test]
fn pad_field_both_pads_full_rider() {
    let mut pads = PadBank::new();
    let f = pads.update(true, true);
    assert_eq!(
        f,
        PAD_A_BIT | PAD_B_BIT,
        "both pads -> 0b11 full rider field"
    );
    assert_eq!(f, 0b11);
}

#[test]
fn polarity_resolves_each_line_against_its_own_bit() {
    // The registered default (`store::BOARD_INPUT_POLARITY` = 0b110): button active low, both
    // pads active high, which is the mapping the firmware held as three literals.
    const FLEET: u8 = 0b110;
    // Idle: the button rests high through its pull-up, the pads rest low with no foot.
    assert_eq!(crate::asserted(0b001, FLEET), 0);
    // A press pulls the button down; a foot pulls a pad up.
    assert_eq!(crate::asserted(0b000, FLEET), crate::BUTTON_LINE);
    assert_eq!(crate::asserted(0b011, FLEET), crate::PAD_A_LINE);
    assert_eq!(crate::asserted(0b101, FLEET), crate::PAD_B_LINE);
    assert_eq!(
        crate::asserted(0b110, FLEET),
        crate::INPUT_LINES,
        "pressed with both feet on"
    );
    // A board wired the other way round on every line reads the same events from the inverse
    // levels, which is the whole point of the field.
    const INVERSE: u8 = 0b001;
    assert_eq!(crate::asserted(0b110, INVERSE), 0);
    assert_eq!(crate::asserted(0b001, INVERSE), crate::INPUT_LINES);
}

#[test]
fn polarity_reserved_bits_are_inert() {
    // Bits 3..7 are reserved zero: a word carrying one resolves the three real lines exactly as
    // the same word without it, and never produces a fourth.
    for levels in 0u8..=255 {
        for polarity in 0u8..=0b111 {
            assert_eq!(
                crate::asserted(levels, polarity),
                crate::asserted(levels, polarity | 0b1111_1000),
            );
            assert_eq!(crate::asserted(levels, polarity) & !crate::INPUT_LINES, 0);
        }
    }
}
