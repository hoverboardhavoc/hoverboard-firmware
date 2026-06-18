//! Throttle filter (4 ms task): the scaled-throttle intermediate and the single-pole IIR low-pass.
//!
//! One raw 16-bit unsigned ADC sample per call (the ADC/DMA path places it in RAM). Two outputs:
//! the un-smoothed SCALED throttle, and the IIR-filtered throttle with the `+200` rest bias.
//!
//! ## Scaled-throttle (unfiltered)
//!
//! ```text
//! scaled = (raw * 0x27F6) >> 15      // unsigned multiply on 32-bit, then >>15
//! ```
//!
//! `0x27F6 / 0x8000 ~= 0.3122`. Published directly as the un-smoothed throttle (signed 16-bit).
//!
//! ## IIR-filtered output
//!
//! The IIR input is the SCALED throttle `s`, NOT the raw sample (the reference path reuses the
//! scaled value for both the baseline capture and the recursive mix; feeding raw would apply an
//! extra ~3.2x gain). The original keeps the baseline as float32 widened to double each step. This
//! project bans software float from the hot path (no FPU), so the carry is reproduced in Q-format
//! ([`I32F32`], 32 fractional bits) with the coefficients reproduced exactly. The very slow tau
//! (~13.3 s at 4 ms) makes the filter sensitive to coefficient precision, so the carry resolution
//! (~2.3e-10) and exact `Ka`/`Kb` are load-bearing; tests validate against an f64 reference.
//!
//! - First call after init (one-shot): capture the current scaled throttle `s` as the baseline and
//!   mark initialized; the state equals the baseline on this call (no deviation yet).
//! - Every subsequent call: `filtered = baseline*Kb + s*Ka`, then `baseline = filtered`.
//!   `Ka = 0.0003`, `Kb = 0.9997` (`Ka + Kb = 1.0`).
//! - Output: `(int16) filtered + 200`, truncating toward zero, clamped (not wrapped) on overflow.
//!
//! The `+200` is a fixed bias: resting throttle reports as +200, deflection moves it above/below.

use fixed::types::I32F32;

/// The throttle scale numerator: `scaled = (raw * SCALE_NUM) >> SCALE_SHIFT`. `0x27F6 / 0x8000`.
pub const SCALE_NUM: u32 = 0x27F6;
/// The throttle scale right shift (`>> 15`, i.e. divide by `0x8000`).
pub const SCALE_SHIFT: u32 = 15;
/// The fixed output bias: resting/centered throttle reports as this value.
pub const OUTPUT_BIAS: i32 = 200;

/// IIR new-sample coefficient `Ka` (exact reference value).
pub const KA: f64 = 0.0003;
/// IIR carry coefficient `Kb` (exact reference value; `Ka + Kb = 1.0`).
pub const KB: f64 = 0.9997;

/// The scaled (un-smoothed) throttle from one raw 16-bit ADC sample.
///
/// `scaled = (raw * 0x27F6) >> 15` as an unsigned 32-bit multiply then shift, published as signed
/// 16-bit. With `raw` bounded 0..=65535 the result is within signed-16 (max ~20459), so the `as i16`
/// never truncates a real value.
#[inline]
pub fn scaled_throttle(raw: u16) -> i16 {
    let scaled = ((raw as u32) * SCALE_NUM) >> SCALE_SHIFT;
    scaled as i16
}

/// The single-pole IIR throttle filter. Holds the Q-format carry (the recursive `baseline` state)
/// and the one-shot init flag.
#[derive(Clone, Copy, Debug)]
pub struct ThrottleFilter {
    /// The recursive baseline / filtered state, in Q-format (the original's float32 `double`-widened
    /// carry). 32 fractional bits hold the slow tau without drift.
    baseline: I32F32,
    /// Coefficient `Ka` in Q (new-sample weight), reproduced exactly from [`KA`].
    ka: I32F32,
    /// Coefficient `Kb` in Q (carry weight), reproduced exactly from [`KB`].
    kb: I32F32,
    /// One-shot: false until the first call captures the baseline.
    initialized: bool,
}

impl ThrottleFilter {
    /// A fresh filter, not yet initialized. The baseline is captured on the first [`step`] call.
    ///
    /// [`step`]: ThrottleFilter::step
    pub fn new() -> Self {
        Self {
            baseline: I32F32::ZERO,
            // Reproduce Ka, Kb in Q-format from the exact reference doubles.
            ka: I32F32::from_num(KA),
            kb: I32F32::from_num(KB),
            initialized: false,
        }
    }

    /// Has the one-shot baseline been captured yet?
    #[inline]
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// The current Q-format filtered value (the recursive baseline), as f64 for inspection/tests.
    #[inline]
    pub fn baseline_f64(&self) -> f64 {
        self.baseline.to_num::<f64>()
    }

    /// Process one raw 16-bit ADC sample (4 ms call) and return the biased signed-16 throttle.
    ///
    /// First call: capture `s = scaled_throttle(raw)` as the baseline (one-shot) and use it as the
    /// state directly. Subsequent calls: `filtered = baseline*Kb + s*Ka`; `baseline = filtered`.
    /// Output is `(int16) filtered + 200`, truncating toward zero, clamped (not wrapped) on overflow.
    pub fn step(&mut self, raw: u16) -> i16 {
        let s = scaled_throttle(raw);
        let s_q = I32F32::from_num(s);

        if !self.initialized {
            // One-shot: the first sample is both baseline and output (no deviation yet).
            self.baseline = s_q;
            self.initialized = true;
        } else {
            // filtered = baseline*Kb + s*Ka ; recursive carry.
            let filtered = self.baseline * self.kb + s_q * self.ka;
            self.baseline = filtered;
        }

        // Output: convert filtered to int (truncate toward zero), add the +200 bias, clamp to i16.
        let filtered_int: i32 = self.baseline.to_num::<i32>();
        let biased = filtered_int.saturating_add(OUTPUT_BIAS);
        biased.clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }
}

impl Default for ThrottleFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// An f64 reference model of the IIR, for host-side validation of the Q implementation. Mirrors the
/// original's float math exactly (scaled input, one-shot baseline, `baseline*Kb + s*Ka`).
#[cfg(test)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ThrottleRefF64 {
    pub baseline: f64,
    pub initialized: bool,
}

#[cfg(test)]
impl ThrottleRefF64 {
    pub fn step(&mut self, raw: u16) -> i16 {
        let s = scaled_throttle(raw) as f64;
        if !self.initialized {
            self.baseline = s;
            self.initialized = true;
        } else {
            self.baseline = self.baseline * KB + s * KA;
        }
        // Truncate toward zero, add bias, clamp.
        let filtered_int = self.baseline.trunc() as i64;
        let biased = filtered_int + OUTPUT_BIAS as i64;
        biased.clamp(i16::MIN as i64, i16::MAX as i64) as i16
    }
}
