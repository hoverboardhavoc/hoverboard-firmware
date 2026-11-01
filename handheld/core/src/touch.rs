//! The resistive touch panel (`specs/handheld-remote.md` sections 4.2 to 4.4): the XPT2046's
//! command bytes and frame assembly, the classification of one sample as Valid or Up, and the lift
//! state machine whose state decides the demand.
//!
//! Positions are in display pixels in the layout's orientation (portrait, y down), carried in
//! sixteenths of a pixel ([`SUB`]) so the lift machine's filter needs no floating point.

/// Sixteenths of a pixel per pixel.
pub const SUB: i32 = 16;

// --- The XPT2046 frame (section 4.2) -------------------------------------------------------------

/// Z1 conversion, 12-bit differential, PENIRQ disabled while converting.
pub const CMD_Z1: u8 = 0xB1;
/// Z2 conversion.
pub const CMD_Z2: u8 = 0xC1;
/// Y position.
pub const CMD_Y: u8 = 0x91;
/// X position.
pub const CMD_X: u8 = 0xD1;
/// X position with the power-down bits clear, so PENIRQ is enabled again after it.
pub const CMD_X_LAST: u8 = 0xD0;
/// The conversions of one sample, in order: Z1, Z2, three Y, three X, the last re-enabling PENIRQ.
pub const COMMANDS: [u8; 8] = [
    CMD_Z1, CMD_Z2, CMD_Y, CMD_Y, CMD_Y, CMD_X, CMD_X, CMD_X_LAST,
];
/// Clocks (bytes) per conversion: the command and two bytes that clock out the result.
const PER: usize = 3;
/// Bytes of one sample's CS window.
pub const FRAME_LEN: usize = COMMANDS.len() * PER;

/// The bytes written in one sample's CS window: each command followed by two zero bytes.
pub const fn tx_frame() -> [u8; FRAME_LEN] {
    let mut f = [0u8; FRAME_LEN];
    let mut i = 0;
    while i < COMMANDS.len() {
        f[i * PER] = COMMANDS[i];
        i += 1;
    }
    f
}

/// The raw conversions of one sample.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Raw {
    pub z1: u16,
    pub z2: u16,
    pub y: [u16; 3],
    pub x: [u16; 3],
}

/// The 12-bit result clocked out after a command: the busy bit, then the result MSB first, then
/// three zero bits.
fn result(hi: u8, lo: u8) -> u16 {
    ((u16::from(hi) << 8 | u16::from(lo)) >> 3) & 0x0FFF
}

impl Raw {
    /// The conversions in the bytes read back during [`tx_frame`].
    pub fn parse(rx: &[u8; FRAME_LEN]) -> Raw {
        let r = |i: usize| result(rx[i * PER + 1], rx[i * PER + 2]);
        Raw {
            z1: r(0),
            z2: r(1),
            y: [r(2), r(3), r(4)],
            x: [r(5), r(6), r(7)],
        }
    }

    /// The pressure figure: larger is a firmer press.
    pub fn z(&self) -> i32 {
        i32::from(self.z1) + 4095 - i32::from(self.z2)
    }
}

/// The median and the spread (largest less smallest) of three readings.
pub fn median_spread(v: [u16; 3]) -> (u16, u16) {
    let mut s = v;
    s.sort_unstable();
    (s[1], s[2] - s[0])
}

// --- Classification (section 4.3) ----------------------------------------------------------------

/// The thresholds a sample is judged by. The start values are section 4.3's; the bench tuning
/// (section 7.2 check 3) replaces them in the source.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Limits {
    /// The `z` that starts a touch.
    pub z_on: i32,
    /// The `z` that keeps a touch going.
    pub z_off: i32,
    /// The widest spread of an axis's three readings, raw counts.
    pub spread_max: u16,
    /// The lowest raw median that is not at a rail.
    pub raw_min: u16,
    /// The highest raw median that is not at a rail.
    pub raw_max: u16,
    /// The largest move between consecutive samples, pixels.
    pub jump_max: i32,
}

/// Section 4.3's start values.
pub const LIMITS: Limits = Limits {
    z_on: 400,
    z_off: 300,
    spread_max: 48,
    raw_min: 100,
    raw_max: 3995,
    jump_max: 24,
};

/// Raw counts to display pixels, per axis: `px = (raw - raw0) * span_px / (raw1 - raw0)`, with
/// the panel's raw X and Y possibly swapped. Measured from the four corners on the bench (section
/// 7.2 check 3); the start values are the common CYD figures, portrait.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Calibration {
    /// Whether the panel's raw X runs along the display's y (portrait on this panel).
    pub swap: bool,
    /// Raw count at display x = 0 and at x = width (either order).
    pub x_raw: (i32, i32),
    /// Raw count at display y = 0 and at y = height.
    pub y_raw: (i32, i32),
    /// Display width, pixels.
    pub width: i32,
    /// Display height, pixels.
    pub height: i32,
}

/// The start calibration: portrait 240 x 320.
pub const CALIBRATION: Calibration = Calibration {
    swap: false,
    x_raw: (3800, 240),
    y_raw: (3700, 200),
    width: 240,
    height: 320,
};

impl Calibration {
    /// The display position of raw medians `(rx, ry)`, in sixteenths of a pixel.
    pub fn to_sub(&self, rx: u16, ry: u16) -> (i32, i32) {
        let (ax, ay) = if self.swap { (ry, rx) } else { (rx, ry) };
        let map = |raw: u16, (r0, r1): (i32, i32), span: i32| {
            let d = r1 - r0;
            if d == 0 {
                0
            } else {
                (i32::from(raw) - r0) * span * SUB / d
            }
        };
        (
            map(ax, self.x_raw, self.width),
            map(ay, self.y_raw, self.height),
        )
    }
}

/// One sample, as the sampler hands it over: what PENIRQ read before the conversions, whether the
/// SPI transfer succeeded, and the conversions.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Reading {
    pub pen_down: bool,
    pub transfer_ok: bool,
    pub raw: Raw,
}

/// A classified sample.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sample {
    /// Anything the sampler cannot vouch for.
    Up,
    /// A point, sixteenths of a pixel, and its `z`.
    Valid { x: i32, y: i32, z: i32 },
}

/// Classify `r` (section 4.3). `touching` is whether a touch is in progress (the `Z_OFF`
/// hysteresis); `near` is the point a new one must be within [`Limits::jump_max`] of, in
/// sixteenths of a pixel (the filtered point while a touch is in progress), or `None`.
pub fn classify(
    r: &Reading,
    touching: bool,
    near: Option<i32>,
    lim: &Limits,
    cal: &Calibration,
) -> Sample {
    if !r.pen_down || !r.transfer_ok {
        return Sample::Up;
    }
    let z = r.raw.z();
    if z < if touching { lim.z_off } else { lim.z_on } {
        return Sample::Up;
    }
    let (mx, sx) = median_spread(r.raw.x);
    let (my, sy) = median_spread(r.raw.y);
    let in_range = |m: u16| (lim.raw_min..=lim.raw_max).contains(&m);
    if sx > lim.spread_max || sy > lim.spread_max || !in_range(mx) || !in_range(my) {
        return Sample::Up;
    }
    let (x, y) = cal.to_sub(mx, my);
    if let Some(n) = near {
        if (y - n).abs() > lim.jump_max * SUB {
            return Sample::Up;
        }
    }
    Sample::Valid { x, y, z }
}

// --- The lift state machine (section 4.4) --------------------------------------------------------

/// Samples that start a touch (8 ms).
pub const N_DOWN: u8 = 4;
/// Consecutive Valid samples that resume a lifting touch.
pub const N_RESUME: u8 = 2;
/// How long a lift waits for the finger to come back before the release is confirmed, ms.
pub const LIFT_CONFIRM_MS: u32 = 40;
/// The sample period, ms.
pub const PERIOD_MS: u32 = 2;
/// With no sample for this long the touch is treated as Up (five periods: the sampler stalled).
pub const STALL_MS: u32 = 10;

/// The touch state.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Lift {
    /// Nothing on the panel.
    Idle,
    /// `n` Valid samples so far, their `y` summed, the last one's `y`.
    Pressing { n: u8, sum: i32, last: i32 },
    /// A touch: its touch-down point `y0` and the filtered point `yf`.
    Down { y0: i32, yf: i32 },
    /// The touch dropped out at `since`; `resume` Valid samples near `yf` so far.
    Lifting {
        y0: i32,
        yf: i32,
        since: u32,
        resume: u8,
    },
}

impl Lift {
    /// Whether this counts as a finger down (section 4.4): every state but [`Lift::Idle`]. The arm
    /// rule arms only once a touch has gone back to `Idle` (section 4.5).
    pub fn finger_down(&self) -> bool {
        !matches!(self, Lift::Idle)
    }

    /// The touch-down point and filtered point of a touch in `Down` or `Lifting`.
    pub fn points(&self) -> Option<(i32, i32)> {
        match *self {
            Lift::Down { y0, yf } | Lift::Lifting { y0, yf, .. } => Some((y0, yf)),
            _ => None,
        }
    }
}

/// The sampler's whole state: the lift machine, the thresholds and the calibration.
#[derive(Clone, Copy, Debug)]
pub struct Touch {
    pub lift: Lift,
    pub limits: Limits,
    pub cal: Calibration,
    /// The last sample's time, ms, for the stall rule.
    pub at: Option<u32>,
    /// The last classified sample (for the debug line).
    pub last: Sample,
}

impl Default for Touch {
    fn default() -> Self {
        Touch::new(LIMITS, CALIBRATION)
    }
}

impl Touch {
    /// An idle panel.
    pub const fn new(limits: Limits, cal: Calibration) -> Self {
        Touch {
            lift: Lift::Idle,
            limits,
            cal,
            at: None,
            last: Sample::Up,
        }
    }

    /// Classify one sample taken at `now` (ms) and step the machine.
    pub fn sample(&mut self, r: &Reading, now: u32) -> Sample {
        let (touching, near) = match self.lift {
            Lift::Idle => (false, None),
            Lift::Pressing { last, .. } => (false, Some(last)),
            Lift::Down { yf, .. } | Lift::Lifting { yf, .. } => (true, Some(yf)),
        };
        let s = classify(r, touching, near, &self.limits, &self.cal);
        self.step(s, now);
        self.at = Some(now);
        self.last = s;
        s
    }

    /// Step the machine on a classified sample at `now` (ms).
    pub fn step(&mut self, s: Sample, now: u32) {
        if let Lift::Lifting { since, .. } = self.lift {
            if now.wrapping_sub(since) >= LIFT_CONFIRM_MS {
                self.lift = Lift::Idle;
            }
        }
        self.lift = match (self.lift, s) {
            (Lift::Idle, Sample::Up) => Lift::Idle,
            (Lift::Idle, Sample::Valid { y, .. }) => Self::pressing(1, y, y),
            (Lift::Pressing { .. }, Sample::Up) => Lift::Idle,
            (Lift::Pressing { n, sum, .. }, Sample::Valid { y, .. }) => {
                Self::pressing(n + 1, sum + y, y)
            }
            (Lift::Down { y0, yf }, Sample::Up) => Lift::Lifting {
                y0,
                yf,
                since: now,
                resume: 0,
            },
            (Lift::Down { y0, yf }, Sample::Valid { y, .. }) => Lift::Down {
                y0,
                yf: yf + (y - yf) / 4,
            },
            (Lift::Lifting { y0, yf, since, .. }, Sample::Up) => Lift::Lifting {
                y0,
                yf,
                since,
                resume: 0,
            },
            (
                Lift::Lifting {
                    y0,
                    yf,
                    since,
                    resume,
                },
                Sample::Valid { y, .. },
            ) => {
                // A Valid sample here is within the jump bound of `yf` (the classifier's `near`).
                let yf = yf + (y - yf) / 4;
                if resume + 1 >= N_RESUME {
                    Lift::Down { y0, yf }
                } else {
                    Lift::Lifting {
                        y0,
                        yf,
                        since,
                        resume: resume + 1,
                    }
                }
            }
        };
    }

    fn pressing(n: u8, sum: i32, last: i32) -> Lift {
        if n >= N_DOWN {
            let mean = sum / i32::from(n);
            Lift::Down { y0: mean, yf: mean }
        } else {
            Lift::Pressing { n, sum, last }
        }
    }

    /// The state as the consumers read it at `now` (ms): a sampler that has produced nothing for
    /// [`STALL_MS`] reads as Up, which is a release. The points of a live touch (`Down` only:
    /// `Lifting` demands zero).
    pub fn view(&self, now: u32) -> TouchView {
        let stalled = self.at.is_none_or(|t| now.wrapping_sub(t) >= STALL_MS);
        if stalled {
            return TouchView {
                drive: None,
                overlay: self.lift.points().map(|p| (p, false)),
            };
        }
        TouchView {
            drive: match self.lift {
                Lift::Down { y0, yf } => Some((y0, yf)),
                _ => None,
            },
            overlay: self
                .lift
                .points()
                .map(|p| (p, matches!(self.lift, Lift::Down { .. }))),
        }
    }
}

/// What the touch means to the demand and the overlay.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct TouchView {
    /// `(y0, yf)` while the demand follows the finger (`Down`, the sampler live).
    pub drive: Option<(i32, i32)>,
    /// `((y0, yf), live)` while the overlay draws: `live` in `Down`, not in `Lifting`.
    pub overlay: Option<((i32, i32), bool)>,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 24 bytes an XPT2046 would clock out for these conversions.
    fn script(z1: u16, z2: u16, y: [u16; 3], x: [u16; 3]) -> [u8; FRAME_LEN] {
        let vals = [z1, z2, y[0], y[1], y[2], x[0], x[1], x[2]];
        let mut f = [0u8; FRAME_LEN];
        for (i, v) in vals.iter().enumerate() {
            let w = v << 3;
            f[i * 3 + 1] = (w >> 8) as u8;
            f[i * 3 + 2] = w as u8;
        }
        f
    }

    #[test]
    fn the_frame_carries_the_section_4_2_commands_in_order() {
        let f = tx_frame();
        assert_eq!(FRAME_LEN, 24);
        let cmds: [u8; 8] = core::array::from_fn(|i| f[i * 3]);
        assert_eq!(cmds, [0xB1, 0xC1, 0x91, 0x91, 0x91, 0xD1, 0xD1, 0xD0]);
        for i in 0..8 {
            assert_eq!((f[i * 3 + 1], f[i * 3 + 2]), (0, 0));
        }
    }

    #[test]
    fn the_twelve_bit_results_assemble_from_the_scripted_exchange() {
        let rx = script(0x123, 0xFFF, [0x800, 0x001, 0x7FF], [0xABC, 0x000, 0xFFF]);
        let r = Raw::parse(&rx);
        assert_eq!(
            r,
            Raw {
                z1: 0x123,
                z2: 0xFFF,
                y: [0x800, 0x001, 0x7FF],
                x: [0xABC, 0x000, 0xFFF]
            }
        );
        // The busy bit (the first bit of the byte after the command) is not part of the result.
        let mut busy = rx;
        busy[1] |= 0x80;
        assert_eq!(Raw::parse(&busy).z1, 0x123);
        assert_eq!(r.z(), 0x123 + 4095 - 0xFFF);
    }

    fn reading(z: i32, x: [u16; 3], y: [u16; 3]) -> Reading {
        // z = z1 + 4095 - z2 with z2 = 4095 - (z - z1); pick z1 = 0.
        Reading {
            pen_down: true,
            transfer_ok: true,
            raw: Raw {
                z1: 0,
                z2: (4095 - z) as u16,
                y,
                x,
            },
        }
    }

    /// A calibration where raw 100..=3995 maps 1:1 onto pixels of a big panel, so raw counts read
    /// as pixels in the tests.
    const IDENT: Calibration = Calibration {
        swap: false,
        x_raw: (0, 4000),
        y_raw: (0, 4000),
        width: 4000,
        height: 4000,
    };

    fn good() -> Reading {
        reading(1000, [2000; 3], [2000; 3])
    }

    #[test]
    fn a_good_sample_is_valid() {
        let s = classify(&good(), false, None, &LIMITS, &IDENT);
        assert_eq!(
            s,
            Sample::Valid {
                x: 2000 * SUB,
                y: 2000 * SUB,
                z: 1000
            }
        );
    }

    #[test]
    fn each_rule_alone_turns_a_valid_sample_up() {
        let c = |r: &Reading| classify(r, false, None, &LIMITS, &IDENT);
        let mut r = good();
        r.pen_down = false;
        assert_eq!(c(&r), Sample::Up, "PENIRQ high");
        let mut r = good();
        r.transfer_ok = false;
        assert_eq!(c(&r), Sample::Up, "failed transfer");
        assert_eq!(
            c(&reading(399, [2000; 3], [2000; 3])),
            Sample::Up,
            "z under Z_ON"
        );
        assert_eq!(
            c(&reading(1000, [2000, 2000, 2049], [2000; 3])),
            Sample::Up,
            "x spread"
        );
        assert_eq!(
            c(&reading(1000, [2000; 3], [1951, 2000, 2000])),
            Sample::Up,
            "y spread"
        );
        assert_eq!(
            c(&reading(1000, [99; 3], [2000; 3])),
            Sample::Up,
            "x at the low rail"
        );
        assert_eq!(
            c(&reading(1000, [2000; 3], [3996; 3])),
            Sample::Up,
            "y at the high rail"
        );
        // Inside every bound: still Valid.
        assert_ne!(c(&reading(1000, [2000, 2000, 2048], [100; 3])), Sample::Up);
        assert_ne!(c(&reading(400, [3995; 3], [2000; 3])), Sample::Up);
        // A jump past JUMP_MAX from the filtered point while a touch is in progress.
        let near = Some(2000 * SUB - 25 * SUB);
        assert_eq!(
            classify(&good(), true, near, &LIMITS, &IDENT),
            Sample::Up,
            "jump"
        );
        let near = Some(2000 * SUB - 24 * SUB);
        assert_ne!(classify(&good(), true, near, &LIMITS, &IDENT), Sample::Up);
    }

    #[test]
    fn a_z_between_z_off_and_z_on_keeps_a_touch_but_starts_none() {
        let r = reading(350, [2000; 3], [2000; 3]);
        assert_eq!(classify(&r, false, None, &LIMITS, &IDENT), Sample::Up);
        assert_ne!(
            classify(&r, true, Some(2000 * SUB), &LIMITS, &IDENT),
            Sample::Up
        );
        let r = reading(299, [2000; 3], [2000; 3]);
        assert_eq!(
            classify(&r, true, Some(2000 * SUB), &LIMITS, &IDENT),
            Sample::Up
        );
    }

    fn at(y: i32) -> Sample {
        Sample::Valid {
            x: 0,
            y: y * SUB,
            z: 1000,
        }
    }

    /// Feed `samples` 2 ms apart from `t`, returning the time after the last.
    fn feed(t: &mut Touch, start: u32, samples: &[Sample]) -> u32 {
        let mut now = start;
        for s in samples {
            t.step(*s, now);
            t.at = Some(now);
            now += PERIOD_MS;
        }
        now
    }

    #[test]
    fn a_touch_down_needs_n_down_valid_samples_and_sets_y0_to_their_mean() {
        let mut t = Touch::default();
        let now = feed(&mut t, 0, &[at(100), at(102), at(104)]);
        assert!(matches!(t.lift, Lift::Pressing { n: 3, .. }));
        assert!(t.lift.finger_down());
        assert_eq!(t.view(now).drive, None, "no demand while pressing");
        feed(&mut t, now, &[at(106)]);
        assert_eq!(
            t.lift,
            Lift::Down {
                y0: 103 * SUB,
                yf: 103 * SUB
            }
        );
        // An Up while pressing goes back to Idle.
        let mut t = Touch::default();
        feed(&mut t, 0, &[at(100), Sample::Up]);
        assert_eq!(t.lift, Lift::Idle);
    }

    fn down_at(y: i32) -> (Touch, u32) {
        let mut t = Touch::default();
        let now = feed(&mut t, 0, &[at(y); N_DOWN as usize]);
        assert!(matches!(t.lift, Lift::Down { .. }));
        (t, now)
    }

    #[test]
    fn one_up_sample_in_down_gives_zero_demand_at_once() {
        let (mut t, now) = down_at(200);
        let now = feed(&mut t, now, &[at(150)]);
        assert!(t.view(now).drive.is_some());
        let now = feed(&mut t, now, &[Sample::Up]);
        assert!(matches!(t.lift, Lift::Lifting { .. }));
        assert_eq!(t.view(now).drive, None);
        assert!(t.lift.finger_down());
    }

    #[test]
    fn a_dropout_shorter_than_lift_confirm_resumes_with_the_same_y0() {
        let (mut t, now) = down_at(200);
        let (y0, _) = t.lift.points().unwrap();
        let now = feed(&mut t, now, &[at(190), at(185)]);
        let now = feed(&mut t, now, &[Sample::Up; 10]); // 20 ms
        assert!(matches!(t.lift, Lift::Lifting { .. }));
        feed(&mut t, now, &[at(184), at(183)]);
        let Lift::Down { y0: back, .. } = t.lift else {
            panic!("did not resume: {:?}", t.lift)
        };
        assert_eq!(back, y0);
    }

    #[test]
    fn a_dropout_longer_confirms_the_release_and_the_next_touch_sets_a_new_y0() {
        let (mut t, now) = down_at(200);
        let now = feed(&mut t, now, &[Sample::Up; 21]); // 42 ms > LIFT_CONFIRM
        assert_eq!(t.lift, Lift::Idle);
        feed(&mut t, now, &[at(120); N_DOWN as usize]);
        assert_eq!(
            t.lift,
            Lift::Down {
                y0: 120 * SUB,
                yf: 120 * SUB
            }
        );
    }

    #[test]
    fn a_resume_that_jumps_past_jump_max_does_not_resume() {
        let mut t = Touch::new(LIMITS, IDENT);
        let r = |y: u16| reading(1000, [2000; 3], [y; 3]);
        let mut now = 0;
        for _ in 0..N_DOWN {
            t.sample(&r(2000), now);
            now += 2;
        }
        assert!(matches!(t.lift, Lift::Down { .. }));
        let mut up = r(2000);
        up.pen_down = false;
        t.sample(&up, now);
        now += 2;
        // Two samples 30 px away: the classifier reads them Up, so the touch stays lifting.
        t.sample(&r(2030), now);
        t.sample(&r(2030), now + 2);
        assert!(matches!(t.lift, Lift::Lifting { .. }), "{:?}", t.lift);
    }

    #[test]
    fn every_non_idle_state_counts_as_finger_down() {
        assert!(!Lift::Idle.finger_down());
        assert!(Lift::Pressing {
            n: 1,
            sum: 0,
            last: 0
        }
        .finger_down());
        assert!(Lift::Down { y0: 0, yf: 0 }.finger_down());
        assert!(Lift::Lifting {
            y0: 0,
            yf: 0,
            since: 0,
            resume: 0
        }
        .finger_down());
    }

    #[test]
    fn a_stalled_sampler_reads_up_after_ten_ms() {
        let (t, now) = down_at(200);
        let last = now - PERIOD_MS;
        assert!(t.view(last + 9).drive.is_some());
        assert_eq!(t.view(last + STALL_MS).drive, None);
        assert_eq!(Touch::default().view(0).drive, None);
    }

    #[test]
    fn the_down_filter_is_an_iir_of_a_quarter() {
        let (mut t, now) = down_at(200);
        feed(&mut t, now, &[at(216)]);
        assert_eq!(
            t.lift,
            Lift::Down {
                y0: 200 * SUB,
                yf: 204 * SUB
            }
        );
    }
}
