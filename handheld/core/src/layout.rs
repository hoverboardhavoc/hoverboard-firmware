//! The Ride view's layout as data (`specs/handheld-remote.md` section 5.2): each widget's fixed
//! rectangle on the 240 x 320 portrait panel, the redraw rule (a widget redraws only when the
//! state it draws from changes), the ring's segments, and the rectangles the throttle overlay
//! covers and vacates (section 5.7).

/// Panel width, pixels (portrait).
pub const WIDTH: i32 = 240;
/// Panel height, pixels.
pub const HEIGHT: i32 = 320;

/// A rectangle, half-open: `x0..x1`, `y0..y1`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Rect {
    pub x0: i32,
    pub y0: i32,
    pub x1: i32,
    pub y1: i32,
}

impl Rect {
    pub const fn new(x0: i32, y0: i32, x1: i32, y1: i32) -> Rect {
        Rect { x0, y0, x1, y1 }
    }

    pub const fn width(&self) -> i32 {
        self.x1 - self.x0
    }

    pub const fn height(&self) -> i32 {
        self.y1 - self.y0
    }

    /// Whether the two share a pixel.
    pub const fn overlaps(&self, o: &Rect) -> bool {
        self.x0 < o.x1 && o.x0 < self.x1 && self.y0 < o.y1 && o.y0 < self.y1
    }

    /// The smallest rectangle holding both.
    pub fn union(&self, o: &Rect) -> Rect {
        Rect::new(
            self.x0.min(o.x0),
            self.y0.min(o.y0),
            self.x1.max(o.x1),
            self.y1.max(o.y1),
        )
    }

    /// The part on the panel; `None` when nothing is.
    pub fn on_panel_part(&self) -> Option<Rect> {
        let r = Rect::new(
            self.x0.max(0),
            self.y0.max(0),
            self.x1.min(WIDTH),
            self.y1.min(HEIGHT),
        );
        (r.x0 < r.x1 && r.y0 < r.y1).then_some(r)
    }

    /// Whether it lies on the panel.
    pub const fn on_panel(&self) -> bool {
        self.x0 >= 0
            && self.y0 >= 0
            && self.x1 <= WIDTH
            && self.y1 <= HEIGHT
            && self.x0 < self.x1
            && self.y0 < self.y1
    }
}

/// The instruments' widgets (section 5.2).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Widget {
    TopStrip,
    /// The battery ring and the speed inside it: one rectangle, the ring's box above the attitude
    /// corners.
    Dial,
    AttitudeLeft,
    AttitudeRight,
    PackLine,
    PhaseLeft,
    PhaseRight,
    ArmBand,
    Statement,
}

/// Every instrument widget, in drawing order.
pub const WIDGETS: [Widget; 9] = [
    Widget::TopStrip,
    Widget::Dial,
    Widget::AttitudeLeft,
    Widget::AttitudeRight,
    Widget::PackLine,
    Widget::PhaseLeft,
    Widget::PhaseRight,
    Widget::ArmBand,
    Widget::Statement,
];

/// The battery ring: centre, radius to the stroke's centre line, stroke width.
pub const RING_CENTRE: (i32, i32) = (120, 108);
pub const RING_RADIUS: i32 = 76;
pub const RING_STROKE: i32 = 10;
/// The speed's centre inside the ring.
pub const SPEED_CENTRE: (i32, i32) = (120, 100);
/// The speed's own rectangle inside the ring: the figure, "km/h" and "R". A change of speed alone
/// redraws this and not the ring.
pub const SPEED_BOX: Rect = Rect::new(72, 72, 168, 142);
/// The ring's segments: ten over 240 degrees from 150 degrees (clockwise, y down, 0 at three
/// o'clock), each 24 degrees less [`RING_GAP_DEG`] at each end.
pub const RING_SEGMENTS: u8 = 10;
pub const RING_START_DEG: f32 = 150.0;
pub const RING_STEP_DEG: f32 = 24.0;
pub const RING_GAP_DEG: f32 = 1.2;

/// The ring segment a pixel at `(dx, dy)` from [`RING_CENTRE`] belongs to, or `None` for a pixel
/// off the stroke, in a gap, or in the opening at the bottom.
pub fn ring_segment(dx: i32, dy: i32) -> Option<u8> {
    let d2 = dx * dx + dy * dy;
    let inner = RING_RADIUS - RING_STROKE / 2;
    let outer = RING_RADIUS + RING_STROKE / 2;
    if d2 < inner * inner || d2 >= outer * outer {
        return None;
    }
    let deg = libm::atan2f(dy as f32, dx as f32).to_degrees();
    // `deg` is in -180..=180, so one turn added brings it to 0..360.
    let mut rel = deg - RING_START_DEG;
    if rel < 0.0 {
        rel += 360.0;
    }
    let seg = (rel / RING_STEP_DEG) as u8;
    if seg >= RING_SEGMENTS {
        return None;
    }
    let within = rel - f32::from(seg) * RING_STEP_DEG;
    (RING_GAP_DEG..=RING_STEP_DEG - RING_GAP_DEG)
        .contains(&within)
        .then_some(seg)
}

impl Widget {
    /// The widget's rectangle.
    pub const fn rect(self) -> Rect {
        match self {
            Widget::TopStrip => Rect::new(0, 0, 240, 20),
            // The ring's outer edge is 81 px from (120, 108); its ends at 150 and 30 degrees less
            // the 1.2 degree gap stop above y 148, clear of the attitude corners at 150.
            Widget::Dial => Rect::new(39, 27, 202, 148),
            Widget::AttitudeLeft => Rect::new(2, 150, 58, 216),
            Widget::AttitudeRight => Rect::new(182, 150, 238, 216),
            Widget::PackLine => Rect::new(60, 192, 180, 210),
            Widget::PhaseLeft => Rect::new(8, 220, 116, 248),
            Widget::PhaseRight => Rect::new(124, 220, 232, 248),
            Widget::ArmBand => Rect::new(8, 254, 232, 290),
            Widget::Statement => Rect::new(4, 296, 236, 318),
        }
    }
}

/// The cards that replace the instruments under the top strip and above the statement line
/// (section 5.5), inside an 8 px gutter.
pub const CARD_AREA: Rect = Rect::new(0, 20, 240, 290);
pub const CARD: Rect = Rect::new(8, 28, 232, 282);
/// The lamp grid's area: the phase bars and the arm band (section 5.6).
pub const LAMP_GRID: Rect = Rect::new(8, 220, 232, 290);
/// The lamp grid's cells: five columns of 41 px with 4 px gaps, two rows of 31 px with 8 px
/// between.
pub const fn lamp_cell(col: i32, row: i32) -> Rect {
    let x0 = LAMP_GRID.x0 + col * (41 + 4);
    let y0 = LAMP_GRID.y0 + row * (31 + 8);
    Rect::new(x0, y0, x0 + 41, y0 + 31)
}

/// The throttle thumb bar's height, pixels.
pub const THUMB_H: i32 = 10;
/// The zero line's height, pixels.
pub const ZERO_H: i32 = 2;
/// The rows above the thumb bar its percent text takes.
pub const THUMB_TEXT_H: i32 = 15;
/// DIFFERENTIAL's turn zero line: its width, and its rows (from under the top strip to the top
/// of the arm zone).
pub const TURN_W: i32 = 2;
pub const TURN_Y: (i32, i32) = (20, crate::arm::ARM_ZONE_Y);
/// The width of the `xf` mark across the thumb bar.
pub const MARK_W: i32 = 4;

/// The zero line at `y0`, full width.
pub fn zero_rect(y0: i32) -> Rect {
    let (a, b) = band(y0, ZERO_H);
    Rect::new(0, a, WIDTH, b)
}

/// The thumb bar at `yf` with its percent text above it, full width.
pub fn thumb_rect(yf: i32) -> Rect {
    let (a, b) = band(yf, THUMB_H);
    Rect::new(0, (a - THUMB_TEXT_H).max(0), WIDTH, b)
}

/// DIFFERENTIAL's turn zero line at `x0`.
pub fn turn_rect(x0: i32) -> Rect {
    let x = x0.clamp(0, WIDTH - TURN_W);
    Rect::new(x, TURN_Y.0, x + TURN_W, TURN_Y.1)
}

/// The character span, `[first, end)`, over which two lines of monospace text differ, or `None`
/// when they are the same: the statement line redraws those columns alone (section 5.5, "the
/// counter's text alone redrawn").
pub fn text_diff(a: &str, b: &str) -> Option<(usize, usize)> {
    let (na, nb) = (a.chars().count(), b.chars().count());
    let first = a.chars().zip(b.chars()).position(|(x, y)| x != y);
    match first {
        None if na == nb => None,
        None => Some((na.min(nb), na.max(nb))),
        Some(f) if na != nb => Some((f, na.max(nb))),
        Some(f) => {
            // Equal lengths: the last differing column, counted from the end.
            let from_end = a
                .chars()
                .rev()
                .zip(b.chars().rev())
                .position(|(x, y)| x != y)
                .unwrap_or(0);
            let end = na - from_end;
            Some((f, end))
        }
    }
}

/// The rows a bar of height `h` centred on row `y` covers, clamped to the panel.
pub const fn band(y: i32, h: i32) -> (i32, i32) {
    let y0 = y - h / 2;
    let a = if y0 < 0 { 0 } else { y0 };
    let b = if y0 + h > HEIGHT { HEIGHT } else { y0 + h };
    (a, b)
}

/// The widgets whose rectangles meet `r`: the ones a paint of `r` draws (section 5.7: what the
/// overlay vacates is painted by every widget that meets it).
pub fn meeting(r: Rect) -> impl Iterator<Item = Widget> {
    WIDGETS.into_iter().filter(move |w| w.rect().overlaps(&r))
}

/// A widget's redraw rule: it keeps the state it last drew, and redraws when the new state
/// differs. The first frame (nothing drawn) always draws.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Drawn<S> {
    last: Option<S>,
}

impl<S: PartialEq + Copy> Drawn<S> {
    /// Whether `next` needs drawing; records it as drawn when it does.
    pub fn update(&mut self, next: S) -> bool {
        if self.last == Some(next) {
            return false;
        }
        self.last = Some(next);
        true
    }

    /// Forget what was drawn (a screen change redraws everything).
    pub fn invalidate(&mut self) {
        self.last = None;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    #[test]
    fn no_two_widget_rectangles_overlap_and_all_are_on_the_panel() {
        for (i, a) in WIDGETS.iter().enumerate() {
            assert!(a.rect().on_panel(), "{a:?}");
            for b in &WIDGETS[i + 1..] {
                assert!(!a.rect().overlaps(&b.rect()), "{a:?} and {b:?}");
            }
        }
        assert!(CARD_AREA.on_panel() && CARD.on_panel() && LAMP_GRID.on_panel());
        for c in 0..5 {
            for r in 0..2 {
                let cell = lamp_cell(c, r);
                assert!(
                    cell.x1 <= LAMP_GRID.x1 && cell.y1 <= LAMP_GRID.y1,
                    "{cell:?}"
                );
            }
        }
    }

    #[test]
    fn the_ring_fits_its_rectangle() {
        let r = Widget::Dial.rect();
        let outer = RING_RADIUS + RING_STROKE / 2;
        assert_eq!(r.x0, RING_CENTRE.0 - outer);
        assert_eq!(r.y0, RING_CENTRE.1 - outer);
        assert!(r.x1 > RING_CENTRE.0 + outer);
        // The segments' lowest point: the outer edge at 151.2 degrees (y down, 0 at three o'clock).
        let lowest = RING_CENTRE.1 as f64 + outer as f64 * (151.2f64).to_radians().sin();
        assert!(lowest < r.y1 as f64, "{lowest}");
    }

    #[test]
    fn a_change_in_its_input_redraws_a_widget_and_an_unrelated_one_does_not() {
        let mut pack: Drawn<(u16, u8)> = Drawn::default();
        let mut speed: Drawn<i32> = Drawn::default();
        assert!(pack.update((312, 72)), "the first frame draws");
        assert!(speed.update(0));
        // The pack voltage changes; the speed does not.
        assert!(pack.update((311, 72)));
        assert!(!speed.update(0));
        assert!(!pack.update((311, 72)));
        pack.invalidate();
        assert!(pack.update((311, 72)));
    }

    #[test]
    fn the_speed_box_lies_inside_the_ring_and_inside_the_dial() {
        let d = Widget::Dial.rect();
        assert!(SPEED_BOX.x0 >= d.x0 && SPEED_BOX.x1 <= d.x1);
        assert!(SPEED_BOX.y0 >= d.y0 && SPEED_BOX.y1 <= d.y1);
        let inner = RING_RADIUS - RING_STROKE / 2;
        for (x, y) in [
            (SPEED_BOX.x0, SPEED_BOX.y0),
            (SPEED_BOX.x1, SPEED_BOX.y0),
            (SPEED_BOX.x0, SPEED_BOX.y1),
            (SPEED_BOX.x1, SPEED_BOX.y1),
        ] {
            let (dx, dy) = (x - RING_CENTRE.0, y - RING_CENTRE.1);
            assert!(dx * dx + dy * dy < inner * inner, "({x}, {y})");
        }
    }

    #[test]
    fn the_ring_segments_run_clockwise_from_150_degrees_with_gaps_and_an_opening() {
        let at = |deg: f32, r: f32| {
            let a = deg.to_radians();
            ring_segment(
                libm::roundf(r * libm::cosf(a)) as i32,
                libm::roundf(r * libm::sinf(a)) as i32,
            )
        };
        let r = RING_RADIUS as f32;
        for i in 0..10 {
            let mid = 150.0 + 24.0 * i as f32 + 12.0;
            assert_eq!(at(mid, r), Some(i), "segment {i}");
        }
        assert_eq!(at(150.0, r), None, "the gap at the start");
        assert_eq!(at(270.0, r), None, "the gap between 4 and 5, at the top");
        assert_eq!(at(90.0, r), None, "the opening, facing down");
        assert_eq!(at(162.0, r - 6.0), None, "inside the stroke");
        assert_eq!(at(162.0, r + 6.0), None, "outside the stroke");
        assert_eq!(at(162.0, r - 4.0), Some(0));
        assert_eq!(at(162.0, r + 4.0), Some(0));
    }

    #[test]
    fn the_overlay_rectangles() {
        assert_eq!(zero_rect(100), Rect::new(0, 99, 240, 101));
        assert_eq!(thumb_rect(100), Rect::new(0, 80, 240, 105));
        assert_eq!(thumb_rect(5), Rect::new(0, 0, 240, 10));
        assert_eq!(turn_rect(120), Rect::new(120, 20, 122, 250));
        assert_eq!(turn_rect(239), Rect::new(238, 20, 240, 250));
        assert!(thumb_rect(318).on_panel());
    }

    #[test]
    fn the_text_diff_is_the_changed_columns() {
        assert_eq!(text_diff("Reading 3/22", "Reading 3/22"), None);
        assert_eq!(text_diff("Reading 3/22", "Reading 4/22"), Some((8, 9)));
        assert_eq!(text_diff("Reading 9/22", "Reading 10/22"), Some((8, 13)));
        assert_eq!(text_diff("abc", "abcde"), Some((3, 5)));
        assert_eq!(text_diff("abcde", "abc"), Some((3, 5)));
        assert_eq!(text_diff("x1y2z", "x3y4z"), Some((1, 4)));
    }

    #[test]
    fn the_overlays_vacated_band_names_exactly_the_widgets_it_meets() {
        let (y0, y1) = band(230, THUMB_H);
        assert_eq!((y0, y1), (225, 235));
        let rows = |y0, y1| Rect::new(0, y0, WIDTH, y1);
        let w: Vec<Widget> = meeting(rows(y0, y1)).collect();
        assert_eq!(w, [Widget::PhaseLeft, Widget::PhaseRight]);
        let w: Vec<Widget> = meeting(rows(145, 155)).collect();
        assert_eq!(
            w,
            [Widget::Dial, Widget::AttitudeLeft, Widget::AttitudeRight]
        );
        let w: Vec<Widget> = meeting(rows(249, 253)).collect();
        assert!(
            w.is_empty(),
            "the gap between the phase bars and the arm band"
        );
        // A turn zero line meets every widget in its columns between the strip and the band.
        let w: Vec<Widget> = meeting(turn_rect(30)).collect();
        assert_eq!(w, [Widget::AttitudeLeft, Widget::PhaseLeft]);
        assert_eq!(band(2, THUMB_H), (0, 7));
        assert_eq!(band(318, THUMB_H), (313, 320));
    }
}
