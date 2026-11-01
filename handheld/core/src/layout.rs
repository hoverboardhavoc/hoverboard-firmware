//! The Ride view's layout as data (`specs/handheld-remote.md` section 5.2): each widget's fixed
//! rectangle on the 240 x 320 portrait panel, the redraw rule (a widget redraws only when the
//! state it draws from changes), and the rows the throttle overlay vacates (section 5.7).

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

    /// Whether the rows `y0..y1` cross this rectangle.
    pub const fn meets_rows(&self, y0: i32, y1: i32) -> bool {
        self.y0 < y1 && y0 < self.y1
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

/// The rows a bar of height `h` centred on row `y` covers, clamped to the panel.
pub const fn band(y: i32, h: i32) -> (i32, i32) {
    let y0 = y - h / 2;
    let a = if y0 < 0 { 0 } else { y0 };
    let b = if y0 + h > HEIGHT { HEIGHT } else { y0 + h };
    (a, b)
}

/// The widgets whose rectangles meet the rows `y0..y1`, which redraw clipped to them when the
/// overlay leaves those rows.
pub fn meeting(y0: i32, y1: i32) -> impl Iterator<Item = Widget> {
    WIDGETS
        .into_iter()
        .filter(move |w| w.rect().meets_rows(y0, y1))
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
    fn the_overlays_vacated_band_names_exactly_the_widgets_it_meets() {
        let (y0, y1) = band(230, THUMB_H);
        assert_eq!((y0, y1), (225, 235));
        let w: Vec<Widget> = meeting(y0, y1).collect();
        assert_eq!(w, [Widget::PhaseLeft, Widget::PhaseRight]);
        let w: Vec<Widget> = meeting(145, 155).collect();
        assert_eq!(
            w,
            [Widget::Dial, Widget::AttitudeLeft, Widget::AttitudeRight]
        );
        let w: Vec<Widget> = meeting(249, 253).collect();
        assert!(
            w.is_empty(),
            "the gap between the phase bars and the arm band"
        );
        assert_eq!(band(2, THUMB_H), (0, 7));
        assert_eq!(band(318, THUMB_H), (313, 320));
    }
}
