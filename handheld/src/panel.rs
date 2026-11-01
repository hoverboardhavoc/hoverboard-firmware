//! Drawing the Ride view on the ILI9341 (`specs/handheld-remote.md` section 5): no framebuffer;
//! each widget keeps the state it last drew and redraws its own rectangle when that changes.

use core::fmt::Write;

use embedded_graphics::mono_font::iso_8859_1::{FONT_10X20, FONT_6X10, FONT_8X13};
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{
    Arc, Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle,
};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use heapless::String;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};
use u8g2_fonts::{fonts, FontRenderer};

use handheld_core::layout::{self, Drawn, Rect, Widget};
use handheld_core::model::{
    ArmBand, Body, Card, Corner, Dial, Dot, LampCell, Lower, Overlay, Pack, PhaseBar, Ride,
    SpeedShow, Statement, Tone, Top, View,
};
use handheld_core::readings::attitude::Attitude;

/// `specs/remote-ui.md` 3.10's tokens in RGB565.
pub mod color {
    use embedded_graphics::pixelcolor::Rgb565;

    const fn hex(v: u32) -> Rgb565 {
        Rgb565::new(
            ((v >> 16) as u8) >> 3,
            ((v >> 8) as u8) >> 2,
            (v as u8) >> 3,
        )
    }
    pub const BG: Rgb565 = hex(0x000000);
    pub const BG_ELEVATED: Rgb565 = hex(0x101012);
    pub const BG_CODE: Rgb565 = hex(0x141417);
    pub const BORDER: Rgb565 = hex(0x1F1F1F);
    pub const FG: Rgb565 = hex(0xF5F5F5);
    pub const FG_MUTED: Rgb565 = hex(0xB2B2B8);
    pub const FG_FAINT: Rgb565 = hex(0x5A5A62);
    pub const PRIMARY: Rgb565 = hex(0xFF5A00);
    pub const NEON: Rgb565 = hex(0x2FD3F0);
    pub const CAUTION: Rgb565 = hex(0xFFD23F);
    pub const WARNING: Rgb565 = hex(0xFF1F7A);
}

use color::*;

fn er(r: Rect) -> Rectangle {
    Rectangle::new(
        Point::new(r.x0, r.y0),
        Size::new(r.width() as u32, r.height() as u32),
    )
}

fn style(font: &'static MonoFont<'static>, fg: Rgb565) -> MonoTextStyle<'static, Rgb565> {
    MonoTextStyleBuilder::new()
        .font(font)
        .text_color(fg)
        .background_color(BG)
        .build()
}

pub fn text<D: DrawTarget<Color = Rgb565>>(
    d: &mut D,
    s: &str,
    at: Point,
    font: &'static MonoFont<'static>,
    fg: Rgb565,
    align: Alignment,
) {
    let ts = TextStyleBuilder::new()
        .alignment(align)
        .baseline(Baseline::Top)
        .build();
    let _ = Text::with_text_style(s, at, style(font, fg), ts).draw(d);
}

pub fn fill<D: DrawTarget<Color = Rgb565>>(d: &mut D, r: Rect, c: Rgb565) {
    let _ = er(r).into_styled(PrimitiveStyle::with_fill(c)).draw(d);
}

/// The caution glyph (3.10): a small filled triangle with a dark bar, `size` px tall.
fn caution_glyph<D: DrawTarget<Color = Rgb565>>(d: &mut D, at: Point, size: i32, c: Rgb565) {
    let tri = embedded_graphics::primitives::Triangle::new(
        at + Point::new(size / 2, 0),
        at + Point::new(0, size),
        at + Point::new(size, size),
    );
    let _ = tri.into_styled(PrimitiveStyle::with_fill(c)).draw(d);
    let _ = Line::new(
        at + Point::new(size / 2, size / 3),
        at + Point::new(size / 2, size * 2 / 3),
    )
    .into_styled(PrimitiveStyle::with_stroke(BG, 1))
    .draw(d);
}

/// What each widget last drew.
#[derive(Default)]
pub struct Cache {
    top: Drawn<u64>,
    body_kind: Drawn<u8>,
    card: Drawn<u64>,
    dial: Drawn<u64>,
    corners: [Drawn<u64>; 2],
    pack: Drawn<u64>,
    lower: Drawn<u64>,
    /// The arm band as last drawn, while the phase bars show.
    band: Option<ArmBand>,
    line: Drawn<u64>,
    armed: Drawn<bool>,
}

/// A cheap hash of a state's `Debug` form: the widgets' states are small, and this keeps the
/// cache to one word each.
fn hash<T: core::fmt::Debug>(t: &T) -> u64 {
    struct H(u64);
    impl core::fmt::Write for H {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            for b in s.bytes() {
                self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x100_0000_01b3);
            }
            Ok(())
        }
    }
    let mut h = H(0xcbf2_9ce4_8422_2325);
    let _ = write!(h, "{t:?}");
    h.0
}

/// Draw what changed in `v`. `debug` is the debug line's two rows, drawn where the statement
/// line is empty.
pub fn draw<D: DrawTarget<Color = Rgb565>>(
    d: &mut D,
    v: &View,
    debug: Option<(&str, &str)>,
    c: &mut Cache,
) {
    if c.top.update(hash(&v.top)) {
        draw_top(d, &v.top);
    }
    let kind = match v.body {
        Body::Card(_) => 0,
        Body::Ride(_) => 1,
    };
    if c.body_kind.update(kind) {
        fill(d, layout::CARD_AREA, BG);
        c.card.invalidate();
        c.dial.invalidate();
        c.corners = Default::default();
        c.pack.invalidate();
        c.lower.invalidate();
    }
    match &v.body {
        Body::Card(card) => {
            if c.card.update(hash(card)) {
                draw_card(d, card);
            }
        }
        Body::Ride(r) => draw_ride(d, r, c),
    }
    let line = (&v.line, debug);
    if c.line.update(hash(&line)) {
        draw_line(d, v.line.as_ref(), debug);
    }
    if c.armed.update(v.armed) || v.armed {
        let col = if v.armed { PRIMARY } else { BG };
        for r in [
            Rect::new(0, 0, 240, 2),
            Rect::new(0, 318, 240, 320),
            Rect::new(0, 0, 2, 320),
            Rect::new(238, 0, 240, 320),
        ] {
            fill(d, r, col);
        }
    }
}

fn draw_top<D: DrawTarget<Color = Rgb565>>(d: &mut D, t: &Top) {
    let r = Widget::TopStrip.rect();
    fill(d, r, BG);
    let mut x = 6;
    for dot in t.dots.iter().flatten() {
        draw_dot(d, dot, &mut x);
    }
    if t.test {
        text(
            d,
            "TEST",
            Point::new(234, 4),
            &FONT_8X13,
            FG_MUTED,
            Alignment::Right,
        );
    } else if t.low {
        text(
            d,
            "LOW",
            Point::new(234, 4),
            &FONT_8X13,
            CAUTION,
            Alignment::Right,
        );
        caution_glyph(d, Point::new(196, 5), 10, CAUTION);
    }
}

fn draw_dot<D: DrawTarget<Color = Rgb565>>(d: &mut D, dot: &Dot, x: &mut i32) {
    if dot.heard {
        let _ = Circle::new(Point::new(*x, 6), 8)
            .into_styled(PrimitiveStyle::with_fill(FG_MUTED))
            .draw(d);
    } else {
        caution_glyph(d, Point::new(*x - 1, 5), 10, CAUTION);
    }
    text(
        d,
        &dot.label,
        Point::new(*x + 12, 5),
        &FONT_6X10,
        FG_MUTED,
        Alignment::Left,
    );
    *x += 12 + 6 * dot.label.len() as i32 + 10;
}

fn draw_card<D: DrawTarget<Color = Rgb565>>(d: &mut D, card: &Card) {
    fill(d, layout::CARD_AREA, BG);
    let r = layout::CARD;
    let _ = RoundedRectangle::with_equal_corners(er(r), Size::new(12, 12))
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(BG_CODE)
                .stroke_color(BORDER)
                .stroke_width(1)
                .build(),
        )
        .draw(d);
    let mid = Point::new(120, 140);
    let card_text = |d: &mut D, s: &str, y: i32, font, fg| {
        let st = MonoTextStyleBuilder::new()
            .font(font)
            .text_color(fg)
            .background_color(BG_CODE)
            .build();
        let ts = TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build();
        let _ = Text::with_text_style(s, Point::new(120, y), st, ts).draw(d);
    };
    match card {
        Card::NoBridge => card_text(d, "No bridge set.", mid.y, &FONT_8X13, FG),
        Card::Connecting(a) => {
            card_text(d, "Connecting to", mid.y - 10, &FONT_8X13, FG);
            let mut s: String<24> = String::new();
            let _ = write!(
                s,
                "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\u{2026}",
                a[0], a[1], a[2], a[3], a[4], a[5]
            );
            card_text(d, &s, mid.y + 8, &FONT_8X13, FG_MUTED);
        }
        Card::MotorOff => {
            power_glyph(d, Point::new(120, 100));
            card_text(d, "Motor controller off", 140, &FONT_10X20, FG);
            card_text(d, "Switch it on.", 168, &FONT_8X13, FG);
        }
    }
}

/// The power glyph: a 300 degree arc open at the top and a bar down through the gap.
fn power_glyph<D: DrawTarget<Color = Rgb565>>(d: &mut D, centre: Point) {
    let st = PrimitiveStyle::with_stroke(FG_MUTED, 3);
    let _ = Arc::with_center(
        centre,
        36,
        Angle::from_degrees(-60.0),
        Angle::from_degrees(300.0),
    )
    .into_styled(st)
    .draw(d);
    let _ = Line::new(centre + Point::new(0, -20), centre + Point::new(0, -2))
        .into_styled(st)
        .draw(d);
}

fn draw_ride<D: DrawTarget<Color = Rgb565>>(d: &mut D, r: &Ride, c: &mut Cache) {
    if c.dial.update(hash(&r.dial)) {
        draw_dial(d, &r.dial);
    }
    for (i, w) in [Widget::AttitudeLeft, Widget::AttitudeRight]
        .iter()
        .enumerate()
    {
        if c.corners[i].update(hash(&r.corners[i])) {
            draw_corner(d, w.rect(), r.corners[i].as_ref());
        }
    }
    if c.pack.update(hash(&r.pack)) {
        draw_pack(d, &r.pack);
    }
    // The band is cached apart from the phase bars, so a hold's progress redraws only the band.
    let (key, band) = match &r.lower {
        Lower::Bars { phase, band } => (hash(&(0u8, phase)), Some(*band)),
        Lower::Grid(cells) => (hash(&(1u8, cells)), None),
    };
    if c.lower.update(key) {
        fill(d, layout::LAMP_GRID, BG);
        match &r.lower {
            Lower::Bars { phase, band } => {
                draw_phase(d, Widget::PhaseLeft.rect(), phase[0].as_ref());
                draw_phase(d, Widget::PhaseRight.rect(), phase[1].as_ref());
                draw_band(d, *band);
            }
            Lower::Grid(cells) => draw_grid(d, cells),
        }
        c.band = band;
    } else if let Some(b) = band {
        if c.band != Some(b) {
            update_band(d, c.band, b);
            c.band = Some(b);
        }
    }
}

fn draw_dial<D: DrawTarget<Color = Rgb565>>(d: &mut D, dial: &Dial) {
    fill(d, Widget::Dial.rect(), BG);
    let (cx, cy) = layout::RING_CENTRE;
    let diameter = (2 * layout::RING_RADIUS) as u32;
    for i in 0..10u8 {
        let start = 150.0 + 24.0 * i as f32 + 1.2;
        let col = if i < dial.lit {
            if dial.caution {
                CAUTION
            } else {
                PRIMARY
            }
        } else {
            BG_CODE
        };
        let _ = Arc::with_center(
            Point::new(cx, cy),
            diameter,
            Angle::from_degrees(start),
            Angle::from_degrees(24.0 - 2.4),
        )
        .into_styled(PrimitiveStyle::with_stroke(col, layout::RING_STROKE as u32))
        .draw(d);
    }
    let (sx, sy) = layout::SPEED_CENTRE;
    let big = FontRenderer::new::<fonts::u8g2_font_logisoso38_tn>();
    let (figure, faint) = match &dial.speed {
        SpeedShow::Value { text, .. } => (text.as_str(), false),
        _ => ("--", true),
    };
    let _ = big.render_aligned(
        figure,
        Point::new(sx, sy),
        VerticalPosition::Center,
        HorizontalAlignment::Center,
        FontColor::WithBackground {
            fg: if faint { FG_FAINT } else { FG },
            bg: BG,
        },
        d,
    );
    match &dial.speed {
        SpeedShow::Silent => text(
            d,
            "no board reporting",
            Point::new(sx, sy + 26),
            &FONT_6X10,
            FG_MUTED,
            Alignment::Center,
        ),
        SpeedShow::NoSpeed => text(
            d,
            "km/h",
            Point::new(sx, sy + 26),
            &FONT_8X13,
            FG_FAINT,
            Alignment::Center,
        ),
        SpeedShow::Value { reverse, .. } => {
            text(
                d,
                "km/h",
                Point::new(sx, sy + 26),
                &FONT_8X13,
                FG_MUTED,
                Alignment::Center,
            );
            if *reverse {
                text(
                    d,
                    "R",
                    Point::new(sx + 22, sy + 26),
                    &FONT_8X13,
                    FG,
                    Alignment::Left,
                );
            }
        }
    }
}

fn draw_corner<D: DrawTarget<Color = Rgb565>>(d: &mut D, r: Rect, c: Option<&Corner>) {
    fill(d, r, BG);
    let Some(c) = c else {
        return;
    };
    let cx = r.x0 + r.width() / 2;
    text(
        d,
        &c.label,
        Point::new(cx, r.y0),
        &FONT_6X10,
        NEON,
        Alignment::Center,
    );
    let centre = Point::new(cx, r.y0 + 12 + 14);
    let radius = 14;
    if let Attitude::Angles { pitch, roll } = c.attitude {
        horizon(d, centre, radius, pitch, roll);
    }
    let _ = Circle::with_center(centre, (2 * radius + 1) as u32)
        .into_styled(PrimitiveStyle::with_stroke(BORDER, 1))
        .draw(d);
    let (a, b) = c.attitude.lines();
    text(
        d,
        &a,
        Point::new(cx, r.y0 + 44),
        &FONT_6X10,
        FG_MUTED,
        Alignment::Center,
    );
    text(
        d,
        &b,
        Point::new(cx, r.y0 + 55),
        &FONT_6X10,
        FG_MUTED,
        Alignment::Center,
    );
}

/// The mini horizon (3.1): `Neon` sky above the horizon, `BgElevated` ground below, the horizon
/// rotated by minus the roll and moved down 1.4 px a degree of pitch, clamped to the radius; the
/// fixed board symbol in `Fg`.
fn horizon<D: DrawTarget<Color = Rgb565>>(d: &mut D, c: Point, radius: i32, pitch: i16, roll: i16) {
    let roll = -(roll as f32) / 100.0 * core::f32::consts::PI / 180.0;
    let shift = (pitch as f32 / 100.0 * 1.4).clamp(-(radius as f32), radius as f32);
    let (s, co) = (libm::sinf(roll), libm::cosf(roll));
    let pixels = (-radius..=radius).flat_map(move |dy| {
        (-radius..=radius).filter_map(move |dx| {
            if dx * dx + dy * dy > radius * radius {
                return None;
            }
            // Signed distance below the rotated, shifted horizon line.
            let below = -(dx as f32) * s + (dy as f32) * co - shift;
            let col = if below.abs() < 0.6 {
                FG_MUTED
            } else if below < 0.0 {
                NEON
            } else {
                BG_ELEVATED
            };
            Some(Pixel(c + Point::new(dx, dy), col))
        })
    });
    let _ = d.draw_iter(pixels);
    let st = PrimitiveStyle::with_stroke(FG, 2);
    let a = radius * 15 / 100;
    let b = radius * 55 / 100;
    let _ = Line::new(c + Point::new(-b, 0), c + Point::new(-a, 0))
        .into_styled(st)
        .draw(d);
    let _ = Line::new(c + Point::new(a, 0), c + Point::new(b, 0))
        .into_styled(st)
        .draw(d);
    let _ = Circle::with_center(c, 3)
        .into_styled(PrimitiveStyle::with_fill(FG))
        .draw(d);
}

fn draw_pack<D: DrawTarget<Color = Rgb565>>(d: &mut D, p: &Pack) {
    let r = Widget::PackLine.rect();
    fill(d, r, BG);
    let col = if p.caution {
        CAUTION
    } else if p.muted {
        FG_MUTED
    } else {
        FG
    };
    text(
        d,
        &p.text,
        Point::new(120, r.y0 + 2),
        &FONT_8X13,
        col,
        Alignment::Center,
    );
}

fn draw_phase<D: DrawTarget<Color = Rgb565>>(d: &mut D, r: Rect, b: Option<&PhaseBar>) {
    fill(d, r, BG);
    let Some(b) = b else {
        return;
    };
    text(
        d,
        &b.label,
        Point::new(r.x0, r.y0 + 4),
        &FONT_6X10,
        FG_MUTED,
        Alignment::Left,
    );
    text(
        d,
        &b.text,
        Point::new(r.x1, r.y0 + 2),
        &FONT_8X13,
        FG,
        Alignment::Right,
    );
    let track = Rect::new(r.x0, r.y0 + 20, r.x1, r.y0 + 24);
    fill(d, track, BG_CODE);
    if let Some(f) = b.fill {
        let w = (track.width() * f as i32) / 1000;
        if w > 0 {
            fill(
                d,
                Rect::new(track.x0, track.y0, track.x0 + w, track.y1),
                PRIMARY,
            );
        }
    }
}

/// One look of the arm band: its fill, outline and outline width, the label and the line under it
/// (empty for none), and their colours.
struct BandLook {
    fill: Rgb565,
    stroke: Rgb565,
    width: u32,
    label: &'static str,
    sub: &'static str,
    lc: Rgb565,
    sc: Rgb565,
}

/// The hold's two looks: the progress fill, and the rest of the band (the can-arm look).
const HOLD_FILLED: BandLook = BandLook {
    fill: PRIMARY,
    stroke: PRIMARY,
    width: 2,
    label: "ARM",
    sub: "hold 1 s",
    lc: BG,
    sc: BG,
};
const HOLD_EMPTY: BandLook = BandLook {
    fill: BG,
    stroke: PRIMARY,
    width: 2,
    label: "ARM",
    sub: "hold 1 s",
    lc: PRIMARY,
    sc: FG_MUTED,
};

fn draw_look<D: DrawTarget<Color = Rgb565>>(d: &mut D, k: &BandLook) {
    let r = Widget::ArmBand.rect();
    let _ = RoundedRectangle::with_equal_corners(er(r), Size::new(8, 8))
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(k.fill)
                .stroke_color(k.stroke)
                .stroke_width(k.width)
                .build(),
        )
        .draw(d);
    let on = |font, fg| {
        MonoTextStyleBuilder::new()
            .font(font)
            .text_color(fg)
            .background_color(k.fill)
            .build()
    };
    let ts = TextStyleBuilder::new()
        .alignment(Alignment::Center)
        .baseline(Baseline::Top)
        .build();
    let y = if k.sub.is_empty() { r.y0 + 8 } else { r.y0 + 3 };
    let _ = Text::with_text_style(k.label, Point::new(120, y), on(&FONT_10X20, k.lc), ts).draw(d);
    if !k.sub.is_empty() {
        let _ = Text::with_text_style(k.sub, Point::new(120, r.y0 + 23), on(&FONT_6X10, k.sc), ts)
            .draw(d);
    }
}

/// `look` drawn over the band's columns `x0..x1` only.
fn draw_look_cols<D: DrawTarget<Color = Rgb565>>(d: &mut D, look: &BandLook, x0: i32, x1: i32) {
    if x1 <= x0 {
        return;
    }
    let r = Widget::ArmBand.rect();
    let cols = er(Rect::new(x0, r.y0, x1, r.y1));
    draw_look(&mut d.clipped(&cols), look);
}

/// The progress fill's right edge for a hold of `ms`.
fn fill_edge(ms: u16) -> i32 {
    let r = Widget::ArmBand.rect();
    r.x0 + r.width() * i32::from(ms) / handheld_core::arm::ARM_HOLD_MS as i32
}

/// How far the progress fill reaches in `b`, for a band that shows the hold's looks.
fn hold_ms(b: ArmBand) -> Option<u16> {
    match b {
        ArmBand::CanArm => Some(0),
        ArmBand::Holding(ms) => Some(ms),
        _ => None,
    }
}

/// The band as section 5.3 draws it.
fn draw_band<D: DrawTarget<Color = Rgb565>>(d: &mut D, b: ArmBand) {
    let r = Widget::ArmBand.rect();
    match b {
        ArmBand::Armed => draw_look(
            d,
            &BandLook {
                fill: PRIMARY,
                stroke: PRIMARY,
                width: 2,
                label: "ARMED",
                sub: "tap to disarm",
                lc: BG,
                sc: BG,
            },
        ),
        ArmBand::CanArm | ArmBand::Holding(_) => {
            let edge = fill_edge(hold_ms(b).unwrap_or(0));
            draw_look_cols(d, &HOLD_FILLED, r.x0, edge);
            draw_look_cols(d, &HOLD_EMPTY, edge, r.x1);
        }
        ArmBand::Release => draw_look(
            d,
            &BandLook {
                fill: PRIMARY,
                stroke: PRIMARY,
                width: 2,
                label: "RELEASE TO ARM",
                sub: "",
                lc: BG,
                sc: BG,
            },
        ),
        ArmBand::CannotArm => draw_look(
            d,
            &BandLook {
                fill: BG,
                stroke: BORDER,
                width: 1,
                label: "ARM",
                sub: "",
                lc: FG_FAINT,
                sc: FG_FAINT,
            },
        ),
    }
}

/// Go from the band as drawn, `prev`, to `next`: a hold's fill that grew draws only the columns it
/// gained (section 5.3); anything else redraws the band.
fn update_band<D: DrawTarget<Color = Rgb565>>(d: &mut D, prev: Option<ArmBand>, next: ArmBand) {
    match (prev.and_then(hold_ms), hold_ms(next)) {
        (Some(a), Some(b)) if b >= a => {
            draw_look_cols(d, &HOLD_FILLED, fill_edge(a), fill_edge(b));
        }
        _ => draw_band(d, next),
    }
}

fn draw_grid<D: DrawTarget<Color = Rgb565>>(d: &mut D, cells: &[LampCell]) {
    for (i, cell) in cells.iter().enumerate() {
        let (row, col) = handheld_core::lamps::grid_cell(i);
        let r = layout::lamp_cell(col as i32, row as i32);
        let (fillc, fg) = if cell.lit {
            (WARNING, BG)
        } else {
            (BG_CODE, FG_FAINT)
        };
        let _ = RoundedRectangle::with_equal_corners(er(r), Size::new(4, 4))
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .fill_color(fillc)
                    .stroke_color(BORDER)
                    .stroke_width(1)
                    .build(),
            )
            .draw(d);
        let st = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(fg)
            .background_color(fillc)
            .build();
        let ts = TextStyleBuilder::new()
            .alignment(Alignment::Center)
            .baseline(Baseline::Top)
            .build();
        let _ = Text::with_text_style(
            cell.lamp.label(),
            Point::new(r.x0 + r.width() / 2, r.y0 + 4),
            st,
            ts,
        )
        .draw(d);
        for (k, m) in cell.marks.iter().enumerate() {
            if let Some((label, on)) = m {
                let mst = MonoTextStyleBuilder::new()
                    .font(&FONT_6X10)
                    .text_color(if *on { fg } else { FG_FAINT })
                    .background_color(fillc)
                    .build();
                let x = r.x0 + 4 + k as i32 * (r.width() / 2);
                let _ = Text::with_text_style(
                    label,
                    Point::new(x, r.y0 + 18),
                    mst,
                    TextStyleBuilder::new().baseline(Baseline::Top).build(),
                )
                .draw(d);
            }
        }
    }
}

fn draw_line<D: DrawTarget<Color = Rgb565>>(
    d: &mut D,
    s: Option<&Statement>,
    debug: Option<(&str, &str)>,
) {
    let r = Widget::Statement.rect();
    fill(d, r, BG);
    match (s, debug) {
        (Some(s), _) => {
            let col = match s.tone {
                Tone::Fg => FG,
                Tone::Muted => FG_MUTED,
                Tone::Caution => CAUTION,
                Tone::Warning => WARNING,
            };
            let mut x = r.x0;
            if matches!(s.tone, Tone::Caution | Tone::Warning) {
                caution_glyph(d, Point::new(x, r.y0 + 6), 10, col);
                x += 14;
            }
            // One line, ellipsised at the width.
            let max = ((r.x1 - x) / 8) as usize;
            let mut t: String<64> = String::new();
            if s.text.chars().count() > max {
                for ch in s.text.chars().take(max.saturating_sub(1)) {
                    let _ = t.push(ch);
                }
                let _ = t.push('\u{2026}');
            } else {
                let _ = t.push_str(&s.text);
            }
            text(
                d,
                &t,
                Point::new(x, r.y0 + 5),
                &FONT_8X13,
                col,
                Alignment::Left,
            );
        }
        (None, Some((a, b))) => {
            text(
                d,
                a,
                Point::new(r.x0, r.y0 + 1),
                &FONT_6X10,
                FG_FAINT,
                Alignment::Left,
            );
            text(
                d,
                b,
                Point::new(r.x0, r.y0 + 11),
                &FONT_6X10,
                FG_FAINT,
                Alignment::Left,
            );
        }
        (None, None) => {}
    }
}

/// The rows the overlay covers: the zero line, and the thumb with its percent above it.
pub fn overlay_bands(o: &Overlay) -> [(i32, i32); 2] {
    let zero = layout::band(o.y0, layout::ZERO_H);
    let (t0, t1) = layout::band(o.yf, layout::THUMB_H);
    [zero, ((t0 - 15).max(0), t1)]
}

/// Redraw every widget that meets rows `y0..y1`, clipped to them: the overlay left those rows
/// (section 5.7).
pub fn redraw_band<D: DrawTarget<Color = Rgb565>>(
    d: &mut D,
    v: &View,
    debug: Option<(&str, &str)>,
    y0: i32,
    y1: i32,
) {
    let band = er(Rect::new(0, y0, layout::WIDTH, y1));
    let mut c = d.clipped(&band);
    fill(&mut c, Rect::new(0, y0, layout::WIDTH, y1), BG);
    for w in layout::meeting(y0, y1) {
        match (w, &v.body) {
            (Widget::TopStrip, _) => draw_top(&mut c, &v.top),
            (Widget::Statement, _) => draw_line(&mut c, v.line.as_ref(), debug),
            (_, Body::Card(_)) => {}
            (Widget::Dial, Body::Ride(r)) => draw_dial(&mut c, &r.dial),
            (Widget::AttitudeLeft, Body::Ride(r)) => {
                draw_corner(&mut c, w.rect(), r.corners[0].as_ref())
            }
            (Widget::AttitudeRight, Body::Ride(r)) => {
                draw_corner(&mut c, w.rect(), r.corners[1].as_ref())
            }
            (Widget::PackLine, Body::Ride(r)) => draw_pack(&mut c, &r.pack),
            (Widget::PhaseLeft | Widget::PhaseRight | Widget::ArmBand, Body::Ride(r)) => {
                match &r.lower {
                    Lower::Bars { phase, band } => match w {
                        Widget::PhaseLeft => draw_phase(&mut c, w.rect(), phase[0].as_ref()),
                        Widget::PhaseRight => draw_phase(&mut c, w.rect(), phase[1].as_ref()),
                        _ => draw_band(&mut c, *band),
                    },
                    Lower::Grid(cells) => draw_grid(&mut c, cells),
                }
            }
        }
    }
    if let Body::Card(card) = &v.body {
        if layout::CARD_AREA.meets_rows(y0, y1) {
            draw_card(&mut c, card);
        }
    }
}

/// The overlay, drawn last: the zero line at `y0`, the thumb bar at `yf` and the demand as a
/// signed percent beside its right end.
pub fn draw_overlay<D: DrawTarget<Color = Rgb565>>(d: &mut D, o: &Overlay) {
    let (z0, z1) = layout::band(o.y0, layout::ZERO_H);
    fill(d, Rect::new(0, z0, layout::WIDTH, z1), FG_MUTED);
    let (t0, t1) = layout::band(o.yf, layout::THUMB_H);
    let col = if o.live { PRIMARY } else { FG_MUTED };
    let _ = RoundedRectangle::with_equal_corners(
        er(Rect::new(2, t0, layout::WIDTH - 2, t1)),
        Size::new(5, 5),
    )
    .into_styled(PrimitiveStyle::with_fill(col))
    .draw(d);
    let mut s: String<8> = String::new();
    let _ = write!(s, "{:+}%", o.percent);
    text(
        d,
        &s,
        Point::new(layout::WIDTH - 4, (t0 - 14).max(0)),
        &FONT_8X13,
        FG,
        Alignment::Right,
    );
}
