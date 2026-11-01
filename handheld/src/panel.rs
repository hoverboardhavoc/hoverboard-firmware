//! Drawing the Ride view on the ILI9341 (`specs/handheld-remote.md` section 5.1): no framebuffer.
//! Each widget keeps the state it last drew; a frame finds the rectangles whose content changed
//! and paints each one through a RAM strip: the strip is filled with `Bg`, every element that
//! meets it is drawn into it (the widgets, the armed frame, the throttle overlay), and the strip
//! goes to the panel as one address window. So a changed widget is overwritten once with its new
//! pixels, never cleared first, and the panel's address window is set once per strip, not per
//! pixel or per glyph.

use core::fmt::Write;

use embedded_graphics::mono_font::iso_8859_1::{FONT_10X20, FONT_6X10, FONT_8X13};
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{
    Arc, Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle,
};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use heapless::{String, Vec};
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};
use u8g2_fonts::{fonts, FontRenderer};

use handheld_core::layout::{self, Drawn, Rect, Widget};
use handheld_core::model::{
    ArmBand, Body, Card, ConnectView, Corner, Dial, Dot, LampCell, Lower, Overlay, Pack, PhaseBar,
    Ride, RowShow, SpeedShow, Statement, Tone, Top, View, CANCEL,
};
use handheld_core::pick::addr_text;
use handheld_core::readings::attitude::Attitude;
use handheld_core::scan::{ROWS, ROWS_Y, ROW_H};

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

/// Rows of the full panel width the strip holds: a full redraw is eight strips.
pub const STRIP_ROWS: usize = 40;
/// The strip's size in pixels; the binary allocates it once.
pub const STRIP_LEN: usize = layout::WIDTH as usize * STRIP_ROWS;

fn er(r: Rect) -> Rectangle {
    Rectangle::new(
        Point::new(r.x0, r.y0),
        Size::new(r.width() as u32, r.height() as u32),
    )
}

/// A rectangle of the panel held in RAM: whatever is drawn into it outside its area is dropped.
pub struct Strip<'a> {
    buf: &'a mut [Rgb565],
    area: Rect,
}

impl Strip<'_> {
    fn index(&self, p: Point) -> Option<usize> {
        let a = &self.area;
        (p.x >= a.x0 && p.x < a.x1 && p.y >= a.y0 && p.y < a.y1)
            .then(|| ((p.y - a.y0) * a.width() + (p.x - a.x0)) as usize)
    }

    /// Whether `r` shares a pixel with the strip.
    fn meets(&self, r: Rect) -> bool {
        self.area.overlaps(&r)
    }
}

impl Dimensions for Strip<'_> {
    fn bounding_box(&self) -> Rectangle {
        er(self.area)
    }
}

impl DrawTarget for Strip<'_> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I: IntoIterator<Item = Pixel<Rgb565>>>(
        &mut self,
        pixels: I,
    ) -> Result<(), Self::Error> {
        for Pixel(p, c) in pixels {
            if let Some(i) = self.index(p) {
                self.buf[i] = c;
            }
        }
        Ok(())
    }

    fn fill_contiguous<I: IntoIterator<Item = Rgb565>>(
        &mut self,
        area: &Rectangle,
        colors: I,
    ) -> Result<(), Self::Error> {
        if area.intersection(&er(self.area)).is_zero_sized() {
            return Ok(());
        }
        for (p, c) in area.points().zip(colors) {
            if let Some(i) = self.index(p) {
                self.buf[i] = c;
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Rgb565) -> Result<(), Self::Error> {
        let i = area.intersection(&er(self.area));
        if i.is_zero_sized() {
            return Ok(());
        }
        let w = self.area.width() as usize;
        let x0 = (i.top_left.x - self.area.x0) as usize;
        let n = i.size.width as usize;
        for y in i.top_left.y..i.top_left.y + i.size.height as i32 {
            let row = (y - self.area.y0) as usize * w;
            self.buf[row + x0..row + x0 + n].fill(color);
        }
        Ok(())
    }
}

/// Paint `rect` on the panel: strip by strip, each filled with `Bg`, drawn by `scene`, and sent as
/// one address window.
fn paint<D: DrawTarget<Color = Rgb565>>(
    panel: &mut D,
    buf: &mut [Rgb565],
    rect: Rect,
    scene: &impl Fn(&mut Strip),
) {
    let Some(rect) = rect.on_panel_part() else {
        return;
    };
    let w = rect.width() as usize;
    let rows = (buf.len() / w).max(1) as i32;
    let mut y = rect.y0;
    while y < rect.y1 {
        let band = Rect::new(rect.x0, y, rect.x1, (y + rows).min(rect.y1));
        let n = w * band.height() as usize;
        let mut s = Strip {
            buf: &mut buf[..n],
            area: band,
        };
        s.buf.fill(BG);
        scene(&mut s);
        let _ = panel.fill_contiguous(&er(band), s.buf.iter().copied());
        y = band.y1;
    }
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
    let _ = d.fill_solid(&er(r), c);
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

/// One frame to show: the view, the debug line's rows (drawn where the statement line is empty),
/// and the throttle overlay.
pub struct Frame<'a> {
    pub view: &'a View,
    pub debug: Option<(&'a str, &'a str)>,
    pub overlay: Option<Overlay>,
}

/// What the panel shows, as last painted.
#[derive(Default)]
pub struct Cache {
    /// Which screen: 0 a card, 1 the Ride view, 2 the Connect screen. `None` before the first
    /// frame.
    screen: Drawn<u8>,
    /// The Connect screen's rows: each one's content but the RSSI, and its RSSI.
    rows: [Drawn<u64>; ROWS],
    rssi: [Drawn<i8>; ROWS],
    top: Drawn<u64>,
    card: Drawn<u64>,
    ring: Drawn<(u8, bool)>,
    speed: Drawn<u64>,
    corners: [Drawn<u64>; 2],
    pack: Drawn<u64>,
    /// 0 the phase bars and the band, 1 the lamp grid.
    lower: Drawn<u8>,
    phase: [Drawn<u64>; 2],
    band: Option<ArmBand>,
    grid: Drawn<u64>,
    /// The statement line as drawn: its text and tone, or the debug rows' hash.
    line: Option<LineDrawn>,
    armed: Drawn<bool>,
    overlay: Option<Overlay>,
}

#[derive(Clone, PartialEq)]
enum LineDrawn {
    Statement(String<64>, Tone),
    Other(u64),
}

/// The rectangles a frame paints.
type Dirty = Vec<Rect, 24>;

fn dirty(d: &mut Dirty, r: Rect) {
    if d.push(r).is_err() {
        // More changes than slots: paint the whole panel in the last one.
        let last = d.len() - 1;
        d[last] = Rect::new(0, 0, layout::WIDTH, layout::HEIGHT);
    }
}

/// What one frame cost.
pub struct Painted {
    /// The whole panel was painted (a screen change).
    pub full: bool,
    /// Rectangles painted.
    pub rects: usize,
}

/// Paint what changed since the last frame (section 5.1). `buf` is the strip's memory, at least
/// one panel row.
pub fn frame<D: DrawTarget<Color = Rgb565>>(
    panel: &mut D,
    buf: &mut [Rgb565],
    f: &Frame,
    c: &mut Cache,
) -> Painted {
    let v = f.view;
    let mut d = Dirty::new();
    let screen = match v.body {
        Body::Card(_) => 0,
        Body::Ride(_) => 1,
        Body::Connect(_) => 2,
    };
    let full = c.screen.update(screen);
    if full {
        *c = Cache {
            screen: c.screen,
            ..Cache::default()
        };
    }
    if c.top.update(hash(&v.top)) {
        dirty(&mut d, Widget::TopStrip.rect());
    }
    match &v.body {
        Body::Card(card) => {
            if c.card.update(hash(card)) {
                dirty(&mut d, layout::CARD_AREA);
            }
        }
        Body::Ride(r) => ride_dirty(&mut d, r, c),
        Body::Connect(cv) => connect_dirty(&mut d, cv, c),
    }
    line_dirty(&mut d, v, f.debug, c);
    if c.armed.update(v.armed) {
        for r in FRAME_EDGES {
            dirty(&mut d, r);
        }
    }
    if c.overlay != f.overlay {
        overlay_dirty(&mut d, c.overlay.as_ref(), f.overlay.as_ref());
        c.overlay = f.overlay;
    }
    let scene = |s: &mut Strip| draw_scene(s, f);
    if full {
        paint(
            panel,
            buf,
            Rect::new(0, 0, layout::WIDTH, layout::HEIGHT),
            &scene,
        );
    } else {
        for r in d.iter() {
            paint(panel, buf, *r, &scene);
        }
    }
    Painted {
        full,
        rects: if full { 1 } else { d.len() },
    }
}

/// Row `i` of the Connect screen, and the part its RSSI takes.
fn row_rect(i: usize) -> Rect {
    let y = ROWS_Y + ROW_H * i as i32;
    Rect::new(0, y, layout::WIDTH, y + ROW_H)
}

fn rssi_rect(i: usize) -> Rect {
    let r = row_rect(i);
    Rect::new(176, r.y0 + 16, r.x1, r.y0 + 28)
}

/// Only a row whose content changed is painted, its RSSI alone when only that changed (section
/// 5.8).
fn connect_dirty(d: &mut Dirty, cv: &ConnectView, c: &mut Cache) {
    for (i, row) in cv.rows.iter().enumerate() {
        let content = hash(&row.as_ref().map(|r| (&r.name, r.addr, r.picked)));
        let rssi = row.as_ref().map_or(i8::MIN, |r| r.rssi);
        if c.rows[i].update(content) {
            dirty(d, row_rect(i));
            c.rssi[i].update(rssi);
        } else if c.rssi[i].update(rssi) {
            dirty(d, rssi_rect(i));
        }
    }
}

fn ride_dirty(d: &mut Dirty, r: &Ride, c: &mut Cache) {
    if c.ring.update((r.dial.lit, r.dial.caution)) {
        dirty(d, Widget::Dial.rect());
        c.speed.update(hash(&r.dial.speed));
    } else if c.speed.update(hash(&r.dial.speed)) {
        dirty(d, layout::SPEED_BOX);
    }
    for (i, w) in [Widget::AttitudeLeft, Widget::AttitudeRight]
        .iter()
        .enumerate()
    {
        if c.corners[i].update(hash(&r.corners[i])) {
            dirty(d, w.rect());
        }
    }
    if c.pack.update(hash(&r.pack)) {
        dirty(d, Widget::PackLine.rect());
    }
    match &r.lower {
        Lower::Bars { phase, band } => {
            if c.lower.update(0) {
                dirty(d, layout::LAMP_GRID);
                c.phase = Default::default();
                c.phase[0].update(hash(&phase[0]));
                c.phase[1].update(hash(&phase[1]));
                c.band = Some(*band);
                c.grid.invalidate();
                return;
            }
            for (i, w) in [Widget::PhaseLeft, Widget::PhaseRight].iter().enumerate() {
                if c.phase[i].update(hash(&phase[i])) {
                    dirty(d, w.rect());
                }
            }
            if c.band != Some(*band) {
                dirty(d, band_change(c.band, *band));
                c.band = Some(*band);
            }
        }
        Lower::Grid(cells) => {
            let first = c.lower.update(1);
            if c.grid.update(hash(cells)) || first {
                dirty(d, layout::LAMP_GRID);
            }
            c.band = None;
        }
    }
}

/// The statement line's change: the columns of the characters that changed when a statement's
/// text changes in place (the read counter), the whole line otherwise.
fn line_dirty(d: &mut Dirty, v: &View, debug: Option<(&str, &str)>, c: &mut Cache) {
    let next = match &v.line {
        Some(s) => LineDrawn::Statement(ellipsised(s), s.tone),
        None => LineDrawn::Other(hash(&debug)),
    };
    if c.line.as_ref() == Some(&next) {
        return;
    }
    let r = Widget::Statement.rect();
    let cols = match (&c.line, &next) {
        (Some(LineDrawn::Statement(a, ta)), LineDrawn::Statement(b, tb)) if ta == tb => {
            layout::text_diff(a, b).map(|(i, j)| {
                let x = line_text_x(*tb);
                Rect::new(x + 8 * i as i32, r.y0, (x + 8 * j as i32).min(r.x1), r.y1)
            })
        }
        _ => Some(r),
    };
    if let Some(cols) = cols {
        dirty(d, cols);
    }
    c.line = Some(next);
}

/// The armed frame's four edges (section 5.3).
const FRAME_EDGES: [Rect; 4] = [
    Rect::new(0, 0, 240, 2),
    Rect::new(0, 318, 240, 320),
    Rect::new(0, 0, 2, 320),
    Rect::new(238, 0, 240, 320),
];

/// The rectangles an overlay covers: its zero line, its thumb bar with the figures, and in
/// DIFFERENTIAL the turn zero line.
fn overlay_rects(o: &Overlay) -> [Option<Rect>; 3] {
    [
        Some(layout::zero_rect(o.y0)),
        Some(layout::thumb_rect(o.yf)),
        o.turn.map(|t| layout::turn_rect(t.x0)),
    ]
}

/// The overlay moved from `old` to `new`: paint what either covers that changed. A thumb that
/// moved a little is one rectangle over both places.
fn overlay_dirty(d: &mut Dirty, old: Option<&Overlay>, new: Option<&Overlay>) {
    let a = old.map(overlay_rects).unwrap_or_default();
    let b = new.map(overlay_rects).unwrap_or_default();
    // The thumb's content (its colour, the figures, the mark) is in the thumb rectangle, so any
    // change of the overlay repaints it in both places.
    for i in 0..3 {
        match (a[i], b[i]) {
            (Some(x), Some(y)) if x == y && i != 1 => {}
            (Some(x), Some(y)) if x.overlaps(&y) || x.y1 == y.y0 || y.y1 == x.y0 => {
                dirty(d, x.union(&y))
            }
            (x, y) => {
                for r in [x, y].into_iter().flatten() {
                    dirty(d, r);
                }
            }
        }
    }
}

/// The band's change from `prev` to `next`: a hold's fill that grew paints only the columns it
/// gained (section 5.3); anything else paints the band.
fn band_change(prev: Option<ArmBand>, next: ArmBand) -> Rect {
    let r = Widget::ArmBand.rect();
    match (prev.and_then(hold_ms), hold_ms(next)) {
        (Some(a), Some(b)) if b >= a => Rect::new(fill_edge(a), r.y0, fill_edge(b), r.y1),
        _ => r,
    }
}

/// Draw everything that meets the strip, back to front: the widgets that meet it
/// (`layout::meeting`), or the card in their place, then the armed frame, then the overlay.
fn draw_scene(s: &mut Strip, f: &Frame) {
    let v = f.view;
    let ride = match &v.body {
        Body::Ride(r) => Some(r),
        Body::Card(card) => {
            if s.meets(layout::CARD) {
                draw_card(s, card);
            }
            None
        }
        Body::Connect(cv) => {
            draw_connect(s, cv);
            if s.meets(Widget::Statement.rect()) {
                draw_line(s, v.line.as_ref(), None);
            }
            return;
        }
    };
    let mut grid_drawn = false;
    for w in layout::meeting(s.area) {
        let Some(r) = ride else {
            match w {
                Widget::TopStrip => draw_top(s, &v.top),
                Widget::Statement => draw_line(s, v.line.as_ref(), f.debug),
                _ => {}
            }
            continue;
        };
        match (w, &r.lower) {
            (Widget::TopStrip, _) => draw_top(s, &v.top),
            (Widget::Statement, _) => draw_line(s, v.line.as_ref(), f.debug),
            (Widget::Dial, _) => draw_dial(s, &r.dial),
            (Widget::AttitudeLeft, _) => draw_corner(s, w.rect(), r.corners[0].as_ref()),
            (Widget::AttitudeRight, _) => draw_corner(s, w.rect(), r.corners[1].as_ref()),
            (Widget::PackLine, _) => draw_pack(s, &r.pack),
            (Widget::PhaseLeft, Lower::Bars { phase, .. }) => {
                draw_phase(s, w.rect(), phase[0].as_ref())
            }
            (Widget::PhaseRight, Lower::Bars { phase, .. }) => {
                draw_phase(s, w.rect(), phase[1].as_ref())
            }
            (Widget::ArmBand, Lower::Bars { band, .. }) => draw_band(s, *band),
            // The lamp grid takes the phase bars' and the band's place, drawn once.
            (Widget::PhaseLeft | Widget::PhaseRight | Widget::ArmBand, Lower::Grid(cells)) => {
                if !grid_drawn {
                    grid_drawn = true;
                    draw_grid(s, cells);
                }
            }
        }
    }
    if v.armed {
        for r in FRAME_EDGES {
            fill(s, r, PRIMARY);
        }
    }
    if let Some(o) = &f.overlay {
        draw_overlay(s, o);
    }
}

/// The Connect screen (section 5.8): the title, the rows that meet the strip.
fn draw_connect(s: &mut Strip, cv: &ConnectView) {
    if s.meets(Widget::TopStrip.rect()) {
        text(
            s,
            "Connect",
            Point::new(4, 4),
            &FONT_8X13,
            FG,
            Alignment::Left,
        );
        text(
            s,
            "scanning",
            Point::new(236, 6),
            &FONT_6X10,
            FG_MUTED,
            Alignment::Right,
        );
    }
    for (i, row) in cv.rows.iter().enumerate() {
        let r = row_rect(i);
        if !s.meets(r) {
            continue;
        }
        if let Some(row) = row {
            draw_row(s, r, row);
        }
        fill(s, Rect::new(r.x0, r.y1 - 1, r.x1, r.y1), BORDER);
    }
}

fn draw_row<D: DrawTarget<Color = Rgb565>>(d: &mut D, r: Rect, row: &RowShow) {
    if row.picked {
        fill(d, Rect::new(r.x0, r.y0, r.x0 + 3, r.y1 - 1), PRIMARY);
    }
    match &row.name {
        Some(n) => text(
            d,
            n,
            Point::new(8, r.y0 + 3),
            &FONT_8X13,
            FG,
            Alignment::Left,
        ),
        None => text(
            d,
            "(no name)",
            Point::new(8, r.y0 + 3),
            &FONT_8X13,
            FG_MUTED,
            Alignment::Left,
        ),
    }
    text(
        d,
        &addr_text(&row.addr),
        Point::new(8, r.y0 + 18),
        &FONT_6X10,
        FG_MUTED,
        Alignment::Left,
    );
    let mut rssi: String<12> = String::new();
    let _ = write!(rssi, "{} dBm", row.rssi);
    text(
        d,
        &rssi,
        Point::new(236, r.y0 + 18),
        &FONT_6X10,
        FG_MUTED,
        Alignment::Right,
    );
}

fn draw_top<D: DrawTarget<Color = Rgb565>>(d: &mut D, t: &Top) {
    let mut x = 6;
    for dot in t.dots.iter().flatten() {
        draw_dot(d, dot, &mut x);
    }
    if t.test {
        text(
            d,
            "TEST",
            Point::new(120, 4),
            &FONT_8X13,
            FG_MUTED,
            Alignment::Center,
        );
    } else if t.low {
        caution_glyph(d, Point::new(100, 5), 10, CAUTION);
        text(
            d,
            "LOW",
            Point::new(114, 4),
            &FONT_8X13,
            CAUTION,
            Alignment::Left,
        );
    }
    // The disconnect control (section 5.8), while connected and disarmed.
    match t.disconnect {
        Some(false) => text(
            d,
            "DISCONNECT",
            Point::new(236, 6),
            &FONT_6X10,
            FG_MUTED,
            Alignment::Right,
        ),
        Some(true) => text(
            d,
            "RELEASE",
            Point::new(236, 6),
            &FONT_6X10,
            PRIMARY,
            Alignment::Right,
        ),
        None => {}
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

/// Centred text on the card's `BgCode`.
fn card_text<D: DrawTarget<Color = Rgb565>>(
    d: &mut D,
    s: &str,
    y: i32,
    font: &'static MonoFont<'static>,
    fg: Rgb565,
) {
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
}

fn draw_card<D: DrawTarget<Color = Rgb565>>(d: &mut D, card: &Card) {
    let _ = RoundedRectangle::with_equal_corners(er(layout::CARD), Size::new(12, 12))
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(BG_CODE)
                .stroke_color(BORDER)
                .stroke_width(1)
                .build(),
        )
        .draw(d);
    let mid = 130;
    match card {
        Card::Connecting { label, addr } => {
            card_text(d, "Connecting to", mid - 18, &FONT_8X13, FG);
            // At most 27 characters fit the card at 8 px.
            let mut l: String<32> = String::new();
            if label.chars().count() > 27 {
                for ch in label.chars().take(26) {
                    let _ = l.push(ch);
                }
                let _ = l.push('\u{2026}');
            } else {
                let _ = l.push_str(label);
            }
            card_text(d, &l, mid, &FONT_8X13, FG);
            card_text(d, &addr_text(addr), mid + 18, &FONT_8X13, FG_MUTED);
            let _ = RoundedRectangle::with_equal_corners(er(CANCEL), Size::new(8, 8))
                .into_styled(
                    PrimitiveStyleBuilder::new()
                        .fill_color(BG_CODE)
                        .stroke_color(BORDER)
                        .stroke_width(1)
                        .build(),
                )
                .draw(d);
            card_text(d, "Cancel", CANCEL.y0 + 12, &FONT_8X13, FG);
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

/// The battery ring, pixel by pixel over the part of its box in the strip (the segment geometry is
/// `layout::ring_segment`), then the speed.
fn draw_dial(s: &mut Strip, dial: &Dial) {
    let lit = if dial.caution { CAUTION } else { PRIMARY };
    let (cx, cy) = layout::RING_CENTRE;
    let outer = layout::RING_RADIUS + layout::RING_STROKE / 2;
    let ring = Rect::new(cx - outer, cy - outer, cx + outer + 1, cy + outer + 1);
    let a = s.area;
    let (x0, x1) = (ring.x0.max(a.x0), ring.x1.min(a.x1));
    let (y0, y1) = (ring.y0.max(a.y0), ring.y1.min(a.y1));
    let w = a.width();
    for y in y0..y1 {
        for x in x0..x1 {
            if let Some(seg) = layout::ring_segment(x - cx, y - cy) {
                let col = if seg < dial.lit { lit } else { BG_CODE };
                s.buf[((y - a.y0) * w + (x - a.x0)) as usize] = col;
            }
        }
    }
    if s.meets(layout::SPEED_BOX) {
        draw_speed(s, &dial.speed);
    }
}

fn draw_speed<D: DrawTarget<Color = Rgb565>>(d: &mut D, speed: &SpeedShow) {
    let (sx, sy) = layout::SPEED_CENTRE;
    let big = FontRenderer::new::<fonts::u8g2_font_logisoso38_tn>();
    let (figure, faint) = match speed {
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
    match speed {
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
    let w = b.fill.map_or(0, |f| (track.width() * f as i32) / 1000);
    fill(
        d,
        Rect::new(track.x0, track.y0, track.x0 + w, track.y1),
        PRIMARY,
    );
    fill(
        d,
        Rect::new(track.x0 + w, track.y0, track.x1, track.y1),
        BG_CODE,
    );
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

/// Where the statement's text starts: after the glyph for a caution or warning line.
fn line_text_x(tone: Tone) -> i32 {
    let x = Widget::Statement.rect().x0;
    if matches!(tone, Tone::Caution | Tone::Warning) {
        x + 14
    } else {
        x
    }
}

/// The statement as drawn: one line, ellipsised at the width.
fn ellipsised(s: &Statement) -> String<64> {
    let r = Widget::Statement.rect();
    let max = ((r.x1 - line_text_x(s.tone)) / 8) as usize;
    let mut t: String<64> = String::new();
    if s.text.chars().count() > max {
        for ch in s.text.chars().take(max.saturating_sub(1)) {
            let _ = t.push(ch);
        }
        let _ = t.push('\u{2026}');
    } else {
        let _ = t.push_str(&s.text);
    }
    t
}

fn draw_line<D: DrawTarget<Color = Rgb565>>(
    d: &mut D,
    s: Option<&Statement>,
    debug: Option<(&str, &str)>,
) {
    let r = Widget::Statement.rect();
    match (s, debug) {
        (Some(s), _) => {
            let col = match s.tone {
                Tone::Fg => FG,
                Tone::Muted => FG_MUTED,
                Tone::Caution => CAUTION,
                Tone::Warning => WARNING,
            };
            if matches!(s.tone, Tone::Caution | Tone::Warning) {
                caution_glyph(d, Point::new(r.x0, r.y0 + 6), 10, col);
            }
            text(
                d,
                &ellipsised(s),
                Point::new(line_text_x(s.tone), r.y0 + 5),
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

/// The overlay (section 5.7), over everything: the zero line at `y0`; in DIFFERENTIAL the turn
/// zero line at `x0`; the thumb bar at `yf`, with the `xf` mark across it in DIFFERENTIAL; and
/// the figures above its right end.
fn draw_overlay<D: DrawTarget<Color = Rgb565>>(d: &mut D, o: &Overlay) {
    fill(d, layout::zero_rect(o.y0), FG_MUTED);
    if let Some(t) = o.turn {
        fill(d, layout::turn_rect(t.x0), FG_MUTED);
    }
    let (t0, t1) = layout::band(o.yf, layout::THUMB_H);
    let col = if o.live { PRIMARY } else { FG_MUTED };
    let _ = RoundedRectangle::with_equal_corners(
        er(Rect::new(2, t0, layout::WIDTH - 2, t1)),
        Size::new(5, 5),
    )
    .into_styled(PrimitiveStyle::with_fill(col))
    .draw(d);
    if let Some(t) = o.turn {
        let x =
            t.xf.clamp(layout::MARK_W / 2, layout::WIDTH - layout::MARK_W / 2);
        fill(
            d,
            Rect::new(x - layout::MARK_W / 2, t0, x + layout::MARK_W / 2, t1),
            FG,
        );
    }
    text(
        d,
        &o.text(),
        Point::new(layout::WIDTH - 4, (t0 - 14).max(0)),
        &FONT_8X13,
        FG,
        Alignment::Right,
    );
}
