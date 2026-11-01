//! Section 7.2 check 2's test pattern (`specs/handheld-remote.md`), behind the `test-pattern`
//! feature.

use embedded_graphics::mono_font::iso_8859_1::{FONT_10X20, FONT_8X13};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Alignment;

use crate::panel::color::{BG, FG, NEON, PRIMARY};
use crate::panel::{fill, text};
use handheld_core::layout::Rect;

/// Section 7.2 check 2's test pattern: red, green, blue and white bars and a line of text, to
/// fix colour order, inversion and orientation.
pub fn test_pattern<D: DrawTarget<Color = Rgb565>>(d: &mut D) {
    let _ = d.clear(BG);
    for (i, c) in [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE, Rgb565::WHITE]
        .iter()
        .enumerate()
    {
        fill(
            d,
            Rect::new(0, 40 + i as i32 * 40, 240, 80 + i as i32 * 40),
            *c,
        );
    }
    text(
        d,
        "TOP  handheld",
        Point::new(4, 4),
        &FONT_10X20,
        FG,
        Alignment::Left,
    );
    text(
        d,
        "R G B W",
        Point::new(120, 210),
        &FONT_10X20,
        PRIMARY,
        Alignment::Center,
    );
    text(
        d,
        "USB is down here",
        Point::new(120, 300),
        &FONT_8X13,
        NEON,
        Alignment::Center,
    );
}
