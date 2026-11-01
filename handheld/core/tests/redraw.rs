//! The panel's drawing (`handheld/src/panel.rs`, compiled here for the host) against a model of
//! the ILI9341 bus as mipidsi 0.10 drives it (`specs/handheld-remote.md` 5.1): how many address
//! windows, SPI writes and bytes a redraw costs, and whether any pixel shows a value it does not
//! end on (the clear-then-draw flicker).
//!
//! The bus model follows mipidsi 0.10 with its default `batch` feature: `fill_solid` and
//! `fill_contiguous` set one window each; `draw_iter` groups the pixels into rows of at most 50
//! adjacent pixels and those into blocks of at most 100 pixels with the same columns on adjacent
//! rows, one window per block. A window is three DCS commands (CASET, RASET, RAMWR), each a
//! command write and an argument write, then the pixels in 4 KB interface-buffer writes.
//!
//! Run with `--nocapture` to print the figures.

#[allow(dead_code)]
#[path = "../../src/panel.rs"]
mod panel;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;

use handheld_core::layout::{HEIGHT, WIDTH};
use handheld_core::model::{
    ArmBand, Body, Corner, Dial, Dot, Lower, Pack, PhaseBar, Ride, SpeedShow, Top, View,
};
use handheld_core::readings::attitude::Attitude;

/// The SPI clock, Hz (section 5.1).
const SPI_HZ: f64 = 40e6;
/// The software cost of one SPI write (CS, DMA set-up and wait, DC), microseconds: an assumption
/// of the model, not a measurement. The device measures the real figure (`view:` log lines).
const WRITE_US: f64 = 5.0;
/// mipidsi's interface buffer (`main.rs`, `PANEL_BUF`).
const IFACE_BUF: u64 = 4096;
const ROW_MAX: usize = 50;
const BLOCK_MAX: usize = 100;

/// The panel as mipidsi drives it, with a shadow of what it shows.
struct Bus {
    shown: Vec<Rgb565>,
    /// Every value a pixel was written with during the current frame.
    written: Vec<Vec<Rgb565>>,
    windows: u64,
    writes: u64,
    bytes: u64,
}

impl Bus {
    fn new() -> Bus {
        let n = (WIDTH * HEIGHT) as usize;
        Bus {
            shown: vec![Rgb565::BLACK; n],
            written: vec![Vec::new(); n],
            windows: 0,
            writes: 0,
            bytes: 0,
        }
    }

    /// Start counting a frame.
    fn frame(&mut self) {
        self.windows = 0;
        self.writes = 0;
        self.bytes = 0;
        for w in &mut self.written {
            w.clear();
        }
    }

    /// One window of `n` pixels: three commands, then the pixels.
    fn window(&mut self, n: u64) {
        self.windows += 1;
        self.writes += 6 + (2 * n).div_ceil(IFACE_BUF);
        self.bytes += 3 + 8 + 2 * n;
    }

    fn put(&mut self, p: Point, c: Rgb565) {
        let i = (p.y * WIDTH + p.x) as usize;
        self.shown[i] = c;
        self.written[i].push(c);
    }

    /// The modelled bus time of the frame, ms.
    fn ms(&self) -> f64 {
        self.bytes as f64 * 8.0 / SPI_HZ * 1e3 + self.writes as f64 * WRITE_US / 1e3
    }

    /// Pixels that showed, during the frame, a value other than the one they end on.
    fn transient(&self) -> usize {
        self.written
            .iter()
            .zip(&self.shown)
            .filter(|(w, end)| w.iter().any(|c| c != *end))
            .count()
    }

    /// Pixels written more than once during the frame.
    fn rewritten(&self) -> usize {
        self.written.iter().filter(|w| w.len() > 1).count()
    }

    fn report(&self, what: &str) -> String {
        format!(
            "{what}: {} windows, {} SPI writes, {} bytes, {:.1} ms modelled \
             ({:.1} ms of it bytes at 40 MHz), {} pixels rewritten, {} transient",
            self.windows,
            self.writes,
            self.bytes,
            self.ms(),
            self.bytes as f64 * 8.0 / SPI_HZ * 1e3,
            self.rewritten(),
            self.transient()
        )
    }
}

impl OriginDimensions for Bus {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for Bus {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I: IntoIterator<Item = Pixel<Rgb565>>>(
        &mut self,
        pixels: I,
    ) -> Result<(), Self::Error> {
        // Rows of adjacent pixels, then blocks of rows (mipidsi's `batch`).
        let mut rows: Vec<(i32, i32, i32)> = Vec::new(); // (y, x_left, x_right)
        for Pixel(p, c) in pixels {
            if p.x < 0 || p.y < 0 || p.x >= WIDTH || p.y >= HEIGHT {
                continue;
            }
            self.put(p, c);
            match rows.last_mut() {
                Some((y, l, r))
                    if *y == p.y && p.x == *r + 1 && ((*r - *l + 1) as usize) < ROW_MAX =>
                {
                    *r = p.x
                }
                _ => rows.push((p.y, p.x, p.x)),
            }
        }
        let mut block: Option<(i32, i32, i32, usize)> = None; // (y_bottom, l, r, pixels)
        for (y, l, r) in rows {
            let n = (r - l + 1) as usize;
            match block {
                Some((yb, bl, br, bn))
                    if y == yb + 1 && l == bl && r == br && bn + n <= BLOCK_MAX =>
                {
                    block = Some((y, bl, br, bn + n))
                }
                Some((_, _, _, bn)) => {
                    self.window(bn as u64);
                    block = Some((y, l, r, n));
                }
                None => block = Some((y, l, r, n)),
            }
        }
        if let Some((_, _, _, bn)) = block {
            self.window(bn as u64);
        }
        Ok(())
    }

    fn fill_contiguous<I: IntoIterator<Item = Rgb565>>(
        &mut self,
        area: &Rectangle,
        colors: I,
    ) -> Result<(), Self::Error> {
        let clip = area.intersection(&self.bounding_box());
        if clip.size.width == 0 || clip.size.height == 0 {
            return Ok(());
        }
        for (p, c) in area.points().zip(colors) {
            if clip.contains(p) {
                self.put(p, c);
            }
        }
        self.window(u64::from(clip.size.width * clip.size.height));
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Rgb565) -> Result<(), Self::Error> {
        let clip = area.intersection(&self.bounding_box());
        if clip.size.width == 0 || clip.size.height == 0 {
            return Ok(());
        }
        for p in clip.points() {
            self.put(p, color);
        }
        self.window(u64::from(clip.size.width * clip.size.height));
        Ok(())
    }
}

fn s<const N: usize>(t: &str) -> heapless::String<N> {
    heapless::String::try_from(t).unwrap()
}

/// A Ride view with every widget showing a value: two boards, speed, pack, attitude, phase bars.
fn ride() -> View {
    let corner = |label: &str, pitch, roll| {
        Some(Corner {
            label: s(label),
            attitude: Attitude::Angles { pitch, roll },
        })
    };
    let bar = |label: &str| {
        Some(PhaseBar {
            label: s(label),
            text: s("6.8 A"),
            fill: Some(340),
        })
    };
    View {
        top: Top {
            dots: [
                Some(Dot {
                    label: s("L"),
                    heard: true,
                }),
                Some(Dot {
                    label: s("R"),
                    heard: true,
                }),
            ],
            low: false,
            test: false,
        },
        body: Body::Ride(Ride {
            dial: Dial {
                lit: 7,
                caution: false,
                speed: SpeedShow::Value {
                    text: s("12.3"),
                    reverse: false,
                },
            },
            corners: [corner("L", 230, -80), corner("R", -120, 410)],
            pack: Pack {
                text: s("39.0 V  72 %"),
                caution: false,
                muted: false,
            },
            lower: Lower::Bars {
                phase: [bar("L"), bar("R")],
                band: ArmBand::CanArm,
            },
        }),
        line: None,
        armed: false,
    }
}

#[test]
fn a_full_redraw_and_a_widget_change() {
    let mut bus = Bus::new();
    let mut cache = panel::Cache::default();
    let v = ride();
    bus.frame();
    panel::draw(&mut bus, &v, None, &mut cache);
    let full = bus.report("full redraw of the Ride view");
    println!("{full}");

    // The speed changes: only the dial redraws.
    let mut v2 = v.clone();
    if let Body::Ride(r) = &mut v2.body {
        r.dial.speed = SpeedShow::Value {
            text: s("12.4"),
            reverse: false,
        };
    }
    bus.frame();
    panel::draw(&mut bus, &v2, None, &mut cache);
    println!("{}", bus.report("the speed changes"));

    // The pack line changes.
    let mut v3 = v2.clone();
    if let Body::Ride(r) = &mut v3.body {
        r.pack.text = s("38.9 V  71 %");
    }
    bus.frame();
    panel::draw(&mut bus, &v3, None, &mut cache);
    println!("{}", bus.report("the pack line changes"));
}
