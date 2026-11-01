//! The XPT2046 on SPI3 (`specs/handheld-remote.md` section 4.2): PENIRQ read as a level, then one
//! CS window of section 4.2's eight conversions. The frame bytes and their assembly are
//! `handheld_core::touch`'s.

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiBus;

use handheld_core::touch::{tx_frame, Raw, Reading, FRAME_LEN};

/// The touch controller: its SPI bus (mode 0, 1 MHz), CS and PENIRQ.
pub struct Xpt2046<SPI, CS, IRQ> {
    spi: SPI,
    cs: CS,
    irq: IRQ,
}

impl<SPI: SpiBus, CS: OutputPin, IRQ: InputPin> Xpt2046<SPI, CS, IRQ> {
    pub fn new(spi: SPI, mut cs: CS, irq: IRQ) -> Self {
        let _ = cs.set_high();
        Xpt2046 { spi, cs, irq }
    }

    /// One sample. PENIRQ high is Up with no conversion; a failed transfer is reported as one.
    pub fn read(&mut self) -> Reading {
        let pen_down = self.irq.is_low().unwrap_or(false);
        if !pen_down {
            return Reading {
                pen_down,
                transfer_ok: true,
                raw: Raw::default(),
            };
        }
        let mut buf = tx_frame();
        let _ = self.cs.set_low();
        let ok = self.spi.transfer_in_place(&mut buf).is_ok() && self.spi.flush().is_ok();
        let _ = self.cs.set_high();
        let rx: [u8; FRAME_LEN] = buf;
        Reading {
            pen_down,
            transfer_ok: ok,
            raw: Raw::parse(&rx),
        }
    }
}
