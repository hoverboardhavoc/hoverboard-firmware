// SPDX-License-Identifier: GPL-3.0-or-later
//! The SPI master: bring-up from a [`SpiConfig`] and a polled full-duplex byte transfer.
//!
//! One register model on every chip this HAL builds (`gd32f10x_spi.h` and `gd32f1x0_spi.h` agree on
//! the offsets and bits used here), so there is no `hal_spi` model cfg.
//!
//! | Register | Offset | Used here |
//! |---|---|---|
//! | `CTL0` | `0x00` | `CKPH` 0, `CKPL` 1, `MSTMOD` 2, `PSC[5:3]`, `SPIEN` 6, `LF` 7, `SWNSS` 8, `SWNSSEN` 9 |
//! | `STAT` | `0x08` | `RBNE` 0, `TBE` 1, `CONFERR` 5, `RXORERR` 6 |
//! | `DATA` | `0x0C` | transmit and receive |
//! | `I2SCTL` | `0x1C` | `I2SSEL` 11, cleared for SPI mode |
//!
//! [`Spi::bring_up`] programs the SPI block only. The caller enables its clock
//! ([`crate::clock::enable_spi`]) and routes its pins ([`Chip::route_spi_pin`]), the calls the
//! golden vectors make separately.
//!
//! What the bring-up takes: master, full duplex, 8-bit frames, software NSS (the caller drives chip
//! select), SPI mode 0 to 3, either bit order. A 16-bit frame or a hardware NSS has no transfer or
//! caller here and is refused with `Unsupported`.

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::config::{NssMode, SpiConfig};
use crate::error::{DescriptorError, SpiError};
use crate::reg::Reg32;

const CTL0: u32 = 0x00;
const STAT: u32 = 0x08;
const DATA: u32 = 0x0C;
const I2SCTL: u32 = 0x1C;

const CTL0_MSTMOD: u32 = 1 << 2;
const CTL0_PSC_SHIFT: u32 = 3;
const CTL0_SPIEN: u32 = 1 << 6;
const CTL0_LF: u32 = 1 << 7;
const CTL0_SWNSS: u32 = 1 << 8;
const CTL0_SWNSSEN: u32 = 1 << 9;
/// The `CTL0` bits the vendor's `spi_init` keeps (`SPI_INIT_MASK`: `SPIEN`, `CRCNT`, `CRCEN`);
/// the rest are rewritten from the parameters.
const SPI_INIT_MASK: u32 = 0x3040;

const I2SCTL_I2SSEL: u32 = 1 << 11;

const STAT_RBNE: u32 = 1 << 0;
const STAT_TBE: u32 = 1 << 1;
const STAT_CONFERR: u32 = 1 << 5;
const STAT_RXORERR: u32 = 1 << 6;

/// Polls one `TBE` or `RBNE` wait may spend. At the slowest SCK the prescaler can reach from a
/// 72 MHz bus (/256, 281 kHz) a byte takes about 2,050 core cycles, so a working bus finishes far
/// inside it; a dead one returns `SpiError::Other`.
pub const SPI_TIMEOUT: u32 = 100_000;

/// The `PSC` code (0 for /2 up to 7 for /256) of the smallest divider that keeps
/// `bus_hz / divider` at or under `target_hz`, or 7 when even /256 is faster than the target.
const fn psc_code(bus_hz: u32, target_hz: u32) -> u32 {
    let mut code = 0;
    while code < 7 && bus_hz / (2 << code) > target_hz {
        code += 1;
    }
    code
}

/// The parameter bits `spi_init` ORs into `CTL0` for `cfg` on a bus at `bus_hz`: `SPI_MASTER`
/// (`MSTMOD | SWNSS`), `SPI_NSS_SOFT` (`SWNSSEN`), the bit order, `CKPL`/`CKPH` from the mode
/// (`mode` is `CPOL << 1 | CPHA`, the same bit positions), and the prescaler.
fn ctl0_params(cfg: &SpiConfig, bus_hz: u32) -> Result<u32, DescriptorError> {
    if cfg.mode > 3 || cfg.data16 || cfg.nss_mode != NssMode::Software {
        return Err(DescriptorError::Unsupported);
    }
    let lf = if cfg.lsb_first { CTL0_LF } else { 0 };
    Ok(CTL0_MSTMOD
        | CTL0_SWNSS
        | CTL0_SWNSSEN
        | lf
        | u32::from(cfg.mode)
        | (psc_code(bus_hz, cfg.target_hz) << CTL0_PSC_SHIFT))
}

/// An SPI master on one instance.
#[derive(Debug)]
pub struct Spi {
    base: u32,
}

impl Spi {
    /// Program `cfg.spi` as the vendor's `spi_init` then `spi_enable` leave it: `CTL0` read, masked
    /// with `SPI_INIT_MASK`, ORed with the parameters and written; `I2SCTL.I2SSEL` cleared; `SPIEN`
    /// set. Refused: an instance that is not an SPI the chip carries (`MissingBase`), or a
    /// configuration outside what the module takes (`Unsupported`).
    pub fn bring_up(
        chip: &Chip,
        clock: &ClockConfig,
        cfg: &SpiConfig,
    ) -> Result<Spi, DescriptorError> {
        // SPI0 is on APB2.
        let bus_hz = match cfg.spi {
            PeriphLabel::Spi0 => clock.apb2_hz(),
            other => return Err(DescriptorError::MissingBase(other)),
        };
        let params = ctl0_params(cfg, bus_hz)?;
        let dev = Spi {
            base: chip.base(cfg.spi)?,
        };
        let ctl0 = dev.reg(CTL0);
        ctl0.write((ctl0.read() & SPI_INIT_MASK) | params);
        dev.reg(I2SCTL).clear_bits(I2SCTL_I2SSEL);
        ctl0.set_bits(CTL0_SPIEN);
        Ok(dev)
    }

    #[inline]
    fn reg(&self, offset: u32) -> Reg32 {
        Reg32::new(self.base, offset)
    }

    /// Poll `STAT` until `flag` is set; a mode fault or overrun seen first is that error, and
    /// [`SPI_TIMEOUT`] polls without it is `Other`.
    fn wait_flag(&self, flag: u32) -> Result<(), SpiError> {
        let mut budget = SPI_TIMEOUT;
        loop {
            let s = self.reg(STAT).read();
            if s & flag != 0 {
                return Ok(());
            }
            if s & STAT_CONFERR != 0 {
                return Err(SpiError::ModeFault);
            }
            if s & STAT_RXORERR != 0 {
                return Err(SpiError::Overrun);
            }
            budget -= 1;
            if budget == 0 {
                return Err(SpiError::Other);
            }
        }
    }

    /// One full-duplex byte: wait `TBE`, write `tx`, wait `RBNE`, read the byte clocked in.
    pub fn transfer_byte(&self, tx: u8) -> Result<u8, SpiError> {
        self.wait_flag(STAT_TBE)?;
        self.reg(DATA).write(u32::from(tx));
        self.wait_flag(STAT_RBNE)?;
        Ok((self.reg(DATA).read() & 0xFF) as u8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> SpiConfig {
        // The bringup_spi0_1m_mode0 golden's request.
        SpiConfig {
            spi: PeriphLabel::Spi0,
            sck: 0x05,
            miso: 0x06,
            mosi: 0x07,
            nss: 0x04,
            mode: 0,
            data16: false,
            target_hz: 1_000_000,
            lsb_first: false,
            nss_mode: NssMode::Software,
        }
    }

    #[test]
    fn one_mhz_from_72_is_divide_by_128() {
        // /64 gives 1.125 MHz, over the target; /128 gives 562.5 kHz.
        assert_eq!(psc_code(72_000_000, 1_000_000), 6);
    }

    #[test]
    fn the_prescaler_is_the_smallest_divider_at_or_under_the_target() {
        assert_eq!(psc_code(72_000_000, 36_000_000), 0);
        assert_eq!(psc_code(72_000_000, 35_999_999), 1);
        assert_eq!(psc_code(72_000_000, 18_000_000), 1);
        // Slower than /256 can reach: clamped to /256.
        assert_eq!(psc_code(72_000_000, 1_000), 7);
        assert_eq!(psc_code(72_000_000, 0), 7);
        for code in 0..8u32 {
            let div = 2u32 << code;
            assert_eq!(psc_code(72_000_000, 72_000_000 / div), code, "/{div}");
        }
    }

    #[test]
    fn the_golden_request_gives_the_vendor_ctl0() {
        // MSTMOD | SWNSS | SWNSSEN | PSC 6; with SPIEN that is the golden's 0x374.
        let bits = ctl0_params(&cfg(), ClockConfig::REFERENCE_72M_IRC8M.apb2_hz()).unwrap();
        assert_eq!(bits, 0x334);
        assert_eq!(bits | CTL0_SPIEN, 0x374);
    }

    #[test]
    fn mode_and_bit_order_land_in_their_bits() {
        for mode in 0..4u8 {
            let c = SpiConfig { mode, ..cfg() };
            assert_eq!(ctl0_params(&c, 72_000_000).unwrap() & 0b11, u32::from(mode));
        }
        let c = SpiConfig {
            lsb_first: true,
            ..cfg()
        };
        assert_eq!(ctl0_params(&c, 72_000_000).unwrap() & CTL0_LF, CTL0_LF);
        // The parameters never carry a bit the init mask keeps.
        assert_eq!(ctl0_params(&c, 72_000_000).unwrap() & SPI_INIT_MASK, 0);
    }

    #[test]
    fn requests_without_a_transfer_here_are_refused() {
        for c in [
            SpiConfig { mode: 4, ..cfg() },
            SpiConfig {
                data16: true,
                ..cfg()
            },
            SpiConfig {
                nss_mode: NssMode::Hardware,
                ..cfg()
            },
        ] {
            assert_eq!(
                ctl0_params(&c, 72_000_000),
                Err(DescriptorError::Unsupported)
            );
        }
    }
}
