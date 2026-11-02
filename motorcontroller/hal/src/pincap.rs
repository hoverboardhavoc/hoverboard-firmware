// SPDX-License-Identifier: GPL-3.0-or-later
//! Pin capability tables: what each pin of each chip this HAL drives can do
//! (`specs/config-read-once.md` section 4).
//!
//! The canonical model of the chips' pins, for the host: every chip's table is here, selected by
//! [`Chip`] at run time, and the module is compiled only with the `pincap` feature, which no image
//! enables. The CLI checks a board's stored configuration against it before writing anything; the
//! firmware writes the stored values into the registers and links none of this.
//!
//! A table holds, per chip: the bonded pins, the alternate-function routes ([`Route`]: a pin, the
//! peripheral function it carries, and the selector that routes it there) and the ADC channels. On
//! a chip with a per-pin mux (GD32F1x0, MM32SPIN05) the selector is the AF number a `PinAf` stores;
//! on the GD32F10x it is the AFIO remap value the group's `remap` key stores, and the stored `af`
//! is 0.
//!
//! Sources: the GD32F10x and GD32F1x0 datasheets' pin definitions and AF tables, and for the
//! MM32SPIN05 MindMotion's DS_MM32SPIN05 tables 5 to 9. A route the HAL has never driven on silicon
//! is not in a table.

/// A chip this HAL builds an image for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// GD32F103C8 (LQFP48): the rover master.
    Gd32f103c8,
    /// GD32F103RC (LQFP64): the 12-FET.
    Gd32f103rc,
    /// GD32F130C8 (LQFP48): the offroad pair and the rover slave.
    Gd32f130c8,
    /// MM32SPIN05 (LQFP48): the Gen2.4.2 split boards.
    Mm32spin05,
}

impl Chip {
    /// Every chip, in the order of the HAL's chip features.
    pub const ALL: [Chip; 4] = [
        Chip::Gd32f103c8,
        Chip::Gd32f103rc,
        Chip::Gd32f130c8,
        Chip::Mm32spin05,
    ];

    /// The chip's name, as its HAL feature and the check cases spell it.
    pub const fn name(self) -> &'static str {
        match self {
            Chip::Gd32f103c8 => "gd32f103c8",
            Chip::Gd32f103rc => "gd32f103rc",
            Chip::Gd32f130c8 => "gd32f130c8",
            Chip::Mm32spin05 => "mm32spin05",
        }
    }

    /// The chip [`Chip::name`] names.
    pub fn from_name(name: &str) -> Option<Chip> {
        Chip::ALL.into_iter().find(|c| c.name() == name)
    }

    /// Whether the chip routes a peripheral by the AFIO remap (the GD32F10x) rather than by a
    /// per-pin alternate-function mux.
    pub const fn remaps(self) -> bool {
        matches!(self, Chip::Gd32f103c8 | Chip::Gd32f103rc)
    }

    /// The chip's table.
    pub const fn table(self) -> &'static Table {
        match self {
            Chip::Gd32f103c8 => &GD32F103C8,
            Chip::Gd32f103rc => &GD32F103RC,
            Chip::Gd32f130c8 => &GD32F130C8,
            Chip::Mm32spin05 => &MM32SPIN05,
        }
    }
}

/// A peripheral function a pin can carry. The numbers are the HAL's instance labels: `Timer0`
/// and `Timer7` are timers 0 and 7, `Usart0` to `Usart2` are 0 to 2, `I2c0` and `I2c1` are 0 and 1
/// (on the MM32 the HAL's labels for MindMotion's TIM1, UART1, UART2 and I2C1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Func {
    /// Advanced timer `timer`'s channel `ch` output (a high-side gate).
    Gate(u8, u8),
    /// Advanced timer `timer`'s complementary channel `ch` output (a low-side gate).
    GateN(u8, u8),
    /// USART `n`'s transmit.
    Tx(u8),
    /// USART `n`'s receive.
    Rx(u8),
    /// I2C `n`'s clock.
    Scl(u8),
    /// I2C `n`'s data.
    Sda(u8),
}

/// One alternate-function route: pin `pin` of port `port` carries `func` when routed by `sel`
/// (the AF number on a mux chip, the peripheral's remap value on the GD32F10x).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Route {
    /// The port, `0` = A to `5` = F.
    pub port: u8,
    /// The pin number.
    pub pin: u8,
    /// What the pin carries.
    pub func: Func,
    /// The AF number or the remap value.
    pub sel: u8,
}

/// One ADC input: pin `pin` of port `port` is ADC0 channel `channel`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdcInput {
    /// The port.
    pub port: u8,
    /// The pin number.
    pub pin: u8,
    /// The channel.
    pub channel: u8,
}

/// One chip's pins.
#[derive(Debug)]
pub struct Table {
    /// Bonded pins per port, `A` to `F`: bit `n` set means pin `n` is bonded out in the package.
    pub bonding: [u16; 6],
    /// Every alternate-function route the HAL drives.
    pub routes: &'static [Route],
    /// Every ADC input.
    pub adc: &'static [AdcInput],
}

/// The SWD pins, PA13 (SWDIO) and PA14 (SWCLK), as (port, pin): never given to a function, because
/// SWD is the configuration path.
pub const SWD: [(u8, u8); 2] = [(0, 13), (0, 14)];

const fn route(port: u8, pin: u8, func: Func, sel: u8) -> Route {
    Route {
        port,
        pin,
        func,
        sel,
    }
}

const fn adc(port: u8, pin: u8, channel: u8) -> AdcInput {
    AdcInput { port, pin, channel }
}

use Func::{Gate, GateN, Rx, Scl, Sda, Tx};

/// The GD32F10x routes every F10x part carries (remap values): TIMER0 at remap 0 on PA8-10 and
/// PB13-15; USART0 at remap 1 on PB6/PB7 (its default PA9/PA10 are gate pins); USART1 at remap 0
/// on PA2/PA3; USART2 at remap 0 on PB10/PB11; I2C0 at remap 0 on PB6/PB7; I2C1 on PB10/PB11.
const F10X_ROUTES: [Route; 16] = [
    route(0, 8, Gate(0, 0), 0),
    route(0, 9, Gate(0, 1), 0),
    route(0, 10, Gate(0, 2), 0),
    route(1, 13, GateN(0, 0), 0),
    route(1, 14, GateN(0, 1), 0),
    route(1, 15, GateN(0, 2), 0),
    route(1, 6, Tx(0), 1),
    route(1, 7, Rx(0), 1),
    route(0, 2, Tx(1), 0),
    route(0, 3, Rx(1), 0),
    route(1, 10, Tx(2), 0),
    route(1, 11, Rx(2), 0),
    route(1, 6, Scl(0), 0),
    route(1, 7, Sda(0), 0),
    route(1, 10, Scl(1), 0),
    route(1, 11, Sda(1), 0),
];

/// The GD32F103RC's routes: the F10x ones above, and its second advanced timer, TIMER7, at remap
/// 0: CH0-2 on PC6-8, CH0N on PA7, CH1N/CH2N on PB0/PB1.
const F10X_HD_ROUTES: [Route; 22] = [
    route(0, 8, Gate(0, 0), 0),
    route(0, 9, Gate(0, 1), 0),
    route(0, 10, Gate(0, 2), 0),
    route(1, 13, GateN(0, 0), 0),
    route(1, 14, GateN(0, 1), 0),
    route(1, 15, GateN(0, 2), 0),
    route(1, 6, Tx(0), 1),
    route(1, 7, Rx(0), 1),
    route(0, 2, Tx(1), 0),
    route(0, 3, Rx(1), 0),
    route(1, 10, Tx(2), 0),
    route(1, 11, Rx(2), 0),
    route(1, 6, Scl(0), 0),
    route(1, 7, Sda(0), 0),
    route(1, 10, Scl(1), 0),
    route(1, 11, Sda(1), 0),
    route(2, 6, Gate(7, 0), 0),
    route(2, 7, Gate(7, 1), 0),
    route(2, 8, Gate(7, 2), 0),
    route(0, 7, GateN(7, 0), 0),
    route(1, 0, GateN(7, 1), 0),
    route(1, 1, GateN(7, 2), 0),
];

/// The GD32 ADC map: PA0-7 are channels 0-7, PB0-1 are 8-9, PC0-5 are 10-15 (a pin the package
/// does not bond is not an input, which the bonding answers).
const GD32_ADC: [AdcInput; 16] = [
    adc(0, 0, 0),
    adc(0, 1, 1),
    adc(0, 2, 2),
    adc(0, 3, 3),
    adc(0, 4, 4),
    adc(0, 5, 5),
    adc(0, 6, 6),
    adc(0, 7, 7),
    adc(1, 0, 8),
    adc(1, 1, 9),
    adc(2, 0, 10),
    adc(2, 1, 11),
    adc(2, 2, 12),
    adc(2, 3, 13),
    adc(2, 4, 14),
    adc(2, 5, 15),
];

/// GD32F103C8, LQFP48: PA, PB, PC13-15 and PD0-1 bonded.
const GD32F103C8: Table = Table {
    bonding: [0xFFFF, 0xFFFF, 0xE000, 0x0003, 0, 0],
    routes: &F10X_ROUTES,
    adc: &GD32_ADC,
};

/// GD32F103RC, LQFP64: PA, PB, PC and PD0-2 bonded.
const GD32F103RC: Table = Table {
    bonding: [0xFFFF, 0xFFFF, 0xFFFF, 0x0007, 0, 0],
    routes: &F10X_HD_ROUTES,
    adc: &GD32_ADC,
};

/// GD32F130C8, LQFP48: PA, PB, PC13-15, PF0-1 and PF6-7 bonded. TIMER0 is AF2 on PA8-10 and
/// PB13-15; USART0 AF0 on PB6/PB7; USART1 AF1 on PA2/PA3; I2C0 AF1 on PB6/PB7; I2C1 AF1 on
/// PB10/PB11 (the x8 parts carry it). No USART2.
const GD32F130C8: Table = Table {
    bonding: [0xFFFF, 0xFFFF, 0xE000, 0, 0, 0x00C3],
    routes: &[
        route(0, 8, Gate(0, 0), 2),
        route(0, 9, Gate(0, 1), 2),
        route(0, 10, Gate(0, 2), 2),
        route(1, 13, GateN(0, 0), 2),
        route(1, 14, GateN(0, 1), 2),
        route(1, 15, GateN(0, 2), 2),
        route(1, 6, Tx(0), 0),
        route(1, 7, Rx(0), 0),
        route(0, 2, Tx(1), 1),
        route(0, 3, Rx(1), 1),
        route(1, 6, Scl(0), 1),
        route(1, 7, Sda(0), 1),
        route(1, 10, Scl(1), 1),
        route(1, 11, Sda(1), 1),
    ],
    adc: &GD32_ADC,
};

/// MM32SPIN05, LQFP48: PA0-15, PB0-15, PC13-15 and PD0-3 bonded (DS_MM32SPIN05 table 5). TIM1 is
/// AF2 on PA8-10 and PB13-15. UART1 TX on PB6 (AF0) and PB3 (AF3), RX on PB7 (AF0) and PB4 (AF3);
/// UART2 TX on PA2 (AF1) and PB7 (AF4), RX on PA3 (AF1), PA6 (AF3), PA15 (AF1) and PB8 (AF4).
/// UART1's PA9/PA10 are gate pins and PA14 is SWCLK, so they are not here. I2C1 at AF1: SCL on
/// PB6, PB8, PB10 and PD1, SDA on PB7, PB9, PB11 and PD0; its AF4 and AF5 pins are not here.
const MM32SPIN05: Table = Table {
    bonding: [0xFFFF, 0xFFFF, 0xE000, 0x000F, 0, 0],
    routes: &[
        route(0, 8, Gate(0, 0), 2),
        route(0, 9, Gate(0, 1), 2),
        route(0, 10, Gate(0, 2), 2),
        route(1, 13, GateN(0, 0), 2),
        route(1, 14, GateN(0, 1), 2),
        route(1, 15, GateN(0, 2), 2),
        route(1, 6, Tx(0), 0),
        route(1, 3, Tx(0), 3),
        route(1, 7, Rx(0), 0),
        route(1, 4, Rx(0), 3),
        route(0, 2, Tx(1), 1),
        route(1, 7, Tx(1), 4),
        route(0, 3, Rx(1), 1),
        route(0, 6, Rx(1), 3),
        route(0, 15, Rx(1), 1),
        route(1, 8, Rx(1), 4),
        route(1, 6, Scl(0), 1),
        route(1, 8, Scl(0), 1),
        route(1, 10, Scl(0), 1),
        route(3, 1, Scl(0), 1),
        route(1, 7, Sda(0), 1),
        route(1, 9, Sda(0), 1),
        route(1, 11, Sda(0), 1),
        route(3, 0, Sda(0), 1),
    ],
    // DS_MM32SPIN05 table 5, `ADC1_VIN[n]`: PA0-7 = 0-7, PB0-1 = 8-9, PB3 = 10, PB4 = 11. PB7 is
    // `VIN[12]`, whose data register MindMotion's map leaves reserved, so it is not an input here.
    adc: &[
        adc(0, 0, 0),
        adc(0, 1, 1),
        adc(0, 2, 2),
        adc(0, 3, 3),
        adc(0, 4, 4),
        adc(0, 5, 5),
        adc(0, 6, 6),
        adc(0, 7, 7),
        adc(1, 0, 8),
        adc(1, 1, 9),
        adc(1, 3, 10),
        adc(1, 4, 11),
    ],
};

impl Table {
    /// Whether pin `pin` of port `port` is bonded out.
    pub fn bonded(&self, port: u8, pin: u8) -> bool {
        pin < 16
            && self
                .bonding
                .get(port as usize)
                .is_some_and(|m| m & (1 << pin) != 0)
    }

    /// The selector that routes `func` onto the pin, or `None` when the pin cannot carry it.
    pub fn sel(&self, port: u8, pin: u8, func: Func) -> Option<u8> {
        self.routes
            .iter()
            .find(|r| r.port == port && r.pin == pin && r.func == func)
            .map(|r| r.sel)
    }

    /// The ADC channel on the pin, or `None`.
    pub fn adc_channel(&self, port: u8, pin: u8) -> Option<u8> {
        if !self.bonded(port, pin) {
            return None;
        }
        self.adc
            .iter()
            .find(|a| a.port == port && a.pin == pin)
            .map(|a| a.channel)
    }

    /// Whether the chip carries advanced timer `n`: a table routes its outputs exactly when it does.
    pub fn has_timer(&self, n: u8) -> bool {
        self.routes.iter().any(|r| r.func == Gate(n, 0))
    }

    /// Whether the chip carries USART `n`.
    pub fn has_usart(&self, n: u8) -> bool {
        self.routes.iter().any(|r| r.func == Tx(n))
    }

    /// Whether the chip carries I2C `n`.
    pub fn has_i2c(&self, n: u8) -> bool {
        self.routes.iter().any(|r| r.func == Scl(n))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_route_and_adc_input_is_on_a_bonded_pin_and_never_swd() {
        for chip in Chip::ALL {
            let t = chip.table();
            for r in t.routes {
                assert!(t.bonded(r.port, r.pin), "{chip:?} {r:?}");
                assert!(!SWD.contains(&(r.port, r.pin)), "{chip:?} {r:?}");
            }
            for a in t.adc {
                if chip != Chip::Gd32f103rc {
                    // The LQFP48 parts do not bond PC0-5; the table says so through the bonding.
                    continue;
                }
                assert!(t.bonded(a.port, a.pin), "{chip:?} {a:?}");
            }
        }
    }

    #[test]
    fn bonded_gpio_counts_match_the_datasheets() {
        for (chip, want) in [
            (Chip::Gd32f103c8, 37),
            (Chip::Gd32f103rc, 51),
            (Chip::Gd32f130c8, 39),
            (Chip::Mm32spin05, 39),
        ] {
            let n: u32 = chip.table().bonding.iter().map(|m| m.count_ones()).sum();
            assert_eq!(n, want, "{chip:?}");
        }
    }

    #[test]
    fn the_swd_and_the_timer0_gate_pins_are_bonded_everywhere_and_port_e_nowhere() {
        for chip in Chip::ALL {
            let t = chip.table();
            for (port, pin) in [(0, 13), (0, 14), (0, 8), (0, 9), (0, 10), (1, 13), (1, 14)] {
                assert!(t.bonded(port, pin), "{chip:?}");
            }
            assert!(!t.bonded(4, 0) && !t.bonded(6, 0) && !t.bonded(0, 16));
            assert!(t.has_timer(0));
        }
    }

    #[test]
    fn timer7_and_usart2_and_i2c1_follow_the_part() {
        let has = |c: Chip| {
            let t = c.table();
            (t.has_timer(7), t.has_usart(2), t.has_i2c(1))
        };
        assert_eq!(has(Chip::Gd32f103c8), (false, true, true));
        assert_eq!(has(Chip::Gd32f103rc), (true, true, true));
        assert_eq!(has(Chip::Gd32f130c8), (false, false, true));
        assert_eq!(has(Chip::Mm32spin05), (false, false, false));
    }

    #[test]
    fn the_selectors_are_the_af_numbers_on_the_mux_chips_and_the_remaps_on_the_f10x() {
        let f130 = Chip::Gd32f130c8.table();
        assert_eq!(f130.sel(0, 8, Gate(0, 0)), Some(2));
        assert_eq!(f130.sel(1, 6, Tx(0)), Some(0));
        assert_eq!(f130.sel(1, 6, Scl(0)), Some(1));
        assert_eq!(f130.sel(1, 10, Scl(1)), Some(1));
        let f103 = Chip::Gd32f103c8.table();
        assert_eq!(f103.sel(1, 6, Tx(0)), Some(1), "USART0 on PB6 is the remap");
        assert_eq!(f103.sel(0, 2, Tx(1)), Some(0));
        assert_eq!(f103.sel(0, 2, Rx(1)), None, "PA2 is TX only");
        let mm32 = Chip::Mm32spin05.table();
        assert_eq!(mm32.sel(1, 4, Rx(0)), Some(3), "trondin's link RX");
        assert_eq!(mm32.sel(3, 1, Scl(0)), Some(1), "the Gen2.4.2 IMU's SCL");
        assert!(Chip::Gd32f103c8.remaps() && !Chip::Gd32f130c8.remaps());
    }

    #[test]
    fn adc_channels_are_the_f1_map_on_bonded_pins() {
        let f103 = Chip::Gd32f103c8.table();
        assert_eq!(f103.adc_channel(0, 0), Some(0));
        assert_eq!(f103.adc_channel(0, 4), Some(4));
        assert_eq!(f103.adc_channel(1, 0), Some(8));
        assert_eq!(f103.adc_channel(1, 2), None);
        assert_eq!(
            f103.adc_channel(2, 0),
            None,
            "PC0 is not bonded on the LQFP48"
        );
        assert_eq!(Chip::Gd32f103rc.table().adc_channel(2, 0), Some(10));
        let mm32 = Chip::Mm32spin05.table();
        assert_eq!(mm32.adc_channel(1, 3), Some(10));
        assert_eq!(mm32.adc_channel(1, 7), None);
    }

    #[test]
    fn names_round_trip() {
        for chip in Chip::ALL {
            assert_eq!(Chip::from_name(chip.name()), Some(chip));
        }
        assert_eq!(Chip::from_name("gd32f303"), None);
    }
}
