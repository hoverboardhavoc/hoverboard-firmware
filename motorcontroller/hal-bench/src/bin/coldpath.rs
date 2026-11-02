// SPDX-License-Identifier: GPL-3.0-or-later
//! `coldpath`: the clock tree, GPIO, I2C and the regular ADC on silicon (specs/hal.md section 8.4).
//!
//! The 72 MHz tree (`clock::configure_tree`), measured against SysTick-free means: the DWT cycle
//! counter over a known number of core cycles is trivially consistent, so the check is the RCU
//! read-back (PLL source, multiplier, switch status) and the flash wait states. I2C: the IMU's bus
//! (I2C0 PB6/PB7, or I2C1 PB10/PB11 with `imu-i2c1`) at the firmware's 400 kHz, WHO_AM_I (`0x75`)
//! of the device at `0x68` read through `embedded-hal`'s `write_read`, then `I2c::bus_clear` and
//! the read again. ADC: ADC0 calibrated, VREFINT (17) and the temperature sensor (16) read raw.
//!
//! Result words: 2 = RCU_CTL, 3 = RCU_CFG0, 4 = FMC_WS, 5 = clock verdict (1 = PLL on and locked,
//! switched to PLL, x18 from IRC8M/2), 6 = I2C bring-up (0 ok, else DescriptorError::code), 7 =
//! WHO_AM_I (0x100 | value, or 0x200 | I2cError::code), 8 = bus_clear (0 ok, else I2cError::code),
//! 9 = WHO_AM_I after the clear (as 7), 10 = ADC calibrate (0 ok), 11 = VREFINT raw (0x1_0000 on
//! error), 12 = temperature raw, 13 = GPIO: the SCL pin's input level read back high after the
//! bring-up's release (1).
#![no_std]
#![no_main]

#[cfg(all(target_os = "none", feature = "gd32"))]
mod image {
    use cortex_m_rt::entry;
    use embedded_hal::i2c::I2c as _;
    use hal::clock::{self, ClockConfig};
    use hal::reg::Reg32;
    use hal::{Chip, I2c, I2cMode, PeriphLabel};
    use hal_bench::{finish, put};
    use panic_halt as _;

    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;

    #[cfg(not(feature = "imu-i2c1"))]
    const BUS: (PeriphLabel, u8, u8) = (PeriphLabel::I2c0, 0x16, 0x17);
    #[cfg(feature = "imu-i2c1")]
    const BUS: (PeriphLabel, u8, u8) = (PeriphLabel::I2c1, 0x1A, 0x1B);

    fn who(bus: &mut I2c) -> u32 {
        let mut b = [0u8];
        match bus.write_read(0x68, &[0x75], &mut b) {
            Ok(()) => 0x100 | u32::from(b[0]),
            Err(e) => 0x200 | u32::from(e.code()),
        }
    }

    #[entry]
    fn main() -> ! {
        let Some(chip) = Chip::take() else { finish(4) };
        let tree = clock::configure_tree(&chip, &CLOCK).is_ok();
        let ctl = Reg32::new(0x4002_1000, 0x00).read();
        let cfg0 = Reg32::new(0x4002_1000, 0x04).read();
        put(2, ctl);
        put(3, cfg0);
        put(4, Reg32::new(0x4002_2000, 0).read());
        let pll_on = ctl & (1 << 24 | 1 << 25) == (1 << 24 | 1 << 25);
        let switched = cfg0 & (0b11 << 2) == 0b10 << 2;
        let x18_irc = cfg0 & (1 << 27 | 0b1111 << 18 | 1 << 16) == (1 << 27 | 1 << 18);
        put(5, u32::from(tree && pll_on && switched && x18_irc));

        let (label, scl, sda) = BUS;
        let pins = (chip.pin(scl), chip.pin(sda));
        let (Ok(p_scl), Ok(p_sda)) = pins else {
            finish(4)
        };
        match I2c::new(
            &chip,
            &CLOCK,
            label,
            (p_scl, p_sda),
            I2cMode::fast(400_000, hal::i2c::FastDuty::Two),
        ) {
            Ok(mut bus) => {
                put(6, 0);
                put(7, who(&mut bus));
                put(
                    8,
                    bus.bus_clear().map_or_else(|e| u32::from(e.code()), |()| 0),
                );
                put(9, who(&mut bus));
            }
            Err(e) => put(6, u32::from(e.code())),
        }
        // The SCL line's level through the HAL's input path: it idles high once the block owns it.
        let level = chip
            .input_group([scl, scl, scl])
            .map_or(0xFF, |g| u32::from(g.read() & 1));
        put(13, level);

        let rcu = chip.rcu_base().unwrap_or(0);
        let _ = clock::enable_adc(rcu, chip.clock(), PeriphLabel::Adc0);
        let adc = chip.adc();
        adc.configure_single(17, 7);
        put(10, u32::from(adc.calibrate().is_err()));
        put(11, adc.read_channel(17).map_or(0x1_0000, u32::from));
        adc.configure_single(16, 7);
        put(12, adc.read_channel(16).map_or(0x1_0000, u32::from));
        finish(4)
    }
}

/// The MM32SPIN05 coldpath, on the Gen2.4.2 master: the HSI 72 MHz tree
/// (`ClockConfig::REFERENCE_72M_HSI`), the IMU on I2C1 at SCL PD1 / SDA PD0, an MPU-6050 clone at
/// `0x69` (`specs/bench-work/mm32/README.md`), at 400 kHz; ADC1 with VREFINT (15) and the
/// temperature sensor (14). The part's ADC has no calibration (word 10 is `0xFF`).
///
/// Result words as the GD32 coldpath, with: 2 = `RCC_CR`, 3 = `RCC_CFGR`, 4 = `FLASH_ACR`, 5 =
/// clock verdict (1 = HSI ready, the 72 MHz output selected, switched to the HSI, AHB / 1, two
/// flash wait states), 14 = `SMPLRT_DIV` (`0x19`) as read, 15 = its write-back with a STOP (0 ok,
/// else `I2cError::code`), 16 = `SMPLRT_DIV` read again (0x100 | value).
#[cfg(all(target_os = "none", feature = "mm32spin05"))]
mod image {
    use cortex_m_rt::entry;
    use embedded_hal::i2c::I2c as _;
    use hal::clock::{self, ClockConfig};
    use hal::reg::Reg32;
    use hal::{Chip, I2c, I2cMode, PeriphLabel};
    use hal_bench::{finish, put};
    use panic_halt as _;

    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_HSI;
    const IMU: u8 = 0x69;
    /// SCL PD1, SDA PD0.
    const BUS: (u8, u8) = (0x31, 0x30);

    fn reg8(bus: &mut I2c, reg: u8) -> u32 {
        let mut b = [0u8];
        match bus.write_read(IMU, &[reg], &mut b) {
            Ok(()) => 0x100 | u32::from(b[0]),
            Err(e) => 0x200 | u32::from(e.code()),
        }
    }

    #[entry]
    fn main() -> ! {
        let Some(chip) = Chip::take() else { finish(4) };
        let tree = clock::configure_tree(&chip, &CLOCK).is_ok();
        let cr = Reg32::new(0x4002_1000, 0x00).read();
        let cfgr = Reg32::new(0x4002_1000, 0x04).read();
        let acr = Reg32::new(0x4002_2000, 0).read();
        put(2, cr);
        put(3, cfgr);
        put(4, acr);
        let hsi = cr & (1 << 1 | 1 << 20) == (1 << 1 | 1 << 20);
        let switched = cfgr & (0b11 << 2) == 0b10 << 2;
        let ahb1 = cfgr & (0b1111 << 4) == 0;
        let latency = acr & 0b111 == 2;
        put(5, u32::from(tree && hsi && switched && ahb1 && latency));

        let (scl, sda) = BUS;
        let (Ok(p_scl), Ok(p_sda)) = (chip.pin(scl), chip.pin(sda)) else {
            finish(4)
        };
        match I2c::new(
            &chip,
            &CLOCK,
            PeriphLabel::I2c0,
            (p_scl, p_sda),
            I2cMode::fast(400_000, hal::i2c::FastDuty::Two),
        ) {
            Ok(mut bus) => {
                put(6, 0);
                put(7, reg8(&mut bus, 0x75));
                let div = reg8(&mut bus, 0x19);
                put(14, div);
                // The write path with a STOP: SMPLRT_DIV written back with the value it holds.
                if div & 0x100 != 0 {
                    let v = (div & 0xFF) as u8;
                    put(
                        15,
                        bus.write(IMU, &[0x19, v])
                            .map_or_else(|e| u32::from(e.code()), |()| 0),
                    );
                    put(16, reg8(&mut bus, 0x19));
                }
                put(
                    8,
                    bus.bus_clear().map_or_else(|e| u32::from(e.code()), |()| 0),
                );
                put(9, reg8(&mut bus, 0x75));
            }
            Err(e) => put(6, u32::from(e.code())),
        }
        let level = chip
            .input_group([scl, scl, scl])
            .map_or(0xFF, |g| u32::from(g.read() & 1));
        put(13, level);

        let rcu = chip.rcu_base().unwrap_or(0);
        let _ = clock::enable_adc(rcu, chip.clock(), PeriphLabel::Adc0);
        let adc = chip.adc();
        put(10, 0xFF);
        adc.configure_single(15, 7);
        put(11, adc.read_channel(15).map_or(0x1_0000, u32::from));
        adc.configure_single(14, 7);
        put(12, adc.read_channel(14).map_or(0x1_0000, u32::from));
        finish(4)
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
