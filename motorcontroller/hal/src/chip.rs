// SPDX-License-Identifier: GPL-3.0-or-later
//! The chip context: the compiled chip's descriptor and the pin bring-ups that need it.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::addr::PeriphLabel;
use crate::descriptor::{McuDescriptor, PageSize, DESCRIPTOR};
use crate::error::DescriptorError;
use crate::gpio::{self, Floating, Input, InputGroup, Pin, PinRole, INPUT_GROUP_LINES};
use crate::pincap::{self, PinClaim};
use crate::reg::Reg32;

/// The clock tree of the compiled chip.
///
/// A marker, not a selector: there is one clock model in a build (the chip's), and the calls that
/// take it ([`crate::clock::enable_timer`] and the rest) keep the shape the firmware calls them
/// with. It carries no runtime value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockPath;

/// The interrupt layout of the compiled chip: a marker, as [`ClockPath`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IrqLayout;

/// The 96-bit factory unique ID, as the three words the part stores at its signature block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UniqueId(pub [u32; 3]);

impl UniqueId {
    /// The ID as 12 bytes, word 0 first, each word little-endian: the order an SWD dump of the
    /// signature block reads.
    pub const fn to_bytes(self) -> [u8; 12] {
        let [a, b, c] = self.0;
        let (a, b, c) = (a.to_le_bytes(), b.to_le_bytes(), c.to_le_bytes());
        [
            a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3], c[0], c[1], c[2], c[3],
        ]
    }
}

/// The context for the compiled chip.
///
/// Zero-sized: every fact it answers is the const [`DESCRIPTOR`]. [`Chip::take`] hands it out once,
/// so there is one owner of the bring-ups that claim pins and instances.
#[derive(Debug, PartialEq, Eq)]
pub struct Chip {
    _private: (),
}

static TAKEN: AtomicBool = AtomicBool::new(false);

impl Chip {
    /// The chip context, the first time it is asked for; `None` after that.
    pub fn take() -> Option<Chip> {
        cortex_m::interrupt::free(|_| {
            if TAKEN.load(Ordering::Relaxed) {
                None
            } else {
                TAKEN.store(true, Ordering::Relaxed);
                Some(Chip { _private: () })
            }
        })
    }

    /// The chip context without the take-once check.
    ///
    /// # Safety
    ///
    /// The caller must not use it to bring up a pin or instance another owner already holds. The
    /// golden-trace bodies and the host tests use it; an image uses [`Chip::take`].
    pub const unsafe fn steal() -> Chip {
        Chip { _private: () }
    }

    /// The compiled chip's descriptor.
    #[inline]
    pub const fn descriptor(&self) -> &'static McuDescriptor {
        &DESCRIPTOR
    }

    /// The clock-tree marker.
    #[inline]
    pub const fn clock(&self) -> ClockPath {
        ClockPath
    }

    /// The interrupt-layout marker.
    #[inline]
    pub const fn irq(&self) -> IrqLayout {
        IrqLayout
    }

    /// The RCU base. Every chip carries one; the `Result` keeps the call shape the firmware uses.
    #[inline]
    pub const fn rcu_base(&self) -> Result<u32, DescriptorError> {
        DESCRIPTOR.addrs.resolve(PeriphLabel::Rcu)
    }

    /// The base of `label`, or `MissingBase` when the chip does not carry it.
    #[inline]
    pub(crate) const fn base(&self, label: PeriphLabel) -> Result<u32, DescriptorError> {
        DESCRIPTOR.addrs.resolve(label)
    }

    /// The factory unique ID, read from the signature block at the descriptor's `UniqueId` base
    /// (`0x1FFF_F7E8` on the GD32F10x and the MM32SPIN05, `0x1FFF_F7AC` on the GD32F1x0).
    pub fn unique_id(&self) -> UniqueId {
        const BASE: u32 = match DESCRIPTOR.addrs.get(PeriphLabel::UniqueId) {
            Some(b) => b,
            None => panic!("every descriptor carries the unique-ID base"),
        };
        UniqueId([
            Reg32::new(BASE, 0x0).read(),
            Reg32::new(BASE, 0x4).read(),
            Reg32::new(BASE, 0x8).read(),
        ])
    }

    /// The flash page size.
    #[inline]
    pub const fn flash_page(&self) -> PageSize {
        DESCRIPTOR.flash_page
    }

    /// The flash size in bytes.
    #[inline]
    pub const fn flash_size_bytes(&self) -> u32 {
        DESCRIPTOR.flash_size_bytes()
    }

    /// Whether the chip carries advanced timer `label`.
    #[inline]
    pub const fn has_advanced_timer(&self, label: PeriphLabel) -> bool {
        label.is_timer() && DESCRIPTOR.addrs.get(label).is_some()
    }

    /// Whether the chip carries I2C instance `label`.
    #[inline]
    pub const fn has_i2c(&self, label: PeriphLabel) -> bool {
        label.is_i2c() && DESCRIPTOR.addrs.get(label).is_some()
    }

    /// `packed` as a floating input with its port clock on, or refused: an SWD pin, a pin the
    /// package does not bond out, or a port the chip does not carry.
    pub fn pin(&self, packed: u8) -> Result<Pin<Input<Floating>>, DescriptorError> {
        let base = self.clocked_port_base(packed)?;
        Ok(Pin::new(base, packed))
    }

    /// The port base of `packed` with its clock on, after the SWD and bonding checks.
    #[inline(never)]
    pub(crate) fn clocked_port_base(&self, packed: u8) -> Result<u32, DescriptorError> {
        pincap::check_not_swd(packed)?;
        let port = port_label(packed)?;
        if !DESCRIPTOR.bonded(packed) {
            return Err(DescriptorError::MissingBase(port));
        }
        let base = self.base(port)?;
        crate::clock::enable_gpio_port(self.rcu_base()?, ClockPath, port)?;
        Ok(base)
    }

    /// Put pin `pin` of `port` in analog mode (an ADC input), with its port clock on.
    pub fn analog_pin(&self, port: PeriphLabel, pin: u8) -> Result<(), DescriptorError> {
        let packed = packed_pin(port, pin)?;
        pincap::check_pin_ownership(self, packed, PinClaim::NonGate)?;
        let base = self.clocked_port_base(packed)?;
        gpio::configure_analog(base, packed);
        Ok(())
    }

    /// A group of up to three input lines read together as one code (line `i` is bit `i`): the hall
    /// sensors, or the button and foot pads. Each line's port clock is enabled; the pin modes are
    /// left as configured.
    pub fn input_group(
        &self,
        pins: [u8; INPUT_GROUP_LINES],
    ) -> Result<InputGroup, DescriptorError> {
        let mut lines = [(0u32, 0u8); INPUT_GROUP_LINES];
        for (slot, &pin) in lines.iter_mut().zip(pins.iter()) {
            *slot = (self.clocked_port_base(pin)?, pin & 0x0F);
        }
        Ok(InputGroup::resolve(lines))
    }

    /// Route a gate pin (an advanced-timer channel or complementary output) to its timer.
    pub fn route_advanced_pwm_pin(&self, pin: u8) -> Result<(), DescriptorError> {
        pincap::check_pin_ownership(self, pin, PinClaim::Gate)?;
        let base = self.clocked_port_base(pin)?;
        gpio::configure_af(base, pin, PinRole::TimerAfPushPull);
        Ok(())
    }

    /// Route a general-timer PWM pin (TIMER1). On the F10x the TIMER1 outputs reach PB3 through the
    /// AFIO partial remap 0, and PB3 is a JTAG pin until the SWJ field frees it.
    ///
    /// Not on the Cortex-M0 build: there TIMER1 (the MM32's 32-bit TIM2) is the HAL's own cycle
    /// counter (`irq::cycle_count`), which the M0 has no DWT for.
    #[cfg(hal_core = "m3")]
    pub fn route_general_pwm_pin(&self, pin: u8) -> Result<(), DescriptorError> {
        pincap::check_pin_ownership(self, pin, PinClaim::NonGate)?;
        let base = self.clocked_port_base(pin)?;
        #[cfg(hal_gpio = "crl")]
        {
            let rcu = self.rcu_base()?;
            crate::afio::set_swj_sw_only(rcu);
            crate::afio::remap_timer1_partial0(rcu);
        }
        gpio::configure_af(base, pin, PinRole::GenTimerAfPushPull);
        Ok(())
    }

    /// Route an SPI pin: `SpiAfPushPull` for SCK, MOSI and a hardware NSS, `SpiInput` for MISO.
    #[cfg(hal_spi = "gd")]
    pub fn route_spi_pin(&self, pin: u8, role: PinRole) -> Result<(), DescriptorError> {
        if !matches!(role, PinRole::SpiAfPushPull | PinRole::SpiInput) {
            return Err(DescriptorError::UnsupportedRole);
        }
        pincap::check_pin_ownership(self, pin, PinClaim::NonGate)?;
        let base = self.clocked_port_base(pin)?;
        gpio::configure_af(base, pin, role);
        Ok(())
    }
}

/// The GPIO port of a packed pin byte.
pub(crate) const fn port_label(pin: u8) -> Result<PeriphLabel, DescriptorError> {
    match pin >> 4 {
        0 => Ok(PeriphLabel::Gpioa),
        1 => Ok(PeriphLabel::Gpiob),
        2 => Ok(PeriphLabel::Gpioc),
        3 => Ok(PeriphLabel::Gpiod),
        5 => Ok(PeriphLabel::Gpiof),
        _ => Err(DescriptorError::MissingBase(PeriphLabel::Gpioa)),
    }
}

/// The packed byte of pin `pin` on `port`.
const fn packed_pin(port: PeriphLabel, pin: u8) -> Result<u8, DescriptorError> {
    let index = match port {
        PeriphLabel::Gpioa => 0,
        PeriphLabel::Gpiob => 1,
        PeriphLabel::Gpioc => 2,
        PeriphLabel::Gpiod => 3,
        PeriphLabel::Gpiof => 5,
        other => return Err(DescriptorError::MissingBase(other)),
    };
    Ok((index << 4) | (pin & 0x0F))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unique_id_bytes_are_the_swd_dump_order() {
        // The rover master's ID as an SWD dump reads it, and the three words the part stores.
        let id = UniqueId([0x3239_dc8a, 0x1238_3533, 0x3333_3455]);
        assert_eq!(
            id.to_bytes(),
            [0x8a, 0xdc, 0x39, 0x32, 0x33, 0x35, 0x38, 0x12, 0x55, 0x34, 0x33, 0x33]
        );
    }

    #[test]
    fn the_unique_id_base_is_the_familys_signature_block() {
        let want = if cfg!(hal_gpio = "ctl") {
            0x1FFF_F7AC
        } else {
            0x1FFF_F7E8
        };
        assert_eq!(DESCRIPTOR.addrs.get(PeriphLabel::UniqueId), Some(want));
    }
}
