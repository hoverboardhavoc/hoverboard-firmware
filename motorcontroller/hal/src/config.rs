// SPDX-License-Identifier: GPL-3.0-or-later
//! The typed configuration values the firmware passes to bring-up calls.
//!
//! Application wiring (pins, baud rates, timer periods, channels) lives here, not in the chip
//! descriptor (specs/hal.md, section 3.1). Pins are packed bytes: `(port << 4) | pin`, port A = 0,
//! B = 1, C = 2, D = 3, F = 5.

use heapless::Vec;

use crate::addr::PeriphLabel;

/// Most regular-group channels an [`AdcConfig`] carries.
pub const MAX_ADC_CHANNELS: usize = 8;
/// Most injected-group channels (the F1 injected sequence is four ranks).
pub const MAX_INJECTED_CHANNELS: usize = 4;
/// Complementary channel pairs on an advanced timer that the PWM bring-up drives.
pub const MAX_PWM_CHANNELS: usize = 3;

/// `(port, pin)` of a packed pin byte.
#[inline]
pub const fn decode_pin(pin: u8) -> (u8, u8) {
    (pin >> 4, pin & 0x0F)
}

/// USART word length.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum WordLen {
    #[default]
    Eight = 0,
    Nine = 1,
}

/// USART parity.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Parity {
    #[default]
    None = 0,
    Even = 1,
    Odd = 2,
}

/// USART stop bits (the `CTL1.STB` code).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum StopBits {
    #[default]
    One = 0,
    Half = 1,
    Two = 2,
    OneAndHalf = 3,
}

/// USART oversampling.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Oversampling {
    #[default]
    By16 = 0,
    By8 = 1,
}

/// A USART frame format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct UsartFrame {
    pub word_len: WordLen,
    pub parity: Parity,
    pub stop: StopBits,
}

impl UsartFrame {
    /// 8 data bits, no parity, one stop bit.
    pub const EIGHT_N_ONE: UsartFrame = UsartFrame {
        word_len: WordLen::Eight,
        parity: Parity::None,
        stop: StopBits::One,
    };
}

/// A USART bring-up request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub struct UsartConfig {
    pub usart: PeriphLabel,
    pub baud: u32,
    pub frame: UsartFrame,
    pub oversampling: Oversampling,
}

/// SPI chip-select handling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum NssMode {
    #[default]
    Software,
    Hardware,
}

/// An SPI bring-up request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiConfig {
    /// The instance.
    pub spi: PeriphLabel,
    /// Clock pin (packed).
    pub sck: u8,
    /// Controller-in pin (packed).
    pub miso: u8,
    /// Controller-out pin (packed).
    pub mosi: u8,
    /// Chip-select pin (packed), routed only in [`NssMode::Hardware`].
    pub nss: u8,
    /// SPI mode 0..3 (`CPOL << 1 | CPHA`).
    pub mode: u8,
    /// 16-bit frames instead of 8.
    pub data16: bool,
    /// The highest SCK frequency the device takes; the prescaler is the smallest that stays at or
    /// under it.
    pub target_hz: u32,
    /// LSB first.
    pub lsb_first: bool,
    /// Chip-select handling.
    pub nss_mode: NssMode,
}

/// The ADC clock divider from PCLK2 (`RCU_CFG0.ADCPSC`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum AdcClockDiv {
    Div2 = 0,
    Div4 = 1,
    #[default]
    Div6 = 2,
    Div8 = 3,
}

impl AdcClockDiv {
    /// The `ADCPSC` field code.
    #[inline]
    pub const fn psc_code(self) -> u32 {
        self as u32
    }
}

/// One regular-group channel and its sample-time code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub struct AdcChannel {
    pub channel: u8,
    pub sample_time: u8,
}

/// A regular-group ADC bring-up request.
#[derive(Debug, Clone, PartialEq, Eq)]
#[allow(missing_docs)]
pub struct AdcConfig {
    pub adc: PeriphLabel,
    pub channels: Vec<AdcChannel, MAX_ADC_CHANNELS>,
    pub left_aligned: bool,
    pub clock_div: AdcClockDiv,
}

impl AdcConfig {
    /// The group includes the temperature sensor (16) or VREFINT (17), which need `TSVREN`.
    #[inline]
    pub fn needs_internal_enable(&self) -> bool {
        self.channels
            .iter()
            .any(|c| c.channel == 16 || c.channel == 17)
    }
}

/// Advanced-timer counter alignment.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum PwmAlign {
    EdgeUp = 0,
    EdgeDown = 1,
    Center1 = 2,
    #[default]
    Center2 = 3,
    Center3 = 4,
}

impl PwmAlign {
    /// The `CTL0` `DIR`/`CAM` bits.
    #[inline]
    pub const fn ctl0_bits(self) -> u32 {
        match self {
            PwmAlign::EdgeUp => 0,
            PwmAlign::EdgeDown => 1 << 4,
            PwmAlign::Center1 => 1 << 5,
            PwmAlign::Center2 => 2 << 5,
            PwmAlign::Center3 => 3 << 5,
        }
    }
}

/// Output-compare PWM mode.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum OcMode {
    #[default]
    Pwm0 = 0,
    Pwm1 = 1,
}

impl OcMode {
    /// The `CHxCOMCTL` code.
    #[inline]
    pub const fn comctl(self) -> u32 {
        match self {
            OcMode::Pwm0 => 0b110,
            OcMode::Pwm1 => 0b111,
        }
    }
}

/// The dead-time and digital-filter clock division (`CTL0.CKDIV`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum ClockDiv {
    Div1 = 0,
    #[default]
    Div2 = 1,
    Div4 = 2,
}

impl ClockDiv {
    /// The `CKDIV` code.
    #[inline]
    pub const fn ckdiv_code(self) -> u32 {
        self as u32
    }
}

/// The master-mode TRGO source (`CTL1.MMC`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum TrgoSource {
    Reset = 0,
    Enable = 1,
    #[default]
    Update = 2,
    Ch0Pulse = 3,
    Ch0Compare = 4,
    Ch1Compare = 5,
    Ch2Compare = 6,
    Ch3Compare = 7,
}

impl TrgoSource {
    /// The `MMC` code.
    #[inline]
    pub const fn mmc(self) -> u32 {
        self as u32
    }
}

/// The break input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub struct BreakConfig {
    pub enabled: bool,
    /// Active level: `true` = high.
    pub level: bool,
}

/// One complementary channel pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmChannelConfig {
    /// Output polarity: `true` = active high.
    pub polarity: bool,
    /// The high-side idle level (`ISOx`).
    pub idle_high: bool,
    /// The low-side idle level (`ISOxN`).
    pub idle_high_n: bool,
}

/// An advanced-timer PWM bring-up request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub struct PwmConfig {
    pub timer: PeriphLabel,
    pub channels: [PwmChannelConfig; MAX_PWM_CHANNELS],
    pub period: u16,
    pub prescaler: u16,
    /// The `CCHP.DTCFG` dead-time code.
    pub dead_time: u8,
    pub brk: BreakConfig,
    /// The channel 3 compare that triggers the ADC.
    pub trigger_compare: u16,
    pub align: PwmAlign,
    /// Auto-reload shadow (`CTL0.ARSE`).
    pub arse: bool,
    pub trigger_oc_mode: OcMode,
    pub trigger_ch_enable: bool,
    /// The repetition counter.
    pub crep: u8,
    pub ckdiv: ClockDiv,
    pub trgo_src: TrgoSource,
}

/// One injected-group channel and its sample-time code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub struct InjectedChannel {
    pub channel: u8,
    pub sample_time: u8,
}

/// How the trigger timer starts the injected group.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerTriggerLink {
    /// The timer's channel 3 compare event.
    Ch3 = 0,
    /// The timer's TRGO.
    Trgo = 1,
}

/// An injected-group ADC bring-up request.
#[derive(Debug, Clone, PartialEq, Eq)]
#[allow(missing_docs)]
pub struct InjectedAdcConfig {
    pub adc: PeriphLabel,
    pub channels: Vec<InjectedChannel, MAX_INJECTED_CHANNELS>,
    pub left_aligned: bool,
    pub trigger_timer: PeriphLabel,
    pub trigger_link: TimerTriggerLink,
    pub clock_div: AdcClockDiv,
}

/// ADC0 and ADC1 in inserted parallel mode (the F10x ADC model only; GD32F10x User Manual Rev2.3
/// section 11.5.3): ADC0's injected group as `primary` describes it, triggered by its timer, and
/// ADC1's converting with it, rank by rank. `secondary[r]` is ADC1's channel for rank `r`, sampled
/// at the same instant and for the same sample time as `primary.channels[r]` (the manual's note 2:
/// exactly the same sampling time on the two channels sampled together).
#[cfg(hal_adc = "f10x")]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InjectedParallelConfig {
    /// ADC0's group: its channels, alignment and trigger. `adc` must be `Adc0`.
    pub primary: InjectedAdcConfig,
    /// ADC1's channel at each of the primary's ranks, as many as it has.
    pub secondary: Vec<u8, MAX_INJECTED_CHANNELS>,
}
