// SPDX-License-Identifier: GPL-3.0-or-later
//! The MM32SPIN05 ADC (MindMotion's `reg_adc.h` and `hal_adc.c`, LibSamples V1.13): the
//! timer-triggered group the period ISR reads and the polled single conversion. The `adc` module of
//! the MM32 build, in place of the GD32 one: the block shares no register with the GD32 ADC.
//!
//! The block has no injected group. It converts the channels whose bits are set in `ADCHS`, in
//! channel order, once per start (`ADCR.MODE` = scan), and keeps each channel's result in its own
//! data register (`ADDRn` at `0x18 + 4n`), so the group the period ISR reads is the scan triggered by
//! TIM1, and a "rank" is a channel's data register: the HAL reads the configured channels in the
//! configuration's order, whatever order the scan converted them in. One sample time
//! (`ADCFG.SAMCTL`) serves every channel. The end of the scan sets `ADSTA.ADIF` (written 1 to
//! clear), the ADC vector's source with `ADCR.ADIE`.
//!
//! | reg      | offset | fields the HAL programs                                         |
//! |----------|--------|-----------------------------------------------------------------|
//! | `ADDATA` | `0x00` | the last conversion                                             |
//! | `ADCFG`  | `0x04` | ADEN(0), TEN(2), VEN(3), PRE(6:4, 14), SAMCTL(13:10)            |
//! | `ADCR`   | `0x08` | ADIE(0), TRGEN(2), TRGSEL(6:4, 18:17), ADST(8), MODE(10:9), ALIGN(11) |
//! | `ADCHS`  | `0x0C` | one enable bit per channel; 14 the temperature sensor, 15 VREFINT |
//! | `ADSTA`  | `0x14` | ADIF(0)                                                         |
//! | `ADDRn`  | `0x18` | channel `n`'s data, `+4` per channel (`0x48`, channel 12's, is reserved) |
//!
//! The clock divider is the block's own `ADCFG.PRE` (PCLK2 / 6 for the single conversion, the
//! configuration's [`AdcClockDiv`] for the group), set by the bring-ups as MindMotion's `ADC_Init`
//! sets it; [`crate::clock::enable_adc`] only gates the clock. The part has no calibration
//! sequence in MindMotion's library, so [`Adc`] has no `calibrate` and the group's bring-up runs
//! none (specs/hal.md section 4.1: a capability the chip lacks is absent from its build).

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::config::{
    AdcClockDiv, InjectedAdcConfig, InjectedChannel, TimerTriggerLink, MAX_INJECTED_CHANNELS,
};
use crate::descriptor::DESCRIPTOR;
use crate::error::{AdcError, BringUpError, DescriptorError};
use crate::reg::{Field, Reg32};

const ADDATA: u32 = 0x00;
const ADCFG: u32 = 0x04;
const ADCR: u32 = 0x08;
const ADCHS: u32 = 0x0C;
const ADSTA: u32 = 0x14;
/// Channel 0's data register; channel `n`'s is at `ADDR0 + 4 * n`.
const ADDR0: u32 = 0x18;

const ADCFG_ADEN: u32 = 1 << 0;
const ADCFG_TEN: u32 = 1 << 2;
const ADCFG_VEN: u32 = 1 << 3;
/// `PRE` (6:4) with its extension `PREL` (14): the ADC clock divider from PCLK2. Codes 0 to 3 are
/// /2, /4, /6 and /8, the codes of [`AdcClockDiv`].
const ADCFG_PRE: u32 = (0b111 << 4) | (1 << 14);
const ADCFG_SAMCTL: u32 = 0b1111 << 10;

const ADCR_ADIE: u32 = 1 << 0;
const ADCR_TRGEN: u32 = 1 << 2;
/// `TRGSEL`: bits 6:4 and their extension 18:17.
const ADCR_TRGSEL: u32 = (0b111 << 4) | (0b11 << 17);
const ADCR_ADST: u32 = 1 << 8;
const ADCR_MODE: u32 = 0b11 << 9;
/// `MODE` = 1: one scan of the enabled channels per start.
const MODE_SCAN: u32 = 0b01 << 9;
const ADCR_ALIGN: u32 = 1 << 11;

const ADSTA_ADIF: u32 = 1 << 0;

/// The temperature sensor's channel (`ADC_Channel_TempSensor`).
const CH_TEMPERATURE: u8 = 14;
/// VREFINT's channel (`ADC_Channel_Vrefint`).
const CH_VREFINT: u8 = 15;
/// The largest sample-time code (`SAMCTL`): 0 to 7 are 1.5 to 239.5 cycles as on the GD32 parts,
/// 8 to 12 the MM32's 2.5 to 6.5.
const MAX_SAMPLE_CODE: u8 = 0x0C;

/// The trigger select (`TRGSEL`) for TIM1's TRGO (`ADC1_ExternalTrigConv_T1_TRIG`: 18:17 = 01,
/// 6:4 = 000), the trigger the proof of concept ran its 16 kHz ISR on.
pub const TRGSEL_T1_TRGO: u32 = 0b01 << 17;
/// The trigger select for TIM1's channel 4 compare (`ADC_CR_TIM1_CC4`: 18:17 = 10), the GD32
/// TIMER0 channel 3 the HAL numbers from 0.
pub const TRGSEL_T1_CC4: u32 = 0b10 << 17;

/// Core cycles between `ADEN` and the trigger enable: 10 us at 72 MHz against the datasheet's
/// 1 us typical power-on time (`tSTAB`, DS_MM32SPIN05 table 36).
pub const ADC_STAB_CYCLES: u32 = 720;

/// Wait out the converter's power-on time (the host has no converter).
#[inline(always)]
fn settle() {
    #[cfg(target_arch = "arm")]
    cortex_m::asm::delay(ADC_STAB_CYCLES);
}

/// Polls of a conversion flag before the wait gives up, as on the GD32 model.
const ADC_TIMEOUT: u32 = 100_000;

/// ADC1's base.
const ADC0_BASE: u32 = match DESCRIPTOR.addrs.get(PeriphLabel::Adc0) {
    Some(b) => b,
    None => panic!("the descriptor carries no ADC0"),
};

/// A channel with a data register: 0 to 11, the temperature sensor and VREFINT.
const fn readable(channel: u8) -> bool {
    channel <= 11 || channel == CH_TEMPERATURE || channel == CH_VREFINT
}

/// The `ADCFG` sensor enables an internal channel needs: both, as MindMotion's one call
/// (`ADC_TempSensorVrefintCmd`) sets them, for either internal channel.
const fn sensor_bits(channel: u8) -> u32 {
    match channel {
        CH_TEMPERATURE | CH_VREFINT => ADCFG_TEN | ADCFG_VEN,
        _ => 0,
    }
}

/// `ADCFG.PRE` for a divider.
const fn pre_bits(div: AdcClockDiv) -> u32 {
    (div.psc_code() << 4) & ADCFG_PRE
}

/// Channel `channel`'s data register offset.
const fn data_offset(channel: u8) -> u32 {
    ADDR0 + 4 * channel as u32
}

/// The single software-started conversion's fields: the clock divider (PCLK2 / 6), immediate
/// mode, right alignment, no trigger (`ADC_Init` with `ADC_CR_IMM`), the sample time and the sensor
/// enables (`ADC_RegularChannelConfig` and `ADC_TempSensorVrefintCmd`), then `ADEN` (`ADC_Cmd`).
const fn single_fields(channel: u8, code: u8) -> [Field; 4] {
    [
        Field {
            offset: ADCFG,
            mask: ADCFG_PRE,
            value: pre_bits(AdcClockDiv::Div6),
        },
        Field {
            offset: ADCR,
            mask: ADCR_ALIGN | ADCR_MODE | ADCR_TRGSEL | ADCR_TRGEN | ADCR_ADIE,
            value: 0,
        },
        Field {
            offset: ADCFG,
            mask: ADCFG_SAMCTL | ADCFG_TEN | ADCFG_VEN,
            value: ((code as u32) << 10) & ADCFG_SAMCTL | sensor_bits(channel),
        },
        Field {
            offset: ADCFG,
            mask: ADCFG_ADEN,
            value: ADCFG_ADEN,
        },
    ]
}

/// The triggered group's `ADCR`: scan mode, the alignment, the trigger select and its enable, and
/// the end-of-scan interrupt the period ISR runs on.
const fn group_adcr(left_aligned: bool, trgsel: u32) -> Field {
    let align = if left_aligned { ADCR_ALIGN } else { 0 };
    Field {
        offset: ADCR,
        mask: ADCR_ALIGN | ADCR_MODE | ADCR_TRGSEL | ADCR_TRGEN | ADCR_ADIE,
        value: align | MODE_SCAN | (trgsel & ADCR_TRGSEL) | ADCR_TRGEN | ADCR_ADIE,
    }
}

/// The trigger select for `cfg`'s timer and link, or `None` where the mux has no slot for it: only
/// TIM1 (`Timer0`) is wired, its TRGO ([`TRGSEL_T1_TRGO`]) and its channel 4 compare
/// ([`TRGSEL_T1_CC4`]). The name is the GD32 build's (its `ETSIC` field).
pub fn etsic_for_link(cfg: &InjectedAdcConfig) -> Option<u32> {
    if cfg.trigger_timer != PeriphLabel::Timer0 {
        return None;
    }
    Some(match cfg.trigger_link {
        TimerTriggerLink::Ch3 => TRGSEL_T1_CC4,
        TimerTriggerLink::Trgo => TRGSEL_T1_TRGO,
    })
}

// --- the polled single conversion -----------------------------------------------------------------

/// ADC1 for polled use: one software-started conversion of one channel.
#[derive(Debug, Clone, Copy)]
pub struct Adc {
    base: u32,
}

impl Chip {
    /// ADC1. No register access; its clock must be on ([`crate::clock::enable_adc`]) before a
    /// conversion.
    #[inline]
    pub const fn adc(&self) -> Adc {
        Adc { base: ADC0_BASE }
    }
}

impl Adc {
    #[inline]
    fn reg(&self, offset: u32) -> Reg32 {
        Reg32::new(self.base, offset)
    }

    /// Configure one software-started conversion of `channel` at sample-time code `sample_time`
    /// (`SAMCTL`, 0..=12) and switch the ADC on: immediate mode, right alignment, no trigger, the
    /// sensor enables for the temperature sensor (14) or VREFINT (15), and `channel` alone selected
    /// in `ADCHS` (none for a channel with no data register). No poll. A code past 12 is masked to
    /// the field.
    pub fn configure_single(&self, channel: u8, sample_time: u8) {
        for f in single_fields(channel, sample_time) {
            f.apply(self.base);
        }
        if readable(channel) {
            self.reg(ADCHS).write(1 << channel);
        }
    }

    /// One conversion of `channel`: clear `ADIF`, enable that channel alone, start (`ADST`), wait
    /// for `ADIF`, read the channel's data register, clear `ADIF` (MindMotion's polled samples clear
    /// it after the read; the clear before covers a flag left by anything else). `AdcError::Other` for a channel with no data
    /// register (12, 13, past 15), before any register access.
    pub fn read_channel(&self, channel: u8) -> Result<u16, AdcError> {
        if !readable(channel) {
            return Err(AdcError::Other);
        }
        self.reg(ADSTA).write(ADSTA_ADIF);
        self.reg(ADCHS).write(1 << channel);
        self.reg(ADCR).set_bits(ADCR_ADST);
        self.wait_eoic()?;
        let value = self.reg(data_offset(channel)).read() as u16;
        self.reg(ADSTA).write(ADSTA_ADIF);
        Ok(value)
    }

    /// Wait for the end of a conversion or scan (`ADSTA.ADIF`).
    pub fn wait_eoic(&self) -> Result<(), AdcError> {
        let sta = self.reg(ADSTA);
        let mut budget = ADC_TIMEOUT;
        while sta.read() & ADSTA_ADIF == 0 {
            budget -= 1;
            if budget == 0 {
                return Err(AdcError::Timeout);
            }
        }
        Ok(())
    }
}

// --- the timer-triggered group --------------------------------------------------------------------

/// A timer-triggered ADC group that configures into a per-period handle.
pub trait TriggeredAdc {
    /// The per-period handle the configuration resolves into.
    type Handle: Copy;

    /// Configure the group from `cfg` and return its handle.
    fn configure(&self, chip: &Chip, cfg: &InjectedAdcConfig)
        -> Result<Self::Handle, BringUpError>;
}

/// The triggered group's bring-up (the GD32 build's injected group; here the triggered scan).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InjectedAdcController;

impl InjectedAdcController {
    /// The controller; the ADC instance comes from the configuration.
    #[inline]
    pub const fn new() -> Self {
        InjectedAdcController
    }
}

/// The one sample-time code of `channels`, or `None` if they differ or one is past the field.
fn shared_sample_time(channels: &[InjectedChannel]) -> Option<u8> {
    let first = channels.first()?.sample_time;
    let same = channels.iter().all(|c| c.sample_time == first);
    (same && first <= MAX_SAMPLE_CODE).then_some(first)
}

impl TriggeredAdc for InjectedAdcController {
    type Handle = InjectedHandle;

    /// Configure `cfg.adc` as the timer-triggered scan of `cfg.channels` and switch it on: the
    /// clock divider (`cfg.clock_div`), the sample time and the sensor enables in `ADCFG`, the
    /// channel bitmap in `ADCHS`, `ADEN` and the converter's power-on time ([`ADC_STAB_CYCLES`]),
    /// then `ADCR` (scan mode, the alignment, the trigger select from [`etsic_for_link`] and its
    /// enable, the end-of-scan interrupt), then `ADST` and `ADIF` cleared.
    ///
    /// On the bench MM32SPIN05 the write that enables the trigger (TRGO = TIM1's channel 4
    /// compare, the counter not yet started) started a scan at once: right after the bring-up
    /// `ADCR.ADST` read 1 and `ADSTA` showed a finished scan, and with `ADST` left set the block
    /// took no trigger again, `ADIF` never rose and the data froze
    /// (`specs/bench-work/mm32/hal-step3/logs/v08-adc-live.txt`, `v16-motor-verify-adc-trace.txt`).
    /// A write of `ADCR` with `ADST` clear restarted it, after which it converted once per trigger
    /// with `ADST` reading 0 (`v11`, `v12`, `v14`). Hence the clear.
    ///
    /// Refused before any register access: a label that is not an ADC (`Unsupported`), one the
    /// chip does not carry (`MissingBase`), no channel, a channel with no data register, a channel
    /// named twice, sample times that differ between channels (the block has one) or a code past
    /// 12, or a trigger timer the mux has no slot for (all `AdcError::Other`). The ADC clock must
    /// already be on.
    fn configure(
        &self,
        chip: &Chip,
        cfg: &InjectedAdcConfig,
    ) -> Result<InjectedHandle, BringUpError> {
        if !cfg.adc.is_adc() {
            return Err(DescriptorError::Unsupported.into());
        }
        let base = chip.base(cfg.adc)?;
        let mut bitmap = 0u32;
        for c in cfg.channels.iter() {
            let bit = 1u32 << (c.channel & 0x1F);
            if !readable(c.channel) || bitmap & bit != 0 {
                return Err(AdcError::Other.into());
            }
            bitmap |= bit;
        }
        let code = shared_sample_time(&cfg.channels).ok_or(AdcError::Other)?;
        let trgsel = etsic_for_link(cfg).ok_or(AdcError::Other)?;
        let sensors = cfg
            .channels
            .iter()
            .fold(0, |s, c| s | sensor_bits(c.channel));

        Field {
            offset: ADCFG,
            mask: ADCFG_PRE | ADCFG_SAMCTL | ADCFG_TEN | ADCFG_VEN,
            value: pre_bits(cfg.clock_div) | (u32::from(code) << 10) | sensors,
        }
        .apply(base);
        Reg32::new(base, ADCHS).write(bitmap);
        // The converter on, and its power-on time, before the trigger is enabled.
        Reg32::new(base, ADCFG).set_bits(ADCFG_ADEN);
        settle();
        group_adcr(cfg.left_aligned, trgsel).apply(base);
        // The scan the trigger enable may have started: ADST cleared, its ADIF with it.
        let adcr = Reg32::new(base, ADCR);
        adcr.write(adcr.read() & !ADCR_ADST);
        Reg32::new(base, ADSTA).write(ADSTA_ADIF);
        Ok(InjectedHandle::new(base, &cfg.channels))
    }
}

/// The period ISR's ADC surface: the configured channels' data registers in configuration order,
/// the flag it clears and the trigger enable it re-asserts, resolved once at bring-up.
#[derive(Debug, Clone, Copy)]
pub struct InjectedHandle {
    data: [Reg32; MAX_INJECTED_CHANNELS],
    sta: Reg32,
    adcr: Reg32,
}

impl InjectedHandle {
    #[inline]
    fn new(base: u32, channels: &[InjectedChannel]) -> Self {
        let mut data = [Reg32::new(base, ADDATA); MAX_INJECTED_CHANNELS];
        for (slot, c) in data.iter_mut().zip(channels.iter()) {
            *slot = Reg32::new(base, data_offset(c.channel));
        }
        Self {
            data,
            sta: Reg32::new(base, ADSTA),
            adcr: Reg32::new(base, ADCR),
        }
    }

    /// Clear the end-of-scan flag (`ADSTA.ADIF`, a written 1 clears it; a plain write, so no other
    /// flag is touched). It is a level source into the NVIC, so the period ISR clears it or
    /// re-enters at once.
    #[inline]
    pub fn clear_eoic(&self) {
        self.sta.write(ADSTA_ADIF);
    }

    /// Set the trigger enable (`ADCR.TRGEN`) again, the GD32 build's per-period re-assert: here a
    /// read, and a write only if `TRGEN` is clear or `ADST` set (the stuck start the bring-up
    /// clears, [`TriggeredAdc::configure`]). The write clears `ADST`: a blind read-modify-write
    /// would write back an `ADST` that reads 1.
    #[inline]
    pub fn reassert_trigger_enable(&self) {
        let v = self.adcr.read();
        if v & (ADCR_TRGEN | ADCR_ADST) != ADCR_TRGEN {
            self.adcr.write((v | ADCR_TRGEN) & !ADCR_ADST);
        }
    }

    /// Whether the scan has completed since the flag was last cleared.
    #[inline]
    pub fn eoic(&self) -> bool {
        self.sta.read() & ADSTA_ADIF != 0
    }

    /// The configured channels' data in configuration order (alignment included). A slot past the
    /// configured length reads `ADDATA`, the last conversion.
    #[inline]
    pub fn read_injected(&self) -> [u16; MAX_INJECTED_CHANNELS] {
        let mut out = [0u16; MAX_INJECTED_CHANNELS];
        for (slot, reg) in out.iter_mut().zip(self.data.iter()) {
            *slot = reg.read() as u16;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reg::fold;

    fn group(list: &[(u8, u8)], link: TimerTriggerLink) -> InjectedAdcConfig {
        InjectedAdcConfig {
            adc: PeriphLabel::Adc0,
            channels: list
                .iter()
                .map(|&(channel, sample_time)| InjectedChannel {
                    channel,
                    sample_time,
                })
                .collect(),
            left_aligned: true,
            trigger_timer: PeriphLabel::Timer0,
            trigger_link: link,
            clock_div: AdcClockDiv::Div6,
        }
    }

    #[test]
    fn adc1_is_at_its_base() {
        assert_eq!(ADC0_BASE, 0x4001_2400);
    }

    #[test]
    fn data_registers_follow_the_vendor_map() {
        assert_eq!(data_offset(0), 0x18);
        assert_eq!(data_offset(11), 0x44);
        assert_eq!(data_offset(CH_TEMPERATURE), 0x50);
        assert_eq!(data_offset(CH_VREFINT), 0x54);
        assert!(!readable(12) && !readable(13) && !readable(16));
    }

    #[test]
    fn the_trigger_is_tim1s_trgo_or_its_channel_4() {
        assert_eq!(
            etsic_for_link(&group(&[(2, 1)], TimerTriggerLink::Trgo)),
            Some(1 << 17)
        );
        assert_eq!(
            etsic_for_link(&group(&[(2, 1)], TimerTriggerLink::Ch3)),
            Some(2 << 17)
        );
        let mut other = group(&[(2, 1)], TimerTriggerLink::Trgo);
        other.trigger_timer = PeriphLabel::Timer7;
        assert_eq!(etsic_for_link(&other), None);
    }

    #[test]
    fn the_group_runs_a_triggered_scan_with_its_interrupt() {
        let regs = fold(&[group_adcr(true, TRGSEL_T1_TRGO)]);
        // ADIE | TRGEN | TRGSEL T1_TRIG | MODE scan | ALIGN left.
        assert_eq!(regs[&ADCR], 1 | (1 << 2) | (1 << 17) | (1 << 9) | (1 << 11));
    }

    #[test]
    fn one_sample_time_serves_every_channel() {
        let same = group(&[(2, 3), (6, 3), (9, 3)], TimerTriggerLink::Trgo);
        assert_eq!(shared_sample_time(&same.channels), Some(3));
        let differ = group(&[(2, 3), (6, 4)], TimerTriggerLink::Trgo);
        assert_eq!(shared_sample_time(&differ.channels), None);
        let past = group(&[(2, 13)], TimerTriggerLink::Trgo);
        assert_eq!(shared_sample_time(&past.channels), None);
    }

    #[test]
    fn a_single_conversion_is_immediate_mode_with_the_sensor_it_needs() {
        let regs = fold(&single_fields(CH_VREFINT, 7));
        assert_eq!(regs[&ADCR], 0);
        let pre6 = 2 << 4;
        assert_eq!(
            regs[&ADCFG],
            ADCFG_ADEN | ADCFG_TEN | ADCFG_VEN | (7 << 10) | pre6
        );
        let regs = fold(&single_fields(3, 2));
        assert_eq!(regs[&ADCFG], ADCFG_ADEN | (2 << 10) | pre6);
    }
}
