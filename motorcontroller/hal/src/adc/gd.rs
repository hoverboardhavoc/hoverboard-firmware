// SPDX-License-Identifier: GPL-3.0-or-later
//! The GD32 ADC: one register model for the regular and injected groups on every GD32 this HAL
//! drives (`gd32f10x_adc.h` and `gd32f1x0_adc.h` give the same offsets and bits). The `f10x` and
//! `f1x0` models of [`crate::adc`] differ only in the sync mode: the F10x ADC0 carries `CTL0.SYNCM`
//! and a second instance to pair with, so `DualAdc` and `Chip::dual_adc` exist on the `f10x` model
//! and are absent from the F1x0 build (specs/hal.md, section 4.1).
//!
//! | reg      | offset | fields the HAL programs                                              |
//! |----------|--------|----------------------------------------------------------------------|
//! | `STAT`   | `0x00` | EOC(1), EOIC(2)                                                      |
//! | `CTL0`   | `0x04` | EOICIE(7), SM(8), SYNCM(19:16) on the F10x ADC0                      |
//! | `CTL1`   | `0x08` | ADCON(0), CTN(1), CLB(2), RSTCLB(3), DAL(11), ETSIC(14:12), ETEIC(15), ETSRC(19:17), ETERC(20), SWRCST(22), TSVREN(23) |
//! | `SAMPT0` | `0x0C` | sample time, channels 10..17, 3 bits each                            |
//! | `SAMPT1` | `0x10` | sample time, channels 0..9                                           |
//! | `RSQ0`   | `0x2C` | RL(23:20), the regular sequence length less one                      |
//! | `RSQ2`   | `0x34` | regular rank 0's channel (4:0)                                       |
//! | `ISQ`    | `0x38` | IL(21:20), the injected length less one; the rank channels, 5 bits each |
//! | `IDATAx` | `0x3C` | injected data, `+4` per rank to `0x48`                               |
//! | `RDATA`  | `0x4C` | regular data; on the F10x ADC0 in sync mode, ADC1's result in 31:16  |
//!
//! The regular group runs two ways: polled ([`Adc::read_channel`]) and split
//! ([`Adc::start_regular`], then [`Adc::regular_result`] on a later pass), the second for a
//! conversion the caller does not wait for (the 12-FET's battery on ADC1, `twelve-fet.md` section
//! 5.4).

use super::{InjectedAdcController, TriggeredAdc, ADC0_BASE, ADC_TIMEOUT};
use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::config::{InjectedAdcConfig, InjectedChannel, TimerTriggerLink, MAX_INJECTED_CHANNELS};
#[cfg(hal_adc = "f10x")]
use crate::descriptor::DESCRIPTOR;
use crate::error::{AdcError, BringUpError, DescriptorError};
use crate::reg::{Field, Reg32};

const STAT: u32 = 0x00;
const CTL0: u32 = 0x04;
const CTL1: u32 = 0x08;
const SAMPT0: u32 = 0x0C;
const SAMPT1: u32 = 0x10;
const RSQ0: u32 = 0x2C;
const RSQ2: u32 = 0x34;
const ISQ: u32 = 0x38;
const IDATA: [u32; MAX_INJECTED_CHANNELS] = [0x3C, 0x40, 0x44, 0x48];
const RDATA: u32 = 0x4C;

const STAT_EOC: u32 = 1 << 1;
const STAT_EOIC: u32 = 1 << 2;

const CTL0_EOICIE: u32 = 1 << 7;
const CTL0_SM: u32 = 1 << 8;

const CTL1_ADCON: u32 = 1 << 0;
const CTL1_CTN: u32 = 1 << 1;
const CTL1_CLB: u32 = 1 << 2;
const CTL1_RSTCLB: u32 = 1 << 3;
const CTL1_DAL: u32 = 1 << 11;
const CTL1_ETSIC: u32 = 0b111 << 12;
const CTL1_ETEIC: u32 = 1 << 15;
const CTL1_ETSRC: u32 = 0b111 << 17;
const CTL1_ETERC: u32 = 1 << 20;
const CTL1_SWRCST: u32 = 1 << 22;
const CTL1_TSVREN: u32 = 1 << 23;

/// `ETSRC = 7`: no hardware trigger, the regular group starts on `SWRCST` (the vendor's
/// `EXTTRIG_REGULAR_NONE`). The block still needs `ETERC` set for the software trigger to start it.
const ETSRC_SOFTWARE: u32 = 0b111 << 17;

const RSQ0_RL: u32 = 0b1111 << 20;
const ISQ_IL: u32 = 0b11 << 20;
/// A rank's channel field in `RSQx`/`ISQ`.
const RANK_FIELD: u32 = 0x1F;
/// A channel's sample-time field in `SAMPTx`.
const SAMPT_FIELD: u32 = 0b111;
/// The highest channel: 16 is the temperature sensor, 17 VREFINT.
const MAX_CHANNEL: u8 = 17;

/// The injected trigger select (`ETSIC`) for TIMER0's TRGO, the same code on both models
/// (`ADC0_1_EXTTRIG_INSERTED_T0_TRGO`, `ADC_EXTTRIG_INSERTED_T0_TRGO`).
pub const ETSIC_T0_TRGO: u32 = 0;
/// The injected trigger select for TIMER0's channel 3 compare (`..._INSERTED_T0_CH3`).
pub const ETSIC_T0_CH3: u32 = 1;

/// ADC1's base, the sync-mode slave.
#[cfg(hal_adc = "f10x")]
const ADC1_BASE: u32 = match DESCRIPTOR.addrs.get(PeriphLabel::Adc1) {
    Some(b) => b,
    None => panic!("an f10x-model ADC chip carries ADC1"),
};

// --- fields, as values --------------------------------------------------------------------------

/// Channels 16 and 17 need `TSVREN`.
const fn is_internal(channel: u8) -> bool {
    channel == 16 || channel == 17
}

/// `channel`'s sample-time field (`SAMPT1` for 0..9, `SAMPT0` for 10..17), or `None` past 17.
const fn sample_time(channel: u8, code: u8) -> Option<Field> {
    let (offset, shift) = if channel < 10 {
        (SAMPT1, 3 * channel as u32)
    } else if channel <= MAX_CHANNEL {
        (SAMPT0, 3 * (channel as u32 - 10))
    } else {
        return None;
    };
    Some(Field {
        offset,
        mask: SAMPT_FIELD << shift,
        value: (code as u32 & SAMPT_FIELD) << shift,
    })
}

/// Regular rank 0's channel (`RSQ2` 4:0).
const fn regular_rank0(channel: u8) -> Field {
    Field {
        offset: RSQ2,
        mask: RANK_FIELD,
        value: channel as u32 & RANK_FIELD,
    }
}

/// `ADCON` set. The first write powers the converter; a later write of 1 with no other `CTL1` bit
/// changing starts a regular conversion.
const ADC_ON: Field = Field {
    offset: CTL1,
    mask: CTL1_ADCON,
    value: CTL1_ADCON,
};

/// The regular group's single software-triggered conversion of `channel` at sample-time `code`,
/// the vendor's polled recipe in its order: scan off, continuous off, right alignment, length 1,
/// rank 0 and the sample time, `TSVREN` for an internal channel, the software trigger source and
/// its enable, then `ADCON`.
const fn single_fields(channel: u8, code: u8) -> [Option<Field>; 6] {
    let tsvren = if is_internal(channel) { CTL1_TSVREN } else { 0 };
    [
        Some(Field {
            offset: CTL0,
            mask: CTL0_SM,
            value: 0,
        }),
        Some(Field {
            offset: CTL1,
            mask: CTL1_CTN | CTL1_DAL | tsvren | CTL1_ETSRC | CTL1_ETERC,
            value: tsvren | ETSRC_SOFTWARE | CTL1_ETERC,
        }),
        Some(Field {
            offset: RSQ0,
            mask: RSQ0_RL,
            value: 0,
        }),
        Some(regular_rank0(channel)),
        sample_time(channel, code),
        Some(ADC_ON),
    ]
}

/// `ISQ` for the injected ranks `channels` (1..=4): `IL = len - 1` and each rank's channel in the
/// vendor's packing, which right-aligns the sequence so the last rank always sits at 19:15
/// (`shift = 15 - (IL - rank) * 5`: a two-rank group packs rank 0 at 14:10 and rank 1 at 19:15).
const fn injected_sequence(channels: &[InjectedChannel]) -> Field {
    let il = channels.len() as u32 - 1;
    let mut mask = ISQ_IL;
    let mut value = il << 20;
    let mut rank = 0;
    while rank < channels.len() {
        let shift = 15 - (il - rank as u32) * 5;
        mask |= RANK_FIELD << shift;
        value |= (channels[rank].channel as u32 & RANK_FIELD) << shift;
        rank += 1;
    }
    Field {
        offset: ISQ,
        mask,
        value,
    }
}

/// The injected group's `CTL0`: scan mode for more than one rank (a multi-rank group converts
/// only rank 0 with `SM` clear), and the end-of-conversion interrupt the period ISR runs on.
const fn injected_ctl0(len: usize) -> Field {
    let sm = if len > 1 { CTL0_SM } else { 0 };
    Field {
        offset: CTL0,
        mask: CTL0_SM | CTL0_EOICIE,
        value: sm | CTL0_EOICIE,
    }
}

/// The injected group's `CTL1`: the alignment, `TSVREN` when a rank is internal (otherwise left
/// as it is), the trigger select `etsic` and the trigger enable.
const fn injected_ctl1(left_aligned: bool, internal: bool, etsic: u32) -> Field {
    let dal = if left_aligned { CTL1_DAL } else { 0 };
    let tsvren = if internal { CTL1_TSVREN } else { 0 };
    Field {
        offset: CTL1,
        mask: CTL1_DAL | tsvren | CTL1_ETSIC | CTL1_ETEIC,
        value: dal | tsvren | ((etsic << 12) & CTL1_ETSIC) | CTL1_ETEIC,
    }
}

/// The injected trigger select (`ETSIC`) for `cfg`'s trigger timer and link, or `None` where the
/// mux has no slot for it: only TIMER0 reaches ADC0's injected trigger on these parts (TIMER0's
/// TRGO = [`ETSIC_T0_TRGO`], its channel 3 compare = [`ETSIC_T0_CH3`]), so a TIMER7 gate set has
/// no injected trigger here.
pub fn etsic_for_link(cfg: &InjectedAdcConfig) -> Option<u32> {
    if cfg.trigger_timer != PeriphLabel::Timer0 {
        return None;
    }
    Some(match cfg.trigger_link {
        TimerTriggerLink::Ch3 => ETSIC_T0_CH3,
        TimerTriggerLink::Trgo => ETSIC_T0_TRGO,
    })
}

// --- the regular group ----------------------------------------------------------------------------

/// An ADC instance for polled use: the regular group's single conversion, the calibration, and a
/// polled read of the injected data.
#[derive(Debug, Clone, Copy)]
pub struct Adc {
    base: u32,
}

impl Chip {
    /// ADC0. No register access; its clock must be on ([`crate::clock::enable_adc`]) before a
    /// conversion.
    #[inline]
    pub const fn adc(&self) -> Adc {
        Adc { base: ADC0_BASE }
    }

    /// ADC instance `label`, from the descriptor. No register access; its clock must be on
    /// ([`crate::clock::enable_adc`]) before a conversion. Refused: a label that is not an ADC
    /// (`Unsupported`), or one the chip does not carry (`MissingBase`).
    #[inline]
    pub const fn adc_instance(&self, label: PeriphLabel) -> Result<Adc, DescriptorError> {
        if !label.is_adc() {
            return Err(DescriptorError::Unsupported);
        }
        match self.base(label) {
            Ok(base) => Ok(Adc { base }),
            Err(e) => Err(e),
        }
    }

    /// ADC0 and ADC1 as the regular-simultaneous pair (the F10x ADC model only).
    #[cfg(hal_adc = "f10x")]
    #[inline]
    pub const fn dual_adc(&self) -> DualAdc {
        DualAdc {
            primary: Adc { base: ADC0_BASE },
            secondary: Adc { base: ADC1_BASE },
        }
    }
}

impl Adc {
    #[inline]
    fn reg(&self, offset: u32) -> Reg32 {
        Reg32::new(self.base, offset)
    }

    /// Configure the regular group for one software-triggered conversion of `channel` at
    /// sample-time code `sample_time` (`ADC_SAMPLETIME_*`, 0..=7), and switch the ADC on. No poll:
    /// calibration is [`Adc::calibrate`]. A channel past 17 gets no sample time.
    pub fn configure_single(&self, channel: u8, sample_time: u8) {
        for f in single_fields(channel, sample_time).into_iter().flatten() {
            f.apply(self.base);
        }
    }

    /// Calibrate (`adc_calibration_enable`): set `RSTCLB` and wait for it to clear, then set `CLB`
    /// and wait for it to clear. [`AdcError::Timeout`] if either bit stays set.
    pub fn calibrate(&self) -> Result<(), AdcError> {
        let ctl1 = self.reg(CTL1);
        ctl1.set_bits(CTL1_RSTCLB);
        self.wait_ctl1_clear(CTL1_RSTCLB)?;
        ctl1.set_bits(CTL1_CLB);
        self.wait_ctl1_clear(CTL1_CLB)
    }

    /// One conversion of `channel`: clear `EOC`, point regular rank 0 at it, start the conversion
    /// on a fresh `SWRCST` edge, wait for `EOC`, read `RDATA`. The channel's sample time is the one
    /// [`Adc::configure_single`] set.
    ///
    /// Both steps before the start are measured on silicon (the rover master, a GD32F103C8,
    /// 2026-11-01, `bench-evidence/2026-11-01/hal-step2/logs/t01-adc-diag-f103.txt`): there
    /// `SWRCST` still reads 1 after the conversion it started, so a second `set` makes no edge
    /// and starts nothing, and reading `RDATA` leaves `EOC` set, so the wait passes at once and
    /// every later read returns the first conversion (VREFINT and the temperature sensor both read
    /// 1671). The `EOC` clear is a plain write of every flag bit but `EOC`: the flags clear on a
    /// written 0, so a read-modify-write could clear an `EOIC` raised between its read and its
    /// write (`tools/golden-trace` `adc/transfer_*`).
    pub fn read_channel(&self, channel: u8) -> Result<u16, AdcError> {
        self.clear_eoc();
        regular_rank0(channel).apply(self.base);
        self.software_trigger();
        self.wait_stat(STAT_EOC)?;
        Ok(self.reg(RDATA).read() as u16)
    }

    /// Start one conversion of the regular group as [`Adc::configure_single`] left it, and return
    /// without waiting: clear `EOC`, then a fresh `SWRCST` edge, the two steps
    /// [`Adc::read_channel`] takes before its start and for the same measured reasons. The result
    /// is [`Adc::regular_result`]'s once the conversion ends (at 239.5 cycles and the /6 ADC clock,
    /// 252 ADC clocks, 21 us).
    #[inline]
    pub fn start_regular(&self) {
        self.clear_eoc();
        self.software_trigger();
    }

    /// The regular group's result (`RDATA`) when a conversion has ended since the last
    /// [`Adc::start_regular`] (`STAT.EOC` set), or `None` while it runs or before any start. No
    /// wait. Reading `RDATA` leaves `EOC` set on silicon ([`Adc::read_channel`]), so a second call
    /// before the next start returns the same result.
    #[inline]
    pub fn regular_result(&self) -> Option<u16> {
        if self.reg(STAT).read() & STAT_EOC == 0 {
            return None;
        }
        Some(self.reg(RDATA).read() as u16)
    }

    /// Clear `STAT.EOC` with a plain write of every flag bit but `EOC` (the flags clear on a
    /// written 0, so a read-modify-write could clear an `EOIC` raised between its read and its
    /// write).
    #[inline]
    fn clear_eoc(&self) {
        self.reg(STAT).write(!STAT_EOC);
    }

    /// A fresh `SWRCST` edge: clear, then set (`adc_software_trigger_enable`). `SWRCST` can still
    /// read 1 after the conversion it started, and a set over a set makes no edge.
    #[inline]
    fn software_trigger(&self) {
        let ctl1 = self.reg(CTL1);
        ctl1.clear_bits(CTL1_SWRCST);
        ctl1.set_bits(CTL1_SWRCST);
    }

    /// Wait for the injected group's end of conversion (`STAT.EOIC`).
    pub fn wait_eoic(&self) -> Result<(), AdcError> {
        self.wait_stat(STAT_EOIC)
    }

    /// Injected data register `n` (`IDATA0`..`IDATA3`, `n` taken mod 4).
    #[inline]
    pub fn read_injected_data(&self, n: u8) -> u16 {
        self.reg(IDATA[(n & 0x3) as usize]).read() as u16
    }

    fn wait_ctl1_clear(&self, bit: u32) -> Result<(), AdcError> {
        let ctl1 = self.reg(CTL1);
        let mut budget = ADC_TIMEOUT;
        while ctl1.read() & bit != 0 {
            budget -= 1;
            if budget == 0 {
                return Err(AdcError::Timeout);
            }
        }
        Ok(())
    }

    fn wait_stat(&self, flag: u32) -> Result<(), AdcError> {
        let stat = self.reg(STAT);
        let mut budget = ADC_TIMEOUT;
        while stat.read() & flag == 0 {
            budget -= 1;
            if budget == 0 {
                return Err(AdcError::Timeout);
            }
        }
        Ok(())
    }
}

// --- the dual-ADC regular-simultaneous mode (F10x) ------------------------------------------------

/// `SYNCM = 6`, regular parallel (`ADC_DAUL_REGULAL_PARALLEL`, the vendor's spelling): ADC0 and
/// ADC1 convert their regular sequences at the same instant.
#[cfg(hal_adc = "f10x")]
const SYNCM_REGULAR_PARALLEL: Field = Field {
    offset: CTL0,
    mask: 0xF << 16,
    value: 6 << 16,
};

/// ADC0 (the master) and ADC1 (the slave) in regular-simultaneous mode. `SYNCM` is ADC0's field
/// only ("these bits are only used in ADC0", GD32F10x User Manual, `ADC_CTL0`).
#[cfg(hal_adc = "f10x")]
#[derive(Debug, Clone, Copy)]
pub struct DualAdc {
    primary: Adc,
    secondary: Adc,
}

#[cfg(hal_adc = "f10x")]
impl DualAdc {
    /// Configure ADC0 for `primary_channel` and ADC1 for `secondary_channel`, each as
    /// [`Adc::configure_single`] at the shared `sample_time` (the manual asks for one sample time
    /// on channels sampled together), then set ADC0's `SYNCM` to regular parallel. No poll.
    pub fn configure_simultaneous(
        &self,
        primary_channel: u8,
        secondary_channel: u8,
        sample_time: u8,
    ) {
        self.primary.configure_single(primary_channel, sample_time);
        self.secondary
            .configure_single(secondary_channel, sample_time);
        SYNCM_REGULAR_PARALLEL.apply(self.primary.base);
    }
}

// --- the timer-triggered injected group -----------------------------------------------------------

impl TriggeredAdc for InjectedAdcController {
    type Handle = InjectedHandle;

    /// Configure `cfg.adc`'s injected group, calibrate, and switch it on: the rank sequence and
    /// sample times, scan mode for more than one rank, the alignment, `TSVREN` for an internal
    /// channel, the timer trigger select ([`etsic_for_link`]) and its enable, the end-of-conversion
    /// interrupt enable, `ADCON`, the calibration, and `ADCON` again (the calibration writes other
    /// `CTL1` bits, so the converter needs the second write to start).
    ///
    /// Refused before any register access: a label that is not an ADC (`Unsupported`), one the
    /// chip does not carry (`MissingBase`), no ranks, a channel past 17 or a sample-time code past 7,
    /// or a trigger timer the mux has no slot for (`AdcError::Other`). [`AdcError::Timeout`] if the
    /// calibration does not finish. The ADC clock must already be on.
    fn configure(
        &self,
        chip: &Chip,
        cfg: &InjectedAdcConfig,
    ) -> Result<InjectedHandle, BringUpError> {
        if !cfg.adc.is_adc() {
            return Err(DescriptorError::Unsupported.into());
        }
        let base = chip.base(cfg.adc)?;
        if cfg.channels.is_empty()
            || cfg
                .channels
                .iter()
                .any(|c| c.channel > MAX_CHANNEL || u32::from(c.sample_time) > SAMPT_FIELD)
        {
            return Err(AdcError::Other.into());
        }
        let etsic = etsic_for_link(cfg).ok_or(AdcError::Other)?;
        let internal = cfg.channels.iter().any(|c| is_internal(c.channel));

        injected_sequence(&cfg.channels).apply(base);
        for c in &cfg.channels {
            if let Some(f) = sample_time(c.channel, c.sample_time) {
                f.apply(base);
            }
        }
        injected_ctl0(cfg.channels.len()).apply(base);
        injected_ctl1(cfg.left_aligned, internal, etsic).apply(base);
        ADC_ON.apply(base);
        Adc { base }.calibrate()?;
        ADC_ON.apply(base);
        Ok(InjectedHandle::new(base))
    }
}

/// The period ISR's ADC surface: the injected data registers, the flag it clears and the trigger
/// enable it re-asserts, resolved once at bring-up.
#[derive(Debug, Clone, Copy)]
pub struct InjectedHandle {
    idata: [Reg32; MAX_INJECTED_CHANNELS],
    stat: Reg32,
    ctl1: Reg32,
}

impl InjectedHandle {
    #[inline]
    fn new(base: u32) -> Self {
        Self {
            idata: [
                Reg32::new(base, IDATA[0]),
                Reg32::new(base, IDATA[1]),
                Reg32::new(base, IDATA[2]),
                Reg32::new(base, IDATA[3]),
            ],
            stat: Reg32::new(base, STAT),
            ctl1: Reg32::new(base, CTL1),
        }
    }

    /// Clear the injected end-of-conversion flag (`STAT.EOIC`, write 0 to clear). It is a level
    /// source into the NVIC that reading `IDATAx` does not clear, so the period ISR clears it or
    /// re-enters at once.
    #[inline]
    pub fn clear_eoic(&self) {
        self.stat.modify(STAT_EOIC, 0);
    }

    /// Set the injected trigger enable (`CTL1.ETEIC`) again: the stock firmware's per-period
    /// `ADC_CTL1 |= 0x8000`. The read-modify-write also writes `ADCON` = 1 over itself, which
    /// starts a regular conversion nothing reads.
    #[inline]
    pub fn reassert_trigger_enable(&self) {
        self.ctl1.modify(CTL1_ETEIC, CTL1_ETEIC);
    }

    /// Whether the injected group has completed a conversion since the flag was last cleared.
    #[inline]
    pub fn eoic(&self) -> bool {
        self.stat.read() & STAT_EOIC != 0
    }

    /// The four injected data registers in rank order (as converted, alignment included). A rank
    /// past the configured length reads whatever its register holds.
    #[inline]
    pub fn read_injected(&self) -> [u16; MAX_INJECTED_CHANNELS] {
        let mut out = [0u16; MAX_INJECTED_CHANNELS];
        for (slot, reg) in out.iter_mut().zip(self.idata.iter()) {
            *slot = reg.read() as u16;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::AdcClockDiv;
    use crate::descriptor::DESCRIPTOR;
    use crate::reg::fold;

    fn chans(list: &[u8]) -> heapless::Vec<InjectedChannel, MAX_INJECTED_CHANNELS> {
        list.iter()
            .map(|&channel| InjectedChannel {
                channel,
                sample_time: 1,
            })
            .collect()
    }

    fn injected(list: &[u8], link: TimerTriggerLink) -> InjectedAdcConfig {
        InjectedAdcConfig {
            adc: PeriphLabel::Adc0,
            channels: chans(list),
            left_aligned: true,
            trigger_timer: PeriphLabel::Timer0,
            trigger_link: link,
            clock_div: AdcClockDiv::Div6,
        }
    }

    /// The fields the injected bring-up applies before the calibration, in its order.
    fn injected_fields(cfg: &InjectedAdcConfig) -> Vec<Field> {
        let mut f = vec![injected_sequence(&cfg.channels)];
        f.extend(
            cfg.channels
                .iter()
                .filter_map(|c| sample_time(c.channel, c.sample_time)),
        );
        let internal = cfg.channels.iter().any(|c| is_internal(c.channel));
        f.push(injected_ctl0(cfg.channels.len()));
        f.push(injected_ctl1(
            cfg.left_aligned,
            internal,
            etsic_for_link(cfg).unwrap(),
        ));
        f.push(ADC_ON);
        f
    }

    fn single(channel: u8, code: u8) -> Vec<Field> {
        single_fields(channel, code).into_iter().flatten().collect()
    }

    #[test]
    fn adc0_is_at_its_base_on_every_chip() {
        assert_eq!(ADC0_BASE, 0x4001_2400);
    }

    #[test]
    fn the_second_adc_and_the_sync_mode_exist_exactly_on_the_multi_adc_chips() {
        let two = DESCRIPTOR.adc_count >= 2;
        assert_eq!(cfg!(hal_adc = "f10x"), two);
        assert_eq!(DESCRIPTOR.addrs.get(PeriphLabel::Adc1).is_some(), two);
    }

    #[test]
    fn the_vrefint_bring_up_reaches_the_golden_final_state() {
        let regs = fold(&single(17, 7));
        assert_eq!(regs[&CTL0], 0x0000_0000, "scan off");
        assert_eq!(regs[&CTL1], 0x009E_0001, "ADCON | ETSRC 7 | ETERC | TSVREN");
        assert_eq!(regs[&SAMPT0], 0x00E0_0000, "channel 17 at 239.5 cycles");
        assert_eq!(regs[&RSQ0], 0x0000_0000, "length 1");
        assert_eq!(regs[&RSQ2], 0x0000_0011, "rank 0 = channel 17");
    }

    #[test]
    fn an_external_channel_leaves_tsvren_alone() {
        let ctl1 = single_fields(4, 1)[1].unwrap();
        assert_eq!(ctl1.mask & CTL1_TSVREN, 0);
        assert_eq!(fold(&single(4, 1))[&CTL1], 0x001E_0001);
    }

    #[test]
    fn the_adc_switches_on_last() {
        let fields = single(4, 1);
        assert_eq!(fields.last(), Some(&ADC_ON));
        assert!(fields[..fields.len() - 1]
            .iter()
            .all(|f| f.offset != CTL1 || f.value & CTL1_ADCON == 0));
    }

    #[test]
    fn sample_times_land_in_their_register_and_shift() {
        for ch in 0..=MAX_CHANNEL {
            let f = sample_time(ch, 0b101).unwrap();
            let (offset, shift) = if ch < 10 {
                (SAMPT1, 3 * ch as u32)
            } else {
                (SAMPT0, 3 * (ch as u32 - 10))
            };
            assert_eq!(
                (f.offset, f.mask, f.value),
                (offset, 7 << shift, 5 << shift),
                "{ch}"
            );
        }
        assert_eq!(sample_time(18, 1), None);
        // A code wider than the field is cut to it.
        assert_eq!(sample_time(0, 0xFF).unwrap().value, 7);
    }

    #[test]
    fn the_injected_bring_up_reaches_the_golden_final_state() {
        let regs = fold(&injected_fields(&injected(&[4, 5], TimerTriggerLink::Ch3)));
        assert_eq!(regs[&CTL0], 0x0000_0180, "SM | EOICIE");
        assert_eq!(regs[&SAMPT1], 0x0000_9000, "ch4, ch5 at 7.5 cycles");
        assert_eq!(
            regs[&ISQ], 0x0012_9000,
            "IL 1, rank 0 = 4 at 14:10, rank 1 = 5 at 19:15"
        );
        assert_eq!(regs[&CTL1], 0x0000_9801, "ADCON | DAL | ETSIC CH3 | ETEIC");
    }

    #[test]
    fn the_firmware_trgo_link_selects_code_zero() {
        let regs = fold(&injected_fields(&injected(
            &[4, 5, 9],
            TimerTriggerLink::Trgo,
        )));
        assert_eq!(regs[&CTL1] & CTL1_ETSIC, 0);
        assert_ne!(regs[&CTL1] & CTL1_ETEIC, 0);
    }

    #[test]
    fn the_injected_sequence_is_right_aligned_for_every_length() {
        // One rank: the channel at 19:15, IL 0.
        assert_eq!(injected_sequence(&chans(&[4])).value, 4 << 15);
        // Three ranks (the firmware's phases plus the battery): 9:5, 14:10, 19:15, IL 2.
        assert_eq!(
            injected_sequence(&chans(&[4, 5, 9])).value,
            (2 << 20) | (4 << 5) | (5 << 10) | (9 << 15)
        );
        // Four ranks: from bit 0, IL 3.
        let f = injected_sequence(&chans(&[1, 2, 3, 17]));
        assert_eq!(f.value, (3 << 20) | 1 | (2 << 5) | (3 << 10) | (17 << 15));
        assert_eq!(f.mask, ISQ_IL | 0x000F_FFFF);
    }

    #[test]
    fn the_twelve_fet_four_rank_group_fills_every_slot() {
        // twelve-fet.md section 5.2: motor 0's pair (PC4, PC5: channels 14, 15), then motor 1's
        // (PA0, PC3: channels 0, 13). Four ranks take all four ISQ slots, rank n at 5n, IL 3.
        let regs = fold(&injected_fields(&injected(
            &[14, 15, 0, 13],
            TimerTriggerLink::Trgo,
        )));
        assert_eq!(
            regs[&ISQ],
            (3 << 20) | 14 | (15 << 5) | (13 << 15),
            "IL 3; rank 2 = channel 0 leaves 14:10 zero"
        );
        assert_eq!(regs[&ISQ] & (RANK_FIELD << 10), 0);
        assert_eq!(
            regs[&SAMPT0],
            (1 << 9) | (1 << 12) | (1 << 15),
            "14, 15, 13 at 7.5"
        );
        assert_eq!(regs[&SAMPT1], 1, "channel 0 at 7.5");
        assert_eq!(regs[&CTL0], CTL0_SM | CTL0_EOICIE);
        assert_eq!(
            regs[&CTL1], 0x0000_8801,
            "ADCON | DAL | ETSIC T0_TRGO | ETEIC"
        );
    }

    #[test]
    fn adc_instance_resolves_from_the_descriptor() {
        // SAFETY: no register access.
        let chip = unsafe { Chip::steal() };
        assert_eq!(
            chip.adc_instance(PeriphLabel::Adc0).unwrap().base,
            ADC0_BASE
        );
        assert!(matches!(
            chip.adc_instance(PeriphLabel::Timer0),
            Err(DescriptorError::Unsupported)
        ));
        for (label, base) in [
            (PeriphLabel::Adc1, 0x4001_2800),
            (PeriphLabel::Adc2, 0x4001_3C00),
        ] {
            match DESCRIPTOR.addrs.get(label) {
                Some(_) => assert_eq!(chip.adc_instance(label).unwrap().base, base),
                None => assert!(matches!(
                    chip.adc_instance(label),
                    Err(DescriptorError::MissingBase(l)) if l == label
                )),
            }
        }
    }

    #[test]
    fn scan_mode_follows_the_rank_count() {
        assert_eq!(injected_ctl0(1).value, CTL0_EOICIE);
        for len in 2..=4 {
            assert_eq!(injected_ctl0(len).value, CTL0_SM | CTL0_EOICIE);
        }
    }

    #[test]
    fn an_internal_rank_sets_tsvren_and_right_alignment_clears_dal() {
        let f = injected_ctl1(false, true, ETSIC_T0_TRGO);
        assert_eq!(f.value & (CTL1_TSVREN | CTL1_DAL), CTL1_TSVREN);
        assert_ne!(f.mask & CTL1_DAL, 0);
        assert_eq!(injected_ctl1(true, false, 0).mask & CTL1_TSVREN, 0);
    }

    #[test]
    fn only_timer0_has_an_injected_trigger() {
        let mut cfg = injected(&[4, 5], TimerTriggerLink::Ch3);
        assert_eq!(etsic_for_link(&cfg), Some(1));
        cfg.trigger_link = TimerTriggerLink::Trgo;
        assert_eq!(etsic_for_link(&cfg), Some(0));
        for timer in [PeriphLabel::Timer7, PeriphLabel::Timer1] {
            cfg.trigger_timer = timer;
            assert_eq!(etsic_for_link(&cfg), None, "{timer:?}");
        }
    }

    #[test]
    fn the_injected_bring_up_refuses_before_any_register_access() {
        // SAFETY: every case below is refused before a register is touched.
        let chip = unsafe { Chip::steal() };
        let c = InjectedAdcController::new();
        let base = injected(&[4, 5], TimerTriggerLink::Trgo);
        let refused = |cfg: InjectedAdcConfig| c.configure(&chip, &cfg).err();

        assert_eq!(
            refused(InjectedAdcConfig {
                adc: PeriphLabel::Timer0,
                ..base.clone()
            }),
            Some(BringUpError::Descriptor(DescriptorError::Unsupported))
        );
        if DESCRIPTOR.adc_count < 2 {
            assert_eq!(
                refused(InjectedAdcConfig {
                    adc: PeriphLabel::Adc1,
                    ..base.clone()
                }),
                Some(BringUpError::Descriptor(DescriptorError::MissingBase(
                    PeriphLabel::Adc1
                )))
            );
        }
        let other = Some(BringUpError::Adc(AdcError::Other));
        assert_eq!(
            refused(InjectedAdcConfig {
                channels: chans(&[]),
                ..base.clone()
            }),
            other
        );
        assert_eq!(
            refused(InjectedAdcConfig {
                channels: chans(&[4, 18]),
                ..base.clone()
            }),
            other
        );
        let mut slow = base.clone();
        slow.channels[1].sample_time = 8;
        assert_eq!(refused(slow), other);
        assert_eq!(
            refused(InjectedAdcConfig {
                trigger_timer: PeriphLabel::Timer7,
                ..base.clone()
            }),
            other
        );
    }

    #[cfg(hal_adc = "f10x")]
    #[test]
    fn the_dual_bring_up_reaches_the_golden_final_state() {
        let mut adc0 = single(4, 1);
        adc0.push(SYNCM_REGULAR_PARALLEL);
        let adc0 = fold(&adc0);
        assert_eq!(adc0[&CTL0], 0x0006_0000, "SM clear, SYNCM regular parallel");
        assert_eq!(adc0[&CTL1], 0x001E_0001);
        assert_eq!(adc0[&SAMPT1], 0x0000_1000);
        assert_eq!(adc0[&RSQ2], 0x0000_0004);
        let adc1 = fold(&single(5, 1));
        assert_eq!(adc1[&CTL0], 0, "SYNCM is ADC0's alone");
        assert_eq!(adc1[&CTL1], 0x001E_0001);
        assert_eq!(adc1[&SAMPT1], 0x0000_8000);
        assert_eq!(adc1[&RSQ2], 0x0000_0005);
        // SAFETY: no register access.
        let d = unsafe { Chip::steal() }.dual_adc();
        assert_eq!(
            (d.primary.base, d.secondary.base),
            (0x4001_2400, 0x4001_2800)
        );
    }
}
