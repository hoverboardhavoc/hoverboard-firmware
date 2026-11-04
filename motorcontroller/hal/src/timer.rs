// SPDX-License-Identifier: GPL-3.0-or-later
//! The advanced timer (TIMER0, and TIMER7 on the high-density F10x): complementary PWM for the
//! bridge, the ADC trigger, the per-period handle and the arming gate.
//!
//! One register model on every GD32 this HAL drives: `gd32f10x_timer.h` and `gd32f1x0_timer.h` give
//! the advanced timer the same offsets and bit positions, so the model is parameterised only by the
//! instance base and build.rs carries no cfg for it.
//!
//! Three objects, split by what they may write:
//!
//! - [`PwmTimer`]: the bring-up ([`PwmTimer::configure`], [`PwmTimer::configure_trigger`],
//!   [`PwmTimer::follow`]) and the counter start. It writes `CCHP` once, with `POEN` clear, and
//!   never sets `POEN`.
//! - [`PwmHandle`]: the period ISR's surface: the three duties (`CH0CV`..`CH2CV`), the trigger
//!   compare (`CH3CV`), the channel output enables in `CHCTL2` and a read of the break flag
//!   (`INTF.BRKIF`). It holds no `CCHP` accessor.
//! - [`ArmGate`]: `CCHP.POEN` (the main output enable, MOE) and the break flag's clear, nothing
//!   else. It is the only object in the HAL that sets `POEN`, so a bug in the period ISR cannot
//!   energise a disarmed bridge.
//!
//! | reg      | offset | fields the HAL programs                                             |
//! |----------|--------|---------------------------------------------------------------------|
//! | `CTL0`   | `0x00` | CEN(0), DIR(4), CAM(6:5), ARSE(7), CKDIV(9:8)                       |
//! | `CTL1`   | `0x04` | MMC(6:4), ISOx(8 + 2n), ISOxN(9 + 2n)                               |
//! | `SMCFG`  | `0x08` | SMC(2:0), TRGS(6:4)                                                 |
//! | `INTF`   | `0x10` | BRKIF(7), read and cleared (write 0 to clear)                       |
//! | `SWEVG`  | `0x14` | UPG(0)                                                              |
//! | `CHCTL0` | `0x18` | CH0 (7:0), CH1 (15:8): MS(1:0), COMSEN(3), COMCTL(6:4)              |
//! | `CHCTL1` | `0x1C` | CH2 (7:0), CH3 (15:8), as `CHCTL0`                                  |
//! | `CHCTL2` | `0x20` | per channel at 4n: EN(0), P(1), NEN(2), NP(3)                       |
//! | `CNT`    | `0x24` | the counter                                                         |
//! | `PSC`    | `0x28` | prescaler                                                           |
//! | `CAR`    | `0x2C` | auto-reload (the period)                                            |
//! | `CREP`   | `0x30` | repetition counter                                                  |
//! | `CHxCV`  | `0x34` | channel compare, `+4` per channel to `CH3CV` at `0x40`              |
//! | `CCHP`   | `0x44` | DTCFG(7:0), PROT(9:8), IOS(10), ROS(11), BRKEN(12), BRKP(13), POEN(15) |

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::config::{
    BreakConfig, OcMode, PwmChannelConfig, PwmConfig, TrgoSource, MAX_PWM_CHANNELS,
};
use crate::error::{DescriptorError, PwmError};
use crate::reg::{Field, Reg32};

const CTL0: u32 = 0x00;
const CTL1: u32 = 0x04;
const SMCFG: u32 = 0x08;
const INTF: u32 = 0x10;
const SWEVG: u32 = 0x14;
const CHCTL0: u32 = 0x18;
const CHCTL1: u32 = 0x1C;
const CHCTL2: u32 = 0x20;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const CAR: u32 = 0x2C;
const CREP: u32 = 0x30;
/// `CH0CV`; channel `n`'s compare is at `CH0CV + 4 * n`.
const CH0CV: u32 = 0x34;
const CH3CV: u32 = CH0CV + 4 * 3;
const CCHP: u32 = 0x44;

const CTL0_CEN: u32 = 1 << 0;
const CTL0_DIR: u32 = 1 << 4;
const CTL0_CAM: u32 = 0b11 << 5;
const CTL0_ARSE: u32 = 1 << 7;
const CTL0_CKDIV: u32 = 0b11 << 8;

const SWEVG_UPG: u32 = 1 << 0;

/// `INTF.BRKIF`, the break flag: set by the hardware when the break input goes active with
/// `BRKEN` set (`TIMER_FLAG_BRK`).
const INTF_BRKIF: u32 = 1 << 7;

/// `SMCFG.SMC`, the slave mode.
const SMCFG_SMC: u32 = 0b111;
/// `SMCFG.TRGS`, the trigger input.
const SMCFG_TRGS: u32 = 0b111 << 4;
/// `SMC = 6`, event mode (`TIMER_SLAVE_MODE_EVENT`): the trigger's rising edge sets `CEN`.
const SMC_EVENT: u32 = 6;

const CTL1_MMC: u32 = 0b111 << 4;
/// `ISOx` within a channel's two-bit idle field at `8 + 2n`.
const ISO: u32 = 1 << 0;
/// `ISOxN` within the field.
const ISON: u32 = 1 << 1;

/// One channel's half of `CHCTL0`/`CHCTL1` that the HAL programs: `CHxMS` (1:0, 0 = output),
/// `CHxCOMSEN` (3) and `CHxCOMCTL` (6:4). The fast-enable (2) and clear-enable (7) bits keep their
/// reset 0.
const OC_HALF: u32 = 0b0111_1011;
/// `CHxCOMSEN`: the compare shadow, so a duty written mid-period takes effect at the next update.
const OC_COMSEN: u32 = 1 << 3;

/// `CHxEN` within a channel's four-bit `CHCTL2` field.
const CH_EN: u32 = 1 << 0;
/// `CHxNEN`.
const CH_NEN: u32 = 1 << 2;
/// `CHxNP`: the complementary output active low.
const CH_NP: u32 = 1 << 3;
/// The output enables of the three bridge channels (`CHxEN` and `CHxNEN`), the bits
/// [`PwmHandle::set_channel_outputs`] writes. The polarity bits are outside it.
const BRIDGE_ENABLES: u32 = (CH_EN | CH_NEN) * 0x111;

const CCHP_DTCFG: u32 = 0xFF;
const CCHP_IOS: u32 = 1 << 10;
const CCHP_ROS: u32 = 1 << 11;
const CCHP_BRKEN: u32 = 1 << 12;
const CCHP_BRKP: u32 = 1 << 13;
/// `POEN`, the main output enable. Named in this file by [`ArmGate`] alone.
const CCHP_POEN: u32 = 1 << 15;

// --- the bring-up's fields, as values -----------------------------------------------------------

/// `CTL0`'s time-base fields (`timer_init` and `timer_auto_reload_shadow_enable`): the alignment
/// (`DIR`, `CAM`), the auto-reload shadow and the dead-time clock divider. `CEN` stays clear.
const fn time_base(cfg: &PwmConfig) -> Field {
    let arse = if cfg.arse { CTL0_ARSE } else { 0 };
    Field {
        offset: CTL0,
        mask: CTL0_DIR | CTL0_CAM | CTL0_ARSE | CTL0_CKDIV,
        value: cfg.align.ctl0_bits() | arse | ((cfg.ckdiv.ckdiv_code() << 8) & CTL0_CKDIV),
    }
}

/// Channel `n`'s output-compare half (`timer_channel_output_mode_config` and
/// `timer_channel_output_shadow_config`): output mode, compare mode `mode`, compare shadow on.
/// Channels 0 and 1 are in `CHCTL0`, 2 and 3 in `CHCTL1`, the odd channel in the high byte.
const fn oc_half(n: u32, mode: OcMode) -> Field {
    let (offset, shift) = if n < 2 {
        (CHCTL0, 8 * n)
    } else {
        (CHCTL1, 8 * (n - 2))
    };
    Field {
        offset,
        mask: OC_HALF << shift,
        value: ((mode.comctl() << 4) | OC_COMSEN) << shift,
    }
}

/// `CHCTL2` for the three bridge channels (`timer_channel_output_config`): both outputs of each
/// pair enabled, the high side active high, the low side active low when the channel's `polarity`
/// is set. With `POEN` clear the enables reach no pin.
const fn bridge_outputs(channels: &[PwmChannelConfig; MAX_PWM_CHANNELS]) -> Field {
    let mut value = 0;
    let mut n = 0;
    while n < MAX_PWM_CHANNELS {
        let np = if channels[n].polarity { CH_NP } else { 0 };
        value |= (CH_EN | CH_NEN | np) << (4 * n);
        n += 1;
    }
    Field {
        offset: CHCTL2,
        mask: 0xFFF,
        value,
    }
}

/// `CTL1`'s idle levels for the three bridge channels: the level each output drives while `POEN`
/// is clear.
const fn idle_levels(channels: &[PwmChannelConfig; MAX_PWM_CHANNELS]) -> Field {
    let mut value = 0;
    let mut n = 0;
    while n < MAX_PWM_CHANNELS {
        let iso = if channels[n].idle_high { ISO } else { 0 };
        let ison = if channels[n].idle_high_n { ISON } else { 0 };
        value |= (iso | ison) << (8 + 2 * n);
        n += 1;
    }
    Field {
        offset: CTL1,
        mask: 0x3F << 8,
        value,
    }
}

/// `CTL1.MMC`, the TRGO source (`timer_master_output_trigger_source_select`).
const fn trgo(src: TrgoSource) -> Field {
    Field {
        offset: CTL1,
        mask: CTL1_MMC,
        value: (src.mmc() << 4) & CTL1_MMC,
    }
}

/// The `TRGS` code by which advanced timer `slave` takes `master`'s TRGO as its trigger, or `None`
/// where the trigger matrix offers no such input. The GD32F10x User Manual Rev2.6 chapter 15 table
/// (page 268): TIMER7's `ITI0` is `TIMER0_TRGO`; TIMER0's four inputs are TIMER4, 1, 2 and 3, so
/// TIMER0 cannot follow TIMER7. No other part this HAL drives carries a second advanced timer.
const fn internal_trigger(slave: PeriphLabel, master: PeriphLabel) -> Option<u32> {
    match (slave, master) {
        // TIMER_SMCFG_TRGSEL_ITI0.
        (PeriphLabel::Timer7, PeriphLabel::Timer0) => Some(0),
        _ => None,
    }
}

/// `SMCFG` for event mode on trigger input `trgs` (`timer_input_trigger_source_select` and
/// `timer_slave_mode_select`): the counter starts on the trigger's first rising edge, and later
/// edges do nothing while `CEN` is set (GD32F10x User Manual Rev2.6 section 15.1.4, master-slave
/// management).
const fn event_slave(trgs: u32) -> Field {
    Field {
        offset: SMCFG,
        mask: SMCFG_SMC | SMCFG_TRGS,
        value: ((trgs << 4) & SMCFG_TRGS) | SMC_EVENT,
    }
}

/// `CH3EN`: the trigger channel's output enable. Its pin (PA11 on TIMER0) is never routed, so the
/// enable drives nothing off-chip.
const CH3_OUTPUT: Field = Field {
    offset: CHCTL2,
    mask: CH_EN << 12,
    value: CH_EN << 12,
};

/// The `CCHP` word (`timer_break_config`, which assigns the whole register): the dead time, both
/// off-states enabled so a disabled or disarmed channel drives its idle level, the break input as
/// configured, protection off, automatic output enable off, and `POEN` clear.
const fn cchp_word(dead_time: u8, brk: BreakConfig) -> u32 {
    let mut word = (dead_time as u32 & CCHP_DTCFG) | CCHP_ROS | CCHP_IOS;
    if brk.enabled {
        word |= CCHP_BRKEN;
        if brk.level {
            word |= CCHP_BRKP;
        }
    }
    word
}

/// The `CHCTL2` enable bits for `enabled`: `CHxEN | CHxNEN` of each channel that drives. No
/// branch: each flag scales its channel's two bits.
#[inline(always)]
const fn output_enables(enabled: [bool; MAX_PWM_CHANNELS]) -> u32 {
    let pair = CH_EN | CH_NEN;
    (enabled[0] as u32 * pair)
        | ((enabled[1] as u32 * pair) << 4)
        | ((enabled[2] as u32 * pair) << 8)
}

// --- the bring-up -------------------------------------------------------------------------------

/// A configured advanced timer: its label, base and period. The bring-up leaves the counter
/// stopped and the bridge disarmed; the per-period writes go through [`PwmTimer::handle`] and the
/// output enable through [`PwmTimer::arm_gate`].
#[derive(Debug, Clone, Copy)]
pub struct PwmTimer {
    label: PeriphLabel,
    base: u32,
    period: u16,
}

impl PwmTimer {
    /// Configure the timer `cfg.timer` for the complementary bridge, with `POEN` clear: the vendor
    /// sequence `timer_init`, then per bridge channel the output, compare-mode, shadow and zero
    /// duty, then `timer_auto_reload_shadow_enable` and `timer_break_config`. The trigger channel
    /// and TRGO are [`PwmTimer::configure_trigger`]'s.
    ///
    /// Refused before any register access: a label that is not an advanced timer (`Unsupported`),
    /// or one the chip does not carry (`MissingBase`). The timer's clock must already be on
    /// ([`crate::clock::enable_timer`]).
    pub fn configure(chip: &Chip, cfg: &PwmConfig) -> Result<PwmTimer, DescriptorError> {
        if !cfg.timer.is_timer() {
            return Err(DescriptorError::Unsupported);
        }
        let base = chip.base(cfg.timer)?;
        let reg = |offset| Reg32::new(base, offset);

        // timer_init: the prescaler, the time base, the period and the repetition counter, then an
        // update event that loads the shadowed prescaler, period and repetition count.
        reg(PSC).write(u32::from(cfg.prescaler));
        time_base(cfg).apply(base);
        reg(CAR).write(u32::from(cfg.period));
        reg(CREP).write(u32::from(cfg.crep));
        reg(SWEVG).write(SWEVG_UPG);

        for n in 0..MAX_PWM_CHANNELS as u32 {
            oc_half(n, OcMode::Pwm0).apply(base);
            reg(CH0CV + 4 * n).write(0);
        }
        bridge_outputs(&cfg.channels).apply(base);
        idle_levels(&cfg.channels).apply(base);
        reg(CCHP).write(cchp_word(cfg.dead_time, cfg.brk));

        Ok(PwmTimer {
            label: cfg.timer,
            base,
            period: cfg.period,
        })
    }

    /// Configure channel 3 as the ADC trigger: an output compare in `oc_mode` with the compare
    /// shadow on and `CH3CV = trigger_compare`, its output enable when `ch_enable`, and the TRGO
    /// source `trgo_src`. With `ch_enable` clear `CHCTL2` is not written.
    pub fn configure_trigger(
        &self,
        trigger_compare: u16,
        oc_mode: OcMode,
        ch_enable: bool,
        trgo_src: TrgoSource,
    ) {
        oc_half(3, oc_mode).apply(self.base);
        Reg32::new(self.base, CH3CV).write(u32::from(trigger_compare));
        if ch_enable {
            CH3_OUTPUT.apply(self.base);
        }
        trgo(trgo_src).apply(self.base);
    }

    /// Make this timer `master`'s event-mode slave: preset the counter to `start_count`, then set
    /// `SMCFG` to event mode on the trigger input that carries `master`'s TRGO. The counter then
    /// starts on the first rising edge of `master`'s TRGO (`twelve-fet.md` section 5.1, steps 2 and
    /// 4: TIMER7 following TIMER0).
    ///
    /// Both timers are configured first ([`PwmTimer::configure`]): its last time-base write is the
    /// `UPG` update event that loads the repetition counter and, in the centre-aligned modes,
    /// initialises the counter to 0 (GD32F10x User Manual Rev2.6 section 15.1.4, centre-aligned
    /// counting), so the preset comes after it and nothing here issues `UPG`.
    ///
    /// Refused before any register access: a pair the trigger matrix does not offer (only TIMER7
    /// following TIMER0, [`PwmError::NoInternalTrigger`]), and a `start_count` above the period
    /// ([`PwmError::DutyOutOfRange`]).
    pub fn follow(&self, master: &PwmTimer, start_count: u16) -> Result<(), PwmError> {
        let trgs = internal_trigger(self.label, master.label).ok_or(PwmError::NoInternalTrigger)?;
        if start_count > self.period {
            return Err(PwmError::DutyOutOfRange);
        }
        // timer_counter_value_config, then the two SMCFG fields in one write.
        Reg32::new(self.base, CNT).write(u32::from(start_count));
        event_slave(trgs).apply(self.base);
        Ok(())
    }

    /// The per-period handle for this timer.
    #[inline]
    pub fn handle(&self) -> PwmHandle {
        PwmHandle::new(self.base, self.period)
    }

    /// The arming gate for this timer, the only writer of `POEN`.
    #[inline]
    pub fn arm_gate(&self) -> ArmGate {
        ArmGate::new(self.base)
    }

    /// Start the counter (`CTL0.CEN`). With `POEN` clear the compare events run and reach no pin.
    #[inline]
    pub fn enable_counter(&self) {
        Reg32::new(self.base, CTL0).modify(CTL0_CEN, CTL0_CEN);
    }
}

// --- the per-period handle ----------------------------------------------------------------------

/// The period ISR's timer surface: the registers it writes, resolved once at bring-up so a write
/// is one store with no lookup.
///
/// It holds no `CCHP` accessor: nothing on it can set `POEN`.
#[derive(Debug, Clone, Copy)]
pub struct PwmHandle {
    /// `CH0CV`, `CH1CV`, `CH2CV`: the bridge duties.
    ch_cv: [Reg32; MAX_PWM_CHANNELS],
    /// `CH3CV`: the ADC trigger compare.
    trig_cv: Reg32,
    /// `CHCTL2`: the channel output enables.
    chctl2: Reg32,
    /// `INTF`: the break flag.
    intf: Reg32,
    /// `CAR`, the bound a duty is checked against.
    period: u16,
}

impl PwmHandle {
    #[inline]
    fn new(base: u32, period: u16) -> Self {
        Self {
            ch_cv: [
                Reg32::new(base, CH0CV),
                Reg32::new(base, CH0CV + 4),
                Reg32::new(base, CH0CV + 8),
            ],
            trig_cv: Reg32::new(base, CH3CV),
            chctl2: Reg32::new(base, CHCTL2),
            intf: Reg32::new(base, INTF),
            period,
        }
    }

    /// Write the three duties to `CH0CV`..`CH2CV`. A duty above the period is refused with
    /// [`PwmError::DutyOutOfRange`] before any of the three is written, so a refused set leaves
    /// the compares as they were.
    #[inline]
    pub fn set_duties(&self, duties: [u16; MAX_PWM_CHANNELS]) -> Result<(), PwmError> {
        for &d in &duties {
            if d > self.period {
                return Err(PwmError::DutyOutOfRange);
            }
        }
        for (cv, &d) in self.ch_cv.iter().zip(duties.iter()) {
            cv.write(u32::from(d));
        }
        Ok(())
    }

    /// Rewrite the trigger compare (`CH3CV`), refused above the period.
    #[inline]
    pub fn rearm_trigger(&self, compare: u16) -> Result<(), PwmError> {
        if compare > self.period {
            return Err(PwmError::DutyOutOfRange);
        }
        self.trig_cv.write(u32::from(compare));
        Ok(())
    }

    /// Enable or disable each bridge channel's two outputs (`CHxEN`, `CHxNEN`) in one
    /// read-modify-write of `CHCTL2`; a disabled channel floats. The polarity bits and the trigger
    /// channel's enable are outside the mask.
    #[inline]
    pub fn set_channel_outputs(&self, enabled: [bool; MAX_PWM_CHANNELS]) {
        self.chctl2.modify(BRIDGE_ENABLES, output_enables(enabled));
    }

    /// Whether the break flag (`INTF.BRKIF`) is set: the break input went active since the flag
    /// was last cleared ([`ArmGate::clear_break_flag`]), which cleared `POEN` in hardware
    /// (`timer_flag_get(TIMER_FLAG_BRK)`).
    #[inline]
    pub fn break_flag(&self) -> bool {
        self.intf.read() & INTF_BRKIF != 0
    }
}

// --- the arming gate ----------------------------------------------------------------------------

/// The bridge's main output enable (`CCHP.POEN`, MOE): the only object in the HAL that sets it.
/// Built from a configured timer ([`PwmTimer::arm_gate`]) and kept apart from the per-period
/// [`PwmHandle`].
#[derive(Debug, Clone, Copy)]
pub struct ArmGate {
    cchp: Reg32,
    intf: Reg32,
}

impl ArmGate {
    #[inline]
    fn new(base: u32) -> Self {
        Self {
            cchp: Reg32::new(base, CCHP),
            intf: Reg32::new(base, INTF),
        }
    }

    /// Set `POEN`: the enabled channel outputs reach the gate drivers.
    #[inline]
    pub fn arm(&self) {
        self.cchp.modify(CCHP_POEN, CCHP_POEN);
    }

    /// Clear `POEN`: every output drops to its configured idle level.
    #[inline]
    pub fn disarm(&self) {
        self.cchp.modify(CCHP_POEN, 0);
    }

    /// Clear the break flag (`INTF.BRKIF`, `timer_flag_clear(TIMER_FLAG_BRK)`): a plain write of
    /// every flag bit but `BRKIF`, since the flags clear on a written 0 and a read-modify-write
    /// could clear a flag raised between its read and its write. A break input still active sets
    /// the flag again at once. Here and not on [`PwmHandle`], so the break's recovery stays with
    /// the arming layer that holds this gate.
    #[inline]
    pub fn clear_break_flag(&self) {
        self.intf.write(!INTF_BRKIF);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{ClockDiv, PwmAlign};
    use crate::reg::fold;

    fn channel(polarity: bool, idle_high: bool, idle_high_n: bool) -> PwmChannelConfig {
        PwmChannelConfig {
            polarity,
            idle_high,
            idle_high_n,
        }
    }

    /// The golden vectors' configuration (`runtime-hal/harness/vectors/timer/pwm_bringup_*`).
    fn golden() -> PwmConfig {
        let ch = channel(true, true, true);
        PwmConfig {
            timer: PeriphLabel::Timer0,
            channels: [ch; 3],
            period: 2250,
            prescaler: 0,
            dead_time: 0x1C,
            brk: BreakConfig {
                enabled: false,
                level: false,
            },
            trigger_compare: 2249,
            align: PwmAlign::Center2,
            arse: true,
            trigger_oc_mode: OcMode::Pwm0,
            trigger_ch_enable: false,
            crep: 0,
            ckdiv: ClockDiv::Div2,
            trgo_src: TrgoSource::Update,
        }
    }

    /// The firmware's configuration (`motor.rs`, `timer_config`): centre-aligned mode 1, no
    /// auto-reload shadow, N-side-only idle high.
    fn firmware() -> PwmConfig {
        PwmConfig {
            channels: [channel(true, false, true); 3],
            align: PwmAlign::Center1,
            arse: false,
            crep: 1,
            ..golden()
        }
    }

    /// The fields `configure` and `configure_trigger` apply, in order (the register writes that
    /// are plain stores are checked separately).
    fn bring_up_fields(cfg: &PwmConfig, trigger: Option<(OcMode, bool, TrgoSource)>) -> Vec<Field> {
        let mut f = vec![time_base(cfg)];
        for n in 0..3 {
            f.push(oc_half(n, OcMode::Pwm0));
        }
        f.push(bridge_outputs(&cfg.channels));
        f.push(idle_levels(&cfg.channels));
        if let Some((mode, ch_enable, src)) = trigger {
            f.push(oc_half(3, mode));
            if ch_enable {
                f.push(CH3_OUTPUT);
            }
            f.push(trgo(src));
        }
        f
    }

    #[test]
    fn the_bring_up_reaches_the_golden_final_state() {
        let regs = fold(&bring_up_fields(&golden(), None));
        assert_eq!(regs[&CTL0], 0x0000_01C0, "CAM centre 2, ARSE, CKDIV /2");
        assert_eq!(regs[&CHCTL0], 0x0000_6868, "CH0, CH1: PWM0 + shadow");
        assert_eq!(
            regs[&CHCTL1], 0x0000_0068,
            "CH2: PWM0 + shadow, CH3 untouched"
        );
        assert_eq!(regs[&CHCTL2], 0x0000_0DDD, "EN | NEN | NP per pair");
        assert_eq!(regs[&CTL1], 0x0000_3F00, "both idle levels high");
        assert_eq!(cchp_word(0x1C, golden().brk), 0x0000_0C1C);
    }

    #[test]
    fn the_trigger_step_reaches_the_golden_final_state() {
        let regs = fold(&bring_up_fields(
            &golden(),
            Some((OcMode::Pwm0, false, TrgoSource::Update)),
        ));
        assert_eq!(regs[&CHCTL1], 0x0000_6868);
        assert_eq!(
            regs[&CTL1], 0x0000_3F20,
            "MMC = update over the idle levels"
        );
        assert_eq!(
            regs[&CHCTL2], 0x0000_0DDD,
            "CH3EN clear: CHCTL2 as the bring-up left it"
        );
    }

    #[test]
    fn the_firmware_configuration_programs_its_documented_values() {
        let regs = fold(&bring_up_fields(
            &firmware(),
            Some((OcMode::Pwm1, true, TrgoSource::Ch3Compare)),
        ));
        assert_eq!(regs[&CTL0], 0x0000_0120, "CAM centre 1, CKDIV /2, no ARSE");
        assert_eq!(regs[&CHCTL1], 0x0000_7868, "CH3 in PWM1");
        assert_eq!(
            regs[&CHCTL2], 0x0000_1DDD,
            "the stock golden's value with CH3EN"
        );
        assert_eq!(
            regs[&CTL1], 0x0000_2A70,
            "N-side idle high, MMC = CH3 compare"
        );
    }

    #[test]
    fn alignment_and_divider_codes_land_in_their_fields() {
        for (align, bits) in [
            (PwmAlign::EdgeUp, 0),
            (PwmAlign::EdgeDown, 1 << 4),
            (PwmAlign::Center1, 1 << 5),
            (PwmAlign::Center2, 2 << 5),
            (PwmAlign::Center3, 3 << 5),
        ] {
            for (ckdiv, code) in [
                (ClockDiv::Div1, 0),
                (ClockDiv::Div2, 1),
                (ClockDiv::Div4, 2),
            ] {
                let f = time_base(&PwmConfig {
                    align,
                    ckdiv,
                    arse: false,
                    ..golden()
                });
                assert_eq!(f.value, bits | (code << 8), "{align:?} {ckdiv:?}");
                assert_eq!(
                    f.value & CTL0_CEN,
                    0,
                    "the bring-up never starts the counter"
                );
            }
        }
    }

    #[test]
    fn polarity_inverts_only_the_low_side() {
        let f = bridge_outputs(&[
            channel(false, false, false),
            channel(true, false, false),
            channel(false, false, false),
        ]);
        assert_eq!(f.value, 0x0000_05D5);
        // The main-output polarity bits (CHxP) stay clear on every channel.
        assert_eq!(f.value & 0x0000_0222, 0);
    }

    #[test]
    fn each_idle_level_has_its_own_bit() {
        let f = idle_levels(&[
            channel(true, true, false),
            channel(true, false, true),
            channel(true, true, true),
        ]);
        assert_eq!(f.value, (0b01 << 8) | (0b10 << 10) | (0b11 << 12));
        assert_eq!(f.mask & (1 << 14), 0, "ISO3 is not a bridge channel's");
    }

    #[test]
    fn the_cchp_word_carries_the_break_and_never_poen() {
        for dead_time in 0..=u8::MAX {
            for (enabled, level) in [(false, false), (false, true), (true, false), (true, true)] {
                let w = cchp_word(dead_time, BreakConfig { enabled, level });
                assert_eq!(w & CCHP_POEN, 0);
                assert_eq!(w & CCHP_DTCFG, u32::from(dead_time));
                assert_eq!(w & (CCHP_ROS | CCHP_IOS), CCHP_ROS | CCHP_IOS);
                assert_eq!(w & CCHP_BRKEN != 0, enabled);
                assert_eq!(w & CCHP_BRKP != 0, enabled && level);
                assert_eq!(w & (0b11 << 8), 0, "protection off");
            }
        }
    }

    #[test]
    fn output_enables_touch_only_the_enable_bits() {
        assert_eq!(output_enables([false; 3]), 0);
        assert_eq!(output_enables([true; 3]), 0x0000_0555);
        assert_eq!(output_enables([true, false, true]), 0x0000_0505);
        assert_eq!(output_enables([false, true, false]), 0x0000_0050);
        assert_eq!(BRIDGE_ENABLES, 0x0000_0555);
        assert_eq!(
            BRIDGE_ENABLES & (CH_NP * 0x1111),
            0,
            "polarity outside the mask"
        );
        assert_eq!(BRIDGE_ENABLES & (CH_EN << 12), 0, "CH3EN outside the mask");
    }

    #[test]
    fn out_of_range_duties_are_refused_before_any_write() {
        // The refusal path returns before touching a register, so it runs on the host against an
        // address nothing maps.
        let h = PwmHandle::new(0, 2250);
        for i in 0..3 {
            let mut d = [0u16; 3];
            d[i] = 2251;
            assert_eq!(h.set_duties(d), Err(PwmError::DutyOutOfRange));
        }
        assert_eq!(h.rearm_trigger(2251), Err(PwmError::DutyOutOfRange));
    }

    #[test]
    fn the_event_slave_sets_event_mode_on_iti0() {
        let trgs = internal_trigger(PeriphLabel::Timer7, PeriphLabel::Timer0).unwrap();
        let f = event_slave(trgs);
        assert_eq!((f.offset, f.mask, f.value), (SMCFG, 0x77, 0x06));
    }

    #[test]
    fn only_timer7_follows_timer0() {
        let labels = [PeriphLabel::Timer0, PeriphLabel::Timer7];
        for slave in labels {
            for master in labels {
                let offered = slave == PeriphLabel::Timer7 && master == PeriphLabel::Timer0;
                assert_eq!(
                    internal_trigger(slave, master).is_some(),
                    offered,
                    "{slave:?} following {master:?}"
                );
            }
        }
    }

    #[test]
    fn follow_refuses_before_any_register_access() {
        // The refusals return before touching a register, so they run on the host against
        // addresses nothing maps.
        let timer = |label| PwmTimer {
            label,
            base: 0,
            period: 2250,
        };
        let (t0, t7) = (timer(PeriphLabel::Timer0), timer(PeriphLabel::Timer7));
        assert_eq!(t0.follow(&t7, 0), Err(PwmError::NoInternalTrigger));
        assert_eq!(t0.follow(&t0, 0), Err(PwmError::NoInternalTrigger));
        assert_eq!(t7.follow(&t7, 0), Err(PwmError::NoInternalTrigger));
        assert_eq!(t7.follow(&t0, 2251), Err(PwmError::DutyOutOfRange));
    }

    #[test]
    fn configure_refuses_a_label_that_is_not_a_carried_advanced_timer() {
        // SAFETY: refusals return before any register access.
        let chip = unsafe { Chip::steal() };
        let cfg = PwmConfig {
            timer: PeriphLabel::Timer1,
            ..golden()
        };
        assert!(matches!(
            PwmTimer::configure(&chip, &cfg),
            Err(DescriptorError::Unsupported)
        ));
        if !chip.has_advanced_timer(PeriphLabel::Timer7) {
            let cfg = PwmConfig {
                timer: PeriphLabel::Timer7,
                ..golden()
            };
            assert!(matches!(
                PwmTimer::configure(&chip, &cfg),
                Err(DescriptorError::MissingBase(PeriphLabel::Timer7))
            ));
        }
    }
}
