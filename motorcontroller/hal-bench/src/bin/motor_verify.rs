// SPDX-License-Identifier: GPL-3.0-or-later
//! `motor-verify`: the motor hot path configured as the firmware configures it, then read back
//! register by register, with the outputs disarmed (specs/hal.md section 8.4).
//!
//! The debug holds are set first (`hal::debug_hold_timers`). TIMER0 at 16 kHz centre-aligned (mode
//! 1, `CAR` 2250, `PSC` 0, `CREP` 1) at the 72 MHz tree, the channel 3 compare on TRGO; the injected
//! ADC group triggered by it; the counter started and one injected end of conversion awaited. MOE is
//! never set: no `ArmGate` is made, so the gate driver inputs sit at their idle levels, the disarmed
//! state the firmware boots into. The motor-era NVIC priorities are applied and read back.
//!
//! On a one-timer chip (the GD32F103C8, the GD32F130C8): the fleet's complementary set routed
//! (PA8-10 / PB13-15), dead time 28, the injected ranks PA0 (ch 0) and PB0 (ch 8).
//!
//! On a chip whose descriptor carries TIMER7 (the GD32F103RC, the 12-FET, `twelve-fet.md` sections
//! 5.1, 5.2 and 10 step 3): both timers configured alike with the 12-FET's dead time 32, TIMER7 made
//! TIMER0's event-mode slave (`PwmTimer::follow`, counter preset to the trigger compare) before
//! TIMER0 starts, and the four-rank injected group, motor 0's pair then motor 1's: PC4 (ch 14), PC5
//! (ch 15), PA0 (ch 0), PC3 (ch 13). No gate pin is routed: the 12-FET has no NRST, so the pins stay
//! the inputs they reset to, the state a blank or freshly reset chip leaves its gate drivers in. With
//! MOE clear neither timer reaches a pin either way.
//!
//! Result words: 2 = CTL0, 3 = CTL1, 4 = CHCTL0, 5 = CHCTL1, 6 = CHCTL2, 7 = PSC, 8 = CAR,
//! 9 = CREP, 10 = CH3CV, 11 = CCHP, 12 = the TIMER0 checks (a bit per failed check: 0 CAR, 1 PSC,
//! 2 dead time, 3 MOE set, 4 CH3 compare, 5 centre-aligned mode 1, 6 TRGO = CH3 compare), 13 =
//! injected bring-up (0 ok), 14 = EOIC seen (1), 15 = injected rank 0 raw, 16 = rank 1 raw, 17 =
//! NVIC priority of the period IRQ, 18 = SHPR3 top byte (SysTick), 19 = priority verdict (1 = 0x10
//! and 0xF0), 20 = MOE after the run (0), 21 = verdict.
//!
//! TIMER7, written only where the descriptor carries it (0 elsewhere): 22..31 = its CTL0, CTL1,
//! CHCTL0, CHCTL1, CHCTL2, PSC, CAR, CREP, CH3CV, CCHP, 32 = SMCFG, 33 = CNT (both before TIMER0
//! starts), 34 = the TIMER7 checks (bits 0, 1, 2, 3, 5 as TIMER0's, 7 CREP not 1, 8 SMCFG not
//! event mode on ITI0, 9 CNT not the preset, 10 CEN set before TIMER0 started, 11 CEN still clear
//! after the injected conversions), 37 = injected rank 2 raw, 38 = rank 3 raw, 39 = CTL0 after the
//! conversions, 40 = TIMER0 CNT and 41 = TIMER7 CNT read back to back after them, 42 = TIMER7 MOE
//! after the run (0).
//!
//! On every chip: 35 = ADC0 `ISQ` read back, 36 = 1 when it is not the configured sequence (`IL`
//! and each rank's channel in the right-aligned packing).
#![no_std]
#![no_main]

#[cfg(all(target_os = "none", feature = "gd32"))]
mod image {
    use cortex_m_rt::entry;
    use hal::clock::{self, ClockConfig};
    use hal::config::{
        AdcClockDiv, BreakConfig, ClockDiv, InjectedAdcConfig, InjectedChannel, OcMode, PwmAlign,
        PwmChannelConfig, PwmConfig, TimerTriggerLink, TrgoSource,
    };
    use hal::descriptor::DESCRIPTOR;
    use hal::reg::Reg32;
    use hal::{Chip, InjectedAdcController, PeriphLabel, PinAf, PinId, PwmTimer, TriggeredAdc};
    use hal_bench::{finish, put};
    use heapless::Vec;
    use panic_halt as _;

    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    /// commutation::ARR: 72 MHz / (2 x 2250) = 16 kHz centre-aligned.
    const PERIOD: u16 = 2250;
    const TRIGGER_COMPARE: u16 = PERIOD - 50;
    /// Whether the chip carries TIMER7, the 12-FET's second bridge.
    const TWO: bool = DESCRIPTOR.adv_timers == 2;
    /// The fleet's 28 on the one-timer boards; the 12-FET stores 32 (`twelve-fet.md` section 9).
    const DEAD_TIME: u8 = if TWO { 32 } else { 28 };
    const T0: u32 = 0x4001_2C00;
    /// TIMER7's base from the descriptor; never addressed where the chip lacks it.
    const T7: u32 = match DESCRIPTOR.addrs.get(PeriphLabel::Timer7) {
        Some(b) => b,
        None => 0,
    };
    const ADC0_ISQ: u32 = 0x4001_2438;
    /// The injected ranks as (port, pin, channel).
    const RANKS: &[(u8, u8, u8)] = if TWO {
        &[(2, 4, 14), (2, 5, 15), (0, 0, 0), (2, 3, 13)]
    } else {
        &[(0, 0, 0), (1, 0, 8)]
    };

    /// `ISQ` for [`RANKS`]: `IL` = ranks less one at 21:20, rank `r`'s channel at
    /// `15 - (IL - r) * 5` (`adc_inserted_channel_config`'s packing).
    const fn isq_expected() -> u32 {
        let il = RANKS.len() as u32 - 1;
        let mut v = il << 20;
        let mut r = 0;
        while r < RANKS.len() {
            v |= (RANKS[r].2 as u32) << (15 - (il - r as u32) * 5);
            r += 1;
        }
        v
    }

    fn config(timer: PeriphLabel) -> PwmConfig {
        let ch = || PwmChannelConfig {
            polarity: true,
            idle_high: false,
            idle_high_n: true,
        };
        PwmConfig {
            timer,
            channels: [ch(), ch(), ch()],
            period: PERIOD,
            prescaler: 0,
            dead_time: DEAD_TIME,
            brk: BreakConfig {
                enabled: false,
                level: false,
            },
            trigger_compare: TRIGGER_COMPARE,
            align: PwmAlign::Center1,
            arse: false,
            trigger_oc_mode: OcMode::Pwm1,
            trigger_ch_enable: true,
            crep: 1,
            ckdiv: ClockDiv::Div2,
            trgo_src: TrgoSource::Ch3Compare,
        }
    }

    /// A timer's CTL0, CTL1, CHCTL0, CHCTL1, CHCTL2, PSC, CAR, CREP, CH3CV, CCHP.
    fn timer_regs(base: u32) -> [u32; 10] {
        let r = |o: u32| Reg32::new(base, o).read();
        [
            r(0x00),
            r(0x04),
            r(0x18),
            r(0x1C),
            r(0x20),
            r(0x28),
            r(0x2C),
            r(0x30),
            r(0x40),
            r(0x44),
        ]
    }

    /// The checks both timers take, at the same bits: 0 CAR, 1 PSC, 2 dead time, 3 MOE set,
    /// 5 centre-aligned mode 1.
    fn time_base_fail(regs: &[u32; 10]) -> u32 {
        let [ctl0, _, _, _, _, psc, car, _, _, cchp] = *regs;
        u32::from(car != u32::from(PERIOD))
            | u32::from(psc != 0) << 1
            | u32::from(cchp & 0xFF != u32::from(DEAD_TIME)) << 2
            | u32::from(cchp & (1 << 15) != 0) << 3
            | u32::from(ctl0 & (0b11 << 5) != 0b01 << 5) << 5
    }

    #[entry]
    fn main() -> ! {
        hal::debug_hold_timers();
        let Some(chip) = Chip::take() else { finish(5) };
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            finish(5);
        }
        hal::irq::apply_motor_era_priorities(chip.irq());
        let rcu = chip.rcu_base().unwrap_or(0);
        let _ = clock::enable_timer(rcu, chip.clock(), PeriphLabel::Timer0);
        if TWO {
            let _ = clock::enable_timer(rcu, chip.clock(), PeriphLabel::Timer7);
        }

        let Ok(timer) = PwmTimer::configure(&chip, &config(PeriphLabel::Timer0)) else {
            finish(5)
        };
        let slave = if TWO {
            match PwmTimer::configure(&chip, &config(PeriphLabel::Timer7)) {
                Ok(t) => Some(t),
                Err(_) => finish(5),
            }
        } else {
            None
        };
        timer.configure_trigger(TRIGGER_COMPARE, OcMode::Pwm1, true, TrgoSource::Ch3Compare);
        if let Some(t7) = slave {
            if t7.follow(&timer, TRIGGER_COMPARE).is_err() {
                finish(5);
            }
        }
        if !TWO {
            // The TIMER0 gate set, PA8-10 and PB13-15, at AF2 on the F1x0 mux (the F10x default
            // remap reads no AF).
            for (port, pin) in [(0, 8), (0, 9), (0, 10), (1, 13), (1, 14), (1, 15)] {
                chip.gate_pin(PinAf { port, pin, af: 2 });
            }
        }

        let regs = timer_regs(T0);
        for (i, v) in regs.into_iter().enumerate() {
            put(2 + i, v);
        }
        let [_, ctl1, _, _, chctl2, _, _, _, ch3cv, _] = regs;
        let mut fail = time_base_fail(&regs);
        fail |= u32::from(ch3cv != u32::from(TRIGGER_COMPARE)) << 4;
        fail |= u32::from(ctl1 & (0b111 << 4) != 7 << 4) << 6;
        // CHCTL2 is recorded, not judged: the bring-up enables the six outputs and the firmware's
        // period ISR coasts them; with POEN clear neither reaches a gate.
        let _ = chctl2;
        put(12, fail);

        let mut fail7 = 0u32;
        if TWO {
            let regs7 = timer_regs(T7);
            for (i, v) in regs7.into_iter().enumerate() {
                put(22 + i, v);
            }
            let smcfg = Reg32::new(T7, 0x08).read();
            let cnt = Reg32::new(T7, 0x24).read();
            put(32, smcfg);
            put(33, cnt);
            fail7 = time_base_fail(&regs7);
            fail7 |= u32::from(regs7[7] != 1) << 7;
            fail7 |= u32::from(smcfg & 0x77 != 0x06) << 8;
            fail7 |= u32::from(cnt != u32::from(TRIGGER_COMPARE)) << 9;
            fail7 |= u32::from(regs7[0] & 1 != 0) << 10;
        }

        let _ = clock::enable_adc(rcu, chip.clock(), PeriphLabel::Adc0);
        let mut chans: Vec<InjectedChannel, 4> = Vec::new();
        for &(port, pin, channel) in RANKS {
            chip.analog_pin(PinId { port, pin });
            let _ = chans.push(InjectedChannel {
                channel,
                sample_time: 1,
            });
        }
        let inj_cfg = InjectedAdcConfig {
            adc: PeriphLabel::Adc0,
            channels: chans,
            left_aligned: true,
            trigger_timer: PeriphLabel::Timer0,
            trigger_link: TimerTriggerLink::Trgo,
            clock_div: AdcClockDiv::Div6,
        };
        let mut eoic = false;
        match InjectedAdcController::new().configure(&chip, &inj_cfg) {
            Ok(inj) => {
                put(13, 0);
                timer.enable_counter();
                for _ in 0..4 {
                    inj.clear_eoic();
                    inj.reassert_trigger_enable();
                    let mut spins = 200_000u32;
                    while spins > 0 && !inj.eoic() {
                        spins -= 1;
                    }
                    eoic = spins > 0;
                    if !eoic {
                        break;
                    }
                }
                let s = inj.read_injected();
                put(15, u32::from(s[0]));
                put(16, u32::from(s[1]));
                if TWO {
                    put(37, u32::from(s[2]));
                    put(38, u32::from(s[3]));
                }
            }
            Err(_) => put(13, 1),
        }
        put(14, u32::from(eoic));
        let isq = Reg32::new(ADC0_ISQ, 0).read();
        let isq_wrong = isq != isq_expected();
        put(35, isq);
        put(36, u32::from(isq_wrong));

        let mut moe7 = 0;
        if TWO {
            let ctl0 = Reg32::new(T7, 0x00).read();
            put(39, ctl0);
            fail7 |= u32::from(ctl0 & 1 == 0) << 11;
            let cnt0 = Reg32::new(T0, 0x24).read();
            let cnt7 = Reg32::new(T7, 0x24).read();
            put(40, cnt0);
            put(41, cnt7);
            put(34, fail7);
            moe7 = (Reg32::new(T7, 0x44).read() >> 15) & 1;
            put(42, moe7);
        }

        let period = hal::irq::period_irq(chip.irq()) as u32;
        let ipr = (Reg32::new(0xE000_E400, (period / 4) * 4).read() >> ((period % 4) * 8)) & 0xFF;
        let shpr3 = Reg32::new(0xE000_ED20, 0).read() >> 24;
        put(17, ipr);
        put(18, shpr3);
        let prio = ipr == 0x10 && shpr3 == 0xF0;
        put(19, u32::from(prio));
        let moe = (Reg32::new(T0, 0x44).read() >> 15) & 1;
        put(20, moe);
        let ok = fail == 0 && eoic && prio && moe == 0 && !isq_wrong && fail7 == 0 && moe7 == 0;
        put(21, u32::from(ok));
        finish(5)
    }
}

/// The MM32SPIN05 motor-verify: TIM1 configured as the GD32 TIMER0 is (the same register model,
/// `timer/*_tim1_mm32` golden traces), the ADC's timer-triggered scan of PA2 (channel 2) and PA6
/// (channel 6), the PFOP part's phase-current op-amp outputs, started by TIM1's TRGO, and the
/// Cortex-M0 priority ladder read back (period vector `0x00`, SysTick `0xC0`).
///
/// The gate pins are NOT routed to TIM1 here, unlike the GD32 build: the Gen2.4.2 board's gate pins
/// and the low-side gate polarity are not verified (`specs/agent-runs/mm32-poc-report.md`), so they
/// stay in their reset state, inputs. With `MOE` clear the timer reaches no pin either way.
///
/// Result words as the GD32 motor-verify (2..21), with 19 = 1 for `0x00` and `0xC0`.
#[cfg(all(target_os = "none", feature = "mm32spin05"))]
mod image {
    use cortex_m_rt::entry;
    use hal::clock::{self, ClockConfig};
    use hal::config::{
        AdcClockDiv, BreakConfig, ClockDiv, InjectedAdcConfig, InjectedChannel, OcMode, PwmAlign,
        PwmChannelConfig, PwmConfig, TimerTriggerLink, TrgoSource,
    };
    use hal::reg::Reg32;
    use hal::{Chip, InjectedAdcController, PeriphLabel, PinId, PwmTimer, TriggeredAdc};
    use hal_bench::{finish, put};
    use heapless::Vec;
    use panic_halt as _;

    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_HSI;
    const PERIOD: u16 = 2250;
    const TRIGGER_COMPARE: u16 = PERIOD - 50;
    const DEAD_TIME: u8 = 28;
    const T1: u32 = 0x4001_2C00;

    #[entry]
    fn main() -> ! {
        let Some(chip) = Chip::take() else { finish(5) };
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            finish(5);
        }
        hal::irq::apply_motor_era_priorities(chip.irq());
        let rcu = chip.rcu_base().unwrap_or(0);
        let _ = clock::enable_timer(rcu, chip.clock(), PeriphLabel::Timer0);

        let ch = || PwmChannelConfig {
            polarity: true,
            idle_high: false,
            idle_high_n: true,
        };
        let cfg = PwmConfig {
            timer: PeriphLabel::Timer0,
            channels: [ch(), ch(), ch()],
            period: PERIOD,
            prescaler: 0,
            dead_time: DEAD_TIME,
            brk: BreakConfig {
                enabled: false,
                level: false,
            },
            trigger_compare: TRIGGER_COMPARE,
            align: PwmAlign::Center1,
            arse: false,
            trigger_oc_mode: OcMode::Pwm1,
            trigger_ch_enable: true,
            crep: 1,
            ckdiv: ClockDiv::Div2,
            trgo_src: TrgoSource::Ch3Compare,
        };
        let Ok(timer) = PwmTimer::configure(&chip, &cfg) else {
            finish(5)
        };
        timer.configure_trigger(TRIGGER_COMPARE, OcMode::Pwm1, true, TrgoSource::Ch3Compare);

        let r = |o: u32| Reg32::new(T1, o).read();
        let (ctl0, ctl1, chctl0, chctl1, chctl2) = (r(0x00), r(0x04), r(0x18), r(0x1C), r(0x20));
        let (psc, car, crep, ch3cv, cchp) = (r(0x28), r(0x2C), r(0x30), r(0x40), r(0x44));
        for (i, v) in [
            ctl0, ctl1, chctl0, chctl1, chctl2, psc, car, crep, ch3cv, cchp,
        ]
        .into_iter()
        .enumerate()
        {
            put(2 + i, v);
        }
        let mut fail = 0u32;
        fail |= u32::from(car != u32::from(PERIOD));
        fail |= u32::from(psc != 0) << 1;
        fail |= u32::from(cchp & 0xFF != u32::from(DEAD_TIME)) << 2;
        fail |= u32::from(cchp & (1 << 15) != 0) << 3;
        fail |= u32::from(ch3cv != u32::from(TRIGGER_COMPARE)) << 4;
        fail |= u32::from(ctl0 & (0b11 << 5) != 0b01 << 5) << 5;
        fail |= u32::from(ctl1 & (0b111 << 4) != 7 << 4) << 6;
        let _ = chctl2;
        put(12, fail);

        let _ = clock::enable_adc(rcu, chip.clock(), PeriphLabel::Adc0);
        for pin in [2u8, 6] {
            chip.analog_pin(PinId { port: 0, pin });
        }
        let mut chans: Vec<InjectedChannel, 4> = Vec::new();
        for channel in [2u8, 6] {
            let _ = chans.push(InjectedChannel {
                channel,
                sample_time: 1,
            });
        }
        let inj_cfg = InjectedAdcConfig {
            adc: PeriphLabel::Adc0,
            channels: chans,
            left_aligned: true,
            trigger_timer: PeriphLabel::Timer0,
            trigger_link: TimerTriggerLink::Trgo,
            clock_div: AdcClockDiv::Div6,
        };
        let mut eoic = false;
        match InjectedAdcController::new().configure(&chip, &inj_cfg) {
            Ok(inj) => {
                put(13, 0);
                timer.enable_counter();
                for _ in 0..4 {
                    inj.clear_eoic();
                    inj.reassert_trigger_enable();
                    let mut spins = 200_000u32;
                    while spins > 0 && !inj.eoic() {
                        spins -= 1;
                    }
                    eoic = spins > 0;
                    if !eoic {
                        break;
                    }
                }
                let s = inj.read_injected();
                put(15, u32::from(s[0]));
                put(16, u32::from(s[1]));
            }
            Err(_) => put(13, 1),
        }
        put(14, u32::from(eoic));

        let period = hal::irq::period_irq(chip.irq()) as u32;
        let ipr = (Reg32::new(0xE000_E400, (period / 4) * 4).read() >> ((period % 4) * 8)) & 0xFF;
        let shpr3 = Reg32::new(0xE000_ED20, 0).read() >> 24;
        put(17, ipr);
        put(18, shpr3);
        let prio = ipr == 0x00 && shpr3 == 0xC0;
        put(19, u32::from(prio));
        let moe = (Reg32::new(T1, 0x44).read() >> 15) & 1;
        put(20, moe);
        put(21, u32::from(fail == 0 && eoic && prio && moe == 0));
        finish(5)
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
