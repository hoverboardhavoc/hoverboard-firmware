// SPDX-License-Identifier: GPL-3.0-or-later
//! `motor-verify`: the motor hot path configured as the firmware configures it, then read back
//! register by register, with the outputs disarmed (specs/hal.md section 8.4).
//!
//! TIMER0 with the fleet's complementary set (PA8-10 / PB13-15), 16 kHz centre-aligned at the
//! 72 MHz tree, dead time 28, the channel 3 compare on TRGO; the injected ADC group on PA0 (ch 0)
//! and PB0 (ch 8) triggered by it; the counter started and one injected end of conversion awaited.
//! MOE is never set: no `ArmGate` is made, so the gate driver inputs sit at their idle levels, the
//! disarmed state the firmware boots into. The motor-era NVIC priorities are applied and read back.
//!
//! Result words: 2 = CTL0, 3 = CTL1, 4 = CHCTL0, 5 = CHCTL1, 6 = CHCTL2, 7 = PSC, 8 = CAR,
//! 9 = CREP, 10 = CH3CV, 11 = CCHP, 12 = the TIMER0 checks (a bit per failed check: 0 CAR, 1 PSC,
//! 2 dead time, 3 MOE set, 4 CH3 compare, 5 centre-aligned mode 1, 6 TRGO = CH3 compare), 13 = injected bring-up (0 ok), 14 = EOIC seen (1), 15 = injected ch0
//! raw, 16 = injected ch1 raw, 17 = NVIC priority of the period IRQ, 18 = SHPR3 top byte (SysTick),
//! 19 = priority verdict (1 = 0x10 and 0xF0), 20 = MOE after the run (0), 21 = verdict.
#![no_std]
#![no_main]

#[cfg(target_os = "none")]
mod image {
    use cortex_m_rt::entry;
    use hal::clock::{self, ClockConfig};
    use hal::config::{
        AdcClockDiv, BreakConfig, ClockDiv, InjectedAdcConfig, InjectedChannel, OcMode, PwmAlign,
        PwmChannelConfig, PwmConfig, TimerTriggerLink, TrgoSource,
    };
    use hal::reg::Reg32;
    use hal::{Chip, InjectedAdcController, PeriphLabel, PwmTimer, TriggeredAdc};
    use hal_bench::{finish, put};
    use heapless::Vec;
    use panic_halt as _;

    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    /// commutation::ARR: 72 MHz / (2 x 2250) = 16 kHz centre-aligned.
    const PERIOD: u16 = 2250;
    const TRIGGER_COMPARE: u16 = PERIOD - 50;
    const DEAD_TIME: u8 = 28;
    const T0: u32 = 0x4001_2C00;

    #[entry]
    fn main() -> ! {
        let Some(chip) = Chip::take() else { finish(5) };
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            finish(5);
        }
        hal::irq::apply_motor_era_priorities(chip.irq());
        let rcu = chip.rcu_base().unwrap_or(0);
        let _ = clock::enable_timer(rcu, chip.clock(), PeriphLabel::Timer0);

        let ch = |h: u8, l: u8| PwmChannelConfig {
            high: h,
            low: l,
            polarity: true,
            idle_high: false,
            idle_high_n: true,
        };
        let cfg = PwmConfig {
            timer: PeriphLabel::Timer0,
            channels: [ch(0x08, 0x1D), ch(0x09, 0x1E), ch(0x0A, 0x1F)],
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
        for pin in [0x08u8, 0x09, 0x0A, 0x1D, 0x1E, 0x1F] {
            if chip.route_advanced_pwm_pin(pin).is_err() {
                finish(5);
            }
        }

        let r = |o: u32| Reg32::new(T0, o).read();
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
        // CHCTL2 is recorded, not judged: the bring-up enables the six outputs and the firmware's
        // period ISR coasts them; with POEN clear neither reaches a gate.
        let _ = chctl2;
        put(12, fail);

        let _ = clock::enable_adc(rcu, chip.clock(), PeriphLabel::Adc0);
        for pin in [0x00u8, 0x10] {
            let port = if pin >> 4 == 0 {
                PeriphLabel::Gpioa
            } else {
                PeriphLabel::Gpiob
            };
            let _ = chip.analog_pin(port, pin & 0x0F);
        }
        let mut chans: Vec<InjectedChannel, 4> = Vec::new();
        let _ = chans.push(InjectedChannel {
            channel: 0,
            sample_time: 1,
        });
        let _ = chans.push(InjectedChannel {
            channel: 8,
            sample_time: 1,
        });
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
        let prio = ipr == 0x10 && shpr3 == 0xF0;
        put(19, u32::from(prio));
        let moe = (Reg32::new(T0, 0x44).read() >> 15) & 1;
        put(20, moe);
        put(21, u32::from(fail == 0 && eoic && prio && moe == 0));
        finish(5)
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
