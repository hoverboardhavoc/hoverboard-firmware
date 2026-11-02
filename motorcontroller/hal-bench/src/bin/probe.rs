// SPDX-License-Identifier: GPL-3.0-or-later
//! `probe`: the compiled descriptor's facts against the silicon (specs/hal.md section 8.4).
//!
//! Reads the device ID (`DBG_IDCODE`), the flash-size word (`0x1FFF_F7E0`) and the factory uid, and
//! checks each candidate instance (TIMER0, TIMER7, ADC0..2, I2C1, USART2): its RCU enable bit is
//! set and read back, and a block the descriptor carries must then take a write-back of a
//! side-effect-free register (TIMER PSC, ADC WDLT, I2C RT, USART GP), restored afterwards.
//!
//! A block the descriptor lacks is never addressed, so its absence is not measured here: on the
//! GD32F130C8 the reserved enable bits read back 1 (measured), so the bit cannot tell either. The first version of this validator read
//! every candidate's base and scratch registers behind a BusFault handler, as runtime-hal's probe
//! did with its scratch register; on the rover master (GD32F103C8) that left the AHB stalled, every
//! debug access answering WAIT, instead of faulting (specs/agent-runs/hal-step2-report.md).
//! Nothing here starts a counter, sets a channel or output enable, or touches MOE.
//!
//! The debug holds are set first (`hal::debug_hold_timers`: TIMER0's, and TIMER7's where the
//! descriptor carries it) and `DBG_CTL0` read back, so a halt mid-run freezes the timers this run
//! clocks.
//!
//! Result words: 2 = DBG_IDCODE, 3 = flash-size word, 4 = descriptor flash KiB, 5 = mismatches (a
//! bit per carried candidate that refused the write-back, bit 8 when the flash-size word's KiB
//! differ from the descriptor's, bit 9 when the timer holds did not read back), 6..8 = uid words,
//! then per candidate `i` (0..7) at 8 + 4i: [label, sticky | writeback << 2 | present << 3, scratch
//! reset value, descriptor-carries]; 40 = `DBG_CTL0` after the holds (want bit 10, and bit 17 on a
//! chip with TIMER7), 41 = its value before.
#![no_std]
#![no_main]

#[cfg(all(target_os = "none", feature = "gd32"))]
mod image {
    use cortex_m_rt::entry;
    use hal::reg::Reg32;
    use hal::{descriptor::DESCRIPTOR, PeriphLabel};
    use hal_bench::{finish, put};
    use panic_halt as _;

    const RCU: u32 = 0x4002_1000;
    const APB2EN: u32 = 0x18;
    const APB1EN: u32 = 0x1C;
    /// `DBG_CTL0` (GD32F10x User Manual Rev2.6 section 10.4.2; the F1x0 has the same register).
    const DBG_CTL0: u32 = 0xE004_2004;
    /// The holds `hal::debug_hold_timers` sets: `TIMER0_HOLD` (bit 10), and `TIMER7_HOLD` (bit 17)
    /// where the descriptor carries TIMER7.
    const HOLDS: u32 = if DESCRIPTOR.adv_timers == 2 {
        1 << 10 | 1 << 17
    } else {
        1 << 10
    };

    /// One candidate: label, base, RCU enable register and bit, scratch register offset, the
    /// pattern written there, and its reset value.
    struct Candidate {
        label: PeriphLabel,
        base: u32,
        en_reg: u32,
        en_bit: u32,
        scratch: u32,
        pattern: u32,
    }

    const CANDIDATES: [Candidate; 7] = [
        Candidate {
            label: PeriphLabel::Timer0,
            base: 0x4001_2C00,
            en_reg: APB2EN,
            en_bit: 11,
            scratch: 0x28,
            pattern: 0x00A5,
        },
        Candidate {
            label: PeriphLabel::Timer7,
            base: 0x4001_3400,
            en_reg: APB2EN,
            en_bit: 13,
            scratch: 0x28,
            pattern: 0x00A5,
        },
        Candidate {
            label: PeriphLabel::Adc0,
            base: 0x4001_2400,
            en_reg: APB2EN,
            en_bit: 9,
            scratch: 0x28,
            pattern: 0x0A5A,
        },
        Candidate {
            label: PeriphLabel::Adc1,
            base: 0x4001_2800,
            en_reg: APB2EN,
            en_bit: 10,
            scratch: 0x28,
            pattern: 0x0A5A,
        },
        Candidate {
            label: PeriphLabel::Adc2,
            base: 0x4001_3C00,
            en_reg: APB2EN,
            en_bit: 15,
            scratch: 0x28,
            pattern: 0x0A5A,
        },
        Candidate {
            label: PeriphLabel::I2c1,
            base: 0x4000_5800,
            en_reg: APB1EN,
            en_bit: 22,
            scratch: 0x20,
            pattern: 0x0011,
        },
        Candidate {
            label: PeriphLabel::Usart2,
            base: 0x4000_4800,
            en_reg: APB1EN,
            en_bit: 18,
            scratch: 0x18,
            pattern: 0x0011,
        },
    ];

    /// The first two words of the factory uid at the descriptor's base.
    fn chip_uid() -> [u32; 2] {
        let base = DESCRIPTOR
            .addrs
            .get(PeriphLabel::UniqueId)
            .unwrap_or(0x1FFF_F7E8);
        [Reg32::new(base, 0).read(), Reg32::new(base, 4).read()]
    }

    #[entry]
    fn main() -> ! {
        let dbg = Reg32::new(DBG_CTL0, 0);
        put(41, dbg.read());
        hal::debug_hold_timers();
        let held = dbg.read();
        put(40, held);

        put(2, Reg32::new(0xE004_2000, 0).read());
        put(3, Reg32::new(0x1FFF_F7E0, 0).read());
        put(4, u32::from(DESCRIPTOR.flash_kib));

        let uid = chip_uid();
        put(6, uid[0]);
        put(7, uid[1]);

        let mut mismatch = 0u32;
        for (i, c) in CANDIDATES.iter().enumerate() {
            let en = Reg32::new(RCU, c.en_reg);
            en.set_bits(1 << c.en_bit);
            let sticky = en.read() & (1 << c.en_bit) != 0;
            let carried = DESCRIPTOR.addrs.get(c.label).is_some();
            let mut writeback = false;
            let mut reset = 0;
            // Only a block both clocked and in the descriptor is addressed: a sticky bit for a
            // block the descriptor lacks is the mismatch itself, found without touching it.
            if sticky && carried {
                let r = Reg32::new(c.base, c.scratch);
                reset = r.read();
                r.write(c.pattern);
                writeback = r.read() == c.pattern;
                r.write(reset);
            } else {
                // Not clocked, never addressed: put the bit back as it was.
                en.clear_bits(1 << c.en_bit);
            }
            // A carried block must take the write-back; a lacking block is not measured (on the
            // GD32F130C8 the reserved enable bits stick, so the bit says nothing about it).
            let present = carried && writeback;
            if carried && !writeback {
                mismatch |= 1 << i;
            }
            let o = 8 + 4 * i;
            put(o, c.label as u32);
            put(
                o + 1,
                u32::from(sticky) | u32::from(writeback) << 2 | u32::from(present) << 3,
            );
            put(o + 2, reset);
            put(o + 3, u32::from(carried));
        }
        if Reg32::new(0x1FFF_F7E0, 0).read() & 0xFFFF != u32::from(DESCRIPTOR.flash_kib) {
            mismatch |= 1 << 8;
        }
        if held & HOLDS != HOLDS {
            mismatch |= 1 << 9;
        }
        put(5, mismatch);
        finish(1)
    }
}

/// The MM32SPIN05 probe. The part has no flash-size register, so word 3 is 0 and the flash size
/// is the descriptor's alone (the alias test of `specs/bench-work/mm32/README.md` measured it). The
/// device ID is `DBGMCU_IDCODE` (`0x4001_3400`, the debug block `hal::dbg` drives). The candidates
/// are the blocks the descriptor carries, each with a register that takes a write-back without a
/// side effect while the block is off: TIM1 and TIM2 `PSC`, ADC1 `ADCMPR` (the window thresholds,
/// window off), I2C1 `SAR` (16-bit, the block disabled), UART1 and UART2 `RXAR` (the receive
/// address match), DMA1 channel 1 `CMAR`.
///
/// Result words as the GD32 probe, and: 40 = `DBGMCU_CR` after `hal::debug_hold_timers` and
/// `FreeWatchdog::freeze_on_debug_halt` (want bits 8 and 10), 41 = its value before. Mismatch bits:
/// a carried candidate that refused the write-back (bit `i`), the device ID not `0xCC44_60B1`
/// (bit 8), the hold bits not read back (bit 9).
#[cfg(all(target_os = "none", feature = "mm32spin05"))]
mod image {
    use cortex_m_rt::entry;
    use hal::reg::{Reg16, Reg32};
    use hal::{descriptor::DESCRIPTOR, FreeWatchdog, PeriphLabel};
    use hal_bench::{finish, put};
    use panic_halt as _;

    const RCC: u32 = 0x4002_1000;
    const AHBENR: u32 = 0x14;
    const APB2ENR: u32 = 0x18;
    const APB1ENR: u32 = 0x1C;
    const DBGMCU: u32 = 0x4001_3400;
    const MM32SPIN05_ID: u32 = 0xCC44_60B1;

    /// One candidate: label, RCC enable register and bit, scratch register offset and width, the
    /// pattern written there.
    struct Candidate {
        label: PeriphLabel,
        en_reg: u32,
        en_bit: u32,
        scratch: u32,
        half: bool,
        pattern: u32,
    }

    const CANDIDATES: [Candidate; 7] = [
        Candidate {
            label: PeriphLabel::Timer0,
            en_reg: APB2ENR,
            en_bit: 11,
            scratch: 0x28,
            half: false,
            pattern: 0x00A5,
        },
        Candidate {
            label: PeriphLabel::Timer1,
            en_reg: APB1ENR,
            en_bit: 0,
            scratch: 0x28,
            half: false,
            pattern: 0x00A5,
        },
        Candidate {
            label: PeriphLabel::Adc0,
            en_reg: APB2ENR,
            en_bit: 9,
            scratch: 0x10,
            half: false,
            pattern: 0x0A5A_05A5,
        },
        Candidate {
            label: PeriphLabel::I2c0,
            en_reg: APB1ENR,
            en_bit: 21,
            scratch: 0x08,
            half: true,
            pattern: 0x0011,
        },
        Candidate {
            label: PeriphLabel::Usart0,
            en_reg: APB2ENR,
            en_bit: 14,
            scratch: 0x28,
            half: false,
            pattern: 0x0011,
        },
        Candidate {
            label: PeriphLabel::Usart1,
            en_reg: APB1ENR,
            en_bit: 17,
            scratch: 0x28,
            half: false,
            pattern: 0x0011,
        },
        Candidate {
            label: PeriphLabel::Dma0,
            en_reg: AHBENR,
            en_bit: 0,
            scratch: 0x14,
            half: false,
            pattern: 0x2000_0A50,
        },
    ];

    fn read(base: u32, c: &Candidate) -> u32 {
        if c.half {
            u32::from(Reg16::new(base, c.scratch).read())
        } else {
            Reg32::new(base, c.scratch).read()
        }
    }

    fn write(base: u32, c: &Candidate, v: u32) {
        if c.half {
            Reg16::new(base, c.scratch).write(v as u16);
        } else {
            Reg32::new(base, c.scratch).write(v);
        }
    }

    #[entry]
    fn main() -> ! {
        let id = Reg32::new(DBGMCU, 0).read();
        put(2, id);
        put(3, 0);
        put(4, u32::from(DESCRIPTOR.flash_kib));
        let uid = DESCRIPTOR
            .addrs
            .get(PeriphLabel::UniqueId)
            .unwrap_or(0x1FFF_F7E8);
        put(6, Reg32::new(uid, 0).read());
        put(7, Reg32::new(uid, 4).read());

        let mut mismatch = 0u32;
        for (i, c) in CANDIDATES.iter().enumerate() {
            let en = Reg32::new(RCC, c.en_reg);
            en.set_bits(1 << c.en_bit);
            let sticky = en.read() & (1 << c.en_bit) != 0;
            let base = DESCRIPTOR.addrs.get(c.label);
            let (mut writeback, mut reset) = (false, 0);
            if let (true, Some(base)) = (sticky, base) {
                reset = read(base, c);
                write(base, c, c.pattern);
                writeback = read(base, c) == c.pattern;
                write(base, c, reset);
            }
            let carried = base.is_some();
            if carried && !writeback {
                mismatch |= 1 << i;
            }
            let o = 8 + 4 * i;
            put(o, c.label as u32);
            put(
                o + 1,
                u32::from(sticky)
                    | u32::from(writeback) << 2
                    | u32::from(carried && writeback) << 3,
            );
            put(o + 2, reset);
            put(o + 3, u32::from(carried));
        }
        if id != MM32SPIN05_ID {
            mismatch |= 1 << 8;
        }

        let cr = Reg32::new(DBGMCU, 0x04);
        put(41, cr.read());
        hal::debug_hold_timers();
        FreeWatchdog::freeze_on_debug_halt();
        let held = cr.read();
        put(40, held);
        if held & (1 << 8 | 1 << 10) != (1 << 8 | 1 << 10) {
            mismatch |= 1 << 9;
        }
        put(5, mismatch);
        finish(1)
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
