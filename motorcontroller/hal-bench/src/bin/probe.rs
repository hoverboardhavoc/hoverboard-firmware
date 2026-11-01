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
//! Result words: 2 = DBG_IDCODE, 3 = flash-size word, 4 = descriptor flash KiB, 5 = mismatches (a
//! bit per carried candidate that refused the write-back, and bit 8 when the flash-size word's KiB
//! differ from the descriptor's), 6..8 = uid words, then
//! per candidate `i` (0..7) at 8 + 4i: [label, sticky | writeback << 2 | present << 3, scratch
//! reset value, descriptor-carries].
#![no_std]
#![no_main]

#[cfg(target_os = "none")]
mod image {
    use cortex_m_rt::entry;
    use hal::reg::Reg32;
    use hal::{descriptor::DESCRIPTOR, PeriphLabel};
    use hal_bench::{finish, put};
    use panic_halt as _;

    const RCU: u32 = 0x4002_1000;
    const APB2EN: u32 = 0x18;
    const APB1EN: u32 = 0x1C;

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
        put(5, mismatch);
        finish(1)
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
