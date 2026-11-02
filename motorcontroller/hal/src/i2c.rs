// SPDX-License-Identifier: GPL-3.0-or-later
//! The I2C master: bring-up, polled transfers and the `embedded-hal` 1.0 `i2c::I2c` trait.
//!
//! Two register models (`hal_i2c`). The GD32F10x and GD32F1x0 carry the same event-based block
//! (`gd32f10x_i2c.h` and `gd32f1x0_i2c.h` agree on every offset and bit used here), the `gd` model
//! in this file. The MM32SPIN05 carries a DesignWare-style block with command and data FIFOs, the
//! `mm32` model in `i2c/mm32.rs`, which shares this file's bus clear, time budget and bus modes.
//! The instance is a base from the descriptor; the bus clock is APB1 from the [`ClockConfig`].
//!
//! The rest of this header is the `gd` model.
//!
//! | Register | Offset | Used here |
//! |---|---|---|
//! | `CTL0` | `0x00` | `I2CEN` 0, `SMBEN` 1, `START` 8, `STOP` 9, `ACKEN` 10, `POAP` 11, `SRESET` 15 |
//! | `CTL1` | `0x04` | `I2CCLK[6:0]`, the APB1 clock in MHz |
//! | `SADDR0` | `0x08` | the own address (7-bit format) |
//! | `DATA` | `0x10` | address byte, transmit and receive |
//! | `STAT0` | `0x14` | `SBSEND` 0, `ADDSEND` 1, `BTC` 2, `RBNE` 6, `TBE` 7, `BERR` 8, `LOSTARB` 9, `AERR` 10, `OUERR` 11 |
//! | `STAT1` | `0x18` | `MASTER` 0, `I2CBSY` 1 |
//! | `CKCFG` | `0x1C` | `CLKC[11:0]`, `DTCY` 14, `FAST` 15 |
//! | `RT` | `0x20` | `RISETIME[6:0]` |
//!
//! # Bring-up
//!
//! [`I2c::new`] clears a bus a slave holds low (the warm-reset case: a slave mid-read when the core
//! reset keeps SDA low until clocked out), routes SCL and SDA as AF open-drain, and programs the
//! block as the vendor's `i2c_clock_config`, `i2c_mode_addr_config`, `i2c_enable` and
//! `i2c_ack_config` leave it. The timing values are [`timing_for`], the vendor formula.
//!
//! # Transfers
//!
//! Transmit is the event handshake: `START`, `SBSEND`, address, `ADDSEND` (cleared by reading
//! `STAT0` then `STAT1`), each byte on `TBE`, a final `BTC`. Receive is the user manual's
//! "Solution B" (GD32F1x0 UM, "Programming model in master receiving mode"): every NACK and STOP
//! decision is made while the block stretches SCL, so interrupt latency in the polling loop cannot
//! misplace them. Solution A, which needs the software to answer within one byte time, wedged the
//! integrated firmware's 250 Hz IMU read on a GD32F130 under interrupt load (`runtime-hal`'s
//! 2026-07-18 bench finding: `CTL0` stuck at `I2CEN|START|STOP|ACKEN`, `I2CBSY` without `MASTER`).
//!
//! A failed transfer runs [`I2c::recover`] before its error returns: clear the sticky error flags,
//! STOP if still master, and if the block stays wedged, pulse `SRESET` and reprogram it. Without
//! that, one failure leaves every later transfer failing.
//!
//! # The time budget
//!
//! Every poll draws from one countdown per bus acquisition (a fresh START to its STOP, including a
//! repeated START), sized from the core clock so its bound is in microseconds:
//! [`worst_case_block_us`]. The firmware asserts in a `const` that twice that bound fits inside the
//! motor's demand-stale coast, because the IMU read is the control task's one blocking call and the
//! commutation ISR floats the phases when the control task goes quiet past the coast.

#[cfg(hal_i2c = "gd")]
use core::cell::Cell;

#[cfg(hal_i2c = "gd")]
use embedded_hal::i2c::{ErrorType, Operation, SevenBitAddress};

#[cfg(hal_i2c = "gd")]
use crate::addr::PeriphLabel;
#[cfg(hal_i2c = "gd")]
use crate::chip::Chip;
#[cfg(hal_i2c = "gd")]
use crate::clock::ClockConfig;
#[cfg(hal_i2c = "gd")]
use crate::error::{DescriptorError, I2cError};
#[cfg(hal_i2c = "gd")]
use crate::gpio::{Floating, Input, Pin};
use crate::gpio::{self, PinRole};
#[cfg(hal_i2c = "gd")]
use crate::pincap::{self, PinClaim};
#[cfg(hal_i2c = "gd")]
use crate::reg::Reg32;

#[cfg(hal_i2c = "mm32")]
#[path = "i2c/mm32.rs"]
mod mm32;
#[cfg(hal_i2c = "mm32")]
pub use mm32::I2c;

#[cfg(hal_i2c = "gd")]
const CTL0: u32 = 0x00;
#[cfg(hal_i2c = "gd")]
const CTL1: u32 = 0x04;
#[cfg(hal_i2c = "gd")]
const SADDR0: u32 = 0x08;
#[cfg(hal_i2c = "gd")]
const DATA: u32 = 0x10;
#[cfg(hal_i2c = "gd")]
const STAT0: u32 = 0x14;
#[cfg(hal_i2c = "gd")]
const STAT1: u32 = 0x18;
#[cfg(hal_i2c = "gd")]
const CKCFG: u32 = 0x1C;
#[cfg(hal_i2c = "gd")]
const RT: u32 = 0x20;

#[cfg(hal_i2c = "gd")]
const CTL0_I2CEN: u32 = 1 << 0;
#[cfg(hal_i2c = "gd")]
const CTL0_SMBEN: u32 = 1 << 1;
#[cfg(hal_i2c = "gd")]
const CTL0_START: u32 = 1 << 8;
#[cfg(hal_i2c = "gd")]
const CTL0_STOP: u32 = 1 << 9;
#[cfg(hal_i2c = "gd")]
const CTL0_ACKEN: u32 = 1 << 10;
/// With `POAP` set, `ACKEN` governs the next byte rather than the current one (Solution B, N = 2).
#[cfg(hal_i2c = "gd")]
const CTL0_POAP: u32 = 1 << 11;
#[cfg(hal_i2c = "gd")]
const CTL0_SRESET: u32 = 1 << 15;

#[cfg(hal_i2c = "gd")]
const CTL1_I2CCLK: u32 = 0x7F;
#[cfg(hal_i2c = "gd")]
const CKCFG_CLKC: u32 = 0xFFF;
#[cfg(hal_i2c = "gd")]
const CKCFG_DTCY: u32 = 1 << 14;
#[cfg(hal_i2c = "gd")]
const CKCFG_FAST: u32 = 1 << 15;
/// The `SADDR0` bits the vendor's `i2c_mode_addr_config` masks the address with.
#[cfg(hal_i2c = "gd")]
const SADDR0_MASK: u32 = 0x3FF;

#[cfg(hal_i2c = "gd")]
const STAT0_SBSEND: u32 = 1 << 0;
#[cfg(hal_i2c = "gd")]
const STAT0_ADDSEND: u32 = 1 << 1;
#[cfg(hal_i2c = "gd")]
const STAT0_BTC: u32 = 1 << 2;
#[cfg(hal_i2c = "gd")]
const STAT0_RBNE: u32 = 1 << 6;
#[cfg(hal_i2c = "gd")]
const STAT0_TBE: u32 = 1 << 7;
#[cfg(hal_i2c = "gd")]
const STAT0_BERR: u32 = 1 << 8;
#[cfg(hal_i2c = "gd")]
const STAT0_LOSTARB: u32 = 1 << 9;
#[cfg(hal_i2c = "gd")]
const STAT0_AERR: u32 = 1 << 10;
#[cfg(hal_i2c = "gd")]
const STAT0_OUERR: u32 = 1 << 11;
/// The sticky error flags, cleared by writing 0.
#[cfg(hal_i2c = "gd")]
const STAT0_ERRORS: u32 = STAT0_BERR | STAT0_LOSTARB | STAT0_AERR | STAT0_OUERR;

#[cfg(hal_i2c = "gd")]
const STAT1_MASTER: u32 = 1 << 0;
#[cfg(hal_i2c = "gd")]
const STAT1_I2CBSY: u32 = 1 << 1;

/// The vendor's clamp bounds for `I2CCLK` and the standard-mode `RT`.
#[cfg(hal_i2c = "gd")]
const I2CCLK_MAX: u32 = 0x48;
#[cfg(hal_i2c = "gd")]
const I2CCLK_MIN: u32 = 0x02;

/// The own address written to `SADDR0`, as the raw register value (the vendor call takes it
/// positioned, not as a 7-bit address to shift). A bus master is never addressed, so the value only
/// has to match what the vendor bring-up writes in the golden trace; `0x24` is `runtime-hal`'s
/// bench-validated value.
#[cfg(hal_i2c = "gd")]
const OWN_ADDR: u32 = 0x24;

/// SCL pulses a bus clear issues at most: a stuck byte's eight data clocks and its ACK clock (NXP
/// UM10204, section 3.1.16, "Bus clear").
const BUS_CLEAR_MAX_PULSES: u32 = 9;

/// Core cycles one poll iteration costs, as an upper bound.
///
/// The divisor that turns the microsecond budget into an iteration count, so the upper end of the
/// range is the safe one: an overstated cost understates the count and the real block ends early.
/// `runtime-hal` disassembled the loop in the release image at 21 core cycles (an APB1 load of
/// `STAT0`, the flag tests, the budget load and store, the back-branch), plus an APB1 bridge stall
/// and flash wait states that the listing cannot count. With the coast assertion the firmware makes,
/// the real loop could cost up to about 1.8 times the disassembled figure before the guarantee
/// fails.
pub const POLL_CYCLES: u32 = 32;

/// Microseconds one bus acquisition may spend polling.
///
/// Above a healthy 14-byte IMU burst (about 0.4 ms at 400 kHz, 1.7 ms at 100 kHz) by 15 and 3.5
/// times; below the firmware's 16 ms coast by enough that twice [`worst_case_block_us`] fits. A
/// longer transfer on a slow bus is split into several acquisitions rather than raising this,
/// because raising it spends the motor's coast.
pub const TXN_BUDGET_US: u32 = 6_000;

/// Poll iterations in [`TXN_BUDGET_US`] at `core_hz`, at least 1.
pub const fn poll_budget(core_hz: u32) -> u32 {
    let n = (core_hz / 1_000_000) * TXN_BUDGET_US / POLL_CYCLES;
    if n == 0 {
        1
    } else {
        n
    }
}

/// A tenth of [`poll_budget`], at least 1: the cap on the bus-idle wait before a fresh START (a
/// healthy bus is idle within a byte time of the last STOP), and the budget of the recovery that
/// runs after a failure, which has to work when the acquisition's budget is spent.
pub const fn idle_budget(core_hz: u32) -> u32 {
    let n = poll_budget(core_hz) / 10;
    if n == 0 {
        1
    } else {
        n
    }
}

/// The most microseconds one failing bus acquisition can block its caller, at core clock `core_hz`.
///
/// Every poll in an acquisition, including the bus-idle wait and the recovery that wait may run,
/// draws from the one [`poll_budget`]; the recovery after a failed transfer has its own
/// [`idle_budget`]. So the bound is `(poll_budget + idle_budget) * POLL_CYCLES` core cycles, which at
/// 72 MHz is `(13_500 + 1_350) * 32 / 72` = 6_600 us. A sub-MHz clock reports `u32::MAX` rather than
/// divide by zero.
pub const fn worst_case_block_us(core_hz: u32) -> u32 {
    let polls = poll_budget(core_hz) + idle_budget(core_hz);
    match (polls * POLL_CYCLES).checked_div(core_hz / 1_000_000) {
        Some(us) => us,
        None => u32::MAX,
    }
}

/// The fast-mode duty cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FastDuty {
    /// Tlow / Thigh = 2 (`DTCY` clear, the vendor's `I2C_DTCY_2`).
    Two,
    /// Tlow / Thigh = 16 / 9 (`DTCY` set, `I2C_DTCY_16_9`).
    SixteenNine,
}

/// The bus mode: the SCL frequency and, in fast mode, the duty cycle. As in the vendor library, the
/// frequency picks the mode: standard up to 100 kHz, fast above it, up to 400 kHz.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cMode {
    /// SCL frequency, Hz.
    pub speed_hz: u32,
    /// The fast-mode duty cycle; not consulted at 100 kHz and below.
    pub duty: FastDuty,
}

impl I2cMode {
    /// A standard-mode bus at `speed_hz` (at most 100 kHz).
    pub const fn standard(speed_hz: u32) -> Self {
        Self {
            speed_hz,
            duty: FastDuty::Two,
        }
    }

    /// A fast-mode bus at `speed_hz` (above 100 kHz, at most 400 kHz) with duty cycle `duty`.
    pub const fn fast(speed_hz: u32, duty: FastDuty) -> Self {
        Self { speed_hz, duty }
    }
}

/// The three timing register values.
#[cfg(hal_i2c = "gd")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct I2cTiming {
    /// `CTL1.I2CCLK`.
    i2cclk: u32,
    /// `CKCFG`.
    ckcfg: u32,
    /// `RT`.
    rt: u32,
}

/// The timing register values for APB1 clock `pclk1_hz` and `mode`, as the vendor's
/// `i2c_clock_config` computes them (the GD32F10x and GD32F1x0 versions are the same code), or
/// `None` for a speed of 0 or above 400 kHz, which the vendor call leaves unprogrammed.
///
/// - `I2CCLK = min(pclk1 / 1 MHz, 0x48)`.
/// - Standard (at most 100 kHz): `RT = clamp(pclk1 / 1 MHz + 1, 2, 0x48)`,
///   `CKCFG = max(pclk1 / (2 * speed), 4)`, masked to `CLKC`.
/// - Fast: `RT = I2CCLK * 300 / 1000 + 1`; `CLKC = pclk1 / (3 * speed)` at duty 2 or
///   `pclk1 / (25 * speed)` with `DTCY` at 16/9, forced to at least 1; `CKCFG = FAST | DTCY | CLKC`.
#[cfg(hal_i2c = "gd")]
const fn timing_for(pclk1_hz: u32, mode: I2cMode) -> Option<I2cTiming> {
    let speed = mode.speed_hz;
    if speed == 0 || speed > 400_000 {
        return None;
    }
    let mhz = pclk1_hz / 1_000_000;
    let i2cclk = if mhz >= I2CCLK_MAX { I2CCLK_MAX } else { mhz };
    if speed <= 100_000 {
        let rise = mhz + 1;
        let rt = if rise >= I2CCLK_MAX {
            I2CCLK_MAX
        } else if rise <= I2CCLK_MIN {
            I2CCLK_MIN
        } else {
            rise
        };
        let clkc = pclk1_hz / (speed * 2);
        let clkc = if clkc < 4 { 4 } else { clkc };
        Some(I2cTiming {
            i2cclk,
            ckcfg: clkc & CKCFG_CLKC,
            rt,
        })
    } else {
        let rt = i2cclk * 300 / 1000 + 1;
        let (clkc, dtcy) = match mode.duty {
            FastDuty::Two => (pclk1_hz / (speed * 3), 0),
            FastDuty::SixteenNine => (pclk1_hz / (speed * 25), CKCFG_DTCY),
        };
        let clkc = if clkc & CKCFG_CLKC == 0 {
            clkc | 1
        } else {
            clkc
        };
        Some(I2cTiming {
            i2cclk,
            ckcfg: CKCFG_FAST | dtcy | clkc,
            rt,
        })
    }
}

/// Clock a stuck slave off the bus through pin closures, and return the SCL pulses issued.
///
/// `drive_scl(true)` and `drive_sda(true)` release the open-drain line; `false` pulls it low. With
/// both lines released, an SDA that reads high is a clean bus and nothing more happens. Otherwise
/// SCL is pulsed until the slave releases SDA, at most [`BUS_CLEAR_MAX_PULSES`] times, and a STOP
/// (SDA rising while SCL is high) returns the slave to idle.
fn bus_clear(
    mut read_sda: impl FnMut() -> bool,
    mut drive_scl: impl FnMut(bool),
    mut drive_sda: impl FnMut(bool),
    mut settle: impl FnMut(),
) -> u32 {
    drive_scl(true);
    drive_sda(true);
    settle();
    if read_sda() {
        return 0;
    }
    let mut pulses = 0;
    while pulses < BUS_CLEAR_MAX_PULSES && !read_sda() {
        drive_scl(false);
        settle();
        drive_scl(true);
        settle();
        pulses += 1;
    }
    drive_sda(false);
    settle();
    drive_sda(true);
    settle();
    pulses
}

/// Whether an `AERR` seen while polling is an address or a data NACK.
#[cfg(hal_i2c = "gd")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Nack {
    Address,
    Data,
}

/// An I2C master on one instance.
///
/// Not `Clone`: it carries the budget left in the acquisition in flight, and a copy would be a
/// second handle on the peripheral with its own count.
#[cfg(hal_i2c = "gd")]
#[derive(Debug)]
pub struct I2c {
    base: u32,
    /// The bring-up timing, which the `SRESET` recovery reprograms.
    timing: I2cTiming,
    /// [`poll_budget`] and [`idle_budget`] at the bring-up core clock.
    poll_budget: u32,
    idle_budget: u32,
    /// Polls left in the current acquisition, shared by every wait in it. A fresh transfer resets
    /// it; a repeated START keeps counting it down.
    txn_left: Cell<u32>,
    /// The SCL and SDA pins `new` took, so [`I2c::bus_clear`] can take them back from the block.
    pins: BusPins,
    /// The bus-clear settle between edges, core cycles (a 100 kHz half period at the bring-up
    /// clock).
    settle_cycles: u32,
}

/// The port base and packed pin of SCL and SDA.
#[derive(Debug, Clone, Copy)]
struct BusPins {
    scl_base: u32,
    scl: u8,
    sda_base: u32,
    sda: u8,
}

impl BusPins {
    /// Both pins as AF open-drain with the pull-up: the block owns the bus from here.
    fn claim_af(&self) {
        gpio::configure_af(self.scl_base, self.scl, PinRole::I2cAfOpenDrain);
        gpio::configure_af(self.sda_base, self.sda, PinRole::I2cAfOpenDrain);
    }
}

#[cfg(hal_i2c = "gd")]
impl I2c {
    /// Bring up `instance` as a master on `pins` (SCL, SDA) at `mode`.
    ///
    /// Refused, before any pin is driven: a pair that is not `instance`'s SCL and SDA in the pin
    /// model ([`pincap::i2c_instance`]), a gate pin, an instance the chip does not carry
    /// (`MissingBase`), or a speed the vendor timing does not cover (`Unsupported`). Then: the
    /// instance's clock, the bus clear on the pins as open-drain outputs, the pins as AF open-drain,
    /// and the block's timing, mode, own address, enable and ACK.
    pub fn new(
        chip: &Chip,
        clock: &ClockConfig,
        instance: PeriphLabel,
        pins: (Pin<Input<Floating>>, Pin<Input<Floating>>),
        mode: I2cMode,
    ) -> Result<I2c, DescriptorError> {
        let (scl, sda) = pins;
        pincap::check_pin_ownership(chip, scl.pin(), PinClaim::NonGate)?;
        pincap::check_pin_ownership(chip, sda.pin(), PinClaim::NonGate)?;
        if pincap::i2c_instance(chip, scl.pin(), sda.pin()) != Some(instance) {
            return Err(DescriptorError::UnsupportedRole);
        }
        let base = chip.base(instance)?;
        let timing = timing_for(clock.apb1_hz(), mode).ok_or(DescriptorError::Unsupported)?;
        crate::clock::enable_i2c(chip.rcu_base()?, chip.clock(), instance)?;

        let pins = BusPins {
            scl_base: scl.port_base(),
            scl: scl.pin(),
            sda_base: sda.port_base(),
            sda: sda.pin(),
        };
        let settle_cycles = (clock.sysclk_hz / 200_000).max(1);
        // A bus still held after this is not an error here: the owner's first transfer finds it,
        // and `bus_clear` is the owner's second attempt.
        let _ = clear_bus(&pins, settle_cycles);
        pins.claim_af();

        let dev = I2c {
            base,
            timing,
            poll_budget: poll_budget(clock.sysclk_hz),
            idle_budget: idle_budget(clock.sysclk_hz),
            txn_left: Cell::new(0),
            pins,
            settle_cycles,
        };
        dev.configure();
        Ok(dev)
    }

    /// Program the block from reset (or after `SRESET`) as the vendor bring-up sequence leaves it:
    /// `CTL1.I2CCLK`, `RT`, `CKCFG` (`i2c_clock_config`), `SMBEN` clear and `SADDR0`
    /// (`i2c_mode_addr_config`), `I2CEN` (`i2c_enable`), `ACKEN` (`i2c_ack_config`).
    fn configure(&self) {
        let t = self.timing;
        self.reg(CTL1).modify(CTL1_I2CCLK, t.i2cclk);
        self.reg(RT).write(t.rt);
        self.reg(CKCFG).write(t.ckcfg);
        self.reg(CTL0).clear_bits(CTL0_SMBEN);
        self.reg(SADDR0).write(OWN_ADDR & SADDR0_MASK);
        self.reg(CTL0).set_bits(CTL0_I2CEN);
        self.set_ack(true);
    }

    /// Clear the bus again on the pins this `I2c` owns, for an owner whose first transfer failed
    /// (the firmware's IMU probe retry): the pins back to open-drain outputs with their latches
    /// high, the clock-pulse sequence, the pins back to AF open-drain, then `SRESET` and the
    /// bring-up programming. The block is reprogrammed either way;
    /// `Err(`[`I2cError::BusHeld`]`)` when SDA still reads low after the sequence.
    pub fn bus_clear(&mut self) -> Result<(), I2cError> {
        let released = clear_bus(&self.pins, self.settle_cycles);
        self.pins.claim_af();
        self.reg(CTL0).write(CTL0_SRESET);
        self.reg(CTL0).write(0);
        self.configure();
        if released {
            Ok(())
        } else {
            Err(I2cError::BusHeld)
        }
    }

    #[inline]
    fn reg(&self, offset: u32) -> Reg32 {
        Reg32::new(self.base, offset)
    }

    #[inline]
    fn set_ack(&self, on: bool) {
        self.reg(CTL0)
            .modify(CTL0_ACKEN, if on { CTL0_ACKEN } else { 0 });
    }

    #[inline]
    fn set_poap(&self, on: bool) {
        self.reg(CTL0)
            .modify(CTL0_POAP, if on { CTL0_POAP } else { 0 });
    }

    #[inline]
    fn start(&self) {
        self.reg(CTL0).set_bits(CTL0_START);
    }

    #[inline]
    fn stop(&self) {
        self.reg(CTL0).set_bits(CTL0_STOP);
    }

    /// The address byte: the 7-bit address and the R/W bit.
    #[inline]
    fn send_address(&self, addr7: u8, read: bool) {
        self.reg(DATA)
            .write((u32::from(addr7) << 1) | u32::from(read));
    }

    #[inline]
    fn receive(&self) -> u8 {
        (self.reg(DATA).read() & 0xFF) as u8
    }

    /// Clear `ADDSEND`: read `STAT0`, then `STAT1`.
    #[inline]
    fn clear_addsend(&self) {
        let _ = self.reg(STAT0).read();
        let _ = self.reg(STAT1).read();
    }

    /// Charge one poll to the acquisition; `false` once it is spent.
    #[inline]
    fn spend(&self) -> bool {
        let left = self.txn_left.get();
        if left == 0 {
            return false;
        }
        self.txn_left.set(left - 1);
        true
    }

    /// Poll `STAT0` until `flag` is set. A bus error, lost arbitration, NACK or overrun seen first
    /// is that error; a spent budget is `Timeout`.
    fn wait_flag(&self, flag: u32, nack: Nack) -> Result<(), I2cError> {
        loop {
            let s = self.reg(STAT0).read();
            if s & flag != 0 {
                return Ok(());
            }
            if s & STAT0_BERR != 0 {
                return Err(I2cError::Bus);
            }
            if s & STAT0_LOSTARB != 0 {
                return Err(I2cError::ArbitrationLoss);
            }
            if s & STAT0_AERR != 0 {
                return Err(match nack {
                    Nack::Address => I2cError::NoAcknowledgeAddress,
                    Nack::Data => I2cError::NoAcknowledgeData,
                });
            }
            if s & STAT0_OUERR != 0 {
                return Err(I2cError::Overrun);
            }
            if !self.spend() {
                return Err(I2cError::Timeout);
            }
        }
    }

    /// Before a fresh START: start the acquisition's budget and wait for `I2CBSY` to clear, for at
    /// most [`idle_budget`] polls. A bus still busy then is recovered once from the acquisition's
    /// budget; still busy after that is `Bus`.
    fn acquire(&self) -> Result<(), I2cError> {
        self.txn_left.set(self.poll_budget);
        let mut cap = self.idle_budget;
        while self.reg(STAT1).read() & STAT1_I2CBSY != 0 {
            cap -= 1;
            if cap == 0 || !self.spend() {
                self.recover(Budget::Acquisition);
                if self.reg(STAT1).read() & STAT1_I2CBSY == 0 {
                    return Ok(());
                }
                return Err(I2cError::Bus);
            }
        }
        Ok(())
    }

    /// Put the block back in a usable state after a failure: clear the sticky error flags; if still
    /// master, STOP and wait for the request to clear (drawing on `budget`); if the block is wedged
    /// (busy without being master, or a START or STOP request still pending), pulse `SRESET` and
    /// reprogram it; leave `POAP` clear and `ACKEN` set for the next transfer.
    fn recover(&self, budget: Budget) {
        let s = self.reg(STAT0).read();
        self.reg(STAT0).write(s & !STAT0_ERRORS);

        if self.reg(STAT1).read() & STAT1_MASTER != 0 {
            self.stop();
            let mut own = self.idle_budget;
            while self.reg(CTL0).read() & CTL0_STOP != 0 {
                let more = match budget {
                    Budget::Acquisition => self.spend(),
                    Budget::Own => {
                        own -= 1;
                        own != 0
                    }
                };
                if !more {
                    break;
                }
            }
        }

        let stat1 = self.reg(STAT1).read();
        let busy_not_master = stat1 & (STAT1_I2CBSY | STAT1_MASTER) == STAT1_I2CBSY;
        let request_stuck = self.reg(CTL0).read() & (CTL0_START | CTL0_STOP) != 0;
        if busy_not_master || request_stuck {
            self.reg(CTL0).write(CTL0_SRESET);
            self.reg(CTL0).write(0);
            self.configure();
        }
        self.set_poap(false);
        self.set_ack(true);
    }

    /// Polled master write of `bytes` to `addr7`. `fresh`: the bus is not already held by this side
    /// (wait for it idle first; false after a write without STOP). `stop`: end with a STOP (false
    /// to leave the bus for a repeated START). An empty `bytes` is the address handshake alone, a
    /// presence check. A failure recovers the block before returning.
    pub fn write_bytes(
        &self,
        addr7: u8,
        bytes: &[u8],
        fresh: bool,
        stop: bool,
    ) -> Result<(), I2cError> {
        if fresh {
            self.acquire()?;
        }
        let r = self.write_inner(addr7, bytes, stop);
        if r.is_err() {
            self.recover(Budget::Own);
        }
        r
    }

    fn write_inner(&self, addr7: u8, bytes: &[u8], stop: bool) -> Result<(), I2cError> {
        self.set_ack(true);
        self.start();
        self.wait_flag(STAT0_SBSEND, Nack::Address)?;
        self.send_address(addr7, false);
        self.wait_flag(STAT0_ADDSEND, Nack::Address)?;
        self.clear_addsend();
        if let [head @ .., last] = bytes {
            for &b in head {
                self.wait_flag(STAT0_TBE, Nack::Data)?;
                self.reg(DATA).write(u32::from(b));
            }
            self.wait_flag(STAT0_TBE, Nack::Data)?;
            self.reg(DATA).write(u32::from(*last));
            self.wait_flag(STAT0_BTC, Nack::Data)?;
        }
        if stop {
            self.stop();
        }
        Ok(())
    }

    /// Polled master read into `buf` from `addr7`, ending with a STOP (Solution B). `fresh` as for
    /// [`I2c::write_bytes`]. An empty `buf` touches nothing. A failure recovers the block before
    /// returning.
    pub fn read_bytes(&self, addr7: u8, buf: &mut [u8], fresh: bool) -> Result<(), I2cError> {
        if buf.is_empty() {
            return Ok(());
        }
        if fresh {
            self.acquire()?;
        }
        let r = self.read_inner(addr7, buf);
        if r.is_err() {
            self.recover(Budget::Own);
        }
        r
    }

    /// The receive sequence per length. The arms are slice patterns, so nothing is indexed and the
    /// 250 Hz IMU read carries no bounds-check panic path.
    fn read_inner(&self, addr7: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        self.set_ack(true);
        if let [_, _] = buf {
            self.set_poap(true);
        }
        self.start();
        self.wait_flag(STAT0_SBSEND, Nack::Address)?;
        self.send_address(addr7, true);
        match buf {
            [] => {}
            [only] => {
                // N = 1: ACK off before ADDSEND clears, STOP after it, read on RBNE.
                self.set_ack(false);
                self.wait_flag(STAT0_ADDSEND, Nack::Address)?;
                self.clear_addsend();
                self.stop();
                self.wait_flag(STAT0_RBNE, Nack::Data)?;
                *only = self.receive();
            }
            [first, second] => {
                // N = 2, POAP set: ACK off before ADDSEND clears (it NACKs the second byte); BTC
                // with both bytes held and SCL stretched; STOP; read both.
                self.set_ack(false);
                self.wait_flag(STAT0_ADDSEND, Nack::Address)?;
                self.clear_addsend();
                self.wait_flag(STAT0_BTC, Nack::Data)?;
                self.stop();
                *first = self.receive();
                self.wait_flag(STAT0_RBNE, Nack::Data)?;
                *second = self.receive();
                self.set_poap(false);
            }
            [head @ .., third_last, second_last, last] => {
                // N > 2: read on RBNE up to the last three; BTC (third-last in DATA, second-last in
                // the shift register, SCL stretched); ACK off; read the third-last; BTC again; STOP;
                // read the last two.
                self.wait_flag(STAT0_ADDSEND, Nack::Address)?;
                self.clear_addsend();
                for b in head.iter_mut() {
                    self.wait_flag(STAT0_RBNE, Nack::Data)?;
                    *b = self.receive();
                }
                self.wait_flag(STAT0_BTC, Nack::Data)?;
                self.set_ack(false);
                *third_last = self.receive();
                self.wait_flag(STAT0_BTC, Nack::Data)?;
                self.stop();
                *second_last = self.receive();
                self.wait_flag(STAT0_RBNE, Nack::Data)?;
                *last = self.receive();
            }
        }
        self.set_ack(true);
        Ok(())
    }
}

/// Which budget a recovery's STOP wait draws on.
#[cfg(hal_i2c = "gd")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Budget {
    /// The acquisition's, for the recovery inside the bus-idle wait (most of it is left there).
    Acquisition,
    /// Its own [`idle_budget`], for the recovery after a failed transfer (the acquisition's may be
    /// spent).
    Own,
}

/// The bus clear on SCL and SDA as open-drain outputs, with a settle of `settle_cycles` (a 100 kHz
/// half period) between edges. A clean bus costs one SDA read. Returns whether SDA reads high
/// (released) at the end.
///
/// Both output latches go high BEFORE the pins become outputs: `configure_output_od` does not
/// preset the latch, and after a warm reset it reads 0, so a pin made an output with its latch
/// low drives the line low at once and the slave sees an SCL edge and a STOP-like SDA edge before
/// the clean-bus check (runtime-hal 71c39e3, "bus clear without the entry glitch"). With the
/// latches high the pins come up released and the first edge on the bus is the sequence's own.
fn clear_bus(pins: &BusPins, settle_cycles: u32) -> bool {
    let BusPins {
        scl_base,
        scl,
        sda_base,
        sda,
    } = *pins;
    let at = move |line: Line| match line {
        Line::Scl => (scl_base, scl),
        Line::Sda => (sda_base, sda),
    };
    clear_bus_with(
        |line, high| {
            let (base, pin) = at(line);
            gpio::set_pin(base, pin, high)
        },
        |line| {
            let (base, pin) = at(line);
            gpio::configure_output_od(base, pin, true)
        },
        || gpio::read_pin(sda_base, sda),
        || settle(settle_cycles),
    )
}

/// One of the two bus lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Line {
    Scl,
    Sda,
}

/// [`clear_bus`] over closures, so the host tests see the order of the pin operations: `latch`
/// sets a line's output latch (`true` = released), `make_output` makes it an open-drain output.
fn clear_bus_with(
    mut latch: impl FnMut(Line, bool),
    mut make_output: impl FnMut(Line),
    mut read_sda: impl FnMut() -> bool,
    mut settle: impl FnMut(),
) -> bool {
    latch(Line::Scl, true);
    latch(Line::Sda, true);
    make_output(Line::Scl);
    make_output(Line::Sda);
    let latch = core::cell::RefCell::new(latch);
    bus_clear(
        &mut read_sda,
        |high| (latch.borrow_mut())(Line::Scl, high),
        |high| (latch.borrow_mut())(Line::Sda, high),
        &mut settle,
    );
    read_sda()
}

/// Busy-wait between two bus-clear edges (the host has no bus to time).
#[inline(always)]
fn settle(cycles: u32) {
    #[cfg(target_arch = "arm")]
    cortex_m::asm::delay(cycles);
    #[cfg(not(target_arch = "arm"))]
    let _ = cycles;
}

#[cfg(hal_i2c = "gd")]
impl ErrorType for I2c {
    type Error = I2cError;
}

#[cfg(hal_i2c = "gd")]
impl embedded_hal::i2c::I2c<SevenBitAddress> for I2c {
    /// The operations as one transaction on `address`. A write is followed by a repeated START
    /// unless it is the last operation, which ends with a STOP; a read always ends with a STOP
    /// (Solution B places it), so an operation after a read starts fresh. An empty read is
    /// skipped; a bus still held at the end is released with a STOP.
    ///
    /// The trait's `write` and `write_read` are this with `[Write]` and `[Write, Read]`: a STOPped
    /// write, and the register-pointer write then repeated-START read the IMU driver makes.
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let count = operations.len();
        let mut held = false;
        for (i, op) in operations.iter_mut().enumerate() {
            let last = i + 1 == count;
            match op {
                Operation::Write(w) => {
                    self.write_bytes(address, w, !held, last)?;
                    held = !last;
                }
                Operation::Read(r) => {
                    if !r.is_empty() {
                        self.read_bytes(address, r, !held)?;
                        held = false;
                    }
                }
            }
        }
        if held {
            self.stop();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(hal_i2c = "gd")]
    const APB1_36M: u32 = 36_000_000;

    #[test]
    #[cfg(hal_i2c = "gd")]
    fn standard_100k_at_36mhz_is_the_vendor_value() {
        // The bringup_i2c0_100k golden: I2CCLK 0x24, RT 0x25, CKCFG 0xB4.
        let t = timing_for(APB1_36M, I2cMode::standard(100_000)).unwrap();
        assert_eq!((t.i2cclk, t.rt, t.ckcfg), (0x24, 0x25, 0xB4));
        assert_eq!(t.ckcfg & (CKCFG_FAST | CKCFG_DTCY), 0);
    }

    #[test]
    #[cfg(hal_i2c = "gd")]
    fn standard_at_the_8mhz_reset_clock() {
        let t = timing_for(8_000_000, I2cMode::standard(100_000)).unwrap();
        assert_eq!((t.i2cclk, t.rt, t.ckcfg), (8, 9, 40));
    }

    #[test]
    #[cfg(hal_i2c = "gd")]
    fn standard_clamps_clkc_and_rt() {
        // CLKC at least 4: 0.5 MHz / (2 * 100 kHz) = 2.
        let t = timing_for(500_000, I2cMode::standard(100_000)).unwrap();
        assert_eq!(t.ckcfg, 4);
        // RT at least 2 (0 MHz + 1 = 1).
        assert_eq!(t.rt, 2);
        // I2CCLK and RT at most 0x48.
        let t = timing_for(108_000_000, I2cMode::standard(100_000)).unwrap();
        assert_eq!((t.i2cclk, t.rt), (0x48, 0x48));
    }

    #[test]
    #[cfg(hal_i2c = "gd")]
    fn fast_400k_duty_two() {
        // 36 MHz / (3 * 400 kHz) = 30; RT = 36 * 300 / 1000 + 1 = 11.
        let t = timing_for(APB1_36M, I2cMode::fast(400_000, FastDuty::Two)).unwrap();
        assert_eq!(t.rt, 11);
        assert_eq!(t.ckcfg, CKCFG_FAST | 30);
    }

    #[test]
    #[cfg(hal_i2c = "gd")]
    fn fast_400k_duty_sixteen_ninths() {
        // 36 MHz / (25 * 400 kHz) = 3, with DTCY.
        let t = timing_for(APB1_36M, I2cMode::fast(400_000, FastDuty::SixteenNine)).unwrap();
        assert_eq!(t.ckcfg, CKCFG_FAST | CKCFG_DTCY | 3);
    }

    #[test]
    #[cfg(hal_i2c = "gd")]
    fn fast_forces_clkc_to_at_least_one() {
        // 8 MHz / (25 * 400 kHz) = 0, forced to 1.
        let t = timing_for(8_000_000, I2cMode::fast(400_000, FastDuty::SixteenNine)).unwrap();
        assert_eq!(t.ckcfg & CKCFG_CLKC, 1);
    }

    #[test]
    #[cfg(hal_i2c = "gd")]
    fn the_speed_picks_the_mode() {
        // fast() at 100 kHz is standard mode, as the vendor call decides by speed.
        let t = timing_for(APB1_36M, I2cMode::fast(100_000, FastDuty::SixteenNine)).unwrap();
        assert_eq!(t.ckcfg & CKCFG_FAST, 0);
        let t = timing_for(APB1_36M, I2cMode::standard(100_001)).unwrap();
        assert_eq!(t.ckcfg & CKCFG_FAST, CKCFG_FAST);
    }

    #[test]
    #[cfg(hal_i2c = "gd")]
    fn speeds_the_vendor_leaves_unprogrammed_are_refused() {
        assert_eq!(timing_for(APB1_36M, I2cMode::standard(0)), None);
        assert_eq!(
            timing_for(APB1_36M, I2cMode::fast(400_001, FastDuty::Two)),
            None
        );
    }

    #[test]
    #[cfg(hal_i2c = "gd")]
    fn the_fleet_tree_feeds_36mhz_to_the_block() {
        assert_eq!(ClockConfig::REFERENCE_72M_IRC8M.apb1_hz(), APB1_36M);
    }

    #[test]
    fn the_block_bound_at_72mhz() {
        assert_eq!(poll_budget(72_000_000), 13_500);
        assert_eq!(idle_budget(72_000_000), 1_350);
        assert_eq!(worst_case_block_us(72_000_000), 6_600);
        // The firmware's assertion: twice the bound inside the 16 ms coast
        // (256 periods * 2 * ARR 2250 / 72 counts per us).
        assert!(worst_case_block_us(72_000_000) * 2 < 256 * 2 * 2250 / 72);
    }

    #[test]
    fn the_block_bound_is_the_budget_in_time_at_any_clock() {
        for mhz in [8u32, 24, 48, 72, 108] {
            let us = worst_case_block_us(mhz * 1_000_000);
            // The budget plus a tenth, give or take the integer division.
            let want = TXN_BUDGET_US + TXN_BUDGET_US / 10;
            assert!(us <= want && us + 2 >= want, "{mhz} MHz: {us} us");
        }
    }

    #[test]
    fn budgets_never_reach_zero() {
        assert_eq!(poll_budget(0), 1);
        assert_eq!(idle_budget(1_000_000), 18);
        assert_eq!(idle_budget(0), 1);
        assert_eq!(worst_case_block_us(999_999), u32::MAX);
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Op {
        Latch(Line, bool),
        Output(Line),
    }

    /// The pin operations `clear_bus_with` makes on a bus whose SDA reads `sda`, and its result.
    fn clear_ops(sda: bool) -> (Vec<Op>, bool) {
        let ops = core::cell::RefCell::new(Vec::new());
        let released = clear_bus_with(
            |l, h| ops.borrow_mut().push(Op::Latch(l, h)),
            |l| ops.borrow_mut().push(Op::Output(l)),
            || sda,
            || {},
        );
        (ops.into_inner(), released)
    }

    #[test]
    fn the_latches_go_high_before_the_pins_become_outputs() {
        // runtime-hal 71c39e3: a pin made an output with its latch at the warm-reset 0 drives the
        // line low at once, an SCL edge and a STOP-like SDA edge before the clean-bus check.
        let (ops, released) = clear_ops(true);
        assert!(released);
        assert_eq!(
            ops[..4],
            [
                Op::Latch(Line::Scl, true),
                Op::Latch(Line::Sda, true),
                Op::Output(Line::Scl),
                Op::Output(Line::Sda),
            ]
        );
        // A clean bus: after that only the sequence's own release of both lines, no low edge.
        assert!(ops[4..].iter().all(|op| matches!(op, Op::Latch(_, true))));
    }

    #[test]
    fn a_held_bus_reports_not_released_after_the_pulses() {
        let (ops, released) = clear_ops(false);
        assert!(!released);
        let scl_lows = ops
            .iter()
            .filter(|op| **op == Op::Latch(Line::Scl, false))
            .count();
        assert_eq!(scl_lows as u32, BUS_CLEAR_MAX_PULSES);
        // Never an output before both latches are high.
        let first_output = ops.iter().position(|op| matches!(op, Op::Output(_)));
        assert_eq!(first_output, Some(2));
    }

    /// A simulated bus: SDA held low by a slave until `release_after` SCL pulses (never if `None`).
    struct Bus {
        release_after: Option<u32>,
        scl_falls: u32,
        sda: Vec<bool>,
    }

    fn run(release_after: Option<u32>) -> (u32, Bus) {
        let bus = core::cell::RefCell::new(Bus {
            release_after,
            scl_falls: 0,
            sda: Vec::new(),
        });
        let n = bus_clear(
            || {
                let b = bus.borrow();
                b.release_after.is_some_and(|r| b.scl_falls >= r)
            },
            |high| {
                if !high {
                    bus.borrow_mut().scl_falls += 1;
                }
            },
            |high| bus.borrow_mut().sda.push(high),
            || {},
        );
        (n, bus.into_inner())
    }

    #[test]
    fn a_clean_bus_gets_no_pulses_and_no_stop() {
        let (n, bus) = run(Some(0));
        assert_eq!(n, 0);
        assert_eq!(bus.scl_falls, 0);
        assert_eq!(bus.sda, [true], "SDA released once, never pulled low");
    }

    #[test]
    fn a_stuck_bus_gets_nine_pulses_and_a_stop() {
        let (n, bus) = run(None);
        assert_eq!(n, BUS_CLEAR_MAX_PULSES);
        assert_eq!(bus.scl_falls, BUS_CLEAR_MAX_PULSES);
        assert_eq!(bus.sda, [true, false, true], "release, then the STOP edge");
    }

    #[test]
    fn pulsing_stops_when_the_slave_lets_go() {
        for release in 1..BUS_CLEAR_MAX_PULSES {
            let (n, bus) = run(Some(release));
            assert_eq!(n, release);
            assert_eq!(bus.scl_falls, release);
            assert_eq!(bus.sda, [true, false, true]);
        }
    }
}
