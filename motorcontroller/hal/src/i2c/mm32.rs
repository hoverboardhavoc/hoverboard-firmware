// SPDX-License-Identifier: GPL-3.0-or-later
//! The MM32SPIN05 I2C master (MindMotion's `reg_i2c.h` and `hal_i2c.c`, LibSamples V1.13): a
//! DesignWare-style block. The `mm32` model of [`crate::i2c`]; the bus clear, the time budget and
//! the bus modes are the parent's.
//!
//! | Register | Offset | Width | Used here |
//! |---|---|---|---|
//! | `CR` (`IC_CON`) | `0x00` | 16 | `MASTER` 0, `SPEED` 2:1 (1 standard, 2 fast), `REPEN` 5, `SLAVEDIS` 6, `EMPINT` 8 |
//! | `TAR` | `0x04` | 16 | the 7-bit target address, written while the block is disabled |
//! | `DR` (`IC_DATA_CMD`) | `0x10` | 16 | a byte to send, or `CMD` (bit 8) to read one; the received byte |
//! | `SSHR`/`SSLR` | `0x14`/`0x18` | 16 | the standard-mode SCL high and low counts |
//! | `FSHR`/`FSLR` | `0x1C`/`0x20` | 16 | the fast-mode SCL high and low counts |
//! | `IMR` | `0x30` | 16 | the interrupt mask, cleared to `INTR_MASK` (0xC000) |
//! | `RAWISR` | `0x34` | 16 | `TX_ABRT` 6, `STOP_DET` 9 |
//! | `RXTLR`/`TXTLR` | `0x38`/`0x3C` | 16 | the FIFO thresholds, 0 |
//! | `TX_ABRT` | `0x54` | 16 | read to clear the abort |
//! | `STOP` (`IC_CLR_STOP_DET`) | `0x60` | 16 | read to clear `STOP_DET` |
//! | `ENR` | `0x6C` | 16 | `ENABLE` 0, `ABORT` 1 |
//! | `SR` | `0x70` | 32 | `TFNF` 1, `TFE` 2, `RFNE` 3, `MST_ACTIV` 5 |
//!
//! The widths are the vendor struct's: the registers to `ENR` are half-words, a 32-bit access
//! would not be the vendor's traffic (the golden traces compare widths).
//!
//! # Bring-up
//!
//! [`I2c::new`] clears a held bus, routes the pins as AF open-drain (AF1, `pincap`) and programs the
//! block as `I2C_Init` (master, the speed, `REPEN`, `SLAVEDIS`, `EMPINT`, the SCL counts by the
//! vendor's formula: [`timing_for`]) and `I2C_Cmd` leave it.
//!
//! # Transfers
//!
//! The block runs the bus from its command FIFO: a byte written to `DR` is sent, `CMD` reads one,
//! a change of direction sends a repeated START (`REPEN`), and a FIFO that runs empty holds the bus
//! (SCL low) rather than ending the transfer. A transfer therefore ends by `ENR.ABORT`, which
//! finishes the byte on the bus and sends the STOP (the vendor's `I2C_GenerateSTOP`). The target
//! address is set at the start of each acquisition with the block disabled, as MindMotion's
//! `I2C_SetDeviceAddr` does; disabling it also empties both FIFOs. A NACK of the address or of a
//! data byte sets `RAWISR.TX_ABRT`, after which the block has sent a STOP and flushed its FIFO.
//! MindMotion's header names no abort-source register, so the HAL cannot tell an address NACK
//! from a data NACK and reports both as [`I2cError::NoAcknowledgeAddress`]. The STOP's own abort
//! raises the same flag, so a NACK of the last byte of a write that ends in a STOP is not
//! reported: the block shows nothing for it before the STOP (on the bench MM32SPIN05,
//! `RAWISR.MST_ON_HOLD` never set while the master held the bus).
//!
//! Reads keep at most [`RX_IN_FLIGHT`] read commands ahead of the bytes taken: the FIFOs are two
//! entries deep (AILIFE's MM32 firmware, measured), so a deeper queue could overrun the receive
//! FIFO while the caller is preempted.

use core::cell::Cell;

use embedded_hal::i2c::{ErrorType, Operation, SevenBitAddress};

use super::{clear_bus, idle_budget, poll_budget, BusPins, I2cMode};
use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::error::{DescriptorError, I2cError};
use crate::gpio::PinAf;
use crate::reg::{Reg16, Reg32};

const CR: u32 = 0x00;
const TAR: u32 = 0x04;
const DR: u32 = 0x10;
const SSHR: u32 = 0x14;
const SSLR: u32 = 0x18;
const FSHR: u32 = 0x1C;
const FSLR: u32 = 0x20;
const IMR: u32 = 0x30;
const RAWISR: u32 = 0x34;
const RXTLR: u32 = 0x38;
const TXTLR: u32 = 0x3C;
const CLR_TX_ABRT: u32 = 0x54;
const CLR_STOP_DET: u32 = 0x60;
const ENR: u32 = 0x6C;
const SR: u32 = 0x70;

const CR_MASTER: u16 = 1 << 0;
const CR_STD: u16 = 1 << 1;
const CR_FAST: u16 = 2 << 1;
const CR_SLAVE10: u16 = 1 << 3;
const CR_MASTER10: u16 = 1 << 4;
const CR_REPEN: u16 = 1 << 5;
const CR_SLAVEDIS: u16 = 1 << 6;
const CR_EMPINT: u16 = 1 << 8;
/// The bits `I2C_Init` clears before it writes `CR`.
const CR_INIT_CLEAR: u16 =
    CR_EMPINT | CR_SLAVEDIS | CR_REPEN | CR_MASTER10 | CR_SLAVE10 | CR_FAST | CR_MASTER;

/// `DR.CMD`: read one byte.
const DR_CMD_READ: u16 = 1 << 8;
/// `INTR_MASK` (`hal_i2c.h`), what `I2C_Init` masks `IMR` with: every interrupt off.
const IMR_KEEP: u16 = 0xC000;
const RAWISR_TX_ABRT: u16 = 1 << 6;
/// `RAWISR.STOP_DET`: a STOP was seen on the bus (sticky until `CLR_STOP_DET` is read).
const RAWISR_STOP_DET: u16 = 1 << 9;
const ENR_ENABLE: u16 = 1 << 0;
const ENR_ABORT: u16 = 1 << 1;
const SR_TFNF: u32 = 1 << 1;
const SR_TFE: u32 = 1 << 2;
const SR_RFNE: u32 = 1 << 3;
const SR_MST_ACTIV: u32 = 1 << 5;

/// Read commands queued ahead of the bytes taken.
pub const RX_IN_FLIGHT: usize = 2;

/// The SCL counts and the speed bit, as `I2C_Init` computes them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Mm32Timing {
    fast: bool,
    hcnt: u16,
    lcnt: u16,
}

/// The SCL counts for APB1 clock `pclk1_hz` and `mode`, `I2C_Init`'s integer arithmetic: the APB1
/// period `p = 1e9 / pclk1` ns, the spike allowance `s = 82 / p`, `m = pclk1 / speed`; standard
/// (at most 100 kHz) `LCNT = (m - 13 - s) / 2`, `HCNT = m - 13 - s - LCNT`; fast
/// `LCNT = (m - 13 - s) / 2 + 4`, `HCNT = m - 13 - LCNT - s`. `None` for a speed of 0, above
/// 400 kHz, or too fast for the clock (a count that would underflow or overflow 16 bits).
const fn timing_for(pclk1_hz: u32, mode: I2cMode) -> Option<Mm32Timing> {
    let speed = mode.speed_hz;
    if speed == 0 || speed > 400_000 || pclk1_hz < 1_000_000 {
        return None;
    }
    let period_ns = 1_000_000_000 / pclk1_hz;
    let spike = 82 / period_ns;
    let m = pclk1_hz / speed;
    let Some(span) = m.checked_sub(13 + spike) else {
        return None;
    };
    let fast = speed > 100_000;
    let lcnt = if fast { span / 2 + 4 } else { span / 2 };
    let Some(hcnt) = span.checked_sub(lcnt) else {
        return None;
    };
    if hcnt == 0 || hcnt > 0xFFFF || lcnt > 0xFFFF {
        return None;
    }
    Some(Mm32Timing {
        fast,
        hcnt: hcnt as u16,
        lcnt: lcnt as u16,
    })
}

/// An I2C master on the MM32's I2C1 (the HAL's `I2c0`).
///
/// Not `Clone`: it carries the budget left in the acquisition in flight.
#[derive(Debug)]
pub struct I2c {
    base: u32,
    timing: Mm32Timing,
    poll_budget: u32,
    idle_budget: u32,
    txn_left: Cell<u32>,
    pins: BusPins,
    settle_cycles: u32,
}

impl I2c {
    /// Bring up `instance` as a master on the stored `scl` and `sda` at `mode`
    /// (`specs/config-read-once.md` section 5; nothing here checks the pins or the instance).
    ///
    /// Refused, before any pin is driven: an instance the chip does not carry (`MissingBase`), or a
    /// speed the vendor timing does not cover (`Unsupported`). Then: the instance's clock, the bus
    /// clear on the pins as open-drain outputs, the pins as AF open-drain on their stored
    /// alternate functions, and the block as `I2C_Init` and `I2C_Cmd` leave it.
    pub fn new(
        chip: &Chip,
        clock: &ClockConfig,
        instance: PeriphLabel,
        scl: PinAf,
        sda: PinAf,
        mode: I2cMode,
    ) -> Result<I2c, DescriptorError> {
        let base = chip.base(instance)?;
        let timing = timing_for(clock.apb1_hz(), mode).ok_or(DescriptorError::Unsupported)?;
        crate::clock::enable_i2c(chip.rcu_base()?, chip.clock(), instance)?;

        let pins = BusPins {
            scl_base: chip.clocked_port_base(scl.port),
            scl: scl.pin,
            scl_af: scl.af,
            sda_base: chip.clocked_port_base(sda.port),
            sda: sda.pin,
            sda_af: sda.af,
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

    #[inline]
    fn reg(&self, offset: u32) -> Reg16 {
        Reg16::new(self.base, offset)
    }

    #[inline]
    fn sr(&self) -> u32 {
        Reg32::new(self.base, SR).read()
    }

    /// Program the block as `I2C_Init` then `I2C_Cmd` leave it: disabled; the SCL counts of the
    /// speed; `CR` cleared of the fields it sets, then written (master, the speed, `REPEN`,
    /// `SLAVEDIS`, `EMPINT`); `IMR` masked to `INTR_MASK`; both FIFO thresholds 0; enabled.
    fn configure(&self) {
        let t = self.timing;
        let enr = self.reg(ENR);
        enr.write(enr.read() & !ENR_ENABLE);
        let (hr, lr) = if t.fast { (FSHR, FSLR) } else { (SSHR, SSLR) };
        self.reg(lr).write(t.lcnt);
        self.reg(hr).write(t.hcnt);
        let cr = self.reg(CR);
        cr.write(cr.read() & !CR_INIT_CLEAR);
        let speed = if t.fast { CR_FAST } else { CR_STD };
        cr.write(CR_EMPINT | CR_SLAVEDIS | CR_REPEN | speed | CR_MASTER);
        let imr = self.reg(IMR);
        imr.write(imr.read() & IMR_KEEP);
        self.reg(RXTLR).write(0);
        self.reg(TXTLR).write(0);
        enr.write(enr.read() | ENR_ENABLE);
    }

    /// Clear the bus again on the pins this `I2c` owns (the firmware's IMU probe retry): the
    /// parent's clock-pulse sequence, the pins back to AF open-drain, then the bring-up
    /// programming. `Err(`[`I2cError::BusHeld`]`)` when SDA still reads low after the sequence.
    pub fn bus_clear(&mut self) -> Result<(), I2cError> {
        let released = clear_bus(&self.pins, self.settle_cycles);
        self.pins.claim_af();
        self.configure();
        if released {
            Ok(())
        } else {
            Err(I2cError::BusHeld)
        }
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

    /// The abort flag: a NACK (or lost arbitration) ended the transfer.
    #[inline]
    fn aborted(&self) -> bool {
        self.reg(RAWISR).read() & RAWISR_TX_ABRT != 0
    }

    /// Poll `SR` until `flag` is set. An abort seen first is a NACK; a spent budget is `Timeout`.
    fn wait_sr(&self, flag: u32) -> Result<(), I2cError> {
        loop {
            if self.sr() & flag != 0 {
                return Ok(());
            }
            if self.aborted() {
                return Err(I2cError::NoAcknowledgeAddress);
            }
            if !self.spend() {
                return Err(I2cError::Timeout);
            }
        }
    }

    /// Before a fresh transfer: start the acquisition's budget, wait at most [`idle_budget`] polls
    /// for the master to go idle (`SR.MST_ACTIV` clear; recovered once if it does not), then point
    /// the block at `addr7` (`TAR`, written with the block disabled, which also empties the FIFOs
    /// and ends a stale abort), and clear the abort and STOP flags.
    fn acquire(&self, addr7: u8) -> Result<(), I2cError> {
        self.txn_left.set(self.poll_budget);
        let mut cap = self.idle_budget;
        while self.sr() & SR_MST_ACTIV != 0 {
            cap -= 1;
            if cap == 0 || !self.spend() {
                self.recover();
                if self.sr() & SR_MST_ACTIV != 0 {
                    return Err(I2cError::Bus);
                }
                break;
            }
        }
        let enr = self.reg(ENR);
        enr.write(enr.read() & !ENR_ENABLE);
        self.reg(TAR).write(u16::from(addr7 & 0x7F));
        enr.write(enr.read() | ENR_ENABLE);
        let _ = self.reg(CLR_TX_ABRT).read();
        let _ = self.reg(CLR_STOP_DET).read();
        Ok(())
    }

    /// End the transfer: `ENR.ABORT` (the byte on the bus finishes, then the STOP), wait for the
    /// block to clear it, read `TX_ABRT` to clear the abort it raises (`I2C_GenerateSTOP`), then
    /// wait for the STOP on the bus (`RAWISR.STOP_DET`, cleared at the acquisition), as
    /// MindMotion's polled sample does. Both waits are bounded by [`idle_budget`].
    fn stop(&self) {
        let enr = self.reg(ENR);
        enr.write(enr.read() | ENR_ABORT);
        let mut own = self.idle_budget;
        while enr.read() & ENR_ABORT != 0 {
            own -= 1;
            if own == 0 {
                break;
            }
        }
        let _ = self.reg(CLR_TX_ABRT).read();
        let mut own = self.idle_budget;
        while self.reg(RAWISR).read() & RAWISR_STOP_DET == 0 {
            own -= 1;
            if own == 0 {
                break;
            }
        }
    }

    /// Put the block back in a usable state after a failure: the STOP, then the bring-up
    /// programming (disabling it empties both FIFOs).
    fn recover(&self) {
        self.stop();
        self.configure();
    }

    /// Polled master write of `bytes` to `addr7`. `fresh`: start a new acquisition (false after a
    /// write without STOP, whose bus this side still holds). `stop`: end with a STOP once the FIFO
    /// has drained (false to leave the bus for a repeated START). An empty `bytes` is refused with
    /// [`I2cError::Other`]: this block cannot send an address without a data byte. A failure
    /// recovers the block before returning.
    pub fn write_bytes(
        &self,
        addr7: u8,
        bytes: &[u8],
        fresh: bool,
        stop: bool,
    ) -> Result<(), I2cError> {
        if bytes.is_empty() {
            return Err(I2cError::Other);
        }
        if fresh {
            self.acquire(addr7)?;
        }
        let r = self.write_inner(bytes, stop);
        if r.is_err() {
            self.recover();
        }
        r
    }

    /// Each byte into `DR`, then a wait for the FIFO to empty (`SR.TFE`), MindMotion's
    /// `EEPROM_WriteByte`: one byte in the FIFO at a time. An abort seen by then (an address NACK)
    /// is the error; otherwise the STOP.
    fn write_inner(&self, bytes: &[u8], stop: bool) -> Result<(), I2cError> {
        for &b in bytes {
            self.reg(DR).write(u16::from(b));
            self.wait_sr(SR_TFE)?;
        }
        if stop {
            if self.aborted() {
                return Err(I2cError::NoAcknowledgeAddress);
            }
            self.stop();
        }
        Ok(())
    }

    /// Polled master read into `buf` from `addr7`, ending with a STOP. `fresh` as for
    /// [`I2c::write_bytes`]; after a write without STOP the first read command sends the repeated
    /// START. An empty `buf` touches nothing. A failure recovers the block before returning.
    pub fn read_bytes(&self, addr7: u8, buf: &mut [u8], fresh: bool) -> Result<(), I2cError> {
        if buf.is_empty() {
            return Ok(());
        }
        if fresh {
            self.acquire(addr7)?;
        }
        let r = self.read_inner(buf);
        if r.is_err() {
            self.recover();
        }
        r
    }

    fn read_inner(&self, buf: &mut [u8]) -> Result<(), I2cError> {
        let len = buf.len();
        let mut issued = 0usize;
        for (taken, slot) in buf.iter_mut().enumerate() {
            loop {
                // Room for a command, then a byte to take: two `SR` reads, as MindMotion's
                // `EEPROM_ReadBuff` polls them.
                if issued < len && issued < taken + RX_IN_FLIGHT && self.sr() & SR_TFNF != 0 {
                    self.reg(DR).write(DR_CMD_READ);
                    issued += 1;
                }
                if self.sr() & SR_RFNE != 0 {
                    *slot = (self.reg(DR).read() & 0xFF) as u8;
                    break;
                }
                if self.aborted() {
                    return Err(I2cError::NoAcknowledgeAddress);
                }
                if !self.spend() {
                    return Err(I2cError::Timeout);
                }
            }
        }
        self.stop();
        Ok(())
    }
}

impl ErrorType for I2c {
    type Error = I2cError;
}

impl embedded_hal::i2c::I2c<SevenBitAddress> for I2c {
    /// The operations as one transaction on `address`. A write is followed by a repeated START
    /// unless it is the last operation, which ends with a STOP; a read always ends with a STOP, so
    /// an operation after a read starts fresh. An empty read is skipped; a bus still held at the
    /// end is released with a STOP.
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
    use crate::i2c::FastDuty;

    const APB1_36M: u32 = 36_000_000;

    #[test]
    fn fast_400k_at_36mhz_is_the_vendor_arithmetic() {
        // p = 27 ns, s = 3, m = 90: LCNT = 74 / 2 + 4 = 41, HCNT = 90 - 13 - 41 - 3 = 33.
        let t = timing_for(APB1_36M, I2cMode::fast(400_000, FastDuty::Two)).unwrap();
        assert_eq!(
            t,
            Mm32Timing {
                fast: true,
                hcnt: 33,
                lcnt: 41
            }
        );
    }

    #[test]
    fn fast_400k_at_the_reference_72mhz_apb1() {
        // p = 13 ns, s = 6, m = 180: LCNT = 161 / 2 + 4 = 84, HCNT = 180 - 13 - 84 - 6 = 77 (the
        // golden trace i2c/bringup_i2c1_400k_mm32).
        let pclk1 = ClockConfig::REFERENCE_72M_HSI.apb1_hz();
        let t = timing_for(pclk1, I2cMode::fast(400_000, FastDuty::Two)).unwrap();
        assert_eq!(
            t,
            Mm32Timing {
                fast: true,
                hcnt: 77,
                lcnt: 84
            }
        );
    }

    #[test]
    fn standard_100k_at_36mhz() {
        // m = 360: LCNT = 344 / 2 = 172, HCNT = 172.
        let t = timing_for(APB1_36M, I2cMode::standard(100_000)).unwrap();
        assert_eq!(
            t,
            Mm32Timing {
                fast: false,
                hcnt: 172,
                lcnt: 172
            }
        );
    }

    #[test]
    fn speeds_the_counts_cannot_express_are_refused() {
        assert_eq!(timing_for(APB1_36M, I2cMode::standard(0)), None);
        assert_eq!(
            timing_for(APB1_36M, I2cMode::fast(400_001, FastDuty::Two)),
            None
        );
        // 1 MHz APB1 at 400 kHz: m = 2, below the 13-count floor.
        assert_eq!(
            timing_for(1_000_000, I2cMode::fast(400_000, FastDuty::Two)),
            None
        );
    }

    #[test]
    fn the_init_clear_covers_every_field_the_write_sets() {
        let set = CR_EMPINT | CR_SLAVEDIS | CR_REPEN | CR_FAST | CR_STD | CR_MASTER;
        assert_eq!(set & !(CR_INIT_CLEAR | CR_STD), 0);
    }
}
