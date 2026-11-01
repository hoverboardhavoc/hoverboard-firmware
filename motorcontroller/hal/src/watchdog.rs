// SPDX-License-Identifier: GPL-3.0-or-later
//! The free watchdog (`FWDGT`, ST's IWDG), clocked from the IRC40K oscillator.
//!
//! The registers this module drives are the same on the GD32F10x and GD32F1x0
//! (`gd32f10x_fwdgt.h`, `gd32f1x0_fwdgt.h`): `CTL` `0x00` (the key register), `PSC` `0x04` (the
//! divider code, /4 to /256), `RLD` `0x08` (the 12-bit reload), `STAT` `0x0C` (`PUD` 0, `RUD` 1,
//! an update to `PSC` or `RLD` still crossing into the IRC40K domain). The F1x0 also has a window
//! register, `WND` `0x10`, with its own busy flag `WUD`; its reset value `0x0FFF` leaves the window
//! off, and this module never writes it, so there is one model and no `hal_fwdgt` cfg.
//!
//! [`FreeWatchdog::start`] writes the unlock key, `PSC` and `RLD`, waits for both updates to
//! finish, then the reload and start keys. That is `runtime-hal`'s order, run on both GD32 parts
//! by the firmware's wdg-bench; the vendor's `fwdgt_config` instead waits for each busy flag before
//! its write, which at boot (both flags clear) reaches the same state.

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::descriptor::DESCRIPTOR;
use crate::error::WatchdogError;
use crate::reg::Reg32;

const CTL: u32 = 0x00;
const PSC: u32 = 0x04;
const RLD: u32 = 0x08;
const STAT: u32 = 0x0C;

const KEY_UNLOCK: u32 = 0x5555;
const KEY_RELOAD: u32 = 0xAAAA;
const KEY_START: u32 = 0xCCCC;

const STAT_PUD: u32 = 1 << 0;
const STAT_RUD: u32 = 1 << 1;

/// The largest reload, the 12-bit `RLD` field.
const RELOAD_MAX: u32 = 0x0FFF;
/// The largest `PSC` code (/256).
const PSC_MAX: u32 = 6;

/// The IRC40K's nominal frequency. The oscillator is loosely trimmed (tens of percent on some
/// parts), so a period computed against it is approximate; [`WdgTimeout`] rounds up so the nominal
/// period is never shorter than asked.
const IRC40K_HZ: u64 = 40_000;

/// Polls the `PUD`/`RUD` wait may spend. An update crosses in a few IRC40K cycles (about 100 us,
/// a few thousand core cycles at 72 MHz), far inside it.
const UPDATE_SPINS: u32 = 100_000;

/// Every chip carries the free watchdog; a descriptor without its base fails the build here.
const FWDGT_BASE: u32 = match DESCRIPTOR.addrs.get(PeriphLabel::Fwdgt) {
    Some(b) => b,
    None => panic!("the descriptor has no FWDGT base"),
};
const RCU_BASE: u32 = match DESCRIPTOR.addrs.get(PeriphLabel::Rcu) {
    Some(b) => b,
    None => panic!("the descriptor has no RCU base"),
};

/// A requested watchdog period.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WdgTimeout {
    millis: u32,
}

impl WdgTimeout {
    /// A period of `ms` milliseconds.
    pub const fn from_millis(ms: u32) -> WdgTimeout {
        WdgTimeout { millis: ms }
    }

    /// The `(PSC code, reload)` pair for the period.
    ///
    /// The period is `4 << code` times `reload + 1` IRC40K ticks. The ticks wanted are
    /// `ceil(40_000 * ms / 1000)`; the code is the smallest (the finest step) whose reload,
    /// `ceil(ticks / divider) - 1`, fits 12 bits, so the nominal period is at or above the request.
    /// Zero asks for the shortest period, `(0, 0)`; longer than /256 with a full reload (about
    /// 26 s) is clamped to that.
    const fn resolve(self) -> (u32, u32) {
        let ticks = (IRC40K_HZ * self.millis as u64).div_ceil(1000);
        if ticks == 0 {
            return (0, 0);
        }
        let mut code = 0;
        while code <= PSC_MAX {
            let reload = ticks.div_ceil(4 << code) - 1;
            if reload <= RELOAD_MAX as u64 {
                return (code, reload as u32);
            }
            code += 1;
        }
        (PSC_MAX, RELOAD_MAX)
    }
}

/// The running free watchdog. Once started it cannot be stopped short of a reset.
#[derive(Debug)]
pub struct FreeWatchdog {
    _private: (),
}

impl FreeWatchdog {
    /// Hold the watchdog's counter while a debugger halts the core (`DBG_CTL0.FWDGT_HOLD`), so a
    /// halted SWD session is not reset out from under. No effect without a debugger.
    pub fn freeze_on_debug_halt() {
        crate::dbg::debug_hold_fwdgt();
    }

    /// Start the IRC40K and wait for it stable, then program and start the watchdog at `timeout`.
    /// `LsiNotStable` if the oscillator never reports stable, `Timeout` if the `PSC`/`RLD` update
    /// never completes; in both cases the watchdog has not been started.
    pub fn start(_chip: &Chip, timeout: WdgTimeout) -> Result<FreeWatchdog, WatchdogError> {
        crate::clock::enable_lsi(RCU_BASE)?;
        let (code, reload) = timeout.resolve();
        let reg = |offset| Reg32::new(FWDGT_BASE, offset);
        reg(CTL).write(KEY_UNLOCK);
        reg(PSC).write(code);
        reg(RLD).write(reload);
        let mut budget = UPDATE_SPINS;
        while reg(STAT).read() & (STAT_PUD | STAT_RUD) != 0 {
            budget -= 1;
            if budget == 0 {
                return Err(WatchdogError::Timeout);
            }
        }
        reg(CTL).write(KEY_RELOAD);
        reg(CTL).write(KEY_START);
        Ok(FreeWatchdog { _private: () })
    }

    /// Reload the counter (the reload key), every main-loop pass.
    #[inline]
    pub fn feed(&mut self) {
        Reg32::new(FWDGT_BASE, CTL).write(KEY_RELOAD);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The nominal period of a pair, in IRC40K ticks.
    fn ticks(code: u32, reload: u32) -> u64 {
        (4u64 << code) * (u64::from(reload) + 1)
    }

    #[test]
    fn the_firmware_500ms_is_divide_by_8() {
        // 20_000 ticks: /4 needs reload 4999 (over 12 bits), /8 needs 2499.
        assert_eq!(WdgTimeout::from_millis(500).resolve(), (1, 2499));
        assert_eq!(ticks(1, 2499), 20_000);
    }

    #[test]
    fn short_periods_take_the_finest_divider() {
        assert_eq!(WdgTimeout::from_millis(100).resolve(), (0, 999));
        assert_eq!(WdgTimeout::from_millis(1).resolve(), (0, 9));
    }

    #[test]
    fn the_stock_6500ms_period() {
        // 260_000 ticks: /64 with reload ceil(4062.5) - 1 = 4062.
        assert_eq!(WdgTimeout::from_millis(6_500).resolve(), (4, 4062));
    }

    #[test]
    fn every_period_rounds_up_and_fits_the_fields() {
        for ms in (0..30_000u32).step_by(7) {
            let (code, reload) = WdgTimeout::from_millis(ms).resolve();
            assert!(code <= PSC_MAX && reload <= RELOAD_MAX, "{ms} ms");
            let want = (IRC40K_HZ * u64::from(ms)).div_ceil(1000);
            let max = ticks(PSC_MAX, RELOAD_MAX);
            assert!(ticks(code, reload) >= want.min(max), "{ms} ms");
            // The finest divider that fits: one step finer would overflow the reload.
            if code > 0 && want <= max {
                assert!(want.div_ceil(4 << (code - 1)) - 1 > u64::from(RELOAD_MAX));
            }
        }
    }

    #[test]
    fn zero_and_overlong_requests_clamp() {
        assert_eq!(WdgTimeout::from_millis(0).resolve(), (0, 0));
        assert_eq!(
            WdgTimeout::from_millis(u32::MAX).resolve(),
            (PSC_MAX, RELOAD_MAX)
        );
        // The longest period: 256 * 4096 / 40 kHz = 26.2 s.
        assert_eq!(WdgTimeout::from_millis(26_214).resolve(), (6, 4095));
        assert_eq!(
            WdgTimeout::from_millis(26_215).resolve(),
            (PSC_MAX, RELOAD_MAX)
        );
    }
}
