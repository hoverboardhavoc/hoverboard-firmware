//! ble-atprobe: BENCH DIAGNOSTIC (not a deliverable). Sends each AT bring-up command to the onboard
//! TTC2541 one at a time and captures the module's UART reply CONTINUOUSLY (tight RBNE poll, no delayed
//! read that would overrun the 1-byte RX register) into a fixed RAM buffer readable over SWD. Purpose:
//! find why our `ble` crate's `AT+NAME` does not take effect on this module while `AT`/`AT+MODE=DATA` do
//! (RoboDurden's identical bytes work). Run on a COLD module (power-cycle first) so it is in command mode.
//!
//! Read the result over SWD: `nm` the `AT_DIAG` symbol, then dump 384 bytes. Layout: 6 records of 64
//! bytes each = [cmd_id:u8, reply_len:u8, reply[0..62]]. cmd_id: 1=AT 2=NAME 3=CON 4=ADV 5=SET 6=MODE.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use embedded_io::{ReadReady, Write};
    use panic_halt as _;

    use embedded_hal::delay::DelayNs;
    use runtime_hal::clock::{ClockConfig, ClockSource};
    use runtime_hal::{detect_chip, Delay, PeriphLabel, Serial};

    const RESET_8M: ClockConfig = ClockConfig {
        sysclk_hz: 8_000_000,
        wait_states: 0,
        source: ClockSource::Irc8m,
        pll_mul: 2,
        ahb_psc: 1,
        apb1_psc: 1,
        apb2_psc: 1,
    };

    const REC: usize = 64; // bytes per record: cmd_id, reply_len, reply[0..62]
    const N: usize = 10;

    /// Fixed RAM capture buffer (read over SWD by `nm` symbol). N records of 64 bytes.
    #[no_mangle]
    static mut AT_DIAG: [u8; REC * N] = [0; REC * N];

    /// Read-only inspection queries to identify the module firmware (TTC2541 dialect). NO destructive
    /// commands (no RENEW/DEFAULT/RESET/CLEAR). cmd_id, bytes.
    const CMDS: [(u8, &[u8]); N] = [
        (1, b"AT\r\n"),
        (2, b"AT+VERSION\r\n"),
        (3, b"AT+VERSION?\r\n"),
        (4, b"AT+HELP\r\n"),
        (5, b"AT+NAME?\r\n"),
        (6, b"AT+ADDR?\r\n"),
        (7, b"AT+LADDR?\r\n"),
        (8, b"AT+ROLE?\r\n"),
        (9, b"AT+BAUD?\r\n"),
        (10, b"AT+PIN?\r\n"),
    ];

    #[entry]
    fn main() -> ! {
        let cp = cortex_m::Peripherals::take().unwrap();
        let chip = detect_chip().unwrap();
        let gpiob = chip.gpiob().unwrap().split();
        let mut serial = Serial::new(
            &chip,
            &RESET_8M,
            PeriphLabel::Usart2,
            (gpiob.pb10, gpiob.pb11),
            ble::at::BAUD,
        )
        .unwrap();
        let mut delay = Delay::new(cp.SYST, 8_000_000);

        // Settle after boot so the module (also cold) is ready for the first AT.
        delay.delay_ms(400);

        for (i, (cmd_id, bytes)) in CMDS.iter().enumerate() {
            // Send the whole command as one buffer (like RoboDurden's btSend).
            let _ = serial.write_all(bytes);
            let _ = serial.flush();

            // Capture the reply CONTINUOUSLY for a ~350 ms window: poll RBNE tightly so each byte is
            // taken before the next overruns the 1-byte register. read_ready() + a 1-byte read.
            let base = i * REC;
            let mut len: usize = 0;
            // ~350 ms at 8 MHz: tight loop, each iter is a couple register reads (~few cycles).
            for _ in 0..400_000u32 {
                if serial.read_ready().unwrap_or(false) {
                    if let Ok(Some(b)) = read_one(&mut serial) {
                        if len < REC - 2 {
                            unsafe {
                                AT_DIAG[base + 2 + len] = b;
                            }
                            len += 1;
                        }
                    }
                }
            }
            unsafe {
                AT_DIAG[base] = *cmd_id;
                AT_DIAG[base + 1] = len as u8;
            }
        }

        // Done: busy-spin so the buffer stays readable over SWD.
        loop {
            nop();
        }
    }

    /// Read exactly one available byte (non-blocking; caller gated on read_ready).
    fn read_one(serial: &mut Serial) -> Result<Option<u8>, ()> {
        use embedded_io::Read;
        let mut one = [0u8; 1];
        match serial.read(&mut one) {
            Ok(1) => Ok(Some(one[0])),
            _ => Ok(None),
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
