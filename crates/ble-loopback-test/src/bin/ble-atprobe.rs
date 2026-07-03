//! ble-atprobe: BENCH DIAGNOSTIC (not a deliverable). Sends each AT bring-up command to the onboard
//! TTC2541 one at a time and captures the module's UART reply CONTINUOUSLY (a tight window of
//! non-blocking `Serial::read` drains; the HAL adapter owns the RBNE/overrun discipline below its
//! API) into a fixed RAM buffer readable over SWD. Purpose:
//! find why our `ble` crate's `AT+NAME` does not take effect on this module while `AT`/`AT+MODE=DATA` do
//! (RoboDurden's identical bytes work). Run on a COLD module (power-cycle first) so it is in command mode.
//!
//! Read the result over SWD: `nm` the `AT_DIAG` symbol, then dump 640 bytes. Layout: 10 records of 64
//! bytes each = [cmd_id:u8, reply_len:u8, reply[0..62]]. cmd_id 1..10 = the CMDS table order (1=AT,
//! then the read-only inspection queries: VERSION, VERSION?, HELP, NAME?, ADDR?, LADDR?, ROLE?,
//! BAUD?, PIN?). Known dialect: 1 -> AT+OK\r\n; the queries -> AT+ERR=2\r\n (verified 2026-07-03).

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use embedded_io::{Read, Write};
    use panic_halt as _;

    use embedded_hal::delay::DelayNs;
    use runtime_hal::clock::ClockConfig;
    use runtime_hal::{detect_chip, Delay, PeriphLabel, Serial};

    /// The 8 MHz reset tree, from its one owner (never `configure_tree`'d; the baud-math + SysTick
    /// source of truth).
    const CLOCK: ClockConfig = ClockConfig::RESET_8M;

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
        // The application owns the one Peripherals::take() (runtime-hal DECISIONS #13; ordering vs
        // detect_chip is unconstrained).
        let cp = cortex_m::Peripherals::take().unwrap();
        let chip = detect_chip().unwrap();
        let gpiob = chip.gpiob().unwrap().split();
        let mut serial = Serial::new(
            &chip,
            &CLOCK,
            PeriphLabel::Usart2,
            (gpiob.pb10, gpiob.pb11),
            ble::at::BAUD,
        )
        .unwrap();
        let mut delay = Delay::new(cp.SYST, CLOCK.sysclk_hz);

        // Settle after boot so the module (also cold) is ready for the first AT.
        delay.delay_ms(400);

        for (i, (cmd_id, bytes)) in CMDS.iter().enumerate() {
            // Send the whole command as one buffer (like RoboDurden's btSend).
            let _ = serial.write_all(bytes);
            let _ = serial.flush();

            // Capture the reply CONTINUOUSLY for a ~350 ms window: the adapter's non-blocking read
            // drains every available byte per pass (and owns the RBNE/overrun discipline), so no
            // byte overruns the 1-byte RX register.
            let base = i * REC;
            let mut len: usize = 0;
            let mut chunk = [0u8; 16];
            // ~350 ms at 8 MHz: tight loop, each empty pass is a few register reads.
            for _ in 0..400_000u32 {
                let n = serial.read(&mut chunk).unwrap_or(0);
                for &b in chunk.iter().take(n) {
                    if len < REC - 2 {
                        unsafe {
                            AT_DIAG[base + 2 + len] = b;
                        }
                        len += 1;
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
}

#[cfg(not(target_os = "none"))]
fn main() {}
