//! ble-loopback: the board-side peer for the ADB BLE-throughput harness (`specs/ble.md`, "Board-side
//! peer: raw byte loopback").
//!
//! It does the minimum: bring the onboard BLE module into transparent data mode via the `ble` crate,
//! then on the returned `Pipe` do ONE thing forever, `read` bytes and `write` them straight back. It is
//! BELOW `link`: no frame parser, no opcodes, no `LinkPort`. It echoes raw bytes verbatim; only the phone
//! app parses the harness's test envelope (to score the returned stream). Bench-only, not a production
//! path.
//!
//! UART via `runtime-hal`: the loopback constructs a `runtime-hal` `Serial` and hands it to
//! `Module::bring_up`, then `read`/`write`s the resulting `ble::Pipe` (which passes through to that
//! USART). The `ble` crate stays HAL-free; this firmware owns the serial.
//!
//! Wiring (the bench F103 master): the CC2541 BLE module is on `PeriphLabel::Usart2` (the ST USART3
//! block), PB10 (TX) / PB11 (RX), 8N1 at the module's fixed 9600 data-mode baud (`ble::at::BAUD`). On the
//! F10x master PB10/PB11 is the default function (no AF mux), which is why `PeriphLabel::Usart2`'s fixed
//! family-default AF is a non-issue here.
//!
//! On a host target (where it cannot link as a cortex-m image, nor the target-gated HAL) it degrades to
//! an empty `main`, so a bare host `cargo build` / `cargo test` over the workspace stays green; the real
//! image is only ever built for the chip. (Same degrade pattern as firmware / store-test / dummy-test.)

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use embedded_io::{Read, Write};
    use panic_halt as _;

    use runtime_hal::clock::ClockConfig;
    use runtime_hal::{detect_chip, Delay, PeriphLabel, Serial};

    /// The clock the image runs at: the 8 MHz reset tree, from its one owner (never calls
    /// `configure_tree`; only the source of truth for `Serial::new`'s BAUD math and the SysTick
    /// `Delay` - the BLE module's 9600 data-mode baud divides cleanly from it).
    const CLOCK: ClockConfig = ClockConfig::RESET_8M;

    #[entry]
    fn main() -> ! {
        // Claim SysTick for the bring-up delay: the application owns the one Peripherals::take()
        // (runtime-hal DECISIONS #13; ordering vs detect_chip is unconstrained).
        let cp = cortex_m::Peripherals::take().unwrap();

        // Detect the silicon at runtime (fail loud on an unknown part: panic_halt, not a guessed layout).
        let chip = detect_chip().unwrap();

        // Split GPIOB (enables the port clock); PB10 (TX) / PB11 (RX) carry the BLE module's UART.
        let gpiob = chip.gpiob().unwrap().split();

        // Bring up the module's UART on Usart2 (the ST USART3 block), PB10/PB11, at the module's fixed
        // 9600 data-mode baud. Serial::new CONSUMES the named pin handles, configures them, enables the
        // peripheral clock, and programs the BRR from the running 8 MHz clock.
        let serial = Serial::new(
            &chip,
            &CLOCK,
            PeriphLabel::Usart2,
            (gpiob.pb10, gpiob.pb11),
            ble::at::BAUD,
        )
        .unwrap();

        // SysTick-backed delay at the 8 MHz reset clock; paces the AT bring-up sequence (~248 ms/step).
        let mut delay = Delay::new(cp.SYST, CLOCK.sysclk_hz);

        // Bring the module into transparent data mode via the `ble` crate. A silent / wedged module
        // fails Error::Probe (no hang); panic_halt halts, and the bench operator power-cycles + reflashes.
        // The advertised name: a short (<=10 char) config value baked at build time via the
        // HB_BLE_NAME env (default "hbloop"). Kept short because the CC2541/TTC2541 silently won't
        // advertise an over-long name (17-char "hb-bench-loopback" never appeared on a scan; RoboDurden's
        // proven names are "Pal"/3 and "CLASSYWALK2"/10). The module PERSISTS its name in NV and phones
        // cache it, so the bench builds a UNIQUE name per flash (HB_BLE_NAME=hbNNN) to tell a fresh
        // bring-up apart from a stale/cached advert; the harness is told the same name (and falls back to
        // the module's stable MAC).
        let mut pipe = ble::Module::new(option_env!("HB_BLE_NAME").unwrap_or("hbloop"))
            .con_interval(16)
            .adv_interval(32)
            .bring_up(serial, &mut delay)
            .unwrap();

        // The one job: raw byte loopback. Read whatever the BLE pipe delivers and write it straight back.
        // No envelope parsing here (the phone scores the echoed stream). `read` blocks for at least one
        // byte (embedded-io Read contract); on a line error it busy-loops on (drops it and retries).
        let mut buf = [0u8; 256];
        loop {
            match pipe.read(&mut buf) {
                Ok(0) => nop(),
                Ok(n) => {
                    // Echo exactly the bytes received, verbatim. write_all retries short writes.
                    let _ = pipe.write_all(&buf[..n]);
                }
                Err(_) => nop(), // a line error: drop it and keep the loopback alive
            }
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
