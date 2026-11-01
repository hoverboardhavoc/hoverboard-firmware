// SPDX-License-Identifier: GPL-3.0-or-later
//! `serial`: the serial adapters on silicon over the inter-board link (specs/hal.md section 8.4,
//! `serial` and `usart-rx`): USART1 on PA2/PA3 at the link's 115,200 baud, `SplitSerial` over a
//! `DmaTx` and a `RingBufferedRx` (the firmware's link stack), between the two boards of a pair.
//!
//! Both boards of a GD32F103 + GD32F130 pair (the rover's) run this image built for their chip. The
//! GD32F10x board (descriptor `node_mcu` 1) is side A, the GD32F1x0 board side B; on a pair of two
//! GD32F130s one board is built with `serial-side-a`. Stage 1: B sends ready tokens (`0x5A`) every
//! 10 ms; A, on the first token, streams 4,096 bytes of a known pattern through `Write`; B counts
//! what `Read` returns until 4,096 bytes or 500 ms of silence. Stage 2 is the same the other way.
//! Leading tokens before a stage's first data byte are discarded (the patterns start at 3 and 11).
//!
//! Result words: 2 = side (0 A, 1 B), 3 = bytes received in this board's receive stage, 4 = bytes
//! that did not match the pattern, 5 = line errors, 6 = ring laps, 7 = transmit timeouts, 8 = bytes
//! sent, 10 = transmit timeouts after the stream's writes, 11 = after its tail, 12 = after the last
//! ready token, 9 = verdict (1 = 4,096 received, 0 wrong, 0 errors, 0 laps, 0 timeouts).
#![no_std]
#![no_main]

#[cfg(target_os = "none")]
mod image {
    use core::ptr::addr_of_mut;
    use cortex_m_rt::entry;
    use embedded_io::{Read, ReadReady, Write};
    use hal::clock::{self, ClockConfig};
    use hal::descriptor::DESCRIPTOR;
    use hal::{Chip, DmaTx, PeriphLabel, RingBufferedRx, SplitSerial, Usart};
    use hal_bench::{cycles, finish, put, wait_ms};
    use panic_halt as _;

    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    const N: u32 = 4096;
    const TOKEN: u8 = 0x5A;

    static mut RING: [u8; 256] = [0; 256];
    static mut TXBUF: [u8; 64] = [0; 64];

    const fn pattern(seed: u8, k: u32) -> u8 {
        (k as u8).wrapping_mul(7).wrapping_add(seed)
    }

    type Link = SplitSerial<RingBufferedRx>;

    fn send_stream(s: &mut Link, seed: u8) -> u32 {
        let mut sent = 0u32;
        let mut chunk = [0u8; 32];
        while sent < N {
            for (i, b) in chunk.iter_mut().enumerate() {
                *b = pattern(seed, sent + i as u32);
            }
            let _ = s.write_all(&chunk);
            sent += chunk.len() as u32;
        }
        put(10, u32::from(s.tx_timeouts()));
        // No `flush`: its poll budget is 12,000 iterations costed at 24 cycles each, about 1.3 ms
        // of real time at the loop's measured speed, shorter than a 32-byte frame's 2.8 ms on the
        // wire, so it counts a timeout on a frame that completes (runtime-hal's constants, the same
        // behaviour; the firmware never flushes). The tail is waited out here instead.
        wait_ms(10);
        put(11, u32::from(s.tx_timeouts()));
        sent
    }

    /// Receive one stage, sending a token every 10 ms until the first data byte when `beckon`.
    fn receive_stream(s: &mut Link, seed: u8, beckon: bool) -> (u32, u32) {
        let (mut got, mut wrong) = (0u32, 0u32);
        let mut last = cycles();
        let mut last_token = 0u32;
        let mut buf = [0u8; 64];
        loop {
            if got == 0 && beckon && cycles().wrapping_sub(last_token) > 10 * 72_000 {
                let _ = s.write_all(&[TOKEN]);
                last_token = cycles();
                put(12, u32::from(s.tx_timeouts()));
            }
            if s.read_ready().unwrap_or(false) {
                let n = s.read(&mut buf).unwrap_or(0);
                for &b in &buf[..n] {
                    if got == 0 && b == TOKEN {
                        continue;
                    }
                    if b != pattern(seed, got) {
                        wrong += 1;
                    }
                    got += 1;
                }
                if n > 0 {
                    last = cycles();
                }
            }
            if got >= N || (got > 0 && cycles().wrapping_sub(last) > 500 * 72_000) {
                return (got, wrong);
            }
        }
    }

    /// Wait for the peer's ready token.
    fn await_token(s: &mut Link) {
        let mut buf = [0u8; 16];
        loop {
            if s.read_ready().unwrap_or(false) {
                let n = s.read(&mut buf).unwrap_or(0);
                if buf[..n].contains(&TOKEN) {
                    return;
                }
            }
        }
    }

    #[entry]
    fn main() -> ! {
        let Some(chip) = Chip::take() else { finish(3) };
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            finish(3);
        }
        hal::irq::enable_cycle_counter();
        hal::irq::apply_motor_era_priorities(chip.irq());
        // SAFETY: the vectors these bring-ups unmask are the HAL's own; enable interrupts once.
        unsafe { cortex_m::interrupt::enable() };

        let side_a = DESCRIPTOR.node_mcu == 1 || cfg!(feature = "serial-side-a");
        put(2, u32::from(!side_a));

        let (Ok(tx), Ok(rx)) = (chip.pin(0x02), chip.pin(0x03)) else {
            finish(3)
        };
        let Ok(usart) = Usart::new(&chip, &CLOCK, PeriphLabel::Usart1, (tx, rx), 115_200) else {
            finish(3)
        };
        let (utx, urx) = usart.split();
        // SAFETY: the one formation of each &'static mut.
        let (ring, txbuf) = unsafe { (&mut *addr_of_mut!(RING), &mut *addr_of_mut!(TXBUF)) };
        let Ok(rx) = RingBufferedRx::new(&chip, urx, PeriphLabel::Usart1, ring) else {
            finish(3)
        };
        let Ok(tx) = DmaTx::new(&chip, utx, txbuf, CLOCK.sysclk_hz, 115_200) else {
            finish(3)
        };
        let mut s = SplitSerial::new(tx, rx, CLOCK.sysclk_hz);

        let (got, wrong, sent);
        if side_a {
            await_token(&mut s);
            sent = send_stream(&mut s, 3);
            let r = receive_stream(&mut s, 11, true);
            (got, wrong) = r;
        } else {
            let r = receive_stream(&mut s, 3, true);
            (got, wrong) = r;
            wait_ms(20);
            await_token(&mut s);
            sent = send_stream(&mut s, 11);
        }
        put(3, got);
        put(4, wrong);
        put(5, u32::from(s.line_errors()));
        put(6, u32::from(s.lap_overruns()));
        put(7, u32::from(s.tx_timeouts()));
        put(8, sent);
        let ok = got == N
            && wrong == 0
            && s.line_errors() == 0
            && s.lap_overruns() == 0
            && s.tx_timeouts() == 0;
        put(9, u32::from(ok));
        finish(3)
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
