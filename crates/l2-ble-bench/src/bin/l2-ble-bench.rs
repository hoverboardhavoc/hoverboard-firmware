//! L2 Tier-3 master-side validator (`specs/l2.md`, "Tier 3 - master <-> Android (the BLE link)").
//!
//! This firmware settles the spec's one bench-gated **open question**: the master only ever sees the
//! CC2541's *UART* output, so does each module-forwarded BLE write (one ATT transaction, <=20 B) land
//! on the master's UART as one idle-gap-delimited burst the master can frame with **DMA + IDLE**, or
//! does the module split / coalesce frames across the bridge (which would force an in-band length
//! delimiter on the BLE L2 frame)?
//!
//! The BLE frame carries an **in-band length delimiter**: `[ len ][ frag-hdr ][ chunk ]` (`len` =
//! frag-hdr..chunk), still no SOF/CRC (the BLE link CRC protects each transaction). This was forced by
//! the Tier-3 measurement below: a lean `[ frag-hdr ][ chunk ]` frame round-trips for single-frame
//! packets, but the module **coalesces** back-to-back forwarded writes (the multi-fragment case) into
//! one UART burst, so the idle gap alone cannot delimit them. The receive path brings the module into
//! data mode via the `ble` crate, arms `runtime-hal` `RingBufferedRx` (circular DMA + USART IDLE) on the
//! module USART, accumulates each idle-delimited burst, **splits it on the length byte** into frames,
//! and pushes each to `link`'s `Reassembler`; a completed packet is echoed back to the phone.
//!
//! The evidence is in `LINK_OBS`, which records TWO distinct things:
//!   - **Raw idle-burst sizes** (`raw_max_burst`, `raw_oversize`, `raw_burst_hist`): the length of each
//!     DMA/IDLE-delivered burst measured BEFORE length-parsing (bytes accumulated since the previous
//!     IDLE gap). A raw burst > 20 B is the **direct coalesce signal** - the module merged >= 2
//!     forwarded writes into one burst, which the lean `[ frag-hdr ][ chunk ]` frame (no length) could
//!     not have split. This reproduces "lean would fail" from THIS shipped firmware, with no need to
//!     resurrect the retired lean build.
//!   - **Parsed-frame stats** (`max_frame` <= 20, `frames_parsed`, `packets`, `echoes`): what the length
//!     delimiter then recovered. `frames_parsed > idle_count` means a coalesced burst was split back
//!     into its frames, which reassemble and round-trip where the lean frame could not.
//!
//! Read over SWD: `nm LINK_OBS`, dump the struct. The phone side (the throughput harness, L2 wrapper)
//! scores round-trip integrity into its result JSON. Busy-spin forever, NEVER `wfi` (GD32 SWD-lockout).

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use core::ptr::addr_of_mut;

    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use panic_halt as _;

    use link::{fragment, Reassembler};
    use runtime_hal::clock::{ClockConfig, ClockSource};
    use runtime_hal::descriptor::ClockPath;
    use runtime_hal::irq::{install, RamVectorTable, MAX_VECTORS};
    use runtime_hal::{
        detect_chip, Delay, Oversampling, PeriphLabel, RingBufferedRx, Serial, Usart, UsartConfig,
        UsartFrame,
    };

    /// The 8 MHz reset IRC8M, no PLL: the proven module bring-up clock (ble-loopback). The CC2541's
    /// 9600 data-mode baud divides cleanly from it; the tree already runs here at reset, so
    /// `configure_tree` is not called. (Tier 3 keeps the module's proven clock; the 72 MHz Tier-2
    /// validation is a separate, inter-board concern.)
    const RESET_8M: ClockConfig = ClockConfig {
        sysclk_hz: 8_000_000,
        wait_states: 0,
        source: ClockSource::Irc8m,
        pll_mul: 2, // unused (no PLL).
        ahb_psc: 1,
        apb1_psc: 1,
        apb2_psc: 1,
    };

    /// Bench L2 frame capacity for the BLE datagram link: the hard 20-byte ATT payload.
    const FRAME_CAP: usize = 20;
    /// Usable chunk per frame: 20 - 1 len - 1 frag-hdr (the length delimiter costs one byte).
    const CHUNK_CAP: usize = FRAME_CAP - 2; // 18
    /// Reassembly + packet bound: 16 fragments x 19 = 304 (the BLE mtu_hint).
    const PKT_MAX: usize = 320;
    /// DMA ring for the module RX. >> one ATT burst (20 B); margin to absorb back-to-back writes at the
    /// ~40 frames/s ceiling without lapping before the loop drains.
    const DMA_CAP: usize = 256;
    /// Persistent length-parse stream buffer cap. One frame is <=20 B, but the module coalesces
    /// back-to-back forwarded writes; a generous cap so a coalesced multi-frame burst is held whole and
    /// split by the length delimiter (and so the raw idle-burst it produces is measured, not clipped).
    const BURST_MAX: usize = 128;

    /// The advertised name (kept short; the module silently won't advertise an over-long name).
    const NAME: &str = "hbL2";

    // --- the SWD-readable result block ----------------------------------------------------------

    /// Largest raw idle-burst length tracked individually in the histogram; bucket [HIST_MAX] catches
    /// anything at or beyond it. Sized past one ATT payload (20) so coalesced multi-frame bursts (e.g.
    /// 2x ~20 B = ~37 B) resolve, not just saturate.
    const HIST_MAX: usize = 40;

    #[repr(C)]
    struct Obs {
        /// 0x4C33_4F42 ("L3OB"), written once the receive path is armed (the module came up).
        magic: u32,
        /// 1 = master (F10x, expected). The module DMA RX channel is F10x-only.
        role: u8,
        /// Non-zero if `detect_chip` failed.
        detect_err: u8,
        /// Non-zero if `Module::bring_up` did not get its AT+OK. On a COLD boot that means the module is
        /// not in command mode (power-cycle needed); on a re-flash (the module already in data mode from
        /// a prior boot) it is EXPECTED - the firmware then assumes data mode and proceeds, so the
        /// adapter can be iterated without a power-cycle each time. **Only trust assume-data-mode when
        /// corroborated by `packets > 0`:** `probe_err == 1` with `packets == 0` means the module is not
        /// bridging (it needs a cold power-cycle), even though `magic`/`alive` look healthy.
        probe_err: u8,
        /// Non-zero if arming `RingBufferedRx` (DMA self-check) failed.
        dma_err: u8,

        /// Total bytes read from the DMA ring (proves whether ANY module-forwarded byte reaches the
        /// master's RX at all - the first fork in diagnosing a 0-burst result).
        raw_bytes: u32,
        /// Times the USART IDLE latch fired = the number of raw idle-delimited bursts measured.
        idle_count: u16,

        // --- RAW idle-burst sizes (measured BEFORE length-parsing = the coalesce signal) ---
        /// Largest raw idle-burst, in bytes (sum of bytes between two IDLE gaps), saturating at 255. A
        /// value > 20 means the module COALESCED >= 2 forwarded writes into one burst - which the lean
        /// (no-length) frame could not have split. This is the direct "lean would fail" evidence.
        raw_max_burst: u8,
        /// Count of raw idle-bursts > 20 B (coalesce events).
        raw_oversize: u16,

        // --- PARSED-frame stats (what the length delimiter recovered) ---
        /// Largest PARSED L2 frame length (`len` + frag-hdr + chunk, always <= 20).
        max_frame: u8,
        /// Frames peeled off the RX byte stream by the length delimiter (across coalesced/split chunks).
        frames_parsed: u16,
        /// Packets reassembled from the parsed frames.
        packets: u16,
        /// Packets echoed back to the phone.
        echoes: u16,
        /// Length of the last reassembled packet.
        last_len: u8,

        /// Loop heartbeat (proves the firmware is live even with no BLE traffic).
        alive: u16,
        /// Histogram of RAW idle-burst lengths: index = burst length, [HIST_MAX] = ">= HIST_MAX".
        raw_burst_hist: [u16; HIST_MAX + 1],
    }

    const MAGIC: u32 = 0x4C33_4F42;

    #[no_mangle]
    static mut LINK_OBS: Obs = Obs {
        magic: 0,
        role: 0,
        detect_err: 0,
        probe_err: 0,
        dma_err: 0,
        raw_bytes: 0,
        idle_count: 0,
        raw_max_burst: 0,
        raw_oversize: 0,
        max_frame: 0,
        frames_parsed: 0,
        packets: 0,
        echoes: 0,
        last_len: 0,
        alive: 0,
        raw_burst_hist: [0; HIST_MAX + 1],
    };

    macro_rules! store {
        ($field:ident, $val:expr) => {{
            // SAFETY: single-threaded firmware; the only writer is this code path, reads are external.
            unsafe {
                let p = addr_of_mut!(LINK_OBS);
                core::ptr::addr_of_mut!((*p).$field).write_volatile($val);
            }
        }};
    }

    /// Bump the raw idle-burst histogram bucket `len` (saturating), without a full struct rewrite.
    fn raw_hist_bump(len: usize) {
        let idx = len.min(HIST_MAX);
        // SAFETY: idx <= HIST_MAX, in bounds; single writer.
        unsafe {
            let p = addr_of_mut!(LINK_OBS);
            let cell = core::ptr::addr_of_mut!((*p).raw_burst_hist[idx]);
            cell.write_volatile(cell.read_volatile().saturating_add(1));
        }
    }

    static mut DMA_BUF: [u8; DMA_CAP] = [0; DMA_CAP];
    static mut VECTORS: RamVectorTable = RamVectorTable {
        slots: [0; MAX_VECTORS],
    };

    #[entry]
    fn main() -> ! {
        let cp = cortex_m::Peripherals::take().unwrap();

        let chip = match detect_chip() {
            Ok(c) => c,
            Err(_) => {
                store!(detect_err, 1);
                halt();
            }
        };
        // The module DMA RX channel (GD USART2_RX) is F10x-only; the bench master is the F103.
        store!(
            role,
            match chip.clock() {
                ClockPath::F10xRcc => 1,
                ClockPath::F1x0Rcu => 2,
            }
        );

        // GPIOB carries the module USART pins PB10 (TX) / PB11 (RX).
        let gpiob = match chip.gpiob() {
            Ok(p) => p.split(),
            Err(_) => halt(),
        };
        let serial = match Serial::new(
            &chip,
            &RESET_8M,
            PeriphLabel::Usart2,
            (gpiob.pb10, gpiob.pb11),
            ble::at::BAUD,
        ) {
            Ok(s) => s,
            Err(_) => halt(),
        };
        let mut delay = Delay::new(cp.SYST, 8_000_000);

        // Bring the module into transparent data mode. `bring_up` CONSUMES the serial either way; on
        // failure the module did not ack (cold-boot => power-cycle needed; re-flash => already in data
        // mode from a prior boot). We record probe_err but PROCEED to arm DMA RX regardless (assume data
        // mode), so the adapter can be iterated without a power-cycle each time. Serial::new already
        // configured the USART2 base, so the re-arm below works whichever branch ran.
        match ble::Module::new(NAME)
            .con_interval(16)
            .adv_interval(32)
            .bring_up(serial, &mut delay)
        {
            Ok(pipe) => {
                let _ = pipe.into_inner();
            }
            Err(_) => {
                // Assume the module is already in data mode (a re-flash without a power-cycle) and
                // proceed, so the adapter can be iterated on the bench without a power-cycle each time.
                // CAVEAT: this is only valid if the module really is bridging - which is ONLY confirmed
                // by `packets > 0` later. `probe_err == 1` with `packets == 0` means the module is NOT
                // bridging (it needs a cold power-cycle); `magic`/`alive` looking healthy does not refute
                // that. Bring-up is deliberately NOT made fatal here (we want bench iteration), so the
                // reader must apply the `packets > 0` corroboration, not trust the green-looking block.
                store!(probe_err, 1);
            }
        }

        // A polled-TX handle on the module USART base for the echo (the AF/base were configured by
        // Serial::new; bring_up reprograms only the peripheral registers). Built before arming RX.
        let cfg = UsartConfig {
            usart: PeriphLabel::Usart2,
            tx: 0,
            rx: 0,
            baud: ble::at::BAUD,
            frame: UsartFrame::EIGHT_N_ONE,
            oversampling: Oversampling::By16,
        };
        let tx = match Usart::bring_up(&chip, &RESET_8M, &cfg) {
            Ok(u) => u,
            Err(_) => halt(),
        };

        // Route interrupts before arming DMA RX (the RingBufferedRx::new contract).
        // SAFETY: RAM init done, no peripheral IRQ enabled yet, VECTORS is a 'static aligned table.
        unsafe { install(&mut *addr_of_mut!(VECTORS), chip.irq()) };
        // SAFETY: enabling interrupts after the table is installed.
        unsafe { cortex_m::interrupt::enable() };

        // Arm DMA + IDLE capture on the module USART. A second handle on the same base (the first is the
        // TX alias). Failure (self-check) => dma_err.
        let usart_rx = match Usart::bring_up(&chip, &RESET_8M, &cfg) {
            Ok(u) => u,
            Err(_) => halt(),
        };
        let dma_buf =
            unsafe { core::slice::from_raw_parts_mut(addr_of_mut!(DMA_BUF) as *mut u8, DMA_CAP) };
        let mut rx = match RingBufferedRx::new(&chip, usart_rx, PeriphLabel::Usart2, dma_buf) {
            Ok(r) => r,
            Err(_) => {
                store!(dma_err, 1);
                write_magic();
                halt();
            }
        };

        write_magic(); // the receive path is armed; the module came up

        let mut reasm: Reassembler<PKT_MAX> = Reassembler::new();
        let mut scratch = [0u8; 32];
        let mut stream = [0u8; BURST_MAX]; // persistent length-parse buffer (the byte stream)
        let mut stream_len = 0usize;
        let mut out = [0u8; PKT_MAX];
        let mut pid: u8 = 0;

        let mut frames_parsed = 0u16;
        let mut max_frame = 0u8;
        let mut packets = 0u16;
        let mut echoes = 0u16;
        let mut alive = 0u16;
        let mut raw_bytes = 0u32;
        let mut idle_count = 0u16;
        // Raw idle-burst measurement (the coalesce signal), independent of the length parser:
        // `raw_burst_len` accumulates bytes since the last IDLE gap; on IDLE it is the raw burst length.
        let mut raw_burst_len = 0usize;
        let mut raw_max_burst = 0u8;
        let mut raw_oversize = 0u16;

        loop {
            alive = alive.wrapping_add(1);
            if alive & 0x03FF == 0 {
                store!(alive, alive);
            }

            // The CC2541 bridge does NOT preserve frame boundaries on the UART side: it coalesces
            // back-to-back forwarded writes AND splits a single write across reads (measured Tier-3).
            // So the master treats the RX as a continuous BYTE STREAM and length-parses frames from a
            // persistent buffer (`stream`/`stream_len`): append every DMA byte, then peel off each
            // complete `[ len ][ frag-hdr ][ chunk ]` frame, leaving any partial frame buffered for the
            // next read. SEPARATELY, the raw idle-burst length (`raw_burst_len`, recorded on each IDLE)
            // is the coalesce signal: a raw burst > 20 B is the module merging >= 2 forwarded writes -
            // which the lean (no-length) frame could not have split. IDLE is the burst delimiter for the
            // raw measurement only; the length delimiter, not IDLE, is the framing authority.
            let n = rx.read(&mut scratch).unwrap_or(0);
            if n > 0 {
                raw_bytes = raw_bytes.wrapping_add(n as u32);
                raw_burst_len += n;
                store!(raw_bytes, raw_bytes);
            }
            for &b in &scratch[..n] {
                if stream_len < BURST_MAX {
                    stream[stream_len] = b;
                    stream_len += 1;
                }
            }
            if rx.take_idle() {
                idle_count = idle_count.wrapping_add(1);
                store!(idle_count, idle_count);
                // Record the raw idle-burst just delimited (bytes since the previous IDLE).
                if raw_burst_len > 0 {
                    raw_hist_bump(raw_burst_len);
                    let rb = raw_burst_len.min(u8::MAX as usize) as u8;
                    if rb > raw_max_burst {
                        raw_max_burst = rb;
                        store!(raw_max_burst, raw_max_burst);
                    }
                    if raw_burst_len > FRAME_CAP {
                        raw_oversize = raw_oversize.wrapping_add(1);
                        store!(raw_oversize, raw_oversize);
                    }
                    raw_burst_len = 0;
                }
            }

            // Peel complete frames off the front of the stream buffer.
            while stream_len >= 1 {
                let flen = stream[0] as usize; // frag-hdr + chunk
                                               // Max valid inner frame is frag-hdr(1) + chunk(CHUNK_CAP=18) = 19: the [len] prefix +
                                               // frag-hdr + chunk must fit one 20 B air transaction. So flen in 1..=19; `flen > 19`
                                               // (== FRAME_CAP - 1) is impossible and means the stream desynced. Matches the Kotlin
                                               // mirror's `flen > 19` reject so the two contracts agree.
                if flen == 0 || flen > FRAME_CAP - 1 {
                    // A length that cannot be valid means the stream desynced (a lost byte). Drop one
                    // byte and re-scan (best-effort resync; production uses the SOF/CRC StreamFramer).
                    stream.copy_within(1..stream_len, 0);
                    stream_len -= 1;
                    continue;
                }
                if stream_len < 1 + flen {
                    break; // partial frame: wait for more bytes
                }
                frames_parsed = frames_parsed.wrapping_add(1);
                if (1 + flen) as u8 > max_frame {
                    max_frame = (1 + flen) as u8;
                }
                let hdr = stream[1];
                if let Some(pkt) = reasm.push(hdr, &stream[2..1 + flen]) {
                    let k = pkt.len().min(out.len());
                    out[..k].copy_from_slice(&pkt[..k]);
                    packets = packets.wrapping_add(1);
                    store!(last_len, k as u8);
                    send_packet(&tx, &mut pid, &out[..k]);
                    echoes = echoes.wrapping_add(1);
                }
                stream.copy_within(1 + flen..stream_len, 0);
                stream_len -= 1 + flen;
                store!(frames_parsed, frames_parsed);
                store!(max_frame, max_frame);
                store!(packets, packets);
                store!(echoes, echoes);
            }

            nop();
        }
    }

    /// Send one packet over the BLE L2 path: fragment to CHUNK_CAP and write each
    /// `[ len ][ frag-hdr ][ chunk ]` frame to the module (len = frag-hdr..chunk, so the receiver can
    /// split frames the module coalesces across the bridge). No pacing is needed - the length
    /// delimiter, not an idle gap, separates frames.
    fn send_packet(tx: &Usart, pid: &mut u8, packet: &[u8]) {
        let p = *pid;
        let _ = fragment(packet, CHUNK_CAP, p, |hdr, chunk| {
            tx.write_byte((1 + chunk.len()) as u8); // len: frag-hdr + chunk
            tx.write_byte(hdr.encode());
            for &b in chunk {
                tx.write_byte(b);
            }
        });
        *pid = (*pid + 1) & 0x07;
    }

    fn write_magic() {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        store!(magic, MAGIC);
    }

    /// Busy-spin forever. NEVER `wfi` (GD32 SWD-lockout rule).
    fn halt() -> ! {
        loop {
            nop();
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
