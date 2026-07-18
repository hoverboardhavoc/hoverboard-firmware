//! L2 Tier-2 on-silicon validator (`specs/l2.md`, "Tier 2 - master <-> slave on silicon").
//!
//! One universal image for the wired GD32 pair. It detects the family and takes a role:
//!   - **F10x (master / driver):** runs the test sequence and records the outcome.
//!   - **F1x0 (slave / responder):** echoes every reassembled packet straight back, forever.
//!
//! Both run **L2 over the inter-board UART** (USART1, PA2 TX / PA3 RX, `link::INTER_BOARD_BAUD`
//! 8N1 - the proven inter-board link, M1 milestone). The L2 logic is the HAL-free `link` crate: TX
//! fragments a packet and frames each fragment (`SOF | len | frag-hdr | chunk | CRC-16`); the wire
//! I/O both ways is `runtime-hal`'s `SplitSerial<RingBufferedRx>` embedded-io adapter (polled TX
//! half + circular-DMA RX; the adapter owns the IDLE-latch and overrun semantics below its API,
//! `runtime-hal specs/serial-adapters.md`), and the received bytes feed `link`'s `StreamFramer` +
//! `Reassembler` - the SOF/len/CRC framer is the framing authority, exactly as the spec requires.
//!
//! The master validates the spec's Tier-2 checks and writes them to `LINK_OBS`:
//!   1. a single-fragment packet round-trips (frames cross intact both directions);
//!   2. a forced multi-fragment packet (larger than the bench frame capacity) reassembles;
//!   3. the per-frame CRC catches an injected bit error (a corrupted frame is dropped, no echo), and
//!      the link resyncs and the next good packet still round-trips.
//!
//! **Bench L2 frame capacity is reduced to 32 B here** (production UART uses ~255, `specs/l2.md`): it
//! keeps the reassembly buffers within the F130 slave's 8 KiB RAM and lets a small packet force the
//! multi-fragment path. The mechanism under test - DMA+IDLE capture, SOF/len/CRC framing, atomic
//! fragmentation/reassembly - is identical at any capacity.
//!
//! Read the result over SWD: `nm` the `LINK_OBS` symbol on each board, dump the struct. The slave
//! never stops (it must answer whenever the master runs); its `LINK_OBS` shows live echo counts.
//! Busy-spin forever, NEVER `wfi` (a bare `wfi` with `DBG_CTL0 = 0` locks GD32 SWD re-attach).

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use core::ptr::addr_of_mut;

    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use panic_halt as _;

    use embedded_io::{Read, Write};
    use link::{encode_stream_frame, fragment, Reassembler, StreamFramer};
    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::descriptor::ClockPath;
    use runtime_hal::irq::{install, RamVectorTable, MAX_VECTORS};
    use runtime_hal::{detect_chip, PeriphLabel, RingBufferedRx, SplitSerial, Usart};
    use test_shared::obs_store;

    // --- clock / link parameters ----------------------------------------------------------------

    /// The **production 72 MHz tree** (IRC8M -> PLL), brought up via `configure_tree`, so this bench
    /// validates L2 in the same clock regime the shipping firmware runs in (DMA servicing latency,
    /// USART IDLE detection, baud divisor, interrupt response). At 72 MHz USART1's input is APB1 =
    /// 36 MHz; the HAL computes the BRR for `link::INTER_BOARD_BAUD` from it (the usart-rx S2
    /// path). Both boards derive the bit clock from their own IRC8M via PLL, so the cross-board
    /// baud error is IRC8M-trim-dominated (at 460800 the BRR error is +0.16%/board and the
    /// measured inter-board skew 0.37%, inside 8N1 margin; `specs/l2.md`, "Baud raised").
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    /// The inter-board UART baud, read from its one owner (`link::INTER_BOARD_BAUD`).
    const BAUD: u32 = link::INTER_BOARD_BAUD;

    /// Bench L2 frame capacity (`frag-hdr` + chunk), reduced from the production ~255 to keep buffers
    /// inside the F130's 8 KiB RAM and to force fragmentation with a small packet.
    const FRAME_CAP: usize = 32;
    /// Usable chunk per frame: capacity minus the one `frag-hdr` byte.
    const CHUNK_CAP: usize = FRAME_CAP - 1;
    /// Largest stream frame on the wire for the bench capacity: `SOF + len + (frag-hdr..chunk) + CRC`.
    const MAX_WIRE_FRAME: usize = 2 + FRAME_CAP + 2;
    /// Reassembly + packet-buffer bound: >= 16 fragments x CHUNK_CAP (= 496).
    const PKT_MAX: usize = 512;
    /// DMA ring capacity. >= the max wire frame with comfortable margin for back-to-back fragments
    /// ("a buffer sized to the max L2 frame", with headroom so a 2-fragment burst never laps).
    const DMA_CAP: usize = 256;

    // --- test packets (master) ------------------------------------------------------------------

    /// Warm-up probe (single fragment).
    const PING: [u8; 4] = [0xF0, 0x01, 0x02, 0x03];
    /// Single-fragment payload (<= CHUNK_CAP, one frame, one byte of overhead).
    const S1: [u8; 8] = [0xA1, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
    /// Forced multi-fragment payload: 50 bytes > CHUNK_CAP (31) => 2 fragments (31 + 19).
    const S2_LEN: usize = 50;
    /// Recovery payload sent after the corrupted frame (must round-trip => the link resynced).
    const REC: [u8; 4] = [0xC3, 0xAB, 0xCD, 0xEF];

    /// Fill `out` with the deterministic S2 packet (value = 0xB0 ^ i) and return its length.
    fn build_s2(out: &mut [u8]) -> usize {
        for (i, b) in out.iter_mut().enumerate().take(S2_LEN) {
            *b = 0xB0u8.wrapping_add(i as u8);
        }
        S2_LEN
    }

    // --- spin budgets -----------------------------------------------------------------------------

    // Spin budgets, in empty RX-poll passes. At 72 MHz a pass is a few hundred cycles, so these are
    // ~0.2-1.5 s each (scaled ~9x from the prior 8 MHz values to hold the same wall-clock margins):
    // ample for an echo (which round-trips in well under 50 ms) while keeping a full run ~2 s so the
    // SWD observation window is not fragile.
    const WARM_TRIES: u32 = 150;
    const WARM_BUDGET: u32 = 360_000;
    const RESP_BUDGET: u32 = 1_080_000;
    /// Wait long enough to be sure a dropped (bad-CRC) frame produced NO echo (an echo would arrive in
    /// well under this); the only phase that ever spins its full budget.
    const CRC_BUDGET: u32 = 540_000;

    // --- the SWD-readable result block ----------------------------------------------------------

    #[repr(C)]
    struct Obs {
        /// 0x4C32_4F42 ("L2OB"), written when the board reaches steady state.
        magic: u32,
        /// 1 = master (F10x), 2 = slave (F1x0).
        role: u8,
        /// Non-zero if `detect_chip` failed.
        detect_err: u8,

        // --- master fields ---
        /// Warm-up echo round-tripped (slave is alive and the link works).
        warmed: u8,
        /// Warm-up attempts used.
        warm_tries: u8,
        /// Single-fragment echo: length received / matched the sent packet.
        s1_recv: u8,
        s1_match: u8,
        /// Multi-fragment echo: length received / matched.
        s2_recv: u8,
        s2_match: u8,
        /// The injected-bit-error frame produced NO echo (the CRC dropped it).
        crc_dropped: u8,
        /// The good packet sent after it round-tripped (the link resynced).
        crc_recovered: u8,
        /// All master checks passed.
        pass: u8,

        // --- slave fields ---
        /// Packets echoed.
        echo_count: u16,
        /// Length of the last packet echoed.
        last_len: u8,
        /// Loop heartbeat (proves the slave is running even with no traffic).
        alive: u16,
    }

    const MAGIC: u32 = 0x4C32_4F42;

    /// The single SWD-readable result instance (read by `nm LINK_OBS`).
    #[no_mangle]
    static mut LINK_OBS: Obs = Obs {
        magic: 0,
        role: 0,
        detect_err: 0,
        warmed: 0,
        warm_tries: 0,
        s1_recv: 0,
        s1_match: 0,
        s2_recv: 0,
        s2_match: 0,
        crc_dropped: 0,
        crc_recovered: 0,
        pass: 0,
        echo_count: 0,
        last_len: 0,
        alive: 0,
    };

    // --- static buffers (the DMA ring + the RAM vector table must be 'static) --------------------

    static mut DMA_BUF: [u8; DMA_CAP] = [0; DMA_CAP];
    static mut VECTORS: RamVectorTable = RamVectorTable {
        slots: [0; MAX_VECTORS],
    };

    // --- entry ----------------------------------------------------------------------------------

    #[entry]
    fn main() -> ! {
        let chip = match detect_chip() {
            Ok(c) => c,
            Err(_) => {
                obs_store!(LINK_OBS, detect_err, 1);
                halt();
            }
        };
        let is_master = match chip.clock() {
            ClockPath::F10xRcc => true,
            ClockPath::F1x0Rcu => false,
        };
        obs_store!(LINK_OBS, role, if is_master { 1 } else { 2 });

        // Bring up the production 72 MHz tree (IRC8M -> PLL) so the link runs in the shipping clock
        // regime. Both boards do this independently before any UART traffic.
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            halt();
        }

        // GPIOA carries the inter-board USART1 pins PA2 (TX) / PA3 (RX). Split once to configure them.
        let gpioa = match chip.gpioa() {
            Ok(p) => p.split(),
            Err(_) => halt(),
        };

        // USART1 on PA2/PA3, link::INTER_BOARD_BAUD 8N1: configures the GPIO AF + base, RX/TX
        // enabled. The BRR is computed from the 72 MHz tree's APB1 = 36 MHz input clock.
        let usart = match Usart::new(
            &chip,
            &CLOCK,
            PeriphLabel::Usart1,
            (gpioa.pa2, gpioa.pa3),
            BAUD,
        ) {
            Ok(u) => u,
            Err(_) => halt(),
        };

        // Split into owned halves (specs/usart-split.md): the RX half is consumed by
        // RingBufferedRx, then both halves ride the SplitSerial embedded-io adapter.
        let (tx, usart_rx) = usart.split();

        // Route interrupts through the RAM vector table and enable them BEFORE arming the DMA RX (the
        // RingBufferedRx::new contract: VTOR flipped before it unmasks the USART/DMA IRQs).
        // SAFETY: RAM init done, no peripheral IRQ enabled yet, VECTORS is a 'static aligned table.
        unsafe { install(&mut *addr_of_mut!(VECTORS), chip.irq()) };
        // SAFETY: enabling interrupts after the table is installed; handlers are registered by `new`.
        unsafe { cortex_m::interrupt::enable() };

        // Arm DMA-driven RX (circular DMA + USART IDLE) over the 'static ring. Consumes `usart_rx`.
        // SAFETY: DMA_BUF is a 'static, the sole DMA ring for this single receiver.
        let dma_buf =
            unsafe { core::slice::from_raw_parts_mut(addr_of_mut!(DMA_BUF) as *mut u8, DMA_CAP) };
        let ring = match RingBufferedRx::new(&chip, usart_rx, PeriphLabel::Usart1, dma_buf) {
            Ok(r) => r,
            Err(_) => halt(),
        };
        // The one wire seam: every TX and RX byte crosses the HAL adapter's embedded-io traits.
        let mut serial = SplitSerial::new(tx, ring);

        let mut framer: StreamFramer = StreamFramer::new();
        let mut reasm: Reassembler<PKT_MAX> = Reassembler::new();
        let mut scratch = [0u8; 64];
        let mut out = [0u8; PKT_MAX];
        let mut pid: u8 = 0;

        if is_master {
            run_master(
                &mut serial,
                &mut framer,
                &mut reasm,
                &mut scratch,
                &mut out,
                &mut pid,
            );
        } else {
            run_slave(
                &mut serial,
                &mut framer,
                &mut reasm,
                &mut scratch,
                &mut out,
                &mut pid,
            );
        }
    }

    /// The bench's wire seam: the HAL embedded-io adapter over the split inter-board USART.
    type Wire = SplitSerial<RingBufferedRx>;

    // --- master: drive the test sequence --------------------------------------------------------

    fn run_master(
        serial: &mut Wire,
        framer: &mut StreamFramer,
        reasm: &mut Reassembler<PKT_MAX>,
        scratch: &mut [u8],
        out: &mut [u8],
        pid: &mut u8,
    ) -> ! {
        // Warm-up: send PING until it round-trips, so the rest of the sequence runs against a known-up
        // slave (handles the master booting before/after the slave).
        let mut warmed = false;
        let mut tries = 0u8;
        for _ in 0..WARM_TRIES {
            tries = tries.saturating_add(1);
            send_packet(serial, pid, &PING);
            if let Some(k) = await_packet(serial, framer, reasm, scratch, out, WARM_BUDGET) {
                if out[..k] == PING[..] {
                    warmed = true;
                    break;
                }
            }
        }
        obs_store!(LINK_OBS, warmed, warmed as u8);
        obs_store!(LINK_OBS, warm_tries, tries);

        // 1. Single-fragment round-trip.
        send_packet(serial, pid, &S1);
        let s1_ok = match await_packet(serial, framer, reasm, scratch, out, RESP_BUDGET) {
            Some(k) => {
                obs_store!(LINK_OBS, s1_recv, k as u8);
                out[..k] == S1[..]
            }
            None => {
                obs_store!(LINK_OBS, s1_recv, 0);
                false
            }
        };
        obs_store!(LINK_OBS, s1_match, s1_ok as u8);

        // 2. Forced multi-fragment round-trip.
        let mut s2 = [0u8; S2_LEN];
        let s2n = build_s2(&mut s2);
        send_packet(serial, pid, &s2[..s2n]);
        let s2_ok = match await_packet(serial, framer, reasm, scratch, out, RESP_BUDGET) {
            Some(k) => {
                obs_store!(LINK_OBS, s2_recv, k as u8);
                out[..k] == s2[..s2n]
            }
            None => {
                obs_store!(LINK_OBS, s2_recv, 0);
                false
            }
        };
        obs_store!(LINK_OBS, s2_match, s2_ok as u8);

        // 3. Injected bit error: a structurally valid frame with one chunk byte flipped so the CRC
        //    fails. The slave's framer must drop it (no echo); then a good packet must still round-trip
        //    (the link resynced past the bad frame).
        send_corrupted_frame(serial);
        let dropped = await_packet(serial, framer, reasm, scratch, out, CRC_BUDGET).is_none();
        obs_store!(LINK_OBS, crc_dropped, dropped as u8);

        send_packet(serial, pid, &REC);
        let recovered = match await_packet(serial, framer, reasm, scratch, out, RESP_BUDGET) {
            Some(k) => out[..k] == REC[..],
            None => false,
        };
        obs_store!(LINK_OBS, crc_recovered, recovered as u8);

        let pass = warmed && s1_ok && s2_ok && dropped && recovered;
        obs_store!(LINK_OBS, pass, pass as u8);

        write_magic();
        halt();
    }

    // --- slave: echo every reassembled packet, forever -----------------------------------------

    fn run_slave(
        serial: &mut Wire,
        framer: &mut StreamFramer,
        reasm: &mut Reassembler<PKT_MAX>,
        scratch: &mut [u8],
        out: &mut [u8],
        pid: &mut u8,
    ) -> ! {
        write_magic(); // the slave reaches steady state immediately (it is a pure responder)
        let mut echo_count = 0u16;
        let mut alive = 0u16;
        loop {
            alive = alive.wrapping_add(1);
            if alive & 0x03FF == 0 {
                obs_store!(LINK_OBS, alive, alive); // periodic heartbeat (not every iteration)
            }
            if let Some(k) = drain_once(serial, framer, reasm, scratch, out) {
                // Echo the reassembled packet straight back (re-fragmented by send_packet).
                let mut echo = [0u8; PKT_MAX];
                echo[..k].copy_from_slice(&out[..k]);
                send_packet(serial, pid, &echo[..k]);
                echo_count = echo_count.wrapping_add(1);
                obs_store!(LINK_OBS, echo_count, echo_count);
                obs_store!(LINK_OBS, last_len, k as u8);
            }
            nop();
        }
    }

    // --- L2 TX: fragment a packet and frame each fragment onto the wire --------------------------

    /// Send one opaque packet over L2: fragment to `CHUNK_CAP`, wrap each fragment in a stream frame
    /// (`SOF | len | frag-hdr | chunk | CRC-16`), and write the bytes through the adapter's
    /// `embedded-io` `Write`. `pid` increments per packet (wraps 0..7).
    fn send_packet(serial: &mut Wire, pid: &mut u8, packet: &[u8]) {
        let p = *pid;
        let _ = fragment(packet, CHUNK_CAP, p, |hdr, chunk| {
            let mut l2 = [0u8; FRAME_CAP];
            l2[0] = hdr.encode();
            l2[1..1 + chunk.len()].copy_from_slice(chunk);
            let mut wire = [0u8; MAX_WIRE_FRAME];
            if let Ok(n) = encode_stream_frame(&l2[..1 + chunk.len()], &mut wire) {
                let _ = serial.write_all(&wire[..n]);
            }
        });
        *pid = (*pid + 1) & 0x07;
    }

    /// Build a structurally valid stream frame, flip one chunk byte so its CRC no longer matches, and
    /// write it. The receiver's framer must drop it on the CRC check.
    fn send_corrupted_frame(serial: &mut Wire) {
        // L2 frame = frag-hdr 0x00 + a 4-byte chunk.
        let l2 = [0x00u8, 0xDE, 0xAD, 0xBE, 0xEF];
        let mut wire = [0u8; MAX_WIRE_FRAME];
        if let Ok(n) = encode_stream_frame(&l2, &mut wire) {
            // Corrupt a chunk byte (index 3 = first chunk byte: SOF, len, frag-hdr, chunk...). The
            // frame structure (SOF/len) stays intact; only the CRC now mismatches.
            wire[4] ^= 0xFF;
            let _ = serial.write_all(&wire[..n]);
        }
    }

    // --- L2 RX: DMA read -> StreamFramer -> Reassembler -----------------------------------------

    /// One RX pass: read whatever the adapter has buffered (its DMA ring; the IDLE latch and any
    /// overrun recovery are owned below its API), feed the StreamFramer, push emitted L2 frames into
    /// the Reassembler. Returns `Some(len)` (copied into `out`) when a packet completes.
    fn drain_once(
        serial: &mut Wire,
        framer: &mut StreamFramer,
        reasm: &mut Reassembler<PKT_MAX>,
        scratch: &mut [u8],
        out: &mut [u8],
    ) -> Option<usize> {
        let n = serial.read(scratch).unwrap_or(0);
        if n == 0 {
            return None;
        }
        let mut completed: Option<usize> = None;
        framer.feed(&scratch[..n], &mut |l2| {
            if l2.is_empty() || completed.is_some() {
                return;
            }
            if let Some(pkt) = reasm.push(l2[0], &l2[1..]) {
                let k = pkt.len().min(out.len());
                out[..k].copy_from_slice(&pkt[..k]);
                completed = Some(k);
            }
        });
        completed
    }

    /// Spin until a packet completes or `budget` empty passes elapse.
    fn await_packet(
        serial: &mut Wire,
        framer: &mut StreamFramer,
        reasm: &mut Reassembler<PKT_MAX>,
        scratch: &mut [u8],
        out: &mut [u8],
        budget: u32,
    ) -> Option<usize> {
        let mut spins = 0u32;
        loop {
            if let Some(k) = drain_once(serial, framer, reasm, scratch, out) {
                return Some(k);
            }
            spins += 1;
            if spins > budget {
                return None;
            }
            nop();
        }
    }

    // --- result helpers -------------------------------------------------------------------------

    fn write_magic() {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        obs_store!(LINK_OBS, magic, MAGIC);
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
