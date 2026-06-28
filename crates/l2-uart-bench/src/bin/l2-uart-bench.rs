//! L2 Tier-2 on-silicon validator (`specs/l2.md`, "Tier 2 - master <-> slave on silicon").
//!
//! One universal image for the wired GD32 pair. It detects the family and takes a role:
//!   - **F10x (master / driver):** runs the test sequence and records the outcome.
//!   - **F1x0 (slave / responder):** echoes every reassembled packet straight back, forever.
//!
//! Both run **L2 over the inter-board UART** (USART1, PA2 TX / PA3 RX, 115200 8N1 - the proven
//! inter-board link, M1 milestone). The L2 logic is the HAL-free `link` crate: TX fragments a packet
//! and frames each fragment (`SOF | len | frag-hdr | chunk | CRC-16`); RX is **DMA-driven** -
//! `runtime-hal` `RingBufferedRx` (circular DMA + USART IDLE) captures the wire into a buffer sized
//! for the link's frames, and the bytes feed `link`'s `StreamFramer` + `Reassembler`. The IDLE latch
//! (`take_idle`) is the frame-complete hint, but the `StreamFramer` (SOF/len/CRC) stays the framing
//! authority, exactly as the spec requires.
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

    use link::{encode_stream_frame, fragment, Reassembler, StreamFramer};
    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::descriptor::ClockPath;
    use runtime_hal::irq::{install, RamVectorTable, MAX_VECTORS};
    use runtime_hal::{
        detect_chip, Oversampling, PeriphLabel, RingBufferedRx, Usart, UsartConfig, UsartFrame,
    };

    // --- clock / link parameters ----------------------------------------------------------------

    /// The **production 72 MHz tree** (IRC8M -> PLL), brought up via `configure_tree`, so this bench
    /// validates L2 in the same clock regime the shipping firmware runs in (DMA servicing latency,
    /// USART IDLE detection, baud divisor, interrupt response). At 72 MHz USART1's input is APB1 =
    /// 36 MHz; the HAL computes BRR for 115200 from it (proven in runtime-hal's usart-rx S2). Both
    /// boards derive the bit clock from their own IRC8M via PLL, so the cross-board baud error is
    /// IRC8M-trim-dominated (115200 8N1 has margin; confirmed on the bench).
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    /// The inter-board UART baud (the M1-proven inter-board rate).
    const BAUD: u32 = 115_200;

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
        /// Multi-fragment echo: length received / matched / an IDLE boundary was seen during its RX.
        s2_recv: u8,
        s2_match: u8,
        s2_idle: u8,
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
        /// An IDLE boundary was observed at least once on the slave's RX.
        slave_idle: u8,
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
        s2_idle: 0,
        crc_dropped: 0,
        crc_recovered: 0,
        pass: 0,
        echo_count: 0,
        last_len: 0,
        slave_idle: 0,
        alive: 0,
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
                store!(detect_err, 1);
                halt();
            }
        };
        let is_master = match chip.clock() {
            ClockPath::F10xRcc => true,
            ClockPath::F1x0Rcu => false,
        };
        store!(role, if is_master { 1 } else { 2 });

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

        // USART1 on PA2/PA3, 115200 8N1: configures the GPIO AF + base, RX/TX enabled. BRR is computed
        // from the 72 MHz tree's APB1 = 36 MHz input clock.
        let usart_rx = match Usart::new(
            &chip,
            &CLOCK,
            PeriphLabel::Usart1,
            (gpioa.pa2, gpioa.pa3),
            BAUD,
        ) {
            Ok(u) => u,
            Err(_) => halt(),
        };

        // A second handle on the same USART base for polled TX (`write_byte(&self)`). `bring_up`
        // reprograms only the peripheral registers (baud/frame/enable), not the GPIO AF that `new`
        // configured, so the two handles coexist: TX uses TDATA/TC, RX-DMA uses RDATA/DENR/IDLE.
        // Built BEFORE arming RX so the RingBufferedRx setup is the last to touch the registers.
        let cfg = UsartConfig {
            usart: PeriphLabel::Usart1,
            tx: 0,
            rx: 0,
            baud: BAUD,
            frame: UsartFrame::EIGHT_N_ONE,
            oversampling: Oversampling::By16,
        };
        let tx = match Usart::bring_up(&chip, &CLOCK, &cfg) {
            Ok(u) => u,
            Err(_) => halt(),
        };

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
        let mut rx = match RingBufferedRx::new(&chip, usart_rx, PeriphLabel::Usart1, dma_buf) {
            Ok(r) => r,
            Err(_) => halt(),
        };

        let mut framer: StreamFramer = StreamFramer::new();
        let mut reasm: Reassembler<PKT_MAX> = Reassembler::new();
        let mut scratch = [0u8; 64];
        let mut out = [0u8; PKT_MAX];
        let mut pid: u8 = 0;

        if is_master {
            run_master(
                &tx,
                &mut rx,
                &mut framer,
                &mut reasm,
                &mut scratch,
                &mut out,
                &mut pid,
            );
        } else {
            run_slave(
                &tx,
                &mut rx,
                &mut framer,
                &mut reasm,
                &mut scratch,
                &mut out,
                &mut pid,
            );
        }
    }

    // --- master: drive the test sequence --------------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    fn run_master(
        tx: &Usart,
        rx: &mut RingBufferedRx,
        framer: &mut StreamFramer,
        reasm: &mut Reassembler<PKT_MAX>,
        scratch: &mut [u8],
        out: &mut [u8],
        pid: &mut u8,
    ) -> ! {
        let mut idle = false;

        // Warm-up: send PING until it round-trips, so the rest of the sequence runs against a known-up
        // slave (handles the master booting before/after the slave).
        let mut warmed = false;
        let mut tries = 0u8;
        for _ in 0..WARM_TRIES {
            tries = tries.saturating_add(1);
            send_packet(tx, pid, &PING);
            if let Some(k) = await_packet(rx, framer, reasm, scratch, out, &mut idle, WARM_BUDGET) {
                if out[..k] == PING[..] {
                    warmed = true;
                    break;
                }
            }
        }
        store!(warmed, warmed as u8);
        store!(warm_tries, tries);

        // 1. Single-fragment round-trip.
        send_packet(tx, pid, &S1);
        let s1_ok = match await_packet(rx, framer, reasm, scratch, out, &mut idle, RESP_BUDGET) {
            Some(k) => {
                store!(s1_recv, k as u8);
                out[..k] == S1[..]
            }
            None => {
                store!(s1_recv, 0);
                false
            }
        };
        store!(s1_match, s1_ok as u8);

        // 2. Forced multi-fragment round-trip.
        let mut s2 = [0u8; S2_LEN];
        let s2n = build_s2(&mut s2);
        send_packet(tx, pid, &s2[..s2n]);
        let mut s2_idle = false;
        let s2_ok = match await_packet(rx, framer, reasm, scratch, out, &mut s2_idle, RESP_BUDGET) {
            Some(k) => {
                store!(s2_recv, k as u8);
                out[..k] == s2[..s2n]
            }
            None => {
                store!(s2_recv, 0);
                false
            }
        };
        store!(s2_match, s2_ok as u8);
        store!(s2_idle, s2_idle as u8);

        // 3. Injected bit error: a structurally valid frame with one chunk byte flipped so the CRC
        //    fails. The slave's framer must drop it (no echo); then a good packet must still round-trip
        //    (the link resynced past the bad frame).
        send_corrupted_frame(tx);
        let dropped =
            await_packet(rx, framer, reasm, scratch, out, &mut idle, CRC_BUDGET).is_none();
        store!(crc_dropped, dropped as u8);

        send_packet(tx, pid, &REC);
        let recovered = match await_packet(rx, framer, reasm, scratch, out, &mut idle, RESP_BUDGET)
        {
            Some(k) => out[..k] == REC[..],
            None => false,
        };
        store!(crc_recovered, recovered as u8);

        let pass = warmed && s1_ok && s2_ok && dropped && recovered;
        store!(pass, pass as u8);

        write_magic();
        halt();
    }

    // --- slave: echo every reassembled packet, forever -----------------------------------------

    #[allow(clippy::too_many_arguments)]
    fn run_slave(
        tx: &Usart,
        rx: &mut RingBufferedRx,
        framer: &mut StreamFramer,
        reasm: &mut Reassembler<PKT_MAX>,
        scratch: &mut [u8],
        out: &mut [u8],
        pid: &mut u8,
    ) -> ! {
        write_magic(); // the slave reaches steady state immediately (it is a pure responder)
        let mut echo_count = 0u16;
        let mut alive = 0u16;
        let mut idle = false;
        loop {
            alive = alive.wrapping_add(1);
            if alive & 0x03FF == 0 {
                store!(alive, alive); // periodic heartbeat (not every iteration)
            }
            if let Some(k) = drain_once(rx, framer, reasm, scratch, out, &mut idle) {
                // Echo the reassembled packet straight back (re-fragmented by send_packet).
                let mut echo = [0u8; PKT_MAX];
                echo[..k].copy_from_slice(&out[..k]);
                send_packet(tx, pid, &echo[..k]);
                echo_count = echo_count.wrapping_add(1);
                store!(echo_count, echo_count);
                store!(last_len, k as u8);
                store!(slave_idle, idle as u8);
            }
            nop();
        }
    }

    // --- L2 TX: fragment a packet and frame each fragment onto the wire --------------------------

    /// Send one opaque packet over L2: fragment to `CHUNK_CAP`, wrap each fragment in a stream frame
    /// (`SOF | len | frag-hdr | chunk | CRC-16`), and write the bytes polled. `pid` increments per
    /// packet (wraps 0..7).
    fn send_packet(tx: &Usart, pid: &mut u8, packet: &[u8]) {
        let p = *pid;
        let _ = fragment(packet, CHUNK_CAP, p, |hdr, chunk| {
            let mut l2 = [0u8; FRAME_CAP];
            l2[0] = hdr.encode();
            l2[1..1 + chunk.len()].copy_from_slice(chunk);
            let mut wire = [0u8; MAX_WIRE_FRAME];
            if let Ok(n) = encode_stream_frame(&l2[..1 + chunk.len()], &mut wire) {
                for &b in &wire[..n] {
                    tx.write_byte(b);
                }
            }
        });
        *pid = (*pid + 1) & 0x07;
    }

    /// Build a structurally valid stream frame, flip one chunk byte so its CRC no longer matches, and
    /// write it. The receiver's framer must drop it on the CRC check.
    fn send_corrupted_frame(tx: &Usart) {
        // L2 frame = frag-hdr 0x00 + a 4-byte chunk.
        let l2 = [0x00u8, 0xDE, 0xAD, 0xBE, 0xEF];
        let mut wire = [0u8; MAX_WIRE_FRAME];
        if let Ok(n) = encode_stream_frame(&l2, &mut wire) {
            // Corrupt a chunk byte (index 3 = first chunk byte: SOF, len, frag-hdr, chunk...). The
            // frame structure (SOF/len) stays intact; only the CRC now mismatches.
            wire[4] ^= 0xFF;
            for &b in &wire[..n] {
                tx.write_byte(b);
            }
        }
    }

    // --- L2 RX: DMA read -> StreamFramer -> Reassembler -----------------------------------------

    /// One RX pass: read whatever the DMA ring holds, feed the StreamFramer, push emitted L2 frames
    /// into the Reassembler. Returns `Some(len)` (copied into `out`) when a packet completes. Sets
    /// `*idle_seen` if the USART IDLE boundary latched (the frame-complete hint).
    fn drain_once(
        rx: &mut RingBufferedRx,
        framer: &mut StreamFramer,
        reasm: &mut Reassembler<PKT_MAX>,
        scratch: &mut [u8],
        out: &mut [u8],
        idle_seen: &mut bool,
    ) -> Option<usize> {
        let n = rx.read(scratch).unwrap_or(0);
        if rx.take_idle() {
            *idle_seen = true;
        }
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
    #[allow(clippy::too_many_arguments)]
    fn await_packet(
        rx: &mut RingBufferedRx,
        framer: &mut StreamFramer,
        reasm: &mut Reassembler<PKT_MAX>,
        scratch: &mut [u8],
        out: &mut [u8],
        idle_seen: &mut bool,
        budget: u32,
    ) -> Option<usize> {
        let mut spins = 0u32;
        loop {
            if let Some(k) = drain_once(rx, framer, reasm, scratch, out, idle_seen) {
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
