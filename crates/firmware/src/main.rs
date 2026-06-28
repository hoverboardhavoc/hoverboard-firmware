//! The universal firmware binary: ONE image that detects which GD32 it is on at boot and runs
//! everywhere (F103 master, F130 slave, 12-FET). There is no per-part build, the binary detects its
//! silicon at runtime and adapts (specs/firmware.md).
//!
//! It is thin: a `main` that wires together libraries it does not own (`store` + its `FmcFlash`, and
//! `runtime-hal`'s `detect_chip`). MVP scope is **boot safe -> detect -> mount the store -> the bare
//! housekeeping loop**; the link and the control layers fill in later (roadmap.md L6-L9).
//!
//! Boot sequence (specs/firmware.md, "Boot sequence"):
//!   1. cortex-m-rt reset (SP/PC set, .data/.bss initialized) -- before `main`.
//!   2. Boot safe: nothing that could drive a motor is touched. The MVP has no motor code at all;
//!      the gate is implicit (we never arm a bridge), but the rule exists first by design.
//!   3. `detect_chip()` -- fail loud if detection fails: the firmware cannot run without knowing its
//!      silicon, so a failed detect panics (panic-halt) rather than guessing a register layout.
//!   4. `Store::mount(FmcFlash::new(&chip))` -- the store replays its log; absent keys read defaults.
//!   5. The bare housekeeping loop.
//!
//! On a host target (where it cannot link as a cortex-m image, nor the target-gated HAL) it degrades
//! to an empty `main`, so a host `cargo build`/`cargo test` over the workspace stays green; the real
//! image is only ever built for the chip. (Same degrade pattern as store-test / dummy-test.)

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use link::{Link, SerialTransport};
    use net::walk::{Emits, Responder, MAX_PORTS, PORT_SWD};
    use panic_halt as _;
    use runtime_hal::descriptor::ClockPath;
    use runtime_hal::detect_chip;
    use store::{FmcFlash, Store};
    use swd_mailbox::{EpochWatch, Mailbox, MailboxSerial, MAILBOX_BASE};

    /// This firmware's L3 protocol/firmware version, reported in `NODE_HELLO`.
    const FW_VER: u16 = 0x0001;

    /// The mailbox L2 link's reassembly buffer size. The mailbox carries single-fragment L3/config
    /// PDUs (<= `net::walk::MAX_PDU` = 64 B), so a small buffer suffices - keeping `Link` off the tight
    /// 8 KiB-image stack/RAM (the default ~4 KiB `MAX_PACKET` would blow it).
    const MAILBOX_PACKET: usize = 256;

    /// Idle poll-cycles (no inbound) the responder waits, while probing, before emitting `PORTS`. Kept
    /// short so it fires well within the controller's per-request retransmit window (a long window
    /// would let each retransmitted `PROBE_PORTS` restart the probe and reset this counter, so `PORTS`
    /// never gets sent). A probe reply over the slow MEM-AP bridge may not be recorded within it, but
    /// the exact `PORTS` classification of the upstream mailbox port is not load-bearing (it reports no
    /// downstream neighbour either way; the master's own address came from first contact).
    const PROBE_IDLE: u32 = 50_000;

    #[entry]
    fn main() -> ! {
        // Boot safe: nothing that could drive a motor is touched (no motor code in the MVP).

        // Initialize the SWD mailbox header FIRST, before any bridge could attach: write
        // magic/version/offsets/caps and zero the indices + epoch/epoch_ack. The mailbox occupies a
        // fixed RESERVED region [MAILBOX_BASE, +REGION_LEN) at the bottom of SRAM (memory.x starts the
        // linked RAM above it), so it is indeterminate at reset and the linker never touches it; without
        // this init the bridge's magic check (Attach step 2) reads garbage. SAFETY: the reserved region
        // is REGION_LEN bytes at the fixed base, owned only here; accessed only through volatile
        // reads/writes via the handle.
        let mailbox = unsafe { Mailbox::from_raw(MAILBOX_BASE as *mut u8) };
        mailbox.init_header();

        // Detect the silicon. Fail loud: the firmware cannot run without knowing its part, so a
        // failed detect panics (panic-halt) rather than guessing a register layout.
        let chip = detect_chip().unwrap();
        let mcu = match chip.clock() {
            ClockPath::F10xRcc => 1, // F10x (the wired master)
            ClockPath::F1x0Rcu => 2, // F1x0 (the wired slave)
        };

        // Mount the store at the detected top-of-flash. FmcFlash derives the absolute store region
        // from the Chip; mount replays the log (absent keys read defaults). Held for the loop so the
        // net Responder can persist `node_address` / `CONFIG_*` writes.
        let mut flash = FmcFlash::new(&chip);
        let mut store = Store::mount(&mut flash).unwrap();

        // L3 over the one SWD-mailbox L2 link. The mailbox is port 0 (an SWD port); a configured board
        // would add its UART/BLE links here. The Responder restores its persisted `node_address` (a
        // board reassigned in a past session reports it), services this link, and persists `CONFIG_*`.
        let mut responder = Responder::new(1, [PORT_SWD; MAX_PORTS], mcu, FW_VER);
        responder.restore_addr(&store);
        let mut link = Link::<_, MAILBOX_PACKET>::new(SerialTransport::new(
            MailboxSerial::firmware(mailbox),
            swd_mailbox::FRAME_CAPACITY,
        ));
        let mut epoch_watch = EpochWatch::new(mailbox);
        let mut probe_idle: u32 = 0;
        let mut rx = [0u8; 256];
        let mut pdu = [0u8; net::walk::MAX_PDU];

        // The cooperative service loop. Busy-spin, NEVER wfi: a wfi park with no DBG_CTL0 debug-hold
        // bits locks SWD re-attach on the GD32 (see the spec). Each tick:
        loop {
            // 1. Epoch handshake: a new bridge session bumped `epoch`. Flush the inbound ring (the
            //    EpochWatch did it), reset the byte-stream framer, then write `epoch_ack` so the bridge
            //    (which waits for epoch_ack == epoch) only produces once we are fully reset.
            if epoch_watch.poll() {
                link.transport_mut().reset();
                epoch_watch.ack();
            }

            // 2. Drain inbound L2 packets -> the net Responder; its replies -> the outbound ring.
            let mut saw_inbound = false;
            while let Some(frame) = link.poll_recv(&mut rx) {
                let n = frame.len().min(pdu.len());
                pdu[..n].copy_from_slice(&frame[..n]);
                saw_inbound = true;
                let mut emits = Emits::new();
                responder.ingest(0, &pdu[..n], &mut store, &mut emits);
                send_emits(&mut link, &emits);
            }

            // 3. Probe window: once the responder is probing its ports (on PROBE_PORTS), wait out a
            //    short idle so any probe reply is recorded, then emit PORTS.
            if responder.probing() {
                probe_idle = if saw_inbound {
                    0
                } else {
                    probe_idle.saturating_add(1)
                };
                if probe_idle >= PROBE_IDLE {
                    let mut emits = Emits::new();
                    responder.poll_probe(&mut emits);
                    send_emits(&mut link, &emits);
                    probe_idle = 0;
                }
            } else {
                probe_idle = 0;
            }

            nop(); // preemptible housekeeping slot (the future control ISR)
        }
    }

    /// Send each of the responder's emitted PDUs out the one mailbox L2 link (port 0). Best-effort
    /// (L2 is best-effort; the controller retransmits the acknowledged control plane).
    fn send_emits<T: link::Transport, const N: usize>(link: &mut Link<T, N>, emits: &Emits) {
        for e in emits {
            let _ = link.send(&e.bytes);
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
