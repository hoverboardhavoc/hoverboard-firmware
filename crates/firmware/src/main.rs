//! The universal firmware binary: ONE image that detects which GD32 it is on at boot and runs
//! everywhere (F103 master, F130 slave, 12-FET). There is no per-part build, the binary detects its
//! silicon at runtime and adapts (specs/firmware.md).
//!
//! It wires the libraries it does not own (`store` + `FmcFlash`, `net`'s L3 `Responder`, `link`'s L2,
//! the `swd-mailbox`, and `runtime-hal`'s detect / clock / USART) into one cooperative service loop:
//! boot-safe -> init the SWD mailbox -> detect -> 72 MHz clock -> mount the store -> bring up the L2
//! links (the SWD mailbox + the inter-board UART) into `net` -> service them forever.
//!
//! L3 over **two** L2 links, both feeding the one `net` Responder:
//!   - **port 0 = the SWD mailbox** (a debugger/host attaches over MEM-AP, no wiring), and
//!   - **port 1 = the inter-board UART** (USART1 PA2/PA3, 115200, 72 MHz - the proven inter-board link),
//!     brought up in **listen** mode (the unconfigured bring-up's link-listen; the BT-probe/BLE phase
//!     is Tier 3, deferred and NOT done here).
//!
//! So the host controller's walk reaches a neighbour board (the slave) THROUGH this board over the
//! UART: a directed `ASSIGN` arriving on the mailbox is forwarded out the UART, the neighbour persists
//! and ACKs, and source-learning routes the reply back. Every board runs this same image.
//!
//! Pin safety: only the **safe** inter-board USART is brought up (USART1 PA2/PA3, clear of any
//! advanced-timer gate pin); there is no motor code and nothing arms a bridge (specs/l3.md, "Pin
//! safety"). Busy-spin, NEVER `wfi` (a wfi with `DBG_CTL0 = 0` locks GD32 SWD re-attach).
//!
//! On a host target it degrades to an empty `main` (it cannot link as a cortex-m image nor the
//! target-gated HAL), so a host `cargo build`/`cargo test` over the workspace stays green.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use core::ptr::addr_of_mut;

    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use embedded_io::{ErrorType, Read, ReadReady, Write};
    use link::{Link, SerialTransport, Transport};
    use net::walk::{Emits, Responder, PORT_SWD, PORT_UART};
    use panic_halt as _;
    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::descriptor::ClockPath;
    use runtime_hal::irq::{install, RamVectorTable, MAX_VECTORS};
    use runtime_hal::{
        detect_chip, Oversampling, PeriphLabel, RingBufferedRx, Usart, UsartConfig, UsartFrame,
    };
    use store::{FmcFlash, Store};
    use swd_mailbox::{EpochWatch, Mailbox, MailboxSerial, MAILBOX_BASE};

    /// This firmware's L3 protocol/firmware version, reported in `NODE_HELLO`.
    const FW_VER: u16 = 0x0001;

    /// The **production 72 MHz tree** (IRC8M -> PLL), the same regime `l2-uart-bench` proved the
    /// inter-board link in (USART1's input is APB1 = 36 MHz; the HAL computes BRR for 115200 from it).
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    /// The inter-board UART baud (the M1-proven inter-board rate).
    const BAUD: u32 = 115_200;

    /// Each L2 link's reassembly buffer size. The links carry single-fragment L3/config PDUs
    /// (<= `net::walk::MAX_PDU` = 64 B), so 128 B holds a whole packet while keeping both `Link`s within
    /// the tight 8 KiB-image RAM budget (the default ~4 KiB `MAX_PACKET` would blow it, and two
    /// 1 KiB vector tables + the store's record buffer already crowd it).
    const PACKET: usize = 128;
    /// The DMA RX ring for the inter-board USART (circular DMA + USART IDLE). >= the max wire frame
    /// (a 64 B PDU stream-framed is ~68 B) with margin for a back-to-back burst.
    const DMA_CAP: usize = 128;
    /// The inter-board UART's L2 frame capacity (frag-hdr + chunk); a whole 64 B PDU rides one fragment.
    const UART_FRAME_CAP: usize = 96;
    /// Each link's `StreamFramer` buffer (the largest stream frame). >= the larger frame_capacity (128,
    /// the mailbox) + 4 stream-overhead = 132; 140 leaves a little headroom. Far below the default
    /// `MAX_STREAM_FRAME` (259), saving ~240 B across the two links' framers in the 8 KiB-image budget.
    const FRAMER_N: usize = 140;
    /// The `UsartSerial` lookahead chunk: bytes pulled from the DMA ring per refill.
    const UART_RX_CHUNK: usize = 32;

    /// Idle poll-cycles (no inbound) the responder waits, while probing, before emitting `PORTS`. Kept
    /// short so it fires well within the controller's per-request retransmit window (a long window lets
    /// each retransmitted `PROBE_PORTS` restart the probe and reset this counter, so `PORTS` never gets
    /// sent). A probe reply over the slow MEM-AP bridge may not be recorded within it, but the upstream
    /// port's `PORTS` classification is not load-bearing (a board's own address came from first contact).
    const PROBE_IDLE: u32 = 50_000;

    /// The DMA RX ring (`'static`, the sole ring for the inter-board receiver).
    static mut DMA_BUF: [u8; DMA_CAP] = [0; DMA_CAP];
    /// The RAM interrupt vector table (`'static`, aligned), for the DMA/USART IDLE IRQs.
    static mut VECTORS: RamVectorTable = RamVectorTable {
        slots: [0; MAX_VECTORS],
    };

    /// An `embedded-io` serial over the inter-board USART: polled TX (`Usart::write_byte`) + DMA RX
    /// (`RingBufferedRx`). `RingBufferedRx::read` is non-blocking (returns 0 when empty) and has no
    /// peek, so a small lookahead chunk backs `ReadReady` (which `SerialTransport` gates `read` on).
    /// Wrapped in `link::SerialTransport`, the same `StreamFramer` carries `l2.md` frames over it - one
    /// L2 code path, the UART is just another byte-stream carrier (the spec's shared shim).
    struct UsartSerial {
        tx: Usart,
        rx: RingBufferedRx,
        buf: [u8; UART_RX_CHUNK],
        head: usize,
        len: usize,
    }

    impl UsartSerial {
        /// Pull a fresh chunk from the DMA ring when the lookahead is drained.
        fn refill(&mut self) {
            if self.head >= self.len {
                let _ = self.rx.take_idle(); // clear the IDLE latch; the StreamFramer owns framing
                self.head = 0;
                self.len = self.rx.read(&mut self.buf).unwrap_or(0);
            }
        }
    }

    impl ErrorType for UsartSerial {
        // A DMA RX read error degrades to "no bytes"; polled TX cannot fail. So the serial is
        // infallible from L2's view.
        type Error = core::convert::Infallible;
    }

    impl Read for UsartSerial {
        fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
            self.refill();
            let n = out.len().min(self.len - self.head);
            out[..n].copy_from_slice(&self.buf[self.head..self.head + n]);
            self.head += n;
            Ok(n)
        }
    }

    impl ReadReady for UsartSerial {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            self.refill();
            Ok(self.len > self.head)
        }
    }

    impl Write for UsartSerial {
        fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
            for &b in data {
                self.tx.write_byte(b);
            }
            Ok(data.len())
        }
        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(()) // write_byte blocks until the byte is on the wire
        }
    }

    #[entry]
    fn main() -> ! {
        // Boot safe: nothing that could drive a motor is touched (no motor code in the MVP).

        // Initialize the SWD mailbox header FIRST, before any bridge could attach (a `.mailbox` reserved
        // region is indeterminate at reset). SAFETY: REGION_LEN bytes at the fixed reserved base, owned
        // only here, accessed only through volatile reads/writes via the handle.
        let mailbox = unsafe { Mailbox::from_raw(MAILBOX_BASE as *mut u8) };
        mailbox.init_header();

        // Detect the silicon (fail loud: a wrong register layout is worse than a halt).
        let chip = detect_chip().unwrap();
        let mcu = match chip.clock() {
            ClockPath::F10xRcc => 1, // F10x (the wired master)
            ClockPath::F1x0Rcu => 2, // F1x0 (the wired slave)
        };

        // Bring up the production 72 MHz tree before the store + UART, so both run in the shipping clock
        // regime (the inter-board baud divisor, flash wait states). Both boards do this independently.
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            halt();
        }

        // Mount the store (held for the loop so the Responder can persist `node_address` / `CONFIG_*`).
        let mut flash = FmcFlash::new(&chip);
        let mut store = Store::mount(&mut flash).unwrap();

        // --- the inter-board UART (USART1 PA2/PA3), the link-listen bring-up (a SAFE USART; no gates) ---
        let gpioa = match chip.gpioa() {
            Ok(p) => p.split(),
            Err(_) => halt(),
        };
        // RX handle (consumed by RingBufferedRx); configures the GPIO AF + base.
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
        // A second handle on the same base for polled TX (reprograms only baud/frame/enable, not the
        // GPIO AF that `new` set), built before arming RX so the DMA setup is last to touch registers.
        let tx_cfg = UsartConfig {
            usart: PeriphLabel::Usart1,
            tx: 0,
            rx: 0,
            baud: BAUD,
            frame: UsartFrame::EIGHT_N_ONE,
            oversampling: Oversampling::By16,
        };
        let tx = match Usart::bring_up(&chip, &CLOCK, &tx_cfg) {
            Ok(u) => u,
            Err(_) => halt(),
        };
        // Route interrupts through the RAM vector table and enable them BEFORE arming DMA RX.
        // SAFETY: RAM init done, no peripheral IRQ enabled yet, VECTORS is a 'static aligned table.
        unsafe { install(&mut *addr_of_mut!(VECTORS), chip.irq()) };
        // SAFETY: the table is installed; RingBufferedRx::new registers + unmasks its handlers.
        unsafe { cortex_m::interrupt::enable() };
        // SAFETY: DMA_BUF is a 'static, the sole DMA ring for this single receiver.
        let dma_buf =
            unsafe { core::slice::from_raw_parts_mut(addr_of_mut!(DMA_BUF) as *mut u8, DMA_CAP) };
        let rx_dma = match RingBufferedRx::new(&chip, usart_rx, PeriphLabel::Usart1, dma_buf) {
            Ok(r) => r,
            Err(_) => halt(),
        };

        // --- L3 over the two L2 links: port 0 = the SWD mailbox, port 1 = the inter-board UART ---
        let mut responder = Responder::new(2, [PORT_SWD, PORT_UART, 0, 0], mcu, FW_VER);
        responder.restore_addr(&store);

        let mut mailbox_link = Link::<_, PACKET>::new(SerialTransport::<_, FRAMER_N>::new(
            MailboxSerial::firmware(mailbox),
            swd_mailbox::FRAME_CAPACITY,
        ));
        let uart_serial = UsartSerial {
            tx,
            rx: rx_dma,
            buf: [0; UART_RX_CHUNK],
            head: 0,
            len: 0,
        };
        let mut uart_link = Link::<_, PACKET>::new(SerialTransport::<_, FRAMER_N>::new(
            uart_serial,
            UART_FRAME_CAP,
        ));

        let mut epoch_watch = EpochWatch::new(mailbox);
        let mut probe_idle: u32 = 0;
        let mut rxbuf = [0u8; PACKET];
        let mut pdu = [0u8; net::walk::MAX_PDU];

        // The cooperative service loop. Busy-spin, NEVER wfi.
        loop {
            // 1. Mailbox epoch handshake (the SWD bridge attaching): flush h2t (the EpochWatch did it),
            //    reset the framer, write epoch_ack.
            if epoch_watch.poll() {
                mailbox_link.transport_mut().reset();
                epoch_watch.ack();
            }

            let mut saw_inbound = false;

            // 2a. Drain the mailbox link (port 0) -> the Responder; route its replies by emit port.
            while let Some(frame) = mailbox_link.poll_recv(&mut rxbuf) {
                let n = frame.len().min(pdu.len());
                pdu[..n].copy_from_slice(&frame[..n]);
                saw_inbound = true;
                let mut emits = Emits::new();
                responder.ingest(0, &pdu[..n], &mut store, &mut emits);
                route_emits(&emits, &mut mailbox_link, &mut uart_link);
            }

            // 2b. Drain the inter-board UART link (port 1) -> the Responder.
            while let Some(frame) = uart_link.poll_recv(&mut rxbuf) {
                let n = frame.len().min(pdu.len());
                pdu[..n].copy_from_slice(&frame[..n]);
                saw_inbound = true;
                let mut emits = Emits::new();
                responder.ingest(1, &pdu[..n], &mut store, &mut emits);
                route_emits(&emits, &mut mailbox_link, &mut uart_link);
            }

            // 3. Probe window: once probing, wait out a short idle (so a probe reply is recorded), then
            //    emit PORTS.
            if responder.probing() {
                probe_idle = if saw_inbound {
                    0
                } else {
                    probe_idle.saturating_add(1)
                };
                if probe_idle >= PROBE_IDLE {
                    let mut emits = Emits::new();
                    responder.poll_probe(&mut emits);
                    route_emits(&emits, &mut mailbox_link, &mut uart_link);
                    probe_idle = 0;
                }
            } else {
                probe_idle = 0;
            }

            nop(); // preemptible housekeeping slot (the future control ISR)
        }
    }

    /// Route the Responder's emitted PDUs to the right L2 link by emit port (0 = mailbox, 1 = UART).
    /// Best-effort (L2 is best-effort; the controller retransmits the acknowledged control plane).
    fn route_emits<TA, const NA: usize, TB, const NB: usize>(
        emits: &Emits,
        port0: &mut Link<TA, NA>,
        port1: &mut Link<TB, NB>,
    ) where
        TA: Transport,
        TB: Transport,
    {
        for e in emits {
            match e.port {
                0 => {
                    let _ = port0.send(&e.bytes);
                }
                1 => {
                    let _ = port1.send(&e.bytes);
                }
                _ => {} // no port 2+ on this board
            }
        }
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
