//! The universal firmware binary: ONE image that detects which GD32 it is on at boot and runs
//! everywhere (F103 master, F130 slave, 12-FET). There is no per-part build, the binary detects its
//! silicon at runtime and adapts (specs/firmware.md).
//!
//! It wires the libraries it does not own (`store` + `FmcFlash`, `net`'s L3 `Responder`, `link`'s L2,
//! the `swd-mailbox`, `ble`'s AT bring-up, and `runtime-hal`'s detect / clock / USART) into one
//! cooperative service loop: boot-safe -> init the SWD mailbox -> detect -> 72 MHz clock -> mount the
//! store -> **bring up the L2 links the spec's way** (the BT-probe BLE link + the link-listen UARTs)
//! into `net` -> service them forever.
//!
//! **Unconfigured bring-up (specs/l3.md, "Unconfigured bring-up").** A board with no `LINK_SET` finds
//! its links over the *safe* USARTs (gate-capable pins denied) in two baud phases:
//!   1. **BT-probe (active, polled, 9600).** Send `AT\r\n`; the one USART that answers `AT+OK\r\n` is a
//!      CC2541 BLE module -> run the `ble.md` AT bring-up to transparent data mode and make it an L2
//!      BLE link. Nothing else answers `AT`, so it is unambiguous.
//!   2. **Link-listen (passive, DMA, `link::INTER_BOARD_BAUD`).** The remaining safe USARTs come up as L2 byte-stream
//!      links and just listen for L3 PDUs.
//!
//! Each live port becomes one of the board's `net` ports; the board stays at `0x00` until assigned,
//! then persists its `LINK_SET` (the bitmask of live ports) alongside its `node_address`. A
//! **configured** boot (non-zero `LINK_SET`) brings up exactly that set, never re-running the probe.
//!
//! The SWD mailbox is always port 0 (a debugger/host attaches over MEM-AP, no wiring); the discovered
//! USART links fill the remaining ports: **port 1 = the inter-board UART** (USART1 PA2/PA3, the proven
//! inter-board link), **port 2 = the BLE module** (the onboard CC2541).
//!
//! **One image, two board families, decided by staged data.** The BLE module and the IMU sit on
//! MIRRORED pins across the fleet: the standard family puts its CC2541 on USART2's PB10/PB11 and its
//! IMU on I2C0's PB6/PB7, while the classywalk offroad family puts its CC2541 on USART0's PB6/PB7 and
//! its IMU on I2C1's PB10/PB11. The chip cannot tell them apart - a bench slave and an offroad board
//! are the same GD32F130 part - so neither peripheral is compiled in. The persisted board layout
//! decides both: `LINK_SET` names which USART wiring carries the module, `imu.scl_pin`/`imu.sda_pin`
//! name the IMU's bus, and `board::plumbing::resolve_ports` is the single owner that turns those
//! into this boot's pin assignment (and refuses to hand any USART a pin the IMU holds).
//!
//! Pin safety (specs/l3.md, "Pin safety"): only the safe USARTs are touched (USART0 PB6/PB7, USART1
//! PA2/PA3, USART2 PB10/PB11 - all clear of any advanced-timer gate pin; USART0's DEFAULT mapping
//! PA9/PA10 is denied, being the high-side gates). Busy-spin, NEVER `wfi` (a wfi with
//! `DBG_CTL0 = 0` locks GD32 SWD re-attach).
//!
//! **The integrated control stack (specs/integration.md, slice 7).** On top of the link spine the
//! image runs the orchestrated pipeline: the SysTick ISR ticks the ISR-safe `scheduler` static at
//! 250 Hz (R1); the loop drains the links (routing the delivered control-block PDUs through
//! `linkctl::decode` into the orchestrator inbox), dispatches the due tasks (the 250 Hz control
//! pipeline + the 16 ms input task, both pure `orchestrator` functions over the `SHELL` static),
//! feeds the IWDG AFTER dispatch (R2, the wdg-bench placement), emits the cyclic payload
//! port-directed on the inter-board UART (addressed boards only), samples the arm fact into the
//! responder each pass (R4) and defers the LINK_SET persist while armed. The IMU comes up
//! plan-gated (the first `BoardPlan` consumer, fail-soft to link-only-plus-throttle); MOE
//! enactment stays a recorded seam (R3, pre-motor). Statics safety: the `SHELL` is touched only
//! from the main thread (the loop and the dispatch callbacks are the same context); the
//! `SCHEDULER` is the one ISR/thread crossing and its atomics are the arbitration.
//!
//! On a host target it degrades to an empty `main` (it cannot link as a cortex-m image nor the
//! target-gated HAL), so a host `cargo build`/`cargo test` over the workspace stays green.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

mod arm;
mod motor;

#[cfg(target_os = "none")]
mod firmware {

    use crate::arm;
    use crate::ble_name;
    use crate::link_drain::bounded_drain;
    use crate::motor;
    use crate::probe_window::poll_window_elapsed;
    use board::plumbing::{read_fields, reserved_set, resolve_ports, AllowlistPort, BoardObs};
    use core::mem::MaybeUninit;
    use core::ptr::{addr_of, addr_of_mut};
    use core::sync::atomic::{AtomicU32, Ordering};
    use cortex_m::asm::nop;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m_rt::entry;
    use embedded_hal::digital::OutputPin;
    use embedded_io::{ErrorType, Read, ReadReady, Write};
    use link::{Link, SerialTransport};
    use linkctl::CyclicState;
    use net::walk::{Emits, Responder, PORT_BLE, PORT_SWD, PORT_UART};
    use orchestrator::{control_task, cyclic_tx, input_task, InputSample, Obs, OrchestratorState};
    use panic_halt as _;
    // Linking-only: supplies the workspace `__INTERRUPTS` flash vector table (crates/vectors),
    // which cortex-m-rt no longer provides once its `device` feature is on.
    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::delay::Delay;
    use runtime_hal::descriptor::ClockPath;
    use runtime_hal::irq::{install, RamVectorTable, MAX_VECTORS};
    use runtime_hal::{
        detect_chip, FreeWatchdog, I2c, I2cMode, InputGroup, PeriphLabel, PolledSerial,
        RingBufferedRx, SplitSerial, Usart, WdgTimeout,
    };
    use scheduler::{systick_load, Scheduler};
    use store::{FmcFlash, Store, CONTROL_MODE, IMU_AXIS_SIGN, IMU_GYRO_BIAS, LINK_SET};
    use swd_mailbox::{EpochWatch, Mailbox, MailboxSerial, MAILBOX_BASE};
    use vectors as _;

    /// This firmware's L3 protocol/firmware version, reported in `NODE_HELLO`.
    const FW_VER: u16 = 0x0001;

    /// The production 72 MHz tree (IRC8M -> PLL): the inter-board baud divisor + flash wait states,
    /// and the sysclk every delay converts against (read as `CLOCK.sysclk_hz`, one owner).
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    /// The inter-board / link-listen UART baud, from its one owner (`link::INTER_BOARD_BAUD`).
    const LINK_BAUD: u32 = link::INTER_BOARD_BAUD;
    /// The CC2541 module's AT-command baud (`ble::at::BAUD`).
    const BT_BAUD: u32 = ble::at::BAUD;

    /// Fixed settle before the first `AT`: a freshly cold-power-cycled CC2541 is not UART-ready for the
    /// first few hundred ms, so the first probe would be lost (or land mid-byte). A `delay`-based wait,
    /// no RAM cost. Warm modules already answer by ~250 ms, so this only delays a cold boot.
    const BLE_COLD_BOOT_SETTLE_MS: u32 = 500;
    /// AT-probe attempts (each ~`STEP_MS` ≈ 248 ms: one `AT\r\n` + an RX-drain window). 16 ≈ a ~4 s
    /// patient window AFTER the settle (so ~4.5 s total) - a cold module's AT-ready time varies, and a
    /// fixed ~750 ms (3 tries) caught it only ~50%. The probe early-exits the instant `AT+OK` arrives,
    /// so a warm/fast module still costs ~one step; only a truly silent port spends the whole budget.
    const BLE_PROBE_ATTEMPTS: u32 = 16;
    /// Bytes of AT-probe RX captured into the SWD diagnostic block ([`BleProbeObs`]). Enough to show the
    /// 7-byte `AT+OK\r\n` plus context (garbage = baud, nothing = not-ready/wiring).
    const OBS_RX_CAP: usize = 64;

    /// Each L2 link's reassembly buffer (the largest packet a link reassembles). The links carry
    /// single-fragment L3/config PDUs (<= `net::walk::MAX_PDU` = 64 B); 72 B holds a whole PDU with a
    /// little margin while keeping the `Link`s small for the 8 KiB-image RAM budget (the floor is
    /// `MAX_PDU` = 64).
    const PACKET: usize = 72;
    /// Per-link `StreamFramer` buffers (`frame_capacity + 4`, the `SerialTransport` rule: the
    /// SOF/len and CRC bytes around the carrier's largest frame). Sized to EACH carrier instead
    /// of one shared 132-byte max (the slice-7 stack budget: the shared constant wasted 144 B of
    /// live loop RAM across the UART/BLE links).
    const MAILBOX_FRAMER_N: usize = swd_mailbox::FRAME_CAPACITY + 4; // 132
    const UART_FRAMER_N: usize = UART_FRAME_CAP + 4; // 100
    const BLE_FRAMER_N: usize = BLE_FRAME_CAP + 4; // 20
    /// The DMA RX ring for the inter-board USART (circular DMA + USART IDLE). >= the max wire frame
    /// with margin for a back-to-back burst.
    const DMA_CAP: usize = 128;
    /// The inter-board UART's L2 frame capacity (frag-hdr + chunk); a whole 64 B PDU rides one
    /// frame. Right-sized 96 -> 72 (round-7 stack reclaim, audit-approved): the largest L2 frame
    /// this port ever carries is a MAX_PDU (64 B) + the 1-byte frag-hdr = 65 <= 71 usable, so 96
    /// bought nothing but 24 B of framer buffer + 24 B of send scratch on the deep drain chain.
    const UART_FRAME_CAP: usize = 72;
    /// The BLE link's L2 frame capacity. The CC2541 bridge is a byte stream (it coalesces/re-chunks),
    /// so this is just the framing chunk size, sized so one stream frame fits a ~20 B BLE ATT write
    /// (SOF + len + 16 B L2 frame + CRC).
    const BLE_FRAME_CAP: usize = 16;
    /// The poll window (`specs/l3.md`, "PROBE_PORTS is answered by an active local probe"): after a
    /// `PROBE_PORTS`, the responder waits this long for its per-port neighbour probes to answer, then
    /// emits `PORTS`. Expressed in **SysTick ticks** (250 Hz) - a wall-clock window that is the same
    /// on every board, NOT a main-loop-pass count: the loop rate varies ~6x between the F103 master
    /// and the F130 slave (and shifts as the image grows), so a pass count tuned to fire inside the
    /// controller's retransmit window on the fast board overshoots it on the slow one and `PORTS`
    /// never gets sent (the silicon-found deviation 2: the slave beaconed its port-probes for 30 s
    /// and never answered).
    ///
    /// Two bounds it must satisfy. It must be **longer** than the worst-case neighbour probe-reply
    /// round-trip so a real neighbour is seen (the BLE bridge is the slowest medium: a `NODE_HELLO`
    /// out and its reply back through the CC2541's data-mode bridge is tens of ms, ~100 ms worst
    /// case); and **shorter** than the controller's `PROBE_PORTS` retransmit (3 s, `swd-bridge`'s
    /// `run_walk`) so `PORTS` fires on the FIRST probe on every board regardless of loop speed.
    ///
    /// 125 ticks = 500 ms sits in that gap: >3x margin over the ~100 ms BLE round-trip (neighbours
    /// are never missed) and 6x under the 3 s retransmit (the walk always advances on the first
    /// probe). Snappier than the original ~1 s intent; a smaller 250 ms would still be safe but
    /// trims the BLE-neighbour margin. DEPENDS on the firmware SysTick tick being live (the slice-7
    /// P0 fix, `register_tick_handler`): before it, `TICK_COUNT` was stuck at 0 and a tick window
    /// would never elapse.
    const POLL_WINDOW_TICKS: u32 = 125;

    /// Per-pass, per-port link drain budget (`specs/integration.md`, "Bounded link drain"): each
    /// loop pass drains at most this many whole packets from each link, instead of an unbounded
    /// `while let Some(..) = poll_recv(..)`. Bounds every pass's servicing cost so the peer's
    /// 250 Hz `CYCLIC_STATE` flood cannot collapse the slower F130 loop into the flood/drain
    /// feedback the bench found (2026-07-17: an unbounded drain starved the slave loop to ~6-15 Hz,
    /// degenerating its 4:1 table timing and flickering the master's `comms_loss`). 8 leaves
    /// generous headroom over any legitimate multi-frame burst (a fragmented packet reassembles
    /// inside one `poll_recv`, so one unit is one whole packet) while keeping the worst-case
    /// per-pass cost small; undrained bytes wait in the DMA ring + `StreamFramer` for the next pass.
    const DRAIN_BUDGET: usize = 8;

    /// `LINK_SET` bit for a live port (a per-port bitmask; the mailbox port 0 is always present and is
    /// not part of the discovered set, so only the USART link ports 1.. are recorded).
    const fn link_bit(port: u8) -> u8 {
        1u8 << port
    }
    /// Port indices (fixed slots): 0 = SWD mailbox, 1 = inter-board UART, 2 = BLE module.
    const PORT_IDX_MAILBOX: u8 = 0;
    const PORT_IDX_UART: u8 = 1;
    const PORT_IDX_BLE: u8 = 2;
    /// The board's fixed port count (mailbox + the two USART link slots; an absent BLE slot classifies
    /// `empty`). The port assignment indexes its answers by the same slot numbering, so the two
    /// counts are one fact and are held to it here.
    const N_PORTS: u8 = 3;
    const _: () = assert!(N_PORTS as usize == board::plumbing::NET_PORTS);

    /// VTOR alignment invariant: the `RAM_VECTORS` static (packed by memory.x's `.ramtables`
    /// section) carries `RamVectorTable`'s own alignment, so as long as the type stays `align(512)`
    /// the table is VTOR-valid (a runtime guard at the call site double-checks the placed address).
    const _: () = assert!(core::mem::align_of::<RamVectorTable>() >= 512);

    /// One entry in the safe-link USART allowlist (`specs/l3.md`, "Pin safety": gate-capable pins
    /// denied). Pins and slots ONLY: which peripheral a pair drives, and whether this silicon can
    /// drive it at all, are the HAL pin model's answers ([`usart_of`]), asked per boot. Baking
    /// either here would be a second copy of `runtime_hal::pincap`'s routing table, and a wrong one
    /// on half the fleet: the same PB6/PB7 pair is USART0 on the F1x0 and no USART at all on the
    /// F10x.
    struct SafeLinkUsart {
        /// This entry's bit in the persisted `LINK_SET` mask: the identity of the WIRING. Unique
        /// per entry, and what the reserved-set computation keys its freeing rule on.
        link_set_bit: u8,
        /// The `net` port slot this entry's link occupies when live. NOT unique: the two BLE
        /// wirings below are alternatives for one board function and share the BLE slot.
        net_port: u8,
        /// The entry's pin pair, packed `(port << 4) | pin`: the allowlist's PIN fact, the
        /// single declaration the board validator's reserved set is computed from
        /// (`specs/board-model.md` check 3) and the bring-up resolves its handles from.
        pins: [u8; 2],
    }

    /// The spec's safe-UART allowlist (`specs/l3.md` §143): USART0 PB6/PB7, USART1 PA2/PA3, USART2
    /// PB10/PB11. Gate-capable pins (USART0-default PA9/PA10) are denied - that is the allowlist's
    /// whole job.
    ///
    /// The first and last entries are the fleet's TWO BLE WIRINGS. The onboard CC2541 hangs off
    /// USART2's PB10/PB11 on the standard family and off USART0's PB6/PB7 on the classywalk
    /// offroad family, and the mirror image is true of the IMU: the pins one board uses for its
    /// module are the pins the other uses for its I2C. Both wirings are therefore allowlisted with
    /// distinct `LINK_SET` bits and one shared `net` slot, and which one a board brings up is
    /// decided by what was staged on it (`resolve_ports`), never by the chip. It cannot be by the
    /// chip: the bench slave and the offroad boards are the same GD32F130 part.
    const SAFE_LINK_USARTS: [SafeLinkUsart; 3] = [
        // PB6/PB7 = USART0 on the F1x0 (Datasheet Rev3.7 Table 2-10, AF0): the offroad family's
        // BLE module. The same pins are the standard family's IMU bus, and on the F10x they reach
        // no USART at all (that would need the AFIO remap the HAL does not implement).
        SafeLinkUsart {
            link_set_bit: 3,
            net_port: PORT_IDX_BLE,
            pins: [0x16, 0x17], // PB6/PB7
        },
        // USART1 (PA2/PA3): the inter-board link (both boards, both families), the link-listen port.
        SafeLinkUsart {
            link_set_bit: PORT_IDX_UART,
            net_port: PORT_IDX_UART,
            pins: [0x02, 0x03], // PA2/PA3
        },
        // PB10/PB11 = USART2 on the F10x: the standard family's onboard CC2541. The F1x0 has no
        // USART2 at all and puts I2C1 on these pins, which is the offroad family's IMU bus.
        SafeLinkUsart {
            link_set_bit: PORT_IDX_BLE,
            net_port: PORT_IDX_BLE,
            pins: [0x1A, 0x1B], // PB10/PB11
        },
    ];

    /// [`SAFE_LINK_USARTS`]'s inter-board-link entry, and the instance its pins drive.
    ///
    /// This slot is the one that does NOT need resolving. PA2/PA3 is USART1's default mapping on
    /// BOTH families (the pin model's `USART_PIN_MAPPINGS` carries exactly one row for it, routable
    /// everywhere), so unlike the BLE slot there is no per-board choice to make and no per-family
    /// answer to look up. Three things keep the named instance honest rather than a stale copy:
    /// `Usart::new` re-derives the pair through `pincap::usart_pins` on every boot and refuses a
    /// mismatch (`SelectorAddrMismatch`), so a drift fails loud at the seam on the first boot; the
    /// R-CAP agreement suite asserts the pair routes on every fleet part at host-test time; and the
    /// const assert below pins the index to the right slot.
    ///
    /// It is also a deliberate flash decision (`specs/decision-flash-budget.md`). Routing this slot
    /// through the per-boot resolver as well costs **816 B**, because the inter-board USART's
    /// instance then stops being a constant and `RingBufferedRx::new`'s DMA-channel selection can
    /// no longer fold. That is most of a 1 KiB budget spent to re-derive a fact with one fleet-wide
    /// answer that the HAL already re-checks. The BLE and IMU slots, which genuinely differ per
    /// board, are resolved; this one is declared and verified.
    const LINK_ENTRY_UART: usize = 1;
    const LINK_USART: PeriphLabel = PeriphLabel::Usart1;
    const _: () = assert!(SAFE_LINK_USARTS[LINK_ENTRY_UART].net_port == PORT_IDX_UART);

    /// The USART instance a safe-link entry's pins drive on THIS chip, if the HAL can bring that
    /// instance up: the one place the firmware asks the pin model "what is on these pins, and can
    /// I use it?".
    ///
    /// Two HAL answers, both required and neither substitutable for the other. `pincap::usart_pins`
    /// is the ROUTING half (does this pair map to a USART on this family, and through which AF -
    /// `USART0_TX` is AF1 at PA9 but AF0 at PB6, so only the mapping can say); `supports_rx` is the
    /// RECEIVE half (can the HAL actually take that instance's RX). `None` from either means the
    /// entry is not a candidate this boot, which is exactly how one allowlist serves both families
    /// without a family test written here.
    fn usart_of(chip: &runtime_hal::Chip, pins: [u8; 2]) -> Option<PeriphLabel> {
        let mapping = runtime_hal::pincap::usart_pins(chip, pins[0], pins[1])?;
        runtime_hal::supports_rx(chip, mapping.usart).then_some(mapping.usart)
    }

    /// The compiled allowlist with the HAL's per-boot answers filled in, the form both the
    /// validator's reserved set and the port assignment consume (their one shared input, so the
    /// pins the validator refuses to a board field and the pins a bring-up drives cannot disagree).
    fn allowlist(chip: &runtime_hal::Chip) -> [AllowlistPort; 3] {
        SAFE_LINK_USARTS.map(|u| AllowlistPort {
            link_set_bit: u.link_set_bit,
            net_port: u.net_port,
            pins: u.pins,
            routable: usart_of(chip, u.pins).is_some(),
        })
    }

    // ---------------------------------------------------------------------------------------------
    // The integrated control stack (specs/integration.md, slice 7): the scheduler static + SysTick
    // ISR (R1), the orchestrator shell static, the CTRL_OBS block, and the task callbacks.
    // ---------------------------------------------------------------------------------------------

    /// IWDG timeout, nominal (integration.md boot delta step 4: two orders above a loop pass;
    /// wdg-bench proved the R2 placement on silicon with this value; the stock interval stays
    /// unrecovered, so nominal stands).
    const WDG_TIMEOUT_MS: u32 = 500;
    /// The task table (integration.md): slot 0 = the 250 Hz control pipeline (reload 1), slot 1 =
    /// the 16 ms input task (reload 4).
    const CONTROL_RELOAD: u16 = 1;
    const INPUT_RELOAD: u16 = 4;
    /// IMU I2C rate: 400 kHz fast mode (the stock reference firmware's rate). At 100 kHz the
    /// 14-byte burst's wire time alone is ~1.7 ms of the 4 ms control budget (the 2026-07-18
    /// control-run cost defect); at 400 kHz it is ~0.45 ms. Clone-validated on the bench:
    /// probe/init/readback + sustained 251/s bursts, zero errors, at 400 kHz fast mode
    /// (specs/bench-evidence/2026-07-18/perf-decomp/08-imubench-400k-soak.log).
    const IMU_I2C_HZ: u32 = 400_000;
    /// Post-`Imu::init` settling pause before the first cyclic read (the caller-owned pause
    /// `specs/imu.md` names; imu-bench used the same 100 ms comfortably).
    const IMU_SETTLE_MS: u32 = 100;

    /// The RAM vector table (.bss, zero-init; `align(512)` for VTOR rides the type). A plain
    /// static so its initialization costs no stack (see the bring-up comment at its use).
    static mut RAM_VECTORS: RamVectorTable = RamVectorTable {
        slots: [0; MAX_VECTORS],
    };

    /// The inter-board USART's DMA RX ring (.bss; same pattern as [`RAM_VECTORS`]).
    static mut DMA_RING: [u8; DMA_CAP] = [0; DMA_CAP];

    /// The 250 Hz scheduler (the upgraded ISR-safe crate). The ONE ISR/thread crossing: the
    /// SysTick ISR calls `tick(&self)` concurrently with the loop's `dispatch(&self)`; the
    /// per-slot atomics are the arbitration (the R1 split). `&mut` access happens only at
    /// bring-up (registration, before the tick source is enabled), per the crate's own
    /// debug-asserted discipline.
    static mut SCHEDULER: Scheduler = Scheduler::new();

    /// SysTick tick count (OBS): ISR-incremented, main-thread read at publish; the atomic is the
    /// crossing.
    static TICK_COUNT: AtomicU32 = AtomicU32::new(0);
    /// Main-loop dispatch passes (OBS): loop-incremented, read at publish (same thread).
    static DISPATCH_COUNT: AtomicU32 = AtomicU32::new(0);
    /// Inter-board UART link recovered LINE-ERROR count (OBS): the `SplitSerial`-absorbed wire
    /// disturbances (a self-healed DMA `LineError`, i.e. an `ERRIE` overrun / framing / noise glitch)
    /// since boot, sampled from the link each loop pass and read at publish. This is the OQ1 split's
    /// clean signal: non-zero with `cyclic_age` staying fresh is the silicon proof the DMA-RX
    /// line-error self-heal path was actually exercised by an induced peer-reboot glitch, not that no
    /// glitch happened (guards the re-run against a false pass). Counted APART from the lap cause
    /// ([`LINK_LAP_OVERRUNS`]) so lap noise on a slow consumer cannot masquerade as a line-error hit.
    static LINK_LINE_ERRORS: AtomicU32 = AtomicU32::new(0);
    /// Inter-board UART link LAP-OVERRUN count (OBS): the `SplitSerial`-absorbed buffer-overrun losses
    /// (the DMA circular buffer lapped the read cursor: the consumer fell behind) since boot. The other
    /// half of the OQ1 split, kept SEPARATE from [`LINK_LINE_ERRORS`] so a wire glitch is
    /// distinguishable from a slow-consumer loss.
    static LINK_LAP_OVERRUNS: AtomicU32 = AtomicU32::new(0);
    /// Gate-1 UART-RX self-heal CONTROLLED-INJECTION hook (permanent test hook, `specs/integration.md`
    /// "Observation"; the stimulus option (c) for the section-5b Gate-1 sign-off). An operator writes a
    /// non-zero value over SWD (by the un-mangled symbol, like `CTRL_OBS`); the loop consumes it once
    /// (swap-to-0, one-shot) and injects ONE line error into the inter-board UART RX exactly as the
    /// ERRIE ISR would record it (`SplitSerial::inject_line_error`). It fabricates no DMA state and
    /// drives the SHIPPING self-heal path, so the next drain surfaces a `LineError`: `link_line_errors`
    /// increments, `link_lap_overruns` stays flat, and `cyclic_age` returns fresh with NO reset (the
    /// framer resyncs on the next frame). Poke it on the MASTER, whose RX carries the slave's traffic.
    /// Zero-initialised (`.bss`, not `.uninit`) so a stale value never self-injects across a reboot.
    #[no_mangle]
    static INJECT_UART_LINE_ERROR: AtomicU32 = AtomicU32::new(0);

    /// The 250 Hz tick body (R1 verbatim): advance the scheduler one tick, nothing else.
    /// Registered through the HAL's G7 tick seam (`runtime_hal::register_tick_handler`), NOT a
    /// `#[exception] SysTick`: `irq::install()` flips VTOR to the HAL-owned RAM vector table,
    /// whose SysTick slot routes to the HAL's `on_systick` -> this registered callback, so a
    /// flash-table exception symbol would be dead code (the slice-7 P0, silicon-found: the HAL
    /// tick counter advanced while the firmware's never moved). The HAL table is the single
    /// owner of the vector; this seam is the supported wiring. Lowest priority (0xF0, set at
    /// bring-up) so comms IRQs preempt it.
    extern "C" fn systick_tick_cb() {
        // SAFETY: shared access to the scheduler static; `tick(&self)` is the crate's ISR-safe
        // entry (atomics inside), sound against the thread-side `dispatch(&self)`.
        unsafe { (*addr_of!(SCHEDULER)).tick() };
        TICK_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    /// The plan-driven input pins (integration.md, "The input task"): a resolve-once
    /// `InputGroup` over up to three configured pins (button, pad A, pad B) plus the per-line
    /// configured mask (an absent field samples as its idle level). Line order: 0 = button
    /// (active-low), 1 = pad A, 2 = pad B (active-high).
    struct InputPins {
        group: Option<InputGroup>,
        has_button: bool,
        has_pad_a: bool,
        has_pad_b: bool,
    }

    /// The orchestrator shell: everything the task callbacks and the loop share, in ONE static
    /// (the spec's "Execution model": main-thread only; the loop and the dispatch callbacks run
    /// in the same context, so borrows never overlap as long as the loop drops its borrow before
    /// `dispatch()`, which the loop body does by scoping).
    struct Shell {
        orch: OrchestratorState,
        /// The plan-gated IMU bus + driver (`None` = not configured / probe failed: fail-soft).
        i2c: Option<I2c>,
        imu: Option<imu::Imu>,
        inputs: InputPins,
        /// Step 8's pending cyclic payload: the 250 Hz callback builds it (addressed boards
        /// only); the loop empties it port-directed onto the inter-board UART.
        cyclic_out: Option<CyclicState>,
        /// Sampled each loop pass from the responder (the address fact lives in `net`).
        addressed: bool,
        /// This boot's ordinal (the CTRL_OBS `.uninit` counter).
        boot_count: u32,
        /// The TICK_COUNT at the previous control run (the dt-honest attitude input; round-4
        /// defect B). Seeded so the first run computes dt = 1.
        last_control_tick: u32,
        /// The motor period counter at the previous control run: the period-liveness observation's
        /// baseline (`specs/motor-integration.md`, "Period liveness": the 250 Hz task expects
        /// `PERIODS` to advance by roughly 64 per tick).
        last_periods: u32,
        /// The period-liveness fault level's hysteresis state (`specs/motor-integration.md`, "the
        /// motor-side fault producers"). Main-thread only, like every other field here: the ISR
        /// publishes the raw counter and this task decides when a shortfall has lasted long enough
        /// to be a fault.
        period_health: motor::PeriodHealth,
    }

    /// The shell static. `None` until the boot path builds it (the state is not
    /// const-constructible); the SysTick interrupt does not touch it (only the scheduler), so
    /// initialization order only has to precede task dispatch, which it does (the tick source is
    /// enabled after).
    static mut SHELL: Option<Shell> = None;

    /// The `CTRL_OBS` RAM record (integration.md, "Observation"): the pipeline observation the
    /// bench reads over SWD (`nm <elf> | grep CTRL_OBS`). Single main-thread writer (the 250 Hz
    /// callback's whole-struct volatile publish); the ISR contributes only through
    /// [`TICK_COUNT`].
    #[repr(C)]
    struct CtrlObs {
        /// [`CTRL_OBS_MAGIC`] once live.
        magic: u32,
        /// Boot ordinal: survives resets in `.uninit` RAM (a cold power-up's garbage magic
        /// restarts it at 1), the IWDG-soak observable.
        boot_count: u32,
        /// SysTick ISR ticks.
        tick_count: u32,
        /// Main-loop dispatch passes.
        dispatch_count: u32,
        /// 250 Hz pipeline passes.
        control_ticks: u32,
        /// 16 ms input passes.
        input_ticks: u32,
        /// Latest attitude pitch, millidegrees.
        pitch_milli: i32,
        /// Ticks since the last accepted peer cyclic.
        cyclic_age: u32,
        /// Ticks since the last accepted drive command.
        drive_age: u32,
        /// INIT records through the enact seam.
        enact_inits: u32,
        /// SHUTDOWN records through the enact seam.
        enact_shutdowns: u32,
        /// The torque setpoint word (the sole-writer row's value).
        torque: i16,
        /// The mode byte.
        mode_byte: u8,
        /// Per-motor MOE bits.
        moe_bits: u8,
        /// The engagement sub-state byte.
        sub_state: u8,
        /// The active control mode (0 = Throttle, 1 = Balance).
        control_mode: u8,
        /// b0 imu_configured, b1 imu_live, b2 comms_loss, b3 mode_fault, b4 imu_loss (the IMU-loss
        /// fault: >= IMU_LOSS_THRESHOLD consecutive failed reads on a configured IMU, folded into
        /// fault_a; `specs/sensing-and-safety.md`, "IMU-loss supervision"), b5 motor_fault (the
        /// motor-side producers -- hall dwell, period-liveness loss, refused cal -- as folded into
        /// fault_a; `specs/motor-integration.md`). Bit 5 is a NEW bit in an existing byte, so
        /// every field offset is unchanged.
        flags: u8,
        /// The live gating-producer level mask (`orchestrator::events::EV_*`: b0 comms_loss,
        /// b1 stop_all, b2 imu_loss, b3 motor_fault, b4 latch A, b5 latch B, b6 mode_fault,
        /// b7 power_request), the level half of the O1 attribution instrument.
        ///
        /// This byte WAS the struct's reserved pad, so spending it moves no field offset: it is
        /// the one spare byte the layout already carried, and the eight producers it names are
        /// exactly the ones whose transition counts are appended at the end. Reading a level and
        /// its count in the same word is what distinguishes "held" from "blipped": an EVEN count
        /// with the level clear is a producer that went and came back between two bench reads
        /// (both edges count), and an ODD count means the level is currently held (or the read
        /// tore). Audit-corrected: this doc originally stated the parity inverted.
        event_levels: u8,
        /// Inter-board UART recovered LINE-ERROR count ([`LINK_LINE_ERRORS`]): `SplitSerial`-absorbed
        /// self-healed DMA-RX wire disturbances (`LineError`: ERRIE overrun / framing / noise) since
        /// boot. Appended (offset-preserving) so the SWD reader's existing field offsets are unchanged.
        /// The self-heal observable: non-zero with `cyclic_age` fresh proves the DMA-RX line-error
        /// recovery ran on an induced peer-reboot glitch. Counted APART from laps (the OQ1 split).
        link_line_errors: u32,
        /// Inter-board UART LAP-OVERRUN count ([`LINK_LAP_OVERRUNS`]): `SplitSerial`-absorbed
        /// buffer-overrun losses (the DMA lapped the read cursor: the consumer fell behind) since boot.
        /// Appended LAST (offset-preserving). The OQ1 split's other half: kept SEPARATE from
        /// `link_line_errors` so lap noise cannot masquerade as a line-error self-heal hit.
        link_lap_overruns: u32,
        /// --- Per-vector ISR entry counts (permanent; the round-6 defect-A observable) ---
        /// Each is a free-running ISR entry count (wrapping u32) from the HAL's `irq::*_ISR_METRIC`.
        /// A bench read takes deltas of these AND of `tick_count` over a host-timed window to get
        /// each vector's rate, so a per-byte storm (thousands/s on the USART1/DMA vectors) is
        /// distinguished from the expected IDLE/wrap rate (hundreds/s) without a throwaway probe
        /// build. Only entry counts are published: the round-7a bench characterisation found the
        /// NVIC-handler-mode CYCCNT deltas carry a ~19x per-entry inflation artifact, so in-ISR
        /// cycle attribution below pass scale is not a trustworthy observable. Appended after the
        /// OQ1 counters, offset-preserving.
        /// SysTick tick body (250 Hz nominal).
        systick_isr_entries: u32,
        /// USART1 RX vector (IDLE boundary + line errors under DMA RX): the inter-board link's port.
        usart1_isr_entries: u32,
        /// DMA-RX vector (circular-DMA half/full wrap servicing) for the inter-board link.
        dma_isr_entries: u32,
        /// Latest attitude roll, millidegrees. Appended LAST (offset-preserving: every prior field
        /// keeps its offset, so the SWD word map through `dma_isr_entries` is unchanged) at word 18
        /// / byte offset 72. Published off the same held Mahony output and `out_to_milli` scale as
        /// `pitch_milli`, so it carries the identical hold-not-zeros semantics; it is the hand-tilt
        /// roll-sign session's live readout (imu-tilt.py `ROLL_WORD`). The ZYX roll sign remains an
        /// open question (`specs/attitude.md`), so the tool reports the observed sign.
        roll_milli: i32,
        /// --- The motor block (`specs/motor-integration.md`, "Observation") ---
        /// Appended LAST, so every prior field keeps its offset (the offset-preserving append
        /// pattern the ISR-metric and roll blocks already used). The values come from the period
        /// ISR's handoff/observation atomics (the ISR is the sole writer of each); this record's
        /// sole writer is still the 250 Hz publish.
        ///
        /// No DWT cycle counters here: round 7a bounded `CYCCNT` as over-reading ~19x in
        /// NVIC-handler mode on this silicon, so period COUNTS are the truthful observable and CPU
        /// attribution comes from PC sampling.
        ///
        /// The free-running 16 kHz period count: the G-EOC observable (its delta over a host-timed
        /// window IS the period-ISR rate) and the liveness signal.
        motor_periods: u32,
        /// Packed motor state: `hall_code | enables << 8 | method << 16 | flags << 24`, flags per
        /// `motor::OBS_*` (b0 configured, b1 current-sense, b2 period-ISR-live, b3 coasting on the
        /// freshness guard, b4/b5/b6 the not-brought-up reasons). The period-ISR-live bit is ORed
        /// in by THIS publisher: it is the 250 Hz task's own observation, and the ISR stays the
        /// sole writer of the atomic.
        motor_state: u32,
        /// The last applied duties, packed `d0 | d1 << 16`.
        motor_duty01: u32,
        /// The last applied duty 2 plus the electrical angle, packed `d2 | angle << 16`.
        motor_duty2_angle: u32,
        /// The motor fault bits (`motor::FAULT_*`) in the low half; the invalid-hall dwell count,
        /// saturated to 16 bits, in the high half.
        motor_fault: u32,
        /// The rotor speed word: the signed edge count per 320-period window, raw.
        motor_speed: i32,
        /// The MEASURED quiet-bridge phase-current zero offsets, packed `offset_a | offset_b << 16`
        /// (`specs/motor-integration.md`, silicon stage 3's acceptance vehicle). Appended LAST, so
        /// every prior field keeps its offset; word 25 in the SWD map.
        ///
        /// Zero when no calibration ran (six-step or sine requested), or when the conversions
        /// never completed. Whether the pair was ACCEPTED is `motor_state`'s flag bit 7
        /// (`motor::OBS_CAL_ACCEPTED`); a refusal additionally shows as `motor::FAULT_INIT_CAL` in
        /// `motor_fault`. The measured pair is published either way, so an out-of-window board
        /// reports where it actually sits rather than only that it was refused.
        motor_cal: u32,
        /// --- The balance-era block (`specs/control.md`, "Observation") ---
        /// Appended LAST, so every prior field keeps its offset (the same offset-preserving append
        /// the ISR-metric, roll and motor blocks used). Word 26 in the SWD map.
        ///
        /// The engagement machine's gating/pickup row: the conditioned up-axis accel count
        /// (`control::gating`; +-4 g at 8192 counts per g, so the machine's `> 500` engage edge is
        /// 0.061 g of gravity on the deck's up-axis and its `< 0` pickup edge is an inverted
        /// deck). A level, right-way-up board must read LARGE and POSITIVE here; negative or
        /// near-zero at level rest is a wrong accel sign map, which is both un-engageable and
        /// sitting on the pickup edge. That check is the balance era's sign gate and it runs
        /// disarmed (`specs/silicon-queue.md`).
        gating_field: i16,
        /// The pre-envelope torque view: the reference the active mode arm fed the engagement
        /// machine this tick, before its gating and soft-start envelope. Packed into word 26's
        /// high half alongside `gating_field` (both are i16 by construction, so the pair costs one
        /// word: the `torque`/`mode_byte`/`moe_bits` packing precedent).
        ///
        /// It is NOT `torque` above: that is the machine's OUTPUT and reads zero whenever the
        /// machine is disengaged, which is every disarmed tick. This word is live regardless of
        /// sub-state, so a hand-tilted disarmed board shows what the controller would command.
        pre_env_torque: i16,
        /// The eight per-producer saturating transition counts (`event_levels`' bit order), the
        /// count half of the O1 attribution instrument: words 27-28, `counts[0..4]` then
        /// `counts[4..8]`, each byte little-endian within its word.
        ///
        /// Counted on every CHANGE of a producer's level, so a transient that asserts and releases
        /// inside one 4 ms tick still scores (2: the assert and the release) where a level read
        /// taken between bench samples would see nothing at all. That is what O1 needs: the arm
        /// session's unexplained SHUTDOWN/re-arm cycles left `enact_inits`/`enact_shutdowns`
        /// stepping with every level already clear by the time anyone looked.
        event_counts: [u8; orchestrator::N_EVENT_PRODUCERS],
    }

    /// `"CTRL"` little-endian.
    const CTRL_OBS_MAGIC: u32 = 0x4C52_5443;

    /// The block lives in `.uninit` (cortex-m-rt's NOLOAD section) so `boot_count` survives a
    /// reset; every field is written before the magic is trusted. Fixed un-mangled symbol for
    /// the SWD reader; raw-pointer access only (the BOARD_OBS discipline).
    #[no_mangle]
    #[link_section = ".uninit.CTRL_OBS"]
    static mut CTRL_OBS: MaybeUninit<CtrlObs> = MaybeUninit::uninit();

    /// Read the prior boot count out of the uninitialized block (garbage-magic = a cold power-up,
    /// restart at 1), called once at boot before anything publishes.
    fn next_boot_count() -> u32 {
        // SAFETY: raw-pointer volatile reads of the uninit block; any bit pattern is a valid
        // u32, and the magic gates whether the count is trusted.
        unsafe {
            let p = addr_of_mut!(CTRL_OBS) as *mut CtrlObs;
            let magic = addr_of!((*p).magic).read_volatile();
            if magic == CTRL_OBS_MAGIC {
                addr_of!((*p).boot_count).read_volatile().wrapping_add(1)
            } else {
                1
            }
        }
    }

    /// Publish one pipeline pass into `CTRL_OBS` (a whole-struct volatile write; the one writer).
    fn publish_obs(o: &Obs, boot_count: u32, period_live: bool) {
        let v = CtrlObs {
            magic: CTRL_OBS_MAGIC,
            boot_count,
            tick_count: TICK_COUNT.load(Ordering::Relaxed),
            dispatch_count: DISPATCH_COUNT.load(Ordering::Relaxed),
            control_ticks: o.control_ticks,
            input_ticks: o.input_ticks,
            pitch_milli: o.pitch_milli_deg,
            cyclic_age: o.cyclic_age,
            drive_age: o.drive_age,
            enact_inits: o.enact_inits,
            enact_shutdowns: o.enact_shutdowns,
            torque: o.torque_setpoint,
            mode_byte: o.mode_byte,
            moe_bits: o.moe_bits,
            sub_state: o.sub_state,
            control_mode: o.control_mode,
            flags: (o.imu_configured as u8)
                | ((o.imu_live as u8) << 1)
                | ((o.comms_loss as u8) << 2)
                | ((o.mode_fault as u8) << 3)
                | ((o.imu_loss as u8) << 4)
                | ((o.motor_fault as u8) << 5),
            event_levels: o.event_levels,
            link_line_errors: LINK_LINE_ERRORS.load(Ordering::Relaxed),
            link_lap_overruns: LINK_LAP_OVERRUNS.load(Ordering::Relaxed),
            systick_isr_entries: runtime_hal::irq::SYSTICK_ISR_METRIC.entries(),
            usart1_isr_entries: runtime_hal::irq::USART1_RX_ISR_METRIC.entries(),
            dma_isr_entries: runtime_hal::irq::DMA_RX_ISR_METRIC.entries(),
            roll_milli: o.roll_milli_deg,
            motor_periods: motor::PERIODS.load(Ordering::Relaxed),
            motor_state: motor::OBS_STATE.load(Ordering::Relaxed)
                | if period_live {
                    motor::OBS_PERIOD_LIVE << 24
                } else {
                    0
                },
            motor_duty01: motor::OBS_DUTY01.load(Ordering::Relaxed),
            motor_duty2_angle: motor::OBS_DUTY2_ANGLE.load(Ordering::Relaxed),
            motor_fault: motor::FAULT.load(Ordering::Relaxed)
                | (motor::INVALID_DWELL.load(Ordering::Relaxed).min(0xFFFF) << 16),
            motor_speed: motor::SPEED.load(Ordering::Relaxed),
            motor_cal: motor::OBS_CAL.load(Ordering::Relaxed),
            gating_field: o.gating_field,
            pre_env_torque: o.pre_env_torque,
            event_counts: o.event_counts,
        };
        // SAFETY: the one writer (main thread), fixed symbol, volatile so the SWD reader sees
        // coherent-enough snapshots (a torn read across fields is acceptable diagnostics).
        unsafe { (addr_of_mut!(CTRL_OBS) as *mut CtrlObs).write_volatile(v) };
    }

    /// The 250 Hz control task (scheduler slot 0): sample the IMU (the firmware-side sampling
    /// wrapper; a failed read is `None` -> the pipeline holds the filter + `imu_live` false, and
    /// the loss counter feeds `fault_a` once the miss stream crosses the threshold), gated by the
    /// retry breaker (`imu_read_due`), run the pure pipeline, build the cyclic payload (step 8,
    /// addressed boards only; the loop sends it), publish OBS.
    fn control_task_cb() {
        // SAFETY: dispatch-callback context == the main thread; the loop's own shell borrows are
        // scoped to end before `dispatch()` (the execution-model discipline).
        let Some(shell) = (unsafe { (*addr_of_mut!(SHELL)).as_mut() }) else {
            return;
        };
        // dt-honest attitude (round-4 defect B): the ticks that ACTUALLY elapsed since the last
        // control run, off the same TICK_COUNT the scheduler accrues from. First run: 1.
        let now_tick = TICK_COUNT.load(Ordering::Relaxed);
        // The IMU-loss retry breaker (specs/integration.md pipeline step 1): once the breaker is
        // open (>= IMU_LOSS_THRESHOLD consecutive failed reads) attempt the blocking read only on
        // the probe cadence, so a stuck-bus read (~10-28 ms of polled I2C) is not burned every
        // 4 ms tick. A skipped read hands the pipeline `None` (still lost); a probe-tick success
        // clears the streak and closes the breaker.
        let sample = if shell.orch.imu_read_due(now_tick) {
            match (&mut shell.i2c, &mut shell.imu) {
                (Some(bus), Some(dev)) => dev.read(bus).ok(),
                _ => None,
            }
        } else {
            None
        };
        let dt_ticks = now_tick.wrapping_sub(shell.last_control_tick).max(1);
        shell.last_control_tick = now_tick;
        // The period-liveness observation and the motor-side fault level, folded BEFORE the pass
        // that consumes them: `fault_a` is assembled inside `control_task`, so a level written
        // after it would act a tick late. How far the 16 kHz ISR advanced per elapsed tick, then
        // the hysteresis latch, then the level the mode machine sees.
        let periods = motor::PERIODS.load(Ordering::Relaxed);
        let per_tick = periods.wrapping_sub(shell.last_periods) / dt_ticks;
        shell.last_periods = periods;
        let period_live = motor::periods_live(per_tick);
        let motor_configured = motor::obs_configured(motor::OBS_STATE.load(Ordering::Relaxed));
        // The liveness supervisor's premise is a RUNNING counter, not merely a configured motor:
        // the shutdown sequence stops the counter deliberately, and that silence is not a wedged
        // vector (`specs/motor-integration.md`, slice 5).
        let motor_running = motor_configured && motor::COUNTER_RUNNING.load(Ordering::Relaxed);
        shell.period_health.update(motor_running, period_live);
        shell.orch.motor_fault = motor::motor_fault_level(
            motor_configured,
            motor::FAULT.load(Ordering::Relaxed),
            shell.period_health.loss(),
            arm::hw::refused(),
        );
        // The OFF-inhibit producer, live at slice 5: the period ISR's raw speed word says whether
        // the wheel is turning, and a turning wheel holds the machine in OFF.
        shell.orch.motor_moving = arm::off_inhibit_from_speed(motor::SPEED.load(Ordering::Relaxed));
        let out = control_task(&mut shell.orch, sample.as_ref(), dt_ticks);
        shell.cyclic_out = cyclic_tx(&shell.orch, shell.addressed);
        let obs = shell.orch.obs();
        // The 250 Hz -> 16 kHz handoff (`specs/motor-integration.md`): this task is the SOLE writer
        // of the demand word, and it writes the +-28500 stock-native torque word verbatim (no
        // rescaling anywhere, `specs/control.md`). The sequence bump is what lets the ISR's
        // freshness guard tell a fresh write from a repeated one, so a demand that happens to
        // repeat its value still counts as fresh.
        motor::DEMAND.store(obs.torque_setpoint as i32, Ordering::Relaxed);
        motor::DEMAND_SEQ.fetch_add(1, Ordering::Relaxed);
        // Step 6's enactment, at last acting (`specs/motor-integration.md`, "MOE enactment"): the
        // mode machine's per-motor allowance, enacted. AFTER the demand publish above, so a
        // shutdown's zeroed demand is the last word written this tick rather than one this same
        // tick overwrites; the arm path re-zeroes it for the same reason.
        arm::hw::enact(out.moe[0], motor_configured, shell.orch.motor_fault);
        publish_obs(&obs, shell.boot_count, period_live);
    }

    /// The 16 ms input task (scheduler slot 1): sample the configured input pins (button
    /// active-low, pads active-high; unconfigured lines sample idle) and run the pure input
    /// pass.
    fn input_task_cb() {
        // SAFETY: as `control_task_cb`.
        let Some(shell) = (unsafe { (*addr_of_mut!(SHELL)).as_mut() }) else {
            return;
        };
        let mut s = InputSample::default();
        if let Some(g) = shell.inputs.group {
            let code = g.read();
            s.button_asserted = shell.inputs.has_button && (code & 0b001) == 0;
            s.pad_a_high = shell.inputs.has_pad_a && (code & 0b010) != 0;
            s.pad_b_high = shell.inputs.has_pad_b && (code & 0b100) != 0;
        }
        input_task(&mut shell.orch, &s);
    }

    /// Build the orchestrator shell into its static (`specs/integration.md` boot delta step 2:
    /// the `ControlDispatch` boot seam rides the [`OrchestratorState`] constructor).
    ///
    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget fix): the Shell value
    /// (the orchestrator state is the image's biggest single object) is constructed here and
    /// written into the static, so `main`'s persistent frame never carries the temporary.
    #[inline(never)]
    fn init_shell(
        control_mode_byte: u8,
        imu_bus: Option<I2c>,
        imu_dev: Option<imu::Imu>,
        inputs: InputPins,
        boot_count: u32,
    ) {
        let imu_configured = imu_dev.is_some();
        // SAFETY: single-threaded boot (only the DMA RX ISR is live, and it reaches only the
        // HAL ring); the one initializing write, before any task dispatch exists.
        unsafe {
            *addr_of_mut!(SHELL) = Some(Shell {
                orch: OrchestratorState::new(
                    control_mode_byte,
                    imu_configured,
                    attitude::Config::default(),
                ),
                i2c: imu_bus,
                imu: imu_dev,
                inputs,
                cyclic_out: None,
                addressed: false,
                boot_count,
                last_control_tick: 0,
                last_periods: 0,
                period_health: motor::PeriodHealth::new(),
            });
        }
    }

    /// Bring the IMU up on the bus the staged layout put it on (the first `BoardPlan` consumer).
    ///
    /// Everything about WHERE the IMU is comes from the validated plan: the pin pair is the one the
    /// board's `imu.scl_pin` / `imu.sda_pin` fields name, and the I2C instance behind that pair is
    /// the pin model's answer for this chip. There is no compiled pin pair and no compiled instance
    /// here, because there is no fleet-wide answer to compile: the standard family's IMU is I2C0 on
    /// PB6/PB7 and the classywalk offroad family's is I2C1 on PB10/PB11, on the same GD32F130 part.
    ///
    /// The typed-pin seam costs nothing to make data-driven. `Chip::pin` hands back the same
    /// `Pin<Input<Floating>>` the named bags do, whichever byte it is asked for, so the two pairs
    /// are two VALUES through one `I2c::new`, not two instantiations of it.
    ///
    /// Fails soft in every refusal (`(None, None)`): the board boots link-only-plus-throttle and
    /// the outcome is observable (`imu_configured` in `CTRL_OBS`).
    ///
    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget idiom, as `init_shell` /
    /// `validate_layout` / `bring_up_ble`). Only the two live handles are returned; the staged gyro
    /// bias, the probe/init working set and `I2c::new`'s own frame are gone before the loop exists,
    /// instead of sitting in `main`'s frame, which is permanent because `main` never returns.
    #[inline(never)]
    fn bring_up_imu<F: store::Flash>(
        chip: &runtime_hal::Chip,
        store: &Store<F>,
        plan: Option<&board::BoardPlan>,
    ) -> (Option<I2c>, Option<imu::Imu>) {
        let ip = match plan.and_then(|p| p.imu) {
            Some(ip) => ip,
            None => return (None, None),
        };
        // The instance behind the staged pair, from the pin model that owns the pair -> instance
        // fact. The validator already accepted this pair as an I2C instance (its `NotI2cPair`
        // check), so this resolves for any plan that carries an IMU; asking the model again is
        // what makes the LABEL handed to `I2c::new` come from the same table as the bus index the
        // plan carries, rather than from a `0 => I2c0` match written out here.
        let Some(instance) =
            runtime_hal::pincap::i2c_instance(chip, ip.scl.packed(), ip.sda.packed())
        else {
            return (None, None);
        };
        // The staged pins as typed handles, with their port clock enabled.
        let (Ok(scl), Ok(sda)) = (chip.pin(ip.scl.packed()), chip.pin(ip.sda.packed())) else {
            return (None, None);
        };
        let Some(model) = imu::model_from_index(ip.model) else {
            return (None, None);
        };
        // Stage the per-board zero-rate gyro bias from the store (IMU_GYRO_BIAS x/y/z at indices
        // 0/1/2; default 0 = uncalibrated). The bench capture for the F130 clone was
        // [48, 13, -88] counts (2026-07-18 imu-bench bias phase).
        let bias = [
            store.get(IMU_GYRO_BIAS.at(0)),
            store.get(IMU_GYRO_BIAS.at(1)),
            store.get(IMU_GYRO_BIAS.at(2)),
        ];
        // Stage the per-board IMU SIGN MAP the same way (IMU_AXIS_SIGN, indices 0..5 =
        // [ax, ay, az, gx, gy, gz]). It is the rotation between the chip's axes and the board
        // frame: a mounting fact, per board, exactly like the bias above. It used to be the
        // compiled default alone, which is the STOCK board's mount and cannot also be right for a
        // differently-mounted one; both bench boards read the up-axis at -0.97 g while level and
        // right way up because of it (`specs/imu.md`, "Board-config fields").
        //
        // 0 = unset -> that index of the compiled reference map (`imu::Config::staged` owns the
        // rule), so an unconfigured board behaves exactly as before and a configured one
        // overrides per axis.
        let mut staged_sign = [0i32; 6];
        for (i, s) in staged_sign.iter_mut().enumerate() {
            *s = store.get(IMU_AXIS_SIGN.at(i as u8));
        }
        let Ok(mut bus) = I2c::new(
            chip,
            &CLOCK,
            instance,
            (scl, sda),
            I2cMode::fast(IMU_I2C_HZ, runtime_hal::i2c::FastDuty::Two),
        ) else {
            return (None, None);
        };
        let mut dev = imu::Imu::new(model, imu::Config::staged(staged_sign, bias));
        if dev.probe(&mut bus).is_ok() && dev.init(&mut bus).is_ok() {
            // The caller-owned post-init settle (specs/imu.md; the imu-bench pause) before the
            // first cyclic read.
            cortex_m::asm::delay((CLOCK.sysclk_hz / 1000) * IMU_SETTLE_MS);
            (Some(bus), Some(dev))
        } else {
            (None, None)
        }
    }

    /// Apply the persisted board layout at boot: validate it (specs/board-model.md checks 1-4),
    /// assert the power latch on the pin the store stages before anything is done with the rest of
    /// the verdict, and publish the outcome.
    ///
    /// The reserved set is the compiled allowlist minus the LINK_SET-freed ports plus SWD (the
    /// plumbing helper owns the freeing rule; the allowlist pin facts come from SAFE_LINK_USARTS,
    /// their single owner), the fields arrive through the registry defaults, and the chip
    /// capabilities through the HalCaps adapter. On Ok, the success record + the BoardPlan the
    /// integration bring-up consumes (the IMU group, the input pins); on Err, the failure record
    /// naming the offending field, and the boot proceeds link-only (the fail-loud contract's
    /// posture).
    ///
    /// **Why the latch comes off `Validated::self_hold` and not off the plan.** SELF_HOLD is this
    /// board's own power rail (see [`assert_self_hold`]), so it must go up even when the layout is
    /// rejected: a board whose motor group is mis-staged still boots link-only by design, and on
    /// battery it stays reachable to be corrected only if its own rail stays latched. The validator
    /// resolves it first and reports it either way, from the same staged fields and the same
    /// reserved set as everything else, so the pin the latch drives and the pin held against every
    /// other field cannot disagree.
    ///
    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget fix): the validator's
    /// working set (fields, reserved set, claims) lives here and is gone before the loop's deep
    /// ingest/append chain exists, instead of inflating `main`'s persistent frame.
    #[inline(never)]
    fn apply_layout<F: store::Flash>(
        chip: &runtime_hal::Chip,
        store: &Store<F>,
        allowlist: &[AllowlistPort],
        link_set: u8,
    ) -> Option<board::BoardPlan> {
        let reserved = reserved_set(allowlist, link_set);
        let v = board::validate(&read_fields(store), &HalCaps { chip }, reserved.as_slice());
        let self_hold = assert_self_hold(chip, v.self_hold);
        let (obs, plan) = match v.plan {
            Ok(plan) => (BoardObs::success(self_hold), Some(plan)),
            Err(e) => (BoardObs::failure(&e, self_hold), None),
        };
        // SAFETY: single-threaded boot, interrupts not yet enabled, the one writer this boot;
        // via a raw pointer so no reference to the `static mut` is formed (the BLE_PROBE_OBS
        // pattern).
        unsafe { *core::ptr::addr_of_mut!(BOARD_OBS) = obs };
        plan
    }

    /// Route one delivered-but-unhandled PDU (the `net` hand-back) into the orchestrator inbox:
    /// the reserved control block `0x10..0x2F` decodes through `linkctl`; everything else stays
    /// dropped (integration.md, "The delivered-PDU hand-back").
    fn route_handback(handed: Option<net::DeliveredPdu>) {
        let Some(d) = handed else { return };
        if !(0x10..=0x2F).contains(&d.opcode) {
            return;
        }
        let Some(payload) = linkctl::decode(d.opcode, &d.payload) else {
            return;
        };
        // SAFETY: main-thread context (the loop's drain), same discipline as the callbacks.
        if let Some(shell) = unsafe { (*addr_of_mut!(SHELL)).as_mut() } {
            shell.orch.inbox.accept(payload);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Board-layout validation (specs/board-model.md, slicing item 4): after the store mounts and
    // before any board-field bring-up, the persisted pin layout is read and validated against the
    // detected chip; the outcome lands in the SWD-readable BOARD_OBS block, and the resulting
    // BoardPlan feeds the integration bring-up (the IMU group + the input pins, its first
    // consumers; specs/integration.md boot delta).
    // ---------------------------------------------------------------------------------------------

    /// Assert this board's power-latch (SELF_HOLD) high, on the pin `board.self_hold` STAGES.
    /// Returns the packed pin actually driven, or `board::ABSENT` if none was.
    ///
    /// Role-agnostic on EVERY board: latch this board's own rail on so it stays up after the
    /// inter-board wake drops. A slave is woken over the cable and would otherwise fall back asleep
    /// once the master stops driving it; a master bridges its own power button. Unconditional,
    /// never gated on chip family (family != master/slave), because role is not yet known here:
    /// identity is positional, assigned later by the walk, and the latch cannot wait for it.
    /// RoboDurden does the same (`main.c:148`).
    ///
    /// **Which pin, and why from the store.** The pin is per-board wiring, so the staged field owns
    /// it: `board.self_hold`'s registry default (PB12, the fleet wiring) is what an unstaged board
    /// reads, so this drives PB12 there exactly as the compiled constant it replaces did. A board
    /// wired differently stages its own byte and this drives THAT pin. There is no compiled latch
    /// pin left to disagree with the field, and the pin held against the rest of the layout is the
    /// same one: the validator claims `Validated::self_hold` before it looks at any other field, so
    /// a second claimant on the latch pin is a named duplicate failure.
    ///
    /// **Ordering.** Called as early as the staged pin can be known: `board::validate` resolves the
    /// latch first and the caller drives it before it does anything with the rest of the verdict,
    /// all inside [`apply_layout`], which runs immediately after `Store::mount`. The latch is
    /// unavoidably post-mount (its pin is persisted data), so the window between power-on and the
    /// assert is the mount itself, which is inside the window the previous hardcoded assert already
    /// had 54 lines further down the boot. This shortens that window; it does not open one. What it
    /// deliberately does NOT do is assert a compiled guess first and correct it after: on a board
    /// wired to a different latch pin, the guess drives some other staged function push-pull for
    /// the length of the mount, which is a worse failure than the one it would be hedging against.
    ///
    /// **Loud on failure.** `None` here means the field is staged absent (this board declares no
    /// latch) or the staged byte named no usable pin (bad encoding, a pin absent on this silicon, a
    /// pin the link layer reserved). Either way nothing is driven, and in the second case
    /// `Validated::plan` carries the failure, so `BOARD_OBS` reports it against `board.self_hold`'s
    /// registry id with `BOARD_OBS.self_hold` reading `ABSENT`. On battery that board powers off;
    /// it does not silently hold a wrong pin high while reporting a clean layout.
    fn assert_self_hold(chip: &runtime_hal::Chip, pin: Option<board::Pin>) -> u8 {
        let Some(pin) = pin else {
            return board::ABSENT;
        };
        // The port clock comes up with the handle (`Chip::pin`), the same way the by-name getters
        // do it. A pin this silicon does not carry was already refused by the resolver.
        let Ok(handle) = chip.pin(pin.packed()) else {
            return board::ABSENT;
        };
        // Dropping the handle leaves the pin configured and driven: the latch has to outlive every
        // frame in this boot, and no later owner may take it (the validator's claim is what stops
        // one being assigned).
        let _ = handle.into_push_pull_output().set_high();
        pin.packed()
    }

    /// The real `board::Capabilities` implementation: a thin adapter over runtime-hal's R-CAP
    /// pin-capability queries (`runtime_hal::pincap`, its `specs/pin-capability.md`) - the
    /// store's `FmcFlash` pattern (a consumer-side trait impl over the HAL primitive; the
    /// capability answers come from the HAL model, never a table here). Packed pin bytes cross
    /// the seam; the named advanced timer maps to the trait's zero-based index (TIMER0 = 0,
    /// TIMER7/TIM8 = 1).
    struct HalCaps<'a> {
        chip: &'a runtime_hal::Chip,
    }

    impl board::Capabilities for HalCaps<'_> {
        fn pin_exists(&self, pin: board::Pin) -> bool {
            runtime_hal::pincap::pin_exists(self.chip, pin.packed())
        }
        fn gate_capable(&self, pin: board::Pin) -> bool {
            runtime_hal::pincap::gate_capable(self.chip, pin.packed())
        }
        fn gate_set(&self, hi: [board::Pin; 3], lo: [board::Pin; 3]) -> Option<u8> {
            runtime_hal::pincap::gate_set(self.chip, hi.map(|p| p.packed()), lo.map(|p| p.packed()))
                .map(|t| if t == PeriphLabel::Timer7 { 1 } else { 0 })
        }
        fn adc_channel(&self, pin: board::Pin) -> Option<u8> {
            runtime_hal::pincap::adc_channel(self.chip, pin.packed())
        }
        fn i2c_pair(&self, scl: board::Pin, sda: board::Pin) -> Option<u8> {
            runtime_hal::pincap::i2c_pair(self.chip, scl.packed(), sda.packed())
        }
    }

    /// The board-layout validator's SWD-readable outcome (`specs/board-model.md`,
    /// "Observability"): magic, result code, the offending field's registry id + index, the
    /// power-latch pin this boot actually drove, and the kind-specific detail word. Read it over
    /// SWD at the address of the `BOARD_OBS` symbol
    /// (`nm <elf> | grep BOARD_OBS`). A `static mut` with a fixed un-mangled symbol, written
    /// once per boot (before any interrupt is enabled) via a raw pointer, never through a
    /// reference: the `BLE_PROBE_OBS` pattern exactly.
    #[no_mangle]
    static mut BOARD_OBS: BoardObs = BoardObs {
        magic: 0,
        result: 0,
        field_id: 0,
        index: 0,
        self_hold: 0,
        detail: 0,
    };

    // ---------------------------------------------------------------------------------------------
    // Serials: the L2 links ride runtime-hal's embedded-io adapters (specs/firmware.md, "The link
    // serials"): SplitSerial<RingBufferedRx> for the inter-board UART, PolledSerial for the BLE
    // module. The one firmware-local wrapper is ObservedSerial (the probe RX tee, below).
    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Cold-boot BLE probe diagnostics: an SWD-readable RAM block recording the AT-probe outcome so the
    // evaluator can characterize a cold-power-cycle boot - `AT+OK` late vs never, garbage (= baud
    // mismatch), or no bytes at all (= not-ready / wiring) - and tune the probe window. Written once
    // per boot during phase 1, before any interrupt is enabled.
    // ---------------------------------------------------------------------------------------------

    /// SWD-readable AT-probe observation. Read it over SWD at the address of the `BLE_PROBE_OBS` symbol
    /// (`nm <elf> | grep BLE_PROBE_OBS`).
    #[repr(C)]
    struct BleProbeObs {
        /// `BleProbeObs::MAGIC` once a probe has written this block (confirms it is live, not stale RAM).
        magic: u32,
        /// AT attempts issued this boot (`== matched_attempt` on success, `== BLE_PROBE_ATTEMPTS` on a miss).
        attempts: u32,
        /// The 1-based attempt `AT+OK` arrived on (0 = never). Elapsed-to-`AT+OK` ≈ this × ~248 ms (`STEP_MS`).
        matched_attempt: u32,
        /// 1 = `AT+OK` seen (command mode), 0 = no AT (silent / not-ready / already in data mode).
        answered: u32,
        /// Total RX bytes seen across the whole probe (0 = no bytes at all -> not-ready or wiring).
        rx_total: u32,
        /// Bytes captured into `rx` (capped at `OBS_RX_CAP`).
        rx_len: u32,
        /// The first `OBS_RX_CAP` RX bytes (spot the 7-byte `AT+OK\r\n` vs garbage = baud mismatch).
        rx: [u8; OBS_RX_CAP],
        /// Deviation-1 observability: `1` if the BLE `Link` was built this boot (`Module::bring_up`
        /// reached transparent data mode), `0` if bring-up aborted after the probe (a dirty-POR
        /// module that answered the initial `AT` but then went AT-deaf through the 20-retry
        /// bring-up). Written once in `main` after phase-1 bring-up returns (appended after `rx`, so
        /// the existing field offsets are unchanged; this word sits at offset `24 + OBS_RX_CAP`).
        /// Lets the bench distinguish a *correctly* empty `PORTS` BLE port - module up, no L3 peer
        /// connected over the bridge (`specs/l3.md`: `PORTS` reports neighbour presence, not local
        /// link liveness) - from an aborted bring-up. On a board with no module the block's `magic`
        /// stays `0`, so this word is ignored with the rest.
        brought_up: u32,
        /// Bytes of the store's `DEVICE_NAME` handed to `AT+NAME=` this boot; `0` = **no rename was
        /// attempted** on this boot. Written only in the command-mode arm (immediately before
        /// [`ble::Module::bring_up`]), so it is the bench's evidence that the staged name actually
        /// reached the AT sequence rather than the module keeping whatever name it already had.
        /// Read it WITH `answered`: `answered = 1, name_len = n` is an n-byte name sent this boot;
        /// `answered = 0, name_len = 0` is the data-mode fallback arm, which by design never
        /// re-handshakes and so never renames; `answered = 1, name_len = 0` is a deliberately
        /// staged EMPTY name (a legal store value, sent verbatim). Appended after `brought_up`, so
        /// the existing field offsets are unchanged; this word sits at offset `28 + OBS_RX_CAP`.
        name_len: u32,
    }

    impl BleProbeObs {
        /// `"BLEP"` little-endian: the live marker.
        const MAGIC: u32 = 0x424C_4550;

        /// Start a fresh boot's probe record.
        fn begin(&mut self) {
            self.magic = Self::MAGIC;
            self.attempts = 0;
            self.matched_attempt = 0;
            self.answered = 0;
            self.rx_total = 0;
            self.rx_len = 0;
            self.brought_up = 0;
            self.name_len = 0;
        }

        /// Record one received byte (tee'd from the probe RX by [`ObservedSerial`]).
        fn push_rx(&mut self, b: u8) {
            self.rx_total = self.rx_total.wrapping_add(1);
            let i = self.rx_len as usize;
            if i < OBS_RX_CAP {
                self.rx[i] = b;
                self.rx_len += 1;
            }
        }
    }

    /// The SWD diagnostic block. A `static mut` (not a functional static) so it keeps the fixed,
    /// un-mangled symbol `BLE_PROBE_OBS` the evaluator reads over SWD; written only here, once per boot,
    /// before interrupts are enabled. Accessed via a raw pointer (never a reference to the `static mut`,
    /// per the `static_mut_refs` lint), so it is single-writer and sound.
    #[no_mangle]
    static mut BLE_PROBE_OBS: BleProbeObs = BleProbeObs {
        magic: 0,
        attempts: 0,
        matched_attempt: 0,
        answered: 0,
        rx_total: 0,
        rx_len: 0,
        rx: [0; OBS_RX_CAP],
        brought_up: 0,
        name_len: 0,
    };

    /// A serial wrapper that tees every received byte into a [`BleProbeObs`] while the AT-probe reads it,
    /// then hands back the inner serial ([`ObservedSerial::into_inner`]) so the resulting data-mode link
    /// does NOT keep teeing the live byte stream. The ONE firmware-local serial wrapper
    /// (specs/firmware.md, "The link serials"): it adapts firmware-owned diagnostics, not the wire.
    struct ObservedSerial<'a> {
        inner: PolledSerial,
        obs: &'a mut BleProbeObs,
    }

    impl<'a> ObservedSerial<'a> {
        fn new(inner: PolledSerial, obs: &'a mut BleProbeObs) -> Self {
            ObservedSerial { inner, obs }
        }
        fn into_inner(self) -> PolledSerial {
            self.inner
        }
    }

    impl ErrorType for ObservedSerial<'_> {
        type Error = core::convert::Infallible;
    }
    impl Read for ObservedSerial<'_> {
        fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
            let n = self.inner.read(out)?;
            for &b in &out[..n] {
                self.obs.push_rx(b);
            }
            Ok(n)
        }
    }
    impl ReadReady for ObservedSerial<'_> {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            self.inner.read_ready()
        }
    }
    impl Write for ObservedSerial<'_> {
        fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
            self.inner.write(data)
        }
        fn flush(&mut self) -> Result<(), Self::Error> {
            self.inner.flush()
        }
    }

    /// The cold-boot-robust AT probe: issue `AT` up to [`BLE_PROBE_ATTEMPTS`] times (each is one
    /// `ble::probe` attempt - `AT\r\n` + an `STEP_MS` RX-drain window), early-exiting on the first exact
    /// `AT+OK\r\n`. Patient enough to catch a cold-power-cycled module whose AT-ready time varies, instead
    /// of racing a fixed short window. Records the attempt count + matching attempt into `observed.obs`;
    /// the RX bytes are tee'd by [`ObservedSerial`].
    fn cold_boot_probe(observed: &mut ObservedSerial, delay: &mut Delay) -> bool {
        for attempt in 1..=BLE_PROBE_ATTEMPTS {
            observed.obs.attempts = attempt;
            if ble::probe(observed, delay, 1).unwrap_or(false) {
                observed.obs.matched_attempt = attempt;
                observed.obs.answered = 1;
                return true;
            }
        }
        false
    }

    /// Phase 1: the BT-probe (active, polled, 9600) on the USART the port assignment gave the BLE
    /// slot.
    ///
    /// WHICH USART that is comes from the assignment, not from here: on the standard family the
    /// module is USART2 on PB10/PB11, on the classywalk offroad family it is USART0 on PB6/PB7, and
    /// the caller has already applied the whole rule (`LINK_SET`, this silicon's routability, and
    /// the refusal to take a staged IMU's pins). `entry` is simply the port that won, so a `None`
    /// caller-side means the board has no BLE port this boot and this is not called at all.
    ///
    /// Configured boards bring up exactly the link-set and never probe outside it; unconfigured
    /// ones probe the routable BLE wiring and the module is whatever answers `AT+OK`.
    ///
    /// `name` is the advertised BLE name the caller read from the store ([`crate::ble_name`], whose
    /// docs hold the whole rule). It is a borrowed `&str` and stays borrowed only for this call:
    /// it goes into `AT+NAME=` and nothing built here keeps it.
    ///
    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget fix): the serial, the
    /// probe tee, and the AT bring-up's working set live here and are gone before the loop's deep
    /// chains exist.
    #[inline(never)]
    fn bring_up_ble(
        chip: &runtime_hal::Chip,
        delay: &mut Delay,
        entry: &SafeLinkUsart,
        configured: bool,
        name: &str,
    ) -> Option<BleLink> {
        let ble_usart = usart_of(chip, entry.pins)?;
        let (Ok(tx), Ok(rx)) = (chip.pin(entry.pins[0]), chip.pin(entry.pins[1])) else {
            return None;
        };
        let serial = PolledSerial::new(chip, &CLOCK, ble_usart, (tx, rx), BT_BAUD).ok()?;

        // Settle: a freshly cold-power-cycled CC2541 is not UART-ready for the first few hundred
        // ms, so the first `AT` would be lost or land mid-byte. A busy-wait (no RAM); ~500 ms only
        // delays a cold boot (a warm module already answers by ~250 ms).
        cortex_m::asm::delay((CLOCK.sysclk_hz / 1000) * BLE_COLD_BOOT_SETTLE_MS);

        // Tee the probe RX into the SWD diagnostic block. SAFETY: single-threaded boot, interrupts
        // not yet enabled, written only here; via a raw pointer, so no reference to the `static
        // mut` is formed.
        let obs = unsafe { &mut *core::ptr::addr_of_mut!(BLE_PROBE_OBS) };
        obs.begin();
        let mut observed = ObservedSerial::new(serial, obs);

        // Patient cold-boot AT probe (retry `AT` until `AT+OK` over a generous window, not a fixed
        // ~750 ms). A CONFIGURED board runs this on EVERY boot, not just the unconfigured
        // discovery: a cold power-cycle resets the CC2541 to command mode, so a BLE-kind port from
        // the link-set must be re-handshaked with the full AT bring-up (`SET=1`) or the module
        // never re-advertises and the board is invisible to the app (l3.md: "A BLE-kind port in
        // the link-set is still brought up with the full ble.md AT bring-up (SET=1) on every
        // boot"). `cold_boot_probe` only borrows the serial, so it stays usable for the data-mode
        // fallback below (`bring_up` would move + drop it on failure).
        let answered_at = cold_boot_probe(&mut observed, delay);
        let serial = observed.into_inner();

        if answered_at {
            // Command mode: full AT bring-up (NAME / intervals / SET=1 -> advertises / MODE=DATA).
            // Transparent data mode after; the link rides the gate type itself. The advertised
            // name is the staged one and only this arm sends it: record its length as the bench's
            // evidence that it reached the AT sequence. SAFETY: same single-threaded boot context,
            // same raw-pointer discipline as `begin()` above (no reference to the `static mut`).
            unsafe {
                (*core::ptr::addr_of_mut!(BLE_PROBE_OBS)).name_len = name.len() as u32;
            }
            ble::Module::new(name)
                .bring_up(serial, delay)
                .ok()
                .map(|pipe| Link::new(SerialTransport::new(pipe, BLE_FRAME_CAP)))
        } else if configured {
            // Data-mode fallback (l3.md): the link-set already identifies this port as the BLE
            // module, but it answered no `AT` even after the FULL patient probe -- a warm reset
            // left it in transparent data mode, still advertising. Register it as a live data-mode
            // link WITHOUT re-handshaking: its BLE identity is known from the link-set, so no AT
            // identification is needed. The patient probe is the prerequisite that makes this safe:
            // an `AT` miss now genuinely means data-mode, not a not-yet-ready cold boot (which the
            // old fixed ~750 ms window misread ~50% of the time, registering a SILENT module live).
            Some(Link::new(SerialTransport::new(
                ble::Pipe::assume_data_mode(serial),
                BLE_FRAME_CAP,
            )))
        } else {
            // Unconfigured + no AT+OK: not a module. On an unconfigured board that is the normal
            // answer for a wiring this board does not have, and it is why the probe is safe to run
            // at all: nothing but a CC2541 replies `AT+OK`.
            None
        }
    }

    // The three concrete L2 links (heterogeneous serials, one L2 code path each). Each link's
    // frame-scratch const (`F`) is its transport's framer capacity, NOT the 255 B protocol
    // ceiling: the scratch lives on the deep drain/send stack chains (round-7a reclaim, ~360 B
    // across poll_recv + send on the 8 KiB parts) and the carrier can never emit a larger frame.
    type MailboxLink =
        Link<SerialTransport<MailboxSerial, MAILBOX_FRAMER_N>, PACKET, MAILBOX_FRAMER_N>;
    type UartLink =
        Link<SerialTransport<SplitSerial<RingBufferedRx>, UART_FRAMER_N>, PACKET, UART_FRAMER_N>;
    // The BLE link rides the data-mode gate type (`ble::Pipe`, specs/ble.md): a link can only be
    // built on a serial KNOWN to be in transparent data mode (handshake arm: `bring_up`; fallback
    // arm: `Pipe::assume_data_mode` from the persisted link-set knowledge).
    type BleLink =
        Link<SerialTransport<ble::Pipe<PolledSerial>, BLE_FRAMER_N>, PACKET, BLE_FRAMER_N>;

    #[entry]
    fn main() -> ! {
        // Boot safe: nothing that could drive a motor is touched (no motor code).

        // The debug-halt backstop, FIRST and unconditionally (`specs/motor-integration.md`, the
        // halt posture): DBG_CTL0's TIMER0_HOLD freezes the advanced timer's counter while the core
        // is halted. It covers the halt nobody initiated -- a probe attach that halts, a hardware
        // breakpoint, a debugger reset -- where the alternative is the recorded FET failure: a
        // free-running counter re-applying the last duties indefinitely with no commutation
        // stepping them. It is not a silencing act on its own (a frozen counter holds the compare
        // outputs at their instantaneous levels), so it is the backstop half of a layered posture
        // whose deliberate half is disarm-then-halt tooling and whose standing rule is the bench
        // rule: never halt while energized.
        //
        // Before the motor bring-up starts the counter, and before anything could halt: the
        // register is power-reset-only, so a debugger setting it in-session is lost at the next
        // power cycle and the firmware must set it on every cold boot. One set-only RMW.
        runtime_hal::debug_hold_timer0();

        // Initialize the SWD mailbox header FIRST, before any bridge could attach. SAFETY: REGION_LEN
        // bytes at the fixed reserved base, owned only here, accessed only through the handle.
        let mailbox = unsafe { Mailbox::from_raw(MAILBOX_BASE as *mut u8) };
        mailbox.init_header();

        // Detect the silicon (fail loud: a wrong register layout is worse than a halt).
        let chip = detect_chip().unwrap();
        // `mcu` is the intrinsic chip-FAMILY tag the board reports in `NODE_HELLO` (NOT a role: family
        // != master/slave). Identity is positional - the walk assigns addresses by where a board sits,
        // never by reading a hardware id - so this tag is informational only.
        let mcu = match chip.clock() {
            ClockPath::F10xRcc => 1, // F10x family
            ClockPath::F1x0Rcu => 2, // F1x0 family
        };

        // Bring up the production 72 MHz tree before the store + UARTs (baud divisor, flash waits).
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            halt();
        }

        // Enable the DWT cycle counter now the clock tree is up, so `CYCCNT` free-runs for the
        // whole run and a bench operator attached over SWD can take pass-scale timing windows
        // against it (the round-7a characterisation: pass-scale/wall-anchored windows are the
        // trustworthy cycle observable; below-pass NVIC-handler-mode deltas are not, which is why
        // CTRL_OBS publishes only ISR entry counts). Cheap, idempotent.
        runtime_hal::irq::enable_cycle_counter();

        // Mount the store; read the persisted address + link-set (0/0 = a fresh, unconfigured board).
        let mut flash = FmcFlash::new(&chip);
        let mut store = Store::mount(&mut flash).unwrap();
        let link_set = store.get(LINK_SET);
        let configured = link_set != 0;

        // The safe-link allowlist with this chip's routability answers filled in: the one input the
        // layout validation and the port assignment share.
        let allowlist = allowlist(&chip);

        // Apply the persisted layout (a popped boot frame: see `apply_layout`). Its FIRST act is
        // the SELF_HOLD assert, on the pin the store stages: the board's own power rail goes up the
        // moment its pin is knowable, before the whole-layout validation that must not be able to
        // withhold it. On the bench both boards run on debugger 3V3 that bypasses the latch, so
        // that assert is a no-op for power there and is verifiable only as the pin's `GPIOx.OCTL`
        // bit; it matters on battery + the inter-board cable.
        let plan = apply_layout(&chip, &store, &allowlist, link_set);

        // Assign the board's pins to functions for this boot, ONCE, before anything is driven
        // (`board::plumbing::resolve_ports`, the single owner). It decides which USART carries the
        // BLE module from the staged `LINK_SET` and this silicon's routability, and it refuses any
        // link port whose pins the validated layout gave to the IMU. Both bring-ups below read
        // their answer from here, so they cannot disagree about who owns a pin.
        let assignment = resolve_ports(
            &allowlist,
            link_set,
            plan.as_ref()
                .and_then(|p| p.imu)
                .map(|ip| [ip.scl.packed(), ip.sda.packed()]),
        );

        // A SysTick busy-delay for the polled AT bring-up (phase 1, before any interrupt is enabled).
        // The application owns the one Peripherals::take() (runtime-hal DECISIONS #13: the HAL uses
        // raw register views internally and never consumes the one-shot flag, so ordering vs
        // detect_chip is unconstrained). take() after detect works; fail loud if somehow taken twice.
        let core = match cortex_m::Peripherals::take() {
            Some(p) => p,
            None => halt(),
        };
        let mut delay = Delay::new(core.SYST, CLOCK.sysclk_hz);

        // === Phase 1: the BT-probe (active, polled, 9600): see `bring_up_ble` (a popped boot
        // frame; the probe/bring-up working set never joins `main`'s persistent frame). ===
        // The assignment already applied `LINK_SET`, routability, and the IMU-pin refusal, so a
        // `None` here means this board has no BLE port this boot and nothing is driven.
        let ble_entry = assignment.port(PORT_IDX_BLE).map(|i| &SAFE_LINK_USARTS[i]);
        // The advertised name is staged data, not a compiled constant (`crate::ble_name`): a
        // flash-borrowed `&str` straight out of the mounted store, so it costs no RAM and no copy.
        // The borrow ends with this statement (nothing built here keeps it), leaving `store`
        // free for the mutable uses further down.
        let mut ble_link: Option<BleLink> = ble_entry.and_then(|e| {
            bring_up_ble(
                &chip,
                &mut delay,
                e,
                configured,
                ble_name::advertised(&store),
            )
        });
        // Deviation-1 observability: record whether the BLE Link came up (bring-up reached data
        // mode) so a bench read distinguishes a correctly-empty `PORTS` BLE port (module up, no L3
        // peer over the bridge) from an aborted bring-up (dirty POR). SAFETY: single writer, raw
        // pointer (no reference to the `static mut`), before any interrupt is enabled (`install()`
        // is below); on a no-module board `begin()` never ran so `magic` stays 0 and this is
        // ignored with the rest of the block.
        unsafe {
            (*core::ptr::addr_of_mut!(BLE_PROBE_OBS)).brought_up = ble_link.is_some() as u32;
        }

        // === Phase 2: link-listen (passive, DMA, link::INTER_BOARD_BAUD) on the inter-board USART (port 1) ===
        //
        // Always brought up (both boards, every boot): it is the proven inter-board link. Configured
        // boards still bring it up iff its port bit is set (it always is for a walked board).
        let want_uart = !configured || link_set & link_bit(PORT_IDX_UART) != 0;
        // The inter-board link: the one safe-link slot with a single fleet-wide answer, declared
        // rather than resolved (see `LINK_USART` for why, including what resolving it costs).
        // `Usart::new` re-derives the pair through the pin model and refuses a mismatch, so a
        // wrong declaration halts here on the first boot instead of driving the wrong peripheral.
        let gpioa = match chip.gpioa() {
            Ok(p) => p.split(),
            Err(_) => halt(),
        };

        // One bring-up, split into owned halves (specs/usart-split.md): the RX half is consumed by
        // RingBufferedRx below, the TX half drives polled TX. No second handle on a live base.
        let usart1 = match Usart::new(&chip, &CLOCK, LINK_USART, (gpioa.pa2, gpioa.pa3), LINK_BAUD)
        {
            Ok(u) => u,
            Err(_) => halt(),
        };
        let (usart1_tx, usart1_rx) = usart1.split();
        // The RAM vector table and the DMA ring are plain zero-initialized statics (.bss): the
        // earlier `cortex_m::singleton!` pattern materialized their init EXPRESSIONS (1 KiB +
        // 128 B) as temporaries in `main`'s frame before copying into the static, which is
        // exactly the stack the deep ingest/append chain needs (the slice-7 stack-budget fix).
        // A zero-init static costs no stack and no copy; the `&'static mut` is formed once,
        // here, before any interrupt exists. The `RamVectorTable`'s `align(512)` (VTOR) rides
        // the type.
        // SAFETY: the one formation of a &mut to this static, single-threaded boot.
        let vectors: &'static mut RamVectorTable = unsafe { &mut *addr_of_mut!(RAM_VECTORS) };
        // VTOR requires the table aligned to its power-of-two granule (`RamVectorTable` is
        // `align(512)`, which the static carries and memory.x's `.ramtables` packing preserves).
        // Guard it: a misplaced table is a silent boot brick (VTOR ignores the low bits), so
        // fail loud here instead.
        if !(vectors.slots.as_ptr() as usize).is_multiple_of(512) {
            halt();
        }
        // Route interrupts through the RAM vector table and enable them BEFORE arming DMA RX.
        // SAFETY (install): RAM init done, no peripheral IRQ enabled yet, `vectors` is a 'static table.
        unsafe { install(vectors, chip.irq()) };
        // The motor-era interrupt-priority ordering (HAL R7, `specs/motor-integration.md`'s
        // execution model): the period (ADC injected-EOC) vector above the USART1 RX vector above
        // the DMA-RX vector above SysTick at the stock 0xF0. Applied here, BEFORE any of those
        // vectors is unmasked (the DMA/USART RX bring-up is below, the period vector is the motor
        // bring-up's last step), and it is the single owner of the SysTick priority too, so this
        // replaces the firmware's own SCB poke. Writing priorities enables nothing.
        runtime_hal::irq::apply_motor_era_priorities(chip.irq());
        // SAFETY (enable): the table is installed; RingBufferedRx::new registers + unmasks its handlers.
        unsafe { cortex_m::interrupt::enable() };
        // SAFETY: as RAM_VECTORS above: the one &mut formation, before the DMA IRQ exists.
        let dma_buf: &'static mut [u8; DMA_CAP] = unsafe { &mut *addr_of_mut!(DMA_RING) };
        let rx_dma = match RingBufferedRx::new(&chip, usart1_rx, LINK_USART, dma_buf) {
            Ok(r) => r,
            Err(_) => halt(),
        };
        let mut uart_link: Option<UartLink> = if want_uart {
            Some(Link::new(SerialTransport::new(
                SplitSerial::new(usart1_tx, rx_dma),
                UART_FRAME_CAP,
            )))
        } else {
            None
        };

        // === The links into `net`: port 0 = mailbox (always), port 1 = UART, port 2 = BLE ===
        let mut mailbox_link: MailboxLink = Link::new(SerialTransport::new(
            MailboxSerial::firmware(mailbox),
            swd_mailbox::FRAME_CAPACITY,
        ));

        // The discovered link-set: the bitmask of live USART links, persisted at assign (below) and
        // read back as `LINK_SET` on every later boot. The bit recorded is the ENTRY's, not the
        // `net` slot's, which is what makes the mask remember WHICH WIRING was found: a BLE module
        // discovered on USART0/PB6-PB7 persists bit 3 and a module on USART2/PB10-PB11 persists
        // bit 2, so the next boot re-selects the same one and frees the other wiring's pins.
        let discovered = (if uart_link.is_some() {
            link_bit(SAFE_LINK_USARTS[LINK_ENTRY_UART].link_set_bit)
        } else {
            0
        }) | (match (ble_link.is_some(), ble_entry) {
            (true, Some(e)) => link_bit(e.link_set_bit),
            _ => 0,
        });

        let mut responder =
            Responder::new(N_PORTS, [PORT_SWD, PORT_UART, PORT_BLE, 0], mcu, FW_VER);
        responder.restore_addr(&store);

        // === The integration boot delta (specs/integration.md, after the existing bring-up) ===

        // 1. IMU bring-up on the bus the staged layout named (the first BoardPlan consumer): I2C0
        //    on PB6/PB7 for the standard family, I2C1 on PB10/PB11 for the classywalk offroad
        //    family, from this one image. The port assignment above already guaranteed no link
        //    port took those pins, so nothing can be driving them. Fails soft: the board boots
        //    link-only-plus-throttle and the outcome is observable (imu_configured in CTRL_OBS).
        let (imu_bus, imu_dev) = bring_up_imu(&chip, &store, plan.as_ref());

        // The plan-driven input pins (button + pads): resolve the configured ones into a
        // branch-free InputGroup; absent fields sample as idle through the per-line mask. Port C
        // (the fleet-default pad B, PC15) needs its clock enabled; A/B already are.
        let inputs = {
            // `match`, not `.map(..).unwrap_or(..)`: at opt-level "z" the closure survived as its
            // own out-of-line function once `main` stopped being one whole-program body (the
            // boot/loop split), costing ~96 B for three field reads.
            let (b, pa, pb) = match plan.as_ref() {
                Some(p) => (
                    p.button.map(|x| x.packed()),
                    p.pad_a.map(|x| x.packed()),
                    p.pad_b.map(|x| x.packed()),
                ),
                None => (None, None, None),
            };
            if [b, pa, pb].iter().flatten().any(|&x| (x >> 4) == 2) {
                let _ = chip.gpioc();
            }
            // `match` for the same reason as above (the `and_then` closure cost ~110 B).
            let group = match b.or(pa).or(pb) {
                Some(f) => chip
                    .input_group([b.unwrap_or(f), pa.unwrap_or(f), pb.unwrap_or(f)])
                    .ok(),
                None => None,
            };
            InputPins {
                group,
                has_button: b.is_some(),
                has_pad_a: pa.is_some(),
                has_pad_b: pb.is_some(),
            }
        };

        // 1b. Motor bring-up, plan-gated (`specs/motor-integration.md` slice 3, DISARMED): the
        //     timer + injected ADC are configured and the 16 kHz period ISR starts stepping the
        //     commutator, with MOE never written (this crate does not name the arming gate at all;
        //     arming is slice 5). A board with no motor group configured, or one this slice cannot
        //     drive, is left exactly as before: the outcome rides in `CTRL_OBS`'s motor block.
        //     Placed after the RAM vector table is installed (the period vector routes through it)
        //     and before the tick source, so the 250 Hz task never runs against a half-built motor.
        //     Slice 5: a successful bring-up hands its configured timer to `arm`, which derives
        //     the one arming gate in the image from it. A board that skips the bring-up never
        //     installs a gate and is therefore UNARMABLE, not merely unarmed.
        let motor_skip = match plan.as_ref().map(|p| &p.motors[0]) {
            Some(m) => match motor::bring_up(&chip, m, store.get(store::MOTOR_METHOD)) {
                Ok(summary) => {
                    arm::hw::install(&summary.timer);
                    None
                }
                Err(skip) => Some(skip),
            },
            None => Some(motor::MotorSkip::Absent),
        };
        if let Some(skip) = motor_skip {
            motor::record_skip(skip);
        }

        // 2. The control-dispatch boot seam rides inside the orchestrator constructor
        //    (CONTROL_MODE byte + the IMU fact: Balance demotes to Throttle with the mode
        //    fault). The shell static is built BEFORE the tick source exists, so no dispatch can
        //    see it half-made; the enabled DMA RX ISR never touches it. Built in a POPPED frame
        //    (`init_shell`): the ~700 B Shell value otherwise materializes in `main`'s
        //    persistent frame before the static write (the slice-7 stack-budget fix).
        init_shell(
            store.get(CONTROL_MODE),
            imu_bus,
            imu_dev,
            inputs,
            next_boot_count(),
        );

        // 3. The tick source (integration.md step-3 order: register the task table, mark the
        //    scheduler's tick-source latch, THEN enable SysTick). The bring-up Delay is done;
        //    free() returns the SYST it consumed.
        {
            // SAFETY: bring-up-time, thread-only registration BEFORE the tick source is enabled
            // (the scheduler crate's debug-asserted discipline; this &mut is exclusive: the
            // SysTick interrupt does not exist yet).
            let sched = unsafe { &mut *addr_of_mut!(SCHEDULER) };
            if sched.register(control_task_cb, CONTROL_RELOAD).is_err() {
                halt();
            }
            if sched.register(input_task_cb, INPUT_RELOAD).is_err() {
                halt();
            }
            // The SysTick tick callback, through the HAL's G7 seam (see `systick_tick_cb`:
            // VTOR points at the HAL RAM table, so this registration IS the SysTick wiring).
            // Ordered before mark_tick_source_enabled() and the SYST enable below; the
            // before-enable ordering is positional (not expressible as a compile-time
            // assertion with the current APIs; the pre-registration window is a safe no-op
            // in the HAL regardless).
            runtime_hal::register_tick_handler(systick_tick_cb);
            sched.mark_tick_source_enabled();
        }
        let mut syst = delay.free();
        let load = match systick_load(CLOCK.sysclk_hz) {
            Some(l) => l,
            None => halt(), // fatal config error per the recovered contract (24-bit LOAD)
        };
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(load);
        syst.clear_current();
        syst.enable_interrupt();
        syst.enable_counter();

        // 4. The watchdog, LAST (every halt() above dies un-armed, never reset-loops).
        //    freeze_on_debug_halt sets DBG_CTL0.FWDGT_HOLD (bit 8 @0xE004_2004, confirmed
        //    identical on GD32F10x and GD32F1x0 against the manuals) so a halted debugger does
        //    not take resets on the bench; the 500 ms timeout is the spec's nominal (the stock
        //    interval stays unrecovered).
        FreeWatchdog::freeze_on_debug_halt();
        let mut wdg = match FreeWatchdog::start(&chip, WdgTimeout::from_millis(WDG_TIMEOUT_MS)) {
            Ok(w) => w,
            Err(_) => halt(),
        };

        let mut epoch_watch = EpochWatch::new(mailbox);

        // Everything above this line is cold boot and stays in `.text`. Hand the built state to
        // the 250 Hz steady-state loop, which is PLACED in the F1x0's zero-wait window (see
        // `service_loop`).
        service_loop(
            &mut epoch_watch,
            &mut mailbox_link,
            &mut uart_link,
            &mut ble_link,
            &mut responder,
            &mut store,
            &mut wdg,
            discovered,
            configured,
        )
    }

    /// The cooperative service loop (the integration.md execution model): service the links,
    /// dispatch the due tasks, feed the watchdog AFTER dispatch (R2), emit the cyclic.
    /// Busy-spin, NEVER `wfi`.
    ///
    /// **Split out of `main` and PLACED in `.hotcode`** (`specs/motor-integration.md`, "Hot-path
    /// flash placement"). This is the 250 Hz steady-state path, and on the GD32F1x0 an instruction
    /// fetched above `0x0800_8000` costs ~8.8 cycles against 1 below it. Compiled into `main` the
    /// loop shared one function body with the whole cold bring-up (detect, clock, store mount,
    /// layout validation, BLE/UART/IMU/motor bring-up), 9,330 B against a window with 5,724 B
    /// free, so the loop could not be placed at all and the F1x0 dropped ~0.4% of its control
    /// passes. Split, only the loop half needs window space; the cold half keeps `.text`, where
    /// its once-per-boot fetches cost nothing.
    ///
    /// `#[inline(never)]` is load-bearing, not a hint: `#[link_section]` says where the emitted
    /// symbol lands and does nothing to stop LLVM inlining the body back into `main`, which would
    /// silently put the loop above the line again with the link still green.
    ///
    /// The state stays owned by `main`'s frame (which is permanent: `main` never returns) and is
    /// borrowed here, so the split copies nothing. It is not free: passing the state to a
    /// non-inlinable callee makes those locals address-taken, and the cold boot path loses the
    /// whole-function optimisation it used to get as one body, +992 B of flashed span measured
    /// raw (+720 B after the three claw-backs recorded in `specs/motor-integration.md`, "Hot-path
    /// flash placement"). Passing the state by VALUE instead was measured worse (+1,080 B).
    #[inline(never)]
    #[link_section = ".hotcode"]
    #[allow(clippy::too_many_arguments)]
    fn service_loop(
        epoch_watch: &mut EpochWatch,
        mailbox_link: &mut MailboxLink,
        uart_link: &mut Option<UartLink>,
        ble_link: &mut Option<BleLink>,
        responder: &mut Responder,
        store: &mut Store<FmcFlash>,
        wdg: &mut FreeWatchdog,
        discovered: u8,
        configured: bool,
    ) -> ! {
        // The tick captured when the current `PROBE_PORTS` started (rising edge of `probing()`);
        // `None` when no probe is in flight. The poll window is measured from it (deviation 2).
        let mut probe_start: Option<u32> = None;
        let mut link_set_saved = configured; // once assigned, persist LINK_SET once
        let mut rxbuf = [0u8; PACKET];
        let mut pdu = [0u8; net::walk::MAX_PDU];
        // ONE reusable emissions scratch for every drain site + the probe window (the slice-7
        // stack-budget fix: four per-site `Emits` locals cost ~300 B each in the loop's persistent
        // frame; exactly one is ever live, so one cleared-and-reused instance is the honest
        // shape).
        let mut emits = Emits::new();

        loop {
            // 1. Mailbox epoch handshake (the SWD bridge attaching): reset the framer, write epoch_ack.
            if epoch_watch.poll() {
                mailbox_link.transport_mut().reset();
                epoch_watch.ack();
            }

            // 2a. Drain the mailbox link (port 0), bounded to DRAIN_BUDGET packets this pass
            //     (integration.md, "Bounded link drain"). `poll_recv` borrows `rxbuf` (not the
            //     link) and the `.map(copy_pdu)` consumes that borrow, so the scrutinee is a plain
            //     length: the link is free in the body to ingest and route the emissions back
            //     across every link. Returning `false` when no packet is ready stops the drain
            //     early (an empty port costs one `poll_recv`, not the whole budget).
            bounded_drain(DRAIN_BUDGET, || {
                let Some(n) = mailbox_link
                    .poll_recv(&mut rxbuf)
                    .map(|f| copy_pdu(f, &mut pdu))
                else {
                    return false;
                };
                emits.clear();
                let handed = responder.ingest(PORT_IDX_MAILBOX, &pdu[..n], store, &mut emits);
                route_emits(&emits, mailbox_link, uart_link, ble_link);
                route_handback(handed);
                true
            });

            // 2a'. Gate-1 controlled-injection hook: if an operator poked INJECT_UART_LINE_ERROR over
            //      SWD, inject ONE line error into the inter-board RX exactly as the ERRIE ISR records
            //      it, then clear the flag (one-shot). Placed before the drain below so this pass's
            //      poll_recv surfaces the self-heal (link_line_errors++ / lap_overruns flat / cyclic_age
            //      fresh, no reset). Drives the shipping self-heal path, fabricating no DMA state.
            if INJECT_UART_LINE_ERROR.swap(0, Ordering::Relaxed) != 0 {
                if let Some(l) = uart_link.as_ref() {
                    l.transport().serial().inject_line_error();
                }
            }

            // 2b. Drain the inter-board UART link (port 1), if it came up. Bounded (integration.md,
            //     "Bounded link drain"): this is the port carrying the peer's 250 Hz CYCLIC_STATE
            //     flood, so an unbounded drain here is exactly what collapsed the F130 loop.
            bounded_drain(DRAIN_BUDGET, || {
                let Some(n) = uart_link
                    .as_mut()
                    .and_then(|l| l.poll_recv(&mut rxbuf))
                    .map(|f| copy_pdu(f, &mut pdu))
                else {
                    return false;
                };
                emits.clear();
                let handed = responder.ingest(PORT_IDX_UART, &pdu[..n], store, &mut emits);
                route_emits(&emits, mailbox_link, uart_link, ble_link);
                route_handback(handed);
                true
            });
            // Sample the inter-board link's two recovered-loss counters into the OBS crossings (the
            // `SplitSerial` absorbs each self-healed RX condition as a counter tick, classified by the
            // OQ1 split: a wire disturbance -> line_errors, a slow-consumer lap -> lap_overruns). Read
            // through the link -> transport -> serial, published by the 250 Hz callback.
            if let Some(l) = uart_link.as_ref() {
                let s = l.transport().serial();
                LINK_LINE_ERRORS.store(s.line_errors() as u32, Ordering::Relaxed);
                LINK_LAP_OVERRUNS.store(s.lap_overruns() as u32, Ordering::Relaxed);
            }

            // 2c. Drain the BLE link (port 2), if a module was brought up. Bounded as above.
            bounded_drain(DRAIN_BUDGET, || {
                let Some(n) = ble_link
                    .as_mut()
                    .and_then(|l| l.poll_recv(&mut rxbuf))
                    .map(|f| copy_pdu(f, &mut pdu))
                else {
                    return false;
                };
                emits.clear();
                let handed = responder.ingest(PORT_IDX_BLE, &pdu[..n], store, &mut emits);
                route_emits(&emits, mailbox_link, uart_link, ble_link);
                route_handback(handed);
                true
            });

            // 3. Probe window (deviation 2 fix): once probing, wait a fixed wall-clock window
            //    (POLL_WINDOW_TICKS, measured on the live SysTick TICK_COUNT) for the per-port
            //    neighbour probes to answer, then emit PORTS - independent of intervening inbound,
            //    so a retransmitted PROBE_PORTS no longer restarts the window and starve `PORTS`.
            //    The start tick is captured on the rising edge of probing() and carried across
            //    passes; the arithmetic is the host-tested `poll_window_elapsed`.
            let (fire, next_start) = poll_window_elapsed(
                responder.probing(),
                probe_start,
                TICK_COUNT.load(Ordering::Relaxed),
                POLL_WINDOW_TICKS,
            );
            probe_start = next_start;
            if fire {
                emits.clear();
                responder.poll_probe(&mut emits);
                route_emits(&emits, mailbox_link, uart_link, ble_link);
            }

            // 4. R4: sample the arm fact into the responder each pass (integration.md; the mode
            //    machine's any_moe_allowed IS the system's arm definition) and refresh the
            //    address fact for the cyclic gate. A scoped shell borrow: it MUST end before
            //    dispatch() below (the callbacks take their own).
            let armed = {
                // SAFETY: main-thread context; the borrow ends at the block's close.
                match unsafe { (*addr_of_mut!(SHELL)).as_mut() } {
                    Some(shell) => {
                        shell.addressed = responder.addr() != net::pdu::NO_ADDRESS;
                        shell.orch.mode.any_moe_allowed()
                    }
                    None => false,
                }
            };
            responder.set_armed(armed);

            // 5. Persist LINK_SET once, at assignment (specs/l3.md: "Once assigned it persists the
            //    set of ports that came up live") - DEFERRED while armed (integration.md R4: no
            //    flash program while armed; the persist-once latch waits for a disarmed pass).
            if !link_set_saved && !armed && responder.addr() != net::pdu::NO_ADDRESS {
                let _ = store.set(LINK_SET, discovered);
                link_set_saved = true;
            }

            // 6. Dispatch the due tasks (the 250 Hz pipeline + the 16 ms input task). Concurrent-
            //    safe against the SysTick tick per the scheduler's R1 split; no interrupt
            //    masking.
            // SAFETY: shared access; dispatch(&self) is the thread-side entry of the split.
            unsafe { (*addr_of!(SCHEDULER)).dispatch() };
            DISPATCH_COUNT.fetch_add(1, Ordering::Relaxed);

            // 7. The watchdog feed, AFTER dispatch, never inside the link servicing (R2, the
            //    wdg-bench silicon-proven placement).
            wdg.feed();

            // 8. Step 8's emission: the pending cyclic payload leaves PORT-DIRECTED on the
            //    inter-board UART (dst 0x00, the point-to-point rule; addressed boards only,
            //    which the builder already gated). Never routed: the 250 Hz stream cannot flood
            //    the BLE/mailbox ports (link-control.md, "Addressing and emission").
            let pending = {
                // SAFETY: main-thread context, after dispatch returned; scoped as above.
                unsafe { (*addr_of_mut!(SHELL)).as_mut() }.and_then(|s| s.cyclic_out.take())
            };
            if let (Some(c), Some(l)) = (pending, uart_link.as_mut()) {
                let mut payload = [0u8; CyclicState::LEN];
                let n = c.encode(&mut payload);
                // The PDU scratch is free here (the drains are done this pass): reuse it as the
                // frame buffer instead of a second 64 B local.
                if let Ok(p) = net::Pdu::new(
                    linkctl::OP_CYCLIC_STATE,
                    responder.addr(),
                    net::pdu::NO_ADDRESS,
                    &payload[..n],
                ) {
                    if let Ok(len) = p.encode(&mut pdu) {
                        let _ = l.send(&pdu[..len]);
                    }
                }
            }
        }
    }

    /// Copy a reassembled frame into the PDU scratch, returning the copied length (the source borrows
    /// `rxbuf`; the copy frees that borrow so the links can be re-borrowed for routing).
    fn copy_pdu(frame: &[u8], pdu: &mut [u8]) -> usize {
        let n = frame.len().min(pdu.len());
        pdu[..n].copy_from_slice(&frame[..n]);
        n
    }

    /// Route the Responder's emitted PDUs to the right L2 link by emit port (0 = mailbox, 1 = UART,
    /// 2 = BLE). Best-effort (L2 is best-effort; the controller retransmits the acknowledged plane). A
    /// port with no live link (an absent BLE module, or a not-brought-up UART) silently drops.
    fn route_emits(
        emits: &Emits,
        mailbox: &mut MailboxLink,
        uart: &mut Option<UartLink>,
        ble: &mut Option<BleLink>,
    ) {
        for e in emits {
            match e.port {
                PORT_IDX_MAILBOX => {
                    let _ = mailbox.send(&e.bytes);
                }
                PORT_IDX_UART => {
                    if let Some(l) = uart.as_mut() {
                        let _ = l.send(&e.bytes);
                    }
                }
                PORT_IDX_BLE => {
                    if let Some(l) = ble.as_mut() {
                        let _ = l.send(&e.bytes);
                    }
                }
                // No slot 3+ on this board: `net` slots are the board's PORTS, and the allowlist's
                // three entries share these three (both BLE wirings land on `PORT_IDX_BLE`).
                _ => {}
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

/// The advertised BLE name's one owner: it is the store's `DEVICE_NAME` (field id `0x10`),
/// verbatim.
///
/// The name reaches air as `AT+NAME=<name>` inside `ble::Module::bring_up`, so a per-board name is
/// STAGED CONFIG (writable over the wire with `CONFIG_WRITE 0x10`, readable back with
/// `CONFIG_READ 0x10`), never a compiled constant. That matters beyond taste: the fleet has two
/// masters, and a scanner can only tell them apart if the name is per-board data.
///
/// **The only fallback is the field's own registered default** (`"hoverboard"`), and it is the
/// store's rule rather than a firmware one: the store returns the registered default when the
/// record is absent, is the wrong type, or is not valid UTF-8. So an unstaged board advertises
/// `"hoverboard"`, and a `CONFIG_READ` of `0x10` returns the exact string a scanner sees. There is
/// no second, hidden name the firmware could substitute, which is the whole point of the change.
///
/// **An explicitly-stored empty string is a legal value and is sent verbatim** (`AT+NAME=\r\n`).
/// It is deliberately NOT read as "keep the module's current name" and deliberately does NOT
/// re-fall-back to the default: either reading would make `CONFIG_READ 0x10` disagree with what is
/// on air, which is the silent-hidden-name failure this owner exists to prevent. It stays loud
/// instead: `BLE_PROBE_OBS.answered = 1` with `name_len = 0` says an empty name was staged and
/// sent, distinct from `answered = 0, name_len = 0` (the data-mode fallback arm, no rename
/// attempted). No length cap is applied here: the module's own name-length limit is a CC2541 fact
/// the `ble` crate does not model yet, and a rejected `AT+NAME` leaves the previous name up.
///
/// Factored out of the target-only bring-up (the `probe_window` / `link_drain` pattern) so the host
/// test run reaches it against a real `Store`; `allow(dead_code)` because its one caller is the
/// target-only phase-1 bring-up, so the host non-test build (which CI clippys with
/// `--all-targets -D warnings`) sees it unused.
mod ble_name {
    /// This boot's advertised BLE name, borrowed from the mounted store's flash (no copy, no RAM).
    /// The borrow is the store's, so holding it blocks a concurrent `set`/`compact` at compile
    /// time; the bring-up call site drops it within the statement.
    ///
    /// **Why the dynamic `get_value` and not the typed `get_str(DEVICE_NAME)`** (which is otherwise
    /// the right door for a `StrField`, and which this was written with first): `get_value` is the
    /// path `CONFIG_READ` already takes, so it is ALREADY in the image, UTF-8 validator and all.
    /// `get_str` is a second STR read site, and LLVM answers it by outlining `core::str::from_utf8`
    /// into a shared 368 B function instead of the ~230 B specialised copy it had inlined into
    /// `Value::decode`. Measured on this tip (`cargo image` + `objcopy`, ELF-fresh): the typed
    /// expression costs **+264 B** of flashed span, this one **+40 B**, for byte-identical
    /// behaviour on every input (both fall back to the field's registered default for an absent,
    /// wrong-type, or non-UTF-8 record; the `ble_name` tests pin that behaviour, not the
    /// expression). 224 B is a quarter of the image's remaining headroom, and the ceiling has
    /// already been raised twice - so this reads the name through the same door the wire face uses.
    /// **Re-measure before "simplifying" this back to `get_str`.**
    #[allow(dead_code)]
    pub fn advertised<'a, F: store::Flash>(store: &'a store::Store<'_, F>) -> &'a str {
        match store.get_value(store::DEVICE_NAME.key()) {
            Ok(store::Value::Str(s)) => s,
            // Unreachable by construction: `DEVICE_NAME` is a registered `STR` field, so
            // `get_value` returns either a decoded `STR` record or that field's registered default
            // (also a `STR`), never `UnknownKey` and never another variant. Answered with the
            // field's OWN default rather than a literal, so even an impossible answer cannot put a
            // name on air that `CONFIG_READ 0x10` would not also report.
            _ => store::DEVICE_NAME.default(),
        }
    }

    #[cfg(test)]
    mod tests {
        use super::advertised;
        use base::error::FlashError;
        use ble::Module;
        use embedded_hal::delay::DelayNs;
        use embedded_io::{ErrorType, Read, ReadReady, Write};
        use store::{Flash, Store, DEVICE_NAME};

        const PAGE: usize = 1024;

        /// A minimal in-RAM [`Flash`] for these tests: a two-page region, erased to `0xFF`, with
        /// halfword-aligned write-once `program` (the silicon rules the store relies on). Store's
        /// own `MockFlash` is `#[cfg(test)]`-internal to that crate, so consumers bring their own
        /// (the `net` walk tests do the same).
        struct TestFlash {
            bytes: Vec<u8>,
        }

        impl TestFlash {
            fn erased() -> Self {
                TestFlash {
                    bytes: vec![0xFFu8; 2 * PAGE],
                }
            }
        }

        impl Flash for TestFlash {
            fn page_size(&self) -> usize {
                PAGE
            }
            fn as_bytes(&self) -> &[u8] {
                &self.bytes
            }
            fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
                let start = page * PAGE;
                let end = start + PAGE;
                if end > self.bytes.len() {
                    return Err(FlashError::OutOfBounds);
                }
                self.bytes[start..end].fill(0xFF);
                Ok(())
            }
            fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
                if !off.is_multiple_of(2) || !bytes.len().is_multiple_of(2) {
                    return Err(FlashError::Misaligned);
                }
                if off + bytes.len() > self.bytes.len() {
                    return Err(FlashError::OutOfBounds);
                }
                for (i, &b) in bytes.iter().enumerate() {
                    if self.bytes[off + i] != 0xFF && b != self.bytes[off + i] {
                        return Err(FlashError::ProgramFailed);
                    }
                }
                self.bytes[off..off + bytes.len()].copy_from_slice(bytes);
                Ok(())
            }
        }

        /// A stub CC2541: records every TX byte and acks each completed `...\r\n` command with the
        /// exact 7-byte `AT+OK\r\n`, which is all `Module::bring_up` waits on. Enough to read the
        /// name off the wire; the `ble` crate owns the full protocol tests.
        struct StubSerial {
            tx: Vec<u8>,
            rx: std::collections::VecDeque<u8>,
        }

        impl ErrorType for StubSerial {
            type Error = core::convert::Infallible;
        }

        impl Read for StubSerial {
            fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                let mut n = 0;
                while n < buf.len() {
                    match self.rx.pop_front() {
                        Some(b) => {
                            buf[n] = b;
                            n += 1;
                        }
                        None => break,
                    }
                }
                Ok(n)
            }
        }

        impl ReadReady for StubSerial {
            fn read_ready(&mut self) -> Result<bool, Self::Error> {
                Ok(!self.rx.is_empty())
            }
        }

        impl Write for StubSerial {
            fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                self.tx.extend_from_slice(buf);
                if self.tx.ends_with(b"\r\n") {
                    self.rx.extend(b"AT+OK\r\n");
                }
                Ok(buf.len())
            }
            fn flush(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }
        }

        struct NoDelay;
        impl DelayNs for NoDelay {
            fn delay_ns(&mut self, _ns: u32) {}
        }

        /// Run the AT bring-up with `name` and return the whole TX stream.
        fn tx_for(name: &str) -> Vec<u8> {
            let stub = StubSerial {
                tx: Vec::new(),
                rx: std::collections::VecDeque::new(),
            };
            let pipe = Module::new(name)
                .bring_up(stub, &mut NoDelay)
                .expect("the stub acks every command, so bring-up reaches data mode");
            pipe.into_inner().tx
        }

        #[test]
        fn an_unstaged_board_advertises_the_registered_default() {
            // Nothing written: the name is DEVICE_NAME's own registered default, which is also
            // exactly what a CONFIG_READ of 0x10 returns. No firmware-side constant is involved.
            let mut flash = TestFlash::erased();
            let store = Store::mount(&mut flash).unwrap();
            assert_eq!(advertised(&store), "hoverboard");
        }

        #[test]
        fn a_staged_name_is_what_gets_advertised() {
            let mut flash = TestFlash::erased();
            let mut store = Store::mount(&mut flash).unwrap();
            store.set_str(DEVICE_NAME, "hb-offroad-m").unwrap();
            assert_eq!(advertised(&store), "hb-offroad-m");
        }

        #[test]
        fn the_newest_staged_name_wins() {
            // The store is an append log: a re-stage (the bench renaming a board) must take, not
            // return the first record written.
            let mut flash = TestFlash::erased();
            let mut store = Store::mount(&mut flash).unwrap();
            store.set_str(DEVICE_NAME, "hb-bench-m").unwrap();
            store.set_str(DEVICE_NAME, "hb-offroad-m").unwrap();
            assert_eq!(advertised(&store), "hb-offroad-m");
        }

        #[test]
        fn an_explicitly_empty_name_is_carried_verbatim() {
            // The documented decision: empty is a legal staged value, NOT a signal to fall back to
            // the default (which would make CONFIG_READ 0x10 disagree with what is on air).
            let mut flash = TestFlash::erased();
            let mut store = Store::mount(&mut flash).unwrap();
            store.set_str(DEVICE_NAME, "").unwrap();
            assert_eq!(advertised(&store), "");
            // ...and it reaches the wire as an empty AT+NAME, which is what `name_len = 0` with
            // `answered = 1` reports on the bench.
            let tx = tx_for(advertised(&store));
            assert!(
                tx.windows(12).any(|w| w == b"AT+NAME=\r\nAT"),
                "an empty staged name is sent as a bare AT+NAME= line"
            );
        }

        #[test]
        fn the_at_sequence_carries_the_staged_name() {
            // The whole slice in one assertion: a name staged over the wire (CONFIG_WRITE 0x10 ->
            // set_str) is the name the module is told to advertise.
            let mut flash = TestFlash::erased();
            let mut store = Store::mount(&mut flash).unwrap();
            store.set_str(DEVICE_NAME, "hb-offroad-m").unwrap();

            let tx = tx_for(advertised(&store));
            let at = |needle: &[u8]| tx.windows(needle.len()).position(|w| w == needle);
            let name = at(b"AT+NAME=hb-offroad-m\r\n").expect("the staged name is on the wire");
            let set = at(b"AT+SET=1\r\n").expect("SET=1 present");
            assert!(
                name < set,
                "the name must be sent BEFORE SET=1 commits it (specs/ble.md ordering)"
            );
            assert!(
                at(b"hb-s6a").is_none(),
                "no compiled name may survive anywhere in the sequence"
            );
        }

        #[test]
        fn the_default_reaches_the_wire_on_an_unstaged_board() {
            let mut flash = TestFlash::erased();
            let store = Store::mount(&mut flash).unwrap();
            let tx = tx_for(advertised(&store));
            assert!(
                tx.windows(21).any(|w| w == b"AT+NAME=hoverboard\r\nA"),
                "an unstaged board advertises the registered default"
            );
        }
    }
}

/// The tick-based poll-window arbitration for the discovery walk's `PROBE_PORTS` handling
/// (deviation 2), factored out of the target-only service loop as a pure function so it is
/// host-testable. The real 250 Hz tick source is hardware, so only silicon exercises the actual
/// cadence; this covers the arithmetic (rising-edge capture, elapsed test, wrap). Compiled on all
/// targets (outside the `target_os = "none"` firmware module) so the workspace host test run
/// reaches its tests; `allow(dead_code)` because its one caller is the target-only loop, so the
/// host non-test build (which CI clippys with `--all-targets -D warnings`) sees it unused.
mod probe_window {
    /// Decide whether the poll window has elapsed and return the start tick to carry to the next
    /// loop pass. `(fire, next_start)`:
    /// - not probing -> `(false, None)`: no probe in flight, clear any captured start.
    /// - probing, no start captured -> `(false, Some(now))`: the rising edge, start the window.
    /// - probing, `now - start >= window` -> `(true, None)`: emit `PORTS`; clear so the next probe
    ///   re-arms (after `poll_probe`, `probing()` goes false anyway, but clearing keeps this pass
    ///   self-consistent).
    /// - probing, still within the window -> `(false, Some(start))`: keep waiting.
    ///
    /// Independent of intervening inbound by construction: a retransmitted `PROBE_PORTS` keeps
    /// `probing()` true (no rising edge), so `start` persists and the window still fires on
    /// schedule. `wrapping_sub` handles a `TICK_COUNT` wrap (~198 days at 250 Hz).
    #[allow(dead_code)]
    pub fn poll_window_elapsed(
        probing: bool,
        start: Option<u32>,
        now: u32,
        window: u32,
    ) -> (bool, Option<u32>) {
        if !probing {
            return (false, None);
        }
        match start {
            None => (false, Some(now)),
            Some(s) => {
                if now.wrapping_sub(s) >= window {
                    (true, None)
                } else {
                    (false, Some(s))
                }
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::poll_window_elapsed;

        #[test]
        fn not_probing_clears_the_start() {
            assert_eq!(poll_window_elapsed(false, Some(10), 20, 125), (false, None));
            assert_eq!(poll_window_elapsed(false, None, 20, 125), (false, None));
        }

        #[test]
        fn rising_edge_captures_now() {
            assert_eq!(poll_window_elapsed(true, None, 42, 125), (false, Some(42)));
        }

        #[test]
        fn waits_until_the_window_elapses_then_fires() {
            // Started at 100, window 125: still waiting at 224, fires at exactly 225 and beyond.
            assert_eq!(
                poll_window_elapsed(true, Some(100), 224, 125),
                (false, Some(100))
            );
            assert_eq!(poll_window_elapsed(true, Some(100), 225, 125), (true, None));
            assert_eq!(poll_window_elapsed(true, Some(100), 300, 125), (true, None));
        }

        #[test]
        fn retransmit_does_not_restart_the_window() {
            // A retransmitted PROBE_PORTS keeps probing() true (no rising edge), so the same start
            // tick is carried through and the window fires on its original schedule - the deviation
            // 2 fix: no inbound-driven reset can starve `PORTS`.
            assert_eq!(
                poll_window_elapsed(true, Some(100), 150, 125),
                (false, Some(100))
            );
            assert_eq!(poll_window_elapsed(true, Some(100), 260, 125), (true, None));
        }

        #[test]
        fn handles_tick_wrap() {
            // start near u32::MAX; now has wrapped past 0. elapsed = now.wrapping_sub(start) = 125.
            let start = u32::MAX - 10;
            let now = start.wrapping_add(125); // = 114
            assert_eq!(
                poll_window_elapsed(true, Some(start), now, 125),
                (true, None)
            );
            assert_eq!(
                poll_window_elapsed(true, Some(start), now.wrapping_sub(1), 125),
                (false, Some(start))
            );
        }
    }
}

/// The bounded per-pass link-drain policy (`specs/integration.md`, "Bounded link drain"), factored
/// out of the target-only service loop as a pure combinator so it is host-testable. The real drain
/// closure touches the target-only `Link`/`Responder`, but the budget arithmetic (drain at most
/// `budget` whole packets per pass, stop early when the port runs dry) is medium-agnostic and lives
/// here with its tests. Compiled on all targets (outside the `target_os = "none"` firmware module)
/// so the workspace host test run reaches its tests; `allow(dead_code)` because its callers are the
/// target-only loop, so the host non-test build (CI clippys with `--all-targets -D warnings`) sees
/// it unused.
mod link_drain {
    /// Drain a link at most `budget` times this pass. `step` performs one poll+ingest+route and
    /// returns `true` if it processed a packet, `false` when the port is drained (which stops the
    /// loop early). Returns the number of packets processed (<= `budget`), the bounded per-pass
    /// cost the fix guarantees.
    ///
    /// The bound is what breaks the flood/drain feedback (2026-07-17 silicon finding): capping the
    /// packets serviced per pass caps every pass's work (reassembly + ingest + routing), so the
    /// loop rate stays high on both fleet families and undrained bytes simply wait in the DMA ring
    /// and framer for the next pass. `CYCLIC_STATE` is latest-wins so a deferred stale cyclic frame
    /// is harmless; non-cyclic PDUs are deferred, never dropped.
    #[allow(dead_code)]
    pub fn bounded_drain<F: FnMut() -> bool>(budget: usize, mut step: F) -> usize {
        let mut processed = 0;
        while processed < budget {
            if !step() {
                break;
            }
            processed += 1;
        }
        processed
    }

    #[cfg(test)]
    mod tests {
        use super::bounded_drain;
        use embedded_io::{ErrorType, Read, ReadReady, Write};
        use link::{Link, SerialTransport};
        use net::Pdu;
        use std::collections::{BTreeSet, VecDeque};

        // --- The bounded-drain combinator in isolation ------------------------------------------

        #[test]
        fn stops_at_the_budget_when_the_port_stays_ready() {
            // A port that always has another packet: the drain processes EXACTLY `budget` and no
            // more (the per-pass bound), leaving the rest for the next pass.
            let mut calls = 0usize;
            let processed = bounded_drain(8, || {
                calls += 1;
                true
            });
            assert_eq!(processed, 8);
            assert_eq!(calls, 8); // never polled a 9th time
        }

        #[test]
        fn stops_early_when_the_port_drains() {
            // The port yields two packets then runs dry: the drain stops on the empty poll well
            // under budget (an idle port costs one poll, not the whole budget).
            let mut remaining = 2i32;
            let mut polls = 0usize;
            let processed = bounded_drain(8, || {
                polls += 1;
                remaining -= 1;
                remaining >= 0
            });
            assert_eq!(processed, 2);
            assert_eq!(polls, 3); // two packets + one empty poll that stopped it
        }

        // --- The policy over a real Link + StreamFramer -----------------------------------------

        /// An in-memory loopback serial: `write` appends to the buffer, `read` pops from its front.
        /// Feeding a `SerialTransport` over this exercises the real `StreamFramer` reassembly the
        /// fix must not break.
        #[derive(Default)]
        struct Loopback {
            buf: VecDeque<u8>,
        }
        impl ErrorType for Loopback {
            type Error = core::convert::Infallible;
        }
        impl Write for Loopback {
            fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
                self.buf.extend(data);
                Ok(data.len())
            }
            fn flush(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }
        }
        impl Read for Loopback {
            fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
                let mut n = 0;
                while n < out.len() {
                    match self.buf.pop_front() {
                        Some(b) => {
                            out[n] = b;
                            n += 1;
                        }
                        None => break,
                    }
                }
                Ok(n)
            }
        }
        impl ReadReady for Loopback {
            fn read_ready(&mut self) -> Result<bool, Self::Error> {
                Ok(!self.buf.is_empty())
            }
        }

        const OP_CYCLIC: u8 = 0x10; // linkctl::OP_CYCLIC_STATE (latest-wins)
        const OP_NONCYCLIC: u8 = 0x11; // linkctl::OP_DRIVE_CMD (must never be dropped)
        const FRAME_CAP: usize = 72; // the inter-board UART's UART_FRAME_CAP

        /// Encode one L3 PDU whose payload's first byte is `tag` (the value/id the drain reads back).
        fn send_pdu(link: &mut Link<SerialTransport<Loopback>>, opcode: u8, tag: u8) {
            let payload = [tag; 4];
            let pdu = Pdu::new(opcode, 0x02, 0x01, &payload).expect("pdu");
            let mut buf = [0u8; 64];
            let n = pdu.encode(&mut buf).expect("encode");
            link.send(&buf[..n]).expect("send");
        }

        #[test]
        fn saturated_rx_is_bounded_latest_cyclic_wins_noncyclic_all_delivered() {
            // A saturated RX: 40 CYCLIC_STATE frames (values 0..40, latest-wins) interleaved with
            // DRIVE_CMD frames (unique ids), far more than one pass's budget can drain.
            let mut link: Link<SerialTransport<Loopback>> =
                Link::new(SerialTransport::new(Loopback::default(), FRAME_CAP));

            const BURST: u8 = 40;
            const BUDGET: usize = 8;
            let mut expected_noncyclic = BTreeSet::new();
            for i in 0..BURST {
                send_pdu(&mut link, OP_CYCLIC, i);
                if i % 10 == 5 {
                    let id = 100 + i;
                    send_pdu(&mut link, OP_NONCYCLIC, id);
                    expected_noncyclic.insert(id);
                }
            }
            // The last frame on the wire is cyclic value BURST-1: the "latest" that must win.

            let mut latest_cyclic: Option<u8> = None;
            let mut got_noncyclic = BTreeSet::new();
            let mut max_per_pass = 0usize;
            let mut passes = 0usize;
            loop {
                let mut rx = [0u8; 64];
                let mut pdu = [0u8; 64];
                let processed = bounded_drain(BUDGET, || {
                    let Some(frame) = link.poll_recv(&mut rx) else {
                        return false;
                    };
                    let n = frame.len().min(pdu.len());
                    pdu[..n].copy_from_slice(&frame[..n]);
                    let p = Pdu::decode(&pdu[..n]).expect("decode");
                    if p.opcode == OP_CYCLIC {
                        latest_cyclic = Some(p.payload[0]);
                    } else {
                        got_noncyclic.insert(p.payload[0]);
                    }
                    true
                });
                // Every pass's work is bounded by the budget: the headline property.
                assert!(processed <= BUDGET, "pass processed {processed} > {BUDGET}");
                max_per_pass = max_per_pass.max(processed);
                passes += 1;
                if processed == 0 {
                    break;
                }
                assert!(passes < 100, "drain did not terminate");
            }

            // The burst exceeds one budget, so at least one pass ran to the cap (proves the bound
            // actually engaged, not that the port was trivially small).
            assert_eq!(max_per_pass, BUDGET, "the cap never engaged");
            // Latest-wins: the freshest CYCLIC_STATE is the one that survives the drain.
            assert_eq!(latest_cyclic, Some(BURST - 1));
            // Non-cyclic PDUs are deferred across passes but NONE is dropped.
            assert_eq!(got_noncyclic, expected_noncyclic);
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
