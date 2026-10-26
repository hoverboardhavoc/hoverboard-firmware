// SPDX-License-Identifier: GPL-3.0-or-later
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
mod ble_carrier;
mod motor;
mod odometer;
/// The host tests' in-RAM `store::Flash`, one copy for the whole crate (see the module).
#[cfg(test)]
mod test_flash;

#[cfg(target_os = "none")]
mod firmware {

    use crate::arm;
    use crate::ble_bringup::{self, BleProbeObs};
    use crate::ble_name;
    use crate::ble_wire::MeteredTx;
    use crate::guest_tx::{emit_samples, GuestSlots, GuestStaged};
    use crate::link_drain::bounded_drain;
    use crate::motor;
    use crate::odometer;
    use crate::probe_window::poll_window_elapsed;
    use board::plumbing::{read_fields, reserved_set, resolve_ports, AllowlistPort, BoardObs};
    use core::mem::MaybeUninit;
    use core::ptr::{addr_of, addr_of_mut};
    use core::sync::atomic::{AtomicU32, Ordering};
    use cortex_m::asm::nop;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m_rt::entry;
    use embedded_hal::digital::OutputPin;
    use link::{Link, SerialTransport};
    use linkctl::{ChipTag, CyclicState};
    use net::walk::{Emits, Responder, MCU_F10X, MCU_F1X0, PORT_BLE, PORT_SWD, PORT_UART};
    use orchestrator::{
        ble_cyclic_tx, ble_peer_tx, control_task, cyclic_tx, input_task, BoardIdentity,
        InputSample, Obs, OrchestratorState,
    };
    use panic_halt as _;
    // Linking-only: supplies the workspace `__INTERRUPTS` flash vector table (crates/vectors),
    // which cortex-m-rt no longer provides once its `device` feature is on.
    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::delay::Delay;
    use runtime_hal::descriptor::ClockPath;
    use runtime_hal::irq::{install, RamVectorTable, MAX_VECTORS};
    use runtime_hal::{
        detect_chip, DmaTx, FreeWatchdog, I2c, I2cMode, InputGroup, PeriphLabel, PolledSerial,
        RingBufferedRx, SplitSerial, Usart, WdgTimeout,
    };
    use scheduler::{systick_load, Scheduler};
    use store::{
        FmcFlash, Store, ATTITUDE_LEVEL_TRIM, BOARD_VBATT_CAL, CONTROL_BATTERY_FLOOR,
        CONTROL_DRIVE_TURN, CONTROL_FRAME, CONTROL_GAIN_MAX, CONTROL_MODE, CONTROL_POS_CLAMP,
        CONTROL_REFERENCE, CONTROL_RIDER_REQUIRED, CONTROL_SF_GAINS, CONTROL_SIDE, IMU_AXIS_ROLE,
        IMU_AXIS_SIGN, IMU_GYRO_BIAS, LINK_SET, NODE_CLASS,
    };
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
    /// The BLE port's one-deep emission slot, sized to the largest PDU the responder can hand
    /// `route_emits` (`net::walk::MAX_PDU`, the bound on an `Emission`'s bytes). The slot holds the
    /// PDU rather than its wire frames, because a PDU over the port's 15 B chunk becomes several
    /// frames and `MeteredTx` encodes them one at a time as the wire frees (`crate::ble_wire`).
    const BLE_EMIT_CAP: usize = net::walk::MAX_PDU; // 64
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
    /// The board's port SLOTS (mailbox + the two USART link slots). The port assignment indexes its
    /// answers by the same slot numbering, so the two counts are one fact and are held to it here.
    ///
    /// How many of those slots are REGISTERED with `net` is a per-boot question, not this one: the
    /// BLE slot exists only when its link does ([`ble_bringup::net_ports`]).
    const N_PORTS: u8 = 3;
    const _: () = assert!(N_PORTS as usize == board::plumbing::NET_PORTS);
    const _: () = assert!(ble_bringup::net_ports(true) == N_PORTS);

    /// VTOR alignment invariant: the `RAM_VECTORS` static (packed by memory.x's `.ramtables`
    /// section) carries `RamVectorTable`'s own alignment, and VTOR accepts a table aligned to a
    /// power of two no smaller than the table itself, so the type is VTOR-valid as long as its
    /// alignment covers its size (a runtime guard at the call site double-checks the placed
    /// address). Stated against the type, not a number: the HAL sizes the table (64 slots, 256 B
    /// since 2026-10-24), and this holds whatever it picks.
    const _: () =
        assert!(core::mem::align_of::<RamVectorTable>() >= core::mem::size_of::<RamVectorTable>());

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

    /// The demand-stale coast in microseconds: the window after which the 16 kHz period ISR floats
    /// all three phases because the 250 Hz control task has not rewritten demand. Derived from the
    /// constants that define it rather than written as "16 ms", so a change to the PWM period or to
    /// the stale threshold moves this with them. The ISR runs center-aligned at `2 * ARR` counts.
    const COAST_US: u32 =
        motor::DEMAND_STALE_PERIODS * 2 * commutation::ARR as u32 / (CLOCK.sysclk_hz / 1_000_000);

    /// The IMU read is the control task's one blocking call, and the control task is what the coast
    /// is waiting for, so the worst case the I2C driver can make a single pass wait has to fit
    /// inside the coast with real margin. Checked at compile time rather than believed: a change to
    /// the HAL's poll budget, to the core clock, to the PWM period or to the stale threshold that
    /// broke this would otherwise be found on a machine trying to stay upright.
    ///
    /// The factor of two is deliberate. Fitting merely "inside" the coast would mean a failing IMU
    /// read spends nearly the whole window the motor has to tolerate a quiet control task, leaving
    /// nothing for the ordinary jitter the coast was sized for in the first place.
    const IMU_READ_BLOCK_FITS_COAST: () =
        assert!(runtime_hal::i2c::worst_case_block_us(CLOCK.sysclk_hz) * 2 < COAST_US);
    const _: () = IMU_READ_BLOCK_FITS_COAST;

    /// The same assertion for the OTHER polled peripheral the main loop blocks on: the serial
    /// write path (`runtime_hal::serial`'s write budget).
    ///
    /// One pass can take more than one write call (the inter-board cyclic emission, plus a PDU
    /// forwarded to the peer), and they share the pass with the control task that has to refresh
    /// the demand word before the coast expires. The same factor of two, for the same reason.
    ///
    /// This bound did not exist before 2026-10-22: every wait on that path was an unbounded
    /// `while !flag {}`, and the offroad master answered it with eleven independent-watchdog
    /// resets in one bench session (`RCU_RSTSCK` bit 29 set on the master, clear on its peer).
    const UART_WRITE_BLOCK_FITS_COAST: () =
        assert!(runtime_hal::serial::worst_case_write_block_us(CLOCK.sysclk_hz) * 2 < COAST_US);
    const _: () = UART_WRITE_BLOCK_FITS_COAST;

    /// The rate the period ISR runs at, derived from the configured clock tree and the timer
    /// contract rather than written down as "16 kHz". TIMER0 is on APB2 and this tree runs APB2
    /// undivided (asserted below), so the timer clock is the sysclk.
    const PERIOD_HZ: u32 = motor::period_isr_hz(CLOCK.sysclk_hz);

    /// TIMER0's clock is the sysclk only while APB2 runs undivided; at any other APB2 prescaler a
    /// GD32 timer is fed `2 x APB2` instead, and `PERIOD_HZ` above would be wrong. The tree is a
    /// named constant either way, so check it here rather than carrying the assumption in prose.
    const TIMER_CLOCK_IS_THE_SYSCLK: () = assert!(CLOCK.apb2_psc == 1);
    const _: () = TIMER_CLOCK_IS_THE_SYSCLK;

    /// The same timer clock as the number the dead-time floor is derived through
    /// (`board::GATE_TIMER_HZ`, which converts the fleet's 500 ns reference into DTG ticks). The
    /// validator cannot see this tree, so the agreement is asserted here, where both are in
    /// scope: a clock change that shortened every staged dead time fails the build rather than
    /// the bridge.
    const GATE_TIMER_CLOCK_MATCHES_THE_FLOOR: () = assert!(CLOCK.sysclk_hz == board::GATE_TIMER_HZ);
    const _: () = GATE_TIMER_CLOCK_MATCHES_THE_FLOOR;

    /// The commutation front end reads the halls once per period and there are six sectors per
    /// electrical revolution, so the period rate sets a hard ceiling on the rotor speed it can
    /// track. Asserted rather than believed, for the same reason `IMU_READ_BLOCK_FITS_COAST` is:
    /// a change to the PWM period or the core clock that dropped the ceiling back into the
    /// machine's working range would otherwise be found on a bench, under load, as a motor that
    /// stops accelerating and starts drawing current instead. That is not hypothetical: it is the
    /// 2026-08-12 session, whose cause was the hall front end silently dropping edges.
    ///
    /// At the reference 16 kHz the ceiling is 2,666 Hz electrical, ~10,666 rpm on the fleet's
    /// 15-pole-pair hub, against a 250 Hz / 1,000 rpm floor.
    const PERIOD_RATE_TRACKS_THE_WORKING_RANGE: () = assert!(
        commutation::foc::tracked_electrical_hz_ceiling(PERIOD_HZ)
            >= commutation::foc::MIN_TRACKED_ELECTRICAL_HZ
    );
    const _: () = PERIOD_RATE_TRACKS_THE_WORKING_RANGE;

    /// The hall debounce window is specified as a TIME and lowered to a period count against
    /// `PERIOD_HZ`. At a high enough period rate that count would not fit the `i16` lockout and
    /// would saturate, silently shortening the window; this asserts the round-trip instead, so the
    /// window a board actually runs is never shorter than the one that was specified.
    const HALL_DEBOUNCE_WINDOW_SURVIVES_THE_PERIOD_RATE: () = assert!(
        commutation::foc::hall_debounce_window_us(PERIOD_HZ) >= commutation::foc::HALL_DEBOUNCE_US
    );
    const _: () = HALL_DEBOUNCE_WINDOW_SURVIVES_THE_PERIOD_RATE;

    /// The RAM vector table (.bss, zero-init; the VTOR alignment rides the type). A plain
    /// static so its initialization costs no stack (see the bring-up comment at its use).
    static mut RAM_VECTORS: RamVectorTable = RamVectorTable {
        slots: [0; MAX_VECTORS],
    };

    /// The inter-board USART's DMA RX ring (.bss; same pattern as [`RAM_VECTORS`]).
    static mut DMA_RING: [u8; DMA_CAP] = [0; DMA_CAP];

    /// The inter-board USART's DMA TX buffer (.bss; same pattern as [`DMA_RING`]). A DMA transfer
    /// reads its source for as long as it runs, so the transmitter owns this copy rather than the
    /// caller's frame. Sized to [`UART_FRAMER_N`], the largest encoded frame this port can emit, so
    /// a whole frame always goes in ONE transfer and never comes back truncated.
    static mut UART_TX_DMA: [u8; UART_FRAMER_N] = [0; UART_FRAMER_N];

    /// The BLE port's DMA RX ring (.bss; same pattern as [`DMA_RING`]). Sized and justified by
    /// [`crate::ble_carrier::RING_CAP`], which also holds why the port has a ring at all.
    static mut BLE_DMA_RING: [u8; crate::ble_carrier::RING_CAP] = [0; crate::ble_carrier::RING_CAP];

    /// The BLE port's DMA TX buffer (.bss; the [`UART_TX_DMA`] pattern): the transmitter's own copy
    /// of the frame its channel is reading (`specs/ble-port-dma-tx.md`). Sized to [`BLE_FRAMER_N`],
    /// the largest encoded frame this port emits, so a frame always goes in ONE transfer.
    static mut BLE_TX_DMA: [u8; BLE_FRAMER_N] = [0; BLE_FRAMER_N];

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
    /// BLE port recovered RX-LOSS counts (OBS), packed `lap_overruns | line_errors << 16`: the
    /// conditions the BLE carrier absorbed on the CC2541's UART since boot, sampled from the BLE link
    /// each loop pass and read at publish. Two u16 saturating counters in one word, the
    /// `motor_fault` packing precedent, because they answer one question between them.
    ///
    /// This instrument is what turned the BLE port's inbound losses from an argument into a number,
    /// and then into this port's DMA ring. Against the POLLED carrier the port used to ship, a 47.5 s
    /// drive capture stepped the low half by **634 in 47.5 s** (~13/s) with the high half flat at
    /// zero: the bytes were arriving and the firmware was missing them, because a polled port's whole
    /// receive buffer is its one-byte data register and the 250 Hz control callback (1,399 us on an
    /// IMU-equipped F130, 1,011 us on the F103; 3,941 us for the bounded wedged-I2C path) is longer
    /// than a 9600-baud character time (1,041.7 us).
    ///
    /// The halves keep their meanings across that carrier change, which is what makes the number
    /// comparable either side of it: the low half is still "the consumer did not come back in time
    /// and inbound bytes are gone", now a DMA ring LAP rather than a one-byte-register overrun, and
    /// the high half is still the wire-glitch half of the OQ1 split, kept apart so a noisy module link
    /// cannot be read as a control loop that stopped listening. What changes is the SCALE, and that is
    /// the point: the buffer that has to be outrun is [`crate::ble_carrier::RING_CAP`] bytes rather
    /// than one, so the low half should now read ZERO, and any non-zero reading is a real finding
    /// rather than the expected cost of a control tick.
    static BLE_RX_LOSSES: AtomicU32 = AtomicU32::new(0);
    /// Mailbox port DROPPED-FRAME count (OBS): the L2 frames the SWD mailbox refused since boot,
    /// sampled from the link each loop pass and read at publish. The third port's loss counter,
    /// beside the inter-board port's two and the BLE port's pair, and the only one that counts an
    /// OUTBOUND loss: the other two count inbound bytes the receiver missed, this one counts replies
    /// the board could not place (`specs/swd-mailbox.md`, "Backpressure", requirement 4).
    ///
    /// Two causes, separated by the OUTBOUND RING and not by `epoch` (`swd_mailbox::MailboxSerial::
    /// refused_writes` holds the rule). Nothing ever clears `epoch`, so a non-zero one means a
    /// bridge attached at some point this boot, not that one is attached now; the usual state on a
    /// board that has been talked to is a non-zero `epoch` and no bridge present:
    ///
    /// - A climbing count with `t2h_used` (`t2h_head - t2h_tail`) at or near the ring size is a real
    ///   backlog: something reached this board and stopped draining its replies. That is the reading
    ///   that explains a missing walk reply.
    /// - A climbing count with `t2h` EMPTY is the at-the-source drop of a board nothing has reached
    ///   this boot, which is the EXPECTED reading on a slave (the host attaches to the master's
    ///   mailbox, so every forwarded walk's port probe scores one) and says nothing is wrong.
    static MAILBOX_DROPS: AtomicU32 = AtomicU32::new(0);

    /// This wheel's LIVE odometer: the total its store last recorded plus every hall edge counted
    /// since boot, in absolute edges (`specs/odometer.md`).
    ///
    /// Written by the service loop, which owns the fold ([`crate::odometer::Odometer`]) and is the
    /// only context that may reach flash with it; read by the 250 Hz pipeline for the cyclic
    /// payload, and by an SWD session by symbol, which is how the bench reads a ride back. One
    /// writer, `Relaxed`, the [`MAILBOX_DROPS`] pattern.
    static ODOMETER_TOTAL: AtomicU32 = AtomicU32::new(0);

    /// Write calls the polled serial adapters ABANDONED because a transmit flag did not come up
    /// inside their write budget, packed `inter-board | BLE << 16`
    /// (`runtime_hal::serial`'s `tx_timeouts`, saturating u16 each).
    ///
    /// **The expected reading is zero on both halves**, and a non-zero one is specific: the port
    /// stopped accepting bytes and the frame was dropped rather than the main thread spinning on
    /// it. Before that bound existed the same condition was an unbounded spin, and the offroad
    /// master answered it with eleven watchdog resets in one session (2026-10-22). Published
    /// because the alternative is inferring it: a 200 ms main-thread pause is ~50 capped 4 ms
    /// waits, and that is a prediction a counter can settle and an argument cannot.
    static TX_TIMEOUTS: AtomicU32 = AtomicU32::new(0);

    /// Why this board last reset: the RCU `RSTSCK` cause field, read and cleared in `main`
    /// immediately after `detect_chip` resolves the RCU base, at the register bit positions the
    /// manuals give. Published as `CTRL_OBS` word 38, where the field's documentation lives.
    ///
    /// `.bss`, written once on the boot path and read-only thereafter, so the 250 Hz publish
    /// re-publishes the same value every pass.
    static RESET_CAUSE: AtomicU32 = AtomicU32::new(0);

    // ===================== The stack high-water instrument (`crate::stack_paint`) ================
    //
    // Three words of `.bss` (12 B, which costs 12 B of the very stack region they measure, and is
    // accounted for in `specs/firmware.md`'s budget). Main-thread only: the boot path writes the
    // first two once, and the 250 Hz publish is the sole reader/writer of all three thereafter.
    //
    /// Words of free stack painted at boot ([`paint_stack`]). Zero until the paint has run, and
    /// zero forever on a board with no free stack left to paint, which publishes as a zero margin.
    static STACK_PAINTED_WORDS: AtomicU32 = AtomicU32::new(0);
    /// The chunked sweep's position within `[0, STACK_MARK)`, in words above `_stack_end`.
    static STACK_SCAN_CURSOR: AtomicU32 = AtomicU32::new(0);
    /// The high-water mark: words of paint still INTACT above `_stack_end`, i.e. the margin in
    /// words. Starts at the painted length (nothing observed yet) and only ever decreases.
    static STACK_MARK: AtomicU32 = AtomicU32::new(0);

    /// Words scanned per 250 Hz publish.
    ///
    /// Sized against the tightest budget in the image rather than by feel, and PRICED off the
    /// emitted loop rather than estimated: the scan compiles to 6 instructions per word
    /// (`ldr` indexed, `cmp`, `bne`, `add`, `cmp`, `bne`), so <= 9 core cycles per word, so 32 words
    /// <= 288 cycles = **4.0 us at 72 MHz**.
    ///
    /// The budget it has to fit inside is the 250 Hz control callback's, which measures 1,011 us on
    /// the F103 against a 1,041.7 us 9600-baud character time
    /// (`specs/bench-evidence/2026-08-02/wide-division/RECORD.md`): ~30 us of margin before the
    /// callback starts costing the BLE port an inbound byte. 4.0 us is 13% of that.
    ///
    /// An UNCHUNKED full scan of this image's 661 painted words would be ~5,950 cycles = ~83 us,
    /// nearly three times the whole margin, on every publish. That would make this instrument a
    /// cause of exactly the losses [`BLE_RX_LOSSES`] exists to count, which is the specific way a
    /// diagnostic stops being free.
    const STACK_SCAN_CHUNK: usize = 32;

    /// Bytes left unpainted immediately below the stack pointer at paint time.
    ///
    /// The paint runs from `_stack_end` up to `MSP - STACK_PAINT_GUARD`, never to `_stack_start`:
    /// everything at and above MSP is the live frame chain that called it, and painting through
    /// that would destroy this function's own return address. 64 B clears a 32 B Cortex-M exception
    /// frame with room to spare, for a fault taken mid-paint (no interrupt source is enabled this
    /// early, so this is headroom, not a live race).
    ///
    /// It costs nothing in accuracy. The measurement is about the LOWEST address reached; leaving
    /// the top 64 B unpainted only means those bytes are never counted as free, which is the
    /// conservative direction.
    const STACK_PAINT_GUARD: usize = 64;

    extern "C" {
        /// cortex-m-rt's `PROVIDE(_stack_end = __euninit)`: the lowest address the stack may reach
        /// before it collides with static data. The floor of the painted region, and the reason the
        /// paint cannot destroy `CTRL_OBS`, which lives BELOW it in `.uninit`.
        static _stack_end: u8;
    }

    /// Paint the free stack and return how many words were painted.
    ///
    /// The region is `[_stack_end, MSP - STACK_PAINT_GUARD)`: everything above the live stack
    /// pointer belongs to the frames that called this one, and everything below `_stack_end` is
    /// static data. **661 words / 2,644 B on the release image** (ELF-measured 2026-08-11:
    /// `_stack_end` 0x2000_0E84, MSP here 0x2000_1918).
    ///
    /// That is deliberately NOT the whole 4,476 B between `_stack_end` and `_stack_start`, and the
    /// difference is not waste: the boot function's own frame is 1,744 B and stays live for the
    /// entire run, so the stack below it is the only stack anything can ever free up. Painting
    /// through a live frame would destroy this function's return address.
    ///
    /// `#[inline(never)]` so the MSP read and the paint loop share one frame that the paint stays
    /// strictly below. Inlined into the caller, a later local of the caller's could be allocated
    /// inside the range the paint had already written.
    #[inline(never)]
    fn paint_stack() -> u32 {
        // The region floor is a link-time constant; the top is the live stack pointer, because the
        // frames that called this one are above it and must survive.
        let floor = addr_of!(_stack_end) as usize;
        let top = (cortex_m::register::msp::read() as usize).saturating_sub(STACK_PAINT_GUARD);
        if top <= floor {
            return 0;
        }
        let words = (top - floor) / 4;
        // SAFETY: `_stack_end` is 4-aligned (cortex-m-rt ALIGNs it) and `[floor, floor + 4*words)`
        // is free stack: above every static (`.bss` and `.uninit` both end at or below `_stack_end`)
        // and strictly below the current stack pointer, so no live frame and no static overlaps it.
        unsafe { crate::stack_paint::paint(floor as *mut u32, words) };
        words as u32
    }

    /// Advance the high-water sweep by one bounded chunk and return the packed `CTRL_OBS` word.
    /// Called once per 250 Hz publish, the same cadence and the same single writer as every other
    /// observation in the block.
    fn sample_stack_margin() -> u32 {
        let painted = STACK_PAINTED_WORDS.load(Ordering::Relaxed) as usize;
        if painted == 0 {
            return 0;
        }
        let floor = addr_of!(_stack_end) as *const u32;
        let cursor = STACK_SCAN_CURSOR.load(Ordering::Relaxed) as usize;
        let mark = STACK_MARK.load(Ordering::Relaxed) as usize;
        // SAFETY: `floor` is 4-aligned with `painted >= mark` readable words above it (the region
        // painted at boot), and the sweep maintains `cursor <= mark`.
        let (next_cursor, next_mark) =
            unsafe { crate::stack_paint::sweep(floor, cursor, mark, STACK_SCAN_CHUNK) };
        STACK_SCAN_CURSOR.store(next_cursor as u32, Ordering::Relaxed);
        STACK_MARK.store(next_mark as u32, Ordering::Relaxed);
        crate::stack_paint::pack(next_mark * 4, painted * 4)
    }
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
    /// configured mask (an absent field samples as its idle level) and the board's polarity
    /// word. Line order: 0 = button, 1 = pad A, 2 = pad B, the order
    /// `store::BOARD_INPUT_POLARITY` numbers its bits in.
    struct InputPins {
        group: Option<InputGroup>,
        has_button: bool,
        has_pad_a: bool,
        has_pad_b: bool,
        /// `store::BOARD_INPUT_POLARITY` (0x6C) as the board stores it: a set bit is a line that
        /// asserts HIGH. Read once at boot, like the pins it goes with; `inputs::asserted` is
        /// the one place it is applied, and masks the reserved bits.
        polarity: u8,
    }

    impl InputPins {
        /// The two pad lines' configured-ness as `CTRL_OBS` word 37 carries it: b0 = pad A, b1 =
        /// pad B. A pad with no pin is sampled as idle and can never assert, and this bit is how a
        /// bench read tells that board from one whose staged pads simply have no foot on them.
        fn pads_configured(&self) -> u8 {
            (self.has_pad_a as u8) | ((self.has_pad_b as u8) << 1)
        }
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
        /// The guest ports' samples ([`GuestSlots`]): the board's own decimated cyclic payload
        /// (5 Hz, `dispatch::ble_cyclic_tx`) and the PEER's forward (2.5 Hz,
        /// `dispatch::ble_peer_tx`), each a latest-wins slot separate from `cyclic_out` because
        /// the ports run at different rates. The mailbox port takes a sample in the pass it was
        /// built; the BLE port takes it when its staging buffer is idle, the own one first
        /// (`specs/link-control.md`, "The peer's state on the BLE port: the forward" and "The
        /// mailbox port carries both samples (2026-10-25)").
        guest: GuestSlots,
        /// Samples each guest port accepted, `CTRL_OBS` words 40 and 41 ([`GuestStaged`]).
        guest_staged: GuestStaged,
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
        /// The last MEASURED period-liveness observation, published into `CTRL_OBS`'s motor-state
        /// flags (`motor::OBS_PERIOD_LIVE`). Held across a catch-up run because that run measures
        /// nothing: its `dt_ticks` is zero, so there is no interval to take a rate over, and
        /// publishing a fresh `false` there would report a stopped ISR on the evidence of no
        /// elapsed time at all.
        period_live: bool,
        /// The period-liveness fault level's hysteresis state (`specs/motor-integration.md`, "the
        /// motor-side fault producers"). Main-thread only, like every other field here: the ISR
        /// publishes the raw counter and this task decides when a shortfall has lasted long enough
        /// to be a fault.
        period_health: motor::PeriodHealth,
        /// The period ISR's over-current trip count as of the previous control run: a change
        /// raises the motor-0 fault latch (`specs/motor-integration.md`, "The hard trip").
        last_trips: u32,
    }

    /// The shell static. `None` until the boot path builds it (the state is not
    /// const-constructible); the SysTick interrupt does not touch it (only the scheduler), so
    /// initialization order only has to precede task dispatch, which it does (the tick source is
    /// enabled after).
    ///
    /// In `.uninit` (cortex-m-rt's NOLOAD section), so the `None` below is NOT loaded at reset:
    /// left in `.data`, the 728 B static's whole init image (all zero but the niche byte that
    /// spells `None`) sat in flash. `main` writes `None` before anything else reads it, which is
    /// the state `.data` used to give it, so every reader still sees `None` until `init_shell`.
    #[link_section = ".uninit.SHELL"]
    static mut SHELL: Option<Shell> = None;

    /// The mounted store's flash adapter, a main-thread static for the same reason [`STORE`] below
    /// is one (it is what that store borrows, so it has to outlive it).
    ///
    /// In `.uninit` beside `SHELL` and `motor::hw::MOTOR`, under the same discipline: `main` writes
    /// the `None` its initializer names before anything can read it, so no reader ever sees reset
    /// garbage and the init image costs no flash.
    #[link_section = ".uninit.FLASH"]
    static mut FLASH: Option<FmcFlash> = None;

    /// The mounted config store, as a main-thread static.
    ///
    /// **Why a static and not a `main` local any more.** The 250 Hz control task's ARM path has to
    /// re-read the value-only store fields before it energizes the bridge
    /// (`specs/integration.md`, "When a stored value takes effect: the arm-time re-read"), and that
    /// task is a bare dispatch callback (`control_task_cb`) that reaches state only through a
    /// static. A `main`-frame local handed to [`service_loop`] by `&mut` is unreachable from there.
    ///
    /// **The discipline is `SHELL`'s, with one honest difference.** The loop and the dispatch
    /// callbacks run in the SAME thread (the `specs/integration.md` execution model), so borrows
    /// never overlap as long as each one is SCOPED to end before `dispatch()`. Every borrow taken
    /// once the dispatch exists is scoped that way, to a single statement, by
    /// `(*addr_of_mut!(STORE)).as_mut()` / `.as_ref()` rather than a reference to the static: the
    /// five in `service_loop` and the one in `re_read_arm_values`.
    ///
    /// The BOOT's borrow is not scoped, and this says so rather than claiming otherwise: `main`
    /// binds one `&'static mut` from here for its ~40 boot reads and never drops it (`main` never
    /// returns), so what makes it sound is ordering, that its last use precedes the entry to
    /// `service_loop`, and therefore precedes the existence of any other borrow. That is the same
    /// pattern this file already uses for `RAM_VECTORS`, `DMA_RING` and `BLE_DMA_RING`: one
    /// `&'static mut` formed once on the boot thread, before the interrupt or the loop that would
    /// contend for it.
    ///
    /// In `.uninit`, as [`FLASH`].
    #[link_section = ".uninit.STORE"]
    static mut STORE: Option<Store<'static, FmcFlash>> = None;

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
        /// fault_a; `specs/motor-integration.md`), b6 imu_staged (the staging ASKED for an IMU:
        /// with b0 clear it is a bring-up that failed, which is a finding, and without it a b0 of
        /// 0 just means no IMU is fitted). b7 is free. Bits 5 and 6 are NEW bits in an existing
        /// byte, so every field offset is unchanged.
        ///
        /// Packed by `orchestrator::Obs::flags_byte`, which owns the bit assignment and is
        /// host-tested; this shell adds nothing to the byte.
        flags: u8,
        /// The live gating-producer level mask (`orchestrator::events::EV_*`: b0 comms_loss,
        /// b1 stop_all (the tilt cutout), b2 imu_loss, b3 motor_fault, b4 latch A, b5 latch B, b6 mode_fault,
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
        ///
        /// The method byte is the one RUNNING, not the one stored: it is read off the records the
        /// ISR actually steps (`motor::method_records`), so it is the bench's way to confirm which
        /// arm is live. It differs from `MOTOR_METHOD` exactly when the capability gate demoted the
        /// request, which `motor_fault`'s `motor::FAULT_METHOD` bit says in so many words.
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
        /// Nonzero on every boot of a configured motor (the calibration is unconditional: the
        /// current limit needs the zeros); zero when no motor was brought up or the conversions
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
        /// The BLE port's recovered RX-LOSS counts ([`BLE_RX_LOSSES`]), packed
        /// `lap_overruns | line_errors << 16`. Appended LAST, so every prior field keeps its offset
        /// (the offset-preserving append the ISR-metric, roll, motor and balance blocks all used);
        /// word 29 in the SWD map.
        ///
        /// The third port's counters, next to `link_line_errors` / `link_lap_overruns`, which are
        /// the inter-board port's, and now the same two quantities read the same way: the BLE port
        /// carries the same DMA-ring receiver the inter-board port does. The low half means what it
        /// means there, that the consumer did not come back before the ring lapped and inbound bytes
        /// are gone.
        ///
        /// **The expected reading is zero, and it was not always.** Against the polled carrier this
        /// port used to ship, the low half stepped ~13/s under a phone drive
        /// (`specs/bench-evidence/2026-08-02/wide-division/RECORD.md` and `specs/ble.md`), because the
        /// buffer that had to be outrun was one byte and a control tick is longer than a character
        /// time. Against the ring the buffer is [`crate::ble_carrier::RING_CAP`] bytes, roughly 67 ms
        /// of wire at 9600 baud, so a non-zero low half is now a genuine finding: it means the loop
        /// missed many consecutive passes, not that it took one ordinary control tick.
        ///
        /// Zero is a real answer here, not an absent one: the counter is published every pass from
        /// boot, and its host tests pin both directions (`crate::ble_carrier`: the measured
        /// caller-away windows cost no bytes AND leave this word at zero), so a zero read on a board
        /// that has been streaming means the bytes survived.
        ble_rx_losses: u32,
        /// The stack high-water instrument ([`STACK_MARK`]; `crate::stack_paint`), packed
        /// `free_bytes | painted_bytes << 16`. Appended LAST, so every prior field keeps its offset
        /// (the offset-preserving append the ISR-metric, roll, motor, balance and BLE blocks all
        /// used); word 30 in the SWD map.
        ///
        /// **Low half: the margin**, the bytes of paint still intact above `_stack_end`, i.e. the
        /// gap between the deepest word the image has ever touched and the top of `.uninit`. This is
        /// the number the >= 250 B floor policy is stated against (`specs/firmware.md`, "RAM/stack
        /// budget"): below it, one more exception frame lands in `CTRL_OBS` itself and the board
        /// starts corrupting the block a bench read is trying to interpret, which is the round-6
        /// signature.
        ///
        /// **High half: the denominator**, the bytes actually painted at boot. It is what makes a
        /// reading interpretable rather than merely a number. `free < painted` is a MEASURED margin:
        /// the stack reached into the paint and the low half is where it stopped. `free == painted`
        /// means the paint was never reached at all, so the margin is only known to be AT LEAST that
        /// much. Without the denominator those two are the same u16 and mean opposite things.
        ///
        /// Reads `0` before the first sweep completes and on a board whose statics have grown until
        /// there is no stack left to paint. Both are the safe direction: an instrument that has not
        /// measured yet reports no margin rather than a comfortable one.
        stack_margin: u32,
        /// The phase-current observation ([`motor::OBS_CURRENT`], packed by
        /// [`motor::pack_motor_current`]; `specs/motor-integration.md`, "The current limit").
        /// Appended LAST, so every prior field keeps its offset; word 31 in the SWD map.
        ///
        /// Bits 0..15: `peak`, the largest phase-current magnitude (i16, stock current counts) over
        /// the last COMPLETED 64-period window. Bits 16..23: `chopped`, the periods the soft limit
        /// floated in that window (0..64). Bits 24..31: `trips`, the over-current trip count's low
        /// byte (wrapping; the full count is [`motor::OVER_CURRENT_TRIPS`]). Amps are
        /// `peak / cal`, where `cal` is this board's own counts per amp
        /// (`store::MOTOR_CURRENT_CAL`, 0x67): a reader-side scale the reader reads off the board.
        /// The effective limit is not published because a reader computes it from
        /// `MOTOR_CURRENT_LIMIT`, that same field and `store::MOTOR_NOISE_FLOOR` (0x6B), which are
        /// exactly [`motor::limit_counts`]' three inputs and all three readable over the mailbox.
        motor_current: u32,
        /// The block's effective battery word ([`Obs::battery`]; `specs/sensing-and-safety.md`,
        /// "The battery word"). Appended LAST; word 32 in the SWD map.
        ///
        /// Bits 0..15: the word as `u16`, centivolts, 0 = UNKNOWN (the balance law then outputs 0
        /// and balance cannot engage). Bits 16..31: zero, reserved.
        battery: u32,
        /// The arm refusals (`specs/integration.md`, "The arm refusals (word 33, permanent)"),
        /// maintained in this packing by `arm::hw` through [`arm::note_re_read_refusal`] /
        /// [`arm::note_confirm_refusal`]. Appended LAST; word 33 in the SWD map.
        ///
        /// Bits 0..7: a saturating count of `arm::ArmStep::ReReadValues` refusals this boot.
        /// Bits 8..15: the same for `arm::ArmStep::ConfirmPeriodsLive`. Bits 16..23: the last
        /// refused IMU frame's cause byte ([`arm::imu_frame_cause_byte`]; 0 = no frame refusal this
        /// boot). Bits 24..31: zero, reserved.
        ///
        /// **Why a count and not a level.** The re-read refusal is RELEASED at the end of the
        /// engage that met it, so the level a bench read would otherwise infer it from is gone by
        /// the time anyone looks: a board refused for a bad axis frame, then left disarmed, reads
        /// exactly like a board that was never engaged. The counts separate "refused once and
        /// recovered" from "refusing every attempt", and the cause byte names which of the three
        /// frame checks failed without a second symbol to resolve. The confirm count is beside it
        /// because the even/odd reading that works for the event counters is not available here:
        /// that refusal is boot-sticky, so the count is the only record of how many attempts it
        /// ate.
        arm_refusals: u32,
        /// The mailbox port's dropped-frame count ([`MAILBOX_DROPS`]), the third port's loss counter.
        /// Appended LAST, so every prior field keeps its offset (the offset-preserving append every
        /// block above used); word 34 in the SWD map.
        ///
        /// A frame the mailbox could not place is DROPPED, which is what `specs/l2.md` already says
        /// every link does, and this word is the only trace it leaves. It used to be a panic instead:
        /// `MailboxSerial` answered `Ok(0)` on a full ring, `write_all` panicked on exactly that, and
        /// a `panic-halt` board stopped feeding the IWDG - the F130 slave's warm reset on every 28th
        /// host walk. So a stepping count here is now the normal, non-fatal form of that condition.
        ///
        /// **Non-zero is expected on a slave** and says nothing is wrong: no bridge attaches to a
        /// slave's mailbox, so each forwarded walk's port probe is dropped at the source and counted.
        /// What separates that from a lost reply is the OUTBOUND RING, read from the mailbox header
        /// in the same SWD session (fixed base, no second symbol): `t2h_head - t2h_tail` empty is the
        /// at-the-source drop, at or near the ring size is a peer that reached this board and
        /// stopped draining. `epoch` does not answer it, because nothing clears `epoch`: non-zero
        /// only means a bridge attached at some point this boot.
        mailbox_drops: u32,
        /// The polled serial adapters' abandoned writes, packed `inter-board | BLE << 16`
        /// ([`TX_TIMEOUTS`]). Appended LAST, so every prior field keeps its offset (the
        /// offset-preserving append the ISR-metric, roll, motor, balance, BLE and stack blocks all
        /// used); word 35 in the SWD map.
        tx_timeouts: u32,
        /// The peak peer cyclic age since boot, in control ticks
        /// ([`orchestrator::LinkInbox::cyclic_age_peak`]). Appended LAST, so every prior field
        /// keeps its offset (the offset-preserving append every block above used); word 36 in the
        /// SWD map.
        ///
        /// The live age beside it in word 7 is a SAMPLE: an SWD poll reads it at about 130 Hz
        /// against an age that advances at 250 Hz, so half the ticks are never seen and catching a
        /// rare excursion is a coin flip. This word is the same quantity LATCHED where it is
        /// maintained, so it has seen every tick. It is what `RIGID_STALE_TICKS` is now set from
        /// (`specs/rover-one-imu.md` section 8); nothing in the firmware reads it, and it changes
        /// no behaviour.
        ///
        /// Monotonic and never cleared: a soak run is one boot, and a reset is the clear.
        cyclic_age_peak: u32,
        /// The foot-pad block (`specs/silicon-queue.md`, "The foot pad on silicon"), packed
        /// `state | changes << 8 | held << 16`. Appended LAST, so every prior field keeps its
        /// offset (the offset-preserving append every block above used); word 37 in the SWD map.
        ///
        /// Bits 0..1: the debounced pad field, b0 pad A, b1 pad B. Bit 2: the LOCAL rider level
        /// (both pads). Bit 3 zero: it was the PID profile selection, which went with the profiles
        /// (`specs/state-feedback.md` section 5). Bit 4: pad A has a configured pin this boot; bit
        /// 5: pad B does. Bits 6..7 zero.
        /// Bits 8..15: the saturating count of passes that CHANGED the field. Bits 16..23: the
        /// saturating count of pad-passes whose debounced state disagreed with the pin. Bits
        /// 24..31 zero, reserved.
        ///
        /// **Why the counts and not only the bits.** The field is a 62.5 Hz quantity and an SWD
        /// poll reads it at about 130 Hz, so a level read catches what it happens to catch. The
        /// two counts have seen every pass, and between them they carry the two things a level
        /// cannot: a floating pad pin steps `changes` with nobody touching the board, and the
        /// one-sample engage against the two-sample release reads as `held` advancing by one per
        /// release and by nothing per engage.
        ///
        /// **Why the configured bits.** Without them a reading of zero is two different boards:
        /// one whose pads are staged and quiet, and one whose pads are absent. The padless half of
        /// the gate is about telling those apart.
        pad_obs: u32,
        /// **Why this board last reset** ([`RESET_CAUSE`]): the RCU `RSTSCK` cause field as the
        /// boot seam read it, before the same seam cleared it. Appended LAST, so every prior
        /// field keeps its offset (the offset-preserving append every block above used); word 38
        /// in the SWD map.
        ///
        /// The bits sit where the manuals put them, unmoved and un-re-encoded (GD32F1x0 User
        /// Manual Rev3.6 section 4.3.10, GD32F10x User Manual Rev2.6 section 5.3.10, which agree
        /// on all of these): LPRSTF 31 deep-sleep/standby, WWDGTRSTF 30, FWDGTRSTF 29 the free
        /// watchdog, SWRSTF 28 a software reset, PORRSTF 27 a power reset, EPRSTF 26 the external
        /// NRST pin, plus the F1x0-only OBLRSTF 25 and V12RSTF 23 that read zero on an F10x.
        ///
        /// **It is a per-boot reading because the seam clears.** The flags are sticky and
        /// accumulate across resets, so an uncleared register reports every reset since the part
        /// was last powered: a read on 2026-10-23 showed power-on and software both set and could
        /// not say which reset had just happened. Clearing at every boot makes each boot's word
        /// name that boot's reset.
        ///
        /// What a bench session gets out of it, which is the reason it exists: a board that
        /// resets whenever a host attaches its SWD mailbox reads EPRSTF or SWRSTF if the PROBE is
        /// resetting it and FWDGTRSTF if the firmware stopped feeding the watchdog, and those two
        /// readings were previously indistinguishable. A storm of 0 ticks per boot with PORRSTF is
        /// a brownout; the same storm with FWDGTRSTF is the watchdog. Both of those were
        /// identified by indirect evidence on 2026-10-23
        /// (`specs/bench-evidence/2026-10-23/rover-rigid/RECORD.md`), a restarting `boot_count`
        /// and a 504 ms uptime respectively.
        ///
        /// Zero is a reading too, not a failure to look: nothing in the cause field was set, which
        /// a core-only debug reset (`VECTRESET`, no system reset asserted) produces. `boot_count`
        /// in word 1 is the fact to read beside it; it is in the same block and the same `mdw`, so
        /// it is not copied here.
        reset_cause: u32,
        /// The period-based wheel rate (`specs/control.md` (o)): `motor::RATE_Q`, the reciprocal of
        /// the periods between two same-direction hall edges in 1/4096 of a count, the
        /// state-feedback law's speed measurement. Appended LAST, offset-preserving, at word 39.
        /// The hand-spin gate reads it: smooth and sign-correct while the wheel turns, decaying to
        /// zero within a second of it stopping, exactly 0 while it rocks between two hall codes.
        motor_rate_q: i32,
        /// Samples the BLE port ACCEPTED (`MeteredTx::stage` took them), packed
        /// `own u16 | forwards u16 << 16`, each half wrapping ([`GuestStaged`]). Appended LAST,
        /// offset-preserving, at word 40 (`specs/link-control.md`, "Counting what is staged
        /// (2026-10-25)").
        ///
        /// Read over SWD while a phone is attached, the high half advances by 2.5 a second if the
        /// forwards leave the board, in which case a forward the phone never shows was lost on the
        /// module or the phone; if it does not, the board skipped them. A sample the port refused
        /// (busy) is not counted.
        ble_staged: u32,
        /// The same for the SWD mailbox port: samples the ring took whole, own in the low half and
        /// forwards in the high half, wrapping; word 41. Advances only while a host is attached; a
        /// sample a full ring refused is counted in word 34 and not here.
        mailbox_staged: u32,
        /// This wheel's position in hall edges, FORWARD POSITIVE (`specs/state-feedback.md`
        /// section 8): `motor::POS_EDGES` as the block carries it, through the side mirror.
        /// Appended LAST, offset-preserving, at word 42. Live armed or not: gate 1 pushes the
        /// disarmed machine forward about a metre and reads this INCREASE by about 170.
        wheel_pos_own: i32,
        /// The PEER's wheel position in hall edges, FORWARD POSITIVE: the peer cyclic's
        /// `wheel_pos` unwrapped across its 16-bit wrap, through the opposite side's mirror; 0
        /// until a peer has been heard. Word 43. On the same push it agrees with word 42 within
        /// 10 %, which is the first measurement of the left wheel's sign.
        wheel_pos_peer: i32,
        /// The state-feedback law's pitch and pitch-rate terms (`specs/state-feedback.md` section
        /// 8), each as a torque word on its own, `term_p` low half, `term_r` high half. Appended
        /// LAST, offset-preserving, at word 44. Live every balance pass, armed or not: gate 1
        /// tilts the disarmed body forward and reads both NEGATIVE.
        term_p: i16,
        term_r: i16,
        /// The position and speed terms, `term_x` low, `term_v` high, word 45. Gate 1 pushes the
        /// disarmed machine forward and reads both NEGATIVE while it rolls.
        term_x: i16,
        term_v: i16,
        /// The slewed speed reference, 1/4096 count saturated to i16, low half; the position error
        /// `x_ref - x` in whole edges, high half. Word 46. Both zero outside RUN.
        v_ref_q: i16,
        e_x: i16,
        /// The FOC observation (`specs/foc-torque.md` section 7), three words of 64-period window
        /// means, each two `i16` packed by [`motor::pack_i16_pair`] and published by one relaxed
        /// store in the period ISR, so a bench read at a few hertz sees one window's pair.
        /// Appended LAST, offset-preserving, at words 47 to 49.
        ///
        /// Word 47 ([`motor::OBS_FOC_PHASE`]): the mean conditioned phase currents, `ia` low half,
        /// `ib` high half, in current counts (amps are counts over `store::MOTOR_CURRENT_CAL`,
        /// 0x67), live in EVERY method from the current limit's own window. Gate O2 runs Clarke and
        /// Park over it on the host, in sine, at a held rotor's angle (word 22).
        foc_phase_mean: u32,
        /// Word 48 ([`motor::OBS_FOC_DQ`]): the FOC arm's mean measured currents, `id` low half,
        /// `iq` high half (`iq` along the torque axis). 0 in other methods.
        foc_dq_mean: u32,
        /// Word 49 ([`motor::OBS_FOC_REF`]): the `iq` reference of the window's closing period,
        /// low half, and the window's mean `vq` after the circular limit, high half. 0 in other
        /// methods. `vq` near 32,000 with `iq` under its reference is the voltage ceiling; `vd`
        /// is not published (the d loop's health is word 48's `id` reading zero).
        foc_ref_vq: u32,
    }

    /// Pin every byte offset the SWD readers key on (`tools/imu-tilt.py`'s word map, the bench
    /// notes, and every ad-hoc `mdw <CTRL_OBS> N`), the imu-bench `Obs` precedent.
    ///
    /// The block is read by SYMBOL AND OFFSET, never by name, and the reader lives outside this
    /// repo's build, so a field inserted into the middle of the struct rather than appended does
    /// not fail anything: it silently renumbers every word after it, and every later diagnosis
    /// reads one field's value under another field's name. That failure is invisible at the bench
    /// and expensive, which is what makes it worth a compile-time assertion rather than a comment
    /// asking for care. This is the only thing standing between "appended LAST (offset-preserving)"
    /// in the docs above and it actually being true.
    ///
    /// Word N of the SWD map is byte offset 4*N for every u32 field, so the two halves of the
    /// contract (the word map and these offsets) are the same statement. Appending is always legal;
    /// changing anything below is a deliberate break that every reader must be updated for.
    const _: () = {
        use core::mem::offset_of;
        // Words 0-10: the counters and ages.
        assert!(offset_of!(CtrlObs, magic) == 0x00);
        assert!(offset_of!(CtrlObs, boot_count) == 0x04);
        assert!(offset_of!(CtrlObs, tick_count) == 0x08);
        assert!(offset_of!(CtrlObs, dispatch_count) == 0x0C);
        assert!(offset_of!(CtrlObs, control_ticks) == 0x10);
        assert!(offset_of!(CtrlObs, input_ticks) == 0x14);
        assert!(offset_of!(CtrlObs, pitch_milli) == 0x18);
        assert!(offset_of!(CtrlObs, cyclic_age) == 0x1C);
        assert!(offset_of!(CtrlObs, drive_age) == 0x20);
        assert!(offset_of!(CtrlObs, enact_inits) == 0x24);
        assert!(offset_of!(CtrlObs, enact_shutdowns) == 0x28);
        // Word 11: the packed torque / mode / MOE word.
        assert!(offset_of!(CtrlObs, torque) == 0x2C);
        assert!(offset_of!(CtrlObs, mode_byte) == 0x2E);
        assert!(offset_of!(CtrlObs, moe_bits) == 0x2F);
        // Word 12: the four state/flag bytes, including the byte that WAS the reserved pad.
        assert!(offset_of!(CtrlObs, sub_state) == 0x30);
        assert!(offset_of!(CtrlObs, control_mode) == 0x31);
        assert!(offset_of!(CtrlObs, flags) == 0x32);
        assert!(offset_of!(CtrlObs, event_levels) == 0x33);
        // Words 13-14: the OQ1 split (the inter-board port's two counters).
        assert!(offset_of!(CtrlObs, link_line_errors) == 0x34);
        assert!(offset_of!(CtrlObs, link_lap_overruns) == 0x38);
        // Words 15-17: the per-vector ISR entry counts.
        assert!(offset_of!(CtrlObs, systick_isr_entries) == 0x3C);
        assert!(offset_of!(CtrlObs, usart1_isr_entries) == 0x40);
        assert!(offset_of!(CtrlObs, dma_isr_entries) == 0x44);
        // Word 18: roll (imu-tilt.py's ROLL_WORD).
        assert!(offset_of!(CtrlObs, roll_milli) == 0x48);
        // Words 19-25: the motor block.
        assert!(offset_of!(CtrlObs, motor_periods) == 0x4C);
        assert!(offset_of!(CtrlObs, motor_state) == 0x50);
        assert!(offset_of!(CtrlObs, motor_duty01) == 0x54);
        assert!(offset_of!(CtrlObs, motor_duty2_angle) == 0x58);
        assert!(offset_of!(CtrlObs, motor_fault) == 0x5C);
        assert!(offset_of!(CtrlObs, motor_speed) == 0x60);
        assert!(offset_of!(CtrlObs, motor_cal) == 0x64);
        // Words 26-28: the balance-era block and the per-producer transition counts.
        assert!(offset_of!(CtrlObs, gating_field) == 0x68);
        assert!(offset_of!(CtrlObs, pre_env_torque) == 0x6A);
        assert!(offset_of!(CtrlObs, event_counts) == 0x6C);
        // Word 29: the BLE port's RX losses.
        assert!(offset_of!(CtrlObs, ble_rx_losses) == 0x74);
        // Word 30: the stack high-water margin.
        assert!(offset_of!(CtrlObs, stack_margin) == 0x78);
        // Word 31: the phase-current observation.
        assert!(offset_of!(CtrlObs, motor_current) == 0x7C);
        // Word 32: the battery word (the battery slice's follow-up append).
        assert!(offset_of!(CtrlObs, battery) == 0x80);
        // Word 33: the arm refusals (the arm-time re-read's retryable refusal, made visible).
        assert!(offset_of!(CtrlObs, arm_refusals) == 0x84);
        // Word 34: the mailbox port's dropped frames (the backpressure slice).
        assert!(offset_of!(CtrlObs, mailbox_drops) == 0x88);
        // Word 35: the polled serial adapters' abandoned writes (the bounded-TX slice).
        assert!(offset_of!(CtrlObs, tx_timeouts) == 0x8C);
        // Word 36: the peak peer cyclic age (the rigid-window instrument).
        assert!(offset_of!(CtrlObs, cyclic_age_peak) == 0x90);
        // Word 37: the foot-pad block.
        assert!(offset_of!(CtrlObs, pad_obs) == 0x94);
        // Word 38: why this boot's reset happened (the RCU cause field, read and cleared at boot).
        assert!(offset_of!(CtrlObs, reset_cause) == 0x98);
        // Word 39: the period-based wheel rate, the state-feedback law's speed measurement.
        assert!(offset_of!(CtrlObs, motor_rate_q) == 0x9C);
        // Words 40-41: samples staged per guest port, BLE then mailbox.
        assert!(offset_of!(CtrlObs, ble_staged) == 0xA0);
        assert!(offset_of!(CtrlObs, mailbox_staged) == 0xA4);
        // Words 42-43: the two wheel positions, own then peer, forward positive.
        assert!(offset_of!(CtrlObs, wheel_pos_own) == 0xA8);
        assert!(offset_of!(CtrlObs, wheel_pos_peer) == 0xAC);
        // Words 44-46: the state-feedback law's four terms, then its reference and position error.
        assert!(offset_of!(CtrlObs, term_p) == 0xB0);
        assert!(offset_of!(CtrlObs, term_r) == 0xB2);
        assert!(offset_of!(CtrlObs, term_x) == 0xB4);
        assert!(offset_of!(CtrlObs, term_v) == 0xB6);
        assert!(offset_of!(CtrlObs, v_ref_q) == 0xB8);
        assert!(offset_of!(CtrlObs, e_x) == 0xBA);
        // Words 47-49: the FOC observation, phase means, then dq means, then reference and vq.
        assert!(offset_of!(CtrlObs, foc_phase_mean) == 0xBC);
        assert!(offset_of!(CtrlObs, foc_dq_mean) == 0xC0);
        assert!(offset_of!(CtrlObs, foc_ref_vq) == 0xC4);
        // And no tail padding hiding a mis-sized field: 50 words (200 B) exactly.
        assert!(core::mem::size_of::<CtrlObs>() == 50 * 4);
    };

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

    /// Publish one pipeline pass into `CTRL_OBS`: one volatile write per field, straight into the
    /// block, in the block's own field order.
    ///
    /// It used to build a whole `CtrlObs` and `write_volatile` it. That is a 136-byte stack
    /// temporary plus a copy of it every 250 Hz pass, and the copy was never atomic anyway (34
    /// words cannot be written in one bus transaction on a Cortex-M3), so the SWD reader sees
    /// exactly what it saw before: the same words at the same offsets, ascending, with a torn read
    /// across fields still possible and still acceptable for diagnostics.
    ///
    /// Each write goes through `addr_of_mut!((*p).field)`, so the OFFSETS still come from the
    /// struct and the `const` assertion block above still owns the layout; nothing here restates
    /// where a field sits.
    ///
    /// `pads_configured` is the one fact in the block the pipeline cannot know: b0 = pad A
    /// resolved to a pin this boot, b1 = pad B did. It is the boot plan's answer
    /// ([`InputPins`]), and it is what separates a quiet staged pad from an absent one in word 37.
    fn publish_obs(
        o: &Obs,
        boot_count: u32,
        period_live: bool,
        pads_configured: u8,
        staged: GuestStaged,
    ) {
        // SAFETY: the one writer (main thread), fixed symbol, raw pointer so no reference to the
        // `static mut` is formed; every target is an initialized-by-construction field of the
        // block and volatile so the reads an SWD session makes see the stores.
        unsafe {
            let p = addr_of_mut!(CTRL_OBS) as *mut CtrlObs;
            addr_of_mut!((*p).magic).write_volatile(CTRL_OBS_MAGIC);
            addr_of_mut!((*p).boot_count).write_volatile(boot_count);
            addr_of_mut!((*p).tick_count).write_volatile(TICK_COUNT.load(Ordering::Relaxed));
            addr_of_mut!((*p).dispatch_count)
                .write_volatile(DISPATCH_COUNT.load(Ordering::Relaxed));
            addr_of_mut!((*p).control_ticks).write_volatile(o.control_ticks);
            addr_of_mut!((*p).input_ticks).write_volatile(o.input_ticks);
            addr_of_mut!((*p).pitch_milli).write_volatile(o.pitch_milli_deg);
            addr_of_mut!((*p).cyclic_age).write_volatile(o.cyclic_age);
            addr_of_mut!((*p).drive_age).write_volatile(o.drive_age);
            addr_of_mut!((*p).enact_inits).write_volatile(o.enact_inits);
            addr_of_mut!((*p).enact_shutdowns).write_volatile(o.enact_shutdowns);
            addr_of_mut!((*p).torque).write_volatile(o.torque_setpoint);
            addr_of_mut!((*p).mode_byte).write_volatile(o.mode_byte);
            addr_of_mut!((*p).moe_bits).write_volatile(o.moe_bits);
            addr_of_mut!((*p).sub_state).write_volatile(o.sub_state);
            addr_of_mut!((*p).control_mode).write_volatile(o.control_mode);
            addr_of_mut!((*p).flags).write_volatile(o.flags_byte());
            addr_of_mut!((*p).event_levels).write_volatile(o.event_levels);
            addr_of_mut!((*p).link_line_errors)
                .write_volatile(LINK_LINE_ERRORS.load(Ordering::Relaxed));
            addr_of_mut!((*p).link_lap_overruns)
                .write_volatile(LINK_LAP_OVERRUNS.load(Ordering::Relaxed));
            addr_of_mut!((*p).systick_isr_entries)
                .write_volatile(runtime_hal::irq::SYSTICK_ISR_METRIC.entries());
            addr_of_mut!((*p).usart1_isr_entries)
                .write_volatile(runtime_hal::irq::USART1_RX_ISR_METRIC.entries());
            addr_of_mut!((*p).dma_isr_entries)
                .write_volatile(runtime_hal::irq::DMA_RX_ISR_METRIC.entries());
            addr_of_mut!((*p).roll_milli).write_volatile(o.roll_milli_deg);
            addr_of_mut!((*p).motor_periods).write_volatile(motor::PERIODS.load(Ordering::Relaxed));
            addr_of_mut!((*p).motor_state).write_volatile(
                motor::OBS_STATE.load(Ordering::Relaxed)
                    | if period_live {
                        motor::OBS_PERIOD_LIVE << 24
                    } else {
                        0
                    },
            );
            addr_of_mut!((*p).motor_duty01)
                .write_volatile(motor::OBS_DUTY01.load(Ordering::Relaxed));
            addr_of_mut!((*p).motor_duty2_angle)
                .write_volatile(motor::OBS_DUTY2_ANGLE.load(Ordering::Relaxed));
            addr_of_mut!((*p).motor_fault).write_volatile(
                motor::FAULT.load(Ordering::Relaxed)
                    | (motor::INVALID_DWELL.load(Ordering::Relaxed).min(0xFFFF) << 16),
            );
            addr_of_mut!((*p).motor_speed).write_volatile(motor::SPEED.load(Ordering::Relaxed));
            addr_of_mut!((*p).motor_cal).write_volatile(motor::OBS_CAL.load(Ordering::Relaxed));
            addr_of_mut!((*p).gating_field).write_volatile(o.gating_field);
            addr_of_mut!((*p).pre_env_torque).write_volatile(o.pre_env_torque);
            addr_of_mut!((*p).event_counts).write_volatile(o.event_counts);
            addr_of_mut!((*p).ble_rx_losses).write_volatile(BLE_RX_LOSSES.load(Ordering::Relaxed));
            addr_of_mut!((*p).stack_margin).write_volatile(sample_stack_margin());
            addr_of_mut!((*p).motor_current)
                .write_volatile(motor::OBS_CURRENT.load(Ordering::Relaxed));
            addr_of_mut!((*p).battery).write_volatile(o.battery as u16 as u32);
            addr_of_mut!((*p).arm_refusals).write_volatile(arm::hw::refusal_obs());
            addr_of_mut!((*p).mailbox_drops).write_volatile(MAILBOX_DROPS.load(Ordering::Relaxed));
            addr_of_mut!((*p).tx_timeouts).write_volatile(TX_TIMEOUTS.load(Ordering::Relaxed));
            addr_of_mut!((*p).cyclic_age_peak).write_volatile(o.cyclic_age_peak);
            addr_of_mut!((*p).pad_obs).write_volatile(
                (o.pad_field & 0b11) as u32
                    | ((o.rider_present as u32) << 2)
                    | (((pads_configured & 0b11) as u32) << 4)
                    | ((o.pad_changes as u32) << 8)
                    | ((o.pad_held as u32) << 16),
            );
            addr_of_mut!((*p).reset_cause).write_volatile(RESET_CAUSE.load(Ordering::Relaxed));
            addr_of_mut!((*p).motor_rate_q).write_volatile(motor::RATE_Q.load(Ordering::Relaxed));
            addr_of_mut!((*p).ble_staged).write_volatile(staged.ble);
            addr_of_mut!((*p).mailbox_staged).write_volatile(staged.mailbox);
            addr_of_mut!((*p).wheel_pos_own).write_volatile(o.wheel_pos_own);
            addr_of_mut!((*p).wheel_pos_peer).write_volatile(o.wheel_pos_peer);
            addr_of_mut!((*p).term_p).write_volatile(o.sf_terms[0]);
            addr_of_mut!((*p).term_r).write_volatile(o.sf_terms[1]);
            addr_of_mut!((*p).term_x).write_volatile(o.sf_terms[2]);
            addr_of_mut!((*p).term_v).write_volatile(o.sf_terms[3]);
            addr_of_mut!((*p).v_ref_q).write_volatile(o.v_ref_q);
            addr_of_mut!((*p).e_x).write_volatile(o.e_x);
            addr_of_mut!((*p).foc_phase_mean)
                .write_volatile(motor::OBS_FOC_PHASE.load(Ordering::Relaxed));
            addr_of_mut!((*p).foc_dq_mean)
                .write_volatile(motor::OBS_FOC_DQ.load(Ordering::Relaxed));
            addr_of_mut!((*p).foc_ref_vq)
                .write_volatile(motor::OBS_FOC_REF.load(Ordering::Relaxed));
        }
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
        // The ticks that ACTUALLY elapsed since the last control run, and ZERO is the normal
        // reading on a CATCH-UP RUN. The scheduler accrues missed slots in its `runcount` and
        // `dispatch()` drains them one per main-loop pass (`crates/scheduler`), so after the main
        // thread has been away this callback runs several times inside one SysTick tick: measured
        // on the offroad pair 2026-10-22, bursts of up to 294 runs at 13,889 runs/s. Those runs
        // are ONE service opportunity, and every consumer here that measures time is gated on it
        // (`specs/link-control.md`, "What a tick of the ages is"). It used to be `.max(1)`, which
        // told the pipeline a tick had passed when none had.
        let dt_ticks = now_tick.wrapping_sub(shell.last_control_tick);
        shell.last_control_tick = now_tick;
        let served = dt_ticks > 0;
        // The IMU-loss retry breaker (specs/integration.md pipeline step 1): once the breaker is
        // open (>= IMU_LOSS_THRESHOLD consecutive failed reads) attempt the blocking read only on
        // the probe cadence, so a failing read is not burned every 4 ms tick. A skipped read hands
        // the pipeline `None` (still lost); a probe-tick success clears the streak and closes the
        // breaker.
        //
        // The breaker rations the cost; it does not bound it, and it cannot: the first
        // IMU_LOSS_THRESHOLD passes after a bus goes bad all pay in full, before it opens. What
        // bounds the cost is the HAL's per-acquisition poll budget, pinned against the motor coast
        // by IMU_READ_BLOCK_FITS_COAST below.
        // `served` first: the IMU is a once-per-tick sample, so a catch-up run does not read it.
        // That is correctness and not only cost: the pipeline's health breaker counts a sample-less
        // call as a MISS, so a burst of reads-that-cannot-be would either storm the bus or storm
        // the breaker. It is also the cost, measured against the absences this is the tail of: a
        // 294-run burst was 294 polled I2C acquisitions in the window the main thread had already
        // overrun.
        let sample = if served && shell.orch.imu_read_due(now_tick) {
            match (&mut shell.i2c, &mut shell.imu) {
                (Some(bus), Some(dev)) => dev.read(bus).ok(),
                _ => None,
            }
        } else {
            None
        };
        // The period-liveness observation and the motor-side fault level, folded BEFORE the pass
        // that consumes them: `fault_a` is assembled inside `control_task`, so a level written
        // after it would act a tick late. How far the 16 kHz ISR advanced per elapsed tick, then
        // the hysteresis latch, then the level the mode machine sees.
        //
        // A catch-up run measures nothing: `dt_ticks` is its divisor, so the rate is undefined
        // (and the division would fault) when no tick has elapsed. `last_periods` is left where
        // it is, so the next SERVED run measures the whole span across the burst rather than a
        // fragment of it, and the hysteresis latch sees one sample per tick as it was designed
        // to.
        let motor_state = motor::OBS_STATE.load(Ordering::Relaxed);
        let motor_configured = motor::obs_configured(motor_state);
        // What the torque word means (`specs/foc-torque.md` section 4): a current while the
        // period ISR runs FOC, so the balance law stops dividing it by the battery. Off the same
        // load as `motor_configured`, so the two cannot describe different words; the byte is the
        // method the ISR runs, after the demotion and the arm-time install.
        shell.orch.block.torque_is_current = motor::obs_torque_is_current(motor_state);
        // The liveness supervisor's premise is a RUNNING counter, not merely a configured motor
        // (`specs/motor-integration.md`, slice 5). The bring-up starts it and nothing stops it
        // again, so the supervisor watches the ISR for the whole of a powered life, armed or not:
        // the counter is the hall-edge measurement's clock, and the OFF interlock needs it turning
        // while the board is disarmed ("The OFF interlock must read a LIVE speed").
        let motor_running = motor_configured && motor::COUNTER_RUNNING.load(Ordering::Relaxed);
        if served {
            let periods = motor::PERIODS.load(Ordering::Relaxed);
            let per_tick = periods.wrapping_sub(shell.last_periods) / dt_ticks;
            shell.last_periods = periods;
            shell.period_live = motor::periods_live(per_tick);
            shell.period_health.update(motor_running, shell.period_live);
        }
        shell.orch.motor_fault = motor::motor_fault_level(
            motor_configured,
            motor::FAULT.load(Ordering::Relaxed),
            shell.period_health.loss(),
            arm::hw::refused(),
        );
        // The hard over-current trip: the ISR counts trips, and a change since the last run raises
        // motor 0's fault latch through its seam. The latch consumes the code this same pass and
        // reaches `fault_a` as `latch_a`, so it is NOT folded into the motor-side level above.
        let trips = motor::OVER_CURRENT_TRIPS.load(Ordering::Relaxed);
        if trips != shell.last_trips {
            shell.last_trips = trips;
            shell.orch.raise_over_current(0);
        }
        // The period ISR's speed cell, read ONCE and feeding both of its consumers, so the
        // OFF-inhibit level and the block row cannot disagree about what the wheel did this tick:
        //
        //   - the OFF-inhibit producer, live at slice 5: a turning wheel holds the machine in OFF,
        //     and so does a wheel nothing is measuring. It is handed this tick's liveness
        //     observation with the word, because the word is the ISR's and means nothing about NOW
        //     while the ISR is not running (`specs/motor-integration.md`, "The OFF interlock must
        //     read a LIVE speed"). The block row below is handed the raw word either way: it is
        //     telemetry and a reading of the last closed window is what it has to report, and its
        //     own consumers run only in RUN, which an unmeasurable speed no longer lets a board
        //     reach;
        //   - the control block's wheel-speed word (`specs/link-control.md`, "`wheel_speed` carries
        //     the raw edge count"), which is this board's ONLY writer of that row. It is folded in
        //     here, the `motor_fault` / `vbatt_raw` pattern, because the row's consumers all run
        //     INSIDE the pass below: the step-off latch's `b_motion`, the engagement blend's
        //     `ref_34` and the cyclic payload. A word written
        //     after the pass would act a tick late.
        //
        // Motor 0's row only: motor 1 has no producer anywhere in this image (the single-motor
        // rule, `crate::motor`'s module header), so its row stays at the constructor's 0.
        let speed = motor::SPEED.load(Ordering::Relaxed);
        shell.orch.motor_moving = arm::off_inhibit(motor_running, shell.period_live, speed);
        shell.orch.block.wheel_speed[0] = motor::wheel_speed_word(speed);
        // The period-based rate beside it (`specs/control.md` (o)): the state-feedback law's speed
        // measurement, already in the law's own 1/4096-count unit, so there is nothing to narrow
        // and nothing to convert. One load, like the count's.
        shell.orch.block.wheel_rate_q = motor::RATE_Q.load(Ordering::Relaxed);
        // The signed position beside it (`specs/state-feedback.md` section 3): the law folds it,
        // the cyclic carries its low half to the peer and the observation publishes it mirrored.
        // One load.
        shell.orch.block.wheel_pos = motor::POS_EDGES.load(Ordering::Relaxed);
        // The battery-sense count the period ISR stored (rank 2 of the injected group, already
        // 12-bit), handed over BEFORE the pass whose source rule converts and filters it
        // (`specs/sensing-and-safety.md`, "The battery word"). Loaded every tick on every board;
        // only a sensing board consumes it, and it stays 0 until the group converts.
        shell.orch.vbatt_raw = motor::VBATT_RAW.load(Ordering::Relaxed) as u16;
        // The last current window the period ISR closed, for the cyclic payload's appended block
        // (`specs/link-control.md`, the `CYCLIC_STATE` layout), unpacked through the inverse of
        // the ISR's own packs, the `vbatt_raw` pattern. Telemetry only: nothing in the pass below
        // reads it, and the limiter that DOES act on these currents acts in the ISR, on the
        // samples themselves.
        //
        // The mean and the duty come from ONE load, because they share a cell for exactly that
        // reason. The peak comes from `OBS_CURRENT`, which the ISR stores from the SAME
        // `take_window` close, so the peak and the mean are the same window's by construction
        // rather than by luck; what is not atomic is the pair of LOADS, so a window boundary
        // falling between them can hand the mean of one window beside the peak of its neighbour.
        // That 4 ms skew is the one every pair of these observations carries (speed, battery and
        // the fault word are read the same way), and it is bounded by being a window apart: the
        // alternative, publishing the peak a second time beside the mean, would put one
        // measurement in two cells.
        let mean_duty = motor::OBS_MEAN_DUTY.load(Ordering::Relaxed);
        shell.orch.phase = orchestrator::PhaseWindow {
            peak: motor::unpack_peak(motor::OBS_CURRENT.load(Ordering::Relaxed)),
            mean: motor::unpack_mean(mean_duty),
            duty_on: motor::unpack_duty_on(mean_duty),
        };
        // The odometer the cyclic payload carries (`specs/odometer.md`): the LIVE total, folded
        // by the service loop, which is the context that owns it and the only one that may write
        // it to flash. Copied here beside the current window for the same reason, that the
        // payload is built inside the pass below.
        shell.orch.odometer = ODOMETER_TOTAL.load(Ordering::Relaxed);
        let out = control_task(&mut shell.orch, sample.as_ref(), dt_ticks);
        shell.cyclic_out = cyclic_tx(&shell.orch, shell.addressed);
        // The guest ports' own decimation (5 Hz) and the peer's state at its own 2.5 Hz, only
        // while fresh. Latest-wins: a payload the loop has not picked up yet is simply
        // overwritten, never queued, so a slow module cannot build a backlog.
        shell.guest.fill(
            ble_cyclic_tx(&shell.orch, shell.addressed),
            ble_peer_tx(&shell.orch, shell.addressed),
        );
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
        // The arm path's first step is the value-row re-read, as a closure over this shell (see
        // `re_read_arm_values`). The other arguments are copied into locals FIRST, so the closure's
        // mutable borrow of `shell` is the only one live across the call.
        let motor_fault = shell.orch.motor_fault;
        let moe = out.moe[0];
        // The end-of-engage seam, as the orchestrator published it (`off_request_clear`: this pass
        // resolved to OFF and the power request is clear). The arm layer's retryable re-read
        // refusal is released there (`specs/integration.md`, "A refused re-read refuses the ARM,
        // not the boot"); the fault latches and the engagement machine clear on the OFF half of the
        // same pass, inside `control_task`. Taken, never recomputed here: the mode and the request
        // are both the orchestrator's facts.
        arm::hw::enact(
            moe,
            motor_configured,
            motor_fault,
            out.off_request_clear,
            || re_read_arm_values(shell),
        );
        publish_obs(
            &obs,
            shell.boot_count,
            shell.period_live,
            shell.inputs.pads_configured(),
            shell.guest_staged,
        );
    }

    /// The 16 ms input task (scheduler slot 1): sample the configured input pins against the
    /// board's staged polarity (`store::BOARD_INPUT_POLARITY`, whose default is the button
    /// active-low and the pads active-high; unconfigured lines sample idle and are dropped here)
    /// and run the pure input pass.
    fn input_task_cb() {
        // SAFETY: as `control_task_cb`.
        let Some(shell) = (unsafe { (*addr_of_mut!(SHELL)).as_mut() }) else {
            return;
        };
        let mut s = InputSample::default();
        if let Some(g) = shell.inputs.group {
            let a = inputs::asserted(g.read(), shell.inputs.polarity);
            s.button_asserted = shell.inputs.has_button && (a & inputs::BUTTON_LINE) != 0;
            s.pad_a_high = shell.inputs.has_pad_a && (a & inputs::PAD_A_LINE) != 0;
            s.pad_b_high = shell.inputs.has_pad_b && (a & inputs::PAD_B_LINE) != 0;
        }
        input_task(&mut shell.orch, &s);
    }

    /// Build the orchestrator shell into its static (`specs/integration.md` boot delta step 2:
    /// the `ControlDispatch` boot seam rides the [`OrchestratorState`] constructor).
    ///
    /// The live-lane words as the STORE holds them (`specs/state-feedback.md` section 6): the four
    /// `CONTROL_SF_GAINS` indices, then the two `CONTROL_REFERENCE` indices, the
    /// `control::GainShadow` row order.
    ///
    /// The registry's per-index defaults are the inert board (zero gains, a dead stick). Range
    /// enforcement is not here: every value goes into `control::GainShadow`, which clamps on the
    /// way in, so a hand-poked out-of-range flash value cannot enter the law.
    ///
    /// Called at boot and again after any `CONFIG_WRITE` persists (the shadow's reconcile), which
    /// is why it is a free function rather than boot-local.
    fn read_gains<F: store::Flash>(s: &Store<F>) -> [i16; control::TUNED] {
        // Written out rather than looped: a constant index folds `at()` to a constant handle, and
        // the loop form measured +40 B for the same reads.
        [
            s.get(CONTROL_SF_GAINS.at(0)),
            s.get(CONTROL_SF_GAINS.at(1)),
            s.get(CONTROL_SF_GAINS.at(2)),
            s.get(CONTROL_SF_GAINS.at(3)),
            s.get(CONTROL_REFERENCE.at(0)),
            s.get(CONTROL_REFERENCE.at(1)),
        ]
    }

    /// The arm-time re-read's whole apply (`specs/integration.md`, "When a stored value takes
    /// effect: the arm-time re-read"): re-read the value row from flash, derive and validate it, and
    /// install all of it. `Ok(())` only if every applicable apply happened; `Err(cause)` REFUSES
    /// the arm (`arm::ArmStep::ReReadValues`) and carries the `CTRL_OBS` word-33 cause byte
    /// ([`arm::imu_frame_cause_byte`], or `0` for a refusal with no frame cause). Because
    /// `arm::rederive` validates before anything is written, an `Err` has applied nothing.
    ///
    /// The five applies are the five owners of the value row: the period ISR's record
    /// (`motor::hw::install_rederived`), the IMU's axis frame (`imu::Imu::set_config`), the control
    /// section's mode, rider requirement, battery floor and position clamp
    /// (`orchestrator::re_apply_control_values`, which also replaces the balance producer record
    /// when the active mode changed), the turn bound and rate, and the gain shadow's maxima.
    /// The IMU apply is conditional on the DEVICE and the derived config both being
    /// present, and they are present together by construction: the bias handed to `rederive` is
    /// `Some` exactly when the device is, and a successful `rederive` carries `Some` config exactly
    /// when it was given a bias. So there is no applicable apply this can skip.
    ///
    /// `#[inline(never)]` with NO `#[link_section]`, deliberately: it must stay OUT of `.hotcode`,
    /// the F1x0 zero-wait first 32 KiB (`crates/firmware/memory.x`). It runs once per arm rather
    /// than once per pass, so window space spent on it would be taken from the 16 kHz and 250 Hz
    /// paths that need it; and `inline(never)` is what stops the body being inlined into
    /// `control_task_cb`, which IS placed in the window.
    #[inline(never)]
    fn re_read_arm_values(shell: &mut Shell) -> Result<(), u8> {
        // The fresh value row. SAFETY: main-thread context (a dispatch callback), the `STORE`
        // discipline: the borrow ends with this statement and the loop holds none across
        // `dispatch()`.
        let values = match unsafe { (*addr_of_mut!(STORE)).as_ref() } {
            Some(st) => arm::read_arm_values(st),
            None => return Err(0),
        };
        // The bring-up-row decode facts, from the runtime rather than re-read, and the IMU's
        // INSTALLED gyro bias, carried through because `IMU_GYRO_BIAS` is not in the value row.
        let Some(boot) = motor::hw::boot_fixed() else {
            return Err(0);
        };
        let bias = shell.imu.as_ref().map(|d| d.config().gyro_bias);
        // Derive and validate EVERYTHING before the first apply below. A refused axis frame is the
        // one refusal that carries a cause an operator can act on, so it travels to `CTRL_OBS` word
        // 33 as the cause byte; the other three exits describe a board that could not have armed
        // anyway (no store, no runtime), so they refuse with no cause and only step the count.
        let r = match arm::rederive(&values, boot, bias) {
            Ok(r) => r,
            Err(e) => return Err(arm::imu_frame_cause_byte(e)),
        };
        if !motor::hw::install_rederived(&r.motor) {
            return Err(0);
        }
        if let (Some(dev), Some(cfg)) = (shell.imu.as_mut(), r.imu) {
            // Reinstalls the axis permutation; touches no bus.
            dev.set_config(cfg);
        }
        // The control section's four fields, through the orchestrator: a changed producer (a mode
        // or a frame) replaces the balance producer records with the engagement machine, which
        // this layer does not own.
        orchestrator::re_apply_control_values(
            &mut shell.orch,
            r.control_mode_byte,
            r.frame_byte,
            r.side_byte,
            r.rider_required_byte,
            r.battery_floor,
            r.pos_clamp,
        );
        let ctl = &mut shell.orch.ctl;
        ctl.drive_turn = r.drive_turn;
        ctl.gains.re_apply_max(r.gain_max);
        Ok(())
    }

    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget fix): the Shell value
    /// (the orchestrator state is the image's biggest single object) is constructed here and
    /// written into the static, so `main`'s persistent frame never carries the temporary.
    ///
    /// The argument list is the Shell's boot inputs one for one (the orchestrator constructor's
    /// inputs, then the bus, the device, the input pins, the boot ordinal and the detected part);
    /// bundling them into a struct for the lint would be a type with this one caller.
    #[inline(never)]
    #[allow(clippy::too_many_arguments)]
    fn init_shell(
        control_mode_byte: u8,
        frame_byte: u8,
        side_byte: u8,
        level_trim_centideg: [i16; 2],
        gains: control::GainShadow,
        vbatt_cal: Option<orchestrator::battery::VbattCal>,
        pos_clamp: i16,
        drive_turn: control::DriveTurn,
        rider_required_byte: u8,
        battery_floor: i16,
        imu_bus: Option<I2c>,
        imu_dev: Option<imu::Imu>,
        imu_staged: bool,
        inputs: InputPins,
        boot_count: u32,
        chip_tag: ChipTag,
    ) {
        let imu_configured = imu_dev.is_some();
        // SAFETY: single-threaded boot (only the DMA RX ISR is live, and it reaches only the
        // HAL ring); the one initializing write, before any task dispatch exists.
        unsafe {
            *addr_of_mut!(SHELL) = Some(Shell {
                orch: OrchestratorState::new(
                    control_mode_byte,
                    frame_byte,
                    side_byte,
                    imu_configured,
                    imu_staged,
                    attitude::Config::staged(level_trim_centideg),
                    gains,
                    vbatt_cal,
                    pos_clamp,
                    drive_turn,
                    rider_required_byte,
                    battery_floor,
                    // What this board says about itself for the life of the boot
                    // (`specs/link-control.md`, the `CYCLIC_STATE` layout, offsets 17 and 18).
                    // The boot ordinal is narrowed to its low byte HERE, once, rather than on
                    // every emission: the wire carries one byte and the full counter stays on
                    // the shell for `CTRL_OBS`.
                    BoardIdentity {
                        boot_tag: boot_count as u8,
                        chip: chip_tag,
                    },
                ),
                i2c: imu_bus,
                imu: imu_dev,
                inputs,
                cyclic_out: None,
                guest: GuestSlots::new(),
                guest_staged: GuestStaged::default(),
                addressed: false,
                boot_count,
                last_control_tick: 0,
                last_periods: 0,
                period_live: false,
                period_health: motor::PeriodHealth::new(),
                last_trips: 0,
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
    /// the outcome is observable (`imu_configured` in `CTRL_OBS`). Every refusal here is a STAGED
    /// IMU that did not come up, which the flags byte's b6 says and b0 cannot: the four `None`
    /// returns above (an instance the pin model will not resolve, an untyped pin, an unknown model
    /// index, a refused frame) plus a bus that will not init and a probe that does not answer all
    /// publish the same `imu_configured = 0` as a board with no IMU fitted at all.
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
        // And the axis ROLES (IMU_AXIS_ROLE, 0 = UP / 1 = PITCH_RATE, chip axis 1..=3, 0 = unset =
        // the stock flat mount's Z / Y): with the signs they are the signed permutation from chip
        // to body axes, applied to the whole sample in `Imu::decode` (`specs/imu.md`).
        let staged_roles = [
            store.get(IMU_AXIS_ROLE.at(0)),
            store.get(IMU_AXIS_ROLE.at(1)),
        ];
        // A frame that is not a proper rotation with distinct roles is REFUSED here, before the
        // bus is touched: the IMU is not brought up (so Balance demotes to Throttle with the mode
        // fault, the existing fail-soft), and BOARD_OBS names the refusal. The rest of the layout
        // stays in force, so the record is rewritten over the validator's success with the latch
        // pin it already carries.
        let cfg = match imu::Config::staged(staged_sign, bias, staged_roles) {
            Ok(cfg) => cfg,
            Err(e) => {
                // SAFETY: single-threaded boot, interrupts not yet enabled; raw-pointer access
                // only (the BOARD_OBS discipline, `apply_layout`).
                unsafe {
                    let obs = core::ptr::addr_of_mut!(BOARD_OBS);
                    *obs = BoardObs::failure(&imu_frame_refusal(e), (*obs).self_hold);
                }
                return (None, None);
            }
        };
        let Ok(mut bus) = I2c::new(
            chip,
            &CLOCK,
            instance,
            (scl, sda),
            I2cMode::fast(IMU_I2C_HZ, runtime_hal::i2c::FastDuty::Two),
        ) else {
            return (None, None);
        };
        let mut dev = imu::Imu::new(model, cfg);
        if dev.probe(&mut bus).is_ok() && dev.init(&mut bus).is_ok() {
            // The caller-owned post-init settle (specs/imu.md; the imu-bench pause) before the
            // first cyclic read.
            cortex_m::asm::delay((CLOCK.sysclk_hz / 1000) * IMU_SETTLE_MS);
            (Some(bus), Some(dev))
        } else {
            (None, None)
        }
    }

    /// The `BOARD_OBS` refusal for an IMU frame `imu::Config::staged` refused, off the shared
    /// [`arm::imu_frame_fault`] mapping: the same pair the arm refusal's `CTRL_OBS` cause byte is
    /// built from, so the boot record and the bench read cannot name different fields for the same
    /// frame. The mapping lives in `arm.rs`'s pure half because this function cannot be host-tested
    /// (it is inside the target-only `mod firmware`) and that one can.
    fn imu_frame_refusal(e: imu::FrameError) -> board::BoardError {
        let (field, first) = arm::imu_frame_fault(e);
        board::BoardError {
            field: board::FieldRef { field, motor: None },
            kind: board::BoardErrorKind::ImuFrame(first),
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
    /// Returned WITH the plan: whether the staging ASKED for an IMU
    /// (`crates/board/src/lib.rs`, `Validated::imu_staged`), which the boot publishes as the
    /// `CTRL_OBS` flags byte b6. It cannot be derived from the plan on the way out: a refused
    /// layout carries no plan at all, and that is one of the cases the bit has to tell apart from
    /// a board with no IMU fitted. It rides out of the validator rather than off a second store
    /// read so the ask the boot publishes is the ask the validator judged.
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
    ) -> (Option<board::BoardPlan>, bool) {
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
        (plan, v.imu_staged)
    }

    /// Route one delivered-but-unhandled PDU (the `net` hand-back): the reserved control block
    /// `0x10..0x2F` decodes through `linkctl` into the orchestrator inbox, the two TUNE opcodes go
    /// to the live gain shadow, and everything else stays dropped (integration.md, "The
    /// delivered-PDU hand-back").
    ///
    /// `out` takes the tune reply, so the caller must route emissions AFTER this rather than
    /// before it.
    fn route_handback(handed: Option<net::DeliveredPdu>, responder: &Responder, out: &mut Emits) {
        let Some(d) = handed else { return };
        if d.opcode == net::OP_TUNE_WRITE || d.opcode == net::OP_TUNE_READ {
            route_tune(&d, responder, out);
            return;
        }
        if !(0x10..=0x2F).contains(&d.opcode) {
            return;
        }
        let Some(payload) = linkctl::decode(d.opcode, &d.payload) else {
            return;
        };
        // SAFETY: main-thread context (the loop's drain), same discipline as the callbacks.
        if let Some(shell) = unsafe { (*addr_of_mut!(SHELL)).as_mut() } {
            // The sender's L3 address goes in with the payload: the inbox scopes mirror liveness
            // to the node that armed the board, so a second controller's traffic cannot hold a
            // rider's arm alive (`LinkInbox::refreshes_mirror`).
            shell.orch.inbox.accept(d.src, payload);
        }
    }

    /// Serve one `TUNE_READ` / `TUNE_WRITE` against the live gain shadow and reply `CONFIG_RESP`
    /// (`specs/rider-ui.md` section 4, the live tune lane).
    ///
    /// This is the whole reason the lane is a separate opcode pair rather than an armed-goes-to-RAM
    /// mode of `CONFIG_WRITE`: the values it moves live HERE, in the control block's RAM, and no
    /// flash path is reachable from this function at all. So it is exempt from the armed refusal by
    /// construction, and `net`'s "no flash program or erase while armed, no exceptions" is left
    /// exactly as audited.
    ///
    /// The split of judgement: `net::decode_tune` reads what the wire can judge (type tag, value
    /// width), and `control::GainShadow` judges what the model owns (which keys are tunable, and
    /// each one's range). Neither knows the other's rules.
    fn route_tune(d: &net::DeliveredPdu, responder: &Responder, out: &mut Emits) {
        let Some((key, op)) = net::decode_tune(d.opcode, &d.payload) else {
            return;
        };
        // SAFETY: main-thread context (the loop's drain), the same discipline as the hand-back's
        // inbox write above. The 250 Hz control pass READS this shadow (once per balance pass); a
        // row written across two passes is transiently mixed, which is bounded by the ranges and
        // accepted for tuning (`specs/rider-ui.md`, "Cross-thread and mid-loop discipline"). No
        // torque step can come of it: in RUN the law ramps its live gains toward the shadow under
        // fixed per-pass caps (`control::config::sf::RAMP_CAPS`), which the control tests pin.
        let Some(shell) = (unsafe { (*addr_of_mut!(SHELL)).as_mut() }) else {
            return;
        };
        let gains = &mut shell.orch.ctl.gains;
        let (status, value) = match op {
            net::TuneOp::Refuse(status) => (status, None),
            net::TuneOp::Read => match gains.get(key.field_id, key.index) {
                Some(v) => (net::walk::CFG_OK, Some(v)),
                None => (net::walk::CFG_UNKNOWN_KEY, None),
            },
            net::TuneOp::Write(v) => match gains.set(key.field_id, key.index, v) {
                Ok(()) => (net::walk::CFG_OK, Some(v)),
                Err(control::TuneError::UnknownKey) => (net::walk::CFG_UNKNOWN_KEY, None),
                Err(control::TuneError::OutOfRange) => (net::walk::CFG_BAD, None),
            },
        };
        responder.reply_tune(d.src, key, status, value, out);
    }

    /// Re-read the stored gains into the live shadow after a `CONFIG_WRITE` persisted
    /// (`specs/rider-ui.md` section 4: a disarmed `CONFIG_WRITE` to a live-lane field also
    /// refreshes the shadow, so flash and RAM converge on save without a reboot; the fields are
    /// 0x79 and 0x7B, `specs/state-feedback.md` section 6).
    ///
    /// Called only on the responder's persist signal, so the store scan is paid on a config write
    /// and never in the steady loop. The shadow takes only the gains whose STORED value moved, so
    /// saving one field cannot revert a gain the tune lane is holding live.
    fn reconcile_gains<F: store::Flash>(responder: &mut Responder, store: &Store<F>) {
        if !responder.take_config_persisted() {
            return;
        }
        let fresh = read_gains(store);
        // SAFETY: main-thread context, as `route_tune` above.
        if let Some(shell) = unsafe { (*addr_of_mut!(SHELL)).as_mut() } {
            shell.orch.ctl.gains.reconcile(fresh);
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
    // serials"): SplitSerial<RingBufferedRx> for the inter-board UART, and the BLE module on a
    // `PolledSerial` for bring-up only, adopted onto a `SplitSerial<RingBufferedRx>` of its own
    // once interrupts exist (`specs/ble.md`, "The RX carrier"), which is the carrier the steady
    // state runs on. There is no firmware-local serial wrapper: the AT probe and the AT handshake
    // run on the same `PolledSerial` (`crate::ble_bringup`), which is what keeps one `drain_ack`
    // body in the image.
    // ---------------------------------------------------------------------------------------------

    /// The SWD diagnostic block ([`crate::ble_bringup::BleProbeObs`]). A `static mut` (not a
    /// functional static) so it keeps the fixed, un-mangled symbol `BLE_PROBE_OBS` the evaluator
    /// reads over SWD; written only by the one `attach` call below, once per boot, before interrupts
    /// are enabled. Accessed via a raw pointer (never a reference to the `static mut`, per the
    /// `static_mut_refs` lint), so it is single-writer and sound.
    #[no_mangle]
    static mut BLE_PROBE_OBS: BleProbeObs = BleProbeObs::NEW;

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
    /// docs hold the whole rule). It is borrowed BYTES and stays borrowed only for this call: it
    /// goes into `AT+NAME=` and nothing built here keeps it.
    ///
    /// It **fails soft**, exactly as [`bring_up_imu`] does: an optional peripheral that is absent,
    /// silent or wedged returns `None`, the board boots without it, and the outcome is observable
    /// (`BLE_PROBE_OBS`, and the BLE port simply not existing in `PORTS`). Nothing here can hold the
    /// boot: the whole window is bounded by `ble_bringup::BOOT_BUDGET_MS`, const-asserted against the
    /// `ble` crate's own pacing.
    ///
    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget fix): the serial, the
    /// probe tee, and the AT bring-up's working set live here and are gone before the loop's deep
    /// chains exist.
    /// Returns the data-mode `Pipe` over the POLLED port plus the USART it was resolved to, NOT a
    /// finished link: the port's steady-state carrier is a DMA ring, and a ring cannot be armed here
    /// because this phase runs before the RAM vector table is installed and interrupts are enabled.
    /// Phase 2 adopts it ([`crate::ble_carrier::adopt`]) once those exist. The resolved instance
    /// travels with the pipe because it is what selects the ring's DMA channel and vectors, and
    /// re-deriving it later would mean running the whole port resolution a second time.
    #[inline(never)]
    fn bring_up_ble(
        chip: &runtime_hal::Chip,
        delay: &mut Delay,
        entry: &SafeLinkUsart,
        configured: bool,
        name: &[u8],
    ) -> Option<(ble::Pipe<PolledSerial>, PeriphLabel)> {
        let ble_usart = usart_of(chip, entry.pins)?;
        let (Ok(tx), Ok(rx)) = (chip.pin(entry.pins[0]), chip.pin(entry.pins[1])) else {
            return None;
        };
        let serial = PolledSerial::new(chip, &CLOCK, ble_usart, (tx, rx), BT_BAUD).ok()?;

        // The bring-up itself is medium-agnostic (`crate::ble_bringup`): everything from the settle
        // to the data-mode gate is generic over the serial, so the bounded-budget/soft-fail contract
        // is host-tested against a module that never answers rather than asserted from the bench.
        // SAFETY: single-threaded boot, interrupts not yet enabled, and this is the one write site
        // of the block this boot; via a raw pointer, so no reference to the `static mut` outlives it.
        let obs = unsafe { &mut *core::ptr::addr_of_mut!(BLE_PROBE_OBS) };
        ble_bringup::attach(serial, delay, configured, name, obs)
            .pipe()
            .map(|p| (p, ble_usart))
    }

    // The three concrete L2 links (heterogeneous serials, one L2 code path each). Each link's
    // frame-scratch const (`F`) is its transport's framer capacity, NOT the 255 B protocol
    // ceiling: the scratch lives on the deep drain/send stack chains (round-7a reclaim, ~360 B
    // across poll_recv + send on the 8 KiB parts) and the carrier can never emit a larger frame.
    type MailboxLink =
        Link<SerialTransport<MailboxSerial, MAILBOX_FRAMER_N>, PACKET, MAILBOX_FRAMER_N>;
    // The inter-board and BLE ports are both on DMA BOTH ways (`runtime-hal/specs/uart-tx-dma.md`,
    // `specs/ble-port-dma-tx.md`): a ring receiving and a one-shot transmitting, so a frame costs
    // the main loop a copy instead of its wire time (0.43 ms at 460800, ~21 ms at the module's 9600).
    // Which carrier a port holds is runtime state inside `SplitSerial` (`TxHalf`), so the TYPE is the
    // same one both links always had and this whole generic stack is instantiated once for the two.
    type UartLink =
        Link<SerialTransport<SplitSerial<RingBufferedRx>, UART_FRAMER_N>, PACKET, UART_FRAMER_N>;
    // The BLE link rides the data-mode gate type (`ble::Pipe`, specs/ble.md): a link can only be
    // built on a serial KNOWN to be in transparent data mode (handshake arm: `bring_up`; fallback
    // arm: `Pipe::assume_data_mode` from the persisted link-set knowledge). Its carrier is the DMA
    // ring the port is adopted into after the AT phase (`crate::ble_carrier`), so the BLE and
    // inter-board links now share one receiver shape and differ only in framer capacity.
    type BleLink = Link<
        SerialTransport<ble::Pipe<crate::ble_carrier::BleSerial>, BLE_FRAMER_N>,
        PACKET,
        BLE_FRAMER_N,
    >;

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

        // Paint the free stack, SECOND: as early as anything can run while still leaving the FET
        // backstop above it, and before the first thing in this function with a frame worth
        // measuring (`Mailbox::from_raw`, `detect_chip`, `clock::configure_tree`).
        //
        // WHERE IT CANNOT GO. Not before `debug_hold_timer0`, which is the halt backstop and stays
        // unconditionally first. Not in `#[pre_init]`, which runs before cortex-m-rt has
        // initialized `.data`/`.bss`, so the paint would be measuring a stack that the `.bss` zero
        // loop then runs over from below. And the FLOOR is `_stack_end`, not `__ebss`: cortex-m-rt
        // places `.uninit` between them, `CTRL_OBS` lives there (ELF-measured when it was the whole
        // of it: `__ebss` 0x2000_0DFC, `__euninit`/`_stack_end` 0x2000_0E74, exactly the block's
        // 120 B; `SHELL` and `MOTOR` joined it in the 2026-10 shrink, each written `None` before any
        // reader), and a paint from `__ebss` would overwrite
        // its magic and its reset-surviving boot counter with a
        // pattern. That failure looks like a cold boot on every reset and like nothing in
        // particular at the bench, which is precisely the class of corruption this instrument must
        // not introduce.
        //
        // COST, priced off the emitted loop rather than estimated. The paint compiles to 3
        // instructions per word (`str` post-indexed, `subs`, `bne`), so <= 6 core cycles per word,
        // and the core is still on the 8 MHz reset IRC here because `clock::configure_tree` has not
        // run yet. The painted region is 661 words (see `paint_stack`), so <= 3,966 cycles =
        // **<= 496 us**, once, on the boot path. Against `ble_bringup::BOOT_BUDGET_MS` (6,000 ms,
        // const-asserted) that is 0.008%, and it is spent on EVERY boot including the fast one.
        //
        // Painting after `configure_tree` would cost 9x less (72 MHz rather than 8 MHz) and is
        // deliberately not done: it would leave the detect and clock-tree frames unmeasured, and
        // detect runs the bus-fault probe, to save 0.44 ms out of 6,000.
        let painted_words = paint_stack();
        STACK_PAINTED_WORDS.store(painted_words, Ordering::Relaxed);
        // Nothing observed yet: the whole painted region is intact until a sweep says otherwise.
        STACK_MARK.store(painted_words, Ordering::Relaxed);

        // `SHELL`, `FLASH` and `STORE` live in `.uninit` (see the statics): give each the `None`
        // its initializer names before any reader can exist. `write`, not `=`, so the reset
        // garbage is never dropped.
        // SAFETY: single-threaded boot, interrupts not yet enabled, no reference formed.
        unsafe { addr_of_mut!(SHELL).write(None) };
        // SAFETY: as above.
        unsafe { addr_of_mut!(FLASH).write(None) };
        // SAFETY: as above.
        unsafe { addr_of_mut!(STORE).write(None) };

        // Initialize the SWD mailbox header FIRST, before any bridge could attach. SAFETY: REGION_LEN
        // bytes at the fixed reserved base, owned only here, accessed only through the handle.
        let mailbox = unsafe { Mailbox::from_raw(MAILBOX_BASE as *mut u8) };
        mailbox.init_header();

        // Detect the silicon (fail loud: a wrong register layout is worse than a halt).
        let chip = detect_chip().unwrap();

        // Why this boot happened, read and then CLEARED, as early as the RCU base is known.
        //
        // The RCU `RSTSCK` cause flags are sticky: they accumulate until something writes RSTFC,
        // and nothing in this image ever did, so the register has been reporting every reset since
        // the part was last powered. A read of it on 2026-10-23 showed power-on and software both
        // set, which is a history and not an answer. Read-then-clear makes each boot's word name
        // that boot's reset and only that one.
        //
        // The position is the earliest one that can exist: the base comes out of the detect
        // result's address table (one owner for where RCU is), and nothing between reset and here
        // touches `RSTSCK` -- the detect probe writes the GPIO clock-enable registers, and the
        // IRC40K enable that shares this register is part of the watchdog bring-up, much later.
        //
        // No `Err` arm: a chip whose descriptor carries no RCU base cannot have its clock tree
        // configured either, and `configure_tree` below halts on exactly that. One instrument does
        // not get to be the second thing deciding what an impossible chip does.
        if let Ok(rcu_base) = chip.rcu_base() {
            RESET_CAUSE.store(runtime_hal::reset_cause_flags(rcu_base), Ordering::Relaxed);
            runtime_hal::clear_reset_cause(rcu_base);
        }
        // `mcu` is the intrinsic chip-FAMILY tag the board reports in `NODE_HELLO` (NOT a role: family
        // != master/slave). Identity is positional - the walk assigns addresses by where a board sits,
        // never by reading a hardware id - so this tag is informational only.
        let mcu = match chip.clock() {
            ClockPath::F10xRcc => MCU_F10X,
            ClockPath::F1x0Rcu => MCU_F1X0,
        };
        // The PART this board publishes in its cyclic payload (`linkctl::ChipTag`), from the two
        // facts the detect probe MEASURED: the family discriminator and the per-instance
        // advanced-timer count. Unlike `mcu` above it is not informational: a controller uses it
        // to pick the capability table that predicts this board's layout verdict, which is why
        // the app has had to ask an operator which part it is talking to
        // (`specs/rider-ui.md` 3.5). Read once, here, where the detect result is.
        let chip_tag = ChipTag::from_detected(
            matches!(chip.clock(), ClockPath::F10xRcc),
            chip.descriptor().adv_timers,
        );

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
        //
        // Both the adapter and the mounted store go into statics (see `STORE`): the arm path in the
        // 250 Hz callback has to reach flash and that callback has no argument to reach it through.
        // `main`'s own borrow below is a `&'static mut` taken once, so every boot read spells
        // `store.get(..)` exactly as it did when `store` was a local.
        // SAFETY: the one write of `FLASH`, on the boot thread, before any other reader of it
        // exists (the mount below is the first, through the reference taken from it).
        unsafe { *addr_of_mut!(FLASH) = Some(FmcFlash::new(&chip)) };
        // SAFETY: the one `&mut` formed from `FLASH`, on the boot thread; it is moved into the
        // store below, which is then the only path to the adapter for the rest of the run.
        let flash: &'static mut FmcFlash = match unsafe { (*addr_of_mut!(FLASH)).as_mut() } {
            Some(f) => f,
            None => halt(),
        };
        // `.unwrap()`, as before: a mount that fails is a torn store region whose compaction could
        // not write, and this boot has nothing to run on.
        // SAFETY: the one write of `STORE`, on the boot thread, before the loop or any task
        // callback exists.
        unsafe { *addr_of_mut!(STORE) = Some(Store::mount(flash).unwrap()) };
        // The boot's single borrow. Its last use is before `service_loop` is entered, so it is dead
        // by the time the loop and the callbacks take their own scoped borrows.
        // SAFETY: as the write above; the boot thread is the only context that exists here.
        let store: &'static mut Store<'static, FmcFlash> =
            match unsafe { (*addr_of_mut!(STORE)).as_mut() } {
                Some(s) => s,
                None => halt(),
            };
        let link_set = store.get(LINK_SET);
        let configured = link_set != 0;
        // The odometer's stored total, motor 0's (`specs/odometer.md`). Read here with the other
        // boot-read fields and handed to the loop, which owns every later read and the one write.
        let odometer_base = store.get(store::MOTOR_ODOMETER.at(0));

        // The safe-link allowlist with this chip's routability answers filled in: the one input the
        // layout validation and the port assignment share.
        let allowlist = allowlist(&chip);

        // Apply the persisted layout (a popped boot frame: see `apply_layout`). Its FIRST act is
        // the SELF_HOLD assert, on the pin the store stages: the board's own power rail goes up the
        // moment its pin is knowable, before the whole-layout validation that must not be able to
        // withhold it. On the bench both boards run on debugger 3V3 that bypasses the latch, so
        // that assert is a no-op for power there and is verifiable only as the pin's `GPIOx.OCTL`
        // bit; it matters on battery + the inter-board cable.
        let (plan, imu_staged) = apply_layout(&chip, store, &allowlist, link_set);

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
        // Phase 1 yields the data-mode pipe over the POLLED port; the link is built in phase 2, after
        // the vector table exists, once the port has been adopted into its DMA-ring carrier.
        let ble_ready: Option<(ble::Pipe<PolledSerial>, PeriphLabel)> = ble_entry.and_then(|e| {
            bring_up_ble(
                &chip,
                &mut delay,
                e,
                configured,
                ble_name::advertised(store),
            )
        });
        // Deviation-1 observability (whether the BLE Link came up) is recorded by `attach` itself,
        // as it returns: it is the outcome of the bring-up, so it is written where the outcome is
        // known rather than re-derived here from the caller's `Option`.

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
        // here, before any interrupt exists. The `RamVectorTable`'s VTOR alignment rides the
        // type.
        // SAFETY: the one formation of a &mut to this static, single-threaded boot.
        let vectors: &'static mut RamVectorTable = unsafe { &mut *addr_of_mut!(RAM_VECTORS) };
        // VTOR requires the table aligned to its power-of-two granule, which `RamVectorTable`
        // carries as its own alignment (256 B for 64 slots) and memory.x's `.ramtables` packing
        // preserves. Guard it against the type's alignment rather than a number written here, so
        // the HAL resizing its table cannot leave this check stating the old granule: a misplaced
        // table is a silent boot brick (VTOR ignores the low bits), so fail loud here instead.
        if !(vectors.slots.as_ptr() as usize)
            .is_multiple_of(core::mem::align_of::<RamVectorTable>())
        {
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
        // The transmit half of the same port, on its own DMA channel (Ch6 on the F103, Ch3 on the
        // F130, where it shares the receive channel's grouped vector). Armed here, beside the ring
        // and after the same `install` + `enable`, because arming it unmasks a vector too.
        //
        // SAFETY: as DMA_RING above, the one &mut formation of this static, before its DMA IRQ
        // exists.
        let tx_buf: &'static mut [u8] = unsafe { &mut *addr_of_mut!(UART_TX_DMA) };
        // `halt()` on failure, matching the ring two statements up: the only failure reachable here
        // is the channel's write-back self-check, on the same DMA0 the ring just armed, and a board
        // whose inter-board transmit silently did not come up has no way to say so.
        let tx_dma = match DmaTx::new(&chip, usart1_tx, tx_buf, CLOCK.sysclk_hz, LINK_BAUD) {
            Ok(t) => t,
            Err(_) => halt(),
        };
        let mut uart_link: Option<UartLink> = if want_uart {
            Some(Link::new(SerialTransport::new(
                SplitSerial::new(tx_dma, rx_dma, CLOCK.sysclk_hz),
                UART_FRAME_CAP,
            )))
        } else {
            None
        };

        // Adopt the BLE port into its steady-state DMA-ring carrier, HERE and not in phase 1: arming
        // a ring registers and unmasks its vectors, so it has to follow the `install` + `enable`
        // above, and it sits beside the inter-board ring because the two are the same act on
        // different ports (`crate::ble_carrier`, which holds the measurement that put it here).
        //
        // SAFETY: as DMA_RING above, the one &mut formation of this static, before its DMA IRQ exists.
        let ble_ring: &'static mut [u8; crate::ble_carrier::RING_CAP] =
            unsafe { &mut *addr_of_mut!(BLE_DMA_RING) };
        // SAFETY: as the ring above: the one `&mut` ever formed from `BLE_TX_DMA`, moved into the
        // BLE port's transmitter for the life of the image.
        let ble_tx_buf: &'static mut [u8] = unsafe { &mut *addr_of_mut!(BLE_TX_DMA) };
        let mut ble_link: Option<BleLink> = ble_ready.map(|(pipe, instance)| {
            // `halt()` on failure, matching the inter-board ring two statements up rather than the
            // soft-fail the BLE PORT gets. Soft-fail answers "is a module present", which was already
            // answered yes by the AT handshake; the only failure reachable here is the DMA channel's
            // write-back self-checks (the ring's and the transmitter's), on the same DMA0 the
            // inter-board ring and transmitter just armed successfully.
            // A controller that fails it after that is broken silicon, not an absent module, and
            // dropping the port silently would leave a board that probed a module and then has no
            // BLE port with nothing to say why.
            let carrier = match crate::ble_carrier::adopt(
                &chip,
                pipe.into_inner(),
                instance,
                ble_ring,
                ble_tx_buf,
                CLOCK.sysclk_hz,
            ) {
                Ok(c) => c,
                Err(_) => halt(),
            };
            // The data-mode gate is re-asserted, not re-negotiated: the handshake that established
            // transparent data mode just completed on this very port, and adopting the receiver
            // changed how bytes are collected, never the module's mode. This is the knowledge arm of
            // `assume_data_mode`, the same one the persisted-link-set path uses.
            Link::new(SerialTransport::new(
                ble::Pipe::assume_data_mode(carrier),
                BLE_FRAME_CAP,
            ))
        });

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

        // The ports this board actually HAS this boot. The BLE slot is registered only if its link
        // came up (`ble_bringup::net_ports`, which holds the reasoning): a dead module leaves no
        // port for the forwarder to flood at and none for `PORTS` to report.
        let mut responder = Responder::new(
            ble_bringup::net_ports(ble_link.is_some()),
            [PORT_SWD, PORT_UART, PORT_BLE, 0],
            mcu,
            NODE_CLASS.default(),
            FW_VER,
        );
        responder.restore_addr(store);

        // === The integration boot delta (specs/integration.md, after the existing bring-up) ===

        // 1. IMU bring-up on the bus the staged layout named (the first BoardPlan consumer): I2C0
        //    on PB6/PB7 for the standard family, I2C1 on PB10/PB11 for the classywalk offroad
        //    family, from this one image. The port assignment above already guaranteed no link
        //    port took those pins, so nothing can be driving them. Fails soft: the board boots
        //    link-only-plus-throttle and the outcome is observable (imu_configured in CTRL_OBS,
        //    read against imu_staged: the flags byte's b0 and b6).
        let (imu_bus, imu_dev) = bring_up_imu(&chip, store, plan.as_ref());

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
                polarity: store.get(store::BOARD_INPUT_POLARITY),
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
            Some(m) => match motor::bring_up(
                &chip,
                m,
                plan.as_ref().and_then(|p| p.vbatt),
                store.get(store::MOTOR_METHOD),
                PERIOD_HZ,
                store.get(store::MOTOR_CURRENT_LIMIT),
                store.get(store::MOTOR_NOISE_FLOOR.at(0)),
            ) {
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
        //    (CONTROL_MODE + CONTROL_FRAME bytes + the IMU fact, the resolution table of
        //    `specs/rover-one-imu.md` section 2: pivoting Balance without an IMU demotes to
        //    Throttle with the mode fault; rigid Balance leads with an IMU and follows without
        //    one). The shell static is built BEFORE the tick source exists, so no dispatch can
        //    see it half-made; the enabled DMA RX ISR never touches it. Built in a POPPED frame
        //    (`init_shell`): the ~700 B Shell value otherwise materializes in `main`'s
        //    persistent frame before the static write (the slice-7 stack-budget fix).
        // The per-board LEVEL TRIMS (ATTITUDE_LEVEL_TRIM, indices 0 = pitch / 1 = roll,
        // centidegrees) ride in with the control mode: the angle this board reads while physically
        // level, subtracted from the published attitude so the balance loop's zero is the board's
        // own level. Per board because the two halves of a chassis are mirror-mounted, which is
        // what stock's per-board cal page absorbed and the only thing its two images differed by.
        // Default 0 = untrimmed, so an unstaged board behaves exactly as before.
        init_shell(
            store.get(CONTROL_MODE),
            store.get(CONTROL_FRAME),
            // Which wheel of a rigid machine this board drives (CONTROL_SIDE, 0 = left): the
            // control block's `role_right` word at last, which the rigid leader's steering split
            // and the state-feedback law's side mirror read. Default 0 (left), the state the app's
            // picker shows as unset: on a right wheel it reverses the law's position and speed
            // terms, which is what gate 1 of `specs/state-feedback.md` reads disarmed.
            store.get(CONTROL_SIDE),
            [
                store.get(ATTITUDE_LEVEL_TRIM.at(0)),
                store.get(ATTITUDE_LEVEL_TRIM.at(1)),
            ],
            // The gain maxima (CONTROL_GAIN_MAX, 0 = gp / 1 = gr / 2 = gx / 3 = gv) bound the
            // shadow until the next arm re-reads them: the boot clamp, the reconcile clamp and the
            // tune lane's refusal.
            control::GainShadow::of_stored(
                read_gains(store),
                [
                    store.get(CONTROL_GAIN_MAX.at(0)),
                    store.get(CONTROL_GAIN_MAX.at(1)),
                    store.get(CONTROL_GAIN_MAX.at(2)),
                    store.get(CONTROL_GAIN_MAX.at(3)),
                ],
            ),
            // The battery-sense calibration (BOARD_VBATT_CAL, 0 = slope / 1 = offset), boot-read
            // and seam-clamped by `VbattCal::new`, on a board whose plan carries `board.vbatt`
            // only: a board that does not sense has no calibration to read.
            plan.as_ref().and_then(|p| p.vbatt).map(|_| {
                orchestrator::battery::VbattCal::new(
                    store.get(BOARD_VBATT_CAL.at(0)),
                    store.get(BOARD_VBATT_CAL.at(1)),
                )
            }),
            // The state-feedback law's position clamp (CONTROL_POS_CLAMP, edges), boot-read and
            // seam-clamped by `control::sf::pos_clamp` in the control section. Default 45.
            store.get(CONTROL_POS_CLAMP),
            // The rigid frame's turn bound and rate (CONTROL_DRIVE_TURN, 0 = turn_max / 1 =
            // turn_slew, torque counts), boot-read and seam-clamped by `DriveTurn::new`. Default
            // turn_max 0 = disabled: the machine drives straight.
            control::DriveTurn::new(
                store.get(CONTROL_DRIVE_TURN.at(0)),
                store.get(CONTROL_DRIVE_TURN.at(1)),
            ),
            // Whether balance mode requires a rider (CONTROL_RIDER_REQUIRED, `specs/control.md`
            // (i)): default 1 = required, so an unstaged board engages exactly as before.
            store.get(CONTROL_RIDER_REQUIRED),
            // The low-battery floor (CONTROL_BATTERY_FLOOR, centivolts, `<= 0` = none,
            // `specs/sensing-and-safety.md`): refuses a balance engage below it, never a disengage.
            store.get(CONTROL_BATTERY_FLOOR),
            imu_bus,
            imu_dev,
            // Whether the staging asked for an IMU at all, from the validator above: published as
            // the `CTRL_OBS` flags byte b6, which is what makes `imu_configured = 0` readable.
            imu_staged,
            inputs,
            next_boot_count(),
            chip_tag,
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
            &mut wdg,
            discovered,
            configured,
            odometer_base,
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
        wdg: &mut FreeWatchdog,
        discovered: u8,
        configured: bool,
        odometer_base: u32,
    ) -> ! {
        // The tick captured when the current `PROBE_PORTS` started (rising edge of `probing()`);
        // `None` when no probe is in flight. The poll window is measured from it (deviation 2).
        let mut probe_start: Option<u32> = None;
        let mut link_set_saved = configured; // once assigned, persist LINK_SET once
                                             // The odometer's running total and its write decision (`specs/odometer.md`). It lives
                                             // here because the write belongs here: the loop is the main thread, it already samples
                                             // the arm fact, and it is where the OTHER deferred persist waits for a disarmed pass.
        let mut odo = odometer::Odometer::new(odometer_base, motor::EDGES.load(Ordering::Relaxed));
        // The BLE port's outbound frame stream, and the one owner every write to it goes through
        // (`crate::ble_wire`): the cyclic samples AND the responder's port-directed emissions, so
        // fragments of one PDU go out in order with nothing between them.
        let mut ble_tx = MeteredTx::<BLE_FRAMER_N, BLE_EMIT_CAP>::new();
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
                // The store comes from the static, in a borrow scoped to this statement (see
                // `STORE`): ingest is where a `CONFIG_WRITE` reaches flash, and it must not hold a
                // borrow across `dispatch()` below. `None` is unreachable after boot (the static is
                // written before the loop is entered); matching rather than unwrapping keeps the
                // impossible case out of the panic machinery.
                // SAFETY: main-thread context, the `SHELL` discipline.
                let handed = match unsafe { (*addr_of_mut!(STORE)).as_mut() } {
                    Some(st) => responder.ingest(PORT_IDX_MAILBOX, &pdu[..n], st, &mut emits),
                    None => None,
                };
                route_handback(handed, responder, &mut emits);
                route_emits(&emits, mailbox_link, uart_link, ble_link, &mut ble_tx);
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
                // The store comes from the static, in a borrow scoped to this statement (see
                // `STORE`): ingest is where a `CONFIG_WRITE` reaches flash, and it must not hold a
                // borrow across `dispatch()` below. `None` is unreachable after boot (the static is
                // written before the loop is entered); matching rather than unwrapping keeps the
                // impossible case out of the panic machinery.
                // SAFETY: main-thread context, the `SHELL` discipline.
                let handed = match unsafe { (*addr_of_mut!(STORE)).as_mut() } {
                    Some(st) => responder.ingest(PORT_IDX_UART, &pdu[..n], st, &mut emits),
                    None => None,
                };
                route_handback(handed, responder, &mut emits);
                route_emits(&emits, mailbox_link, uart_link, ble_link, &mut ble_tx);
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
                // The TX half of the same read-through-the-link: the abandoned writes, low half
                // (`TX_TIMEOUTS`). The BLE port's are the high half, sampled at its own drain.
                TX_TIMEOUTS.fetch_and(0xFFFF_0000, Ordering::Relaxed);
                TX_TIMEOUTS.fetch_or(s.tx_timeouts() as u32, Ordering::Relaxed);
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
                // The store comes from the static, in a borrow scoped to this statement (see
                // `STORE`): ingest is where a `CONFIG_WRITE` reaches flash, and it must not hold a
                // borrow across `dispatch()` below. `None` is unreachable after boot (the static is
                // written before the loop is entered); matching rather than unwrapping keeps the
                // impossible case out of the panic machinery.
                // SAFETY: main-thread context, the `SHELL` discipline.
                let handed = match unsafe { (*addr_of_mut!(STORE)).as_mut() } {
                    Some(st) => responder.ingest(PORT_IDX_BLE, &pdu[..n], st, &mut emits),
                    None => None,
                };
                route_handback(handed, responder, &mut emits);
                route_emits(&emits, mailbox_link, uart_link, ble_link, &mut ble_tx);
                true
            });
            // Sample the BLE port's two recovered-loss counters into the OBS crossing, the same
            // read-through-the-link the inter-board port gets above, one layer deeper: the `Pipe`
            // is the BLE link's carrier and it borrows the `SplitSerial` underneath. Sampled AFTER
            // the drain, so a loss the drain just absorbed is in this pass's number rather than the
            // next one's. Packed `lap_overruns | line_errors << 16` ([`BLE_RX_LOSSES`]).
            if let Some(l) = ble_link.as_ref() {
                let s = l.transport().serial().serial();
                BLE_RX_LOSSES.store(
                    s.lap_overruns() as u32 | ((s.line_errors() as u32) << 16),
                    Ordering::Relaxed,
                );
                TX_TIMEOUTS.fetch_and(0x0000_FFFF, Ordering::Relaxed);
                TX_TIMEOUTS.fetch_or((s.tx_timeouts() as u32) << 16, Ordering::Relaxed);
            }

            // 2d. If any of the three drains PERSISTED a config write, re-read the fields the
            //     control block shadows (`specs/rider-ui.md` section 4). Once per pass rather than
            //     per drain, on a level the responder raises, so the store scan is paid on a config
            //     write and never in the steady loop.
            // SAFETY: main-thread context; the borrow ends with this statement, well before
            // `dispatch()` (the `STORE` discipline).
            if let Some(st) = unsafe { (*addr_of_mut!(STORE)).as_ref() } {
                reconcile_gains(responder, st);
            }

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
                route_emits(&emits, mailbox_link, uart_link, ble_link, &mut ble_tx);
            }

            // 4. R4: sample the arm fact into the responder each pass (integration.md; the mode
            //    machine's any_moe_allowed IS the system's arm definition) and refresh the
            //    address fact for the cyclic gate. A scoped shell borrow: it MUST end before
            //    dispatch() below (the callbacks take their own).
            //    The odometer's two inputs come from the same borrow and the same instant, so
            //    the write decision below cannot mix an arm fact from one pass with a motion fact
            //    from another: `motor_moving` is `arm::off_inhibit`, the OFF interlock's own
            //    predicate, computed in the 250 Hz pass (`specs/odometer.md`).
            let (armed, at_rest) = {
                // SAFETY: main-thread context; the borrow ends at the block's close.
                match unsafe { (*addr_of_mut!(SHELL)).as_mut() } {
                    Some(shell) => {
                        shell.addressed = responder.addr() != net::pdu::NO_ADDRESS;
                        (shell.orch.mode.any_moe_allowed(), !shell.orch.motor_moving)
                    }
                    None => (false, false),
                }
            };
            responder.set_armed(armed);

            // 5. Persist LINK_SET once, at assignment (specs/l3.md: "Once assigned it persists the
            //    set of ports that came up live") - DEFERRED while armed (integration.md R4: no
            //    flash program while armed; the persist-once latch waits for a disarmed pass).
            if !link_set_saved && !armed && responder.addr() != net::pdu::NO_ADDRESS {
                // SAFETY: main-thread context, as the ingest borrows above.
                if let Some(st) = unsafe { (*addr_of_mut!(STORE)).as_mut() } {
                    let _ = st.set(LINK_SET, discovered);
                }
                link_set_saved = true;
            }

            // 5b. The odometer (`specs/odometer.md`). Fold the ISR's edge count into the running
            //     total every pass, publish it for the cyclic payload and for an SWD read, and
            //     persist it when the decision says to: disarmed, wheel stopped, a ride since the
            //     last write, and something actually moved.
            //
            //     `due` is the ONLY thing that decides, and it answers `None` whenever `armed` for
            //     every other input (host-tested exhaustively), which is how an internal writer
            //     honours the rule the L3 `CONFIG_WRITE` path honours through `CFG_ARMED`. The
            //     store observes no arm state and never will; the caller gates it
            //     (`specs/storage-layer.md`, "Who enforces writes only while disarmed").
            //
            //     A `Full` is answered with a compaction and one retry, the `CONFIG_WRITE`
            //     pattern. It is safe HERE for the same reason it is safe there, and for one
            //     more: the board is disarmed AND its wheel has stopped, so the 60 ms repack
            //     never falls on a turning wheel. The attempt is recorded either way, so a region
            //     that is genuinely out of space is retried once per ride, not once per pass.
            odo.fold(motor::EDGES.load(Ordering::Relaxed), armed);
            ODOMETER_TOTAL.store(odo.total(), Ordering::Relaxed);
            if let Some(total) = odo.due(armed, at_rest) {
                // SAFETY: main-thread context, as the ingest borrows above.
                let stored = match unsafe { (*addr_of_mut!(STORE)).as_mut() } {
                    Some(st) => {
                        let key = store::MOTOR_ODOMETER.at(0);
                        match st.set(key, total) {
                            Ok(()) => true,
                            Err(store::StoreError::Full) => {
                                st.compact().is_ok() && st.set(key, total).is_ok()
                            }
                            Err(_) => false,
                        }
                    }
                    None => false,
                };
                odo.settled(total, stored);
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
                let mut payload = [0u8; CyclicState::ENCODED_LEN];
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

            // 8b. The SAME payload to the BLE port, decimated to 5 Hz by `ble_cyclic_tx` and handed
            //     to the port's DMA transmit a WHOLE frame at a time (never `Link::send`).
            //
            //     Why not a send: the module's UART is 9600 baud, so the sample's two frames are
            //     41.7 ms of wire time (20 B + 20 B; `orchestrator::BLE_CYCLIC_DIVISOR` carries
            //     the arithmetic). A blocking send would hold this loop for it, and the 16 kHz ISR
            //     floats all three phases if the 250 Hz control task (dispatched from THIS loop at
            //     step 6, not from an ISR) has not refreshed DEMAND within 16 ms. The port's
            //     transmit is a one-shot DMA channel (`crate::ble_carrier`), and a frame is handed
            //     to it only when `write_ready` says it is idle, so a handover is a copy and a
            //     channel start and the wire time is spent in the background. One visit a pass is
            //     enough: a 20 B frame is ~21 ms of wire, several passes even at the two ticks a
            //     pass the loop runs with a phone attached (`specs/ble-port-dma-tx.md`).
            //
            //     Bounded, never queued: a new payload is picked up only when the owner is idle, and
            //     the own-sample slot is latest-wins, so a slow or absent module drops samples
            //     rather than accruing a backlog.
            //
            //     The port is shared with the responder's port-directed emissions, so both go
            //     through the one owner (`crate::ble_wire::MeteredTx`), which hands the port exactly
            //     one frame at a time and holds an arriving emission in a one-deep slot. An emission
            //     over the port's 15 B chunk (a PORTS reply, a CONFIG_RESP carrying a string) is
            //     several frames, and the slot holds it until all of them are out. `route_emits`
            //     below is the other half of that rule.
            //
            //     Addressing: dst 0x00, the same point-to-point rule as the inter-board emission
            //     (`net::pdu::NO_ADDRESS` is "the one peer on a point-to-point link"). The BLE link
            //     carries exactly one connected central, and a received dst-0x00 PDU is delivered
            //     locally by the peer's forwarder. Deliberately NOT routed through `originate()`:
            //     a dst-0x00 originate matches no route and floods every port.
            //
            //     The port carries a second sample source: the PEER's state, forwarded under the
            //     peer's own address at 2.5 Hz (`specs/link-control.md`, "The peer's state on the
            //     BLE port: the forward"). It waits in its own latest-wins slot and is picked up only
            //     on a pass where the port is idle and the board's own slot is empty, so the own
            //     sample always goes first and the two never contend for the staging buffer: when
            //     both are due on one tick the forward follows the own sample's two frames.
            //
            //     The SWD mailbox port is the second guest port and gets both samples too, the
            //     identical PDUs, in the pass they were built and only while a host is attached
            //     (`specs/link-control.md`, "The mailbox port carries both samples (2026-10-25)").
            //     Its write is all-or-nothing into RAM with no wait, so it needs no owner, adds
            //     nothing to the demand-stale bound, and a full ring drops the sample.
            //     `crate::guest_tx::emit_samples` holds both rules and counts what each port
            //     accepted (`CTRL_OBS` words 40 and 41).
            {
                // SAFETY: main-thread context, after dispatch returned; the borrow ends with this
                // block, and nothing it calls touches the shell.
                if let Some(s) = unsafe { (*addr_of_mut!(SHELL)).as_mut() } {
                    emit_samples(
                        &mut s.guest,
                        responder.addr(),
                        mailbox_link,
                        ble_link.as_mut().map(|l| (l, &mut ble_tx)),
                        &mut s.guest_staged,
                        &mut pdu,
                    );
                }
            }
            if let Some(l) = ble_link.as_mut() {
                // Hand the waiting frame to the port if its channel is idle.
                ble_tx.pump(l);
            }

            // Sample the mailbox port's dropped-frame count into its OBS crossing, the same
            // read-through-the-link the other two ports get above. Sampled HERE, after the last
            // emission site of the pass (the guest samples just above; the drains and the
            // probe-window fire route emissions too), so a frame this pass dropped is in this
            // pass's number. The counter itself is the serial's, saturating at u16::MAX.
            MAILBOX_DROPS.store(
                mailbox_link.transport().serial().refused_writes() as u32,
                Ordering::Relaxed,
            );
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
    ///
    /// The BLE port goes through `ble_tx`, the staged-TX owner, NOT straight at the link. Two
    /// reasons, both in `crate::ble_wire`: a cyclic sample is two fragments over ~40 ms of wire, and
    /// a raw send between them would land in the reassembler as a new set and tear the sample; and
    /// a raw `Link::send` on that 9600-baud port waits on the channel for each frame after the
    /// first, tens of milliseconds from the same pass that dispatches the 250 Hz control task, past
    /// the 16 ms the 16 kHz ISR coasts before it floats all three phases. `ble_tx.send` queues
    /// instead, and step 8b's visit hands it over.
    fn route_emits(
        emits: &Emits,
        mailbox: &mut MailboxLink,
        uart: &mut Option<UartLink>,
        ble: &mut Option<BleLink>,
        ble_tx: &mut MeteredTx<BLE_FRAMER_N, BLE_EMIT_CAP>,
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
                        ble_tx.send(l, &e.bytes);
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
/// **The only fallback is the field's own registered default** (`"Hoverboard"`), and it is the
/// store's rule rather than a firmware one: the store returns the registered default when the
/// record is absent or is the wrong type. So an unstaged board advertises `"Hoverboard"`, and a
/// `CONFIG_READ` of `0x10` returns the exact bytes a scanner sees. There is no second, hidden name
/// the firmware could substitute, which is the whole point of the change.
///
/// **The name is BYTES, not a `&str`.** The board no longer validates a `STR` record as UTF-8
/// (`specs/decision-flash-budget.md`, shrink round 2 item 4), so a record that is not valid UTF-8
/// now advertises verbatim instead of being ignored in favour of the default. That is the whole of
/// the behaviour change here, and it keeps the invariant this owner exists for: whatever
/// `CONFIG_READ 0x10` reports is what goes on air. Nothing downstream wanted text anyway -
/// `ble::Module` writes the bytes straight after `AT+NAME=`.
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
    /// **Why the dynamic `get_value` and not the typed `get_text(DEVICE_NAME)`** (which is
    /// otherwise the right door for a `StrField`, and which this was written with first):
    /// `get_value` is the path `CONFIG_READ` already takes, so it is ALREADY in the image, while
    /// the typed getter is a second STR read site.
    ///
    /// The ORIGINAL reason is GONE and the margin with it. It used to be that `get_str` made LLVM
    /// outline `core::str::from_utf8` into a shared 368 B function instead of the ~230 B
    /// specialised copy inlined into `Value::decode`, which measured the typed expression at
    /// +264 B of flashed span against +40 B for this one. Shrink round 2 item 4
    /// (`specs/decision-flash-budget.md`) removed the validation entirely, so there is no
    /// validator left to outline. Re-measured with that change in (`cargo image` + `objcopy`,
    /// ELF-fresh): `get_text` costs **+20 B** over this form, not +224 B.
    ///
    /// So the form is kept because it is still the smaller of the two, but 20 B is small enough
    /// that this is now a preference rather than a finding: switch to `store.get_text(DEVICE_NAME)`
    /// freely if the typed door reads better, and expect to pay 20 B. Behaviour is identical on
    /// every input either way - both fall back to the field's registered default for an absent or
    /// wrong-type record, and both return a non-UTF-8 record verbatim. The `ble_name` tests pin
    /// that behaviour, not the expression.
    #[allow(dead_code)]
    pub fn advertised<'a, F: store::Flash>(store: &'a store::Store<'_, F>) -> &'a [u8] {
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
        use ble::Module;
        use embedded_hal::delay::DelayNs;
        use embedded_io::{ErrorType, Read, ReadReady, Write};
        use store::{Store, DEVICE_NAME};

        /// The crate's one in-RAM [`Flash`] for host tests (`crate::test_flash`).
        use crate::test_flash::TestFlash;

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
        fn tx_for(name: &[u8]) -> Vec<u8> {
            let stub = StubSerial {
                tx: Vec::new(),
                rx: std::collections::VecDeque::new(),
            };
            let b = Module::new(name)
                .bring_up(stub, &mut NoDelay)
                .expect("the stub acks every command, so bring-up reaches data mode");
            b.pipe.into_inner().tx
        }

        #[test]
        fn an_unstaged_board_advertises_the_registered_default() {
            // Nothing written: the name is DEVICE_NAME's own registered default, which is also
            // exactly what a CONFIG_READ of 0x10 returns. No firmware-side constant is involved.
            let mut flash = TestFlash::erased();
            let store = Store::mount(&mut flash).unwrap();
            assert_eq!(advertised(&store), b"Hoverboard");
        }

        #[test]
        fn a_staged_name_is_what_gets_advertised() {
            let mut flash = TestFlash::erased();
            let mut store = Store::mount(&mut flash).unwrap();
            store.set_str(DEVICE_NAME, "hb-offroad-m").unwrap();
            assert_eq!(advertised(&store), b"hb-offroad-m");
        }

        #[test]
        fn the_newest_staged_name_wins() {
            // The store is an append log: a re-stage (the bench renaming a board) must take, not
            // return the first record written.
            let mut flash = TestFlash::erased();
            let mut store = Store::mount(&mut flash).unwrap();
            store.set_str(DEVICE_NAME, "hb-bench-m").unwrap();
            store.set_str(DEVICE_NAME, "hb-offroad-m").unwrap();
            assert_eq!(advertised(&store), b"hb-offroad-m");
        }

        #[test]
        fn an_explicitly_empty_name_is_carried_verbatim() {
            // The documented decision: empty is a legal staged value, NOT a signal to fall back to
            // the default (which would make CONFIG_READ 0x10 disagree with what is on air).
            let mut flash = TestFlash::erased();
            let mut store = Store::mount(&mut flash).unwrap();
            store.set_str(DEVICE_NAME, "").unwrap();
            assert_eq!(advertised(&store), b"");
            // ...and it reaches the wire as an empty AT+NAME, which is what `name_len = 0` with
            // `answered = 1` reports on the bench.
            let tx = tx_for(advertised(&store));
            assert!(
                tx.windows(12).any(|w| w == b"AT+NAME=\r\nAT"),
                "an empty staged name is sent as a bare AT+NAME= line"
            );
        }

        #[test]
        fn a_name_that_is_not_utf8_is_advertised_verbatim() {
            // The shrink-round-2 behaviour change (`specs/decision-flash-budget.md`, item 4),
            // pinned at the consumer rather than only in the store: the board no longer validates
            // a STR record, so bytes that are not valid UTF-8 reach the air as themselves instead
            // of being ignored in favour of the default. Staged through the dynamic face, which is
            // the only door that can produce such a record (`set_str` takes a `&str`): a
            // configurator writing raw bytes over `CONFIG_WRITE 0x10`, or a half-programmed page.
            let mut flash = TestFlash::erased();
            let mut store = Store::mount(&mut flash).unwrap();
            let raw: &[u8] = &[b'h', 0xFF, b'b'];
            store
                .set_value(DEVICE_NAME.key(), store::Value::Str(raw))
                .unwrap();
            assert_eq!(advertised(&store), raw);
            // ...and it reaches the AT sequence unchanged, rather than the default taking its place.
            let tx = tx_for(advertised(&store));
            assert!(
                tx.windows(13).any(|w| w == b"AT+NAME=h\xFFb\r\n"),
                "the staged bytes are written verbatim after AT+NAME="
            );
            assert!(
                !tx.windows(10).any(|w| w == b"Hoverboard"),
                "the registered default must not have been substituted"
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
                tx.windows(21).any(|w| w == b"AT+NAME=Hoverboard\r\nA"),
                "an unstaged board advertises the registered default"
            );
        }
    }
}

/// The BLE module's boot bring-up: the settle, the patient AT probe with its SWD-readable record,
/// and the decision between a full AT handshake, the configured board's data-mode fallback, and
/// "no module this boot". Everything here is generic over the serial, so it is the same code the
/// image runs and the code the host tests drive against a module that never answers. Only the
/// HAL-touching half (building the `PolledSerial` on the assigned USART, wrapping the returned pipe
/// in the L2 `Link`) stays in the target-only firmware module, as `bring_up_ble`.
///
/// **The contract: an optional peripheral may not hold the boot.** The BLE module is optional in
/// exactly the sense the IMU is (`bring_up_imu`, "Fails soft in every refusal"), and it gets the
/// same treatment:
///
/// - **Bounded.** The whole window is at most [`BOOT_BUDGET_MS`], and that is a CHECKED property:
///   the budget is const-asserted against the `ble` crate's own pacing (`ble::probe_ms` /
///   `ble::bring_up_ms`), so lengthening a drain window or a retry count fails the build instead of
///   quietly lengthening every board's boot. It was not checked before, and the cost was the
///   2026-08-03 offroad session: an AT-deaf module after a dirty power-on reset, the MCU behind it
///   still in phase 1 with no shell, no control stack, no motors and no telemetry, while the module
///   itself kept advertising and accepting GATT on its own power -- which reads from the phone
///   exactly like an app bug.
/// - **Soft-failing.** Absent, silent and wedged all end the same way: [`Attached::Absent`], the
///   board boots without BLE, and nothing further is attempted.
/// - **Observable.** The outcome lands in the same `BLE_PROBE_OBS` block a healthy boot writes
///   (`answered` / `attempts` / `matched_attempt` / `brought_up`), so a bench read tells a module
///   that was never there from one that never answered from one that answered and then went deaf.
/// - **Not retried later.** One bounded attempt per boot, deliberately. A module in transparent data
///   mode cannot be re-probed (re-entry from data mode is not a supported path, `specs/ble.md`), so a
///   retry would have to run the whole AT sequence again; each of its windows blocks for `STEP_MS`,
///   which is 15x the 16 ms the 16 kHz ISR coasts before it floats all three phases, so a retry from
///   the live loop is a torque-path hazard, not a convenience. The recovery for an AT-deaf CC2541 is
///   a power cycle (bench-established), and that resets the MCU too, so the next boot re-probes: the
///   retry already exists, at the only moment it can work.
///
/// Compiled on all targets (outside the `target_os = "none"` firmware module) so the workspace host
/// test run reaches its tests, hence the module-wide `dead_code` allowance: on the host non-test
/// build its callers (the target-only boot path) do not exist.
mod ble_bringup {
    #![allow(dead_code)]

    use ble::{Module, Pipe};
    use embedded_hal::delay::DelayNs;
    use embedded_io::{Read, ReadReady, Write};

    /// Fixed settle before the first `AT`: a freshly cold-power-cycled CC2541 is not UART-ready for the
    /// first few hundred ms, so the first probe would be lost (or land mid-byte). Warm modules already
    /// answer by ~250 ms, so this only delays a cold boot.
    pub const COLD_BOOT_SETTLE_MS: u32 = 500;

    /// AT-probe attempts (each one `ble::probe` attempt: `AT\r\n` + a whole `ble::STEP_MS` ≈ 248 ms
    /// RX-drain window). 16 ≈ a ~4 s patient window AFTER the settle - a cold module's AT-ready time
    /// varies, and a fixed ~750 ms (3 tries) caught it only ~50%. The probe early-exits the instant
    /// `AT+OK` arrives, so a warm/fast module still costs ~one step; only a silent port spends it all.
    ///
    /// This patience is the DISCOVERY budget and it is kept: cutting it is how a slow module gets
    /// misread as absent (unconfigured) or as already-in-data-mode (configured), and neither failure
    /// is observable from the phone. What the budget fix removes is the *redundant* patience below.
    pub const PROBE_ATTEMPTS: u32 = 16;

    /// The AT handshake's own state-0 re-probe, once the probe above has already answered.
    ///
    /// `ble::Module::bring_up` re-probes because it is a standalone API that knows nothing about its
    /// caller; this caller has just seen `AT+OK` within the last window, so the default 20 retries buy
    /// nothing and cost up to ~5 s of blocking boot. They are pure cost in precisely the failure this
    /// module exists for: a dirty-POR module that answers one `AT` and then goes deaf spends the whole
    /// 20 before the boot may continue, and no resend has ever recovered one (it needs a power cycle).
    /// One confirming resend keeps state 0's "advance only on `AT+OK`" contract at the cheapest price.
    pub const HANDSHAKE_PROBE_RETRIES: u32 = 1;

    /// The hard ceiling on how long a BLE module may delay this board's boot, dead or alive.
    ///
    /// Its value is not a wish: it is the settle plus the full discovery patience plus one handshake
    /// pass, rounded up, and the assertion below is what keeps it that. The pre-budget worst case
    /// (the full probe, then 20 more inside `bring_up`) was ~10.5 s and nothing in the build noticed.
    pub const BOOT_BUDGET_MS: u32 = 6_000;

    /// The bound, checked at compile time rather than believed: settle + probe + handshake.
    const _: () = assert!(
        COLD_BOOT_SETTLE_MS
            + ble::probe_ms(PROBE_ATTEMPTS)
            + ble::bring_up_ms(HANDSHAKE_PROBE_RETRIES)
            <= BOOT_BUDGET_MS
    );

    /// The board's `net` port count for this boot: the mailbox (port 0) and the inter-board UART
    /// (port 1) are structural, and the BLE port (port 2, the last slot) is registered **only if its
    /// link exists**.
    ///
    /// A port that never came up is not a port. `specs/l3.md` already says so ("each port that comes
    /// up becomes one of the board's local ports"), but the count handed to the `Responder` was the
    /// compiled maximum, so a board whose module was dead still declared a BLE port: the forwarder
    /// flooded every unrouted frame at it (dropped in `route_emits`, since there is no link to send
    /// on) and `PORTS` reported it to the controller as a port of the board. That is a half-modelled
    /// fact - a port with a kind and an index but no carrier - and the controller cannot tell it from
    /// a live module with no peer connected. Registering the slot only when the link exists is the
    /// whole fix; no other index moves, because BLE is the last slot.
    pub const fn net_ports(ble_up: bool) -> u8 {
        2 + ble_up as u8
    }

    /// SWD-readable AT-probe observation. Read it over SWD at the address of the `BLE_PROBE_OBS` symbol
    /// (`nm <elf> | grep BLE_PROBE_OBS`).
    #[repr(C)]
    pub struct BleProbeObs {
        /// `BleProbeObs::MAGIC` once a probe has written this block (confirms it is live, not stale RAM).
        magic: u32,
        /// AT attempts issued this boot (`== matched_attempt` on success, `== PROBE_ATTEMPTS` on a miss).
        attempts: u32,
        /// The 1-based attempt `AT+OK` arrived on (0 = never). Elapsed-to-`AT+OK` ≈ this × ~248 ms (`STEP_MS`).
        matched_attempt: u32,
        /// 1 = `AT+OK` seen (command mode), 0 = no AT (silent / not-ready / already in data mode).
        answered: u32,
        /// Deviation-1 observability: `1` if the BLE `Link` was built this boot (either arm: the AT
        /// handshake reached transparent data mode, or the configured board took the data-mode
        /// fallback), `0` if it was not (no module, or one that answered the initial `AT` and then
        /// went deaf through the handshake). Written by [`attach`] as it returns; this word sits at
        /// offset `0x10`. Lets the bench distinguish a *correctly* empty `PORTS` BLE port -
        /// module up, no L3 peer connected over the bridge (`specs/l3.md`: `PORTS` reports neighbour
        /// presence, not local link liveness) - from a bring-up that produced no port at all. On a
        /// board with no module the block's `magic` stays `0`, so this word is ignored with the rest.
        brought_up: u32,
        /// Bytes of the store's `DEVICE_NAME` handed to `AT+NAME=` this boot; `0` = **no rename was
        /// attempted** on this boot. Written only in the command-mode arm (immediately before
        /// [`ble::Module::bring_up`]), so it is the bench's evidence that the staged name actually
        /// reached the AT sequence rather than the module keeping whatever name it already had.
        /// Read it WITH `answered`: `answered = 1, name_len = n` is an n-byte name sent this boot;
        /// `answered = 0, name_len = 0` is the data-mode fallback arm, which by design never
        /// re-handshakes and so never renames; `answered = 1, name_len = 0` is a deliberately
        /// staged EMPTY name (a legal store value, sent verbatim). Offset `0x14`.
        name_len: u32,
        /// Bit per `ble::AtStep` (0 = NAME, 1 = CON_INTERVAL, 2 = ADV_INTERVAL, 3 = SET,
        /// 4 = MODE=DATA): that step's `AT+OK` arrived, i.e. the module said the command TOOK.
        /// Offset `0x18`.
        at_acked: u32,
        /// Bit per `ble::AtStep`, same numbering: the module answered `AT+ERR`, i.e. it REFUSED the
        /// command. A step in NEITHER mask answered nothing, which is a third state and not the
        /// same fact (`ble::Ack`). Offset `0x1C`.
        ///
        /// This is what makes a rename verifiable end to end. Read it WITH `name_len`:
        /// `name_len = n` with bit 0 set in `at_acked` is a rename that TOOK; the same `name_len`
        /// with bit 0 set here is a rename the module REFUSED (the board is still advertising its
        /// old name, and the fault is not in the firmware's staging); `name_len = n` with bit 0 in
        /// neither is a rename whose fate the module never stated. Before this existed all three
        /// looked identical, and `name_len` alone only ever proved the bytes were SENT.
        at_refused: u32,
    }

    impl BleProbeObs {
        /// `"BLEP"` little-endian: the live marker.
        const MAGIC: u32 = 0x424C_4550;

        /// The zeroed block a board boots with: `magic = 0` says no probe has written it, so every
        /// other word is stale RAM and must be ignored. The `static mut`'s initializer.
        pub const NEW: BleProbeObs = BleProbeObs {
            magic: 0,
            attempts: 0,
            matched_attempt: 0,
            answered: 0,
            brought_up: 0,
            name_len: 0,
            at_acked: 0,
            at_refused: 0,
        };

        /// Start a fresh boot's probe record.
        fn begin(&mut self) {
            self.magic = Self::MAGIC;
            self.attempts = 0;
            self.matched_attempt = 0;
            self.answered = 0;
            self.brought_up = 0;
            self.name_len = 0;
            self.at_acked = 0;
            self.at_refused = 0;
        }
    }

    /// What phase 1 produced. Three outcomes, kept apart because they are three different facts
    /// about the board (and the middle one is the only reason the wire ever sees a module the
    /// firmware never handshaked this boot).
    pub enum Attached<S> {
        /// The full AT bring-up ran and the module reported itself transparent: a data-mode pipe.
        Handshaked(Pipe<S>),
        /// A CONFIGURED board whose known-BLE port answered no `AT` after the full patient probe: a
        /// warm reset left the module in transparent data mode, still advertising, so the port is
        /// registered live WITHOUT re-handshaking (`specs/l3.md`). Its BLE identity comes from the
        /// link-set, so no AT identification is needed.
        DataMode(Pipe<S>),
        /// No BLE link this boot: nothing answered on an unconfigured board's candidate wiring, or a
        /// configured board's module answered `AT` and then failed the handshake. The board boots
        /// without BLE and no port is registered for it.
        Absent,
    }

    impl<S> Attached<S> {
        /// The data-mode pipe, if a link exists at all. The caller builds its L2 link on this and
        /// treats `None` as "no BLE port this boot"; which of the two live arms produced it is a
        /// bench-diagnosis fact (`answered` in the observation block), not a routing one.
        pub fn pipe(self) -> Option<Pipe<S>> {
            match self {
                Attached::Handshaked(p) | Attached::DataMode(p) => Some(p),
                Attached::Absent => None,
            }
        }

        /// Whether a BLE link came up (the `net` port question, and `brought_up` in the block).
        pub fn is_up(&self) -> bool {
            !matches!(self, Attached::Absent)
        }
    }

    /// The cold-boot-robust AT probe: issue `AT` up to [`PROBE_ATTEMPTS`] times (each is one
    /// `ble::probe` attempt - `AT\r\n` + a whole `ble::STEP_MS` RX-drain window), early-exiting on the
    /// first exact `AT+OK\r\n`. Patient enough to catch a cold-power-cycled module whose AT-ready time
    /// varies, instead of racing a fixed short window. Records the attempt count + matching attempt
    /// into `obs`.
    ///
    /// It probes the SAME serial type the AT handshake then runs on, rather than a wrapper around
    /// it, so `ble::probe`, `ble::bring_up` and the `drain_ack` body underneath them are one
    /// instantiation in the image instead of two.
    fn cold_boot_probe<S, D>(serial: &mut S, delay: &mut D, obs: &mut BleProbeObs) -> bool
    where
        S: Read + Write + ReadReady,
        D: DelayNs,
    {
        for attempt in 1..=PROBE_ATTEMPTS {
            obs.attempts = attempt;
            if ble::probe(serial, delay, 1).unwrap_or(false) {
                obs.matched_attempt = attempt;
                obs.answered = 1;
                return true;
            }
        }
        false
    }

    /// Bring the module up on an already-built serial, within [`BOOT_BUDGET_MS`], recording the
    /// outcome in `obs`. Never fails: every refusal is an [`Attached`] variant.
    ///
    /// `configured` is the board's own knowledge that this port IS a BLE module (its bit is in the
    /// persisted link-set); it is what makes the data-mode fallback safe, and on an unconfigured board
    /// its absence is why a silent port is simply not a module.
    ///
    /// `name` is the advertised BLE name the caller read from the store (`crate::ble_name`, whose docs
    /// hold the whole rule). Borrowed bytes, only for this call: it goes into `AT+NAME=` and nothing
    /// built here keeps it.
    pub fn attach<S, D>(
        mut serial: S,
        delay: &mut D,
        configured: bool,
        name: &[u8],
        obs: &mut BleProbeObs,
    ) -> Attached<S>
    where
        S: Read + Write + ReadReady,
        D: DelayNs,
    {
        obs.begin();

        // Settle: a freshly cold-power-cycled CC2541 is not UART-ready for the first few hundred ms,
        // so the first `AT` would be lost or land mid-byte. Part of the budget, like everything else.
        delay.delay_ms(COLD_BOOT_SETTLE_MS);

        // Patient cold-boot AT probe (retry `AT` until `AT+OK` over a generous window, not a fixed
        // ~750 ms). A CONFIGURED board runs this on EVERY boot, not just the unconfigured discovery:
        // a cold power-cycle resets the CC2541 to command mode, so a BLE-kind port from the link-set
        // must be re-handshaked with the full AT bring-up (`SET=1`) or the module never re-advertises
        // and the board is invisible to the app (l3.md). `cold_boot_probe` only borrows the serial, so
        // it stays usable for the data-mode fallback below (`bring_up` would move + drop it).
        let answered_at = cold_boot_probe(&mut serial, delay, obs);

        let attached = if answered_at {
            // Command mode: full AT bring-up (NAME / intervals / SET=1 -> advertises / MODE=DATA).
            // Transparent data mode after; the link rides the gate type itself. The advertised name
            // is the staged one and only this arm sends it: record its length as the bench's evidence
            // that it reached the AT sequence.
            obs.name_len = name.len() as u32;
            let brought = Module::new(name)
                // The probe above has already proven command mode, so the handshake does not re-run
                // a 20-deep patience budget over it (see `HANDSHAKE_PROBE_RETRIES`).
                .probe_retries(HANDSHAKE_PROBE_RETRIES)
                .bring_up(serial, delay);
            // Published on BOTH arms. A refused `AT+MODE=DATA` is the one fatal case (no pipe at all,
            // `ble::bring_up`) and it is precisely the boot whose record matters most: it is the
            // module stating why there is no BLE link this boot, and it carries every earlier step's
            // answer with it. Publishing only from the success arm would leave both masks 0 exactly
            // there, indistinguishable from a serial error mid-sequence.
            let report = match &brought {
                Ok(b) => Some(b.report),
                Err(ble::Error::ModeRefused { report }) => Some(*report),
                Err(_) => None,
            };
            if let Some(report) = report {
                obs.at_acked = report.acked as u32;
                obs.at_refused = report.refused as u32;
            }
            match brought {
                Ok(b) => Attached::Handshaked(b.pipe),
                // It answered `AT`, so it was in command mode, not data mode: the fallback below
                // would be a claim about the wire that this boot just disproved. No pipe.
                Err(_) => Attached::Absent,
            }
        } else if configured {
            // Data-mode fallback (l3.md): the link-set already identifies this port as the BLE
            // module, but it answered no `AT` even after the FULL patient probe -- a warm reset left
            // it in transparent data mode, still advertising. The patient probe is the prerequisite
            // that makes this safe: an `AT` miss now genuinely means data-mode, not a not-yet-ready
            // cold boot (which the old fixed ~750 ms window misread ~50% of the time, registering a
            // SILENT module live).
            Attached::DataMode(Pipe::assume_data_mode(serial))
        } else {
            // Unconfigured + no AT+OK: not a module. On an unconfigured board that is the normal
            // answer for a wiring this board does not have, and it is why the probe is safe to run
            // at all: nothing but a CC2541 replies `AT+OK`.
            Attached::Absent
        };

        obs.brought_up = attached.is_up() as u32;
        attached
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use embedded_io::ErrorType;
        use net::pdu::{Opcode, Pdu, NO_ADDRESS};
        use net::walk::{Emits, Responder, MAX_PDU, PORT_BLE, PORT_SWD, PORT_UART};
        use std::boxed::Box;
        use std::collections::VecDeque;
        use std::vec;
        use std::vec::Vec;
        use store::Store;

        /// A CC2541 stand-in with a scripted AT personality: `answers(n)` decides whether the n-th
        /// completed `...\r\n` command (1-based, counting every command of the whole bring-up) is
        /// answered with the exact 7-byte `AT+OK\r\n`. Everything else is silence, which is the
        /// failure this module exists for: silence is indistinguishable from absence at the wire, so
        /// it is the boot's job not to care.
        struct ScriptedModule {
            answers: Box<dyn Fn(usize) -> bool>,
            commands: usize,
            rx: VecDeque<u8>,
            tx: Vec<u8>,
            pending: Vec<u8>,
        }

        impl ScriptedModule {
            fn new(answers: impl Fn(usize) -> bool + 'static) -> Self {
                ScriptedModule {
                    answers: Box::new(answers),
                    commands: 0,
                    rx: VecDeque::new(),
                    tx: Vec::new(),
                    pending: Vec::new(),
                }
            }
            /// AT-deaf: the 2026-08-03 offroad module, still advertising, answering nothing.
            fn deaf() -> Self {
                Self::new(|_| false)
            }
            /// Healthy: acks every command.
            fn healthy() -> Self {
                Self::new(|_| true)
            }
        }

        impl ErrorType for ScriptedModule {
            type Error = core::convert::Infallible;
        }
        impl Read for ScriptedModule {
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
        impl ReadReady for ScriptedModule {
            fn read_ready(&mut self) -> Result<bool, Self::Error> {
                Ok(!self.rx.is_empty())
            }
        }
        impl Write for ScriptedModule {
            fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                self.tx.extend_from_slice(buf);
                self.pending.extend_from_slice(buf);
                while let Some(i) = self.pending.windows(2).position(|w| w == b"\r\n") {
                    self.pending.drain(..i + 2);
                    self.commands += 1;
                    if (self.answers)(self.commands) {
                        self.rx.extend(b"AT+OK\r\n");
                    }
                }
                Ok(buf.len())
            }
            fn flush(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }
        }

        /// A delay that runs no clock and simply accounts for the time asked of it: the boot budget
        /// in milliseconds, measured rather than asserted. Every wait in the bring-up goes through a
        /// `DelayNs`, so this totals the whole blocking window.
        struct Budget {
            ns: u64,
        }
        impl Budget {
            fn new() -> Self {
                Budget { ns: 0 }
            }
            fn ms(&self) -> u32 {
                (self.ns / 1_000_000) as u32
            }
        }
        impl DelayNs for Budget {
            fn delay_ns(&mut self, ns: u32) {
                self.ns += ns as u64;
            }
            fn delay_us(&mut self, us: u32) {
                self.ns += us as u64 * 1_000;
            }
            fn delay_ms(&mut self, ms: u32) {
                self.ns += ms as u64 * 1_000_000;
            }
        }

        /// The crate's one in-RAM [`Flash`] for host tests (`crate::test_flash`): the tests here
        /// mount a REAL store behind the responder over it.
        use crate::test_flash::TestFlash;

        /// Phase 1 against a scripted module: the outcome, the observation block, and the milliseconds
        /// of boot it cost.
        fn phase_one(
            module: ScriptedModule,
            configured: bool,
        ) -> (Attached<ScriptedModule>, BleProbeObs, u32) {
            let mut obs = BleProbeObs::NEW;
            let mut budget = Budget::new();
            let attached = attach(module, &mut budget, configured, b"hb-offroad-m", &mut obs);
            (attached, obs, budget.ms())
        }

        #[test]
        fn a_module_that_never_answers_costs_a_bounded_boot_window() {
            let (attached, obs, ms) = phase_one(ScriptedModule::deaf(), false);

            assert!(
                matches!(attached, Attached::Absent),
                "a silent port on an unconfigured board is not a module"
            );
            assert!(
                ms <= BOOT_BUDGET_MS,
                "a dead module delayed the boot {ms} ms, past the {BOOT_BUDGET_MS} ms budget"
            );
            // ...and the budget arithmetic the const assertion is built on is the REAL pacing: the
            // measured window is exactly the settle plus the whole probe patience, so `ble::probe_ms`
            // models what the code does rather than what it was hoped to do.
            assert_eq!(ms, COLD_BOOT_SETTLE_MS + ble::probe_ms(PROBE_ATTEMPTS));

            // The 2026-08-03 signature, now a recorded outcome instead of a boot that never ended:
            // the full attempt budget spent, nothing ever answered, no link.
            assert_eq!(obs.attempts, PROBE_ATTEMPTS);
            assert_eq!(obs.answered, 0);
            assert_eq!(obs.matched_attempt, 0);
            assert_eq!(obs.brought_up, 0);
            assert_eq!(
                obs.magic,
                BleProbeObs::MAGIC,
                "the block is live, not stale RAM"
            );
        }

        #[test]
        fn a_module_that_answers_once_and_goes_deaf_costs_a_bounded_boot_window() {
            // The dirty-POR shape, at its worst: the module wakes just in time to answer the LAST
            // probe attempt, then goes deaf for the handshake. This is the boot that used to spend
            // the whole discovery budget AND a second, longer one inside `Module::bring_up`.
            let (attached, obs, ms) =
                phase_one(ScriptedModule::new(|n| n == PROBE_ATTEMPTS as usize), true);

            assert!(
                matches!(attached, Attached::Absent),
                "a module that answered AT was in command mode, so the data-mode fallback would be \
                 a claim this boot just disproved"
            );
            assert!(
                ms <= BOOT_BUDGET_MS,
                "an AT-deaf module delayed the boot {ms} ms, past the {BOOT_BUDGET_MS} ms budget"
            );
            assert_eq!(obs.answered, 1, "it did answer once");
            assert_eq!(obs.matched_attempt, PROBE_ATTEMPTS);
            assert_eq!(obs.brought_up, 0, "and produced no link");
        }

        #[test]
        fn the_boot_carries_on_without_the_module() {
            // The property the dead module violated: everything AFTER phase 1 still happens. The
            // board that gets no BLE link registers the ports it actually has, and its L3 responder
            // is live on them -- so a controller on the mailbox or the inter-board UART still walks,
            // addresses and configures the board, which is how the bench reaches a board whose
            // module is dead.
            let (attached, _, _) = phase_one(ScriptedModule::deaf(), false);
            assert!(!attached.is_up());

            let ports = net_ports(attached.is_up());
            assert_eq!(
                ports, 2,
                "mailbox + inter-board UART, and no phantom BLE port"
            );

            let mut flash = TestFlash::erased();
            let mut store = Store::mount(&mut flash).unwrap();
            let mut resp = Responder::new(
                ports,
                [PORT_SWD, PORT_UART, PORT_BLE, 0],
                net::walk::MCU_F1X0,
                store::NODE_CLASS.default(),
                0x0001,
            );

            // A controller asks the board to report itself: it answers, over the port it has.
            let mut emits = Emits::new();
            let mut buf = [0u8; MAX_PDU];
            let probe = Pdu::from_op(Opcode::ProbePorts, 0x80, NO_ADDRESS, &[]);
            let n = probe.encode(&mut buf).unwrap();
            resp.ingest(0, &buf[..n], &mut store, &mut emits);
            emits.clear();
            resp.poll_probe(&mut emits);

            let reply = emits
                .iter()
                .map(|e| Pdu::decode(&e.bytes).expect("a PORTS reply decodes"))
                .find(|p| p.opcode == Opcode::Ports.to_u8())
                .expect("the board reports its ports without a BLE module");
            assert_eq!(reply.payload[0], 2, "PORTS reports two ports");
            let kinds: Vec<u8> = reply.payload[1..].chunks(4).map(|entry| entry[1]).collect();
            assert_eq!(kinds, vec![PORT_SWD, PORT_UART]);
            assert!(
                !kinds.contains(&PORT_BLE),
                "a port that never came up is not a port: the controller must not be told the \
                 board has a BLE link it does not have"
            );
        }

        #[test]
        fn a_healthy_module_still_handshakes_and_gets_its_port() {
            let (attached, obs, ms) = phase_one(ScriptedModule::healthy(), true);

            let pipe = match attached {
                Attached::Handshaked(p) => p,
                _ => panic!("a module that acks every command reaches transparent data mode"),
            };
            assert!(ms <= BOOT_BUDGET_MS);
            assert_eq!(obs.answered, 1);
            assert_eq!(
                obs.matched_attempt, 1,
                "a healthy module answers the first AT"
            );
            assert_eq!(obs.brought_up, 1);
            assert_eq!(obs.name_len, "hb-offroad-m".len() as u32);
            assert_eq!(
                obs.at_acked, 0b1_1111,
                "every AT step acked (NAME/CON/ADV/SET/MODE)"
            );
            assert_eq!(
                net_ports(true),
                3,
                "the BLE port is registered when it exists"
            );

            // The staged name reached the wire, through the shortened handshake probe: cutting the
            // redundant retries must not cut a configuration step.
            let tx = pipe.into_inner().tx;
            let at = |needle: &[u8]| tx.windows(needle.len()).position(|w| w == needle);
            assert!(at(b"AT+NAME=hb-offroad-m\r\n").is_some());
            assert!(at(b"AT+SET=1\r\n").is_some());
            assert!(at(b"AT+MODE=DATA\r\n").is_some());
        }

        #[test]
        fn a_configured_board_still_falls_back_to_data_mode() {
            // The l3.md fallback is untouched by the budget: a configured board's known-BLE port that
            // answers nothing is a module a warm reset left transparent, and it stays registered.
            let (attached, obs, ms) = phase_one(ScriptedModule::deaf(), true);
            assert!(matches!(attached, Attached::DataMode(_)));
            assert!(ms <= BOOT_BUDGET_MS);
            assert_eq!(obs.answered, 0);
            assert_eq!(obs.brought_up, 1);
            assert_eq!(
                obs.name_len, 0,
                "the fallback never re-handshakes, so never renames"
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

/// The BLE port's outbound frame stream and its ONE owner (`specs/link-control.md`, "Addressing and
/// emission"; `specs/ble-port-dma-tx.md`).
///
/// The port is a shared resource with two writers: the 5 Hz cyclic telemetry, and the responder's
/// port-directed emissions (a `CONFIG_READ` reply, a `CFG_ARMED` refusal, a walk probe or forward,
/// a `PORTS` response). The module's UART is 9600 8N1, so a byte is 1.042 ms, and one cyclic
/// sample is TWO frames of 20 B, 41.7 ms of WIRE time. That is unavoidable. What is not
/// unavoidable is paying it in CPU time.
///
/// **The bound this exists to hold is 16 ms, and it is not the watchdog.** The 250 Hz control task
/// (IMU sample, balance law, torque word) is dispatched from the MAIN LOOP, the same pass that
/// hands these frames over, and the 16 kHz commutation ISR floats all three phases if `DEMAND` has
/// not been refreshed within `DEMAND_STALE_PERIODS` = 256 periods = 16 ms (`crate::motor`). So a
/// main-loop pass that blocks longer than 16 ms cuts motor torque mid-balance. The 500 ms IWDG is
/// three decimal orders away from being the operative limit. Emissions are NOT gated on armed
/// state (`Responder::ingest` has no armed gate; the R4 gate refuses only the flash persist, and a
/// refusal is itself a reply), so a phone can trigger this path at will against a balancing board.
///
/// So NOTHING here blocks on the wire. The port's transmit is a one-shot DMA channel
/// (`crate::ble_carrier`, `runtime_hal::DmaTx`): a WHOLE wire frame is copied into the channel's
/// buffer and leaves in the background, and the channel is busy until it has. Both writers hand
/// whole PDUs to [`MeteredTx`], which encodes them a frame at a time and hands each frame to the
/// port only when `write_ready` says the channel is idle, so a handover is a copy and a channel
/// start and never a wait. A frame still takes its wire time; the CPU does not.
///
/// It used to put the frames out a BYTE at a time from the main loop, as far as the elapsed time
/// had paid for and the polled transmit register would take. Then the loop's pass rate set the
/// port's byte rate: at the one to two ticks a pass the loop runs with a phone attached, the port
/// moved about 256 B/s against the 285 B/s the two samples needed, and the peer's forward, which
/// only goes out when the port is idle, was superseded in its slot (`specs/ble-port-dma-tx.md`).
/// A frame per handover takes the pass rate out of it: at two ticks a pass a 20 B frame is three
/// passes of wire.
///
/// **Ordering is a correctness requirement, at two scales.** A frame that has begun cannot be
/// abandoned partway, and nothing may be written inside it: the receiver's length-driven framer
/// would consume the other bytes as the body it is still waiting for. The DMA channel takes a
/// frame whole and refuses another until it is done, so that half is the carrier's. Above the
/// frame: a PDU larger than the port's 15 B chunk becomes SEVERAL frames, which must go out in
/// index order with nothing in between, because a `FRAG_IDX` 0 arriving mid-reassembly discards
/// the set in progress (`link::reasm`, atomic-or-discard). So `MeteredTx` holds ONE encoded frame
/// waiting for the port, and the rest of its PDU in a one-deep slot behind it, encoding the next
/// fragment only once the frame ahead has been handed over. Anything arriving meanwhile waits in
/// that slot ([`send`](MeteredTx::send)) or is dropped ([`stage`](MeteredTx::stage)).
///
/// **Emissions are not size-bounded to one frame**, and must not be: a `PORTS` reply is 16 B at this
/// board's three ports, and a `CONFIG_RESP` carrying the device name is 17 to 19 B. Both are over
/// the 15 B chunk, both are real traffic (the BLE walk's `PROBE_PORTS` is answered by the first; the
/// per-board name is read and written over BLE with the second), and a size-based drop would be
/// DETERMINISTIC, which is the one loss the acknowledged plane's retransmit cannot recover: every
/// retry drops identically. So the slot holds the PDU rather than an encoded frame, and
/// `promote` encodes its fragments one at a time as the port takes them.
///
/// **The two writers get opposite overflow policies, because their PDUs are worth different
/// things.** A telemetry sample is one observation of a 5 Hz stream and the next is 200 ms behind
/// it, so a sample arriving while the port is busy is DROPPED and the newer one staged later:
/// queueing it would only put a stale reading on the wire. An emission is a distinct reply that
/// nothing else supersedes (losing a `CONFIG_READ` response costs the controller a retransmit), so
/// it is QUEUED, one deep, and telemetry yields to it. Two deep is where the queue stops: the
/// second emission is dropped and the FIRST kept, because the wire, not the buffer, is the
/// bottleneck (a ~20 ms frame against a few-millisecond pass), so any depth only defers the same
/// drop while adding reply latency, and dropping the older frame would reorder replies against
/// their requests for nothing. What a sample's own fragments do NOT do is hold the slot against a
/// reply: see [`MeteredTx::send`] for why they yield.
///
/// BLE traffic CAN overflow that slot on its own: replies are systematically longer than the
/// requests that ask for them (a `CONFIG_READ` request is 10 wire bytes, its `i32` reply 16 and its
/// string reply two or three frames), so back-to-back reads from the phone outrun the port with no
/// second source involved. What makes the policy hold is not that overflow cannot happen, it is what
/// happens when it does: every op that reaches this port on the acknowledged plane (`specs/l3.md`:
/// `PROBE_PORTS`, `ASSIGN`, `CONFIG_READ`/`CONFIG_WRITE`) is idempotent and retransmitted by the
/// controller on timeout, so an overflow drop costs a retry rather than a reply. That is what
/// separates it from the deterministic size-based drop above, which no retry could ever clear.
///
/// Compiled on all targets (outside the `target_os = "none"` firmware module) so the workspace host
/// test run reaches its tests over a real `Link`/`SerialTransport`; `allow(dead_code)` because its
/// callers are the target-only loop, so the host non-test build (CI clippys with `--all-targets
/// -D warnings`) sees it unused.
mod ble_wire {
    use embedded_io::{Read, ReadReady, Write, WriteReady};
    use link::{Link, SerialTransport};
    use linkctl::CyclicState;
    use orchestrator::dispatch::PeerCyclic;

    /// The frame waiting for the port, and the PDU whose remaining fragments are queued behind it.
    /// `N` is the link's stream-frame size (`BLE_FRAMER_N`), so `buf` holds the largest frame the
    /// port can emit; `Q` is the largest PDU the port can be handed (`BLE_EMIT_CAP`).
    ///
    /// The state lives in the service loop's frame (which is permanent) and is borrowed by the
    /// routing. The queue costs `Q + 2` words, 72 B at `Q` = 64: the price of holding a PDU that has
    /// still to be cut into frames rather than a single frame that is already cut. What it buys is
    /// the delivery of every emission, up to the port's own `net::walk::MAX_PDU` bound, without a
    /// blocking `Link::send`.
    pub struct MeteredTx<const N: usize, const Q: usize> {
        buf: [u8; N],
        /// Length of the encoded frame in `buf` still owed to the port; 0 when none is. A frame here
        /// has consumed its place in the set (and, for the last fragment, the PID), so it goes out
        /// before anything else does.
        len: usize,
        /// The PDU still owed to the wire, as the caller handed it over: unencoded, because a PDU
        /// over the link's 15 B chunk becomes several wire frames and only ONE of them fits `buf`.
        /// Empty (`pending_len == 0`) once the last fragment has moved into `buf`.
        pending: [u8; Q],
        pending_len: usize,
        /// The index of the next fragment of `pending` to encode. The set shares one PID and the
        /// receiver tears a set whose fragments arrive out of order, so this only ever advances.
        pending_frag: usize,
        /// Whether the PDU in the slot is a telemetry SAMPLE ([`stage`](MeteredTx::stage)) rather
        /// than an emission ([`send`](MeteredTx::send)).
        ///
        /// It decides who yields when an emission arrives while a PDU still owes fragments, which
        /// is the common case because the cyclic payload is two fragments on this port: a sample
        /// yields, per the module's overflow rule, and an emission is never dropped for one.
        pending_is_sample: bool,
    }

    impl<const N: usize, const Q: usize> Default for MeteredTx<N, Q> {
        fn default() -> Self {
            Self::new()
        }
    }

    #[allow(dead_code)]
    impl<const N: usize, const Q: usize> MeteredTx<N, Q> {
        pub const fn new() -> Self {
            MeteredTx {
                buf: [0u8; N],
                len: 0,
                pending: [0u8; Q],
                pending_len: 0,
                pending_frag: 0,
                pending_is_sample: false,
            }
        }

        /// Nothing is waiting for the port: the next PDU staged here is the next thing handed to
        /// it. The port itself may still be sending the last frame handed over; that frame is the
        /// channel's now, and the next handover waits for it ([`pump`](MeteredTx::pump)).
        pub fn idle(&self) -> bool {
            !self.framed() && self.pending_len == 0
        }

        /// A frame is encoded and waiting for the port. It owns the next handover: nothing else
        /// may go to the port before it, and it may not be dropped.
        fn framed(&self) -> bool {
            self.len != 0
        }

        /// Take `packet` into the one-deep slot, recording whether it is a telemetry sample.
        /// Refuses a PDU longer than the slot, and an empty one (nothing here emits either: an
        /// `Emission`'s bytes are a `net::walk::PduBuf`, bounded by `Q`, and a PDU always carries
        /// its 3-byte L3 header).
        fn queue(&mut self, packet: &[u8], is_sample: bool) -> bool {
            if packet.is_empty() || packet.len() > Q {
                return false;
            }
            self.pending[..packet.len()].copy_from_slice(packet);
            self.pending_len = packet.len();
            self.pending_frag = 0;
            self.pending_is_sample = is_sample;
            true
        }

        /// Encode the next fragment of the queued PDU into `buf` once the frame ahead of it has
        /// been handed to the port. Called at the head of every entry point, so the next fragment
        /// never waits longer than the frame ahead of it and the slot frees the moment the LAST
        /// fragment is encoded.
        ///
        /// This is where a multi-fragment emission is sequenced: one wire frame at a time, in index
        /// order, with nothing else able to take the port in between (`stage` and `send` both
        /// refuse while `pending_len != 0`). A refusal from the link (a PDU needing more than
        /// `MAX_FRAGMENTS`, which nothing at `Q` = 64 against a 15 B chunk can reach) drops the
        /// whole PDU rather than leaving a half-sent set owed to the wire.
        fn promote<S, const P: usize>(&mut self, link: &mut Link<SerialTransport<S, N>, P, N>)
        where
            S: Read + Write + ReadReady,
        {
            if self.framed() || self.pending_len == 0 {
                return;
            }
            match link.stage_fragment(
                &self.pending[..self.pending_len],
                self.pending_frag,
                &mut self.buf,
            ) {
                Some(s) => {
                    self.len = s.len;
                    if s.more {
                        self.pending_frag += 1;
                    } else {
                        self.pending_len = 0;
                        self.pending_frag = 0;
                    }
                }
                None => {
                    self.pending_len = 0;
                    self.pending_frag = 0;
                }
            }
        }

        /// Stage a telemetry `packet`, to be handed to the port by [`pump`](MeteredTx::pump).
        /// Nothing reaches the port here, and nothing blocks.
        ///
        /// A sample arriving while the owner is busy (a frame waiting for the port, or a PDU still
        /// owed fragments) is DROPPED and reported by the `false` return, never queued: the caller's
        /// source is a latest-wins slot 200 ms from its successor, so queueing this one would only
        /// put a stale reading on a wire that an emission has better use for. A slow or absent
        /// module therefore costs samples rather than accruing a backlog.
        pub fn stage<S, const P: usize>(
            &mut self,
            link: &mut Link<SerialTransport<S, N>, P, N>,
            packet: &[u8],
        ) -> bool
        where
            S: Read + Write + ReadReady,
        {
            self.promote(link);
            if !self.idle() || !self.queue(packet, true) {
                return false;
            }
            self.promote(link);
            self.framed()
        }

        /// Queue `packet` as one whole L2 PACKET on this port, behind whatever is already waiting.
        /// Nothing reaches the port here and nothing blocks: the bytes are encoded through the same
        /// `Link::stage_fragment` seam the telemetry uses (so the framing and the PID stay the
        /// link's), and [`pump`](MeteredTx::pump) hands them over on the following visits, one wire
        /// frame at a time.
        ///
        /// **Any PDU the port can be handed goes out whole**, including the ones that do not fit a
        /// single 16 B frame: a `PORTS` reply is 16 B at this board's three ports and a
        /// `CONFIG_RESP` carrying the device name is 17 to 19 B, both over the 15 B chunk. Those
        /// fragment, and the fragments keep the port to themselves until the set is complete
        /// (`Link::stage_fragment`: a `FRAG_IDX` 0 arriving mid-reassembly discards the set in
        /// progress, so an interleaved frame would destroy the emission, not merely delay it).
        ///
        /// Three cases. The slot is empty: this PDU takes it, behind the frame waiting for the port
        /// if there is one. The slot holds a TELEMETRY SAMPLE's remaining fragments: the sample
        /// yields and this PDU takes the slot, per the module's overflow rule. The slot holds
        /// another EMISSION: this PDU is DROPPED and the one already queued is kept. A drop is
        /// invisible here by design, exactly as `Link::send`'s was: L2 is best-effort and the
        /// acknowledged plane above retransmits.
        ///
        /// **Why a sample yields its remaining fragments and not just its slot.** On this port the
        /// cyclic payload is two frames, so a sample can hold the slot across a frame of wire time.
        /// Refusing an emission for that window would invert the priority this module is built on:
        /// a sample is 200 ms from its successor and worth nothing stale, while a reply costs the
        /// controller a retransmit. The half-delivered sample costs the receiver nothing either:
        /// this PDU's `FRAG_IDX` 0 arrives under a NEW PID, and the reassembler discards a set
        /// whose PID changes mid-way (`link::reasm`, atomic-or-discard), so the torn sample is
        /// dropped there rather than mixed into anything. The frame already ENCODED is never
        /// dropped: it has taken its place in the set and goes out first.
        pub fn send<S, const P: usize>(
            &mut self,
            link: &mut Link<SerialTransport<S, N>, P, N>,
            packet: &[u8],
        ) where
            S: Read + Write + ReadReady,
        {
            self.promote(link);
            if (self.pending_len != 0 && !self.pending_is_sample) || !self.queue(packet, false) {
                return;
            }
            self.promote(link);
        }

        /// One visit to the port: if a frame is waiting and the port's transmit channel is idle,
        /// hand the WHOLE frame over. A busy channel is left alone, so a visit costs a status read
        /// when there is nothing to do and a copy plus a channel start when there is, and never a
        /// wait. The next fragment of the set is encoded by whichever entry point runs next, each
        /// of which promotes first.
        ///
        /// A frame the port did not take whole (a write error, or a carrier shorter than the frame,
        /// which is a sizing bug: the DMA buffer is `N` bytes) abandons the rest of its set
        /// (best-effort, as everywhere on L2: a torn set is discarded by the receiver whole, so
        /// holding the remaining fragments would only occupy the port).
        pub fn pump<S, const P: usize>(&mut self, link: &mut Link<SerialTransport<S, N>, P, N>)
        where
            S: Read + Write + ReadReady + WriteReady,
        {
            self.promote(link);
            if !self.framed() {
                return;
            }
            let serial = link.transport_mut().serial_mut();
            if !matches!(serial.write_ready(), Ok(true)) {
                return;
            }
            let whole = matches!(serial.write(&self.buf[..self.len]), Ok(n) if n == self.len);
            self.len = 0;
            if !whole {
                self.pending_len = 0;
                self.pending_frag = 0;
            }
        }

        /// Stage one `CYCLIC_STATE` sample (the board's own, or the peer's forward) under `src`,
        /// dst 0x00 (the point-to-point rule, `specs/link-control.md`, "Addressing and emission"),
        /// through the one sample encoder both guest ports share ([`crate::guest_tx::sample_pdu`]).
        /// `pdu` is the caller's PDU scratch. Dropped, as [`stage`](MeteredTx::stage) drops, when
        /// the owner is busy; the return is `stage`'s, `true` only for a sample the port took, which
        /// is what the staged counter counts (`specs/link-control.md`, "Counting what is staged").
        pub fn stage_sample<S, const P: usize>(
            &mut self,
            link: &mut Link<SerialTransport<S, N>, P, N>,
            src: u8,
            state: &CyclicState,
            pdu: &mut [u8],
        ) -> bool
        where
            S: Read + Write + ReadReady,
        {
            match crate::guest_tx::sample_pdu(src, state, pdu) {
                Some(len) => self.stage(link, &pdu[..len]),
                None => false,
            }
        }
    }

    /// One sample as a guest port takes it: the address it goes out under, the state, and whether
    /// it is the PEER's forward (the high half of the staged counter) rather than the board's own.
    pub struct Sample {
        pub src: u8,
        pub state: CyclicState,
        pub forward: bool,
    }

    /// The sample the BLE port takes next, from the two latest-wins slots the control pass fills:
    /// the board's own (5 Hz, sent under `own_src`) first, and the peer's forward (2.5 Hz, under
    /// the peer's address) only when the own slot is empty (`specs/link-control.md`, "The peer's
    /// state on the BLE port: the forward"). The caller asks only while the owner is idle, so the
    /// two never contend for the staging buffer.
    #[allow(dead_code)]
    pub fn next_sample(
        own: &mut Option<CyclicState>,
        own_src: u8,
        peer: &mut Option<PeerCyclic>,
    ) -> Option<Sample> {
        own.take()
            .map(|state| Sample {
                src: own_src,
                state,
                forward: false,
            })
            .or_else(|| {
                peer.take().map(|p| Sample {
                    src: p.src,
                    state: p.state,
                    forward: true,
                })
            })
    }

    #[cfg(test)]
    mod tests {
        use super::MeteredTx;
        use embedded_io::{ErrorType, Read, ReadReady, Write, WriteReady};
        use link::{Link, SerialTransport};
        use std::collections::VecDeque;

        /// The firmware's BLE link shape: 16 B L2 frame capacity, 20 B stream frames, 72 B packets.
        const FRAME_CAP: usize = 16;
        const FRAMER_N: usize = FRAME_CAP + 4;
        const PACKET: usize = 72;

        /// One 8N1 character at 9600 baud (`ble::at::BAUD`): 10 bit times, 1.0417 ms.
        const BYTE_US: u64 = 10 * 1_000_000 / 9600;
        /// The bound the whole module exists to hold. The 16 kHz commutation ISR floats all three
        /// phases once `DEMAND` is `DEMAND_STALE_PERIODS` = 256 periods stale (`crate::motor`), and
        /// the 250 Hz control task that refreshes it is dispatched from the same main-loop pass that
        /// hands these frames over. NOT the 500 ms IWDG, which is three decimal orders slacker.
        const DEMAND_STALE_US: u64 = 256 * 1_000_000 / 16_000;
        /// A main-loop pass's own work, well inside one frame's wire time so the channel is found
        /// busy on most visits and the readiness rule actually bites.
        const PASS_US: u64 = 100;

        /// The BLE port's transmit as the firmware ships it: a one-shot DMA channel at 9600 8N1
        /// (`runtime_hal::DmaTx` under `runtime_hal::SplitSerial`), modelled where it matters. A
        /// write on an idle channel takes the whole frame, up to the channel's `FRAMER_N`-byte
        /// buffer, in one transfer; the channel is then busy for the frame's wire time, and its
        /// bytes reach the far end when that time is up. Time is a virtual microsecond clock the
        /// test advances a pass at a time.
        ///
        /// The write side mirrors `SplitSerial`'s DMA arm: a write while the channel is busy WAITS
        /// for it (the adapter retries inside its bounded budget, because `Ok(0)` on a non-empty
        /// buffer is forbidden), and `blocked_us` totals what the port made its caller wait for.
        /// That total is the number under test: the owner only writes when `write_ready` says the
        /// channel is idle, so it must stay zero. `write_ready` is the adapter's `!in_flight`.
        ///
        /// Delivered bytes land in `buf`, which is also the read side, so the receiving framer sees
        /// exactly the bytes that went on the wire; `transfers` records each transfer's bytes and
        /// start time, which is what the whole-frame and ordering cases read.
        #[derive(Default)]
        struct DmaWire {
            buf: VecDeque<u8>,
            in_flight: Vec<u8>,
            now_us: u64,
            free_at_us: u64,
            blocked_us: u64,
            transfers: Vec<(u64, Vec<u8>)>,
        }

        impl DmaWire {
            /// The channel's transfer is done once its wire time has passed: deliver its bytes.
            fn settle(&mut self) {
                if self.now_us >= self.free_at_us && !self.in_flight.is_empty() {
                    self.buf.extend(self.in_flight.drain(..));
                }
            }
            fn channel_idle(&mut self) -> bool {
                self.settle();
                self.now_us >= self.free_at_us
            }
        }

        impl ErrorType for DmaWire {
            type Error = core::convert::Infallible;
        }
        impl Write for DmaWire {
            fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
                if data.is_empty() {
                    return Ok(0);
                }
                if !self.channel_idle() {
                    self.blocked_us += self.free_at_us - self.now_us;
                    self.now_us = self.free_at_us;
                    self.settle();
                }
                let n = data.len().min(FRAMER_N);
                self.in_flight.extend_from_slice(&data[..n]);
                self.transfers.push((self.now_us, data[..n].to_vec()));
                self.free_at_us = self.now_us + n as u64 * BYTE_US;
                Ok(n)
            }
            fn flush(&mut self) -> Result<(), Self::Error> {
                if !self.channel_idle() {
                    self.blocked_us += self.free_at_us - self.now_us;
                    self.now_us = self.free_at_us;
                    self.settle();
                }
                Ok(())
            }
        }
        impl Read for DmaWire {
            fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
                self.settle();
                let n = out.len().min(self.buf.len());
                for b in out.iter_mut().take(n) {
                    *b = self.buf.pop_front().unwrap();
                }
                Ok(n)
            }
        }
        impl ReadReady for DmaWire {
            fn read_ready(&mut self) -> Result<bool, Self::Error> {
                self.settle();
                Ok(!self.buf.is_empty())
            }
        }
        impl WriteReady for DmaWire {
            fn write_ready(&mut self) -> Result<bool, Self::Error> {
                Ok(self.channel_idle())
            }
        }

        /// The one-deep emission slot, `net::walk::MAX_PDU` as the firmware sizes it
        /// (`BLE_EMIT_CAP`): the largest PDU an `Emission` can carry.
        const EMIT_CAP: usize = 64;

        type TestLink = Link<SerialTransport<DmaWire, FRAMER_N>, PACKET, FRAMER_N>;
        type Tx = MeteredTx<FRAMER_N, EMIT_CAP>;

        fn link() -> TestLink {
            Link::new(SerialTransport::new(DmaWire::default(), FRAME_CAP))
        }

        /// A 22 B PDU, which frames to TWO fragments of 20 and 12 wire bytes: a multi-fragment
        /// sample, the shape the cyclic emission stages on this port (the shipping 30 B PDU is two
        /// 20 B frames; what the cases here rest on is that it is two).
        const CYCLIC_PDU: [u8; 22] = [
            0x10, 0x01, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
        ];
        /// A second, distinguishable PDU standing for any BLE-bound emission (a `CONFIG_READ` reply,
        /// a `CFG_ARMED` refusal, a walk probe or forward): what the responder hands `route_emits`
        /// for the BLE port.
        const REPLY_PDU: [u8; 8] = [0x21, 0x01, 0xA0, 0xDE, 0xAD, 0xBE, 0xEF, 0x01];
        /// A third, so a queue-overflow case can say WHICH emission survived.
        const SECOND_REPLY_PDU: [u8; 6] = [0x21, 0x02, 0xB0, 0xC0, 0xFF, 0xEE];
        /// A `PORTS` reply at this board's three ports, the exact size the walk emits: a 3 B L3
        /// header, the port count, and 4 B of descriptor per port, 16 B in all. ONE byte past the
        /// 15 B chunk this link's 16 B frame capacity leaves, so it is the first emission that has
        /// to fragment, and the one the BLE walk cannot complete without.
        const PORTS_PDU: [u8; 16] = [
            0x30, 0x01, 0x00, 3, 0x00, 0x01, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x02, 0x03, 0x00,
            0x00,
        ];
        /// A `CONFIG_RESP` carrying a string value, the store's `DEVICE_NAME`: 3 B L3 header + field
        /// id + index + type tag + length + the 12 bytes of "hb-offroad-m" = 19 B. Comfortably above
        /// the fragmentation boundary rather than sitting on it.
        const NAME_RESP_PDU: [u8; 19] = [
            0x41, 0x01, 0x00, 0x10, 0x00, 0x05, 0x0C, b'h', b'b', b'-', b'o', b'f', b'f', b'r',
            b'o', b'a', b'd', b'-', b'm',
        ];
        /// The largest emission this port can be handed: `net::walk::MAX_PDU`, the bound on an
        /// `Emission`'s bytes. Five fragments on a 15 B chunk.
        const MAX_EMISSION: [u8; 64] = {
            let mut p = [0u8; 64];
            let mut i = 0;
            while i < 64 {
                p[i] = i as u8;
                i += 1;
            }
            p
        };

        /// Blocking charged to the port so far, in microseconds.
        fn blocked_us(l: &TestLink) -> u64 {
            l.transport().serial().blocked_us
        }
        /// Each transfer the port was handed, in order.
        fn transfers(l: &TestLink) -> Vec<Vec<u8>> {
            l.transport()
                .serial()
                .transfers
                .iter()
                .map(|(_, t)| t.clone())
                .collect()
        }

        /// One main-loop pass: visit the port, then let the clock advance by the pass's own work.
        /// Returns the blocking this pass cost, which is what the 16 ms bound is about.
        fn pass(tx: &mut Tx, l: &mut TestLink) -> u64 {
            let before = blocked_us(l);
            tx.pump(l);
            let cost = blocked_us(l) - before;
            l.transport_mut().serial_mut().now_us += PASS_US;
            cost
        }

        /// Run passes until nothing is waiting for the port, returning the worst single pass's
        /// blocking. Bounded so a stuck state cannot spin.
        fn drain(tx: &mut Tx, l: &mut TestLink) -> u64 {
            let mut worst = 0;
            let mut n = 0;
            while !tx.idle() && n < 4000 {
                worst = worst.max(pass(tx, l));
                n += 1;
            }
            worst
        }

        /// Everything the receiver reassembles once the last transfer has had its wire time.
        fn received(l: &mut TestLink) -> Vec<Vec<u8>> {
            let s = l.transport_mut().serial_mut();
            s.now_us = s.now_us.max(s.free_at_us);
            let mut out = [0u8; PACKET];
            let mut v = Vec::new();
            while let Some(p) = l.poll_recv(&mut out) {
                v.push(p.to_vec());
            }
            v
        }

        /// Hand `pdu` to the emission path on a quiet port, drain it, and return what the receiver
        /// reassembled together with the worst single pass's blocking.
        fn emit_and_drain(pdu: &[u8]) -> (u64, Vec<Vec<u8>>) {
            let mut l = link();
            let mut tx = Tx::new();
            tx.send(&mut l, pdu);
            let worst = drain(&mut tx, &mut l);
            (worst, received(&mut l))
        }

        /// **The P1 regression.** A `PORTS` reply is 16 B at this board's three ports, one byte over
        /// the single-fragment chunk. The owner must fragment it and deliver it whole: the BLE
        /// walk's `PROBE_PORTS` is answered by exactly this PDU, and a size-based drop is
        /// deterministic, so the controller's 3 s retransmit re-drops it forever.
        #[test]
        fn a_ports_sized_emission_reaches_the_receiver_intact() {
            let (worst, got) = emit_and_drain(&PORTS_PDU);
            assert_eq!(got, vec![PORTS_PDU.to_vec()], "the PORTS reply");
            assert_eq!(worst, 0, "a main-loop pass waited on the BLE wire");
        }

        /// The same, comfortably above the boundary rather than on it: a `CONFIG_RESP` carrying the
        /// device name. Reading or writing the board's name over BLE is this PDU.
        #[test]
        fn a_string_config_resp_reaches_the_receiver_intact() {
            let (worst, got) = emit_and_drain(&NAME_RESP_PDU);
            assert_eq!(got, vec![NAME_RESP_PDU.to_vec()], "the string CONFIG_RESP");
            assert_eq!(worst, 0, "a main-loop pass waited on the BLE wire");
        }

        /// The ceiling: the largest PDU an emission can carry (`net::walk::MAX_PDU`), five fragments
        /// deep, still arrives whole and still costs no pass any wait.
        #[test]
        fn the_largest_emission_the_port_carries_reaches_the_receiver_intact() {
            let (worst, got) = emit_and_drain(&MAX_EMISSION);
            assert_eq!(got, vec![MAX_EMISSION.to_vec()], "the largest emission");
            assert_eq!(worst, 0, "a main-loop pass waited on the BLE wire");
        }

        /// **The carrier's half of the ordering rule.** Every transfer the port is handed is ONE
        /// whole encoded frame, so nothing can land inside a frame: the frames of a sample with an
        /// emission arriving mid-sample, as the port saw them, are each a complete frame (SOF, the
        /// length its byte announces, CRC) and there are exactly as many transfers as frames.
        #[test]
        fn every_transfer_is_one_whole_frame() {
            let mut l = link();
            let mut tx = Tx::new();
            assert!(tx.stage(&mut l, &CYCLIC_PDU));
            pass(&mut tx, &mut l);
            tx.send(&mut l, &NAME_RESP_PDU);
            drain(&mut tx, &mut l);

            // The frames the same link encodes for the two PDUs, from an untouched link so the
            // PIDs line up with the run above (they are that link's first two packets).
            let mut expect = Vec::new();
            let mut model = link();
            for pdu in [&CYCLIC_PDU[..], &NAME_RESP_PDU[..]] {
                let mut i = 0;
                loop {
                    let mut f = [0u8; FRAMER_N];
                    let s = model.stage_fragment(pdu, i, &mut f).expect("stages");
                    expect.push(f[..s.len].to_vec());
                    if !s.more {
                        break;
                    }
                    i += 1;
                }
            }
            assert_eq!(
                transfers(&l),
                expect,
                "each transfer is one whole frame, in set order"
            );
            assert_eq!(
                received(&mut l),
                vec![CYCLIC_PDU.to_vec(), NAME_RESP_PDU.to_vec()]
            );
        }

        /// The timing bound for the case most likely to regress it: a MULTI-FRAGMENT emission
        /// arriving while a sample is going out. No single pass may block past the demand-stale
        /// coast, and here no pass waits at all: the owner only hands a frame to an idle channel.
        ///
        /// Both arrive. Once the sample's first fragment is handed over, the `promote` that opens
        /// `send` encodes its second, so that frame owns the next handover and the emission queues
        /// behind it.
        #[test]
        fn a_multi_fragment_emission_mid_sample_never_blocks_a_pass() {
            let mut l = link();
            let mut tx = Tx::new();
            assert!(tx.stage(&mut l, &CYCLIC_PDU));
            let mut worst = pass(&mut tx, &mut l).max(pass(&mut tx, &mut l));

            let before = blocked_us(&l);
            tx.send(&mut l, &NAME_RESP_PDU);
            let emission_pass = blocked_us(&l) - before;
            worst = worst.max(emission_pass);
            worst = worst.max(drain(&mut tx, &mut l));

            assert!(
                worst < DEMAND_STALE_US,
                "worst pass blocked {worst} us, past the {DEMAND_STALE_US} us demand-stale coast \
                 (the emission's own pass cost {emission_pass} us)"
            );
            assert_eq!(worst, 0, "a main-loop pass waited on the BLE wire");
            assert_eq!(
                received(&mut l),
                vec![CYCLIC_PDU.to_vec(), NAME_RESP_PDU.to_vec()]
            );
        }

        /// Fragments of one PDU keep the port to themselves. A telemetry sample offered on EVERY
        /// pass while a multi-fragment emission is going out is refused every time: a frame handed
        /// over between two fragments would land in the reassembler as a new set (`FRAG_IDX` 0
        /// discards the set in progress), destroying the emission. The sample staged after it
        /// arrives after it.
        #[test]
        fn a_sample_never_lands_between_two_fragments_of_an_emission() {
            let mut l = link();
            let mut tx = Tx::new();
            tx.send(&mut l, &PORTS_PDU);

            let mut n = 0;
            while !tx.idle() && n < 4000 {
                assert!(
                    !tx.stage(&mut l, &CYCLIC_PDU),
                    "a sample took the port between two fragments of an emission"
                );
                pass(&mut tx, &mut l);
                n += 1;
            }
            assert!(n > 0 && n < 4000, "the emission never drained ({n} passes)");

            assert!(tx.stage(&mut l, &CYCLIC_PDU), "the port never freed up");
            drain(&mut tx, &mut l);
            assert_eq!(
                received(&mut l),
                vec![PORTS_PDU.to_vec(), CYCLIC_PDU.to_vec()],
                "the emission was torn, or the sample after it never went out"
            );
        }

        /// **The headline invariant.** A telemetry frame is going out, a phone-triggered emission
        /// arrives (the worst case: `CONFIG_READ` replies and `CFG_ARMED` refusals are not gated on
        /// armed state, so this happens while balancing), and both go out. No single main-loop pass
        /// may block past the demand-stale coast, because a pass that does floats all three motor
        /// phases.
        #[test]
        fn no_main_loop_pass_blocks_past_the_demand_stale_coast() {
            let mut l = link();
            let mut tx = Tx::new();
            assert!(tx.stage(&mut l, &CYCLIC_PDU));

            let mut worst = pass(&mut tx, &mut l);
            let before = blocked_us(&l);
            tx.send(&mut l, &REPLY_PDU);
            let emission_pass = blocked_us(&l) - before;
            worst = worst.max(emission_pass);
            worst = worst.max(drain(&mut tx, &mut l));

            assert!(
                worst < DEMAND_STALE_US,
                "worst pass blocked {worst} us, past the {DEMAND_STALE_US} us demand-stale coast \
                 (the emission's own pass cost {emission_pass} us)"
            );
            // Not merely under the line: the port is only ever written when the channel is
            // already idle, so a pass waits for nothing at all.
            assert_eq!(worst, 0, "a main-loop pass waited on the BLE wire");
        }

        /// The same requirement for the far commoner case: an emission on a quiet port. Queueing
        /// writes nothing; the next visit hands it over.
        #[test]
        fn an_emission_on_a_quiet_port_costs_the_pass_nothing() {
            let mut l = link();
            let mut tx = Tx::new();
            tx.send(&mut l, &REPLY_PDU);
            assert_eq!(
                blocked_us(&l),
                0,
                "the emission blocked the pass instead of queueing"
            );
            assert!(
                transfers(&l).is_empty(),
                "queueing hands nothing to the port"
            );

            let worst = drain(&mut tx, &mut l);
            assert_eq!(worst, 0);
            assert_eq!(received(&mut l), vec![REPLY_PDU.to_vec()]);
        }

        /// The receiver's view of the priority rule: an emission arriving while a SAMPLE still owes
        /// fragments takes the slot from it. The sample's encoded first fragment is not dropped (it
        /// has its place), so it goes out, and the receiver discards it on the PID change rather
        /// than reassembling a short packet; the emission arrives whole and nothing trails it.
        ///
        /// The window is real: a sample staged while the channel is still sending the frame before
        /// it holds its first fragment encoded and its second unencoded until the channel frees.
        #[test]
        fn an_emission_takes_the_slot_from_a_waiting_sample_and_the_torn_sample_is_discarded() {
            let mut l = link();
            let mut tx = Tx::new();
            tx.send(&mut l, &REPLY_PDU);
            pass(&mut tx, &mut l); // the reply is on the wire; the channel is busy
            assert!(tx.idle());
            assert!(
                tx.stage(&mut l, &CYCLIC_PDU),
                "the sample is staged behind it"
            );
            tx.send(&mut l, &SECOND_REPLY_PDU);
            assert_eq!(drain(&mut tx, &mut l), 0);

            assert_eq!(
                received(&mut l),
                vec![REPLY_PDU.to_vec(), SECOND_REPLY_PDU.to_vec()],
                "the second reply did not survive, or the torn sample reached the receiver"
            );
            assert_eq!(
                transfers(&l).len(),
                3,
                "reply, the sample's first frame, reply"
            );
        }

        /// A sample on a QUIET port still goes out whole, both fragments, with nothing else
        /// touching the port: the yielding above is a collision rule, not the common path.
        #[test]
        fn a_sample_on_a_quiet_port_goes_out_whole() {
            let mut l = link();
            let mut tx = Tx::new();
            assert!(tx.stage(&mut l, &CYCLIC_PDU));
            let worst = drain(&mut tx, &mut l);
            assert_eq!(worst, 0, "a main-loop pass waited on the BLE wire");
            let sizes: Vec<usize> = transfers(&l).iter().map(|t| t.len()).collect();
            assert_eq!(sizes, vec![20, 12], "two transfers, one per frame");
            assert_eq!(received(&mut l), vec![CYCLIC_PDU.to_vec()]);
        }

        /// Telemetry's overflow policy: a sample arriving while the owner is busy is dropped, never
        /// queued, so no backlog of stale readings accrues.
        #[test]
        fn a_sample_arriving_mid_drain_is_dropped_not_queued() {
            let mut l = link();
            let mut tx = Tx::new();
            assert!(tx.stage(&mut l, &CYCLIC_PDU));
            pass(&mut tx, &mut l);
            assert!(!tx.stage(&mut l, &CYCLIC_PDU), "no backlog accrues");
        }

        /// Telemetry also yields to a queued emission: the one slot is the reply's, not a sample's.
        #[test]
        fn a_sample_is_dropped_while_an_emission_is_queued() {
            let mut l = link();
            let mut tx = Tx::new();
            assert!(tx.stage(&mut l, &CYCLIC_PDU));
            pass(&mut tx, &mut l);
            tx.send(&mut l, &REPLY_PDU);
            assert!(
                !tx.stage(&mut l, &CYCLIC_PDU),
                "a sample took the slot the queued emission is waiting in"
            );
        }

        /// Emissions' overflow policy, the opposite one: the queue is one deep, and a second
        /// emission arriving behind a queued one is DROPPED while the first is kept, so replies keep
        /// the order their requests arrived in. Only an emission can hold the slot against another:
        /// a sample yields it to a reply (`MeteredTx::send`).
        #[test]
        fn a_second_queued_emission_is_dropped_and_the_first_kept() {
            let mut l = link();
            let mut tx = Tx::new();
            assert!(tx.stage(&mut l, &CYCLIC_PDU));
            pass(&mut tx, &mut l);
            tx.send(&mut l, &REPLY_PDU);
            tx.send(&mut l, &SECOND_REPLY_PDU);
            assert_eq!(blocked_us(&l), 0, "the overflow blocked the pass");

            drain(&mut tx, &mut l);
            assert_eq!(
                received(&mut l),
                vec![CYCLIC_PDU.to_vec(), REPLY_PDU.to_vec()],
                "the older emission was dropped in favour of the newer, or the overflow got out"
            );
        }

        /// The slot frees as soon as the frame ahead has been handed over, so a burst of emissions
        /// drains one per frame rather than stalling behind a single queued one.
        #[test]
        fn the_slot_frees_once_the_frame_ahead_is_handed_over() {
            let mut l = link();
            let mut tx = Tx::new();
            tx.send(&mut l, &REPLY_PDU); // goes out on the first visit
            tx.send(&mut l, &SECOND_REPLY_PDU); // queued behind it
            let worst = drain(&mut tx, &mut l);
            assert_eq!(worst, 0);
            assert_eq!(
                received(&mut l),
                vec![REPLY_PDU.to_vec(), SECOND_REPLY_PDU.to_vec()],
                "the queued emission never followed the one ahead of it"
            );
            assert!(tx.idle(), "the port did not return to quiet");
        }

        /// The wire time itself is untouched, and must be: the channel is busy for a frame's
        /// character times, and the owner does not hand it the next frame until then. What changed
        /// is who waits for it: nobody.
        #[test]
        fn a_frame_still_takes_its_wire_time() {
            let mut l = link();
            let mut tx = Tx::new();
            assert!(tx.stage(&mut l, &CYCLIC_PDU));
            drain(&mut tx, &mut l);
            let t = &l.transport().serial().transfers;
            assert_eq!(t.len(), 2);
            let gap = t[1].0 - t[0].0;
            assert!(
                gap >= t[0].1.len() as u64 * BYTE_US,
                "the second frame was handed over {gap} us after the first, inside its wire time"
            );
            assert_eq!(blocked_us(&l), 0, "and none of it was paid by the CPU");
        }

        // --- The send step against a main loop slower than its tick -----------------------------
        //
        // `specs/bench-evidence/2026-10-25/esp-tool-e2e/RECORD.md`: with the phone attached the
        // offroad master took one to two SysTick ticks per main-loop pass, and when the port moved
        // bytes a visit at a time the peer's forward was superseded in its slot. These run the
        // loop's one BLE visit (8b, after dispatch) over virtual time against `DmaWire`.

        /// One SysTick tick, `scheduler::TICK_HZ`.
        const TICK_US: u64 = 1_000_000 / scheduler::TICK_HZ as u64;
        /// The control run inside a pass, ahead of the visit: the IMU-equipped F130's measured
        /// callback (`specs/bench-evidence/2026-08-02/wide-division/RECORD.md`).
        const CONTROL_US: u64 = 1_399;
        /// The two slots' cadences in ticks: the own sample at 5 Hz
        /// (`orchestrator::BLE_CYCLIC_DIVISOR`) and the forward at 2.5 Hz
        /// (`orchestrator::dispatch::BLE_PEER_CYCLIC_TICKS`).
        const OWN_TICKS: u32 = orchestrator::BLE_CYCLIC_DIVISOR;
        const PEER_TICKS: u32 = orchestrator::dispatch::BLE_PEER_CYCLIC_TICKS;
        const OWN_SRC: u8 = 0x01;
        const PEER_SRC: u8 = 0x02;
        /// Simulated wall time per run.
        const RUN_US: u64 = 60_000_000;

        /// What reached the receiver: samples made and delivered per source, and the longest gap
        /// between two delivered forwards.
        struct Delivered {
            own_made: u32,
            own_sent: u32,
            peer_made: u32,
            peer_sent: u32,
            peer_worst_gap_us: u64,
        }

        fn sample() -> linkctl::CyclicState {
            linkctl::CyclicState::decode(&[0u8; linkctl::CyclicState::ENCODED_LEN]).unwrap()
        }

        /// Run the send step for [`RUN_US`] with every pass `pass_us` long, the samples the
        /// shipping payload (`sample()` encodes to the real `CYCLIC_STATE` PDU, two 20 B frames).
        fn run_slow_loop(pass_us: u64) -> Delivered {
            let mut l = link();
            let mut tx = Tx::new();
            let mut own: Option<linkctl::CyclicState> = None;
            let mut peer: Option<orchestrator::PeerCyclic> = None;
            let mut pdu = [0u8; net::walk::MAX_PDU];
            let mut out = [0u8; PACKET];
            let mut d = Delivered {
                own_made: 0,
                own_sent: 0,
                peer_made: 0,
                peer_sent: 0,
                peer_worst_gap_us: 0,
            };
            let mut last_control_tick = 0u32;
            let mut last_peer_us = 0u64;
            let tick = |l: &TestLink| (l.transport().serial().now_us / TICK_US) as u32;
            while l.transport().serial().now_us < RUN_US {
                // The drains and the rest of the pass ahead of dispatch.
                l.transport_mut().serial_mut().now_us += pass_us - CONTROL_US;
                // 6. The control run fills the slots, latest-wins, at their tick cadences.
                let now = tick(&l);
                for k in last_control_tick + 1..=now {
                    if k % OWN_TICKS == 0 {
                        own = Some(sample());
                        d.own_made += 1;
                    }
                    if k % PEER_TICKS == PEER_TICKS / 4 {
                        peer = Some(orchestrator::PeerCyclic {
                            src: PEER_SRC,
                            state: sample(),
                        });
                        d.peer_made += 1;
                    }
                }
                last_control_tick = now;
                l.transport_mut().serial_mut().now_us += CONTROL_US;
                // 8b, after dispatch: stage when idle, then the pass's one visit.
                if tx.idle() {
                    if let Some(s) = super::next_sample(&mut own, OWN_SRC, &mut peer) {
                        tx.stage_sample(&mut l, s.src, &s.state, &mut pdu);
                    }
                }
                tx.pump(&mut l);
                // The receiver.
                while let Some(p) = l.poll_recv(&mut out) {
                    let now_us = l.transport().serial().now_us;
                    match net::Pdu::decode(p).map(|p| p.src) {
                        Ok(OWN_SRC) => d.own_sent += 1,
                        Ok(PEER_SRC) => {
                            d.peer_sent += 1;
                            d.peer_worst_gap_us = d.peer_worst_gap_us.max(now_us - last_peer_us);
                            last_peer_us = now_us;
                        }
                        other => panic!("an unexpected PDU reached the receiver: {other:?}"),
                    }
                }
            }
            assert_eq!(blocked_us(&l), 0, "a pass waited on the BLE wire");
            d
        }

        /// Both streams at their cadences: nearly every own sample, at least nine forwards in ten,
        /// and never more than two forward periods (800 ms) without one, so the app's peer chip
        /// does not go stale.
        fn at_cadence(d: &Delivered) -> bool {
            let peer_period_us = PEER_TICKS as u64 * TICK_US;
            d.own_sent * 100 >= d.own_made * 95
                && d.peer_sent * 10 >= d.peer_made * 9
                && d.peer_worst_gap_us <= 2 * peer_period_us
        }

        /// The pass lengths the offroad master measured with the phone attached: 1.0, 1.23 and
        /// 1.78 ticks per pass (`specs/bench-evidence/2026-10-24/l3-node-class/RECORD.md`), and
        /// the 2.01 of the 2026-10-25 session (`specs/bench-evidence/2026-10-25/esp-tool-e2e/
        /// RECORD.md`).
        const MEASURED_PASS_US: [u64; 4] = [4_000, 4_920, 7_120, 8_040];

        /// **The forward regression.** At every measured pass length, both the board's own sample
        /// and the peer's forward reach the phone at their cadences. With the port moving bytes a
        /// visit at a time the forward kept 81 per cent at 1.78 ticks and lost its cadence at 2.01;
        /// with a frame per handover the pass rate no longer sets the port's rate.
        #[test]
        fn both_samples_keep_their_cadence_at_every_measured_pass_length() {
            for pass_us in MEASURED_PASS_US {
                let d = run_slow_loop(pass_us);
                assert!(
                    at_cadence(&d),
                    "{pass_us} us passes: own {}/{}, forward {}/{}, worst forward gap {} us",
                    d.own_sent,
                    d.own_made,
                    d.peer_sent,
                    d.peer_made,
                    d.peer_worst_gap_us
                );
            }
        }

        /// `specs/ble-port-dma-tx.md`, "Tests": at the 2.01 ticks per pass the offroad master
        /// measured with the phone attached, both samples keep their cadence. It failed while the
        /// port moved about two bytes a pass, under the 285 B/s the two samples need: the own
        /// sample always went first, and the forward waited in its slot past its 100-tick cadence
        /// and was superseded there.
        #[test]
        fn both_samples_keep_their_cadence_at_two_ticks_a_pass() {
            let d = run_slow_loop(8_040);
            assert!(
                at_cadence(&d),
                "own {}/{}, forward {}/{}, worst forward gap {} us",
                d.own_sent,
                d.own_made,
                d.peer_sent,
                d.peer_made,
                d.peer_worst_gap_us
            );
        }
    }
}

/// The two guest ports' telemetry: the board's own `CYCLIC_STATE` sample (5 Hz) and the peer's
/// forward (2.5 Hz) go to EVERY guest port with a consumer, the BLE port through its one TX owner
/// and the SWD mailbox port straight into its ring (`specs/link-control.md`, "The mailbox port
/// carries both samples (2026-10-25)"), and each port counts what it accepted ("Counting what is
/// staged (2026-10-25)").
///
/// Outside the `target_os = "none"` firmware module, the `ble_wire` precedent, so the host test run
/// drives the real step against a real mailbox ring and a real BLE staging owner. `allow(dead_code)`
/// because the only non-test caller is the target-only service loop.
#[allow(dead_code)]
mod guest_tx {
    use crate::ble_wire::{next_sample, MeteredTx};
    use embedded_io::{Read, ReadReady, Write, WriteReady};
    use link::{Link, SerialTransport};
    use linkctl::CyclicState;
    use orchestrator::dispatch::PeerCyclic;
    use swd_mailbox::MailboxSerial;

    /// The guest ports' two latest-wins sample slots, and which of them the control pass refilled
    /// since the loop last emitted.
    ///
    /// The slots are the BLE port's: a sample waits in one until the port's owner is idle, which
    /// on the 9600-baud module can be several passes. The mailbox port takes a sample only in the
    /// pass it was built and never from a slot it has already seen, which is what the two `fresh`
    /// flags record: without them a sample still waiting for the BLE port would go into the ring
    /// once per pass. They cost no RAM the slots did not already pad.
    pub struct GuestSlots {
        own: Option<CyclicState>,
        peer: Option<PeerCyclic>,
        own_fresh: bool,
        peer_fresh: bool,
    }

    impl GuestSlots {
        pub const fn new() -> Self {
            GuestSlots {
                own: None,
                peer: None,
                own_fresh: false,
                peer_fresh: false,
            }
        }

        /// The control pass's hand-off: whatever the two cadences built this run
        /// (`orchestrator::ble_cyclic_tx`, `orchestrator::ble_peer_tx`) overwrites its slot,
        /// latest-wins, and marks it fresh. A catch-up burst that fills a slot twice before the
        /// loop runs leaves the later sample and one fresh mark.
        pub fn fill(&mut self, own: Option<CyclicState>, peer: Option<PeerCyclic>) {
            if let Some(c) = own {
                self.own = Some(c);
                self.own_fresh = true;
            }
            if let Some(p) = peer {
                self.peer = Some(p);
                self.peer_fresh = true;
            }
        }
    }

    /// Samples staged per guest port, `CTRL_OBS` words 40 (BLE) and 41 (mailbox): each
    /// `own u16 | forwards u16 << 16`, counted when the port ACCEPTED the sample (the `stage` that
    /// took it, the `send` that placed it), wrapping within its half
    /// (`specs/link-control.md`, "Counting what is staged (2026-10-25)").
    #[derive(Clone, Copy, Default)]
    pub struct GuestStaged {
        pub ble: u32,
        pub mailbox: u32,
    }

    /// One accepted sample into a packed staged word: the low half for the board's own, the high
    /// half for a forward, each wrapping on its own so a full half never carries into the other.
    pub fn bump(word: u32, forward: bool) -> u32 {
        let shift = if forward { 16 } else { 0 };
        let half = ((word >> shift) as u16).wrapping_add(1) as u32;
        (word & !(0xFFFF << shift)) | (half << shift)
    }

    /// The `CYCLIC_STATE` PDU both guest ports carry, `[0x10][src][dst 0x00][27 B]`, encoded into
    /// `pdu`; the encoded length, or `None` if `pdu` is too small. dst 0x00 by the point-to-point
    /// rule: each guest port carries exactly one peer (`specs/link-control.md`, "Addressing and
    /// emission"). The one encoder, so the two ports' bytes are the same bytes.
    pub fn sample_pdu(src: u8, state: &CyclicState, pdu: &mut [u8]) -> Option<usize> {
        let mut payload = [0u8; CyclicState::ENCODED_LEN];
        let n = state.encode(&mut payload);
        net::Pdu::new(
            linkctl::OP_CYCLIC_STATE,
            src,
            net::pdu::NO_ADDRESS,
            &payload[..n],
        )
        .ok()?
        .encode(pdu)
        .ok()
    }

    /// Put one sample into the mailbox ring; `true` if the ring took it whole. A full ring refuses
    /// it (all-or-nothing, `specs/swd-mailbox.md`, "Backpressure") and the serial counts the
    /// refusal in word 34; the sample is dropped, latest-wins.
    fn mailbox_sample<const MP: usize, const MN: usize>(
        mailbox: &mut Link<SerialTransport<MailboxSerial, MN>, MP, MN>,
        src: u8,
        state: &CyclicState,
        pdu: &mut [u8],
    ) -> bool {
        match sample_pdu(src, state, pdu) {
            Some(len) => mailbox.send(&pdu[..len]).is_ok(),
            None => false,
        }
    }

    /// Step 8b's samples for one pass, on both guest ports.
    ///
    /// **The mailbox port** takes each sample built since the last pass, in this pass, and only
    /// while a host is attached (`MailboxSerial::outbound_has_consumer`): a board with nothing on
    /// SWD neither writes the ring nor inflates word 34. No TX owner: a write is one copy into RAM
    /// with no wait, so this adds nothing to the 16 ms demand-stale bound. It
    /// goes FIRST, because it reads the slots without emptying them and the BLE port then takes
    /// what it takes.
    ///
    /// **The BLE port** is unchanged: when its owner is idle it takes the next sample from the
    /// slots (the own one first) and stages it for the owner's visit, which is the caller's next
    /// step.
    pub fn emit_samples<
        S,
        const BP: usize,
        const BN: usize,
        const Q: usize,
        const MP: usize,
        const MN: usize,
    >(
        slots: &mut GuestSlots,
        own_src: u8,
        mailbox: &mut Link<SerialTransport<MailboxSerial, MN>, MP, MN>,
        ble: Option<(
            &mut Link<SerialTransport<S, BN>, BP, BN>,
            &mut MeteredTx<BN, Q>,
        )>,
        staged: &mut GuestStaged,
        pdu: &mut [u8],
    ) where
        S: Read + Write + ReadReady + WriteReady,
    {
        let own_fresh = core::mem::take(&mut slots.own_fresh);
        let peer_fresh = core::mem::take(&mut slots.peer_fresh);
        if mailbox.transport().serial().outbound_has_consumer() {
            if let (true, Some(c)) = (own_fresh, slots.own.as_ref()) {
                if mailbox_sample(mailbox, own_src, c, pdu) {
                    staged.mailbox = bump(staged.mailbox, false);
                }
            }
            if let (true, Some(p)) = (peer_fresh, slots.peer.as_ref()) {
                if mailbox_sample(mailbox, p.src, &p.state, pdu) {
                    staged.mailbox = bump(staged.mailbox, true);
                }
            }
        }
        if let Some((l, tx)) = ble {
            if tx.idle() {
                if let Some(s) = next_sample(&mut slots.own, own_src, &mut slots.peer) {
                    if tx.stage_sample(l, s.src, &s.state, pdu) {
                        staged.ble = bump(staged.ble, s.forward);
                    }
                }
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::{bump, emit_samples, GuestSlots, GuestStaged};
        use crate::ble_wire::MeteredTx;
        use embedded_io::{ErrorType, Read, ReadReady, Write, WriteReady};
        use link::{Link, SerialTransport};
        use linkctl::CyclicState;
        use orchestrator::dispatch::PeerCyclic;
        use std::collections::VecDeque;
        use swd_mailbox::{layout, Bridge, Mailbox, MailboxSerial, FRAME_CAPACITY, REGION_LEN};

        /// The firmware's two guest links: the BLE port's 16 B frame (20 B stream frames) and the
        /// mailbox port's 128 B frame (132 B), both with the 72 B packet buffer.
        const BLE_FRAME_CAP: usize = 16;
        const BLE_N: usize = BLE_FRAME_CAP + 4;
        const MB_N: usize = FRAME_CAPACITY + 4;
        const PACKET: usize = 72;
        const EMIT_CAP: usize = 64;
        const OWN_SRC: u8 = 0x01;
        const PEER_SRC: u8 = 0x02;

        /// A BLE wire that takes every frame at once: what is under test here is which samples
        /// reach the port, not the port's timing, which `ble_wire`'s tests own.
        #[derive(Default)]
        struct Wire {
            buf: VecDeque<u8>,
        }
        impl ErrorType for Wire {
            type Error = core::convert::Infallible;
        }
        impl Write for Wire {
            fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
                self.buf.extend(data);
                Ok(data.len())
            }
            fn flush(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }
        }
        impl Read for Wire {
            fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
                let n = out.len().min(self.buf.len());
                for b in out.iter_mut().take(n) {
                    *b = self.buf.pop_front().unwrap();
                }
                Ok(n)
            }
        }
        impl ReadReady for Wire {
            fn read_ready(&mut self) -> Result<bool, Self::Error> {
                Ok(!self.buf.is_empty())
            }
        }
        impl WriteReady for Wire {
            fn write_ready(&mut self) -> Result<bool, Self::Error> {
                Ok(true)
            }
        }

        type BleLink = Link<SerialTransport<Wire, BLE_N>, PACKET, BLE_N>;
        type MbLink = Link<SerialTransport<MailboxSerial, MB_N>, PACKET, MB_N>;

        /// The mailbox region in host RAM, `u32`-backed for the header's alignment, with the
        /// firmware's endpoint and the host's reading end over it.
        struct Rig {
            ram: Box<[u32]>,
            board: MbLink,
            host: MbLink,
            ble: BleLink,
            tx: MeteredTx<BLE_N, EMIT_CAP>,
            slots: GuestSlots,
            staged: GuestStaged,
        }

        impl Rig {
            fn new(attached: bool) -> Self {
                let mut ram = vec![0u32; REGION_LEN.div_ceil(4)].into_boxed_slice();
                // SAFETY: `ram` is 4-byte aligned, REGION_LEN bytes, and lives in the rig as long
                // as every handle made from it.
                let mb = unsafe { Mailbox::from_raw(ram.as_mut_ptr() as *mut u8) };
                mb.init_header();
                if attached {
                    Bridge::attach(mb).unwrap();
                }
                Rig {
                    ram,
                    board: Link::new(SerialTransport::new(
                        MailboxSerial::firmware(mb),
                        FRAME_CAPACITY,
                    )),
                    host: Link::new(SerialTransport::new(
                        MailboxSerial::bridge(mb),
                        FRAME_CAPACITY,
                    )),
                    ble: Link::new(SerialTransport::new(Wire::default(), BLE_FRAME_CAP)),
                    tx: MeteredTx::new(),
                    slots: GuestSlots::new(),
                    staged: GuestStaged::default(),
                }
            }

            /// One step 8b: the samples, then the BLE owner's visits run to completion, as the
            /// loop's would over the following passes.
            fn pass(&mut self) {
                let mut pdu = [0u8; net::walk::MAX_PDU];
                emit_samples(
                    &mut self.slots,
                    OWN_SRC,
                    &mut self.board,
                    Some((&mut self.ble, &mut self.tx)),
                    &mut self.staged,
                    &mut pdu,
                );
                while !self.tx.idle() {
                    self.tx.pump(&mut self.ble);
                }
            }

            fn mailbox_pdus(&mut self) -> Vec<Vec<u8>> {
                let mut out = [0u8; PACKET];
                let mut v = Vec::new();
                while let Some(p) = self.host.poll_recv(&mut out) {
                    v.push(p.to_vec());
                }
                v
            }

            fn ble_pdus(&mut self) -> Vec<Vec<u8>> {
                let mut out = [0u8; PACKET];
                let mut v = Vec::new();
                while let Some(p) = self.ble.poll_recv(&mut out) {
                    v.push(p.to_vec());
                }
                v
            }

            fn refused(&self) -> u16 {
                self.board.transport().serial().refused_writes()
            }

            /// The outbound ring's fill, read from the header as a host tool reads it.
            fn ring_used(&self) -> u32 {
                let w = |off: usize| self.ram[off / 4];
                w(layout::T2H_HEAD).wrapping_sub(w(layout::T2H_TAIL))
            }
        }

        /// Two distinguishable states, so a port that carried the wrong one shows.
        fn state(tag: u8) -> CyclicState {
            let mut b = [0u8; CyclicState::ENCODED_LEN];
            b[0] = tag;
            CyclicState::decode(&b).unwrap()
        }

        fn forward(tag: u8) -> PeerCyclic {
            PeerCyclic {
                src: PEER_SRC,
                state: state(tag),
            }
        }

        fn src(pdu: &[u8]) -> u8 {
            net::Pdu::decode(pdu).unwrap().src
        }

        /// Ports: with a host attached, each due own sample and each forward reaches BOTH the BLE
        /// staging owner and the mailbox link, and the bytes are identical on the two.
        #[test]
        fn each_sample_reaches_both_guest_ports_with_identical_bytes() {
            let mut r = Rig::new(true);
            r.slots.fill(Some(state(7)), Some(forward(9)));
            // The own sample goes out on both ports in this pass; the BLE port takes the forward
            // on the next idle pass, the mailbox port already has it.
            r.pass();
            r.pass();
            let mb = r.mailbox_pdus();
            let ble = r.ble_pdus();
            assert_eq!(mb.len(), 2, "the mailbox port carried own + forward");
            assert_eq!(
                mb, ble,
                "the two ports carry the same PDUs, in the same order"
            );
            assert_eq!(src(&mb[0]), OWN_SRC);
            assert_eq!(src(&mb[1]), PEER_SRC);
            assert_eq!(mb[0][0], linkctl::OP_CYCLIC_STATE);
            assert_eq!(
                mb[0][2],
                net::pdu::NO_ADDRESS,
                "dst 0x00 on the mailbox port"
            );
            assert_eq!(mb[0].len(), 3 + CyclicState::ENCODED_LEN);
        }

        /// A sample still waiting in its slot for a busy BLE port is NOT put in the ring again on
        /// the passes it waits: the mailbox port takes each sample once, in the pass it was built.
        #[test]
        fn a_sample_waiting_for_the_ble_port_goes_into_the_ring_once() {
            let mut r = Rig::new(true);
            r.slots.fill(Some(state(1)), Some(forward(2)));
            for _ in 0..5 {
                r.pass();
            }
            assert_eq!(r.mailbox_pdus().len(), 2);
            assert_eq!(r.ble_pdus().len(), 2);
            assert_eq!(r.staged.mailbox, 1 | 1 << 16);
            assert_eq!(r.staged.ble, 1 | 1 << 16);
        }

        /// Ports: with no host attached the mailbox link is not written at all, and its refused
        /// count stays zero, so word 34 is not inflated by telemetry on a board nothing reached.
        #[test]
        fn with_no_consumer_the_mailbox_is_not_written_and_nothing_is_refused() {
            let mut r = Rig::new(false);
            for _ in 0..10 {
                r.slots.fill(Some(state(3)), Some(forward(4)));
                r.pass();
            }
            assert_eq!(r.ring_used(), 0, "the outbound ring stayed empty");
            assert_eq!(r.refused(), 0);
            assert_eq!(r.staged.mailbox, 0);
            assert_eq!(r.ble_pdus().len(), 10, "the BLE port is unaffected");
        }

        /// Ports and Counters: a full ring drops the sample, counts exactly one refused write, and
        /// leaves the mailbox word where it was. A 28 B PDU is 33 B on the ring, so seven fit in
        /// the 256 B ring with nothing draining it and the eighth is refused.
        #[test]
        fn a_full_ring_drops_the_sample_and_counts_one_refused_write() {
            let mut r = Rig::new(true);
            for k in 0..7 {
                r.slots.fill(Some(state(k)), None);
                r.pass();
            }
            assert_eq!(r.refused(), 0);
            assert_eq!(r.staged.mailbox, 7);
            let used = r.ring_used();
            r.slots.fill(Some(state(8)), None);
            r.pass();
            assert_eq!(r.refused(), 1, "one refused write for the dropped sample");
            assert_eq!(
                r.staged.mailbox, 7,
                "a refused sample is not counted as staged"
            );
            assert_eq!(r.ring_used(), used, "nothing of it entered the ring");
            assert_eq!(r.staged.ble, 8, "the BLE port took it");
            let tags: Vec<u8> = r.mailbox_pdus().iter().map(|p| p[3]).collect();
            assert_eq!(
                tags,
                (0..7).collect::<Vec<u8>>(),
                "the queued seven, intact"
            );
        }

        /// Counters: each staged sample advances its port's word by one, own in the low half and
        /// forwards in the high half; a sample the BLE port refused (busy) is not counted.
        #[test]
        fn the_staged_words_count_own_and_forwards_in_their_own_halves() {
            let mut r = Rig::new(true);
            r.slots.fill(Some(state(1)), None);
            r.pass();
            assert_eq!(r.staged.ble, 1);
            assert_eq!(r.staged.mailbox, 1);
            r.slots.fill(None, Some(forward(2)));
            r.pass();
            assert_eq!(r.staged.ble, 1 | 1 << 16);
            assert_eq!(r.staged.mailbox, 1 | 1 << 16);

            // A BLE port still draining refuses the stage: no count there, and the sample waits.
            r.slots.fill(Some(state(3)), None);
            let mut pdu = [0u8; net::walk::MAX_PDU];
            assert!(r.tx.stage(&mut r.ble, &[0x21, 0x01, 0xA0, 0xDE]));
            emit_samples(
                &mut r.slots,
                OWN_SRC,
                &mut r.board,
                Some((&mut r.ble, &mut r.tx)),
                &mut r.staged,
                &mut pdu,
            );
            assert_eq!(r.staged.ble, 1 | 1 << 16, "the busy port took nothing");
            assert_eq!(
                r.staged.mailbox,
                2 | 1 << 16,
                "the mailbox port took it at once"
            );
        }

        /// Counting what is staged (`specs/link-control.md`, 2026-10-25): an emission queued at
        /// any phase of the own sample's going out, the forward due on any tick, costs no forward.
        /// The BLE path takes from a slot only after `MeteredTx::idle`, and `idle` means no frame
        /// is waiting and nothing is queued, so the `promote` that opens `stage` has nothing to
        /// move and `stage` cannot refuse.
        #[test]
        fn an_emission_queued_at_any_phase_costs_no_forward() {
            let emits: [&[u8]; 3] = [&[0x21, 0x01, 0xA0, 0xDE], &[0x22; 16], &[0x23; 30]];
            for e in emits {
                for phase in 0..125u32 {
                    for peer_phase in [0u32, 25, 50, 75] {
                        let mut r = Rig::new(false);
                        let mut pdu = [0u8; net::walk::MAX_PDU];
                        for t in 1..=450u32 {
                            if t % 125 == phase {
                                r.tx.send(&mut r.ble, e);
                            }
                            r.tx.pump(&mut r.ble);
                            let own = (t <= 400 && t % 50 == 0).then(|| state(1));
                            let peer = (t <= 400 && t % 100 == peer_phase).then(|| forward(2));
                            r.slots.fill(own, peer);
                            emit_samples(
                                &mut r.slots,
                                OWN_SRC,
                                &mut r.board,
                                Some((&mut r.ble, &mut r.tx)),
                                &mut r.staged,
                                &mut pdu,
                            );
                            r.tx.pump(&mut r.ble);
                        }
                        assert_eq!(
                            r.staged.ble,
                            8 | 4 << 16,
                            "emission {} B at phase {phase}, forward at {peer_phase}",
                            e.len()
                        );
                    }
                }
            }
        }

        /// The halves wrap independently: a full own half rolls to zero without touching the
        /// forwards, and the reverse.
        #[test]
        fn each_half_wraps_on_its_own() {
            assert_eq!(bump(0x0003_FFFF, false), 0x0003_0000);
            assert_eq!(bump(0xFFFF_0005, true), 0x0000_0005);
            assert_eq!(bump(0, true), 0x0001_0000);
        }
    }
}

/// The stack high-water instrument (`specs/firmware.md`, "RAM/stack budget"): paint the free stack
/// with a known word at boot, then find the deepest word the running image ever destroyed.
///
/// The arithmetic lives here, outside the `target_os = "none"` firmware module, so the workspace
/// host-test run reaches it: the region is just words in memory, and a synthetic region with a known
/// deepest touch exercises exactly the code silicon runs. `allow(dead_code)` because the only
/// non-test callers are in the target-only firmware module, so the host non-test build (which CI
/// clippys with `--all-targets -D warnings`) sees these unused, the `probe_window` precedent.
///
/// **Why paint at all, rather than sum the prologues.** The prologue-sum estimate for this image was
/// 630-850 B and it was dead wrong: the binding chain is a blocking IMU burst with the USART1 RX ISR
/// landing on top of it, which no static sum finds because it is not one call chain. Paint measures
/// what actually happened, ISRs included.
///
/// **What it cannot see**, recorded so the number is not over-read: a frame that is ALLOCATED but
/// never written leaves the paint intact underneath it, so the mark is a lower bound on depth and
/// therefore an UPPER bound on the margin. Every paint-based high-water has this property. It is the
/// reason the floor policy is >= 250 B rather than >= 0.
mod stack_paint {
    /// The paint word, `"STAK"` little-endian, matching `CTRL_OBS_MAGIC`'s house style.
    ///
    /// A word of real stack traffic that happens to equal the pattern reads as intact paint, which
    /// is why the value is chosen to be implausible as data: return addresses on this part are
    /// `0x0800_xxxx`, RAM pointers `0x2000_xxxx`, peripheral addresses `0x4000_xxxx`, and the
    /// measured deep-chain word was `0x4000_4400` (the USART1 base). Four printable ASCII bytes are
    /// none of those.
    pub const PAINT: u32 = 0x4B41_5453;

    /// Write [`PAINT`] over `words` words starting at `base`.
    ///
    /// Volatile, and that is load-bearing twice over: nothing ever reads these words back through
    /// this pointer, so plain stores are dead and LLVM may delete them outright, and a `memset` the
    /// optimizer might otherwise form would make the cost harder to price than a word loop whose
    /// per-word cost can be read off the disassembly.
    ///
    /// # Safety
    /// `base` must be 4-byte aligned and `[base, base + words)` must be writable memory that no
    /// live data occupies for the duration of the call. On the firmware side that is the free stack
    /// below the current stack pointer, whose floor is `_stack_end`: painting from `__ebss` instead
    /// would run through the `.uninit` section and destroy `CTRL_OBS` (its magic, its
    /// reset-surviving boot counter, and every field the bench reads).
    #[allow(dead_code)]
    pub unsafe fn paint(base: *mut u32, words: usize) {
        for i in 0..words {
            // SAFETY: the caller guarantees `words` writable words at `base`.
            unsafe { base.add(i).write_volatile(PAINT) };
        }
    }

    /// Scan upward from `from` while the paint is intact, and return the index of the first word
    /// that is not [`PAINT`] (or `words` if the whole window is intact).
    ///
    /// Index 0 is the LOWEST address. The stack grows DOWN into the region from the top, so the
    /// surviving paint is a prefix `[0, mark)` and the first dirty index IS the deepest word the
    /// image ever reached. Scanning up from the bottom is the conservative direction: it reports
    /// the lowest dirty word, so an allocated-but-unwritten frame word makes the answer no more
    /// optimistic than the truth.
    ///
    /// Volatile because the region is live stack: an ISR can push into it while this runs. A torn
    /// read costs at most one stale sample of a value the next sweep re-derives.
    ///
    /// # Safety
    /// `base` must be 4-byte aligned with `words` readable words at `base`, and `from <= words`.
    #[allow(dead_code)]
    pub unsafe fn advance(base: *const u32, words: usize, from: usize) -> usize {
        let mut i = from;
        // SAFETY: the caller guarantees `words` readable words at `base`; `i < words` throughout.
        while i < words && unsafe { base.add(i).read_volatile() } == PAINT {
            i += 1;
        }
        i
    }

    /// One BOUNDED step of the high-water sweep: scan at most `chunk` words of `[cursor, mark)` and
    /// return `(next_cursor, next_mark)`.
    ///
    /// A full scan of the intact paint costs one read per free word, and on this image that is
    /// ~1,100 words. Paid every 250 Hz publish it would add ~56 us to a control callback that
    /// already measures 1,011 us on the F103 against a 1,041.7 us BLE character time, i.e. the
    /// instrument would start costing the BLE port the very inbound bytes another instrument exists
    /// to count. So the sweep is chunked: the per-pass cost is bounded by `chunk` regardless of how
    /// much stack is free, and a whole sweep completes over `ceil(mark / chunk)` passes.
    ///
    /// **Scanning rarely costs no accuracy**, which is what makes chunking free rather than a
    /// trade. The painted region is a LATCH: destroyed paint is never restored, so the deepest touch
    /// is a property of the memory and not of when it is read. A sweep that arrives late still finds
    /// the same mark. This is the whole reason the mark is not sampled per pass.
    ///
    /// The sweep restarts from 0 every time rather than resuming above the previous mark, because a
    /// DEEPER excursion destroys paint BELOW the old mark, where a cursor carried upward would never
    /// look again. Restarting is also what keeps this exactly equivalent to a full rescan.
    ///
    /// # Safety
    /// `base` must be 4-byte aligned with at least `mark` readable words at `base`, and
    /// `cursor <= mark`.
    #[allow(dead_code)]
    pub unsafe fn sweep(
        base: *const u32,
        cursor: usize,
        mark: usize,
        chunk: usize,
    ) -> (usize, usize) {
        let end = mark.min(cursor.saturating_add(chunk));
        // SAFETY: `end <= mark` and the caller guarantees `mark` readable words at `base`.
        let i = unsafe { advance(base, end, cursor) };
        if i < end {
            // A dirty word inside this window: it is the new (deeper) mark. Restart the sweep.
            (0, i)
        } else if end >= mark {
            // Swept all of `[0, mark)` with the paint intact: the mark stands. Restart the sweep.
            (0, mark)
        } else {
            // More window to cover on later passes.
            (end, mark)
        }
    }

    /// Pack the published word: free bytes in the low half, painted bytes in the high half.
    ///
    /// Two saturating `u16` halves that cannot wrap into each other (the `motor_fault` /
    /// `ble_rx_losses` packing precedent), because they answer one question between them: the
    /// denominator is what distinguishes a MEASURED margin from "the paint was never reached, so
    /// the margin is at least this". `memory.x` sizes RAM to the smallest part (8 KiB) for every
    /// board, so neither half can legitimately exceed a `u16`; saturating rather than truncating
    /// means a future layout change reports an implausible maximum instead of a small, believable,
    /// wrong number.
    #[allow(dead_code)]
    pub fn pack(free_bytes: usize, painted_bytes: usize) -> u32 {
        (free_bytes.min(0xFFFF) as u32) | ((painted_bytes.min(0xFFFF) as u32) << 16)
    }
    #[cfg(test)]
    mod tests {
        use super::{advance, pack, paint, sweep, PAINT};

        /// Run the chunked sweep for `passes` passes from a cold start and return the mark it holds.
        /// The firmware drives exactly this loop, one pass per 250 Hz publish.
        ///
        /// # Safety
        /// `base` must have `words` readable words.
        unsafe fn settle(base: *const u32, words: usize, chunk: usize, passes: u32) -> usize {
            let (mut cursor, mut mark) = (0usize, words);
            for _ in 0..passes {
                // SAFETY: the caller guarantees `words` readable words, and the sweep maintains
                // `cursor <= mark <= words` across passes.
                let (c, m) = unsafe { sweep(base, cursor, mark, chunk) };
                cursor = c;
                mark = m;
            }
            mark
        }

        /// A region really carries the pattern after a paint, and the paint is the ONLY thing that
        /// put it there (the region starts as something else entirely).
        #[test]
        fn paint_fills_the_whole_region() {
            let mut region = vec![0xAAAA_5555u32; 64];
            // SAFETY: an owned, exclusively-borrowed allocation of exactly 64 words.
            unsafe { paint(region.as_mut_ptr(), region.len()) };
            assert!(
                region.iter().all(|&w| w == PAINT),
                "the paint left a word unpainted"
            );
        }

        /// The paint writes INSIDE its length and nowhere else. This is the property that says a
        /// paint bounded at `_stack_end` cannot reach down into `.uninit` and destroy `CTRL_OBS`
        /// (its magic, its reset-surviving boot counter, and every field the bench reads), which is
        /// the one way this instrument could corrupt a boot in a way that looks like anything else.
        #[test]
        fn paint_stays_inside_its_length() {
            let mut buf = vec![0u32; 68];
            let below: u32 = 0xDEAD_0001;
            let above: u32 = 0xDEAD_0002;
            buf[0] = below;
            buf[1] = below;
            buf[66] = above;
            buf[67] = above;
            // SAFETY: paints buf[2..66], 64 words inside a 68-word owned allocation.
            unsafe { paint(buf[2..].as_mut_ptr(), 64) };
            assert_eq!(buf[0], below, "the paint wrote BELOW its base");
            assert_eq!(buf[1], below, "the paint wrote BELOW its base");
            assert_eq!(buf[66], above, "the paint ran PAST its length");
            assert_eq!(buf[67], above, "the paint ran PAST its length");
            assert!(buf[2..66].iter().all(|&w| w == PAINT));
        }

        /// A zero-length paint touches nothing (the degenerate region: statics grown until there is
        /// no stack left to paint, which the firmware reports as a zero margin rather than painting
        /// through whatever sits below).
        #[test]
        fn a_zero_length_paint_writes_nothing() {
            let mut buf = vec![0xDEAD_0003u32; 4];
            // SAFETY: zero words at a valid base.
            unsafe { paint(buf.as_mut_ptr(), 0) };
            assert!(buf.iter().all(|&w| w == 0xDEAD_0003));
        }

        /// The high-water computation against a synthetic painted region with a KNOWN deepest
        /// touch: 64 words painted, the top 20 overwritten (a stack that grew down to word 44), so
        /// 44 words of paint survive and the margin is 44 words.
        #[test]
        fn the_scan_finds_a_known_deepest_touch() {
            let mut region = vec![0u32; 64];
            // SAFETY: an owned 64-word allocation.
            unsafe { paint(region.as_mut_ptr(), region.len()) };
            for (i, w) in region.iter_mut().enumerate().skip(44) {
                *w = 0x2000_1000 + i as u32; // plausible stack traffic: addresses, not the pattern
            }
            // SAFETY: same allocation, read-only.
            assert_eq!(unsafe { advance(region.as_ptr(), region.len(), 0) }, 44);
            // And the chunked sweep the firmware actually runs reaches the same answer.
            // SAFETY: same allocation, read-only.
            assert_eq!(unsafe { settle(region.as_ptr(), 64, 8, 20) }, 44);
        }

        /// An untouched region reports the whole region as free: the "the paint was never reached"
        /// reading, which the tool must be able to tell apart from a measured margin (it is the
        /// case where free == painted, and the margin is a LOWER bound rather than a measurement).
        #[test]
        fn an_untouched_region_reports_the_whole_region_free() {
            let mut region = vec![0u32; 32];
            // SAFETY: an owned 32-word allocation.
            unsafe { paint(region.as_mut_ptr(), region.len()) };
            // SAFETY: same allocation, read-only.
            assert_eq!(unsafe { advance(region.as_ptr(), region.len(), 0) }, 32);
            // SAFETY: same allocation, read-only.
            assert_eq!(unsafe { settle(region.as_ptr(), 32, 8, 20) }, 32);
        }

        /// A region whose very first word is already dirty reports zero free: the overflow reading,
        /// the one the >= 250 B floor policy exists to keep the fleet away from.
        #[test]
        fn a_touched_floor_reports_zero_free() {
            let mut region = vec![0u32; 32];
            // SAFETY: an owned 32-word allocation.
            unsafe { paint(region.as_mut_ptr(), region.len()) };
            region[0] = 0x4000_4400;
            // SAFETY: same allocation, read-only.
            assert_eq!(unsafe { advance(region.as_ptr(), region.len(), 0) }, 0);
            // SAFETY: same allocation, read-only.
            assert_eq!(unsafe { settle(region.as_ptr(), 32, 8, 20) }, 0);
        }

        /// THE load-bearing property of the chunked design: however the sweep is chunked, it settles
        /// on exactly what a single full rescan would report. The instrument spreads the scan over
        /// many 250 Hz passes to bound its per-pass cost, and that is only allowed to be an
        /// optimization, never a different answer.
        #[test]
        fn the_chunked_sweep_agrees_with_a_full_rescan() {
            for deepest in [0usize, 1, 17, 50, 63, 64] {
                let mut region = vec![0u32; 64];
                // SAFETY: an owned 64-word allocation.
                unsafe { paint(region.as_mut_ptr(), region.len()) };
                for w in region.iter_mut().skip(deepest) {
                    *w = 0x4000_4400;
                }
                // SAFETY: same allocation, read-only.
                let full = unsafe { advance(region.as_ptr(), region.len(), 0) };
                assert_eq!(full, deepest);
                for chunk in [1usize, 3, 8, 64, 4096] {
                    // 200 passes is far more than `ceil(64 / 1)` needs, so every chunk size has
                    // settled: the assertion is about the answer, not the pass count.
                    // SAFETY: same allocation, read-only.
                    let settled = unsafe { settle(region.as_ptr(), 64, chunk, 200) };
                    assert_eq!(
                        settled, full,
                        "chunk {chunk} settled on {settled}, a full rescan says {full}"
                    );
                }
            }
        }

        /// The mark only ever DEEPENS: a later, deeper excursion moves it down, and a sweep that
        /// runs afterwards cannot walk it back up. This is the property that makes the published
        /// word a high-water mark rather than a sample of whatever the stack was doing recently.
        ///
        /// It is also the case the first cut of this instrument got WRONG. Resuming the scan from
        /// the previous mark looks like a sound amortization and is not: a deeper excursion destroys
        /// paint BELOW the old mark, exactly where a cursor carried upward never looks again. The
        /// sweep restarts from 0 for that reason.
        #[test]
        fn the_mark_only_deepens() {
            let mut region = vec![0u32; 64];
            // SAFETY: an owned 64-word allocation.
            unsafe { paint(region.as_mut_ptr(), region.len()) };
            for w in region.iter_mut().skip(40) {
                *w = 0x4000_4400;
            }
            let (mut cursor, mut mark) = (0usize, 64usize);
            for _ in 0..40 {
                // SAFETY: same allocation, read-only.
                let (c, m) = unsafe { sweep(region.as_ptr(), cursor, mark, 8) };
                cursor = c;
                mark = m;
            }
            assert_eq!(mark, 40, "the first excursion sets the mark");

            // A deeper excursion, destroying paint BELOW the current mark.
            for w in region.iter_mut().skip(25) {
                *w = 0x4000_4400;
            }
            for _ in 0..40 {
                // SAFETY: same allocation, read-only.
                let (c, m) = unsafe { sweep(region.as_ptr(), cursor, mark, 8) };
                cursor = c;
                mark = m;
            }
            assert_eq!(mark, 25, "a deeper touch must move the mark down");

            // Nothing ever repaints, so no later sweep can report more free stack than the deepest
            // excursion left behind.
            for _ in 0..200 {
                // SAFETY: same allocation, read-only.
                let (c, m) = unsafe { sweep(region.as_ptr(), cursor, mark, 8) };
                cursor = c;
                mark = m;
                assert_eq!(mark, 25, "the mark walked back up");
            }
        }

        /// The per-pass cost is bounded by the chunk, whatever the region: the sweep never reads
        /// more than `chunk` words in one pass. That bound is the reason this can ship inside a
        /// 250 Hz callback that is already tight against the BLE port's character time, so it is
        /// asserted rather than argued.
        #[test]
        fn one_pass_reads_at_most_a_chunk() {
            let mut region = vec![0u32; 1024];
            // SAFETY: an owned 1024-word allocation.
            unsafe { paint(region.as_mut_ptr(), region.len()) };
            let chunk = 64usize;
            let (mut cursor, mut mark) = (0usize, 1024usize);
            for _ in 0..64 {
                let before = cursor;
                // SAFETY: same allocation, read-only.
                let (c, m) = unsafe { sweep(region.as_ptr(), cursor, mark, chunk) };
                // Either the cursor advanced by at most a chunk, or the sweep wrapped to 0 having
                // covered at most a chunk on its way.
                if c != 0 {
                    assert!(
                        c > before && c - before <= chunk,
                        "one pass advanced {before} -> {c}, past the {chunk}-word bound"
                    );
                }
                cursor = c;
                mark = m;
            }
        }

        /// A whole sweep of a fully intact region completes in the expected number of passes, so
        /// the published mark is at most that many 250 Hz ticks stale.
        #[test]
        fn a_sweep_completes_in_ceil_words_over_chunk_passes() {
            let mut region = vec![0u32; 100];
            // SAFETY: an owned 100-word allocation.
            unsafe { paint(region.as_mut_ptr(), region.len()) };
            let (mut cursor, mut mark) = (0usize, 100usize);
            let mut passes = 0;
            loop {
                // SAFETY: same allocation, read-only.
                let (c, m) = unsafe { sweep(region.as_ptr(), cursor, mark, 8) };
                passes += 1;
                cursor = c;
                mark = m;
                if cursor == 0 {
                    break;
                }
                assert!(passes < 100, "the sweep never wrapped");
            }
            assert_eq!(passes, 100_usize.div_ceil(8), "100 words in 8-word chunks");
        }

        /// The published word's two halves are independent and cannot bleed into each other (the
        /// `motor_fault` / `ble_rx_losses` packing precedent).
        #[test]
        fn the_published_halves_do_not_bleed() {
            assert_eq!(pack(0, 0), 0);
            assert_eq!(pack(408, 4488), 408 | (4488 << 16));
            assert_eq!(pack(0xFFFF, 0xFFFF), 0xFFFF_FFFF);
            // Saturating, not wrapping: a region larger than a u16 can express must not report a
            // small number. (memory.x caps RAM at 8 KiB for every part, so this is a guard against
            // a future layout change, not a live case.)
            assert_eq!(pack(0x1_0000, 0x1_0000), 0xFFFF_FFFF);
            assert_eq!(pack(0x1_2345, 4), 0xFFFF | (4 << 16));
        }

        /// The pattern is not something the deep chain this instrument was built to measure would
        /// write. The round-9/11b high-water word was `0x4000_4400` (the USART1 base); return
        /// addresses are `0x0800_xxxx` and RAM pointers `0x2000_xxxx`.
        #[test]
        fn the_pattern_is_implausible_as_stack_traffic() {
            for plausible in [0x4000_4400u32, 0x0800_1234, 0x2000_0C6C, 0, u32::MAX] {
                assert_ne!(PAINT, plausible);
            }
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
