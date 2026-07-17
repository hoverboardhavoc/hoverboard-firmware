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
//!   2. **Link-listen (passive, DMA, 115200).** The remaining safe USARTs come up as L2 byte-stream
//!      links and just listen for L3 PDUs.
//!
//! Each live port becomes one of the board's `net` ports; the board stays at `0x00` until assigned,
//! then persists its `LINK_SET` (the bitmask of live ports) alongside its `node_address`. A
//! **configured** boot (non-zero `LINK_SET`) brings up exactly that set, never re-running the probe.
//!
//! The SWD mailbox is always port 0 (a debugger/host attaches over MEM-AP, no wiring); the discovered
//! USART links fill the remaining ports: **port 1 = the inter-board UART** (USART1 PA2/PA3, the proven
//! inter-board link), **port 2 = the BLE module** (USART2 PB10/PB11, the master's onboard CC2541).
//!
//! Pin safety (specs/l3.md, "Pin safety"): only the safe USARTs are touched (USART1 PA2/PA3, USART2
//! PB10/PB11 - clear of any advanced-timer gate pin). There is no motor code and nothing arms a
//! bridge. Busy-spin, NEVER `wfi` (a wfi with `DBG_CTL0 = 0` locks GD32 SWD re-attach).
//!
//! HAL gap scoped out: the spec's third safe USART, **USART0-remap (PB6/PB7)**, has no `runtime-hal`
//! AFIO-remap primitive yet - `runtime_hal::supports_rx` answers false for it, so its allowlist
//! entry is skipped at the call sites (the capability is the HAL model's answer, l3.md, never a
//! baked flag here). On the bench those pins are the IMU's I2C0 (no UART peer), so this costs no
//! bench coverage (the port would classify `empty`); the remap arrives with runtime-hal's
//! usart-pin-remap.md.
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

#[cfg(target_os = "none")]
mod firmware {

    use board::plumbing::{read_fields, reserved_set, AllowlistPort, BoardObs};
    use core::mem::MaybeUninit;
    use core::ptr::{addr_of, addr_of_mut};
    use core::sync::atomic::{AtomicU32, Ordering};
    use cortex_m::asm::nop;
    use cortex_m::peripheral::scb::SystemHandler;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m_rt::{entry, exception};
    use embedded_hal::digital::OutputPin;
    use embedded_io::{ErrorType, Read, ReadReady, Write};
    use link::{Link, SerialTransport};
    use linkctl::CyclicState;
    use net::walk::{Emits, Responder, PORT_BLE, PORT_SWD, PORT_UART};
    use orchestrator::{control_task, cyclic_tx, input_task, InputSample, Obs, OrchestratorState};
    use panic_halt as _;
    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::delay::Delay;
    use runtime_hal::descriptor::ClockPath;
    use runtime_hal::irq::{install, RamVectorTable, MAX_VECTORS};
    use runtime_hal::{
        detect_chip, FreeWatchdog, I2c, I2cMode, InputGroup, PeriphLabel, PolledSerial,
        RingBufferedRx, SplitSerial, Usart, WdgTimeout,
    };
    use scheduler::{systick_load, Scheduler};
    use store::{FmcFlash, Store, CONTROL_MODE, LINK_SET};
    use swd_mailbox::{EpochWatch, Mailbox, MailboxSerial, MAILBOX_BASE};

    /// This firmware's L3 protocol/firmware version, reported in `NODE_HELLO`.
    const FW_VER: u16 = 0x0001;

    /// The production 72 MHz tree (IRC8M -> PLL): the inter-board baud divisor + flash wait states,
    /// and the sysclk every delay converts against (read as `CLOCK.sysclk_hz`, one owner).
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    /// The inter-board / link-listen UART baud, from its one owner (`link::INTER_BOARD_BAUD`).
    const LINK_BAUD: u32 = link::INTER_BOARD_BAUD;
    /// The CC2541 module's AT-command baud (`ble::at::BAUD`).
    const BT_BAUD: u32 = ble::at::BAUD;

    /// The advertised BLE device name set by the AT bring-up. **Bump the suffix per bench run** so a
    /// scanner does not show a cached name for the module's (fixed) MAC - the "cached-name trap".
    const BLE_NAME: &str = "hb-s6a";
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
    /// The inter-board UART's L2 frame capacity (frag-hdr + chunk); a whole 64 B PDU rides one frame.
    const UART_FRAME_CAP: usize = 96;
    /// The BLE link's L2 frame capacity. The CC2541 bridge is a byte stream (it coalesces/re-chunks),
    /// so this is just the framing chunk size, sized so one stream frame fits a ~20 B BLE ATT write
    /// (SOF + len + 16 B L2 frame + CRC).
    const BLE_FRAME_CAP: usize = 16;
    /// Idle poll-cycles (no inbound) the responder waits, while probing, before emitting `PORTS`. Kept
    /// short so it fires within the controller's retransmit window (a long window lets each retransmitted
    /// `PROBE_PORTS` restart the probe and reset this counter, so `PORTS` never gets sent).
    const PROBE_IDLE: u32 = 50_000;

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
    /// `empty`).
    const N_PORTS: u8 = 3;

    /// VTOR alignment invariant: the `RAM_VECTORS` static (packed by memory.x's `.ramtables`
    /// section) carries `RamVectorTable`'s own alignment, so as long as the type stays `align(512)`
    /// the table is VTOR-valid (a runtime guard at the call site double-checks the placed address).
    const _: () = assert!(core::mem::align_of::<RamVectorTable>() >= 512);

    /// One entry in the safe-link USART allowlist (`specs/l3.md`, "Pin safety": gate-capable pins
    /// denied). `pins`/`baud` are filled at the call site; this records the spec's PIN-SAFETY
    /// allowlist + which `net` port slot the link takes. Whether `runtime-hal` can actually bring an
    /// entry up is NOT recorded here: that capability is the HAL model's answer
    /// (`runtime_hal::supports_rx`, per l3.md - never a baked consumer flag), queried per boot at
    /// the call sites.
    struct SafeLinkUsart {
        /// The HAL peripheral.
        usart: PeriphLabel,
        /// The `net` port slot this USART's link occupies. Doubles as the port's `LINK_SET` bit
        /// (`link_bit(port)` is what gets persisted), so it is also the `link_set_bit` the
        /// reserved-set computation keys the freeing rule on.
        port: u8,
        /// The entry's pin pair, packed `(port << 4) | pin`: the allowlist's PIN fact, the
        /// single declaration the board validator's reserved set is computed from
        /// (`specs/board-model.md` check 3). The bring-up call sites drive the same pins as
        /// typed handles (e.g. `gpioa.pa2`).
        pins: [u8; 2],
    }

    /// The spec's safe-UART allowlist (`specs/l3.md` §143): USART0-remap PB6/PB7, USART1 PA2/PA3,
    /// USART2 PB10/PB11. Gate-capable pins (USART0-default PA9/PA10) are denied - that is the
    /// allowlist's whole job. Entries the HAL cannot bring up yet (`supports_rx` false: USART0 until
    /// the AFIO remap primitive lands) are skipped at the call sites, costing no bench coverage (on
    /// the bench PB6/PB7 is the IMU's I2C, which would classify `empty` anyway).
    const SAFE_LINK_USARTS: [SafeLinkUsart; 3] = [
        // USART0-remap (PB6/PB7): allowlisted by the spec; supports_rx answers false until the HAL's
        // AFIO remap primitive exists, so the call sites skip it.
        SafeLinkUsart {
            usart: PeriphLabel::Usart0,
            port: 3,
            pins: [0x16, 0x17], // PB6/PB7
        },
        // USART1 (PA2/PA3): the inter-board link (both boards), the link-listen port.
        SafeLinkUsart {
            usart: PeriphLabel::Usart1,
            port: PORT_IDX_UART,
            pins: [0x02, 0x03], // PA2/PA3
        },
        // USART2 (PB10/PB11): the master's onboard CC2541 BLE module, the BT-probe port.
        SafeLinkUsart {
            usart: PeriphLabel::Usart2,
            port: PORT_IDX_BLE,
            pins: [0x1A, 0x1B], // PB10/PB11
        },
    ];

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
    /// IMU I2C rate: 100 kHz standard mode (the imu-bench gate image's silicon-proven rate).
    const IMU_I2C_HZ: u32 = 100_000;
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

    /// The 250 Hz SysTick ISR (R1 verbatim): advance the scheduler one tick, nothing else.
    /// Lowest priority (0xF0, set at bring-up) so comms IRQs preempt it.
    #[exception]
    fn SysTick() {
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
        /// b0 imu_configured, b1 imu_live, b2 comms_loss, b3 mode_fault.
        flags: u8,
        /// Reserved pad.
        _pad: u8,
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
    fn publish_obs(o: &Obs, boot_count: u32) {
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
                | ((o.mode_fault as u8) << 3),
            _pad: 0,
        };
        // SAFETY: the one writer (main thread), fixed symbol, volatile so the SWD reader sees
        // coherent-enough snapshots (a torn read across fields is acceptable diagnostics).
        unsafe { (addr_of_mut!(CTRL_OBS) as *mut CtrlObs).write_volatile(v) };
    }

    /// The 250 Hz control task (scheduler slot 0): sample the IMU (the firmware-side sampling
    /// wrapper; a failed read is `None` -> the pipeline's zero sample + `imu_live` false), run
    /// the pure pipeline, build the cyclic payload (step 8, addressed boards only; the loop
    /// sends it), publish OBS.
    fn control_task_cb() {
        // SAFETY: dispatch-callback context == the main thread; the loop's own shell borrows are
        // scoped to end before `dispatch()` (the execution-model discipline).
        let Some(shell) = (unsafe { (*addr_of_mut!(SHELL)).as_mut() }) else {
            return;
        };
        let sample = match (&mut shell.i2c, &mut shell.imu) {
            (Some(bus), Some(dev)) => dev.read(bus).ok(),
            _ => None,
        };
        let _out = control_task(&mut shell.orch, sample.as_ref());
        shell.cyclic_out = cyclic_tx(&shell.orch, shell.addressed);
        publish_obs(&shell.orch.obs(), shell.boot_count);
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
            });
        }
    }

    /// Validate the persisted board layout (specs/board-model.md checks 1-4): the reserved set
    /// is the compiled allowlist minus the LINK_SET-freed ports plus SWD (the plumbing helper
    /// owns the freeing rule; the allowlist pin facts come from SAFE_LINK_USARTS, their single
    /// owner), the fields arrive through the registry defaults, and the chip capabilities
    /// through the HalCaps adapter. On Ok, the success record + the BoardPlan the integration
    /// bring-up consumes (the IMU group, the input pins); on Err, the failure record naming the
    /// offending field, and the boot proceeds link-only (the fail-loud contract's posture).
    ///
    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget fix): the validator's
    /// working set (fields, reserved set, claims) lives here and is gone before the loop's deep
    /// ingest/append chain exists, instead of inflating `main`'s persistent frame.
    #[inline(never)]
    fn validate_layout<F: store::Flash>(
        chip: &runtime_hal::Chip,
        store: &Store<F>,
        link_set: u8,
    ) -> Option<board::BoardPlan> {
        let allowlist = SAFE_LINK_USARTS.map(|u| AllowlistPort {
            link_set_bit: u.port,
            pins: u.pins,
        });
        let reserved = reserved_set(&allowlist, link_set);
        let (obs, plan) = match board::validate(
            &read_fields(store),
            &HalCaps { chip },
            reserved.as_slice(),
            Some(BOOT_SELF_HOLD),
        ) {
            Ok(plan) => (BoardObs::success(), Some(plan)),
            Err(e) => (BoardObs::failure(&e), None),
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

    /// The compiled pre-mount self-hold assert pin (PB12, packed): "the pre-mount value of
    /// `board.self_hold`'s default" (`specs/board-model.md`, Migration), declared once here. The
    /// early-boot latch assert drives this same pin through its typed handle (`gpiob.pb12`,
    /// below); the validator reserves it against every field except `board.self_hold` itself.
    const BOOT_SELF_HOLD: u8 = 0x1C;

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
    /// "Observability"): magic, result code, the offending field's registry id + index, and the
    /// kind-specific detail word. Read it over SWD at the address of the `BOARD_OBS` symbol
    /// (`nm <elf> | grep BOARD_OBS`). A `static mut` with a fixed un-mangled symbol, written
    /// once per boot (before any interrupt is enabled) via a raw pointer, never through a
    /// reference: the `BLE_PROBE_OBS` pattern exactly.
    #[no_mangle]
    static mut BOARD_OBS: BoardObs = BoardObs {
        magic: 0,
        result: 0,
        field_id: 0,
        index: 0,
        pad: 0,
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

    /// Phase 1: the BT-probe (active, polled, 9600) on the safe USART that carries the module.
    ///
    /// On the bench the module is USART2 (the master's CC2541). Configured boards bring up exactly
    /// the link-set: re-establish the BLE link only if its port bit is set, and never probe a port
    /// outside the set. Unconfigured boards run the cheap probe; the one that answers AT+OK is it.
    /// The allowlist entry for the BLE port (USART2) is capability-gated by the HAL model
    /// (supports_rx: per-chip and self-updating, l3.md). The pins are consumed unconditionally
    /// (they are the BLE USART's; nothing else may claim them).
    ///
    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget fix): the serial, the
    /// probe tee, and the AT bring-up's working set live here and are gone before the loop's deep
    /// chains exist.
    #[inline(never)]
    fn bring_up_ble<TX, RX>(
        chip: &runtime_hal::Chip,
        delay: &mut Delay,
        pins: (runtime_hal::Pin<TX>, runtime_hal::Pin<RX>),
        configured: bool,
        link_set: u8,
    ) -> Option<BleLink> {
        let ble_usart = SAFE_LINK_USARTS
            .iter()
            .find(|u| u.port == PORT_IDX_BLE && runtime_hal::supports_rx(chip, u.usart))
            .map(|u| u.usart)?;
        let want_ble = if configured {
            link_set & link_bit(PORT_IDX_BLE) != 0
        } else {
            true // unconfigured: probe the allowlisted BLE port
        };
        if !want_ble {
            return None;
        }
        let serial = PolledSerial::new(chip, &CLOCK, ble_usart, pins, BT_BAUD).ok()?;

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
            // Transparent data mode after; the link rides the gate type itself.
            ble::Module::new(BLE_NAME)
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
            // Unconfigured + no AT+OK: not a module (e.g. the IMU's I2C0 on USART0-remap).
            None
        }
    }

    // The three concrete L2 links (heterogeneous serials, one L2 code path each).
    type MailboxLink = Link<SerialTransport<MailboxSerial, MAILBOX_FRAMER_N>, PACKET>;
    type UartLink = Link<SerialTransport<SplitSerial<RingBufferedRx>, UART_FRAMER_N>, PACKET>;
    // The BLE link rides the data-mode gate type (`ble::Pipe`, specs/ble.md): a link can only be
    // built on a serial KNOWN to be in transparent data mode (handshake arm: `bring_up`; fallback
    // arm: `Pipe::assume_data_mode` from the persisted link-set knowledge).
    type BleLink = Link<SerialTransport<ble::Pipe<PolledSerial>, BLE_FRAMER_N>, PACKET>;

    #[entry]
    fn main() -> ! {
        // Boot safe: nothing that could drive a motor is touched (no motor code).

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

        // Mount the store; read the persisted address + link-set (0/0 = a fresh, unconfigured board).
        let mut flash = FmcFlash::new(&chip);
        let mut store = Store::mount(&mut flash).unwrap();
        let link_set = store.get(LINK_SET);
        let configured = link_set != 0;

        // Validate the persisted board layout (a popped boot frame: see `validate_layout`).
        let plan = validate_layout(&chip, &store, link_set);

        // A SysTick busy-delay for the polled AT bring-up (phase 1, before any interrupt is enabled).
        // The application owns the one Peripherals::take() (runtime-hal DECISIONS #13: the HAL uses
        // raw register views internally and never consumes the one-shot flag, so ordering vs
        // detect_chip is unconstrained). take() after detect works; fail loud if somehow taken twice.
        let mut core = match cortex_m::Peripherals::take() {
            Some(p) => p,
            None => halt(),
        };
        let mut delay = Delay::new(core.SYST, CLOCK.sysclk_hz);

        // GPIO ports carrying the safe-USART pins (PA2/PA3 = USART1; PB10/PB11 = USART2).
        let gpioa = match chip.gpioa() {
            Ok(p) => p.split(),
            Err(_) => halt(),
        };
        let gpiob = match chip.gpiob() {
            Ok(p) => p.split(),
            Err(_) => halt(),
        };

        // SELF_HOLD (PB12) high, role-agnostic on EVERY board: latch this board's own power rail on so
        // it stays up after the inter-board wake drops. A slave is woken over the cable and would
        // otherwise fall back asleep once the master stops driving it; a master bridges its own power
        // button. Asserted here at boot because role is not yet known (identity is positional, assigned
        // later by the walk) and the latch cannot wait for it - so it must be unconditional, never gated
        // on chip family (family != master/slave). RoboDurden does the same (`main.c:148`). PB12 is
        // otherwise unused here (BLE = PB10/PB11, inter-board link = PA2/PA3). On the bench both boards
        // run on debugger 3V3 that bypasses the latch, so this is a no-op for power there (verifiable
        // only as `GPIOB.OCTL` bit 12 = 1); it matters on battery + the inter-board cable. PB12 is
        // the pin [`BOOT_SELF_HOLD`] declares packed (the pre-mount value of `board.self_hold`'s
        // default, which the validator reserved above); this is its typed-handle assert.
        let mut self_hold = gpiob.pb12.into_push_pull_output();
        let _ = self_hold.set_high();

        // === Phase 1: the BT-probe (active, polled, 9600): see `bring_up_ble` (a popped boot
        // frame; the probe/bring-up working set never joins `main`'s persistent frame). ===
        let mut ble_link: Option<BleLink> = bring_up_ble(
            &chip,
            &mut delay,
            (gpiob.pb10, gpiob.pb11),
            configured,
            link_set,
        );

        // === Phase 2: link-listen (passive, DMA, 115200) on the inter-board USART (port 1) ===
        //
        // Always brought up (both boards, every boot): it is the proven inter-board link. Configured
        // boards still bring it up iff its port bit is set (it always is for a walked board).
        let want_uart = !configured || link_set & link_bit(PORT_IDX_UART) != 0;
        // The allowlist entry for the inter-board link port (USART1), capability-gated by the HAL
        // model (supports_rx answers true for USART1 on both families).
        let uart_usart = SAFE_LINK_USARTS
            .iter()
            .find(|u| u.port == PORT_IDX_UART && runtime_hal::supports_rx(&chip, u.usart))
            .map(|u| u.usart)
            .unwrap_or(PeriphLabel::Usart1);

        // One bring-up, split into owned halves (specs/usart-split.md): the RX half is consumed by
        // RingBufferedRx below, the TX half drives polled TX. No second handle on a live base.
        let usart1 = match Usart::new(&chip, &CLOCK, uart_usart, (gpioa.pa2, gpioa.pa3), LINK_BAUD)
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
        // SAFETY (enable): the table is installed; RingBufferedRx::new registers + unmasks its handlers.
        unsafe { cortex_m::interrupt::enable() };
        // SAFETY: as RAM_VECTORS above: the one &mut formation, before the DMA IRQ exists.
        let dma_buf: &'static mut [u8; DMA_CAP] = unsafe { &mut *addr_of_mut!(DMA_RING) };
        let rx_dma = match RingBufferedRx::new(&chip, usart1_rx, uart_usart, dma_buf) {
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

        // The discovered link-set: the bitmask of live USART ports (persisted at assign, below).
        let discovered = (if uart_link.is_some() {
            link_bit(PORT_IDX_UART)
        } else {
            0
        }) | (if ble_link.is_some() {
            link_bit(PORT_IDX_BLE)
        } else {
            0
        });

        let mut responder =
            Responder::new(N_PORTS, [PORT_SWD, PORT_UART, PORT_BLE, 0], mcu, FW_VER);
        responder.restore_addr(&store);

        // === The integration boot delta (specs/integration.md, after the existing bring-up) ===

        // 1. IMU bring-up, plan-gated (the first BoardPlan consumer). The typed-pin seam: the
        //    HAL consumes named pin handles, and the one hardware-I2C pair whose handles are
        //    free here is I2C0 on PB6/PB7 (the silicon-proven standard-family IMU bus; the I2C1
        //    pair PB10/PB11 is the BLE USART's, consumed by that bring-up when live). Any other
        //    validated pair fails soft: the board boots link-only-plus-throttle and the outcome
        //    is observable (imu_configured in CTRL_OBS).
        let mut imu_bus: Option<I2c> = None;
        let mut imu_dev: Option<imu::Imu> = None;
        if let Some(ip) = plan.as_ref().and_then(|p| p.imu) {
            if ip.bus == 0 && ip.scl.packed() == 0x16 && ip.sda.packed() == 0x17 {
                if let Some(model) = imu::model_from_index(ip.model) {
                    if let Ok(mut bus) = I2c::new(
                        &chip,
                        &CLOCK,
                        PeriphLabel::I2c0,
                        (gpiob.pb6, gpiob.pb7),
                        I2cMode::standard(IMU_I2C_HZ),
                    ) {
                        let mut dev = imu::Imu::new(model, imu::Config::default());
                        if dev.probe(&mut bus).is_ok() && dev.init(&mut bus).is_ok() {
                            // The caller-owned post-init settle (specs/imu.md; the imu-bench
                            // pause) before the first cyclic read.
                            cortex_m::asm::delay((CLOCK.sysclk_hz / 1000) * IMU_SETTLE_MS);
                            imu_bus = Some(bus);
                            imu_dev = Some(dev);
                        }
                    }
                }
            }
        }

        // The plan-driven input pins (button + pads): resolve the configured ones into a
        // branch-free InputGroup; absent fields sample as idle through the per-line mask. Port C
        // (the fleet-default pad B, PC15) needs its clock enabled; A/B already are.
        let inputs = {
            let (b, pa, pb) = plan
                .as_ref()
                .map(|p| {
                    (
                        p.button.map(|x| x.packed()),
                        p.pad_a.map(|x| x.packed()),
                        p.pad_b.map(|x| x.packed()),
                    )
                })
                .unwrap_or((None, None, None));
            if [b, pa, pb].iter().flatten().any(|&x| (x >> 4) == 2) {
                let _ = chip.gpioc();
            }
            let filler = b.or(pa).or(pb);
            let group = filler.and_then(|f| {
                chip.input_group([b.unwrap_or(f), pa.unwrap_or(f), pb.unwrap_or(f)])
                    .ok()
            });
            InputPins {
                group,
                has_button: b.is_some(),
                has_pad_a: pa.is_some(),
                has_pad_b: pb.is_some(),
            }
        };

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
            sched.mark_tick_source_enabled();
        }
        let mut syst = delay.free();
        let load = match systick_load(CLOCK.sysclk_hz) {
            Some(l) => l,
            None => halt(), // fatal config error per the recovered contract (24-bit LOAD)
        };
        // SAFETY: priority write before the SysTick interrupt is enabled; 0xF0 = lowest (the
        // stock priority: comms IRQs preempt the tick).
        unsafe { core.SCB.set_priority(SystemHandler::SysTick, 0xF0) };
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
        let mut probe_idle: u32 = 0;
        let mut link_set_saved = configured; // once assigned, persist LINK_SET once
        let mut rxbuf = [0u8; PACKET];
        let mut pdu = [0u8; net::walk::MAX_PDU];
        // ONE reusable emissions scratch for every drain site + the probe window (the slice-7
        // stack-budget fix: four per-site `Emits` locals cost ~300 B each in main's persistent
        // frame; exactly one is ever live, so one cleared-and-reused instance is the honest
        // shape).
        let mut emits = Emits::new();

        // The cooperative service loop (the integration.md execution model): service the links,
        // dispatch the due tasks, feed the watchdog AFTER dispatch (R2), emit the cyclic.
        // Busy-spin, NEVER wfi.
        loop {
            // 1. Mailbox epoch handshake (the SWD bridge attaching): reset the framer, write epoch_ack.
            if epoch_watch.poll() {
                mailbox_link.transport_mut().reset();
                epoch_watch.ack();
            }

            let mut saw_inbound = false;

            // 2a. Drain the mailbox link (port 0). `poll_recv` borrows `rxbuf` (not the link) and the
            //     `.map(copy_pdu)` consumes that borrow, so the scrutinee is a plain length: the link
            //     is free in the body to ingest and route the emissions back across every link.
            while let Some(n) = mailbox_link
                .poll_recv(&mut rxbuf)
                .map(|f| copy_pdu(f, &mut pdu))
            {
                saw_inbound = true;
                emits.clear();
                let handed = responder.ingest(PORT_IDX_MAILBOX, &pdu[..n], &mut store, &mut emits);
                route_emits(&emits, &mut mailbox_link, &mut uart_link, &mut ble_link);
                route_handback(handed);
            }

            // 2b. Drain the inter-board UART link (port 1), if it came up.
            while let Some(n) = uart_link
                .as_mut()
                .and_then(|l| l.poll_recv(&mut rxbuf))
                .map(|f| copy_pdu(f, &mut pdu))
            {
                saw_inbound = true;
                emits.clear();
                let handed = responder.ingest(PORT_IDX_UART, &pdu[..n], &mut store, &mut emits);
                route_emits(&emits, &mut mailbox_link, &mut uart_link, &mut ble_link);
                route_handback(handed);
            }

            // 2c. Drain the BLE link (port 2), if a module was brought up.
            while let Some(n) = ble_link
                .as_mut()
                .and_then(|l| l.poll_recv(&mut rxbuf))
                .map(|f| copy_pdu(f, &mut pdu))
            {
                saw_inbound = true;
                emits.clear();
                let handed = responder.ingest(PORT_IDX_BLE, &pdu[..n], &mut store, &mut emits);
                route_emits(&emits, &mut mailbox_link, &mut uart_link, &mut ble_link);
                route_handback(handed);
            }

            // 3. Probe window: once probing, wait out a short idle, then emit PORTS.
            if responder.probing() {
                probe_idle = if saw_inbound {
                    0
                } else {
                    probe_idle.saturating_add(1)
                };
                if probe_idle >= PROBE_IDLE {
                    emits.clear();
                    responder.poll_probe(&mut emits);
                    route_emits(&emits, &mut mailbox_link, &mut uart_link, &mut ble_link);
                    probe_idle = 0;
                }
            } else {
                probe_idle = 0;
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
                _ => {} // no port 3+ on this board (USART0-remap deferred; see SAFE_LINK_USARTS)
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
