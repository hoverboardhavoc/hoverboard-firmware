// SPDX-License-Identifier: GPL-3.0-or-later
//! The firmware binary: one image per chip, chosen by its chip feature (Cargo.toml `[features]`:
//! `gd32f103c8`, `gd32f130c8`, `gd32f103rc`; `cargo image-f103`, `cargo image-f130`,
//! `cargo image-f103rc`), on the `hal` crate
//! (specs/hal.md). Each image is built for its chip: no detection, the chip's flash vector table,
//! its own flash and SRAM.
//!
//! It wires the libraries it does not own (`store` + `FmcFlash`, `net`'s L3 `Responder`, `link`'s L2,
//! the `swd-mailbox`, and `hal`'s chip context / clock / USART) into one cooperative service loop:
//! boot-safe -> init the SWD mailbox -> take the chip -> 72 MHz clock -> mount the store -> **bring
//! up what the layout configures** (the links, the IMU, the inputs, the motor) -> service them
//! forever.
//!
//! **Three ports** (`specs/configured-links.md` section 4). The SWD mailbox is always port 0 (a
//! debugger, a host or the hoverboard bridge attaches over MEM-AP, no wiring); ports 1 and 2 are the
//! two inter-board links, each brought up from its stored pins (`links[i].tx`/`links[i].rx`) when both are
//! set. A board with nothing stored brings up its mailbox and no UART, and is configured through the
//! bridge on its SWD header (`specs/hoverboard-bridge.md`). A board stays at `0x00` until assigned,
//! then persists its `node_address`.
//!
//! **Each subsystem brings up what it can** (`specs/configured-links.md` section 3). There is no
//! layout validator: each bring-up reads its own fields (`crate::layout`) and asks the HAL, and a
//! failure leaves that function absent, adds an entry to the status record (`BOARD_OBS`, served as
//! `STATUS_READ` and flagged by `EV_CONFIG` on the wire) and never halts the boot. The IMU, for
//! one, sits on mirrored pins across the fleet (I2C0's PB6/PB7 on the standard family, I2C1's
//! PB10/PB11 on the classywalk offroad family, on the same GD32F130 part), so the stored layout
//! decides. Busy-spin, NEVER `wfi` (a wfi with `DBG_CTL0 = 0` locks GD32 SWD re-attach).
//!
//! **The integrated control stack (specs/integration.md, slice 7).** On top of the link spine the
//! image runs the orchestrated pipeline: the SysTick ISR ticks the ISR-safe `scheduler` static at
//! 250 Hz (R1); the loop drains the links (routing the delivered control-block PDUs through
//! `linkctl::decode` into the orchestrator inbox), dispatches the due tasks (the 250 Hz control
//! pipeline + the 16 ms input task, both pure `orchestrator` functions over the `SHELL` static),
//! feeds the IWDG AFTER dispatch (R2, the wdg-bench placement), emits the cyclic payload
//! port-directed on each inter-board link (addressed boards only) and samples the arm fact into the
//! responder each pass (R4). The IMU comes up from its staged pins (fail-soft to
//! link-only-plus-throttle); MOE
//! enactment stays a recorded seam (R3, pre-motor). Statics safety: the `SHELL` is touched only
//! from the main thread (the loop and the dispatch callbacks are the same context); the
//! `SCHEDULER` is the one ISR/thread crossing and its atomics are the arbitration.
//!
//! On a host target it degrades to an empty `main` (it cannot link as a cortex-m image nor the
//! target-gated HAL), so a host `cargo build`/`cargo test` over the workspace stays green.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

// Which image this build is (Cargo.toml `[features]`): one chip feature is that chip's image on
// `hal` (specs/hal.md). None or two is refused on the target; the host build needs none.
#[cfg(any(
    all(feature = "gd32f103c8", feature = "gd32f130c8"),
    all(feature = "gd32f103c8", feature = "gd32f103rc"),
    all(feature = "gd32f103c8", feature = "mm32spin05"),
    all(feature = "gd32f130c8", feature = "gd32f103rc"),
    all(feature = "gd32f130c8", feature = "mm32spin05"),
    all(feature = "gd32f103rc", feature = "mm32spin05")
))]
compile_error!(
    "firmware: enable one chip feature (gd32f103c8, gd32f130c8, gd32f103rc or mm32spin05), not two"
);
#[cfg(all(
    target_os = "none",
    not(any(
        feature = "gd32f103c8",
        feature = "gd32f130c8",
        feature = "gd32f103rc",
        feature = "mm32spin05"
    ))
))]
compile_error!(
    "firmware: an image is built for one chip; enable gd32f103c8, gd32f130c8, gd32f103rc or \
     mm32spin05 (cargo image-f103, cargo image-f130, cargo image-f103rc, cargo image-mm32)"
);

mod arm;
mod layout;
mod motor;
mod observer;
mod odometer;
/// The host tests' in-RAM `store::Flash`, one copy for the whole crate (see the module).
#[cfg(test)]
mod test_flash;
mod tones;

#[cfg(target_os = "none")]
mod firmware {

    use crate::arm;
    use crate::guest_tx::{
        emit_mailbox_samples, encode_cyclic, more_motors, GuestSlots, GuestStaged, MoreMotors,
        OWN_LEN,
    };
    use crate::imu_attempts;
    use crate::layout;
    use crate::link_drain::bounded_drain;
    use crate::motor;
    use crate::odometer;
    use crate::probe_window::poll_window_elapsed;
    use crate::tones;
    use board::status::{Reason, StatusRecord};
    use core::mem::MaybeUninit;
    use core::ptr::{addr_of, addr_of_mut};
    use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
    use cortex_m::asm::nop;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m_rt::entry;
    use embedded_hal::digital::OutputPin;
    use hal::clock::{self, ClockConfig};
    use hal::{
        DescriptorError, DmaTx, FreeWatchdog, I2c, I2cMode, InputGroup, PeriphLabel,
        RingBufferedRx, SplitSerial, Usart, WdgTimeout,
    };
    use link::{Link, SerialTransport};
    use linkctl::{ChipTag, CyclicMotor, CyclicState};
    use net::walk::{Emits, Identity, Responder, CLASS_MOTOR, PORT_SWD, PORT_UART};
    use orchestrator::{
        ble_cyclic_tx, ble_peer_tx, control_task, cyclic_tx, input_task, BoardIdentity,
        InputSample, Obs, OrchestratorState,
    };
    use panic_halt as _;
    use scheduler::{systick_load, Scheduler};
    use store::{
        FmcFlash, Store, ATTITUDE_LEVEL_TRIM, BOARD_BATTERY_CAL, CONTROL_REFERENCE,
        CONTROL_SF_GAINS, IMU_GYRO_BIAS,
    };
    use swd_mailbox::{EpochWatch, Mailbox, MailboxSerial, MAILBOX_BASE};

    /// This firmware's version, reported in `IDENTITY`.
    const FW_VER: u16 = 0x0001;

    /// The production 72 MHz tree (IRC8M -> PLL; the MM32SPIN05's HSI -> PLL): the inter-board
    /// baud divisor + flash wait states, and the sysclk every delay converts against (read as
    /// `CLOCK.sysclk_hz`, one owner).
    #[cfg(not(feature = "mm32spin05"))]
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
    #[cfg(feature = "mm32spin05")]
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_HSI;
    /// The inter-board / link-listen UART baud, from its one owner (`link::INTER_BOARD_BAUD`).
    const LINK_BAUD: u32 = link::INTER_BOARD_BAUD;

    /// Each L2 link's reassembly buffer (the largest packet a link reassembles). The links carry
    /// single-fragment L3/config PDUs (<= `net::walk::MAX_PDU` = 64 B); 72 B holds a whole PDU with a
    /// little margin while keeping the `Link`s small for the 8 KiB-image RAM budget (the floor is
    /// `MAX_PDU` = 64).
    const PACKET: usize = 72;
    /// Per-link `StreamFramer` buffers (`frame_capacity + 4`, the `SerialTransport` rule: the
    /// SOF/len and CRC bytes around the carrier's largest frame). Sized to EACH carrier instead
    /// of one shared 132-byte max (the slice-7 stack budget).
    const MAILBOX_FRAMER_N: usize = swd_mailbox::FRAME_CAPACITY + 4; // 132
    const UART_FRAMER_N: usize = UART_FRAME_CAP + 4; // 76
    /// The DMA RX ring for the inter-board USART (circular DMA + USART IDLE). >= the max wire frame
    /// with margin for a back-to-back burst.
    const DMA_CAP: usize = 128;
    /// The inter-board UART's L2 frame capacity (frag-hdr + chunk); a whole 64 B PDU rides one
    /// frame. Right-sized 96 -> 72 (round-7 stack reclaim, audit-approved): the largest L2 frame
    /// this port ever carries is a MAX_PDU (64 B) + the 1-byte frag-hdr = 65 <= 71 usable, so 96
    /// bought nothing but 24 B of framer buffer + 24 B of send scratch on the deep drain chain.
    const UART_FRAME_CAP: usize = 72;
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
    /// round-trip so a real neighbour is seen (it was sized when the CC2541's 9600-baud data-mode
    /// bridge was a port, the slowest medium the board ever had: ~100 ms worst case); and **shorter** than the controller's `PROBE_PORTS` retransmit (3 s, `swd-bridge`'s
    /// `run_walk`) so `PORTS` fires on the FIRST probe on every board regardless of loop speed.
    ///
    /// 125 ticks = 500 ms sits in that gap: >3x margin over that ~100 ms round-trip (neighbours
    /// are never missed) and 6x under the 3 s retransmit (the walk always advances on the first
    /// probe). DEPENDS on the firmware SysTick tick being live (the slice-7
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

    /// Port indices (fixed slots, `specs/configured-links.md` section 4): 0 = the SWD mailbox, 1 =
    /// link 0, 2 = link 1. Link `i` is port `PORT_IDX_LINK0 + i`.
    const PORT_IDX_MAILBOX: u8 = 0;
    const PORT_IDX_LINK0: u8 = 1;
    /// How many inter-board links a board can have (`store::LINKS`), as an array length.
    const N_LINKS: usize = store::LINKS as usize;
    /// The board's `net` ports: the mailbox and both link ports, on every boot. A link that is not
    /// configured or did not come up is still a REGISTERED port: an emission to it drops, and a
    /// `PROBE_PORTS` reports it `empty`.
    const N_PORTS: u8 = 1 + store::LINKS;

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
        assert!(hal::i2c::worst_case_block_us(CLOCK.sysclk_hz) * 2 < COAST_US);
    const _: () = IMU_READ_BLOCK_FITS_COAST;

    /// The same assertion for the OTHER polled peripheral the main loop blocks on: the serial
    /// write path (`hal::serial`'s write budget).
    ///
    /// One pass can take more than one write call (the inter-board cyclic emission, plus a PDU
    /// forwarded to the peer), and they share the pass with the control task that has to refresh
    /// the demand word before the coast expires. The same factor of two, for the same reason.
    ///
    /// This bound did not exist before 2026-10-22: every wait on that path was an unbounded
    /// `while !flag {}`, and the offroad master answered it with eleven independent-watchdog
    /// resets in one bench session (`RCU_RSTSCK` bit 29 set on the master, clear on its peer).
    const UART_WRITE_BLOCK_FITS_COAST: () =
        assert!(hal::serial::worst_case_write_block_us(CLOCK.sysclk_hz) * 2 < COAST_US);
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

    /// Link 0's DMA RX ring. NOT `.bss`: it lives in cortex-m-rt's `.uninit` (shrink round 7), which
    /// the zero loop never visits, so the link bring-up zeroes it explicitly at the one place the
    /// `&'static mut` is formed. Nothing reads a ring byte the DMA has not written
    /// (the reader stays behind the live write index), so the zeroing keeps the SWD-visible bytes
    /// identical to the `.bss` days rather than guarding a read.
    #[link_section = ".uninit.DMA_RING"]
    static mut DMA_RING: MaybeUninit<DmaRing<DMA_CAP>> = MaybeUninit::uninit();

    /// Link 1's DMA RX ring, in cortex-m-rt's `.uninit` (the gap's free 64 B are too small for its
    /// 128 B; `specs/configured-links.md` section 4), zeroed where it is formed as ring 0 is.
    #[link_section = ".uninit.DMA_RING_1"]
    static mut DMA_RING_1: MaybeUninit<DmaRing<DMA_CAP>> = MaybeUninit::uninit();

    /// A DMA ring's storage, 4-aligned. The alignment is for the ZEROING, not the DMA: `[0; N]`
    /// written through a pointer the compiler knows is word-aligned lowers to the `memclr4` the
    /// image already links, where a plain `[u8; N]` (align 1) pulled in a second, byte-wise
    /// `__aeabi_memset` body (measured +92 B).
    #[repr(C, align(4))]
    struct DmaRing<const N: usize>([u8; N]);

    /// Each link's DMA TX buffer (.bss), index = the link. A DMA transfer reads its source for as
    /// long as it runs, so the transmitter owns this copy rather than the caller's frame. Sized to
    /// [`UART_FRAMER_N`], the largest encoded frame a link can emit, so a whole frame always goes in
    /// ONE transfer and never comes back truncated.
    static mut LINK_TX_DMA: [[u8; UART_FRAMER_N]; N_LINKS] = [[0; UART_FRAMER_N]; N_LINKS];

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
    /// Mailbox port DROPPED-FRAME count (OBS): the L2 frames the SWD mailbox refused since boot,
    /// sampled from the link each loop pass and read at publish. The second port's loss counter,
    /// beside the inter-board port's two, and the only one that counts an OUTBOUND loss: those
    /// count inbound bytes the receiver missed, this one counts replies the board could not place (`specs/swd-mailbox.md`, "Backpressure", requirement 4).
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

    /// Write calls the inter-board port's serial adapter ABANDONED because a transmit flag did not
    /// come up inside its write budget (`hal::serial`'s `tx_timeouts`, saturating u16): link
    /// 0's in the low half, link 1's in the high half (`specs/configured-links.md` section 4).
    ///
    /// **The expected reading is zero**, and a non-zero one is specific: the port
    /// stopped accepting bytes and the frame was dropped rather than the main thread spinning on
    /// it. Before that bound existed the same condition was an unbounded spin, and the offroad
    /// master answered it with eleven watchdog resets in one session (2026-10-22). Published
    /// because the alternative is inferring it: a 200 ms main-thread pause is ~50 capped 4 ms
    /// waits, and that is a prediction a counter can settle and an argument cannot.
    static TX_TIMEOUTS: AtomicU32 = AtomicU32::new(0);

    /// Why this board last reset: the RCU `RSTSCK` cause field, read and cleared in `main`
    /// immediately after the chip context gives the RCU base, at the register bit positions the
    /// manuals give. Published as `CTRL_OBS` word 38, where the field's documentation lives.
    ///
    /// `.bss`, written once on the boot path and read-only thereafter, so the 250 Hz publish
    /// re-publishes the same value every pass.
    static RESET_CAUSE: AtomicU32 = AtomicU32::new(0);

    /// The IMU bring-up's attempt count ([`imu_attempts::ImuAttempts::retries`]): 0 the first probe and init
    /// answered, 1 the second did after a bus clear, 2 the IMU did not come up. Published as
    /// `CTRL_OBS` word 51 (`specs/imu-probe-retry.md`).
    ///
    /// `.bss`, written once on the boot path, as [`RESET_CAUSE`]. A board that never probed (no IMU
    /// staged, or a refusal before the bus) reads 0 here; the flags byte's b0 and b6 tell those apart.
    static IMU_RETRIES: AtomicU8 = AtomicU8::new(0);

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
    /// It was sized against the 250 Hz control callback's margin under the CC2541 port's 9600-baud
    /// character time (1,011 us measured on the F103 against 1,041.7 us,
    /// `specs/bench-evidence/2026-08-02/wide-division/RECORD.md`): ~30 us, of which 4.0 us is 13%.
    /// That port is gone, and the bound it set is kept: an UNCHUNKED full scan of the painted
    /// region would be tens of microseconds on every publish, which is the specific way a
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
    /// static data. About 3,600 B on the release image since shrink round 7 (2026-10-27:
    /// `_stack_end` 0x2000_0C74, `main`'s frame 1,312 B); it was 661 words / 2,644 B on
    /// 2026-08-11 and 2,196 B on 2026-10-26, when `main`'s frame was 2,248 B.
    ///
    /// That is deliberately NOT the whole region between `_stack_end` and `_stack_start`, and the
    /// difference is not waste: `main`'s own frame, which since round 7 is the loop's state
    /// (`LoopState`) and nothing else, stays live for the entire run, so the stack below it is the
    /// only stack anything can ever free up. Painting through a live frame would destroy this
    /// function's return address. The bring-up itself (`boot`) runs BELOW this line in a frame
    /// that pops, so its depth is measured by the instrument like any other chain's.
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

    /// The 250 Hz tick (R1 verbatim), the SysTick vector in the chip's flash vector table: the
    /// entry count `CTRL_OBS` publishes, then advance the scheduler one tick, nothing else. Lowest
    /// priority (0xF0, set at bring-up) so comms IRQs preempt it.
    #[cortex_m_rt::exception]
    fn SysTick() {
        hal::irq::SYSTICK_ISR_METRIC.record();
        // SAFETY: shared access to the scheduler static; `tick(&self)` is the crate's ISR-safe
        // entry (atomics inside), sound against the thread-side `dispatch(&self)`.
        unsafe { (*addr_of!(SCHEDULER)).tick() };
        // The Cortex-M0 has no read-modify-write atomics (specs/hal.md section 6); this handler is
        // the counter's one writer, so a load and a store are the whole update there.
        #[cfg(target_has_atomic = "32")]
        TICK_COUNT.fetch_add(1, Ordering::Relaxed);
        #[cfg(not(target_has_atomic = "32"))]
        TICK_COUNT.store(
            TICK_COUNT.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
    }

    /// The descriptor's identity bytes are values the wire allocates: the `NODE_HELLO` mcu byte is
    /// one of `net`'s family tags and the cyclic chip tag names a part (`ChipTag::Unknown` is the
    /// tag for "cannot name it", which an image built for its chip always can).
    ///
    /// Not on the MM32SPIN05: its descriptor's bytes (4 and 4) have no `net` or `linkctl` name,
    /// because its image is a CI-only size build that never runs (specs/hal.md section 9, step 3).
    /// The names come with an image that can be installed.
    #[cfg(not(feature = "mm32spin05"))]
    const CHIP_IDENTITY_IS_ALLOCATED: () = {
        let d = &hal::descriptor::DESCRIPTOR;
        assert!(d.node_mcu == net::walk::MCU_F10X || d.node_mcu == net::walk::MCU_F1X0);
        assert!(!matches!(ChipTag::from_u8(d.chip_tag), ChipTag::Unknown));
    };
    #[cfg(not(feature = "mm32spin05"))]
    const _: () = CHIP_IDENTITY_IS_ALLOCATED;

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
        orch: OrchestratorState<{ motor::COUNT }>,
        /// The plan-gated IMU bus + driver (`None` = not configured / probe failed: fail-soft).
        i2c: Option<I2c>,
        imu: Option<imu::Imu>,
        inputs: InputPins,
        /// Step 8's pending cyclic payload: the 250 Hz callback builds it (addressed boards
        /// only); the loop empties it port-directed onto the inter-board UART.
        cyclic_out: Option<CyclicState>,
        /// [`Self::cyclic_out`]'s motor-1 blocks ([`MoreMotors`]), written with it: on a two-motor
        /// image motor 1's words, on a one-motor image nothing.
        cyclic_more: MoreMotors,
        /// The guest port's samples ([`GuestSlots`]): the board's own decimated cyclic payload
        /// (5 Hz, `dispatch::ble_cyclic_tx`) and the PEER's forward (2.5 Hz,
        /// `dispatch::ble_peer_tx`), each a latest-wins slot separate from `cyclic_out` because
        /// the ports run at different rates. The mailbox port takes a sample in the pass after it
        /// was built (`specs/link-control.md`, "The mailbox port carries both samples
        /// (2026-10-25)").
        guest: GuestSlots,
        /// Samples the mailbox port accepted, `CTRL_OBS` word 41 ([`GuestStaged`]).
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
        /// The flux observer's live words and the facts its parameters are derived from
        /// (`crate::observer`): the tune lane writes them, a persisted write reconciles them, the
        /// arm row gives them the calibration, and every change installs the derived parameters
        /// into the period ISR (`motor::hw::install_observer`).
        observer: crate::observer::ObserverLoop,
        /// The period ISR's over-current trip count as of the previous control run: a change
        /// raises the motor-0 fault latch (`specs/motor-integration.md`, "The hard trip").
        last_trips: [u32; motor::COUNT],
        /// Each motor's DC-link limit and its derate (`specs/optional-protections.md` section
        /// 3.2), `None` without `current.dc_link.pin`: taken from the bring-up's boot-fixed facts
        /// once at boot and stepped by this task alone after it.
        #[cfg(feature = "gd32f103rc")]
        dc_link: [Option<motor::DcLinkLimit>; motor::COUNT],
        /// The tone player (`specs/tones.md` section 4.2), stepped once per served tick.
        player: tones::Player,
        /// The battery alarm's two filtered levels (section 4.4).
        alarm: tones::BatteryAlarm,
        /// `machine.battery.warn`, centivolts, read once at boot (`<= 0` = no warning).
        battery_warn: i16,
    }

    /// The three tunes as the player reads them: the store's current bytes of each index of the tone field,
    /// borrowed for the one call (`specs/tones.md` section 4.2).
    struct StoreTunes<'a>(&'a Store<'static, FmcFlash>);

    /// Each event's tune, the tone field at that event's index, by the event's discriminant: an
    /// indexed load rather than a match.
    const TUNES: [store::BlobField; 3] = {
        let mut t = [store::TONE.at(store::TONE_HORN); 3];
        t[tones::Event::BatteryCritical as usize] = store::TONE.at(store::TONE_BATTERY_CRITICAL);
        t[tones::Event::BatteryLow as usize] = store::TONE.at(store::TONE_BATTERY_LOW);
        t
    };

    impl tones::Tunes for StoreTunes<'_> {
        fn with<R>(&self, e: tones::Event, f: impl FnOnce(&[u8]) -> R) -> R {
            f(self.0.get_bytes(TUNES[e as usize]))
        }
    }

    /// One served tick of the tones (`specs/tones.md` section 4): the battery alarm's levels from
    /// the block's effective battery word against the floor in force and the boot-read warning,
    /// the horn from the `INPUTS` mirror, then the player, whose period goes to the period ISR.
    ///
    /// `#[inline(never)]` with no `#[link_section]`, the `re_read_arm_values` placement: it runs
    /// once per tick but reads the store only at a note start, so it stays out of the F1x0's
    /// zero-wait window that the 16 kHz and 250 Hz paths need.
    #[inline(never)]
    fn tone_tick(shell: &mut Shell, dt_ticks: u32) {
        let dt_ms = dt_ticks.saturating_mul(tones::TICK_MS);
        let (battery_critical, battery_low) = shell.alarm.step(
            shell.orch.block.battery,
            shell.orch.ctl.dispatch.battery_floor(),
            shell.battery_warn,
            dt_ms,
        );
        let levels = tones::Levels {
            horn: shell.orch.inbox.remote_horn(),
            battery_critical,
            battery_low,
        };
        // SAFETY: main-thread context (a dispatch callback), the `STORE` discipline: the borrow
        // ends with this statement and the loop holds none across `dispatch()`.
        let p = match unsafe { (*addr_of_mut!(STORE)).as_ref() } {
            Some(store) => shell.player.tick(levels, dt_ms, &StoreTunes(store)),
            None => 0,
        };
        motor::TONE_PERIOD.store(p, Ordering::Relaxed);
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
    /// pattern this file already uses for `RAM_VECTORS` and `DMA_RING`: one
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
        /// The live gating-producer level mask in the counter domain (`orchestrator::events::counted`:
        /// b0 comms_loss, b1 stop_all (the tilt cutout), b2 imu_loss, b3 motor_fault (the union of
        /// the four `EV_MOTOR_*` causes, which only the cyclic `fault` word carries apart), b4 latch A, b5 latch B, b6 mode_fault,
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
        /// Inter-board links' recovered LINE-ERROR count, summed over the links
        /// ([`LINK_LINE_ERRORS`]): `SplitSerial`-absorbed self-healed DMA-RX wire disturbances (`LineError`: ERRIE overrun / framing / noise) since
        /// boot. Appended (offset-preserving) so the SWD reader's existing field offsets are unchanged.
        /// The self-heal observable: non-zero with `cyclic_age` fresh proves the DMA-RX line-error
        /// recovery ran on an induced peer-reboot glitch. Counted APART from laps (the OQ1 split).
        link_line_errors: u32,
        /// Inter-board links' LAP-OVERRUN count, summed over the links ([`LINK_LAP_OVERRUNS`]):
        /// `SplitSerial`-absorbed buffer-overrun losses (the DMA lapped the read cursor: the consumer fell behind) since boot.
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
        /// Word 29, ZERO. It was the CC2541 port's recovered RX-loss counts; the port went
        /// (`specs/decision-cc2541-removal.md`) and the word keeps its offset, publishing zero,
        /// because the bench reads this block by offset and `tools/stack-margin.sh` finds word 30
        /// by symbol plus offset. A later change may re-purpose it.
        retired_29: u32,
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
        /// maintained in this packing by `arm::hw` through [`arm::note_confirm_refusal`]. Appended
        /// LAST; word 33 in the SWD map.
        ///
        /// Bits 8..15: a saturating count of `arm::ArmStep::ConfirmPeriodsLive` refusals this
        /// boot. Bits 0..7 and 16..31: zero (the re-read's refusal count and the refused IMU frame's
        /// cause byte, until the arm row stopped refusing: `specs/config-read-once.md` section 5).
        /// The refusal is boot-sticky, so the count is the only record of how many attempts it ate.
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
        /// The inter-board links' abandoned writes ([`TX_TIMEOUTS`]): link 0 in the low half, link 1
        /// in the high half. Appended LAST, so every prior field keeps its offset (the
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
        /// The period-based wheel rate (`specs/control.md` (o)): `motor::WHEEL_RATE_Q12`, the reciprocal of
        /// the periods between two same-direction hall edges in 1/4096 of a count, the
        /// state-feedback law's speed measurement. Appended LAST, offset-preserving, at word 39.
        /// The hand-spin gate reads it: smooth and sign-correct while the wheel turns, decaying to
        /// zero within a second of it stopping, exactly 0 while it rocks between two hall codes.
        motor_rate_q: i32,
        /// Word 40, ZERO. It was the CC2541 port's staged samples (own and phone forwards); the
        /// port went (`specs/decision-cc2541-removal.md`) and the word keeps its offset, publishing
        /// zero. A later change may re-purpose it.
        retired_40: u32,
        /// Samples the SWD mailbox port's ring took whole, own in the low half and forwards in the
        /// high half, wrapping; word 41 (`specs/link-control.md`, "Counting what is staged
        /// (2026-10-25)"). Advances only while a host is attached; a
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
        /// Word 50 ([`motor::OBS_FOC_HOLD`], round 3): the hold word. Bits 0..7 the periods of
        /// the window on which `phase_a`'s sample was held, 8..15 `phase_b`'s, 16..23 the
        /// periods the bridge was floated, 24..31 the cause byte (bit 24 disarmed, 25 the
        /// freshness guard, 26 the chop, 27 both phases held in one period, 28..29 the offset
        /// calibration's block-pair count this boot, 30 that calibration refused for
        /// disagreement). Live in every method.
        foc_hold: u32,
        /// Word 51, low byte ([`IMU_RETRIES`]): the boot's IMU attempts, 0 first attempt, 1
        /// second attempt after a bus clear, 2 not up (`specs/imu-probe-retry.md`). Appended LAST,
        /// offset-preserving. The bench reads it beside each boot report; nothing else consumes it.
        imu_retries: u8,
        /// The rest of word 51, published zero so the word reads clean out of `.uninit`.
        pad_51: [u8; 3],
        /// The flux observer in shadow (`specs/flux-observer.md`, "What it publishes"), motor 0's,
        /// three words closed at the 64-period window beside words 47 to 50, each published by one
        /// relaxed store in the period ISR. Appended LAST, offset-preserving, at words 52 to 54.
        ///
        /// Word 52 ([`motor::OBS_FOC_ANGLES`]): `theta` at the close, the front end's angle plus
        /// the alignment, the angle the inverse Park used, low half; the observer's angle high
        /// half; both `u16`, 65536 a revolution. 0 unless FOC.
        foc_angles: u32,
        /// Word 53 ([`motor::OBS_FOC_TRACK`]): the window's angle-error mean (observer minus hall,
        /// less the offset and the bin's leak lead) low half, its spread high half, `i16` angle
        /// units.
        foc_track: u32,
        /// Word 54 ([`motor::OBS_FOC_FLUX`]): the window's RMS flux magnitude, microvolt-seconds,
        /// low half; the tracking word high half: bits 16..23 the lock bin (255 none), 24 this
        /// window tracked, 25 the observer is the angle source (0 in S1), 26 the front end in its
        /// interpolation window at the close.
        foc_flux: u32,
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
        // Word 29: retired (the CC2541 port's RX losses), publishes zero.
        assert!(offset_of!(CtrlObs, retired_29) == 0x74);
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
        // Word 40: retired (the CC2541 port's staged samples), publishes zero. Word 41: the
        // mailbox port's.
        assert!(offset_of!(CtrlObs, retired_40) == 0xA0);
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
        // Word 50: the hold word (round 3).
        assert!(offset_of!(CtrlObs, foc_hold) == 0xC8);
        // Word 51: the IMU bring-up's attempt count, low byte.
        assert!(offset_of!(CtrlObs, imu_retries) == 0xCC);
        assert!(offset_of!(CtrlObs, pad_51) == 0xCD);
        // Words 52-54: the flux observer in shadow, angles, then error, then flux and tracking.
        assert!(offset_of!(CtrlObs, foc_angles) == 0xD0);
        assert!(offset_of!(CtrlObs, foc_track) == 0xD4);
        assert!(offset_of!(CtrlObs, foc_flux) == 0xD8);
        // And no tail padding hiding a mis-sized field: 55 words (220 B) exactly.
        assert!(core::mem::size_of::<CtrlObs>() == 55 * 4);
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
                .write_volatile(hal::irq::SYSTICK_ISR_METRIC.entries());
            addr_of_mut!((*p).usart1_isr_entries)
                .write_volatile(hal::irq::USART1_RX_ISR_METRIC.entries());
            addr_of_mut!((*p).dma_isr_entries)
                .write_volatile(hal::irq::DMA_RX_ISR_METRIC.entries());
            addr_of_mut!((*p).roll_milli).write_volatile(o.roll_milli_deg);
            addr_of_mut!((*p).motor_periods)
                .write_volatile(motor::PERIODS[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).motor_state).write_volatile(
                motor::OBS_STATE[0].load(Ordering::Relaxed)
                    | if period_live {
                        motor::OBS_PERIOD_LIVE << 24
                    } else {
                        0
                    },
            );
            addr_of_mut!((*p).motor_duty01)
                .write_volatile(motor::OBS_DUTY01[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).motor_duty2_angle)
                .write_volatile(motor::OBS_DUTY2_ANGLE[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).motor_fault).write_volatile(
                motor::FAULT[0].load(Ordering::Relaxed)
                    | (motor::INVALID_DWELL[0].load(Ordering::Relaxed).min(0xFFFF) << 16),
            );
            addr_of_mut!((*p).motor_speed).write_volatile(motor::SPEED[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).motor_cal).write_volatile(motor::OBS_CAL[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).gating_field).write_volatile(o.gating_field);
            addr_of_mut!((*p).pre_env_torque).write_volatile(o.pre_env_torque);
            addr_of_mut!((*p).event_counts).write_volatile(o.event_counts);
            addr_of_mut!((*p).retired_29).write_volatile(0);
            addr_of_mut!((*p).stack_margin).write_volatile(sample_stack_margin());
            addr_of_mut!((*p).motor_current)
                .write_volatile(motor::OBS_CURRENT[0].load(Ordering::Relaxed));
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
            addr_of_mut!((*p).motor_rate_q)
                .write_volatile(motor::WHEEL_RATE_Q12[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).retired_40).write_volatile(0);
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
                .write_volatile(motor::OBS_FOC_PHASE[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).foc_dq_mean)
                .write_volatile(motor::OBS_FOC_DQ[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).foc_ref_vq)
                .write_volatile(motor::OBS_FOC_REF[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).foc_hold)
                .write_volatile(motor::OBS_FOC_HOLD[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).imu_retries).write_volatile(IMU_RETRIES.load(Ordering::Relaxed));
            addr_of_mut!((*p).pad_51).write_volatile([0; 3]);
            addr_of_mut!((*p).foc_angles)
                .write_volatile(motor::OBS_FOC_ANGLES[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).foc_track)
                .write_volatile(motor::OBS_FOC_TRACK[0].load(Ordering::Relaxed));
            addr_of_mut!((*p).foc_flux)
                .write_volatile(motor::OBS_FOC_FLUX[0].load(Ordering::Relaxed));
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
        // `dispatch()` drains them one per main-loop pass (`motorcontroller/scheduler`), so after the main
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
        let motor_state = motor::OBS_STATE[0].load(Ordering::Relaxed);
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
            let periods = motor::PERIODS[0].load(Ordering::Relaxed);
            let per_tick = periods.wrapping_sub(shell.last_periods) / dt_ticks;
            shell.last_periods = periods;
            shell.period_live = motor::periods_live(per_tick);
            shell.period_health.update(motor_running, shell.period_live);
        }
        // The level folds every brought-up motor's fault word: a hall or calibration fault on
        // either motor of a two-motor board shuts the board down, as its one mode machine does.
        shell.orch.motor_fault =
            motor::OBS_STATE
                .iter()
                .zip(motor::FAULT.iter())
                .fold(0, |level, (state, fault)| {
                    level
                        | motor::motor_fault_level(
                            motor::obs_configured(state.load(Ordering::Relaxed)),
                            fault.load(Ordering::Relaxed),
                            shell.period_health.loss(),
                            arm::hw::refused(),
                        )
                });
        // The hard over-current trip: the ISR counts trips per motor, and a change since the last
        // run raises that motor's fault latch through its seam. The latch consumes the code this
        // same pass and reaches `fault_a` (motor 0) or `fault_b` (motor 1), so it is NOT folded
        // into the motor-side level above.
        for (m, (word, last)) in motor::OVER_CURRENT_TRIPS
            .iter()
            .zip(shell.last_trips.iter_mut())
            .enumerate()
        {
            let trips = word.load(Ordering::Relaxed);
            if trips != *last {
                *last = trips;
                shell.orch.raise_over_current(m);
            }
        }
        // The protections beside the hard trip (`specs/optional-protections.md` section 6.2). The
        // break: each break-enabled motor's `ext_trip` is its timer's break flag, so a break
        // latches that motor's fault on this RUN pass. The DC-link limit: each chain's derate
        // stepped on the window the ISR last closed, and its reading and factor published.
        #[cfg(feature = "gd32f103rc")]
        let breaks = motor::hw::break_flags();
        #[cfg(feature = "gd32f103rc")]
        {
            for (m, level) in breaks.iter().enumerate() {
                if let Some(level) = *level {
                    shell.orch.set_ext_trip(m, level);
                }
            }
            for (m, dc) in shell.dc_link.iter_mut().enumerate() {
                if let Some(dc) = dc {
                    let obs = dc.step(motor::DC_LINK_SUM[m].load(Ordering::Relaxed));
                    motor::OBS_DC_LINK[m].store(obs, Ordering::Relaxed);
                }
            }
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
        // One row per motor, and the OFF inhibit holds while EITHER wheel turns.
        let mut moving = false;
        for (row, word) in shell
            .orch
            .block
            .wheel_speed
            .iter_mut()
            .zip(motor::SPEED.iter())
        {
            let speed = word.load(Ordering::Relaxed);
            moving |= arm::off_inhibit(motor_running, shell.period_live, speed);
            *row = motor::wheel_speed_word(speed);
        }
        shell.orch.motor_moving = moving;
        // The period-based rate beside it (`specs/control.md` (o)): the state-feedback law's speed
        // measurement, already in the law's own 1/4096-count unit, so there is nothing to narrow
        // and nothing to convert. One load, like the count's.
        //
        // One row per motor: motor 1's rides the cyclic's motor-1 block (`specs/link-control.md`).
        for (row, word) in shell
            .orch
            .block
            .wheel_rate_q12
            .iter_mut()
            .zip(motor::WHEEL_RATE_Q12.iter())
        {
            *row = word.load(Ordering::Relaxed);
        }
        // The signed position beside it (`specs/state-feedback.md` section 3): the law folds it,
        // the cyclic carries its low half to the peer and the observation publishes it mirrored.
        // One load.
        shell.orch.block.wheel_pos = motor::POS_EDGES[0].load(Ordering::Relaxed);
        // The battery-sense count the period ISR stored (rank 2 of the injected group, already
        // 12-bit), handed over BEFORE the pass whose source rule converts and filters it
        // (`specs/sensing-and-safety.md`, "The battery word"). Loaded every tick on every board;
        // only a sensing board consumes it, and it stays 0 until the group converts. On the
        // two-motor image the count is ADC2's regular conversion instead, started at the end of
        // the previous pass and read here (`specs/twelve-fet.md` section 5.4).
        motor::hw::battery_read();
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
        //
        // One window per motor, each from its own motor's two cells: motor 0's rides the base's
        // block, motor 1's the motor-1 block (`specs/link-control.md`, "The motor-1 block").
        for (i, window) in shell.orch.phase.iter_mut().enumerate() {
            let mean_duty = motor::OBS_MEAN_DUTY[i].load(Ordering::Relaxed);
            *window = orchestrator::PhaseWindow {
                peak: motor::unpack_peak(motor::OBS_CURRENT[i].load(Ordering::Relaxed)),
                mean: motor::unpack_mean(mean_duty),
                duty_on: motor::unpack_duty_on(mean_duty),
            };
        }
        // The odometer the cyclic payload carries (`specs/odometer.md`): the LIVE total, folded
        // by the service loop, which is the context that owns it and the only one that may write
        // it to flash. Copied here beside the current window for the same reason, that the
        // payload is built inside the pass below.
        shell.orch.odometer = ODOMETER_TOTAL.load(Ordering::Relaxed);
        let out = control_task(&mut shell.orch, sample.as_ref(), dt_ticks);
        // The tones, after the pass that wrote this tick's battery word and mirror. A catch-up run
        // has no elapsed time to play.
        if served {
            tone_tick(shell, dt_ticks);
        }
        // The base, and on a two-motor image motor 1's block from this same pass's rows.
        let more = more_motors(&shell.orch);
        // The flux observer's bus scale for the coming periods (`specs/flux-observer.md`,
        // "Voltage sensing"): a sensing board's own count through its calibration, else the
        // effective word the pass just chose, 0 while that is unknown.
        let local_word = shell.orch.vbatt.map_or(0, |v| v.word());
        let bus = shell
            .observer
            .bus_scale(local_word, shell.orch.block.battery);
        motor::OBS_BUS.store(bus.pack(), Ordering::Relaxed);
        shell.cyclic_out = cyclic_tx(&shell.orch, shell.addressed);
        if shell.cyclic_out.is_some() {
            shell.cyclic_more = more;
        }
        // The guest ports' own decimation (5 Hz) and the peer's state at its own 2.5 Hz, only
        // while fresh. Latest-wins: a payload the loop has not picked up yet is simply
        // overwritten, never queued, so a slow module cannot build a backlog.
        shell.guest.fill(
            ble_cyclic_tx(&shell.orch, shell.addressed),
            more,
            ble_peer_tx(&shell.orch, shell.addressed),
        );
        let obs = shell.orch.obs();
        // The 250 Hz -> 16 kHz handoff (`specs/motor-integration.md`): this task is the SOLE writer
        // of the demand word, and it writes the +-28500 stock-native torque word verbatim (no
        // rescaling anywhere, `specs/control.md`), with one exception: on a motor with a DC-link
        // limit the word is multiplied by its derate factor, exact at full
        // (`specs/optional-protections.md` section 3.2). The sequence bump is what lets the ISR's
        // freshness guard tell a fresh write from a repeated one, so a demand that happens to
        // repeat its value still counts as fresh.
        //
        // One word per motor, each the orchestrator's setpoint for that motor, all written before
        // the one sequence bump, so the ISR sees both demands of a pass as fresh together.
        for (m, (word, demand)) in motor::DEMAND.iter().zip(out.torque_setpoint).enumerate() {
            #[cfg(feature = "gd32f103rc")]
            let demand = shell.dc_link[m].map_or(demand, |dc| dc.scale(demand));
            #[cfg(not(feature = "gd32f103rc"))]
            let _ = m;
            word.store(demand as i32, Ordering::Relaxed);
        }
        // The Cortex-M0 has no read-modify-write atomics; the control task and the arm sequence
        // (`arm::hw::zero_demand`) run in the one main-thread context, so a load and a store are
        // the whole update there.
        #[cfg(target_has_atomic = "32")]
        motor::DEMAND_SEQ.fetch_add(1, Ordering::Relaxed);
        #[cfg(not(target_has_atomic = "32"))]
        motor::DEMAND_SEQ.store(
            motor::DEMAND_SEQ.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
        // Step 6's enactment, at last acting (`specs/motor-integration.md`, "MOE enactment"): the
        // mode machine's per-motor allowance, enacted. AFTER the demand publish above, so a
        // shutdown's zeroed demand is the last word written this tick rather than one this same
        // tick overwrites; the arm path re-zeroes it for the same reason.
        // The arm path's first step is the value-row re-read, as a closure over this shell (see
        // `re_read_arm_values`). The other arguments are copied into locals FIRST, so the closure's
        // mutable borrow of `shell` is the only one live across the call.
        let motor_fault = shell.orch.motor_fault;
        arm::hw::enact(out.moe, motor_configured, motor_fault != 0, || {
            re_read_arm_values(shell)
        });
        // The break's recovery (`specs/optional-protections.md` section 3.3): on a pass that left
        // the board in OFF, where every latch has just cleared, each break-enabled timer's flag
        // clears too, through the arming layer. A break input still active sets it again at once,
        // so the next RUN pass re-latches.
        #[cfg(feature = "gd32f103rc")]
        if out.mode_byte == state::Mode::Off.as_byte() {
            arm::hw::clear_break_flags(breaks.map(|b| b.is_some()));
        }
        publish_obs(
            &obs,
            shell.boot_count,
            shell.period_live,
            shell.inputs.pads_configured(),
            shell.guest_staged,
        );
        // The next battery conversion, which the next pass reads (two motors; nothing otherwise).
        motor::hw::battery_start();
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

    /// The arm-row function (`specs/config-read-once.md` section 5): read the arm row in one store
    /// pass, convert it, and install it into its five owners: the period ISR's record
    /// (`motor::hw::install_rederived`), the IMU's axis frame (`imu::Imu::set_config`), the control
    /// section's mode, rider requirement, battery floor and position clamp
    /// (`orchestrator::re_apply_control_values`, which also replaces the balance producer record
    /// when the active mode changed), the turn bound and rate, and the gain shadow's maxima. The
    /// boot calls it once, after the bring-ups; the arm sequence calls it as its first step. It
    /// refuses nothing: the row is checked before it is written.
    ///
    /// `#[inline(never)]` with NO `#[link_section]`, deliberately: it must stay OUT of `.hotcode`,
    /// the F1x0 zero-wait first 32 KiB (`motorcontroller/firmware/memory.x`). It runs once per arm
    /// rather than once per pass, so window space spent on it would be taken from the 16 kHz and
    /// 250 Hz paths that need it; and `inline(never)` is what stops the body being inlined into
    /// `control_task_cb`, which IS placed in the window.
    #[inline(never)]
    fn re_read_arm_values(shell: &mut Shell) {
        // SAFETY: main-thread context (the boot, or a dispatch callback), the `STORE` discipline:
        // the borrow ends with this statement and the loop holds none across `dispatch()`.
        let Some(values) = (unsafe { (*addr_of_mut!(STORE)).as_ref() }).map(arm::read_arm_values)
        else {
            return;
        };
        // The bring-up's floor and accepted offsets, from the runtime; a board with no motor
        // runtime installs the rest.
        let boot = motor::hw::boot_fixed();
        // The IMU's INSTALLED gyro bias, carried through: `IMU_GYRO_BIAS` is a boot calibration.
        let bias = shell.imu.as_ref().map(|d| d.config().gyro_bias);
        let r = arm::rederive(
            &values,
            boot.map(|b| {
                b.unwrap_or(motor::BootFixed {
                    noise_floor: 0,
                    accepted_offsets: None,
                    #[cfg(feature = "gd32f103rc")]
                    dc_link: None,
                    dead_time: 0,
                })
            }),
            bias,
        );
        // The arm resets each DC-link derate to full (`specs/optional-protections.md` section
        // 3.2).
        #[cfg(feature = "gd32f103rc")]
        for dc in shell.dc_link.iter_mut().flatten() {
            dc.reset_derate();
        }
        // Installs each motor that has a runtime, all in one critical section.
        motor::hw::install_rederived(&r.motor);
        // The flux observer's parameters at the calibration just read and the bring-up's dead
        // time (`specs/flux-observer.md`, "Units": `R` and `L` are pre-scaled at arm time).
        shell
            .observer
            .set_motor_facts(values.current_cal, boot.map(|b| b.map(|b| b.dead_time)));
        motor::hw::install_observer(&shell.observer.params());
        if let (Some(dev), Some(cfg)) = (shell.imu.as_mut(), r.imu) {
            // Reinstalls the axis permutation; touches no bus.
            dev.set_config(cfg);
        }
        orchestrator::re_apply_control_values(
            &mut shell.orch,
            r.control_mode_byte,
            r.frame_byte,
            r.leader_byte,
            r.side_byte,
            r.rider_required_byte,
            r.battery_floor,
            r.pos_clamp,
        );
        let ctl = &mut shell.orch.ctl;
        ctl.drive_turn = r.drive_turn;
        ctl.gains.re_apply_max(r.gain_max);
        // The board now runs the stored arm-effect values: `FLAG_REARM` clears
        // (`specs/machine-settings.md` section 5).
        shell.orch.machine.armed();
    }

    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget fix): the Shell value
    /// (the orchestrator state is the image's biggest single object) is constructed here and
    /// written into the static, so the boot frame never carries the temporary (since shrink
    /// round 7 that frame is `boot`'s and pops; it was `main`'s and permanent).
    ///
    /// The argument list is the Shell's boot inputs one for one (the orchestrator constructor's
    /// inputs, then the bus, the device, the input pins, the boot ordinal and the part); the arm
    /// row's values are not among them, because the arm-row function installs them right after
    /// (`specs/config-read-once.md` section 5).
    #[inline(never)]
    #[allow(clippy::too_many_arguments)]
    fn init_shell(
        level_trim_centideg: [i16; 2],
        gains: control::GainShadow,
        vbatt_cal: Option<orchestrator::battery::VbattCal>,
        imu_bus: Option<I2c>,
        imu_dev: Option<imu::Imu>,
        imu_staged: bool,
        inputs: InputPins,
        boot_count: u32,
        chip_tag: ChipTag,
        config_fault: bool,
        machine: orchestrator::Machine,
        battery_warn: i16,
        observer: crate::observer::ObserverLoop,
    ) {
        let imu_configured = imu_dev.is_some();
        // SAFETY: single-threaded boot (only the DMA RX ISR is live, and it reaches only the
        // HAL ring); the one initializing write, before any task dispatch exists.
        unsafe {
            *addr_of_mut!(SHELL) = Some(Shell {
                orch: OrchestratorState::new(
                    imu_configured,
                    imu_staged,
                    attitude::Config::staged(level_trim_centideg),
                    gains,
                    vbatt_cal,
                    // What this board says about itself for the life of the boot
                    // (`specs/link-control.md`, the `CYCLIC_STATE` layout, offsets 17 and 18).
                    // The boot ordinal is narrowed to its low byte HERE, once, rather than on
                    // every emission: the wire carries one byte and the full counter stays on
                    // the shell for `CTRL_OBS`.
                    BoardIdentity {
                        boot_tag: boot_count as u8,
                        chip: chip_tag,
                        config_fault,
                    },
                    machine,
                ),
                i2c: imu_bus,
                imu: imu_dev,
                inputs,
                cyclic_out: None,
                cyclic_more: [CyclicMotor::ZERO; motor::COUNT - 1],
                guest: GuestSlots::new(),
                guest_staged: GuestStaged::default(),
                addressed: false,
                boot_count,
                last_control_tick: 0,
                last_periods: 0,
                period_live: false,
                period_health: motor::PeriodHealth::new(),
                last_trips: [0; motor::COUNT],
                #[cfg(feature = "gd32f103rc")]
                dc_link: [None; motor::COUNT],
                player: tones::Player::new(PERIOD_HZ),
                alarm: tones::BatteryAlarm::new(),
                battery_warn,
                observer,
            });
        }
    }

    /// Bring the IMU up on its stored bus (`specs/config-read-once.md` section 5): the I2C
    /// instance and the two pins with their alternate functions, as stored, and on the GD32F10x the
    /// I2C's stored remap, when `imu.model` is non-zero (`layout::ImuBus`); the probe checks the
    /// model's `WHO_AM_I`, and a model index the imu crate does not know is recorded `NotUp`.
    ///
    /// The frame installed here is the reference one with the stored gyro bias; the arm-row
    /// function installs the stored signs and roles right after, as it does at every arm.
    ///
    /// Fails soft (`(None, None)`), recorded in the status record as `NotUp`: the board boots
    /// link-only-plus-throttle, and `CTRL_OBS`'s `imu_configured` reads 0 beside `imu_staged` (the
    /// flags byte's b0 and b6).
    ///
    /// `#[inline(never)]`: a POPPED boot frame (the slice-7 stack-budget idiom, as `init_shell`).
    /// Only the two live handles are returned; the staged gyro bias, the probe/init working set and
    /// `I2c::new`'s own frame are gone before the loop exists.
    #[inline(never)]
    fn bring_up_imu<F: store::Flash>(
        chip: &hal::Chip,
        store: &Store<F>,
        bus: Option<layout::ImuBus>,
        status: &mut StatusRecord,
    ) -> (Option<I2c>, Option<imu::Imu>) {
        let Some(b) = bus else {
            return (None, None);
        };
        let scl_id = store::IMU_SCL_PIN.id();
        let Some(model) = imu::model_from_index(b.model) else {
            status.record(store::IMU_MODEL.id(), 0, Reason::NotUp, 0);
            return (None, None);
        };
        let instance = i2c_label(b.i2c);
        // The GD32F10x routes the I2C by its AFIO remap, the stored one.
        #[cfg(any(feature = "gd32f103c8", feature = "gd32f103rc"))]
        hal::afio::remap(instance, b.remap);
        // The per-board zero-rate gyro bias from the store (IMU_GYRO_BIAS x/y/z at indices
        // 0/1/2; default 0 = uncalibrated), a boot calibration.
        let bias = [
            store.get(IMU_GYRO_BIAS.at(0)),
            store.get(IMU_GYRO_BIAS.at(1)),
            store.get(IMU_GYRO_BIAS.at(2)),
        ];
        let cfg = imu::Config::staged([0; 3], bias, [0; 2]);
        let mut bus = match I2c::new(
            chip,
            &CLOCK,
            instance,
            pin_af(b.scl),
            pin_af(b.sda),
            I2cMode::fast(IMU_I2C_HZ, hal::i2c::FastDuty::Two),
        ) {
            Ok(bus) => bus,
            Err(e) => {
                status.record(scl_id, 0, Reason::NotUp, e.code());
                return (None, None);
            }
        };
        let mut dev = imu::Imu::new(model, cfg);
        // Probe and init at most twice, with a bus clear between (`specs/imu-probe-retry.md`): a
        // warm reset taken while the IMU drove SDA can fail the first probe and no later one.
        let attempts = imu_attempts::run(
            &mut bus,
            |bus| dev.probe(bus).is_ok() && dev.init(bus).is_ok(),
            I2c::bus_clear,
        );
        IMU_RETRIES.store(attempts.retries(), Ordering::Relaxed);
        match attempts.not_up_detail(|e| e.code()) {
            None => {
                // The caller-owned post-init settle (specs/imu.md; the imu-bench pause) before the
                // first cyclic read.
                cortex_m::asm::delay((CLOCK.sysclk_hz / 1000) * IMU_SETTLE_MS);
                (Some(bus), Some(dev))
            }
            Some(detail) => {
                status.record(scl_id, 0, Reason::NotUp, detail);
                (None, None)
            }
        }
    }

    /// Route one delivered-but-unhandled PDU (the `net` hand-back): the reserved control block
    /// `0x10..0x2F` decodes through `linkctl` into the orchestrator inbox, the two TUNE opcodes go
    /// to the live gain shadow, and everything else stays dropped (integration.md, "The
    /// delivered-PDU hand-back").
    ///
    /// `out` takes the tune reply, so the caller must route emissions AFTER this rather than
    /// before it. `ingress` is the port the PDU arrived on: a `CYCLIC_STATE` on a link's port is
    /// that link's peer, whose machine digest the arm gate reads (`specs/machine-settings.md`
    /// section 4.2).
    fn route_handback(
        handed: Option<net::DeliveredPdu>,
        ingress: u8,
        responder: &Responder,
        out: &mut Emits,
    ) {
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
            if let (linkctl::Payload::CyclicState(c), Some(link)) =
                (&payload, ingress.checked_sub(PORT_IDX_LINK0))
            {
                shell.orch.machine.hear(link as usize, c.machine);
            }
            // The sender's L3 address goes in with the payload: the inbox scopes mirror liveness
            // to the node that armed the board, so a second controller's traffic cannot hold a
            // rider's arm alive (`LinkInbox::refreshes_mirror`).
            shell.orch.inbox.accept(d.src, payload);
        }
    }

    /// Serve one `TUNE_READ` / `TUNE_WRITE` against the live gain shadow and reply `CONFIG_RESP`
    /// (`specs/remote-ui.md` section 4, the live tune lane).
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
        // accepted for tuning (`specs/remote-ui.md`, "Cross-thread and mid-loop discipline"). No
        // torque step can come of it: in RUN the law ramps its live gains toward the shadow under
        // fixed per-pass caps (`control::config::sf::RAMP_CAPS`), which the control tests pin.
        let Some(shell) = (unsafe { (*addr_of_mut!(SHELL)).as_mut() }) else {
            return;
        };
        if crate::observer::ObserverLoop::owns(key.field_id) {
            // The flux observer's words (`specs/flux-observer.md`, "Fields"): live in RAM, the
            // parameters re-derived and installed on a write, no flash touched.
            let o = &mut shell.observer;
            let (status, value) = match op {
                net::TuneOp::Refuse(status) => (status, None),
                net::TuneOp::Read => match o.get(key.field_id, key.index) {
                    Some(v) => (net::walk::CFG_OK, Some(v)),
                    None => (net::walk::CFG_UNKNOWN_KEY, None),
                },
                net::TuneOp::Write(v) => match o.set(key.field_id, key.index, v) {
                    Ok(()) => {
                        motor::hw::install_observer(&o.params());
                        (net::walk::CFG_OK, Some(v))
                    }
                    Err(control::TuneError::UnknownKey) => (net::walk::CFG_UNKNOWN_KEY, None),
                    Err(control::TuneError::OutOfRange) => (net::walk::CFG_BAD, None),
                },
            };
            responder.reply_tune(d.src, key, status, value, out);
            return;
        }
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

    /// The ONE consumer of the responder's persist signal (`Responder::take_config_persisted`,
    /// taken once per loop pass): a `CONFIG_WRITE` persisted, so hand the event to everything that
    /// holds a fact derived from the store (`specs/machine-settings.md` section 4.1).
    ///
    /// - The live gain shadow re-reads the stored gains (`specs/remote-ui.md` section 4: a
    ///   disarmed `CONFIG_WRITE` to a live-lane field also refreshes the shadow, so flash and RAM
    ///   converge on save without a reboot; the fields are 0x27 and 0x29,
    ///   `specs/state-feedback.md` section 6). It takes only the gains whose STORED value moved,
    ///   so saving one field cannot revert a gain the tune lane is holding live.
    /// - The machine digest and the two stored digests behind `FLAG_REARM` and `FLAG_REBOOT` are
    ///   recomputed (`orchestrator::Digests`).
    ///
    /// Called only on the signal, so the store scans are paid on a config write and never in the
    /// steady loop.
    fn on_config_persisted<F: store::Flash>(responder: &mut Responder, store: &Store<F>) {
        if !responder.take_config_persisted() {
            return;
        }
        let fresh = read_gains(store);
        let digests = orchestrator::Digests::of(store);
        // SAFETY: main-thread context, as `route_tune` above.
        if let Some(shell) = unsafe { (*addr_of_mut!(SHELL)).as_mut() } {
            shell.orch.ctl.gains.reconcile(fresh);
            shell.orch.machine.persisted(digests);
            observer_persisted(&mut shell.observer, store);
        }
    }

    /// The flux observer's share of a persisted write: re-read its two fields, move the words
    /// flash moved, and install the parameters when any did. Out of line, off the service loop's
    /// zero-wait placement: it runs on a config write only.
    #[inline(never)]
    fn observer_persisted<F: store::Flash>(
        o: &mut crate::observer::ObserverLoop,
        store: &Store<F>,
    ) {
        if o.reconcile(crate::observer::read(store)) {
            motor::hw::install_observer(&o.params());
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Direct bring-up (`specs/configured-links.md` section 3): each subsystem reads its own fields
    // and asks the HAL; a failure costs only that function and adds an entry to BOARD_OBS.
    // ---------------------------------------------------------------------------------------------

    /// A stored pin with its alternate function, as the HAL takes it.
    #[inline]
    fn pin_af([port, pin, af]: layout::Pin3) -> hal::PinAf {
        hal::PinAf { port, pin, af }
    }

    /// The HAL's label for a stored I2C instance number.
    #[inline]
    fn i2c_label(n: u8) -> PeriphLabel {
        if n == 1 {
            PeriphLabel::I2c1
        } else {
            PeriphLabel::I2c0
        }
    }

    /// The HAL's label for a stored USART instance number.
    #[inline]
    fn usart_label(n: u8) -> PeriphLabel {
        match n {
            0 => PeriphLabel::Usart0,
            1 => PeriphLabel::Usart1,
            _ => PeriphLabel::Usart2,
        }
    }

    /// Assert this board's power-latch (SELF_HOLD) high, on the stored `power.self_hold` pin, FIRST
    /// of every bring-up, and record the pin driven in the status record. With none stored no pin
    /// is touched (`specs/no-fleet-pins.md`).
    ///
    /// Role-agnostic on EVERY board: latch this board's own rail on so it stays up after the
    /// inter-board wake drops. A slave is woken over the cable and would otherwise fall back asleep
    /// once the master stops driving it; a master bridges its own power button. Unconditional,
    /// never gated on chip family, because role is not known here. RoboDurden does the same
    /// (`main.c:148`). It is unavoidably post-mount (its pin is persisted data), so the window
    /// between power-on and the assert is the mount itself.
    fn assert_self_hold(chip: &hal::Chip, pin: Option<layout::Pin>, status: &mut StatusRecord) {
        let Some([port, pin]) = pin else {
            return;
        };
        // Dropping the handle leaves the pin configured and driven: the latch has to outlive every
        // frame in this boot.
        let _ = chip
            .pin(hal::PinId { port, pin })
            .into_push_pull_output()
            .set_high();
        status.self_hold_port = port;
        status.self_hold_pin = pin;
    }

    /// The boot status record (`specs/configured-links.md` section 5): what did not come up, and
    /// why. Written during boot through the one `&mut` [`boot`] forms, fixed after it, read over SWD
    /// at the address of this un-mangled symbol (`tools/swdobs.py`), and served by `STATUS_READ`
    /// from its entries. Never read through a reference while it is written.
    #[no_mangle]
    static mut BOARD_OBS: StatusRecord = StatusRecord::new();

    // ---------------------------------------------------------------------------------------------
    // Serials: the L2 links ride hal's embedded-io adapters (specs/firmware.md, "The link
    // serials"): SplitSerial<RingBufferedRx> for the inter-board UART, the mailbox's own serial for
    // port 0. There is no firmware-local serial wrapper.
    // ---------------------------------------------------------------------------------------------

    // The two concrete L2 links (heterogeneous serials, one L2 code path each). Each link's
    // frame-scratch const (`F`) is its transport's framer capacity, NOT the 255 B protocol
    // ceiling: the scratch lives on the deep drain/send stack chains (round-7a reclaim, ~360 B
    // across poll_recv + send on the 8 KiB parts) and the carrier can never emit a larger frame.
    type MailboxLink =
        Link<SerialTransport<MailboxSerial, MAILBOX_FRAMER_N>, PACKET, MAILBOX_FRAMER_N>;
    // The inter-board port is on DMA BOTH ways (`motorcontroller/hal/src/usart_tx.rs`): a ring
    // receiving and a one-shot transmitting, so a frame costs the main loop a copy instead of its
    // wire time (0.43 ms at 460800).
    type UartLink =
        Link<SerialTransport<SplitSerial<RingBufferedRx>, UART_FRAMER_N>, PACKET, UART_FRAMER_N>;

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
        hal::debug_hold_timers();

        // Paint the free stack, SECOND: as early as anything can run while still leaving the FET
        // backstop above it, and before the first thing in this function with a frame worth
        // measuring (`Mailbox::from_raw`, `Chip::take`, `clock::configure_tree`).
        //
        // WHERE IT CANNOT GO. Not before `debug_hold_timers`, which is the halt backstop and stays
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
        // run yet. The painted region is about 900 words (see `paint_stack`), so <= 5,400 cycles =
        // **<= 675 us**, once, on the boot path, spent on EVERY boot.
        //
        // Painting after `configure_tree` would cost 9x less (72 MHz rather than 8 MHz) and is
        // not done: it would leave the clock-tree frames unmeasured, to save 0.6 ms of boot.
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

        // The state the loop borrows for the life of the image lives HERE, in `main`'s frame, and
        // since shrink round 7 (2026-10-27) it is the ONLY thing that does: every other value the
        // bring-up makes is scaffolding, and it lives in `boot`'s frame, which pops before the loop
        // starts. `main`'s frame used to hold both (2,248 B, held forever, under every chain in the
        // image); now it is the state alone, about 1.3 KB (`specs/firmware.md`, "RAM/stack budget").
        // `MaybeUninit`, so no 1.3 KB of zeroing and no temporary: `boot` writes each field in
        // place, exactly once.
        let mut state = MaybeUninit::<LoopState>::uninit();
        boot(&mut state);
        // SAFETY: `boot` returned, and it writes every field of the state before it does (its last
        // act is the scalar writes at its end); this is the one `&mut` ever formed to the state,
        // and `main` never returns, so the borrow it hands the loop is as good as `'static`.
        let state = unsafe { state.assume_init_mut() };

        // Everything in `boot` is cold boot and stays in `.text`. Hand the built state to the
        // 250 Hz steady-state loop, which is PLACED in the F1x0's zero-wait window (see
        // `service_loop`).
        service_loop(
            &mut state.epoch_watch,
            &mut state.mailbox_link,
            &mut state.links,
            &mut state.responder.0,
            &mut state.wdg,
            state.odometer_base,
        )
    }

    /// What [`service_loop`] borrows for the life of the image: the two links, the responder,
    /// the watchdog, the mailbox epoch watch and the three boot facts the loop reads. It is a
    /// struct, not a set of locals in `main`, for one reason: a local of `main` is in `main`'s
    /// frame, and `main`'s frame is the one frame in the image that never pops. Everything that
    /// is NOT in here (the `Chip`, the motor plan, the split USART halves, the IMU handles, the input pins, the motor summary) is bring-up
    /// scaffolding, built and consumed in [`boot`] and gone before the loop's deep chains exist.
    ///
    /// Built IN PLACE: [`boot`] gets `&mut MaybeUninit<LoopState>` and each large field is written
    /// by its own `#[inline(never)]` placer ([`place_link`], [`place_mailbox_link`],
    /// [`place_responder`]) that takes the slot's raw pointer. The placer
    /// is where the construction temporaries live (a `Link` is built from a transport built from a
    /// carrier built from two DMA halves, each a value on the way), and the placer's frame pops.
    /// Built as a struct literal at the end of `boot`, those temporaries would be `boot`'s frame
    /// and the finished value would be copied into `main`'s on top of them, so the boot would be
    /// about 1.3 KB DEEPER than it is today for exactly as long as the deepest bring-up callee
    /// runs; the measured boot peak is what the paint instrument reports from now on, because
    /// `boot`'s frame is below the paint line.
    struct LoopState {
        epoch_watch: EpochWatch,
        mailbox_link: MailboxLink,
        /// The inter-board links, index = the link (port `PORT_IDX_LINK0 + i`); `None` where the
        /// link is not configured or did not come up.
        links: [Option<UartLink>; N_LINKS],
        /// Wrapped to word alignment: `Responder` is `align(2)`, and `Responder::new`'s fill of
        /// the slot through an `align(2)` pointer linked the byte-wise `__aeabi_memset` (92 B)
        /// beside the word-wise `__aeabi_memset4` the rest of the image uses; word-aligned, it
        /// uses the one already there.
        responder: Aligned4<Responder>,
        wdg: FreeWatchdog,
        /// Motor 0's stored odometer total at boot (`specs/odometer.md`).
        odometer_base: u32,
    }

    /// Word alignment for a field whose type is narrower (see `LoopState::responder`).
    #[repr(C, align(4))]
    struct Aligned4<T>(T);

    /// The whole bring-up, from the mailbox header to the watchdog, in a frame that POPS.
    ///
    /// `#[inline(never)]` is the point, not a hint: inlined into `main` this body's locals would be
    /// `main`'s frame again, which is the 2,248 B this split exists to free. The hardware side
    /// effects here are ordered: SELF_HOLD first of every bring-up, the vector table before any
    /// ring, the rings before the links, the shell before the tick source, the watchdog last; the
    /// long-lived values are written into `state`.
    ///
    /// `state` is written exactly once per field; the one read is the responder's `set_status`
    /// after its placer wrote it.
    #[inline(never)]
    fn boot(state: &mut MaybeUninit<LoopState>) {
        let st = state.as_mut_ptr();

        // Initialize the SWD mailbox header FIRST, before any bridge could attach. SAFETY: REGION_LEN
        // bytes at the fixed reserved base, owned only here, accessed only through the handle.
        let mailbox = unsafe { Mailbox::from_raw(MAILBOX_BASE as *mut u8) };
        mailbox.init_header();

        // The image is built for its chip: the context is the compiled chip's, with no detection
        // (specs/hal.md section 4.2).
        let chip = match hal::Chip::take() {
            Some(c) => c,
            None => halt(),
        };

        // Why this boot happened, read and then CLEARED, as early as the RCU base is known.
        //
        // The RCU `RSTSCK` cause flags are sticky: they accumulate until something writes RSTFC,
        // and nothing in this image ever did, so the register has been reporting every reset since
        // the part was last powered. A read of it on 2026-10-23 showed power-on and software both
        // set, which is a history and not an answer. Read-then-clear makes each boot's word name
        // that boot's reset and only that one.
        //
        // The position is the earliest one that can exist: the base comes out of the chip's
        // address table (one owner for where RCU is), and nothing between reset and here touches
        // `RSTSCK`; the IRC40K enable that shares this register is part of the watchdog bring-up,
        // much later.
        //
        // No `Err` arm: a chip whose descriptor carries no RCU base cannot have its clock tree
        // configured either, and `configure_tree` below halts on exactly that. One instrument does
        // not get to be the second thing deciding what an impossible chip does.
        if let Ok(rcu_base) = chip.rcu_base() {
            RESET_CAUSE.store(hal::reset_cause_flags(rcu_base), Ordering::Relaxed);
            hal::clear_reset_cause(rcu_base);
        }
        // `mcu` is the intrinsic chip-FAMILY tag the board reports in `IDENTITY` (NOT a role: family
        // != master/slave). Addressing is positional - the walk assigns addresses by where a board
        // sits, never by reading a hardware id - so this tag is informational only. `chip_tag` is
        // the PART this board publishes in its cyclic payload (`linkctl::ChipTag`); unlike `mcu` it
        // is not informational: a controller uses it to pick the capability table that predicts
        // this board's layout verdict (`specs/remote-ui.md` 3.5). Both are the descriptor's
        // constants (specs/hal.md section 4.2); `CHIP_IDENTITY_IS_ALLOCATED` pins them to the
        // wire's allocated values.
        let (mcu, chip_tag) = (
            chip.descriptor().node_mcu,
            ChipTag::from_u8(chip.descriptor().chip_tag),
        );

        // Bring up the production 72 MHz tree before the store + UARTs (baud divisor, flash waits).
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            halt();
        }

        // Enable the DWT cycle counter now the clock tree is up, so `CYCCNT` free-runs for the
        // whole run and a bench operator attached over SWD can take pass-scale timing windows
        // against it (the round-7a characterisation: pass-scale/wall-anchored windows are the
        // trustworthy cycle observable; below-pass NVIC-handler-mode deltas are not, which is why
        // CTRL_OBS publishes only ISR entry counts). Cheap, idempotent. Not on the Cortex-M0,
        // which has no DWT.
        #[cfg(not(feature = "mm32spin05"))]
        hal::irq::enable_cycle_counter();

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
        // The odometer's stored total, motor 0's (`specs/odometer.md`). Read here with the other
        // boot-read fields and handed to the loop, which owns every later read and the one write.
        let odometer_base = store.get(store::MOTOR_ODOMETER.at(0));

        // The boot status record (`specs/configured-links.md` section 5): every bring-up below that
        // leaves its function absent adds an entry here. The one `&mut` to the static, formed once;
        // its last use is the `set_status` hand-over at the end of this function.
        // SAFETY: single-threaded boot, before any interrupt is enabled; nothing else forms a
        // reference to `BOARD_OBS` while this one is live.
        let status: &mut StatusRecord = unsafe { &mut *addr_of_mut!(BOARD_OBS) };

        // The pin, instance and remap keys in one store pass (`specs/config-read-once.md`
        // section 5): the values the bring-ups below write into the registers, as stored.
        let layout = layout::read(store);

        // The power latch FIRST of every bring-up, on the stored pin: the board's own power rail
        // goes up the moment its pin is knowable. On the bench both boards run on debugger 3V3
        // that bypasses the latch, so the assert is verifiable only as the pin's `GPIOx.OCTL` bit;
        // it matters on battery + the inter-board cable.
        assert_self_hold(&chip, layout.self_hold, status);

        // The application owns the one Peripherals::take() (the HAL uses raw register views
        // internally and never consumes the one-shot flag); fail loud if somehow taken twice.
        // Its SYST becomes the 250 Hz tick source below.
        let core = match cortex_m::Peripherals::take() {
            Some(p) => p,
            None => halt(),
        };

        // The image runs from its chip's flash vector table; interrupts are enabled BEFORE arming
        // DMA RX.
        // The motor-era interrupt-priority ordering (HAL R7, `specs/motor-integration.md`'s
        // execution model): the period (ADC injected-EOC) vector above every USART RX vector above
        // the DMA vectors above SysTick at the stock 0xF0. Applied here, BEFORE any of those
        // vectors is unmasked (the link bring-up is below, the period vector is the motor
        // bring-up's last step), and it is the single owner of the SysTick priority too. Writing
        // priorities enables nothing.
        hal::irq::apply_motor_era_priorities(chip.irq());
        // SAFETY (enable): the vectors are the flash table's; RingBufferedRx::new unmasks its handlers.
        unsafe { cortex_m::interrupt::enable() };

        // === The links (`specs/configured-links.md` section 4): link i is port 1 + i ===
        //
        // Each link exists when both its pins are stored, and is built in place in the loop's state
        // by a popped frame ([`place_link`]). Unset, its pins are not touched. Every slot is
        // written, `None` for a link that is absent, so the state is whole when `boot` returns.
        let mut configured = [false; N_LINKS];
        for (i, c) in configured.iter_mut().enumerate() {
            // SAFETY: a pointer to slot `i` of the `links` field of `main`'s uninitialized state,
            // written once by the placer and read by nobody until `boot` returns.
            let slot = unsafe { addr_of_mut!((*st).links).cast::<Option<UartLink>>().add(i) };
            *c = place_link(slot, i as u8, &chip, layout.links[i], status);
        }

        // The mailbox, port 0, always.
        place_mailbox_link(unsafe { addr_of_mut!((*st).mailbox_link) }, mailbox);

        place_responder(
            // SAFETY: a `&mut MaybeUninit` to one field of `main`'s uninitialized state: the
            // field is written by the placer and read by nobody until `boot` returns.
            unsafe {
                &mut *addr_of_mut!((*st).responder).cast::<MaybeUninit<Aligned4<Responder>>>()
            },
            Identity {
                class: CLASS_MOTOR,
                mcu,
                fw_ver: FW_VER,
                // The GD32's factory unique ID (`specs/machine-settings.md` section 6.1), for a
                // client matching a file to a board; addressing never reads it.
                uid: chip.unique_id().to_bytes(),
            },
            store,
        );

        // === The integration boot delta (specs/integration.md, after the existing bring-up) ===

        // 1. IMU bring-up on the bus its staged pins name: I2C0 on PB6/PB7 for the standard
        //    family, I2C1 on PB10/PB11 for the classywalk offroad family, from this one image.
        //    Fails soft: the board boots link-only-plus-throttle and the outcome is observable
        //    (imu_configured in CTRL_OBS, read against imu_staged: the flags byte's b0 and b6, and
        //    the status record).
        let (imu_bus, imu_dev) = bring_up_imu(&chip, store, layout.imu, status);
        // Whether the store asks for an IMU (`imu.model` non-zero, the switch): published as
        // `CTRL_OBS` b6 so a bench read tells "never asked" from "asked and nothing came up".
        let imu_staged = layout.imu_model != 0;

        // The input pins (button + pads), each stored on its own, read together through one
        // branch-free InputGroup; an absent line samples as idle through the per-line mask.
        // Resolving each pin through the HAL enables its port clock (port C for a pad on PC15).
        let inputs = {
            let [b, pa, pb] = layout.inputs;
            let group = b.or(pa).or(pb).map(|f| {
                chip.input_group(
                    [b.unwrap_or(f), pa.unwrap_or(f), pb.unwrap_or(f)]
                        .map(|[port, pin]| hal::PinId { port, pin }),
                )
            });
            InputPins {
                has_button: b.is_some(),
                has_pad_a: pa.is_some(),
                has_pad_b: pb.is_some(),
                group,
                polarity: store.get(store::BOARD_INPUT_POLARITY),
            }
        };

        // 1b. Motor bring-up (`specs/motor-integration.md` slice 3, DISARMED): motor 0's stored
        //     configuration (`layout::read`), then the timer + injected ADC are configured and the
        //     16 kHz period ISR starts stepping the commutator. MOE is written only by the
        //     calibration of a motor stored `low_side_on`, through the arming layer's
        //     `calibration_moe` and cleared before the period vector (`specs/twelve-fet.md` 5.5). A board with no motor stored is left exactly
        //     as before: the outcome rides in `CTRL_OBS`'s motor block, and a bring-up step that
        //     fails in the status record. Placed after the interrupt priorities are set and before
        //     the tick source, so the 250 Hz task never runs against a half-built motor. A
        //     successful bring-up hands its configured timer to `arm`, which derives the one arming
        //     gate in the image from it. A board that skips the bring-up never installs a gate and
        //     is therefore UNARMABLE, not merely unarmed.
        // The buzzer (`specs/tones.md` section 2), on its stored pin: a push-pull
        // output driven low before the period ISR that toggles it is unmasked, handed to the motor
        // runtime. A board whose motor does not come up has no period ISR, and its buzzer stays
        // low and silent.
        let buzzer = layout.buzzer.map(|[port, pin]| {
            let mut p = chip.pin(hal::PinId { port, pin }).into_push_pull_output();
            let _ = p.set_low();
            p
        });
        // Every motor the image drives (`motor::COUNT`), each recorded at its own index: a motor
        // that fails a step is `NotUp` there and the other comes up alone, except that motor 1
        // rides on motor 0 (`specs/twelve-fet.md` section 5.5).
        let outcomes = motor::bring_up(
            &chip,
            &layout.motors,
            layout.battery,
            buzzer,
            PERIOD_HZ,
            core::array::from_fn(|m| {
                let m = m as u8;
                motor::BootRead {
                    noise_floor: store.get(store::MOTOR_NOISE_FLOOR.at(m)),
                    // The DC-link scale and limit, boot-read beside the floor on the image that
                    // reads the input (`specs/optional-protections.md` section 4).
                    #[cfg(feature = "gd32f103rc")]
                    dc_link_cal: store.get(store::MOTOR_DC_LINK_CAL.at(m)),
                    #[cfg(feature = "gd32f103rc")]
                    dc_link_limit: store.get(store::MOTOR_DC_LINK_LIMIT.at(m)),
                }
            }),
            arm::hw::calibration_moe,
        );
        for (m, outcome) in outcomes.iter().enumerate() {
            match *outcome {
                Ok(summary) => arm::hw::install(m, &summary.timer),
                Err(skip) => {
                    if let motor::MotorSkip::StepFailed(_) | motor::MotorSkip::NoMotor0 = skip {
                        status.record(store::MOTOR_GATE_HI_A.id(), m as u8, Reason::NotUp, 0);
                    }
                    motor::record_skip(m, skip);
                }
            }
        }

        // 2. The control-dispatch boot seam rides inside the orchestrator constructor
        //    (CONTROL_MODE + CONTROL_FRAME + CONTROL_LEADER bytes + the IMU fact, the resolution
        //    table of `specs/rigid-leader.md` section 5.1: pivoting Balance without an IMU demotes
        //    to Throttle with the mode fault; rigid Balance leads when the leader flag is set,
        //    with the mode fault if no IMU is up, and follows when it is clear). The shell static is built BEFORE the tick source exists, so no dispatch can
        //    see it half-made; the enabled DMA RX ISR never touches it. Built in a POPPED frame
        //    (`init_shell`): the ~700 B Shell value otherwise materializes in `main`'s
        //    boot frame before the static write (the slice-7 stack-budget fix).
        // The per-board LEVEL TRIMS (ATTITUDE_LEVEL_TRIM, indices 0 = pitch / 1 = roll,
        // centidegrees) ride in with the control mode: the angle this board reads while physically
        // level, subtracted from the published attitude so the balance loop's zero is the board's
        // own level. Per board because the two halves of a chassis are mirror-mounted, which is
        // what stock's per-board cal page absorbed and the only thing its two images differed by.
        // Default 0 = untrimmed, so an unstaged board behaves exactly as before.
        // The gain and reference words, read once: the shadow is built from them under the
        // compiled maxima, and reconciled with them once the arm-row function has installed the
        // stored ones (below).
        let gains_flash = read_gains(store);
        init_shell(
            [
                store.get(ATTITUDE_LEVEL_TRIM.at(0)),
                store.get(ATTITUDE_LEVEL_TRIM.at(1)),
            ],
            control::GainShadow::of_stored(gains_flash),
            // The battery-sense calibration (BOARD_BATTERY_CAL, 0 = slope / 1 = offset), boot-read
            // and seam-clamped by `VbattCal::new`, on a board that stores `battery_sense.pin` only:
            // a board that does not sense has no calibration to read.
            layout.battery.map(|_| {
                orchestrator::battery::VbattCal::new(
                    store.get(BOARD_BATTERY_CAL.at(0)),
                    store.get(BOARD_BATTERY_CAL.at(1)),
                )
            }),
            imu_bus,
            imu_dev,
            // Whether the staging asked for an IMU at all: published as the `CTRL_OBS` flags byte
            // b6, which is what makes `imu_configured = 0` readable.
            imu_staged,
            inputs,
            next_boot_count(),
            chip_tag,
            // Whether any configured function did not come up: `EV_CONFIG` on the wire for the
            // whole run (`specs/configured-links.md` section 5). Every bring-up is done by here.
            !status.is_empty(),
            // What the board stores and runs, and the peers its configured links expect
            // (`specs/machine-settings.md` sections 4 and 5).
            orchestrator::Machine::new(orchestrator::Digests::of(store), configured),
            // The low-battery warning threshold (`specs/tones.md` section 4.4), boot-read.
            store.get(store::BOARD_BATTERY_WARN),
            // The flux observer's two fields, read once, and the battery calibration its bus
            // scale converts with on a sensing board; the arm row below gives them the current
            // calibration and installs the parameters.
            crate::observer::ObserverLoop::of_stored(
                crate::observer::read(store),
                layout.battery.map(|_| {
                    (
                        store.get(BOARD_BATTERY_CAL.at(0)),
                        store.get(BOARD_BATTERY_CAL.at(1)),
                    )
                }),
            ),
        );
        // 2b. The arm-row function, once (`specs/config-read-once.md` section 5, boot step 5): the
        //     control and motor values the board runs until the next arm re-reads them, installed
        //     before the tick source exists.
        // SAFETY: the shell was written just above; single-threaded boot, no task exists yet.
        if let Some(shell) = unsafe { (*addr_of_mut!(SHELL)).as_mut() } {
            // The DC-link chains the bring-up built, before the arm-row function resets their
            // derates (`specs/optional-protections.md` section 3.2).
            #[cfg(feature = "gd32f103rc")]
            {
                shell.dc_link = motor::hw::boot_fixed().map(|b| b.and_then(|b| b.dc_link));
            }
            re_read_arm_values(shell);
            // The shadow was built under the compiled maxima; with the stored ones installed, a
            // stored gain above a compiled maximum but within its stored one runs as stored, as it
            // did when the boot read the maxima first. Nothing is tuned yet, so the reconcile
            // moves only the words the compiled bound had clamped.
            shell.orch.ctl.gains.reconcile(gains_flash);
        }

        // 3. The tick source (integration.md step-3 order: register the task table, mark the
        //    scheduler's tick-source latch, THEN enable SysTick).
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
            // The tick source is the `SysTick` vector in the flash table, enabled below.
            sched.mark_tick_source_enabled();
        }
        let mut syst = core.SYST;
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
        let wdg = match FreeWatchdog::start(&chip, WdgTimeout::from_millis(WDG_TIMEOUT_MS)) {
            Ok(w) => w,
            Err(_) => halt(),
        };

        // The status record is fixed from here: hand its entries to the responder, which answers
        // `STATUS` from them (`specs/configured-links.md` section 5). `status`'s last use was
        // above, so the shared borrow formed here is the only reference to the static.
        // SAFETY: as the `&mut` above; the responder field was written by its placer.
        unsafe {
            (*addr_of_mut!((*st).responder))
                .0
                .set_status((*addr_of!(BOARD_OBS)).entries())
        };

        // The remaining fields, LAST, so `main` finds the state complete the moment this returns.
        // SAFETY: the one write of each; `st` points at `main`'s `MaybeUninit`, no reference to
        // which exists while this runs.
        unsafe {
            addr_of_mut!((*st).wdg).write(wdg);
            addr_of_mut!((*st).epoch_watch).write(EpochWatch::new(mailbox));
            addr_of_mut!((*st).odometer_base).write(odometer_base);
        }
    }

    /// Bring link `i` up from its stored configuration and place it into its slot of the loop's
    /// state (`specs/configured-links.md` section 4): the USART as stored, the receive ring and the
    /// one-shot transmitter on its split halves, and the `Link` over them. A POPPED boot frame
    /// (shrink round 7): the ring receiver, the transmitter and the `SplitSerial` are each a value
    /// on the way to the `Link`, and here they are this frame's; the `Link` is written into the
    /// slot through the raw pointer, never returned by value.
    ///
    /// Not stored (`layout::Link` absent), the slot is `None` and no pin is touched. A HAL
    /// constructor that refuses leaves the slot `None` and records `NotUp` against `links[i].tx` at
    /// index `i`; it never halts. The two buffers are this link's own statics, formed once each.
    ///
    /// Returns whether the link is CONFIGURED: the expectation of a peer the arm gate holds the
    /// board to whether or not the USART came up (`specs/machine-settings.md` section 4.2).
    ///
    /// Runs AFTER `irq::install` and `cortex_m::interrupt::enable()`: arming either half registers
    /// and unmasks a vector.
    #[inline(never)]
    fn place_link(
        slot: *mut Option<UartLink>,
        i: u8,
        chip: &hal::Chip,
        stored: Option<layout::Link>,
        status: &mut StatusRecord,
    ) -> bool {
        let link = stored.and_then(|l| bring_up_link(i, l, chip, status));
        // SAFETY: `slot` is slot `i` of the `links` field of `main`'s `MaybeUninit<LoopState>`,
        // written here once and read by nobody until `boot` returns.
        unsafe { slot.write(link) };
        stored.is_some()
    }

    /// [`place_link`]'s body: the link, or `None` with the reason recorded.
    #[inline(always)]
    fn bring_up_link(
        i: u8,
        l: layout::Link,
        chip: &hal::Chip,
        status: &mut StatusRecord,
    ) -> Option<UartLink> {
        let field = store::LINK_TX.id();
        let usart = usart_label(l.usart);
        // The GD32F10x routes the USART by its AFIO remap, the stored one.
        #[cfg(any(feature = "gd32f103c8", feature = "gd32f103rc"))]
        hal::afio::remap(usart, l.remap);
        let refused = |status: &mut StatusRecord, e: DescriptorError| {
            status.record(field, i, Reason::NotUp, e.code())
        };
        // One bring-up, split into owned halves (specs/usart-split.md): the RX half is consumed by
        // the ring, the TX half by the transmitter.
        let (usart_tx, usart_rx) =
            match Usart::new(chip, &CLOCK, usart, pin_af(l.tx), pin_af(l.rx), LINK_BAUD) {
                Ok(u) => u.split(),
                Err(e) => {
                    refused(status, e);
                    return None;
                }
            };
        // SAFETY: the one &mut formation of this link's ring and transmit buffer (each `i` is
        // placed once), before its DMA IRQ exists. The ring `write` is the zeroing `.bss` would
        // have done (both rings are NOLOAD, see their declarations).
        let (ring, tx_buf): (&'static mut [u8; DMA_CAP], &'static mut [u8]) = unsafe {
            let ring = if i == 0 {
                addr_of_mut!(DMA_RING)
            } else {
                addr_of_mut!(DMA_RING_1)
            };
            (
                &mut (*ring).write(DmaRing([0; DMA_CAP])).0,
                (*addr_of_mut!(LINK_TX_DMA)).get_mut(i as usize)?,
            )
        };
        let rx_dma = match RingBufferedRx::new(chip, usart_rx, usart, ring) {
            Ok(r) => r,
            Err(e) => {
                refused(status, e);
                return None;
            }
        };
        // The transmit half of the same port, on its own DMA channel, armed beside the ring and
        // after the same `install` + `enable`, because arming it unmasks a vector too.
        let tx_dma = match DmaTx::new(chip, usart_tx, tx_buf, CLOCK.sysclk_hz, LINK_BAUD) {
            Ok(t) => t,
            Err(e) => {
                refused(status, e);
                return None;
            }
        };
        Some(Link::new(SerialTransport::new(
            SplitSerial::new(tx_dma, rx_dma, CLOCK.sysclk_hz),
            UART_FRAME_CAP,
        )))
    }

    /// Place the mailbox link (port 0, always present) into the loop's state. A POPPED boot frame
    /// (shrink round 7): the 264 B `Link` is this frame's temporary, not `boot`'s.
    #[inline(never)]
    fn place_mailbox_link(slot: *mut MailboxLink, mailbox: Mailbox) {
        // SAFETY: as `place_link`'s write: the `mailbox_link` field of `main`'s state, once.
        unsafe {
            slot.write(Link::new(SerialTransport::new(
                MailboxSerial::firmware(mailbox),
                swd_mailbox::FRAME_CAPACITY,
            )))
        };
    }

    /// Place the L3 responder into the loop's state with its persisted address restored. A POPPED
    /// boot frame (shrink round 7): the 280 B `Responder` is built and restored here, then written.
    ///
    /// The slot is a `&mut MaybeUninit` rather than the raw pointer the other placers take so the
    /// compiler sees the slot's word alignment (a reference carries it, a raw pointer does not):
    /// `Responder::new`'s fill of the fresh value then lowers to the word-wise `__aeabi_memset4` the
    /// image already links instead of a second, byte-wise `__aeabi_memset` body (measured +42 B).
    #[inline(never)]
    fn place_responder<F: store::Flash>(
        slot: &mut MaybeUninit<Aligned4<Responder>>,
        identity: Identity,
        store: &Store<F>,
    ) {
        let mut responder = Responder::new(N_PORTS, [PORT_SWD, PORT_UART, PORT_UART, 0], identity);
        responder.restore_addr(store);
        slot.write(Aligned4(responder));
    }

    /// The cooperative service loop (the integration.md execution model): service the links,
    /// dispatch the due tasks, feed the watchdog AFTER dispatch (R2), emit the cyclic.
    /// Busy-spin, NEVER `wfi`.
    ///
    /// **Split out of `main` and PLACED in `.hotcode`** (`specs/motor-integration.md`, "Hot-path
    /// flash placement"). This is the 250 Hz steady-state path, and on the GD32F1x0 an instruction
    /// fetched above `0x0800_8000` costs ~8.8 cycles against 1 below it. Compiled into `main` the
    /// loop shared one function body with the whole cold bring-up (chip, clock, store mount,
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
        links: &mut [Option<UartLink>; N_LINKS],
        responder: &mut Responder,
        wdg: &mut FreeWatchdog,
        odometer_base: u32,
    ) -> ! {
        // The tick captured when the current `PROBE_PORTS` started (rising edge of `probing()`);
        // `None` when no probe is in flight. The poll window is measured from it (deviation 2).
        let mut probe_start: Option<u32> = None;
        // The odometer's running total and its write decision (`specs/odometer.md`). It lives
        // here because the write belongs here: the loop is the main thread, it already samples
        // the arm fact, and it is where the OTHER deferred persist waits for a disarmed pass.
        let mut odo =
            odometer::Odometer::new(odometer_base, motor::EDGES[0].load(Ordering::Relaxed));
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
                route_handback(handed, PORT_IDX_MAILBOX, responder, &mut emits);
                route_emits(&emits, mailbox_link, links);
                true
            });

            // 2a'. Gate-1 controlled-injection hook: if an operator poked INJECT_UART_LINE_ERROR over
            //      SWD, inject ONE line error into the inter-board RX exactly as the ERRIE ISR records
            //      it, then clear the flag (one-shot). Placed before the drain below so this pass's
            //      poll_recv surfaces the self-heal (link_line_errors++ / lap_overruns flat / cyclic_age
            //      fresh, no reset). Drives the shipping self-heal path, fabricating no DMA state.
            //      The Cortex-M0 has no swap: a load and a store, the operator's poke being the
            //      only other writer.
            #[cfg(target_has_atomic = "32")]
            let inject = INJECT_UART_LINE_ERROR.swap(0, Ordering::Relaxed);
            #[cfg(not(target_has_atomic = "32"))]
            let inject = {
                let v = INJECT_UART_LINE_ERROR.load(Ordering::Relaxed);
                INJECT_UART_LINE_ERROR.store(0, Ordering::Relaxed);
                v
            };
            if inject != 0 {
                // Into the first link that came up.
                if let Some(l) = links.iter().flatten().next() {
                    l.transport().serial().inject_line_error();
                }
            }

            // 2b. Drain each inter-board link that came up (link i is port 1 + i). Bounded
            //     (integration.md, "Bounded link drain"): a link carries the peer's 250 Hz
            //     CYCLIC_STATE flood, so an unbounded drain here is exactly what collapsed the F130
            //     loop. One body over the index, so the second link costs no second copy.
            for i in 0..N_LINKS {
                bounded_drain(DRAIN_BUDGET, || {
                    let Some(n) = links
                        .get_mut(i)
                        .and_then(|l| l.as_mut())
                        .and_then(|l| l.poll_recv(&mut rxbuf))
                        .map(|f| copy_pdu(f, &mut pdu))
                    else {
                        return false;
                    };
                    emits.clear();
                    // The store comes from the static, in a borrow scoped to this statement (see
                    // `STORE`): ingest is where a `CONFIG_WRITE` reaches flash, and it must not
                    // hold a borrow across `dispatch()` below. `None` is unreachable after boot;
                    // matching rather than unwrapping keeps the impossible case out of the panic
                    // machinery.
                    // SAFETY: main-thread context, the `SHELL` discipline.
                    let port = PORT_IDX_LINK0 + i as u8;
                    let handed = match unsafe { (*addr_of_mut!(STORE)).as_mut() } {
                        Some(st) => responder.ingest(port, &pdu[..n], st, &mut emits),
                        None => None,
                    };
                    route_handback(handed, port, responder, &mut emits);
                    route_emits(&emits, mailbox_link, links);
                    true
                });
            }
            // Sample the links' recovered-loss counters into the OBS crossings (the `SplitSerial`
            // absorbs each self-healed RX condition as a counter tick, classified by the OQ1 split:
            // a wire disturbance -> line_errors, a slow-consumer lap -> lap_overruns), summed over
            // the links. The abandoned writes go one link per half of `TX_TIMEOUTS` (low = link 0,
            // high = link 1). Read through the link -> transport -> serial, published by the 250 Hz
            // callback.
            {
                let (mut line, mut lap, mut tx_to) = (0u32, 0u32, 0u32);
                for (i, l) in links.iter().enumerate() {
                    if let Some(l) = l {
                        let s = l.transport().serial();
                        line += s.line_errors() as u32;
                        lap += s.lap_overruns() as u32;
                        tx_to |= (s.tx_timeouts() as u32) << (16 * (i & 1));
                    }
                }
                LINK_LINE_ERRORS.store(line, Ordering::Relaxed);
                LINK_LAP_OVERRUNS.store(lap, Ordering::Relaxed);
                TX_TIMEOUTS.store(tx_to, Ordering::Relaxed);
            }

            // 2c. If either drain PERSISTED a config write, re-read the fields the
            //     control block shadows (`specs/remote-ui.md` section 4) and the digests
            //     (`specs/machine-settings.md` section 4.1). Once per pass rather than per drain,
            //     on a level the responder raises, so the store scans are paid on a config write
            //     and never in the steady loop.
            // SAFETY: main-thread context; the borrow ends with this statement, well before
            // `dispatch()` (the `STORE` discipline).
            if let Some(st) = unsafe { (*addr_of_mut!(STORE)).as_ref() } {
                on_config_persisted(responder, st);
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
                route_emits(&emits, mailbox_link, links);
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
            odo.fold(motor::EDGES[0].load(Ordering::Relaxed), armed);
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
            // One writer (this loop): a load and a store on the Cortex-M0.
            #[cfg(target_has_atomic = "32")]
            DISPATCH_COUNT.fetch_add(1, Ordering::Relaxed);
            #[cfg(not(target_has_atomic = "32"))]
            DISPATCH_COUNT.store(
                DISPATCH_COUNT.load(Ordering::Relaxed).wrapping_add(1),
                Ordering::Relaxed,
            );

            // 7. The watchdog feed, AFTER dispatch, never inside the link servicing (R2, the
            //    wdg-bench silicon-proven placement).
            wdg.feed();

            // 8. Step 8's emission: the pending cyclic payload leaves PORT-DIRECTED on the
            //    inter-board UART (dst 0x00, the point-to-point rule; addressed boards only,
            //    which the builder already gated). Never routed: the 250 Hz stream cannot flood
            //    the mailbox port (link-control.md, "Addressing and emission").
            let pending = {
                // SAFETY: main-thread context, after dispatch returned; scoped as above.
                unsafe { (*addr_of_mut!(SHELL)).as_mut() }
                    .and_then(|s| s.cyclic_out.take().map(|c| (c, s.cyclic_more)))
            };
            if let Some((c, more)) = pending {
                let mut payload = [0u8; OWN_LEN];
                let n = encode_cyclic(&c, &more, &mut payload);
                // The PDU scratch is free here (the drains are done this pass): reuse it as the
                // frame buffer instead of a second 64 B local. The same frame goes out every link
                // that came up: each is point-to-point to its own neighbour.
                if let Ok(p) = net::Pdu::new(
                    linkctl::OP_CYCLIC_STATE,
                    responder.addr(),
                    net::pdu::NO_ADDRESS,
                    &payload[..n],
                ) {
                    if let Ok(len) = p.encode(&mut pdu) {
                        for l in links.iter_mut().flatten() {
                            let _ = l.send(&pdu[..len]);
                        }
                    }
                }
            }

            // 8b. The SAME payload to the SWD mailbox port, decimated to 5 Hz by `ble_cyclic_tx`,
            //     and the PEER's state forwarded under the peer's own address at 2.5 Hz
            //     (`specs/link-control.md`, "The mailbox port carries both samples (2026-10-25)"):
            //     the identical PDUs, in the pass after they were built and only while a host is
            //     attached. The bridge on the SWD header re-publishes both on its BLE link. The
            //     write is all-or-nothing into RAM with no wait, so it needs no owner, adds nothing
            //     to the demand-stale bound, and a full ring drops the sample.
            //     `crate::guest_tx::emit_mailbox_samples` holds the rule and counts what the port
            //     accepted (`CTRL_OBS` word 41).
            {
                // SAFETY: main-thread context, after dispatch returned; the borrow ends with this
                // block, and nothing it calls touches the shell.
                if let Some(s) = unsafe { (*addr_of_mut!(SHELL)).as_mut() } {
                    emit_mailbox_samples(
                        &mut s.guest,
                        responder.addr(),
                        mailbox_link,
                        &mut s.guest_staged,
                        &mut pdu,
                    );
                }
            }

            // Sample the mailbox port's dropped-frame count into its OBS crossing, the same
            // read-through-the-link the inter-board port gets above. Sampled HERE, after the last
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

    /// Route the Responder's emitted PDUs to the right L2 link by emit port (0 = mailbox, 1 + i =
    /// link i). Best-effort (L2 is best-effort; the controller retransmits the acknowledged plane).
    /// A port with no live link (a link not configured or not brought up) silently drops.
    fn route_emits(
        emits: &Emits,
        mailbox: &mut MailboxLink,
        links: &mut [Option<UartLink>; N_LINKS],
    ) {
        for e in emits {
            if e.port == PORT_IDX_MAILBOX {
                let _ = mailbox.send(&e.bytes);
            } else if let Some(Some(l)) =
                links.get_mut(e.port.wrapping_sub(PORT_IDX_LINK0) as usize)
            {
                let _ = l.send(&e.bytes);
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

/// The IMU's boot probe and init, at most twice with a bus clear between
/// (`specs/imu-probe-retry.md`), factored out of the target-only `bring_up_imu` over closures so the
/// attempt logic is host-testable against a fake bus. Compiled on all targets for the host test run;
/// `allow(dead_code)` because its one caller is target-only (the same reason as [`probe_window`]).
#[allow(dead_code)]
mod imu_attempts {
    /// What the attempts came to.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ImuAttempts<E> {
        /// Probe and init both answered, on the first attempt (0) or the second (1).
        Up(u8),
        /// Both attempts failed: `NotUp`, detail 0.
        Down,
        /// The bus clear between the attempts failed (`BusHeld`); no second attempt ran.
        ClearFailed(E),
    }

    impl<E> ImuAttempts<E> {
        /// The `CTRL_OBS` word-51 count: 0 or 1 when the IMU came up, 2 when it did not (both
        /// attempts failed, or the clear between them did).
        pub fn retries(&self) -> u8 {
            match self {
                ImuAttempts::Up(n) => *n,
                ImuAttempts::Down | ImuAttempts::ClearFailed(_) => 2,
            }
        }

        /// The `NotUp` status detail when the IMU did not come up, `None` when it did: 0 for an
        /// IMU that did not answer either attempt, the HAL's error `code` for a failed clear.
        /// `inline(always)`: outlined, this one match at its one call site cost 44 B of flash.
        #[inline(always)]
        pub fn not_up_detail(&self, code: impl FnOnce(&E) -> u8) -> Option<u8> {
            match self {
                ImuAttempts::Up(_) => None,
                ImuAttempts::Down => Some(0),
                ImuAttempts::ClearFailed(e) => Some(code(e)),
            }
        }
    }

    /// Run `attempt` (probe, then init; `true` when both succeed) on `bus`; on failure run `clear`
    /// once and, if it succeeds, `attempt` once more. One call site for `attempt`, so the probe and
    /// init it inlines are in the image once.
    pub fn run<B, E>(
        bus: &mut B,
        mut attempt: impl FnMut(&mut B) -> bool,
        mut clear: impl FnMut(&mut B) -> Result<(), E>,
    ) -> ImuAttempts<E> {
        let mut retries = 0;
        loop {
            if attempt(bus) {
                return ImuAttempts::Up(retries);
            }
            if retries == 1 {
                return ImuAttempts::Down;
            }
            if let Err(e) = clear(bus) {
                return ImuAttempts::ClearFailed(e);
            }
            retries = 1;
        }
    }

    #[cfg(test)]
    mod tests {
        use super::{run, ImuAttempts};

        /// The bus `bring_up_imu` drives, faked: whether each attempt answers, in order, whether
        /// the clear reports a held line, and what was asked of it.
        struct FakeBus {
            answers: &'static [bool],
            held: bool,
            probes: usize,
            clears: usize,
        }

        impl FakeBus {
            fn new(answers: &'static [bool], held: bool) -> Self {
                FakeBus {
                    answers,
                    held,
                    probes: 0,
                    clears: 0,
                }
            }

            fn attempt(&mut self) -> bool {
                let ok = self.answers[self.probes];
                self.probes += 1;
                ok
            }

            fn clear(&mut self) -> Result<(), &'static str> {
                self.clears += 1;
                if self.held {
                    Err("BusHeld")
                } else {
                    Ok(())
                }
            }
        }

        fn run_on(bus: &mut FakeBus) -> ImuAttempts<&'static str> {
            run(bus, FakeBus::attempt, FakeBus::clear)
        }

        #[test]
        fn a_first_attempt_that_answers_is_one_probe_and_no_clear() {
            let mut bus = FakeBus::new(&[true], false);
            let r = run_on(&mut bus);
            assert_eq!(r, ImuAttempts::Up(0));
            assert_eq!(r.retries(), 0);
            assert_eq!(r.not_up_detail(|_| 0x88), None);
            assert_eq!((bus.probes, bus.clears), (1, 0));
        }

        #[test]
        fn a_second_attempt_that_answers_is_one_clear_and_count_one() {
            let mut bus = FakeBus::new(&[false, true], false);
            let r = run_on(&mut bus);
            assert_eq!(r, ImuAttempts::Up(1));
            assert_eq!(r.retries(), 1);
            assert_eq!(r.not_up_detail(|_| 0x88), None);
            assert_eq!((bus.probes, bus.clears), (2, 1));
        }

        #[test]
        fn both_attempts_failing_is_down_and_count_two() {
            let mut bus = FakeBus::new(&[false, false], false);
            let r = run_on(&mut bus);
            assert_eq!(r, ImuAttempts::Down);
            assert_eq!(r.retries(), 2);
            assert_eq!(r.not_up_detail(|_| 0x88), Some(0), "NotUp, detail 0");
            assert_eq!((bus.probes, bus.clears), (2, 1));
        }

        #[test]
        fn a_held_bus_after_the_clear_gives_up_without_a_second_probe() {
            let mut bus = FakeBus::new(&[false], true);
            let r = run_on(&mut bus);
            assert_eq!(r, ImuAttempts::ClearFailed("BusHeld"));
            assert_eq!(r.retries(), 2);
            assert_eq!(
                r.not_up_detail(|e| if *e == "BusHeld" { 0x88 } else { 0 }),
                Some(0x88),
                "NotUp with the clear's error"
            );
            assert_eq!((bus.probes, bus.clears), (1, 1));
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

/// The guest port's telemetry: the board's own `CYCLIC_STATE` sample (5 Hz) and the peer's forward
/// (2.5 Hz) go into the SWD mailbox port's ring, and the port counts what it accepted
/// (`specs/link-control.md`, "The mailbox port carries both samples (2026-10-25)" and "Counting
/// what is staged (2026-10-25)"). The bridge on the board's SWD header re-publishes both on its own
/// BLE link, which is how a phone sees them.
///
/// Outside the `target_os = "none"` firmware module so the host test run drives the real step
/// against a real mailbox ring. `allow(dead_code)` because the only non-test caller is the
/// target-only service loop.
#[allow(dead_code)]
mod guest_tx {
    use crate::motor;
    use link::{Link, SerialTransport};
    use linkctl::{CyclicMotor, CyclicState};
    use orchestrator::dispatch::PeerCyclic;
    use orchestrator::OrchestratorState;
    use swd_mailbox::MailboxSerial;

    /// The motor-1 blocks of this board's own `CYCLIC_STATE` emission (`specs/link-control.md`,
    /// "The motor-1 block"): one per motor past the first, `motor::COUNT - 1`. Every slot that holds
    /// an own emission holds its base (`CyclicState`, motor 0's words and the board's) beside these,
    /// written together. On a one-motor image this is an array of none, so the slots, the builder
    /// and the encoder carry nothing of the block.
    pub type MoreMotors = [CyclicMotor; motor::COUNT - 1];

    /// The own emission's payload length: 34 B on a one-motor image, 46 B on a two-motor one.
    pub const OWN_LEN: usize = CyclicState::ENCODED_LEN + (motor::COUNT - 1) * CyclicMotor::LEN;
    const _: () = assert!(
        OWN_LEN
            == if motor::COUNT == 1 {
                CyclicState::ENCODED_LEN
            } else {
                CyclicState::TWO_MOTOR_LEN
            }
    );

    /// Each further motor's block from the state the pass built the base from
    /// (`orchestrator::cyclic_motor`), so the blocks are the same pass's as the base.
    pub fn more_motors(orch: &OrchestratorState<{ motor::COUNT }>) -> MoreMotors {
        core::array::from_fn(|i| orchestrator::cyclic_motor(orch, i + 1))
    }

    /// Encode a base and the `K` blocks after it into `out`, returning the byte count: the own
    /// emission with [`MoreMotors`], a peer's forwarded base with none.
    pub fn encode_cyclic<const K: usize>(
        base: &CyclicState,
        more: &[CyclicMotor; K],
        out: &mut [u8; OWN_LEN],
    ) -> usize {
        let mut n = base.encode(out);
        for m in more {
            n += m.encode(&mut out[n..]);
        }
        n
    }

    /// The two samples the control pass built since the loop last emitted, latest-wins: a catch-up
    /// burst that builds one twice before the loop runs leaves the later one. The loop empties both
    /// every pass, so a sample goes into the ring at most once, in the pass after it was built.
    pub struct GuestSlots {
        own: Option<CyclicState>,
        /// The own sample's motor-1 blocks, written with [`Self::own`].
        own_more: MoreMotors,
        peer: Option<PeerCyclic>,
    }

    impl GuestSlots {
        pub const fn new() -> Self {
            GuestSlots {
                own: None,
                own_more: [CyclicMotor::ZERO; motor::COUNT - 1],
                peer: None,
            }
        }

        /// The control pass's hand-off: whatever the two cadences built this run
        /// (`orchestrator::ble_cyclic_tx`, `orchestrator::ble_peer_tx`) overwrites its slot.
        /// `more` is the own sample's blocks ([`more_motors`] of the same pass).
        pub fn fill(
            &mut self,
            own: Option<CyclicState>,
            more: MoreMotors,
            peer: Option<PeerCyclic>,
        ) {
            if own.is_some() {
                self.own = own;
                self.own_more = more;
            }
            if peer.is_some() {
                self.peer = peer;
            }
        }
    }

    /// Samples the mailbox port staged, `CTRL_OBS` word 41: `own u16 | forwards u16 << 16`, counted
    /// when the ring took the sample, wrapping within its half (`specs/link-control.md`, "Counting
    /// what is staged (2026-10-25)"). Word 40 beside it was the CC2541 port's and publishes zero.
    #[derive(Clone, Copy, Default)]
    pub struct GuestStaged {
        pub mailbox: u32,
    }

    /// One accepted sample into a packed staged word: the low half for the board's own, the high
    /// half for a forward, each wrapping on its own so a full half never carries into the other.
    pub fn bump(word: u32, forward: bool) -> u32 {
        let shift = if forward { 16 } else { 0 };
        let half = ((word >> shift) as u16).wrapping_add(1) as u32;
        (word & !(0xFFFF << shift)) | (half << shift)
    }

    /// The `CYCLIC_STATE` PDU the mailbox port carries, `[0x10][src][dst 0x00][payload]`, encoded
    /// into `pdu`; the encoded length, or `None` if `pdu` is too small. dst 0x00 by the
    /// point-to-point rule: the port carries exactly one peer (`specs/link-control.md`,
    /// "Addressing and emission").
    pub fn sample_pdu<const K: usize>(
        src: u8,
        state: &CyclicState,
        more: &[CyclicMotor; K],
        pdu: &mut [u8],
    ) -> Option<usize> {
        let mut payload = [0u8; OWN_LEN];
        let n = encode_cyclic(state, more, &mut payload);
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
    fn mailbox_sample<const MP: usize, const MN: usize, const K: usize>(
        mailbox: &mut Link<SerialTransport<MailboxSerial, MN>, MP, MN>,
        src: u8,
        state: &CyclicState,
        more: &[CyclicMotor; K],
        pdu: &mut [u8],
    ) -> bool {
        match sample_pdu(src, state, more, pdu) {
            Some(len) => mailbox.send(&pdu[..len]).is_ok(),
            None => false,
        }
    }

    /// Step 8b's samples for one pass: each sample built since the last pass goes into the mailbox
    /// ring, the own one first, and only while a host is attached
    /// (`MailboxSerial::outbound_has_consumer`): a board with nothing on SWD neither writes the ring
    /// nor inflates word 34. Both slots are emptied either way. No TX owner: a write is one copy
    /// into RAM with no wait, so this adds nothing to the 16 ms demand-stale bound.
    pub fn emit_mailbox_samples<const MP: usize, const MN: usize>(
        slots: &mut GuestSlots,
        own_src: u8,
        mailbox: &mut Link<SerialTransport<MailboxSerial, MN>, MP, MN>,
        staged: &mut GuestStaged,
        pdu: &mut [u8],
    ) {
        let own = slots.own.take();
        let peer = slots.peer.take();
        if !mailbox.transport().serial().outbound_has_consumer() {
            return;
        }
        if let Some(c) = own {
            if mailbox_sample(mailbox, own_src, &c, &slots.own_more, pdu) {
                staged.mailbox = bump(staged.mailbox, false);
            }
        }
        // The peer's base alone: the inbox keeps a peer's base, so a forward is 34 B whatever the
        // peer sent (`specs/link-control.md`, "The motor-1 block").
        if let Some(p) = peer {
            if mailbox_sample(mailbox, p.src, &p.state, &[], pdu) {
                staged.mailbox = bump(staged.mailbox, true);
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::{bump, emit_mailbox_samples, GuestSlots, GuestStaged, MoreMotors, OWN_LEN};
        use crate::motor;
        use link::{Link, SerialTransport};
        use linkctl::{CyclicMotor, CyclicState};
        use orchestrator::dispatch::PeerCyclic;
        use swd_mailbox::{layout, Bridge, Mailbox, MailboxSerial, FRAME_CAPACITY, REGION_LEN};

        /// The firmware's mailbox link: the 128 B frame (132 B) with the 72 B packet buffer.
        const MB_N: usize = FRAME_CAPACITY + 4;
        const PACKET: usize = 72;
        const OWN_SRC: u8 = 0x01;
        const PEER_SRC: u8 = 0x02;

        type MbLink = Link<SerialTransport<MailboxSerial, MB_N>, PACKET, MB_N>;

        /// The mailbox region in host RAM, `u32`-backed for the header's alignment, with the
        /// firmware's endpoint and the host's reading end over it.
        struct Rig {
            ram: Box<[u32]>,
            board: MbLink,
            host: MbLink,
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
                    slots: GuestSlots::new(),
                    staged: GuestStaged::default(),
                }
            }

            /// One step 8b.
            fn pass(&mut self) {
                let mut pdu = [0u8; net::walk::MAX_PDU];
                emit_mailbox_samples(
                    &mut self.slots,
                    OWN_SRC,
                    &mut self.board,
                    &mut self.staged,
                    &mut pdu,
                );
            }

            fn mailbox_pdus(&mut self) -> Vec<Vec<u8>> {
                let mut out = [0u8; PACKET];
                let mut v = Vec::new();
                while let Some(p) = self.host.poll_recv(&mut out) {
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

        /// Motor 1's words, distinguishable from anything a base carries.
        const MOTOR1: CyclicMotor = CyclicMotor {
            wheel_speed: -7,
            phase_peak: 0x0506,
            phase_mean: 0x0102,
            duty_on: 0x0A0B,
            wheel_rate: 0x1122_3344,
        };

        /// The board's own sample's blocks: motor 1's on the host's two-motor count.
        const MORE: MoreMotors = [MOTOR1; motor::COUNT - 1];

        fn forward(tag: u8) -> PeerCyclic {
            PeerCyclic {
                src: PEER_SRC,
                state: state(tag),
            }
        }

        fn src(pdu: &[u8]) -> u8 {
            net::Pdu::decode(pdu).unwrap().src
        }

        /// Ports: with a host attached, a due own sample and a forward both reach the mailbox link
        /// in the pass after they were built, own first, each under its own source address.
        #[test]
        fn both_samples_reach_the_mailbox_port_own_first() {
            let mut r = Rig::new(true);
            r.slots.fill(Some(state(7)), MORE, Some(forward(9)));
            r.pass();
            let mb = r.mailbox_pdus();
            assert_eq!(mb.len(), 2, "the mailbox port carried own + forward");
            assert_eq!(src(&mb[0]), OWN_SRC);
            assert_eq!(src(&mb[1]), PEER_SRC);
            assert_eq!(mb[0][0], linkctl::OP_CYCLIC_STATE);
            assert_eq!(
                mb[0][2],
                net::pdu::NO_ADDRESS,
                "dst 0x00 on the mailbox port"
            );
            assert_eq!(mb[0].len(), 3 + OWN_LEN);
            assert_eq!(mb[0][3], 7);
            assert_eq!(mb[1][3], 9);
        }

        /// The own sample of a two-motor image (the host's count) is the 46 B payload with motor
        /// 1's block at offset 34, decoding to the base and that block; the forward is the peer's
        /// 34 B base, whatever the peer sent (`specs/link-control.md`, "The motor-1 block").
        #[test]
        fn the_own_sample_carries_motor_1_and_the_forward_is_the_base() {
            assert_eq!(motor::COUNT, 2, "the host build runs the two-motor paths");
            assert_eq!(OWN_LEN, CyclicState::TWO_MOTOR_LEN);
            let mut r = Rig::new(true);
            r.slots.fill(Some(state(7)), MORE, Some(forward(9)));
            r.pass();
            let mb = r.mailbox_pdus();
            let payload = net::Pdu::decode(&mb[0]).unwrap().payload.to_vec();
            assert_eq!(
                CyclicState::decode_with_motor1(&payload),
                Ok((state(7), Some(MOTOR1)))
            );
            let fwd = net::Pdu::decode(&mb[1]).unwrap().payload.to_vec();
            assert_eq!(fwd.len(), CyclicState::ENCODED_LEN);
            assert_eq!(CyclicState::decode_with_motor1(&fwd), Ok((state(9), None)));
        }

        /// A sample goes into the ring once: the passes after the one that emitted it put nothing
        /// in, and a catch-up burst that built two before the loop ran sends the later one.
        #[test]
        fn each_sample_goes_into_the_ring_once_latest_wins() {
            let mut r = Rig::new(true);
            r.slots.fill(Some(state(1)), MORE, Some(forward(2)));
            r.slots.fill(Some(state(3)), MORE, None);
            for _ in 0..5 {
                r.pass();
            }
            let tags: Vec<u8> = r.mailbox_pdus().iter().map(|p| p[3]).collect();
            assert_eq!(tags, vec![3, 2]);
            assert_eq!(r.staged.mailbox, 1 | 1 << 16);
        }

        /// Ports: with no host attached the mailbox link is not written at all, its refused count
        /// stays zero (so word 34 is not inflated by telemetry on a board nothing reached), and
        /// the samples are not held for a host that attaches later.
        #[test]
        fn with_no_consumer_the_mailbox_is_not_written_and_nothing_is_refused() {
            let mut r = Rig::new(false);
            for _ in 0..10 {
                r.slots.fill(Some(state(3)), MORE, Some(forward(4)));
                r.pass();
            }
            assert_eq!(r.ring_used(), 0, "the outbound ring stayed empty");
            assert_eq!(r.refused(), 0);
            assert_eq!(r.staged.mailbox, 0);
            assert!(r.slots.own.is_none() && r.slots.peer.is_none());
        }

        /// Ports and Counters: a full ring drops the sample, counts exactly one refused write, and
        /// leaves the mailbox word where it was. The host's own sample is the two-motor 46 B
        /// payload, 54 B on the ring, so four fit in the 256 B ring with nothing draining it and
        /// the fifth is refused (`specs/link-control.md`, "The motor-1 block", the fit).
        #[test]
        fn a_full_ring_drops_the_sample_and_counts_one_refused_write() {
            const FIT: u8 = 4;
            let mut r = Rig::new(true);
            for k in 0..FIT {
                r.slots.fill(Some(state(k)), MORE, None);
                r.pass();
            }
            assert_eq!(r.refused(), 0);
            assert_eq!(r.staged.mailbox, FIT as u32);
            let used = r.ring_used();
            r.slots.fill(Some(state(8)), MORE, None);
            r.pass();
            assert_eq!(r.refused(), 1, "one refused write for the dropped sample");
            assert_eq!(
                r.staged.mailbox, FIT as u32,
                "a refused sample is not counted as staged"
            );
            assert_eq!(r.ring_used(), used, "nothing of it entered the ring");
            let tags: Vec<u8> = r.mailbox_pdus().iter().map(|p| p[3]).collect();
            assert_eq!(
                tags,
                (0..FIT).collect::<Vec<u8>>(),
                "the queued four, intact"
            );
        }

        /// Counters: each staged sample advances the word by one, own in the low half and forwards
        /// in the high half.
        #[test]
        fn the_staged_word_counts_own_and_forwards_in_their_own_halves() {
            let mut r = Rig::new(true);
            r.slots.fill(Some(state(1)), MORE, None);
            r.pass();
            assert_eq!(r.staged.mailbox, 1);
            r.slots.fill(None, MORE, Some(forward(2)));
            r.pass();
            assert_eq!(r.staged.mailbox, 1 | 1 << 16);
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
    /// measured 1,011 us on the F103, which is the specific way a diagnostic stops being free. So
    /// the sweep is chunked: the per-pass cost is bounded by `chunk` regardless of how
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
        /// 250 Hz callback, so it is asserted rather than argued.
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
