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
//! On a host target it degrades to an empty `main` (it cannot link as a cortex-m image nor the
//! target-gated HAL), so a host `cargo build`/`cargo test` over the workspace stays green.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {

    use board::plumbing::{read_fields, reserved_set, AllowlistPort, BoardObs};
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use embedded_hal::digital::OutputPin;
    use embedded_io::{ErrorType, Read, ReadReady, Write};
    use link::{Link, SerialTransport};
    use net::walk::{Emits, Responder, PORT_BLE, PORT_SWD, PORT_UART};
    use panic_halt as _;
    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::delay::Delay;
    use runtime_hal::descriptor::ClockPath;
    use runtime_hal::irq::{install, RamVectorTable, MAX_VECTORS};
    use runtime_hal::{detect_chip, PeriphLabel, PolledSerial, RingBufferedRx, SplitSerial, Usart};
    use store::{FmcFlash, Store, LINK_SET};
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
    const BLE_NAME: &str = "hb-s5a";
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
    /// Each link's `StreamFramer` buffer (the largest stream frame). Floored at the largest carrier's
    /// `frame_capacity` (the mailbox, 128) + the 4-byte SOF/len/CRC overhead = 132.
    const FRAMER_N: usize = 132;
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

    /// VTOR alignment invariant: the `cortex_m::singleton!` that places `VECTORS` carries
    /// `RamVectorTable`'s alignment through its `MaybeUninit`, so as long as the type stays `align(512)`
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
    // Board-layout validation (specs/board-model.md, slicing item 4): after the store mounts and
    // before any board-field bring-up, the persisted pin layout is read and validated against the
    // detected chip; the outcome lands in the SWD-readable BOARD_OBS block. NOTHING consumes the
    // resulting BoardPlan yet (build-what-you-exercise: no board-field bring-up exists), so a
    // valid and an invalid layout boot identically today (the link bring-up below IS the
    // link-only posture); the record is the observable difference.
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

    // The three concrete L2 links (heterogeneous serials, one L2 code path each).
    type MailboxLink = Link<SerialTransport<MailboxSerial, FRAMER_N>, PACKET>;
    type UartLink = Link<SerialTransport<SplitSerial<RingBufferedRx>, FRAMER_N>, PACKET>;
    // The BLE link rides the data-mode gate type (`ble::Pipe`, specs/ble.md): a link can only be
    // built on a serial KNOWN to be in transparent data mode (handshake arm: `bring_up`; fallback
    // arm: `Pipe::assume_data_mode` from the persisted link-set knowledge).
    type BleLink = Link<SerialTransport<ble::Pipe<PolledSerial>, FRAMER_N>, PACKET>;

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

        // Validate the persisted board layout (specs/board-model.md checks 1-4): the reserved
        // set is the compiled allowlist minus the LINK_SET-freed ports plus SWD (the plumbing
        // helper owns the freeing rule; the allowlist pin facts come from SAFE_LINK_USARTS,
        // their single owner), the fields arrive through the registry defaults, and the chip
        // capabilities through the HalCaps adapter. On Ok, the success record and NOTHING else
        // (no bring-up consumes the BoardPlan yet); on Err, the failure record naming the
        // offending field - and the boot below proceeds unchanged either way, which IS the
        // fail-loud contract's link-only posture (allowlist links + mailbox + config stay up,
        // no board-field bring-up exists to withhold).
        let allowlist = SAFE_LINK_USARTS.map(|u| AllowlistPort {
            link_set_bit: u.port,
            pins: u.pins,
        });
        let reserved = reserved_set(&allowlist, link_set);
        let obs = match board::validate(
            &read_fields(&store),
            &HalCaps { chip: &chip },
            reserved.as_slice(),
            Some(BOOT_SELF_HOLD),
        ) {
            Ok(_plan) => BoardObs::success(),
            Err(e) => BoardObs::failure(&e),
        };
        // SAFETY: single-threaded boot, interrupts not yet enabled, the one writer this boot;
        // via a raw pointer so no reference to the `static mut` is formed (the BLE_PROBE_OBS
        // pattern).
        unsafe { *core::ptr::addr_of_mut!(BOARD_OBS) = obs };

        // A SysTick busy-delay for the polled AT bring-up (phase 1, before any interrupt is enabled).
        // The application owns the one Peripherals::take() (runtime-hal DECISIONS #13: the HAL uses
        // raw register views internally and never consumes the one-shot flag, so ordering vs
        // detect_chip is unconstrained). take() after detect works; fail loud if somehow taken twice.
        let core = match cortex_m::Peripherals::take() {
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

        // === Phase 1: the BT-probe (active, polled, 9600) on the safe USART that carries the module ===
        //
        // On the bench the module is USART2 (the master's CC2541). Configured boards bring up exactly
        // the link-set: re-establish the BLE link only if its port bit is set, and never probe a port
        // outside the set. Unconfigured boards run the cheap probe; the one that answers AT+OK is it.
        // The allowlist entry for the BLE port (USART2), capability-gated by the HAL model
        // (supports_rx: per-chip and self-updating, l3.md).
        let ble_usart = SAFE_LINK_USARTS
            .iter()
            .find(|u| u.port == PORT_IDX_BLE && runtime_hal::supports_rx(&chip, u.usart))
            .map(|u| u.usart);
        let want_ble = ble_usart.is_some()
            && if configured {
                link_set & link_bit(PORT_IDX_BLE) != 0
            } else {
                true // unconfigured: probe the allowlisted BLE port
            };

        let mut ble_link: Option<BleLink> = None;
        if let (true, Some(inst)) = (want_ble, ble_usart) {
            if let Ok(serial) =
                PolledSerial::new(&chip, &CLOCK, inst, (gpiob.pb10, gpiob.pb11), BT_BAUD)
            {
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
                let answered_at = cold_boot_probe(&mut observed, &mut delay);
                let serial = observed.into_inner();

                if answered_at {
                    // Command mode: full AT bring-up (NAME / intervals / SET=1 -> advertises / MODE=DATA).
                    if let Ok(pipe) = ble::Module::new(BLE_NAME).bring_up(serial, &mut delay) {
                        // Transparent data mode now; the link rides the gate type itself.
                        ble_link = Some(Link::new(SerialTransport::new(pipe, BLE_FRAME_CAP)));
                    }
                } else if configured {
                    // Data-mode fallback (l3.md): the link-set already identifies this port as the BLE
                    // module, but it answered no `AT` even after the FULL patient probe -- a warm reset
                    // left it in transparent data mode, still advertising. Register it as a live data-mode
                    // link WITHOUT re-handshaking: its BLE identity is known from the link-set, so no AT
                    // identification is needed. The patient probe is the prerequisite that makes this safe:
                    // an `AT` miss now genuinely means data-mode, not a not-yet-ready cold boot (which the
                    // old fixed ~750 ms window misread ~50% of the time, registering a SILENT module live).
                    ble_link = Some(Link::new(SerialTransport::new(
                        ble::Pipe::assume_data_mode(serial),
                        BLE_FRAME_CAP,
                    )));
                }
                // Unconfigured + no AT+OK: not a module (e.g. the IMU's I2C0 on USART0-remap); stays empty.
            }
        }

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
        // The RAM vector table, the DMA ring, and the L2 `Link`s all live in `'static` storage carved
        // by `cortex_m::singleton!` (a safe `&'static mut` via an internal `MaybeUninit` + once-guard):
        // this keeps the bulky `Link`s out of `main`'s stack frame (the deep ASSIGN->flash-write peak
        // stays in budget) and removes the `static mut` + raw-pointer `unsafe` the audit flagged. The
        // `RamVectorTable`'s `align(512)` (VTOR) is preserved (the singleton's `MaybeUninit` carries it).
        let vectors = cortex_m::singleton!(: RamVectorTable = RamVectorTable {
            slots: [0; MAX_VECTORS],
        })
        .unwrap();
        // VTOR requires the table aligned to its power-of-two granule (`RamVectorTable` is `align(512)`,
        // which the singleton's `MaybeUninit` carries). Guard it: a misplaced table is a silent boot
        // brick (VTOR ignores the low bits), so fail loud here instead.
        if !(vectors.slots.as_ptr() as usize).is_multiple_of(512) {
            halt();
        }
        // Route interrupts through the RAM vector table and enable them BEFORE arming DMA RX.
        // SAFETY (install): RAM init done, no peripheral IRQ enabled yet, `vectors` is a 'static table.
        unsafe { install(vectors, chip.irq()) };
        // SAFETY (enable): the table is installed; RingBufferedRx::new registers + unmasks its handlers.
        unsafe { cortex_m::interrupt::enable() };
        let dma_buf = cortex_m::singleton!(: [u8; DMA_CAP] = [0; DMA_CAP]).unwrap();
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

        let mut epoch_watch = EpochWatch::new(mailbox);
        let mut probe_idle: u32 = 0;
        let mut link_set_saved = configured; // once assigned, persist LINK_SET once
        let mut rxbuf = [0u8; PACKET];
        let mut pdu = [0u8; net::walk::MAX_PDU];

        // The cooperative service loop. Busy-spin, NEVER wfi.
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
                let mut emits = Emits::new();
                responder.ingest(PORT_IDX_MAILBOX, &pdu[..n], &mut store, &mut emits);
                route_emits(&emits, &mut mailbox_link, &mut uart_link, &mut ble_link);
            }

            // 2b. Drain the inter-board UART link (port 1), if it came up.
            while let Some(n) = uart_link
                .as_mut()
                .and_then(|l| l.poll_recv(&mut rxbuf))
                .map(|f| copy_pdu(f, &mut pdu))
            {
                saw_inbound = true;
                let mut emits = Emits::new();
                responder.ingest(PORT_IDX_UART, &pdu[..n], &mut store, &mut emits);
                route_emits(&emits, &mut mailbox_link, &mut uart_link, &mut ble_link);
            }

            // 2c. Drain the BLE link (port 2), if a module was brought up.
            while let Some(n) = ble_link
                .as_mut()
                .and_then(|l| l.poll_recv(&mut rxbuf))
                .map(|f| copy_pdu(f, &mut pdu))
            {
                saw_inbound = true;
                let mut emits = Emits::new();
                responder.ingest(PORT_IDX_BLE, &pdu[..n], &mut store, &mut emits);
                route_emits(&emits, &mut mailbox_link, &mut uart_link, &mut ble_link);
            }

            // 3. Probe window: once probing, wait out a short idle, then emit PORTS.
            if responder.probing() {
                probe_idle = if saw_inbound {
                    0
                } else {
                    probe_idle.saturating_add(1)
                };
                if probe_idle >= PROBE_IDLE {
                    let mut emits = Emits::new();
                    responder.poll_probe(&mut emits);
                    route_emits(&emits, &mut mailbox_link, &mut uart_link, &mut ble_link);
                    probe_idle = 0;
                }
            } else {
                probe_idle = 0;
            }

            // 4. Persist LINK_SET once, at assignment (specs/l3.md: "Once assigned it persists the set
            //    of ports that came up live"). The Responder wrote node_address on the ASSIGN; record
            //    the discovered link-set alongside it, so the next (configured) boot skips the probe.
            if !link_set_saved && responder.addr() != net::pdu::NO_ADDRESS {
                let _ = store.set(LINK_SET, discovered);
                link_set_saved = true;
            }

            nop(); // preemptible housekeeping slot (the future control ISR)
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
