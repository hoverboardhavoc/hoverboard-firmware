//! The BLE port's L2 carrier: the serial the `ble::Pipe` wraps once the AT bring-up is done.
//!
//! The BLE port is the one link port whose carrier has to change identity mid-boot, and this module
//! is where that is decided and named. The AT bring-up is a polled conversation (`crates/ble`: write
//! a command, drain its `AT+OK` ack within the step window, repeat), it runs before the RAM vector
//! table is installed, and it owns the port exclusively while it runs. The steady-state data path
//! wants the opposite: nothing polled, no per-byte CPU work, and a receive buffer deep enough that
//! the control loop's absence costs no bytes. So the port is brought up polled, and ADOPTED into its
//! steady-state carrier once interrupts exist ([`adopt`]).
//!
//! # Why the carrier is not the polled port
//!
//! At 9600 8N1 a character takes **1,041.7 us**. The 250 Hz control callback is DWT-bracketed at
//! **1,011 us** on the F103 and **1,399 us** on an IMU-equipped F130, and the bounded wedged-I2C path
//! at **3,941 us** worst case (`specs/bench-evidence/2026-08-02/wide-division/RECORD.md`). On an
//! IMU-equipped board the callback is LONGER than a character time, so a polled port whose entire
//! receive buffer is the one-byte data register is structurally deaf once per control tick.
//!
//! That is not a worry, it is a measurement: a 47.5 s drive capture sampling `CTRL_OBS` over SWD
//! while driving from a phone stepped `rx_ovr` 2,507 -> 3,141, **634 overruns in 47.5 s** (~13/s),
//! with `rx_lerr` flat at **zero** throughout. Line errors are what a marginal radio gives you and
//! overruns are what you get when the receiver was not looking, so a flat zero on the first rules the
//! link out: the bytes arrived and the firmware missed them. Each loss fails one frame's CRC, L2 does
//! not retransmit, and at 20 Hz drive frames four consecutive losses zero the demand and the wheels
//! sag. Ten demand sags in that window, every one with `rx_ovr` stepping across it.
//!
//! # What the carrier is
//!
//! [`BleSerial`] is a DMA ring: circular DMA refills a `'static` buffer with no CPU involvement per
//! byte, and the reader drains bytes behind the live write position. The receive buffer stops being
//! one byte and becomes [`RING_CAP`], so a caller-away window costs nothing until the DMA laps the
//! whole buffer, which at 9600 baud takes far longer than any window the image has.
//!
//! The DMA ring is chosen over the interrupt-buffered receiver deliberately, on two counts:
//!
//! - **Latency immunity.** An interrupt-driven receiver must be serviced within a character time or
//!   its one-byte register overruns exactly as the polled port's does, which makes it a question of
//!   NVIC priority against a 1,399 us control ISR. The DMA engine writes RAM with no CPU involvement
//!   at all, so the receive path cannot be starved by ISR latency; only a full lap loses data.
//! - **It is already in the image.** The inter-board link ships `SplitSerial<RingBufferedRx>`, so the
//!   ring backend, its reader and the adapter are already monomorphized. The interrupt-buffered
//!   backend would be a second receiver type linked for one port.
//!
//! # Ordering
//!
//! [`adopt`] must run AFTER `irq::install` has flipped `VTOR` and interrupts are enabled: arming a
//! ring registers and unmasks its vectors, so the table it routes through has to exist first. That is
//! why the BLE port is brought up in boot phase 1 and adopted in phase 2, next to the inter-board
//! ring, rather than becoming a link where it is probed.

#![cfg(any(target_arch = "arm", test))]

use runtime_hal::error::DescriptorError;
use runtime_hal::{Chip, PeriphLabel, PolledSerial, RingBufferedRx, SplitSerial};

/// The BLE port's steady-state L2 carrier: the polled TX half plus a DMA-ring receiver.
///
/// The type the `ble::Pipe` wraps and `link`'s `SerialTransport` drives for the life of the link.
pub type BleSerial = SplitSerial<RingBufferedRx>;

/// The BLE port's DMA RX ring capacity, in bytes.
///
/// Sized from the wire, not guessed. Two independent bounds meet here and 64 clears both:
///
/// - **The caller-away window.** The worst window in the image is the bounded wedged-I2C path at
///   3,941 us, about four characters at 9600 8N1. 64 bytes is a 16x margin on that.
/// - **The largest inbound burst.** The module's UART bridge coalesces forwarded writes up to 64 B,
///   which is also `net::walk::MAX_PDU`, so one full burst fits without relying on the reader
///   interleaving.
///
/// A lap takes 64 characters = **66.7 ms** of wire time to fill, which is more than sixteen 250 Hz
/// control periods, so the reader has to miss many consecutive passes before a byte is at risk.
pub const RING_CAP: usize = 64;

/// Adopt the AT-phase polled port into the steady-state DMA-ring carrier.
///
/// Unwraps the polled serial back to its `Usart`, splits it into owned halves, arms circular DMA RX
/// on the RX half and rejoins the TX half through the adapter. The port keeps its configuration
/// across the swap (the `Usart` carries it), so nothing is re-derived and the baud is not re-written.
///
/// `instance` is the resolved BLE USART for this board (USART2 on the standard family, USART0 on the
/// classywalk offroad one); it selects the DMA channel and the vectors. `ring` is the `'static`
/// buffer the DMA writes.
///
/// The caller must already have installed the RAM vector table and enabled interrupts; see the module
/// docs. Errors are the HAL's arming errors, of which the reachable one is the DMA channel's
/// write-back self-check failing.
pub fn adopt(
    chip: &Chip,
    polled: PolledSerial,
    instance: PeriphLabel,
    ring: &'static mut [u8],
) -> Result<BleSerial, DescriptorError> {
    let (tx, rx) = polled.into_usart().split();
    let ring_rx = RingBufferedRx::new(chip, rx, instance, ring)?;
    Ok(SplitSerial::new(tx, ring_rx))
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_io::Read;
    use runtime_hal::addr::AddrTable;
    use runtime_hal::clock::ClockConfig;
    use runtime_hal::config::{Oversampling, UsartConfig, UsartFrame};
    use runtime_hal::descriptor::{
        AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize,
    };
    use runtime_hal::reg::mock;
    use runtime_hal::usart::Usart;

    /// The BLE module's baud (`ble::at::BAUD`), and the character time that follows from it.
    const BAUD: u32 = 9_600;
    /// One 8N1 character is 10 bit-times: 10 / 9600 s = 1,041.7 us, in tenths of a microsecond.
    const CHAR_TIME_TENTHS_US: u32 = 10_417;

    /// The measured caller-away windows this port has to survive, in microseconds
    /// (`specs/bench-evidence/2026-08-02/wide-division/RECORD.md`): the 250 Hz control callback on
    /// the F103 and on an IMU-equipped F130, and the bounded wedged-I2C worst case.
    const WINDOWS_US: [u32; 3] = [1_011, 1_399, 3_941];

    /// How many characters land in a caller-away window of `us` microseconds.
    fn chars_in_window(us: u32) -> usize {
        (us * 10 / CHAR_TIME_TENTHS_US) as usize + 1
    }

    /// The two BLE wirings this one image ships, as (label, descriptor bits, USART instance).
    ///
    /// Mirrored across the fleet: the standard family puts the CC2541 on USART2 (PB10/PB11), the
    /// classywalk offroad family on USART0 (PB6/PB7). Both are exercised, because the carrier must
    /// hold on both and the DMA channel/vector resolution differs between them.
    fn wirings() -> [(&'static str, ClockPath, IrqLayout, PeriphLabel, u32); 2] {
        [
            (
                "F10x / USART2 (PB10-PB11)",
                ClockPath::F10xRcc,
                IrqLayout::F10xSeparate,
                PeriphLabel::Usart2,
                0x4000_4800,
            ),
            (
                "F1x0 / USART0 (PB6-PB7)",
                ClockPath::F1x0Rcu,
                IrqLayout::F1x0Grouped,
                PeriphLabel::Usart0,
                0x4001_3800,
            ),
        ]
    }

    fn bench_chip(clock: ClockPath, irq: IrqLayout, usart: PeriphLabel, base: u32) -> Chip {
        let mut addrs = AddrTable::new();
        addrs.set(usart, base);
        addrs.set(PeriphLabel::Rcu, 0x4002_1000);
        Chip::from_descriptor(McuDescriptor {
            gpio: GpioPath::ApbCrlCrh,
            clock,
            adc: AdcPath::Single,
            irq,
            addrs,
            flash_page: PageSize::K1,
            flash_kib: 64,
            adv_timers: 1,
            adc_count: 2,
        })
    }

    /// The BLE port as the AT bring-up leaves it: a polled serial on the module's USART at 9600.
    fn at_phase_port(chip: &Chip, usart: PeriphLabel) -> PolledSerial {
        let cfg = UsartConfig {
            usart,
            baud: BAUD,
            frame: UsartFrame::EIGHT_N_ONE,
            oversampling: Oversampling::By16,
        };
        let port = PolledSerial::from_usart(
            Usart::bring_up(chip, &ClockConfig::REFERENCE_72M_IRC8M, &cfg).expect("USART bring-up"),
        );
        // Declare the device read side effect the polled receiver depends on (reading the data
        // register clears RBNE). The HAL owns the offsets; without it a drained byte appears to
        // arrive over and over, which would make a polled carrier's failure unreadable.
        port.mock_declare_device_reads();
        port
    }

    /// **The property.** A character that arrives while the caller is away is still delivered.
    ///
    /// This is the defect the BLE port was measured to have and the reason its carrier changed, so it
    /// is asserted against the carrier the port actually ships ([`BleSerial`]) rather than against a
    /// model of one. The windows are the measured ones, not round numbers: at 9600 8N1 the F130's
    /// 1,399 us control callback is longer than a character time, so a carrier that cannot hold a
    /// window loses data on an ordinary control tick, not only under stress.
    ///
    /// Run against the polled carrier this port used to ship, it fails: the one-byte receive register
    /// is the whole buffer, so the window's second character is gone before the caller returns.
    #[test]
    fn a_character_arriving_while_the_caller_is_away_is_still_delivered() {
        for (name, clock, irq, usart, base) in wirings() {
            for window_us in WINDOWS_US {
                let _g = mock::lock();
                mock::reset();
                // The device behaviour the DMA path depends on, declared by the harness rather than
                // manufactured by the code under test: writing the DMA INTC clears the matching
                // INTF bits (the wrap-counter ISR's flag clear).
                mock::w1c_pair(
                    runtime_hal::dma::DMA0_BASE + 0x04,
                    runtime_hal::dma::DMA0_BASE,
                );
                runtime_hal::usart_rx::reset_for_test();
                runtime_hal::dma::reset_for_test();
                runtime_hal::irq::mock_vtor::reset();

                let chip = bench_chip(clock, irq, usart, base);
                let polled = at_phase_port(&chip, usart);

                // Adopt the port exactly as boot phase 2 does, then let the vectors exist.
                let ring: &'static mut [u8] = vec![0u8; RING_CAP].leak();
                let mut carrier = adopt(&chip, polled, usart, ring).expect("arm the BLE ring");
                runtime_hal::irq::install_mock(irq, 0x2000_4000);

                // The caller is away for the whole window: characters arrive, nothing reads.
                let n = chars_in_window(window_us);
                let sent: Vec<u8> = (0..n).map(|i| 0x40 + i as u8).collect();
                for &b in &sent {
                    carrier.inject_rx_byte(b);
                }

                // The caller returns and drains.
                let mut got = vec![];
                let mut buf = [0u8; 32];
                for _ in 0..4 {
                    match carrier.read(&mut buf) {
                        Ok(0) => break,
                        Ok(k) => got.extend_from_slice(&buf[..k]),
                        Err(e) => panic!("{name}, {window_us} us: {e:?}"),
                    }
                }

                assert_eq!(
                    got, sent,
                    "{name}: a {window_us} us caller-away window ({n} characters at {BAUD} 8N1) \
                     must cost no bytes"
                );
            }
        }
    }

    /// The negative control, and what makes a bench zero mean something: the carrier that holds a
    /// window must also report that it held it. `ble_rx_losses` (CTRL_OBS word 29) is packed
    /// `lap_overruns | line_errors << 16`, and both halves stay at zero across every window.
    ///
    /// Without this, a carrier that silently dropped bytes and a carrier that lost none would read
    /// the same on the bench.
    #[test]
    fn holding_the_window_leaves_the_loss_instrument_at_zero() {
        for (name, clock, irq, usart, base) in wirings() {
            let _g = mock::lock();
            mock::reset();
            mock::w1c_pair(
                runtime_hal::dma::DMA0_BASE + 0x04,
                runtime_hal::dma::DMA0_BASE,
            );
            runtime_hal::usart_rx::reset_for_test();
            runtime_hal::dma::reset_for_test();
            runtime_hal::irq::mock_vtor::reset();

            let chip = bench_chip(clock, irq, usart, base);
            let polled = at_phase_port(&chip, usart);
            let ring: &'static mut [u8] = vec![0u8; RING_CAP].leak();
            let mut carrier = adopt(&chip, polled, usart, ring).expect("arm the BLE ring");
            runtime_hal::irq::install_mock(irq, 0x2000_4000);

            // Several windows back to back, drained between them, well past one full lap of the ring.
            let mut buf = [0u8; 32];
            for round in 0..8u8 {
                for i in 0..chars_in_window(3_941) {
                    carrier.inject_rx_byte(round * 16 + i as u8);
                }
                let _ = carrier.read(&mut buf).expect("no condition");
            }

            let packed = carrier.lap_overruns() as u32 | ((carrier.line_errors() as u32) << 16);
            assert_eq!(
                packed, 0,
                "{name}: ble_rx_losses must read 0 when nothing was dropped"
            );
        }
    }
}
