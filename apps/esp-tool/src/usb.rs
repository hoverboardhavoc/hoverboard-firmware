//! The USB console (`specs/esp-tool.md`, "The terminal tool", "The USB console is a second way
//! in"): the C3's USB-Serial-JTAG port carries the same L2 stream as the BLE link, as the ESP's
//! port 2, so a host on USB (`hoverboard-cli --serial`, or `--tcp` through a relay) is a
//! controller like a central on BLE.
//!
//! The bytes are not interpreted here: what the host sends goes to the service loop as
//! [`UsbEvent::Data`], and the loop hands back USB packets of at most [`PACKET`] bytes through
//! [`OUT`]. The port has no connect event; the router attaches it on the first bytes.
//!
//! The peripheral holds what it sends until the host reads it, and a host with the port closed
//! reads nothing, so a write that has not gone out in [`WRITE_WAIT`] is given up: the rest of the
//! queue is dropped and the service loop told the host is gone. The firmware never stalls on it.
//!
//! Nothing else writes to the port once the firmware runs: there is no log, and only a panic's
//! backtrace (`esp-backtrace` through `esp-println`) reaches it, after which the stream is dead in
//! any case. The ROM and the second-stage bootloader print at reset; the host's stream decoder
//! resyncs past those bytes.

use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration};
use embedded_io_async::{Read, Write};
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_hal::Async;
use heapless09::Vec;

/// The USB-Serial-JTAG endpoint size: the most one read returns, and one write's worth.
pub const PACKET: usize = 64;

/// How long a write may wait for the host to read it before the host counts as gone.
const WRITE_WAIT: Duration = Duration::from_millis(100);

/// What the USB side tells the service loop.
pub enum UsbEvent {
    /// Bytes from the host.
    Data(Vec<u8, PACKET>),
    /// The host stopped reading: drop the port's stream until it sends again.
    Gone,
}

/// USB side to service loop.
pub static EVENTS: Channel<CriticalSectionRawMutex, UsbEvent, 16> = Channel::new();
/// Service loop to USB side: one USB packet each.
pub static OUT: Channel<CriticalSectionRawMutex, Vec<u8, PACKET>, 32> = Channel::new();

/// Run the USB console forever.
#[embassy_executor::task]
pub async fn run(usb: UsbSerialJtag<'static, Async>) {
    let (mut rx, mut tx) = usb.split();
    join(
        async {
            let mut buf = [0u8; PACKET];
            loop {
                // The driver's errors are `Infallible`.
                let Ok(n) = rx.read(&mut buf).await;
                if n > 0 {
                    let data = Vec::from_slice(&buf[..n]).unwrap_or_default();
                    EVENTS.send(UsbEvent::Data(data)).await;
                }
            }
        },
        async {
            loop {
                let p = OUT.receive().await;
                let sent = with_timeout(WRITE_WAIT, async {
                    tx.write_all(&p).await?;
                    tx.flush().await
                })
                .await;
                if !matches!(sent, Ok(Ok(()))) {
                    while OUT.try_receive().is_ok() {}
                    EVENTS.send(UsbEvent::Gone).await;
                }
            }
        },
    )
    .await;
}
