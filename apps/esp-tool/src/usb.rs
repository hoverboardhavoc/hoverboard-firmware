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
//! The host can close and reopen the port at any time with no event here. Two things keep the
//! console answering across that:
//!
//! - **Sending is polled, never interrupt-driven.** esp-hal's interrupt handler for this
//!   peripheral clears both the transmit and the receive flag whenever either fires. With a
//!   transmit wait armed, a receive flag raised just before the receive side armed its own wait is
//!   cleared unseen; the received bytes then sit in the FIFO, the endpoint refuses the host's
//!   further writes, and nothing wakes the reader again until a reset. Sending by polling the FIFO
//!   leaves the receive side the only user of the interrupt, and the reader also rechecks the FIFO
//!   every [`RX_RECHECK`] in case a wakeup is lost anyway.
//! - **A quiet gap drops a partial frame.** A host that went away mid-frame leaves the stream
//!   decoder waiting for the rest of a frame whose declared length would swallow the next host's
//!   requests. Bytes that arrive after [`QUIET`] with none are preceded by [`UsbEvent::Resync`].
//!
//! Nothing else writes to the port once the firmware runs: there is no log, and only a panic's
//! backtrace (`esp-backtrace` through `esp-println`) reaches it, after which the stream is dead in
//! any case. The ROM and the second-stage bootloader print at reset; the host's stream decoder
//! resyncs past those bytes.

use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_io_async::Read;
use esp_hal::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagTx};
use esp_hal::Async;
use heapless09::Vec;

/// The USB-Serial-JTAG endpoint size: the most one read returns, and one write's worth.
pub const PACKET: usize = 64;

/// How long a write may wait for the host to read it before the host counts as gone.
const WRITE_WAIT: Duration = Duration::from_millis(100);
/// How often a write waiting for FIFO room looks again.
const WRITE_POLL: Duration = Duration::from_micros(250);
/// How often the reader looks at the FIFO with no wakeup.
const RX_RECHECK: Duration = Duration::from_millis(10);
/// A gap with no bytes from the host after which a partial frame is stale. A frame is at most
/// 259 B, which a host that is still sending delivers in well under this.
const QUIET: Duration = Duration::from_millis(250);

/// What the USB side tells the service loop.
pub enum UsbEvent {
    /// Bytes from the host.
    Data(Vec<u8, PACKET>),
    /// The host was quiet for [`QUIET`]: drop any partial frame before the bytes that follow.
    Resync,
    /// The host stopped reading: drop the port's stream until it sends again.
    Gone,
}

/// USB side to service loop.
pub static EVENTS: Channel<CriticalSectionRawMutex, UsbEvent, 16> = Channel::new();
/// Service loop to USB side: one USB packet each.
pub static OUT: Channel<CriticalSectionRawMutex, Vec<u8, PACKET>, 32> = Channel::new();

/// Put `p` in the FIFO and commit it as a packet, waiting for room at most [`WRITE_WAIT`]. False
/// when the host did not make room in time.
async fn send(tx: &mut UsbSerialJtagTx<'static, Async>, p: &[u8]) -> bool {
    let deadline = Instant::now() + WRITE_WAIT;
    for &b in p {
        while tx.write_byte_nb(b).is_err() {
            if Instant::now() >= deadline {
                return false;
            }
            Timer::after(WRITE_POLL).await;
        }
    }
    // Commit the packet; the next write waits for the host to take it.
    let _ = tx.flush_tx_nb();
    true
}

/// Run the USB console forever.
#[embassy_executor::task]
pub async fn run(usb: UsbSerialJtag<'static, Async>) {
    let (mut rx, mut tx) = usb.split();
    join(
        async {
            let mut buf = [0u8; PACKET];
            let mut last = Instant::now();
            loop {
                // The driver's errors are `Infallible`; a timeout just looks at the FIFO again.
                let Ok(Ok(n)) = with_timeout(RX_RECHECK, rx.read(&mut buf)).await else {
                    continue;
                };
                if n == 0 {
                    continue;
                }
                let now = Instant::now();
                if now - last >= QUIET {
                    EVENTS.send(UsbEvent::Resync).await;
                }
                last = now;
                let data = Vec::from_slice(&buf[..n]).unwrap_or_default();
                EVENTS.send(UsbEvent::Data(data)).await;
            }
        },
        async {
            loop {
                let p = OUT.receive().await;
                if !send(&mut tx, &p).await {
                    while OUT.try_receive().is_ok() {}
                    EVENTS.send(UsbEvent::Gone).await;
                }
            }
        },
    )
    .await;
}
