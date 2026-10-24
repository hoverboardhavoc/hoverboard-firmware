//! The byte links besides BLE (`specs/esp-tool.md`, "The terminal tool"): the ESP's USB console
//! opened directly (`--serial`), or reached through a serial-to-TCP relay (`--tcp`). Both carry
//! the same L2 stream as the BLE link and go under the same [`Framed`](crate::wire::Framed).

use std::io::{ErrorKind, Read, Write};
use std::net::TcpStream;
use std::time::Duration;

use crate::wire::ByteLink;

/// A read timeout the OS accepts (zero means "block forever" to some of them, an error to others).
fn at_least_1ms(t: Duration) -> Duration {
    t.max(Duration::from_millis(1))
}

/// The USB console's CDC port. The rate is set for form: USB CDC ignores it.
pub struct Serial {
    port: Box<dyn serialport::SerialPort>,
}

impl Serial {
    /// Open `device` (`/dev/ttyACM0`, `/dev/cu.usbmodem*`). DTR and RTS are left as the open sets
    /// them: on the C3's USB-Serial-JTAG a change of one without the other resets the chip.
    pub fn open(device: &str) -> Result<Self, String> {
        let port = serialport::new(device, 115_200)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| format!("{device}: {e}"))?;
        Ok(Serial { port })
    }
}

impl ByteLink for Serial {
    fn write(&mut self, bytes: &[u8]) {
        let _ = self.port.write_all(bytes);
        let _ = self.port.flush();
    }

    fn read(&mut self, timeout: Duration) -> Vec<u8> {
        let _ = self.port.set_timeout(at_least_1ms(timeout));
        let mut buf = [0u8; 512];
        match self.port.read(&mut buf) {
            Ok(n) => buf[..n].to_vec(),
            Err(_) => Vec::new(),
        }
    }
}

/// The USB console through a relay that moves its bytes over TCP (`socat` on the machine the ESP
/// is plugged into).
pub struct Tcp {
    stream: TcpStream,
    closed: bool,
}

impl Tcp {
    /// Connect to `addr` (`host:port`).
    pub fn connect(addr: &str) -> Result<Self, String> {
        let stream = TcpStream::connect(addr).map_err(|e| format!("{addr}: {e}"))?;
        let _ = stream.set_nodelay(true);
        Ok(Tcp {
            stream,
            closed: false,
        })
    }
}

impl ByteLink for Tcp {
    fn write(&mut self, bytes: &[u8]) {
        let _ = self.stream.write_all(bytes);
    }

    fn read(&mut self, timeout: Duration) -> Vec<u8> {
        if self.closed {
            // Nothing more will come: wait out the timeout rather than spin on end-of-stream.
            std::thread::sleep(timeout);
            return Vec::new();
        }
        let _ = self.stream.set_read_timeout(Some(at_least_1ms(timeout)));
        let mut buf = [0u8; 512];
        match self.stream.read(&mut buf) {
            Ok(n) if n > 0 => buf[..n].to_vec(),
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => Vec::new(),
            end => {
                let why = end.map_or_else(|e| e.to_string(), |_| "closed".to_string());
                eprintln!("hoverboard-cli: the relay connection ended ({why})");
                self.closed = true;
                Vec::new()
            }
        }
    }
}
