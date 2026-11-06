//! `--serial` keeps its deadlines (`specs/cli-serial-deadlines.md`): a USB console that enumerates
//! but whose device does not read, as a C3 that panics at boot leaves its USB-Serial-JTAG. The fake
//! port below behaves as the host's tty does for such a device: writes go into the driver's queue
//! until it is full, nothing ever leaves it, a drain of it (`tcdrain`, which `flush` is) waits for
//! ever, and reads time out.

use std::io;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use hoverboard_cli::transport::Serial;
use hoverboard_cli::wire::Framed;
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};

/// The host driver's output queue.
const QUEUE: usize = 4096;
/// The port name the fake reports.
const NAME: &str = "/dev/ttyACM9";

#[derive(Default)]
struct Queue {
    queued: usize,
    timeout: Duration,
}

/// A port whose device neither reads nor writes.
#[derive(Clone, Default)]
struct Deaf(Arc<Mutex<Queue>>);

impl Deaf {
    fn timeout(&self) -> Duration {
        self.0.lock().unwrap().timeout
    }
}

impl io::Read for Deaf {
    fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
        std::thread::sleep(self.timeout());
        Err(io::ErrorKind::TimedOut.into())
    }
}

impl io::Write for Deaf {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut q = self.0.lock().unwrap();
        if q.queued + buf.len() > QUEUE {
            let t = q.timeout;
            drop(q);
            std::thread::sleep(t);
            return Err(io::ErrorKind::TimedOut.into());
        }
        q.queued += buf.len();
        Ok(buf.len())
    }

    /// `tcdrain`: until the queue is empty, which it never is once written to.
    fn flush(&mut self) -> io::Result<()> {
        while self.0.lock().unwrap().queued > 0 {
            std::thread::sleep(Duration::from_millis(10));
        }
        Ok(())
    }
}

impl SerialPort for Deaf {
    fn name(&self) -> Option<String> {
        Some(NAME.into())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(115_200)
    }
    fn data_bits(&self) -> serialport::Result<DataBits> {
        Ok(DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<FlowControl> {
        Ok(FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<Parity> {
        Ok(Parity::None)
    }
    fn stop_bits(&self) -> serialport::Result<StopBits> {
        Ok(StopBits::One)
    }
    fn timeout(&self) -> Duration {
        Deaf::timeout(self)
    }
    fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _: DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _: FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _: Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _: StopBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
        self.0.lock().unwrap().timeout = timeout;
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(self.0.lock().unwrap().queued as u32)
    }
    fn clear(&self, which: ClearBuffer) -> serialport::Result<()> {
        if matches!(which, ClearBuffer::Output | ClearBuffer::All) {
            self.0.lock().unwrap().queued = 0;
        }
        Ok(())
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Ok(Box::new(self.clone()))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}

/// A framed wire over a [`Deaf`] port.
fn deaf() -> Framed<Serial> {
    Framed::new(Serial::over(Box::new(Deaf::default()), NAME))
}

/// Run `f` on its own thread and fail if it has not returned within `limit`: a call that blocks
/// is reported rather than hanging the test run. Returns `f`'s result and how long it took.
fn within<T: Send + 'static>(
    what: &str,
    limit: Duration,
    f: impl FnOnce() -> T + Send + 'static,
) -> (T, Duration) {
    let (tx, rx) = mpsc::channel();
    let t0 = Instant::now();
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    match rx.recv_timeout(limit) {
        Ok(t) => (t, t0.elapsed()),
        Err(_) => panic!("{what} still blocked after {limit:?}"),
    }
}

/// What the link said when the command ended.
fn fault(w: &mut Framed<Serial>) -> Option<String> {
    w.inner().fault().map(str::to_string)
}

/// The link error every command here ends with.
const DOES_NOT_READ: &str = "the bridge on /dev/ttyACM9 does not read: is it booting?";

#[test]
fn a_port_that_does_not_read_ends_every_command_within_its_deadline() {
    use hoverboard_cli::guest::{self, HELLO_TRIES, HELLO_WAIT};
    use hoverboard_cli::nodes::{self, REPLY_WAIT, TRIES};
    use hoverboard_cli::transport::WRITE_WAIT;
    use net::pdu::GUEST_FIRST;

    // `machine nodes`: the walk's first contact, three tries of `REPLY_WAIT`.
    let limit = (WRITE_WAIT + REPLY_WAIT) * TRIES + Duration::from_secs(1);
    let ((r, f), took) = within("the walk", limit, || {
        let mut w = deaf();
        (nodes::start(&mut w).err(), fault(&mut w))
    });
    assert!(r
        .unwrap()
        .starts_with("the walk got no answer to NodeHello"));
    assert_eq!(f.as_deref(), Some(DOES_NOT_READ));
    assert!(took >= REPLY_WAIT * TRIES, "{took:?}");

    // `bridge info 0x01` and `bridge set 0x01 ...`: the attach, `HELLO_TRIES` of `HELLO_WAIT`.
    let limit = (WRITE_WAIT + HELLO_WAIT) * HELLO_TRIES + Duration::from_secs(1);
    let ((r, f), _) = within("the attach", limit, || {
        let mut w = deaf();
        (
            guest::attach(&mut w, HELLO_TRIES, HELLO_WAIT).is_none(),
            fault(&mut w),
        )
    });
    assert!(r);
    assert_eq!(f.as_deref(), Some(DOES_NOT_READ));

    // A request to an address that does not answer, the per-request wait.
    let limit = (WRITE_WAIT + REPLY_WAIT) * TRIES + Duration::from_secs(1);
    let ((r, f), _) = within("a request to 0x01", limit, || {
        let mut w = deaf();
        (
            nodes::identify(&mut w, GUEST_FIRST, 0x01).err(),
            fault(&mut w),
        )
    });
    assert_eq!(r.unwrap(), "no Identity from 0x01");
    assert_eq!(f.as_deref(), Some(DOES_NOT_READ));
}
