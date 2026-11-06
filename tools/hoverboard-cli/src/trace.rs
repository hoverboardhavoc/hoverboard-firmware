//! `board trace` (`specs/flux-observer.md`, "Two instruments read it"; `specs/link-control.md`,
//! "`FOC_TRACE`" and "`TRACE_SET`"): subscribe a board to its `FOC_TRACE` stream, renew the
//! subscription at 2 Hz, print a line a frame, and write the CSV, the bench's record of the FOC
//! loop and the flux observer's shadow. This is the only CSV writer for the stream: the app draws
//! it and does not export.
//!
//! The schedule and the formats are here, where the tests reach them; `main.rs` runs the loop on
//! a link.

use std::time::Duration;

use linkctl::{FocTrace, TraceSet, OP_FOC_TRACE, OP_TRACE_SET};
use net::pdu::Pdu;

/// The keepalive period: the subscription expires 1.5 s after the last `TRACE_SET`
/// (`linkctl::TRACE_TIMEOUT_TICKS`), and the console renews at 2 Hz, the `INPUTS` cadence.
pub const RENEW_PERIOD: Duration = Duration::from_millis(500);

/// The divisor the console asks for: every 5th window, 50 Hz, the board's fastest
/// (`TraceSet::DIVISOR_MIN`).
pub const CONSOLE_DIVISOR: u8 = TraceSet::DIVISOR_MIN;

/// A `TRACE_SET` PDU from `src` to `dst`.
pub fn trace_set_pdu(src: u8, dst: u8, set: &TraceSet) -> Vec<u8> {
    let mut b = [0u8; TraceSet::LEN];
    let n = set.encode(&mut b);
    let p = Pdu::new(OP_TRACE_SET, src, dst, &b[..n]).expect("a control opcode is a valid opcode");
    let mut buf = [0u8; 16];
    let n = p.encode(&mut buf).expect("two bytes fit");
    buf[..n].to_vec()
}

/// The subscription's schedule: a `TRACE_SET` to `board` at once and every [`RENEW_PERIOD`], the
/// first with the reset bit when asked, and the frames from `board` picked out of what arrives.
pub struct Trace {
    src: u8,
    board: u8,
    motor: u8,
    reset: bool,
    next: Duration,
}

impl Trace {
    /// A schedule from guest `src` to `board` for motor `motor`; `reset` clears the board's
    /// tracking table with the first `TRACE_SET`.
    pub fn new(src: u8, board: u8, motor: u8, reset: bool) -> Self {
        Trace {
            src,
            board,
            motor,
            reset,
            next: Duration::ZERO,
        }
    }

    /// When the next `TRACE_SET` falls due.
    pub fn next_due(&self) -> Duration {
        self.next
    }

    /// The `TRACE_SET` due at `at`, if one is; a late call sends one and resumes the cadence
    /// from `at`.
    pub fn due(&mut self, at: Duration) -> Option<Vec<u8>> {
        if at < self.next {
            return None;
        }
        let reset = if std::mem::take(&mut self.reset) {
            TraceSet::RESET
        } else {
            0
        };
        self.next = if self.next + RENEW_PERIOD > at {
            self.next + RENEW_PERIOD
        } else {
            at + RENEW_PERIOD
        };
        Some(trace_set_pdu(
            self.src,
            self.board,
            &TraceSet {
                divisor: CONSOLE_DIVISOR | reset,
                motor: self.motor,
            },
        ))
    }

    /// The `TRACE_SET` that ends the subscription at once (divisor 0), sent
    /// [`crate::guest::STOP_REPEATS`] times at exit.
    pub fn stop(&self) -> Vec<u8> {
        trace_set_pdu(
            self.src,
            self.board,
            &TraceSet {
                divisor: 0,
                motor: self.motor,
            },
        )
    }

    /// The `FOC_TRACE` frame in `pkt`, if it is one from the traced board.
    pub fn frame(&self, pkt: &[u8]) -> Option<FocTrace> {
        let p = Pdu::decode(pkt).ok()?;
        if p.opcode != OP_FOC_TRACE || p.src != self.board {
            return None;
        }
        FocTrace::decode(p.payload).ok()
    }
}

/// Degrees in an angle unit (65536 a revolution).
const DEG: f64 = 360.0 / 65_536.0;

/// The CSV's columns: the time, then every field of the frame in its order, the flags split into
/// their four bits, the error in degrees beside its units, and the sixteen bins.
pub fn csv_header() -> String {
    let mut c: Vec<String> = [
        "t",
        "window",
        "motor",
        "method",
        "id",
        "iq",
        "iq_ref",
        "vq",
        "hall_angle",
        "obs_angle",
        "err_mean",
        "err_spread",
        "err_mean_deg",
        "err_spread_deg",
        "flux_uvs",
        "speed",
        "held_a",
        "held_b",
        "coasted",
        "cause",
        "hall_fault",
        "tracked",
        "observer_source",
        "in_window",
        "lock_bin",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    c.extend((0..16).map(|k| format!("track{k}")));
    c.join(",")
}

/// One frame as a CSV row, at `t` since the run started.
pub fn csv_row(t: Duration, f: &FocTrace) -> String {
    let bit = |b: u8| u8::from(f.flags & b != 0).to_string();
    let mut c = vec![
        format!("{:.3}", t.as_secs_f64()),
        f.window.to_string(),
        f.motor.to_string(),
        f.method.to_string(),
        f.id.to_string(),
        f.iq.to_string(),
        f.iq_ref.to_string(),
        f.vq.to_string(),
        f.hall_angle.to_string(),
        f.obs_angle.to_string(),
        f.err_mean.to_string(),
        f.err_spread.to_string(),
        format!("{:.2}", f.err_mean as f64 * DEG),
        format!("{:.2}", f.err_spread as f64 * DEG),
        f.flux_mag.to_string(),
        f.speed.to_string(),
        f.held_a.to_string(),
        f.held_b.to_string(),
        f.coasted.to_string(),
        format!("{:#04x}", f.cause),
        bit(FocTrace::FLAG_HALL_FAULT),
        bit(FocTrace::FLAG_TRACKED),
        bit(FocTrace::FLAG_OBSERVER_SOURCE),
        bit(FocTrace::FLAG_IN_WINDOW),
        f.lock_bin.to_string(),
    ];
    c.extend(f.track.iter().map(|b| b.to_string()));
    c.join(",")
}

/// One frame as a terminal line: the window, the loop's means, the observer against the halls,
/// the flux, the verdict and the lock bin (`-` for none).
pub fn line(t: Duration, f: &FocTrace) -> String {
    let lock = if f.lock_bin == 255 {
        "-".to_string()
    } else {
        f.lock_bin.to_string()
    };
    format!(
        "{:7.3} w{:5} m{} me{} id {:6} iq {:6} ref {:6} vq {:6} | hall {:5} obs {:5} err {:6.1} \
         spread {:5.1} deg | flux {:5} uV.s | speed {:4} | {}{} lock {}",
        t.as_secs_f64(),
        f.window,
        f.motor,
        f.method,
        f.id,
        f.iq,
        f.iq_ref,
        f.vq,
        f.hall_angle,
        f.obs_angle,
        f.err_mean as f64 * DEG,
        f.err_spread as f64 * DEG,
        f.flux_mag,
        f.speed,
        if f.flags & FocTrace::FLAG_TRACKED != 0 {
            "tracked"
        } else {
            "-------"
        },
        if f.flags & FocTrace::FLAG_HALL_FAULT != 0 {
            " HALL FAULT"
        } else {
            ""
        },
        lock,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame() -> FocTrace {
        FocTrace {
            window: 0x1234,
            id: -2,
            iq: 300,
            iq_ref: 258,
            vq: -31_999,
            hall_angle: 0xABCD,
            obs_angle: 0xA0B0,
            err_mean: -273,
            err_spread: 182,
            flux_mag: 21_000,
            speed: -7,
            held_a: 18,
            held_b: 27,
            coasted: 5,
            cause: 0x2A,
            flags: FocTrace::FLAG_TRACKED | FocTrace::FLAG_IN_WINDOW,
            method: 2,
            lock_bin: 4,
            motor: 1,
            track: [
                255, 255, 255, 255, 250, 249, 0, 125, 1, 2, 3, 4, 5, 6, 7, 255,
            ],
        }
    }

    /// The pinned frame's bytes (shared/linkctl's `foc_trace_encodes_to_the_pinned_bytes_and_back`)
    /// as a PDU from the board, through the schedule's filter, to the CSV header and row.
    #[test]
    fn the_csv_header_and_a_row_from_a_pinned_frame() {
        assert_eq!(
            csv_header(),
            "t,window,motor,method,id,iq,iq_ref,vq,hall_angle,obs_angle,err_mean,err_spread,\
             err_mean_deg,err_spread_deg,flux_uvs,speed,held_a,held_b,coasted,cause,hall_fault,\
             tracked,observer_source,in_window,lock_bin,track0,track1,track2,track3,track4,track5,\
             track6,track7,track8,track9,track10,track11,track12,track13,track14,track15"
        );
        let mut payload = [0u8; FocTrace::LEN];
        frame().encode(&mut payload);
        let p = Pdu::new(OP_FOC_TRACE, 0x02, 0x00, &payload).unwrap();
        let mut buf = [0u8; 64];
        let n = p.encode(&mut buf).unwrap();
        let tr = Trace::new(0x80, 0x02, 1, false);
        let f = tr.frame(&buf[..n]).expect("a frame from the traced board");
        assert_eq!(f, frame());
        assert_eq!(
            csv_row(Duration::from_millis(1500), &f),
            "1.500,4660,1,2,-2,300,258,-31999,43981,41136,-273,182,-1.50,1.00,21000,-7,18,27,5,\
             0x2a,0,1,0,1,4,255,255,255,255,250,249,0,125,1,2,3,4,5,6,7,255"
        );
        assert_eq!(
            csv_header().split(',').count(),
            csv_row(Duration::ZERO, &f).split(',').count()
        );
        // Another board's frame, or another opcode, is not the trace's.
        let other = Pdu::new(OP_FOC_TRACE, 0x03, 0x00, &payload).unwrap();
        let n = other.encode(&mut buf).unwrap();
        assert_eq!(tr.frame(&buf[..n]), None);
        assert!(line(Duration::ZERO, &f).contains("tracked lock 4"));
    }

    /// The keepalive: at once, then every 500 ms, the reset bit on the first alone, and the stop
    /// frame's divisor 0.
    #[test]
    fn the_subscription_renews_at_two_hertz_and_stops_with_divisor_zero() {
        let mut tr = Trace::new(0x80, 0x02, 1, true);
        let set = |pkt: Vec<u8>| {
            let p = Pdu::decode(&pkt).unwrap();
            assert_eq!((p.opcode, p.src, p.dst), (OP_TRACE_SET, 0x80, 0x02));
            TraceSet::decode(p.payload).unwrap()
        };
        let first = set(tr.due(Duration::ZERO).unwrap());
        assert_eq!(
            first,
            TraceSet {
                divisor: TraceSet::RESET | 5,
                motor: 1
            }
        );
        assert_eq!(tr.due(Duration::from_millis(499)), None);
        let second = set(tr.due(Duration::from_millis(500)).unwrap());
        assert_eq!(
            second,
            TraceSet {
                divisor: 5,
                motor: 1
            }
        );
        assert_eq!(tr.next_due(), Duration::from_millis(1000));
        // Late: one frame, and the cadence resumes from then.
        assert!(tr.due(Duration::from_millis(2300)).is_some());
        assert_eq!(tr.next_due(), Duration::from_millis(2800));
        assert_eq!(
            set(tr.stop()),
            TraceSet {
                divisor: 0,
                motor: 1
            }
        );
    }
}
