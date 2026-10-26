//! The console's L3 guest (`specs/hoverboard-bridge.md`, "`watch` and `drive`"): the attach, the
//! `CYCLIC_STATE` bookkeeping `watch` prints, and the frames `drive` sends, checked against the
//! `net` and `linkctl` codecs.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::Command;
use std::time::{Duration, Instant};

use hoverboard_cli::guest::{
    attach, drive_pdu, inputs_pdu, Drive, Tally, DRIVE_PERIOD, INPUTS_PERIOD,
};
use hoverboard_cli::wire::{ByteLink, Framed, Wire};
use linkctl::{
    ChipTag, CyclicObs, CyclicState, DriveCmd, DriveKind, Inputs, OP_CYCLIC_STATE, OP_DRIVE_CMD,
    OP_INPUTS,
};
use net::pdu::{Opcode, Pdu, GUEST_FIRST, NO_ADDRESS};
use net::walk::{KIND_CONTROLLER, PROTO_VER};

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

fn state(mode: u8, peer_demand: i16, battery: u16) -> CyclicState {
    CyclicState {
        pitch: 0,
        roll: 0,
        wheel_speed: 0,
        battery,
        mode,
        fault: 0,
        flags: 0,
        peer_demand,
        wheel_pos: 0,
        obs: CyclicObs {
            phase_peak: 0,
            phase_mean: 0,
            duty_on: 0,
            boot_tag: 0,
            chip: ChipTag::Unknown,
            odometer: 0,
        },
    }
}

fn pdu(opcode: u8, src: u8, dst: u8, payload: &[u8]) -> Vec<u8> {
    let mut buf = [0u8; 64];
    let n = Pdu::new(opcode, src, dst, payload)
        .unwrap()
        .encode(&mut buf)
        .unwrap();
    buf[..n].to_vec()
}

fn cyclic(src: u8, s: &CyclicState) -> Vec<u8> {
    let mut b = [0u8; CyclicState::ENCODED_LEN];
    let n = s.encode(&mut b);
    pdu(OP_CYCLIC_STATE, src, NO_ADDRESS, &b[..n])
}

#[test]
fn the_tally_counts_each_source_with_its_rate_and_longest_gap() {
    let mut t = Tally::new();
    for at in [0, 200, 410, 600, 800] {
        assert!(t.feed(&cyclic(0x01, &state(3, 0, 310)), ms(at)));
    }
    for at in [100, 520] {
        assert!(t.feed(&cyclic(0x02, &state(2, -40, 0)), ms(at)));
    }
    // Not counted: another opcode, a CYCLIC_STATE of the wrong length, a runt.
    let mut d = [0u8; DriveCmd::LEN];
    DriveCmd {
        kind: DriveKind::Neutral,
        value: 0,
        steer: 0,
    }
    .encode(&mut d);
    assert!(!t.feed(&pdu(OP_DRIVE_CMD, 0x01, 0x02, &d), ms(900)));
    assert!(!t.feed(&pdu(OP_CYCLIC_STATE, 0x01, 0x00, &[0; 13]), ms(900)));
    assert!(!t.feed(&[OP_CYCLIC_STATE, 0x01], ms(900)));

    assert_eq!(t.frames(0x01), 5);
    assert_eq!(t.frames(0x02), 2);
    assert_eq!(t.frames(0x03), 0);
    assert_eq!(t.longest_gap(0x01), ms(210));
    assert_eq!(t.longest_gap(0x02), ms(420));
    assert!((t.rate(0x01, ms(1000)) - 5.0).abs() < 1e-9);
    assert!((t.rate(0x02, ms(2000)) - 1.0).abs() < 1e-9);
    assert_eq!(
        t.line(ms(1000)),
        "t=1s  0x01 5 frames 5.0/s gap 0.21s mode 3 peer_demand 0 battery 310 | \
         0x02 2 frames 2.0/s gap 0.42s mode 2 peer_demand -40 battery 0"
    );
    assert_eq!(
        t.summary(ms(1000)),
        [
            "0x01: 5 frames in 1.0s, 5.00/s, longest gap 0.21s",
            "0x02: 2 frames in 1.0s, 2.00/s, longest gap 0.42s",
        ]
    );
}

#[test]
fn a_tally_with_nothing_heard_says_so() {
    let t = Tally::new();
    assert_eq!(t.line(ms(3000)), "t=3s  no CYCLIC_STATE heard");
    assert!(t.summary(ms(3000)).is_empty());
    assert_eq!(t.rate(0x01, Duration::ZERO), 0.0);
}

#[test]
fn drive_frames_decode_with_the_linkctl_codecs() {
    let cmd = DriveCmd {
        kind: DriveKind::Throttle,
        value: -1200,
        steer: 0,
    };
    let f = drive_pdu(0x88, 0x02, &cmd);
    let p = Pdu::decode(&f).unwrap();
    assert_eq!((p.opcode, p.src, p.dst), (OP_DRIVE_CMD, 0x88, 0x02));
    assert_eq!(DriveCmd::decode(p.payload).unwrap(), cmd);
    assert_eq!(p.payload.len(), DriveCmd::LEN);

    let i = Inputs {
        buttons: Inputs::BUTTON_POWER,
        rider: 0,
    };
    let f = inputs_pdu(0x88, 0x01, &i);
    let p = Pdu::decode(&f).unwrap();
    assert_eq!((p.opcode, p.src, p.dst), (OP_INPUTS, 0x88, 0x01));
    let got = Inputs::decode(p.payload).unwrap();
    assert!(got.power_request() && !got.rider_present());
    assert_eq!(p.payload.len(), Inputs::LEN);
}

/// (opcode, dst, payload) of a frame.
fn parts(f: &[u8]) -> (u8, u8, Vec<u8>) {
    let p = Pdu::decode(f).unwrap();
    assert_eq!(p.src, 0x88);
    (p.opcode, p.dst, p.payload.to_vec())
}

#[test]
fn drive_sends_throttle_at_20_hz_and_power_at_2_hz_to_each_node() {
    let mut d = Drive::new(0x88, vec![0x01, 0x02], 500);
    let mut sent = Vec::new();
    let mut at = Duration::ZERO;
    while at < ms(1000) {
        sent.extend(d.due(at));
        at += ms(10);
    }
    let count = |op: u8, dst: u8| {
        sent.iter()
            .filter(|f| {
                let (o, d, _) = parts(f);
                o == op && d == dst
            })
            .count()
    };
    let per_s = |p: Duration| (1000 / p.as_millis()) as usize;
    for node in [0x01, 0x02] {
        assert_eq!(count(OP_DRIVE_CMD, node), per_s(DRIVE_PERIOD));
        assert_eq!(count(OP_INPUTS, node), per_s(INPUTS_PERIOD));
    }
    for f in &sent {
        let (op, _, payload) = parts(f);
        if op == OP_DRIVE_CMD {
            let c = DriveCmd::decode(&payload).unwrap();
            assert_eq!((c.kind, c.value, c.steer), (DriveKind::Throttle, 500, 0));
        } else {
            assert_eq!(op, OP_INPUTS);
            assert!(Inputs::decode(&payload).unwrap().power_request());
        }
    }
    // The first frames go out at once, power before throttle.
    let first: Vec<_> = Drive::new(0x88, vec![0x01], 500)
        .due(Duration::ZERO)
        .iter()
        .map(|f| parts(f).0)
        .collect();
    assert_eq!(first, [OP_INPUTS, OP_DRIVE_CMD]);
}

#[test]
fn a_late_drive_call_sends_once_and_resumes_the_cadence() {
    let mut d = Drive::new(0x88, vec![0x01], 500);
    assert_eq!(d.due(Duration::ZERO).len(), 2);
    // 300 ms late: one throttle frame, not six.
    let late = d.due(ms(350));
    assert_eq!(late.len(), 1);
    assert_eq!(parts(&late[0]).0, OP_DRIVE_CMD);
    assert_eq!(d.next_due(), ms(400));
    assert!(d.due(ms(350)).is_empty());
}

#[test]
fn the_stop_round_is_neutral_then_power_0_to_each_node() {
    let d = Drive::new(0x88, vec![0x01, 0x02], 500);
    let round: Vec<_> = d.stop().iter().map(|f| parts(f)).collect();
    assert_eq!(round.len(), 4);
    for (i, (op, dst, payload)) in round.iter().enumerate() {
        assert_eq!(*dst, [0x01, 0x02][i % 2]);
        if i < 2 {
            assert_eq!(*op, OP_DRIVE_CMD);
            assert_eq!(DriveCmd::decode(payload).unwrap().kind, DriveKind::Neutral);
        } else {
            assert_eq!(*op, OP_INPUTS);
            let p = Inputs::decode(payload).unwrap();
            assert!(!p.power_request() && !p.rider_present());
        }
    }
}

/// A board behind the bridge: it answers the `answer_on`th `NODE_HELLO` with a grant, after a
/// `CYCLIC_STATE` and a grant addressed to someone else, and records what was sent.
struct Board {
    sent: Vec<Vec<u8>>,
    inbox: VecDeque<Vec<u8>>,
    answer_on: usize,
}

impl Wire for Board {
    fn send(&mut self, f: &[u8]) {
        self.sent.push(f.to_vec());
        if self.sent.len() == self.answer_on {
            let grant = |dst: u8, addr: u8| {
                pdu(
                    Opcode::NodeHello.to_u8(),
                    0x01,
                    dst,
                    &[0x01, PROTO_VER, 0, 0, 0x10, addr],
                )
            };
            self.inbox.push_back(cyclic(0x01, &state(3, 0, 310)));
            self.inbox.push_back(grant(0x90, 0x91));
            self.inbox.push_back(grant(GUEST_FIRST, 0x88));
        }
    }
    fn recv(&mut self, _: Duration) -> Option<Vec<u8>> {
        self.inbox.pop_front()
    }
    fn window(&self) -> usize {
        espsvc::WINDOW_BLE
    }
}

#[test]
fn attach_sends_node_hello_to_the_one_peer_and_adopts_the_grant() {
    let mut b = Board {
        sent: Vec::new(),
        inbox: VecDeque::new(),
        answer_on: 2,
    };
    assert_eq!(attach(&mut b, 3, ms(1)), Some(0x88));
    assert_eq!(b.sent.len(), 2);
    for f in &b.sent {
        let p = Pdu::decode(f).unwrap();
        assert_eq!(p.known(), Some(Opcode::NodeHello));
        assert_eq!((p.src, p.dst), (GUEST_FIRST, NO_ADDRESS));
        assert_eq!(p.payload, [KIND_CONTROLLER]);
    }
}

#[test]
fn attach_gives_up_after_its_tries() {
    let mut b = Board {
        sent: Vec::new(),
        inbox: VecDeque::new(),
        answer_on: 0,
    };
    assert_eq!(attach(&mut b, 3, ms(1)), None);
    assert_eq!(b.sent.len(), 3);
}

/// The board's end of the `--tcp` link: the console's bytes, framed as the bridge relays them.
struct Sock(TcpStream);

impl ByteLink for Sock {
    fn write(&mut self, bytes: &[u8]) {
        let _ = self.0.write_all(bytes);
    }
    fn read(&mut self, timeout: Duration) -> Vec<u8> {
        let _ = self.0.set_read_timeout(Some(timeout.max(ms(1))));
        let mut buf = [0u8; 512];
        let n = self.0.read(&mut buf).unwrap_or(0);
        buf[..n].to_vec()
    }
    fn window(&self) -> usize {
        espsvc::WINDOW_STREAM
    }
}

/// Serve one console connection for `run`: grant `0x88` to the first `NODE_HELLO`, then emit the
/// master's `CYCLIC_STATE` (src `0x01`, 5 Hz) and its peer's forward (src `0x02`, 2.5 Hz), dst
/// `0x00`. Returns every frame the console sent.
fn serve_board(listener: TcpListener, run: Duration) -> Vec<Vec<u8>> {
    let (sock, _) = listener.accept().unwrap();
    let mut w = Framed::new(Sock(sock));
    let mut got = Vec::new();
    let mut t0 = None;
    let (mut next_own, mut next_fwd) = (Duration::ZERO, Duration::ZERO);
    let start = Instant::now();
    while start.elapsed() < run {
        if let Some(f) = w.recv(ms(5)) {
            let p = Pdu::decode(&f).unwrap();
            if p.known() == Some(Opcode::NodeHello) && t0.is_none() {
                let g = pdu(
                    Opcode::NodeHello.to_u8(),
                    0x01,
                    p.src,
                    &[0x01, PROTO_VER, 0, 0, 0x10, 0x88],
                );
                w.send(&g);
                t0 = Some(Instant::now());
            }
            got.push(f);
        }
        if let Some(t0) = t0 {
            let at = t0.elapsed();
            if at >= next_own {
                w.send(&cyclic(0x01, &state(3, 0, 310)));
                next_own += ms(200);
            }
            if at >= next_fwd {
                w.send(&cyclic(0x02, &state(3, 0, 0)));
                next_fwd += ms(400);
            }
        }
    }
    got
}

fn cli(args: &[&str]) -> (bool, String, String) {
    let o = Command::new(env!("CARGO_BIN_EXE_hoverboard-cli"))
        .args(args)
        .output()
        .unwrap();
    (
        o.status.success(),
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    )
}

#[test]
fn the_binary_drives_over_tcp_and_prints_the_watch_line() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let board = std::thread::spawn(move || serve_board(listener, Duration::from_secs(5)));
    let (ok, out, err) = cli(&[&tcp, "drive", "300", "2", "0x01"]);
    assert!(ok, "{err}");
    assert!(err.contains("attached as guest 0x88"), "{err}");
    let lines: Vec<&str> = out.lines().collect();
    assert!(lines[0].starts_with("t=1s  0x01 "), "{out}");
    assert!(lines[0].contains(" | 0x02 "), "{out}");
    assert!(lines[1].starts_with("t=2s  0x01 "), "{out}");
    assert_eq!(lines[2], "neutral sent, power 0 sent", "{out}");
    assert!(lines[3].starts_with("0x01: "), "{out}");
    assert!(lines[4].starts_with("0x02: "), "{out}");

    let sent = board.join().unwrap();
    let after_hello: Vec<(u8, u8, Vec<u8>)> = sent[1..]
        .iter()
        .map(|f| {
            let p = Pdu::decode(f).unwrap();
            assert_eq!((p.src, p.dst), (0x88, 0x01));
            (p.opcode, p.dst, p.payload.to_vec())
        })
        .collect();
    let n = |op: u8, pick: &dyn Fn(&[u8]) -> bool| {
        after_hello
            .iter()
            .filter(|(o, _, pl)| *o == op && pick(pl))
            .count()
    };
    let throttle = n(OP_DRIVE_CMD, &|pl| {
        DriveCmd::decode(pl).unwrap().kind == DriveKind::Throttle
    });
    let power = n(OP_INPUTS, &|pl| Inputs::decode(pl).unwrap().power_request());
    // 2 s at 20 Hz and 2 Hz, give or take the scheduling at the edges.
    assert!((38..=41).contains(&throttle), "{throttle} throttle frames");
    assert!((4..=5).contains(&power), "{power} power frames");
    // The last six: neutral then power 0, three rounds.
    let tail = &after_hello[after_hello.len() - 6..];
    for pair in tail.chunks(2) {
        assert_eq!(pair[0].0, OP_DRIVE_CMD);
        assert_eq!(
            DriveCmd::decode(&pair[0].2).unwrap().kind,
            DriveKind::Neutral
        );
        assert_eq!(pair[1].0, OP_INPUTS);
        assert!(!Inputs::decode(&pair[1].2).unwrap().power_request());
    }
}

#[test]
fn the_binary_watches_over_tcp_and_sends_nothing_after_the_attach() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let board = std::thread::spawn(move || serve_board(listener, Duration::from_secs(4)));
    let (ok, out, err) = cli(&[&tcp, "watch", "2", "0x01", "0x02"]);
    assert!(ok, "{err}");
    let lines: Vec<&str> = out.lines().collect();
    assert_eq!(lines.len(), 4, "{out}");
    assert!(lines[1].starts_with("t=2s  0x01 1"), "{out}");
    assert!(
        lines[1].contains("mode 3 peer_demand 0 battery 310 | 0x02 "),
        "{out}"
    );
    let sent = board.join().unwrap();
    assert_eq!(sent.len(), 1);
    assert_eq!(
        Pdu::decode(&sent[0]).unwrap().known(),
        Some(Opcode::NodeHello)
    );
}
