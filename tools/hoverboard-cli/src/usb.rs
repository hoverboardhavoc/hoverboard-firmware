//! `usb scan` (`specs/cli-discovery.md` section 1): every port with the C3's USB ids, its MAC from
//! the host's port list, and the name of the bridge that answers there. The name comes from one
//! attach and one read of `device.name` at the attached node: no walk, no address assigned.

use std::time::Duration;

use net::pdu::NO_ADDRESS;

use crate::guest;
use crate::install::{c3_ports, PortInfo};
use crate::nodes;
use crate::wire::Wire;

/// What `usb scan` prints for a port where nothing answers the attach.
pub const NO_BRIDGE: &str = "(no bridge)";

/// The node on `wire`, attached once within `wait`: the guest address it granted and its
/// `device.name` (`the bridge` when the value is not a string); `None` when the attach or the read
/// gets no answer.
pub fn attached_name<W: Wire>(wire: &mut W, wait: Duration) -> Option<(u8, String)> {
    let src = guest::attach(wire, 1, wait)?.guest;
    let name = match nodes::read(wire, src, NO_ADDRESS, store::DEVICE_NAME.key())
        .ok()?
        .value()
    {
        Some(store::Value::Str(s)) => String::from_utf8_lossy(s).into_owned(),
        _ => "the bridge".to_string(),
    };
    Some((src, name))
}

/// `usb scan`'s lines: one per C3 port in `ports` ([`c3_ports`]), sorted by port name, each the
/// port, its MAC (the USB serial number, in lower case; `-` without one) and what `name_on` gives
/// for the port, [`NO_BRIDGE`] when it gives nothing.
pub fn scan_lines(
    ports: &[PortInfo],
    mut name_on: impl FnMut(&str) -> Option<String>,
) -> Vec<String> {
    let mut c3 = c3_ports(ports);
    c3.sort_by(|a, b| a.name.cmp(&b.name));
    c3.iter()
        .map(|p| {
            let mac = p
                .serial
                .as_deref()
                .map_or_else(|| "-".to_string(), str::to_lowercase);
            let name = name_on(&p.name).unwrap_or_else(|| NO_BRIDGE.to_string());
            format!("{}  {mac}  {name}", p.name)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::install::C3_USB;
    use net::pdu::{Opcode, Pdu};
    use net::walk::PROTO_VER;
    use std::collections::VecDeque;

    fn encode(op: Opcode, src: u8, dst: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = [0u8; net::walk::MAX_PDU];
        let n = Pdu::new(op.to_u8(), src, dst, payload)
            .unwrap()
            .encode(&mut buf)
            .unwrap();
        buf[..n].to_vec()
    }

    /// A bridge at `0x03` named `name`: `NODE_HELLO` grants guest `0x88`, a `CONFIG_READ` of
    /// `device.name` answers the name. Counts what it was sent.
    struct Bridge {
        name: &'static str,
        replies: VecDeque<Vec<u8>>,
        sent: Vec<Opcode>,
    }

    impl Wire for Bridge {
        fn send(&mut self, pdu: &[u8]) {
            let p = Pdu::decode(pdu).unwrap();
            let op = p.known().unwrap();
            self.sent.push(op);
            let reply = match op {
                Opcode::NodeHello => {
                    encode(Opcode::NodeHello, 0x03, p.src, &[0x03, PROTO_VER, 0x88])
                }
                Opcode::ConfigRead => {
                    let key = store::DEVICE_NAME.key();
                    assert_eq!(p.payload, [key.field_id, key.index]);
                    let mut v = vec![key.field_id, key.index, 0, store::Type::Str.tag()];
                    v.extend_from_slice(self.name.as_bytes());
                    encode(Opcode::ConfigResp, 0x03, p.src, &v)
                }
                _ => return,
            };
            self.replies.push_back(reply);
        }
        fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
            self.replies.pop_front()
        }
        fn window(&self) -> usize {
            1
        }
    }

    /// Nothing on the port answers.
    struct Silent;

    impl Wire for Silent {
        fn send(&mut self, _pdu: &[u8]) {}
        fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
            None
        }
        fn window(&self) -> usize {
            1
        }
    }

    fn port(name: &str, usb: Option<(u16, u16)>, serial: Option<&str>) -> PortInfo {
        PortInfo {
            name: name.into(),
            usb,
            serial: serial.map(str::to_string),
        }
    }

    #[test]
    fn the_name_is_one_attach_and_one_read() {
        let mut b = Bridge {
            name: "hb-right",
            replies: VecDeque::new(),
            sent: Vec::new(),
        };
        assert_eq!(
            attached_name(&mut b, Duration::from_millis(10)),
            Some((0x88, "hb-right".to_string()))
        );
        assert_eq!(b.sent, [Opcode::NodeHello, Opcode::ConfigRead]);
        assert_eq!(attached_name(&mut Silent, Duration::from_millis(10)), None);
    }

    #[test]
    fn only_c3_ports_sorted_with_the_mac_and_the_name_or_no_bridge() {
        let ports = [
            port("/dev/ttyACM3", Some(C3_USB), Some("44:BD:8D:25:A3:7C")),
            port("/dev/ttyUSB0", Some((0x0403, 0x6001)), Some("A10K1ZXY")),
            port("/dev/ttyACM2", Some(C3_USB), Some("14:63:93:77:36:F0")),
            port("/dev/ttyS0", None, None),
            port("/dev/ttyACM0", Some(C3_USB), Some("28:37:2F:54:BF:7C")),
            port("/dev/ttyACM1", Some(C3_USB), None),
        ];
        let mut asked = Vec::new();
        let lines = scan_lines(&ports, |p| {
            asked.push(p.to_string());
            let name = match p {
                "/dev/ttyACM0" => "hb-bridge-bf7c",
                "/dev/ttyACM3" => "hb-right",
                _ => return attached_name(&mut Silent, Duration::from_millis(10)).map(|a| a.1),
            };
            let mut b = Bridge {
                name,
                replies: VecDeque::new(),
                sent: Vec::new(),
            };
            attached_name(&mut b, Duration::from_millis(10)).map(|a| a.1)
        });
        assert_eq!(
            lines,
            [
                "/dev/ttyACM0  28:37:2f:54:bf:7c  hb-bridge-bf7c",
                "/dev/ttyACM1  -  (no bridge)",
                "/dev/ttyACM2  14:63:93:77:36:f0  (no bridge)",
                "/dev/ttyACM3  44:bd:8d:25:a3:7c  hb-right",
            ]
        );
        // Only the C3 ports were opened.
        assert_eq!(
            asked,
            [
                "/dev/ttyACM0",
                "/dev/ttyACM1",
                "/dev/ttyACM2",
                "/dev/ttyACM3"
            ]
        );
        // macOS's tty twin is the same device, listed once.
        let mac = [
            port(
                "/dev/tty.usbmodem101",
                Some(C3_USB),
                Some("28:37:2F:54:BF:7C"),
            ),
            port(
                "/dev/cu.usbmodem101",
                Some(C3_USB),
                Some("28:37:2F:54:BF:7C"),
            ),
        ];
        assert_eq!(
            scan_lines(&mac, |_| None),
            ["/dev/cu.usbmodem101  28:37:2f:54:bf:7c  (no bridge)"]
        );
        assert!(scan_lines(&[], |_| None).is_empty());
    }
}
