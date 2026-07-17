//! Deliver an INPUTS L7 payload to an addressed board over the SWD mailbox, so a bench session can
//! drive the firmware's mode machine from the host.
//!
//! Usage: `swd-mailbox-inputs <openocd-host:port> [--base HEX] [--dst ADDR] [--buttons BYTE]
//!         [--throttle N] [--rider BYTE]`
//!
//! The tool attaches the mailbox, runs the L3 walk (the same bring-up the config writer uses, so it
//! learns the master's assigned address and confirms the firmware is live), then sends ONE INPUTS PDU
//! (`linkctl::OP_INPUTS` = 0x12) carrying the `linkctl::Inputs` payload (throttle i16, buttons u8,
//! rider u8, little-endian, encoded by `linkctl` -- the canonical owner of the bytes). The PDU is
//! delivered to `--dst` (default: the walked master address), `src` = the controller's guest address.
//!
//! INPUTS is best-effort / latest-wins (`specs/link-control.md`): there is no reply, so the tool
//! encodes + sends once and confirms the send. The firmware holds the last-received level, so the
//! delivered `buttons` bit0 (power_request) persists after the tool exits.
//!
//! Arming the armed CONFIG_WRITE gate (`--buttons 1` asserts power_request, walking the mode machine
//! OFF->INIT->READY->RUN so `any_moe_allowed` = armed); `--buttons 0` clears it (disarm):
//!
//! ```text
//! swd-mailbox-inputs 127.0.0.1:6666 --dst 0x01 --buttons 1   # arm (power_request = 1)
//! swd-mailbox-inputs 127.0.0.1:6666 --dst 0x01 --buttons 0   # disarm (power_request = 0)
//! ```

use std::process::ExitCode;
use std::time::Duration;

use linkctl::{Inputs, OP_INPUTS};
use net::Pdu;
use swd_bridge::openocd::OpenOcdTcl;
use swd_bridge::walk::WalkDriver;
use swd_bridge::{HostMailbox, MAILBOX_BASE};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("FAIL: {e}");
            ExitCode::FAILURE
        }
    }
}

const USAGE: &str = "usage: swd-mailbox-inputs <host:port> [--base HEX] [--dst ADDR] \
     [--buttons BYTE] [--throttle N] [--rider BYTE]";

/// Encode an INPUTS L3 PDU (opcode `0x12`) carrying `inputs`, from `src` to `dst`. The payload bytes
/// come from `linkctl::Inputs::encode` (the canonical owner); this only wraps them in the L3 header.
fn encode_inputs_pdu(src: u8, dst: u8, inputs: &Inputs) -> Vec<u8> {
    let mut payload = [0u8; Inputs::LEN];
    inputs.encode(&mut payload);
    // OP_INPUTS (0x12) is a fixed, valid opcode (not 0x00/0xFF), so both calls are infallible; the
    // buffer is sized exactly to the L3 header + the committed INPUTS prefix.
    let pdu = Pdu::new(OP_INPUTS, src, dst, &payload).expect("OP_INPUTS is a valid opcode");
    let mut buf = [0u8; net::pdu::HEADER_LEN + Inputs::LEN];
    let n = pdu
        .encode(&mut buf)
        .expect("buf fits L3 header + INPUTS payload");
    buf[..n].to_vec()
}

/// Parse a `0x`-hex or decimal `u8` (addresses, byte fields).
fn parse_u8(s: &str) -> Result<u8, String> {
    let r = s
        .strip_prefix("0x")
        .map(|h| u8::from_str_radix(h, 16))
        .unwrap_or_else(|| s.parse::<u8>());
    r.map_err(|_| format!("bad u8 value {s:?}"))
}

/// Parse a decimal (or `0x`-hex) `i16` for the throttle word.
fn parse_i16(s: &str) -> Result<i16, String> {
    if let Some(h) = s.strip_prefix("0x") {
        // A hex throttle is a raw bit pattern (e.g. 0xFFFF = -1).
        return u16::from_str_radix(h, 16)
            .map(|u| u as i16)
            .map_err(|_| format!("bad i16 value {s:?}"));
    }
    s.parse::<i16>().map_err(|_| format!("bad i16 value {s:?}"))
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let endpoint = args.next().ok_or(USAGE)?;

    let mut base = MAILBOX_BASE;
    let mut dst: Option<u8> = None;
    let mut buttons: u8 = 0;
    let mut throttle: i16 = 0;
    let mut rider: u8 = 0;

    let mut it = args;
    while let Some(a) = it.next() {
        let mut val = || it.next().ok_or_else(|| format!("{a} needs a value"));
        match a.as_str() {
            "--base" => {
                let b = val()?;
                base = u32::from_str_radix(b.trim_start_matches("0x"), 16)
                    .map_err(|_| format!("bad --base {b:?}"))?;
            }
            "--dst" => dst = Some(parse_u8(&val()?)?),
            "--buttons" => buttons = parse_u8(&val()?)?,
            "--throttle" => throttle = parse_i16(&val()?)?,
            "--rider" => rider = parse_u8(&val()?)?,
            other => return Err(format!("unknown argument {other:?}\n{USAGE}")),
        }
    }

    // Attach the mailbox + walk to bring the L3 link up and learn the master's address.
    let mem = OpenOcdTcl::connect(&endpoint).map_err(|e| e.to_string())?;
    let mut host = HostMailbox::new(mem, base);
    host.validate().map_err(|e| e.to_string())?;
    host.attach().map_err(|e| e.to_string())?;
    host.wait_flush_ack(200)
        .map_err(|_| "firmware never wrote epoch_ack (no poll-site running?)".to_string())?;

    let mut walk = WalkDriver::new(host);
    walk.run_walk(Duration::from_secs(30))
        .map_err(|e| e.to_string())?;

    // Default dst to the walked master; src is the controller's guest address.
    let dst = match dst {
        Some(d) => d,
        None => walk
            .master_addr()
            .ok_or("walk assigned no address; pass --dst explicitly")?,
    };
    let src = walk.guest_addr();

    let inputs = Inputs {
        throttle,
        buttons,
        rider,
    };
    let pdu = encode_inputs_pdu(src, dst, &inputs);
    walk.send_pdu(&pdu).map_err(|e| e.to_string())?;

    println!(
        "sent INPUTS 0x{src:02x}->0x{dst:02x}: throttle={throttle} buttons={buttons:#04x} \
         (power_request={}) rider={rider:#04x} (rider_present={})",
        inputs.power_request(),
        inputs.rider_present(),
    );
    println!("  PDU bytes: {pdu:02x?}");
    println!("PASS: INPUTS delivered (best-effort, no reply); the firmware holds the last level");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_power_request_pdu_bytes() {
        // buttons bit0 = power_request; throttle/rider zero. L3 header [op, src, dst] then the
        // little-endian INPUTS payload [throttle_lo, throttle_hi, buttons, rider].
        let inp = Inputs {
            throttle: 0,
            buttons: Inputs::BUTTON_POWER,
            rider: 0,
        };
        let pdu = encode_inputs_pdu(0x80, 0x01, &inp);
        assert_eq!(pdu, vec![0x12, 0x80, 0x01, 0x00, 0x00, 0x01, 0x00]);
    }

    #[test]
    fn encodes_disarm_pdu_bytes() {
        // buttons = 0 clears power_request.
        let inp = Inputs {
            throttle: 0,
            buttons: 0,
            rider: 0,
        };
        let pdu = encode_inputs_pdu(0x80, 0x01, &inp);
        assert_eq!(pdu, vec![0x12, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn encodes_throttle_and_rider_little_endian() {
        // throttle -1 = 0xFFFF LE; rider bit0 set; buttons power+bit1.
        let inp = Inputs {
            throttle: -1,
            buttons: Inputs::BUTTON_POWER | 0x02,
            rider: Inputs::RIDER_PRESENT,
        };
        let pdu = encode_inputs_pdu(0x81, 0x02, &inp);
        assert_eq!(pdu, vec![0x12, 0x81, 0x02, 0xff, 0xff, 0x03, 0x01]);
    }

    #[test]
    fn parse_u8_hex_and_dec() {
        assert_eq!(parse_u8("0x01"), Ok(1));
        assert_eq!(parse_u8("255"), Ok(255));
        assert!(parse_u8("0x1ff").is_err());
    }

    #[test]
    fn parse_i16_signed_and_hex() {
        assert_eq!(parse_i16("-1"), Ok(-1));
        assert_eq!(parse_i16("0xffff"), Ok(-1));
        assert_eq!(parse_i16("300"), Ok(300));
    }
}
