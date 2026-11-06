//! The remembered bridge (`specs/handheld-remote.md` section 3.2, "The stored pick"): one record in
//! the first 4 KB sector of the `nvs` data partition, in the handheld's own format (not ESP-IDF
//! NVS). The binary reads and writes the sector with `esp-storage`; the record is encoded,
//! decoded and checked here.
//!
//! The record: magic `HHB1`, the six address octets (most significant first, as the screen shows
//! them), the label's length and its bytes (the advertised name, at most [`LABEL_MAX`] B), and a
//! CRC-32 (IEEE, little-endian) over everything before it. A sector without a valid record is "no
//! pick".

use core::fmt::Write;

use heapless::String;

/// The record's first four bytes.
pub const MAGIC: [u8; 4] = *b"HHB1";
/// The longest label, bytes.
pub const LABEL_MAX: usize = 29;
/// The longest record, rounded up to the flash's 4-byte word.
pub const RECORD_MAX: usize = (4 + 6 + 1 + LABEL_MAX + 4).next_multiple_of(4);

/// A picked bridge: its advertised address and the name it advertised.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Pick {
    /// Most significant octet first.
    pub addr: [u8; 6],
    pub label: String<LABEL_MAX>,
}

impl Pick {
    /// A pick of `addr` named `name`, the name cut at a character boundary to [`LABEL_MAX`] bytes.
    pub fn new(addr: [u8; 6], name: &str) -> Pick {
        let mut label = String::new();
        for c in name.chars() {
            if label.push(c).is_err() {
                break;
            }
        }
        Pick { addr, label }
    }

    /// The label to show: the name, or the address when the bridge advertised none.
    pub fn shown(&self) -> String<LABEL_MAX> {
        if self.label.is_empty() {
            let mut s = String::new();
            let _ = s.push_str(&addr_text(&self.addr));
            s
        } else {
            self.label.clone()
        }
    }
}

/// The address as the screens show it: six colon-separated lower-case octets, most significant
/// first.
pub fn addr_text(a: &[u8; 6]) -> String<17> {
    let mut s = String::new();
    let _ = write!(
        s,
        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        a[0], a[1], a[2], a[3], a[4], a[5]
    );
    s
}

/// Whether `a` is a static random address (its top two bits set), which is how every bridge
/// advertises (section 3.2). Any other address is connected to as a public one.
pub fn is_static_random(a: &[u8; 6]) -> bool {
    a[0] & 0xC0 == 0xC0
}

/// CRC-32/IEEE (the reflected 0xEDB88320 polynomial, initial and final XOR all ones).
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in bytes {
        crc ^= u32::from(b);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// The record for `p`, padded with `0xFF` (erased flash) to [`RECORD_MAX`].
pub fn encode(p: &Pick) -> [u8; RECORD_MAX] {
    let mut r = [0xFF; RECORD_MAX];
    let label = p.label.as_bytes();
    r[..4].copy_from_slice(&MAGIC);
    r[4..10].copy_from_slice(&p.addr);
    r[10] = label.len() as u8;
    let end = 11 + label.len();
    r[11..end].copy_from_slice(label);
    let crc = crc32(&r[..end]);
    r[end..end + 4].copy_from_slice(&crc.to_le_bytes());
    r
}

/// The pick a sector's first bytes hold, or `None`: a wrong magic (an erased sector included), a
/// label over [`LABEL_MAX`] or not UTF-8, or a CRC that does not match.
pub fn decode(r: &[u8]) -> Option<Pick> {
    if r.len() < 11 || r[..4] != MAGIC {
        return None;
    }
    let n = usize::from(r[10]);
    let end = 11 + n;
    if n > LABEL_MAX || r.len() < end + 4 {
        return None;
    }
    let crc = u32::from_le_bytes(r[end..end + 4].try_into().ok()?);
    if crc != crc32(&r[..end]) {
        return None;
    }
    let label = core::str::from_utf8(&r[11..end]).ok()?;
    let mut addr = [0; 6];
    addr.copy_from_slice(&r[4..10]);
    Some(Pick::new(addr, label))
}

/// Whether picking `p` writes the sector: only when it differs from the record held.
pub fn needs_write(held: Option<&Pick>, p: &Pick) -> bool {
    held != Some(p)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bridge() -> Pick {
        Pick::new([0xe0, 0x6e, 0xf1, 0x12, 0x34, 0x56], "hoverboard-left")
    }

    #[test]
    fn the_crc_is_crc32_ieee() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn the_record_round_trips() {
        let r = encode(&bridge());
        assert_eq!(&r[..4], b"HHB1");
        assert_eq!(decode(&r), Some(bridge()));
        let none = Pick::new([1, 2, 3, 4, 5, 6], "");
        assert_eq!(decode(&encode(&none)), Some(none));
        let long = Pick::new([1; 6], "abcdefghijklmnopqrstuvwxyz0123456789");
        assert_eq!(long.label.len(), LABEL_MAX);
        assert_eq!(decode(&encode(&long)), Some(long));
    }

    #[test]
    fn a_bad_crc_a_wrong_magic_an_erased_sector_and_a_long_label_read_as_no_pick() {
        let good = encode(&bridge());
        let mut r = good;
        r[12] ^= 1;
        assert_eq!(decode(&r), None, "bad CRC");
        let mut r = good;
        r[3] = b'2';
        assert_eq!(decode(&r), None, "wrong magic");
        assert_eq!(decode(&[0xFF; 4096]), None, "erased sector");
        // A label of 30 B with its CRC right: still no pick.
        let mut r = [0u8; 64];
        r[..4].copy_from_slice(&MAGIC);
        r[10] = 30;
        let end = 11 + 30;
        let crc = crc32(&r[..end]);
        r[end..end + 4].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode(&r), None, "label over 29 B");
    }

    #[test]
    fn a_pick_equal_to_the_record_writes_nothing() {
        let held = bridge();
        assert!(!needs_write(Some(&held), &bridge()));
        assert!(needs_write(None, &bridge()));
        let renamed = Pick::new(held.addr, "hoverboard-right");
        assert!(needs_write(Some(&held), &renamed), "a label that differs");
        let moved = Pick::new([0xe0, 0, 0, 0, 0, 1], &held.label);
        assert!(needs_write(Some(&held), &moved), "an address that differs");
    }

    #[test]
    fn the_address_text_and_kind() {
        assert_eq!(addr_text(&bridge().addr), "e0:6e:f1:12:34:56");
        assert!(is_static_random(&bridge().addr));
        assert!(!is_static_random(&[0x20, 0x6e, 0xf1, 0x12, 0x34, 0x56]));
        assert_eq!(
            Pick::new([0xc0, 0, 0, 0, 0, 1], "").shown(),
            "c0:00:00:00:00:01"
        );
        assert_eq!(bridge().shown(), "hoverboard-left");
    }
}
