//! The tune format (`specs/tones.md` section 3): a `BLOB` value that is a sequence of 3-byte
//! notes and nothing else (no header, no count). The one owner of the note layout: the board's
//! player reads notes through [`note_at`], the configuration tools write and render them through
//! [`Note::encode`] and [`Note::decode`].
//!
//! | bytes | field | meaning |
//! |---|---|---|
//! | 0..2 | `hz: u16` LE | 0 = rest |
//! | 2 | `dur: u8` | duration in [`DUR_UNIT_MS`] units; 0 = skip the note |
//!
//! A trailing partial note (fewer than [`NOTE_LEN`] bytes) is not a note: [`count`] and
//! [`note_at`] leave it out.

/// The bytes of one note.
pub const NOTE_LEN: usize = 3;

/// The unit of [`Note::dur`], in milliseconds.
pub const DUR_UNIT_MS: u32 = 10;

/// One note of a tune.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Note {
    /// The pitch in Hz, `0` = a rest.
    pub hz: u16,
    /// The duration in [`DUR_UNIT_MS`] units, `0` = the note is skipped.
    pub dur: u8,
}

impl Note {
    /// The note a tune's three bytes encode.
    pub const fn decode(b: [u8; NOTE_LEN]) -> Note {
        Note {
            hz: u16::from_le_bytes([b[0], b[1]]),
            dur: b[2],
        }
    }

    /// This note's three bytes.
    pub const fn encode(self) -> [u8; NOTE_LEN] {
        let h = self.hz.to_le_bytes();
        [h[0], h[1], self.dur]
    }

    /// Whether the note sounds or rests for any time at all (`dur > 0`).
    pub const fn playable(self) -> bool {
        self.dur != 0
    }

    /// The note's duration in milliseconds.
    pub const fn ms(self) -> u32 {
        self.dur as u32 * DUR_UNIT_MS
    }
}

/// The whole notes `tune` holds.
pub const fn count(tune: &[u8]) -> usize {
    tune.len() / NOTE_LEN
}

/// Note `i` of `tune`, or `None` past its last whole note.
pub fn note_at(tune: &[u8], i: usize) -> Option<Note> {
    let b = tune.get(i * NOTE_LEN..i * NOTE_LEN + NOTE_LEN)?;
    Some(Note::decode([b[0], b[1], b[2]]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_note_is_hz_little_endian_then_dur() {
        let n = Note { hz: 2000, dur: 15 };
        assert_eq!(n.encode(), [0xD0, 0x07, 15]);
        assert_eq!(Note::decode([0xD0, 0x07, 15]), n);
        assert_eq!(n.ms(), 150);
    }

    #[test]
    fn a_trailing_partial_note_is_not_a_note() {
        let t = [0xD0, 0x07, 15, 0x00, 0x00];
        assert_eq!(count(&t), 1);
        assert_eq!(note_at(&t, 0), Some(Note { hz: 2000, dur: 15 }));
        assert_eq!(note_at(&t, 1), None);
        assert_eq!(count(&[]), 0);
    }
}
