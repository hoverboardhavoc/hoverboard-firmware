//! The tune field kind (`specs/tones.md` section 5): a tune's bytes as raw `hz/dur` notes, and the
//! two written forms that become those bytes, raw notes and RTTTL. The note layout is the store's
//! (`store::tune`); this module only reads and writes text.

use net::walk::CONFIG_RESP_VALUE_MAX;
use store::tune::{note_at, Note, DUR_UNIT_MS, NOTE_LEN};

/// The most notes a tune holds: what one `CONFIG_RESP` carries back (19).
pub const MAX_NOTES: usize = CONFIG_RESP_VALUE_MAX / NOTE_LEN;

/// The lowest pitch a sounding note takes, Hz.
pub const HZ_MIN: u16 = 20;
/// The highest pitch a sounding note takes, Hz: the period ISR's highest tone.
pub const HZ_MAX: u16 = 8000;

/// A tune's bytes as raw notes, `"2000/15 0/10 2000/15"`; a trailing partial note, which the board
/// ignores, is left out.
pub fn render(tune: &[u8]) -> String {
    (0..store::tune::count(tune))
        .filter_map(|i| note_at(tune, i))
        .map(|n| format!("{}/{}", n.hz, n.dur))
        .collect::<Vec<_>>()
        .join(" ")
}

/// A written tune as its bytes: RTTTL when the text contains `:`, else raw `hz/dur` notes separated
/// by spaces. `Err` names what is wrong and where.
pub fn parse(text: &str) -> Result<Vec<u8>, String> {
    let notes = if text.contains(':') {
        rtttl(text)?
    } else {
        raw(text)?
    };
    if notes.len() > MAX_NOTES {
        return Err(format!(
            "{} notes, more than the {MAX_NOTES} a tune holds",
            notes.len()
        ));
    }
    Ok(notes.iter().flat_map(|n| n.encode()).collect())
}

/// Whether `hz` is a pitch a note may take: a rest, or `20..=8000`.
fn pitch_ok(hz: u32) -> bool {
    hz == 0 || (u32::from(HZ_MIN)..=u32::from(HZ_MAX)).contains(&hz)
}

/// Raw notes: `hz/dur` pairs separated by spaces.
fn raw(text: &str) -> Result<Vec<Note>, String> {
    text.split_whitespace()
        .enumerate()
        .map(|(i, tok)| {
            let at = |what: &str| format!("note {} ({tok}): {what}", i + 1);
            let (hz, dur) = tok.split_once('/').ok_or_else(|| at("not hz/dur"))?;
            let hz: u32 = hz.parse().map_err(|_| at("hz is not a number"))?;
            if !pitch_ok(hz) {
                return Err(at(&format!("hz {hz} is outside 0 and {HZ_MIN}..={HZ_MAX}")));
            }
            let dur: u32 = dur.parse().map_err(|_| at("dur is not a number"))?;
            if dur == 0 {
                return Err(at("dur 0 plays nothing"));
            }
            let dur = u8::try_from(dur).map_err(|_| at("dur is more than 255 (2.55 s)"))?;
            Ok(Note { hz: hz as u16, dur })
        })
        .collect()
}

/// RTTTL: `name:d=4,o=5,b=120:c6,8p,a#.5`. The defaults `d` (duration), `o` (octave) and `b`
/// (beats per minute) take RTTTL's own values when left out (4, 6 and 63). Each note is
/// `[dur]note[#][.][octave]` (a dot after the octave is read the same), `p` a rest, `.` 1.5 times
/// the duration. A4 = 440 Hz, equal temperament, rounded to the Hz. Durations round to the nearest
/// 10 ms (at least one unit); a note longer than 2.55 s becomes back-to-back notes of its pitch.
fn rtttl(text: &str) -> Result<Vec<Note>, String> {
    let mut parts = text.splitn(3, ':');
    let _name = parts.next();
    let (Some(head), Some(body)) = (parts.next(), parts.next()) else {
        return Err("RTTTL is name:defaults:notes".into());
    };
    let (mut d, mut o, mut b) = (4u32, 6u32, 63u32);
    for kv in head.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        let bad = || format!("defaults ({kv}): not d=, o= or b= with a number");
        let (k, v) = kv.split_once('=').ok_or_else(bad)?;
        let v: u32 = v.trim().parse().map_err(|_| bad())?;
        match k.trim() {
            "d" if valid_dur(v) => d = v,
            "o" if valid_octave(v) => o = v,
            "b" if v > 0 => b = v,
            _ => return Err(bad()),
        }
    }
    let mut out = Vec::new();
    for (i, tok) in body.split(',').map(str::trim).enumerate() {
        let at = |what: &str| format!("note {} ({tok}): {what}", i + 1);
        let s = tok.to_ascii_lowercase();
        let digits = s.chars().take_while(char::is_ascii_digit).count();
        let dur = if digits == 0 {
            d
        } else {
            let v: u32 = s[..digits].parse().map_err(|_| at("bad duration"))?;
            if !valid_dur(v) {
                return Err(at("duration is not 1, 2, 4, 8, 16 or 32"));
            }
            v
        };
        let mut rest = s[digits..].chars().peekable();
        let semitone = match rest.next() {
            Some('c') => Some(0),
            Some('d') => Some(2),
            Some('e') => Some(4),
            Some('f') => Some(5),
            Some('g') => Some(7),
            Some('a') => Some(9),
            Some('b' | 'h') => Some(11),
            Some('p') => None,
            _ => return Err(at("no note letter")),
        };
        let sharp = rest.next_if_eq(&'#').is_some();
        let mut dotted = rest.next_if_eq(&'.').is_some();
        let oct: String = std::iter::from_fn(|| rest.next_if(char::is_ascii_digit)).collect();
        dotted |= rest.next_if_eq(&'.').is_some();
        if rest.next().is_some() {
            return Err(at("unexpected characters"));
        }
        let octave = if oct.is_empty() {
            o
        } else {
            let v: u32 = oct.parse().map_err(|_| at("bad octave"))?;
            if !valid_octave(v) {
                return Err(at("octave is not 0 to 8"));
            }
            v
        };
        let hz = match semitone {
            None => 0,
            Some(st) => {
                let midi = (octave + 1) * 12 + st + u32::from(sharp);
                let hz = (440.0 * 2f64.powf((f64::from(midi) - 69.0) / 12.0)).round() as u32;
                if !pitch_ok(hz) {
                    return Err(at(&format!("{hz} Hz is outside {HZ_MIN}..={HZ_MAX}")));
                }
                hz
            }
        };
        let mut ms = 240_000.0 / f64::from(b) / f64::from(dur);
        if dotted {
            ms *= 1.5;
        }
        let mut units = ((ms / f64::from(DUR_UNIT_MS)).round() as u32).max(1);
        while units > 0 {
            let n = units.min(u32::from(u8::MAX));
            out.push(Note {
                hz: hz as u16,
                dur: n as u8,
            });
            units -= n;
        }
    }
    Ok(out)
}

/// An RTTTL duration: a whole note divided by 1, 2, 4, 8, 16 or 32.
fn valid_dur(v: u32) -> bool {
    matches!(v, 1 | 2 | 4 | 8 | 16 | 32)
}

/// An RTTTL octave the tune may name.
fn valid_octave(v: u32) -> bool {
    v <= 8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn notes(text: &str) -> Vec<(u16, u8)> {
        let b = parse(text).unwrap();
        (0..store::tune::count(&b))
            .map(|i| note_at(&b, i).map(|n| (n.hz, n.dur)).unwrap())
            .collect()
    }

    #[test]
    fn raw_parses_and_renders_back() {
        let text = "2000/15 0/10 2000/15 0/100";
        let b = parse(text).unwrap();
        assert_eq!(b, [0xD0, 0x07, 15, 0, 0, 10, 0xD0, 0x07, 15, 0, 0, 100]);
        assert_eq!(render(&b), text);
        // Any run of spaces separates notes; the empty text is the empty tune.
        assert_eq!(parse("  440/5   0/1 ").unwrap().len(), 6);
        assert_eq!(parse("").unwrap(), Vec::<u8>::new());
        assert_eq!(render(&[]), "");
    }

    #[test]
    fn render_leaves_out_a_trailing_partial_note() {
        assert_eq!(render(&[0xD0, 0x07, 15, 0x01]), "2000/15");
    }

    #[test]
    fn raw_refusals_name_the_note() {
        for (text, want) in [
            ("2000/15 19/10", "note 2 (19/10): hz 19 is outside"),
            ("8001/1", "note 1 (8001/1): hz 8001"),
            ("2000/0", "note 1 (2000/0): dur 0"),
            ("2000/256", "note 1 (2000/256): dur is more than 255"),
            ("2000", "note 1 (2000): not hz/dur"),
            ("x/1", "note 1 (x/1): hz is not a number"),
        ] {
            let e = parse(text).unwrap_err();
            assert!(e.starts_with(want), "{text}: {e}");
        }
        // The edges are accepted.
        assert_eq!(notes("20/1 8000/255 0/1"), [(20, 1), (8000, 255), (0, 1)]);
    }

    #[test]
    fn more_than_nineteen_notes_is_refused() {
        assert_eq!(MAX_NOTES, 19);
        let ok = vec!["440/1"; 19].join(" ");
        assert_eq!(parse(&ok).unwrap().len(), 57);
        let e = parse(&vec!["440/1"; 20].join(" ")).unwrap_err();
        assert!(e.contains("20 notes, more than the 19"), "{e}");
        // Counted after the RTTTL conversion: two notes split into four is four.
        let e = parse(&format!("x:d=1,o=4,b=30:{}", ["c"; 10].join(","))).unwrap_err();
        assert!(e.contains("more than the 19"), "{e}");
    }

    /// Hand-computed: b=120 makes a quarter 500 ms; c6 is MIDI 84, 440 * 2^(15/12) = 1046.50;
    /// a#5 is MIDI 82, 932.33, dotted quarter 750 ms; a 32nd is 62.5 ms, 6 units; a5 is 880.
    #[test]
    fn rtttl_with_every_default_given() {
        assert_eq!(
            notes("t:d=4,o=5,b=120:c6,8p,a#.,32a"),
            [(1047, 50), (0, 25), (932, 75), (880, 6)]
        );
    }

    /// Left out, the defaults are RTTTL's own, each on its own: d=4, o=6, b=63 (a quarter of
    /// 952.4 ms, 95 units). e6 is MIDI 88, 1318.51 Hz; a6 is 1760.
    #[test]
    fn rtttl_header_defaults_one_at_a_time() {
        // b left out: 63.
        assert_eq!(notes("x:d=4,o=5:a"), [(880, 95)]);
        // o left out: 6.
        assert_eq!(notes("x:d=4,b=120:e"), [(1319, 50)]);
        // d left out: 4.
        assert_eq!(notes("x:o=4,b=60:a"), [(440, 100)]);
        // All left out.
        assert_eq!(notes("x::a"), [(1760, 95)]);
    }

    /// The sharp and the dot either side of the octave; `h` is B; case does not matter.
    #[test]
    fn rtttl_sharp_dot_and_rest() {
        // c#5 is MIDI 73, 554.37; a dotted 8th at b=100 is 450 ms.
        assert_eq!(notes("x:d=4,o=5,b=100:8c#.5"), [(554, 45)]);
        assert_eq!(notes("x:d=4,o=5,b=100:8c#5."), [(554, 45)]);
        assert_eq!(notes("x:d=4,o=5,b=100:8C#5"), [(554, 30)]);
        assert_eq!(notes("x:d=4,o=4,b=100:h"), [(494, 60)]);
        assert_eq!(notes("x:d=16,o=5,b=100:p"), [(0, 15)]);
    }

    /// A whole note at b=30 is 8 s: 800 units, split 255 + 255 + 255 + 35 at the same pitch.
    #[test]
    fn rtttl_splits_a_note_past_two_and_a_half_seconds() {
        assert_eq!(
            notes("x:d=1,o=4,b=30:a"),
            [(440, 255), (440, 255), (440, 255), (440, 35)]
        );
    }

    #[test]
    fn rtttl_refusals_name_the_position() {
        for (text, want) in [
            ("x:d=4,o=5,b=120:c,q", "note 2 (q): no note letter"),
            ("x:d=4,o=5,b=120:3c", "note 1 (3c): duration is not"),
            ("x:d=4,o=5,b=120:c9", "note 1 (c9): octave is not"),
            ("x:d=4,o=5,b=120:cx", "note 1 (cx): unexpected"),
            ("x:d=4,o=0,b=120:c", "note 1 (c): 16 Hz is outside"),
            ("x:d=3:c", "defaults (d=3)"),
            ("x:z=1:c", "defaults (z=1)"),
            ("x:c", "RTTTL is name:defaults:notes"),
        ] {
            let e = parse(text).unwrap_err();
            assert!(e.starts_with(want), "{text}: {e}");
        }
    }
}
