//! The Ride view's readings (`specs/handheld-remote.md` sections 5.2 and 7.1): the app's display
//! formulas (`apps/remote/app/src/main/java/com/hoverboard/remote/model/`, `specs/remote-ui.md`
//! 3.1, 3.1a, 3.6, 3.6b and 3.6d) as pure functions, with the app's test vectors.
//!
//! Nothing here draws. Each function takes what a board sent (`linkctl::CyclicState`) and the ride
//! facts read from its store (`specs/handheld-remote.md` 3.5), each an `Option` that is `None`
//! while unread, and returns what the figure shows, including its no-value form.

pub mod attitude;
pub mod battery;
pub mod current;
pub mod missing;
pub mod slots;
pub mod speed;

/// `control.side`'s right-wheel byte (`store::CONTROL_SIDE`, `shared/store/src/field.rs`: `0` left,
/// the default, `1` right, any other byte left). The store crate carries the vocabulary only in its
/// host-only name table, so the byte is stated here; the app's mirror is `Fields.ControlSide.RIGHT`.
pub const SIDE_RIGHT: u8 = 1;

/// `control.side`'s left-wheel byte, the default ([`SIDE_RIGHT`] for the source).
pub const SIDE_LEFT: u8 = 0;

/// Rounds half up, as Kotlin's `Double.roundToInt` (`Math.round`, `floor(x + 0.5)`) does, which
/// the speed figure is decided by (`SpeedReading.kt`, `Kmh.shown`).
fn round_half_up(x: f64) -> i32 {
    libm::floor(x + 0.5) as i32
}

/// Writes `tenths` (a figure in tenths) as "2.3", "-0.8", "0.0": the one-decimal form every Ride
/// figure prints, with the sign only on a figure that is not zero.
fn write_tenths<const N: usize>(out: &mut heapless::String<N>, tenths: i32) {
    use core::fmt::Write;
    let sign = if tenths < 0 { "-" } else { "" };
    let a = tenths.unsigned_abs();
    // Every caller sizes `N` for its figure's range, so the write cannot run out of room.
    let _ = write!(out, "{sign}{}.{}", a / 10, a % 10);
}

#[cfg(test)]
pub(crate) mod test_support {
    use linkctl::{ChipTag, CyclicObs, CyclicState};

    /// A board sitting still, the apps' test telemetry (`BatteryCurveTest.kt` and
    /// `CurrentReadingTest.kt`, `telemetry`): every word 0 but the ones a test sets.
    pub fn state() -> CyclicState {
        CyclicState {
            pitch: 0,
            roll: 0,
            wheel_speed: 0,
            battery: 0,
            mode: 0,
            fault: 0,
            flags: 0,
            peer_demand: 0,
            wheel_pos: 0,
            obs: CyclicObs {
                phase_peak: 0,
                phase_mean: 0,
                duty_on: 0,
                boot_tag: 1,
                chip: ChipTag::F103C8,
                odometer: 0,
            },
            wheel_rate: 0,
            machine: 0,
        }
    }
}
