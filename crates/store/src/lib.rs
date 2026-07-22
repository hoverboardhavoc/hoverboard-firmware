//! Layer 1: the flash key-value config store.
//!
//! A firmware-owned, wear-aware, **log-structured** store of flat `field_id`/`index`/`type`/`value`
//! records over one 16-bit key namespace. It is a **direct** read/write store: a read returns the
//! latest committed record from flash (or the field default), a write appends a record immediately,
//! and `compact` reclaims a full region. No arm/disarm state, no RAM value-cache.
//!
//! The whole core is generic over the [`Flash`](flash::Flash) seam (its region-relative flash
//! dependency), so the record codec, log scan, append, and compaction are fully host-tested against an
//! in-RAM [`MockFlash`](flash::MockFlash) that models the silicon write rules (erase fills `0xFFFF`;
//! `program` is halfword-aligned and write-once). The on-target `FmcFlash` adapter (the only
//! `runtime-hal` dependency) is a thin, target-gated adapter added in Pass 2; this crate is
//! `base`-only.
//!
//! `no_std`; the host test suite uses `std`.

#![no_std]
// The host test suite needs std (MockFlash's Vec, assert!, formatting); the crate itself is no_std.
#[cfg(test)]
extern crate std;

pub mod field;
pub mod flash;
pub mod geometry;
pub mod key;
pub mod record;
pub mod store;
pub mod value;

// The crafted store-region builders for the host-planted scenarios (COMPACT / TORN_* / FULL), shared
// byte-identically by the emulator runner and the hardware runner. Behind `test-fields` (it references
// the reserved test fields / scenario ids) and alloc-free, so it does not compile into production.
#[cfg(feature = "test-fields")]
pub mod scenarios;

// The on-target FmcFlash adapter: the only runtime-hal dependency, target-gated to thumbv7m so the
// host build (which uses MockFlash) never compiles it or links the HAL.
#[cfg(target_arch = "arm")]
pub mod fmc_flash;

#[cfg(test)]
mod tests;

pub use field::{
    lookup, registry, BlobField, Field, FieldDef, StrField, BOARD_BUTTON, BOARD_BUZZER,
    BOARD_SELF_HOLD, BOARD_VBATT, CONTROL_MODE, DEVICE_NAME, IMU_GYRO_BIAS, IMU_MODEL, IMU_SCL_PIN,
    IMU_SDA_PIN, LED_GREEN, LED_ORANGE, LED_RED, LINK_SET, MOTOR_ALIGN_OFFSET, MOTOR_CURRENT_LIMIT,
    MOTOR_CURRENT_SENSE, MOTOR_DEAD_TIME, MOTOR_DIRECTION, MOTOR_GATE_HI_A, MOTOR_GATE_HI_B,
    MOTOR_GATE_HI_C, MOTOR_GATE_LO_A, MOTOR_GATE_LO_B, MOTOR_GATE_LO_C, MOTOR_HALL_A, MOTOR_HALL_B,
    MOTOR_HALL_C, MOTOR_METHOD, NODE_ADDRESS, PAD_A, PAD_B, PIN_ABSENT, REGISTRY_LEN, SOME_BLOB,
};
// The store-test fields, value consts, and scenario ids are gated behind `test-fields` (see field.rs);
// re-export them only when the feature is on so a production build's API stays the genuine tunables.
#[cfg(feature = "test-fields")]
pub use field::{
    COMPACT, FULL, PERSIST, TORN_HEADER, TORN_PAYLOAD, T_BLOB, T_BLOB_VAL, T_KEY, T_STR_VAL, T_VAL,
    VAR_VALUE,
};
pub use flash::Flash;
pub use key::{Key, Scalar, Type};
pub use store::{DynError, Store, StoreError};
pub use value::Value;

#[cfg(target_arch = "arm")]
pub use fmc_flash::FmcFlash;

/// Drives every tier's SCALAR scenarios: it takes a `&mut Flash` and the host-packed
/// `cmd = (scenario << 16) | phase`, dispatches to the scenario, **cold-mounts** (the "reboot": all
/// in-RAM state is discarded and the frontier rebuilt from flash, so a surviving value is provably
/// from flash, not RAM), does one step, and returns the scalar read-back (the host re-derives the
/// expected and asserts, non-circular). The variable-value scenario ([`VAR_VALUE`]) is driven through
/// [`run_var`] instead (it returns multi-byte data).
///
/// On host this compiles over [`MockFlash`](flash::MockFlash); the thumbv7m `store-test` wrapper over
/// [`FmcFlash`](fmc_flash::FmcFlash) calls the same function.
///
/// Behind `test-fields` (it drives the reserved test fields / scenario ids), so it does not compile
/// into a production build.
#[cfg(feature = "test-fields")]
pub fn run<F: Flash>(flash: &mut F, cmd: u32) -> u32 {
    let (scenario, phase) = (cmd >> 16, cmd & 0xFFFF);
    // Cold mount = the "reboot". For the host-planted recovery scenarios (COMPACT / TORN_*), mount
    // itself performs the recovery (skip a torn payload, auto-compact a torn header), so a plain
    // mount + read is the whole device step.
    let mut store = Store::mount(flash).unwrap();
    match scenario {
        // phase 0 = set (the append *is* the persist); any other phase = cold-mount + read.
        PERSIST => {
            if phase == 0 {
                store.set(T_KEY, T_VAL).unwrap();
                0
            } else {
                store.get(T_KEY)
            }
        }
        // Host-planted: the region already holds the records; the device just cold-mounts (recovering
        // as needed) and reads the surviving T_KEY, which the host asserts against T_VAL.
        COMPACT | TORN_PAYLOAD | TORN_HEADER => store.get(T_KEY),
        // Full -> compact -> retry: the host plants a near-full active page, then phase 0 here writes
        // T_KEY, which returns Full; the device compacts and retries the same write. Phase 1 reads back.
        FULL => {
            if phase == 0 {
                match store.set(T_KEY, T_VAL) {
                    Ok(()) => 0,
                    Err(StoreError::Full) => {
                        store.compact().unwrap();
                        store.set(T_KEY, T_VAL).unwrap();
                        0
                    }
                    Err(e) => panic!("unexpected store error in FULL: {e:?}"),
                }
            } else {
                store.get(T_KEY)
            }
        }
        _ => 0,
    }
}

/// Drives the variable-value ([`VAR_VALUE`]) scenario, which carries a multi-byte read-back the
/// scalar [`run`] cannot. Phase 0 sets both a `STR` ([`DEVICE_NAME`] = [`T_STR_VAL`]) and a `BLOB`
/// ([`T_BLOB`] = [`T_BLOB_VAL`]); phase 1 reads the `STR` back, phase 2 reads the `BLOB` back. On a
/// read phase it copies the read-back bytes into `out` and returns the byte count (the firmware
/// publishes `out[..n]` as `TestResult.buf`/`len`; the host compares byte-identical). Phase 0 (the
/// write) returns `0`. A non-`VAR_VALUE` scenario returns `0` and writes nothing.
///
/// The STR round-trip reuses the genuine [`DEVICE_NAME`] field (its "hoverboard" default differs from
/// [`T_STR_VAL`], so a no-write read still returns the default, not the written value).
///
/// `out` must be at least as large as the longest read-back ([`T_STR_VAL`] / [`T_BLOB_VAL`]); a
/// shorter slice truncates (the firmware sizes `out` to `RESULT_BUF_LEN`).
///
/// Behind `test-fields` (it drives the reserved test fields / scenario ids), so it does not compile
/// into a production build.
#[cfg(feature = "test-fields")]
pub fn run_var<F: Flash>(flash: &mut F, cmd: u32, out: &mut [u8]) -> usize {
    let (scenario, phase) = (cmd >> 16, cmd & 0xFFFF);
    if scenario != VAR_VALUE {
        return 0;
    }
    let mut store = Store::mount(flash).unwrap();
    match phase {
        0 => {
            // The two appends *are* the persist; reboot happens between phases (a fresh mount).
            store.set_str(DEVICE_NAME, T_STR_VAL).unwrap();
            store.set_bytes(T_BLOB, T_BLOB_VAL).unwrap();
            0
        }
        1 => copy_into(store.get_str(DEVICE_NAME).as_bytes(), out),
        _ => copy_into(store.get_bytes(T_BLOB), out),
    }
}

/// Copy `src` into `out` (truncating to `out.len()`), returning the number of bytes copied.
#[cfg(feature = "test-fields")]
fn copy_into(src: &[u8], out: &mut [u8]) -> usize {
    let n = core::cmp::min(src.len(), out.len());
    out[..n].copy_from_slice(&src[..n]);
    n
}
