//! Host tests for the MPU-6050 front-end. The I2C bus is mocked with `embedded-hal-mock`'s `I2c`,
//! which scripts the exact write / write-read transactions and asserts they all fire (`done`). The
//! scaled / signed / biased outputs are hand-computed and checked against an f64 reference.

extern crate std;

use super::*;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction};
use std::vec;

/// f64 reference for the gyro scale, used to validate the Q reproduction (spec section 5 / 7.2).
const GYRO_SCALE_REF: f64 = (500.0 / 32768.0) * core::f64::consts::PI / 180.0;

/// Q tolerance: I32F32 carries 32 fractional bits, so a single count * scale is exact to ~2e-10.
/// 1e-7 relative is a comfortable bound that still proves we are not on the datasheet-rounded 65.5
/// (which would be ~5.5e-4 off).
const Q_TOL: f64 = 1e-7;

fn approx(got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol,
        "approx fail: got {got}, want {want}, tol {tol}"
    );
}

#[test]
fn gyro_scale_reproduces_constant_within_q_tolerance() {
    // The literal spec constant matches the derived (500/32768)*(pi/180).
    approx(GYRO_SCALE, GYRO_SCALE_REF, 1e-9);
    // And the Q reproduction matches the f64 reference.
    let q: f64 = Fix::from_num(GYRO_SCALE).to_num();
    approx(q, GYRO_SCALE_REF, Q_TOL);
    // It is the exact 65.536 full-scale math, not the datasheet-rounded 65.5.
    let datasheet_rounded = (1.0 / 65.5) * core::f64::consts::PI / 180.0;
    assert!(
        (q - datasheet_rounded).abs() > 1e-7,
        "scale must not be the rounded 65.5 value"
    );
}

#[test]
fn init_writes_exact_config_bytes_to_exact_registers() {
    // Spec section 5: PWR_MGMT_1=0x00, SMPLRT_DIV=0x00, CONFIG=0x00, GYRO_CONFIG=0x08,
    // ACCEL_CONFIG=0x08, in order, each a two-byte register write.
    let expected = vec![
        Transaction::write(ADDR, vec![0x6B, 0x00]),
        Transaction::write(ADDR, vec![0x19, 0x00]),
        Transaction::write(ADDR, vec![0x1A, 0x00]),
        Transaction::write(ADDR, vec![0x1B, 0x08]),
        Transaction::write(ADDR, vec![0x1C, 0x08]),
    ];
    let mut i2c = I2cMock::new(&expected);
    let mut imu = Imu::new(MPU6050, Config::default());
    imu.init(&mut i2c).unwrap();
    i2c.done();
}

#[test]
fn read_issues_burst_and_decodes_payload_with_sign_and_scale() {
    // Payload: accel X=4096, Y=-4096, Z=8192; temp=0; gyro X=100, Y=-200, Z=300, all big-endian.
    let payload = vec![
        0x10, 0x00, // AX = 0x1000 = 4096
        0xF0, 0x00, // AY = 0xF000 = -4096
        0x20, 0x00, // AZ = 0x2000 = 8192
        0x00, 0x00, // TEMP = 0
        0x00, 0x64, // GX = 100
        0xFF, 0x38, // GY = -200
        0x01, 0x2C, // GZ = 300
    ];
    let expected = vec![Transaction::write_read(ADDR, vec![0x3B], payload)];
    let mut i2c = I2cMock::new(&expected);

    // Default sign map (-1,+1,-1,-1,+1,-1), zero gyro bias.
    let mut imu = Imu::new(MPU6050, Config::default());
    let s = imu.read(&mut i2c).unwrap();
    i2c.done();

    // Accel: sign only, no bias. AX:-1*4096, AY:+1*-4096, AZ:-1*8192.
    assert_eq!(s.accel_raw, [-4096, -4096, -8192]);

    // Gyro corrected counts: GX:-1*100=-100, GY:+1*-200=-200, GZ:-1*300=-300; bias 0.
    assert_eq!(s.gyro_raw, [-100, -200, -300]);

    // Gyro rad/s: corrected count * 0.000266316114 (f64 reference).
    for (i, &c) in [-100i32, -200, -300].iter().enumerate() {
        let want = c as f64 * GYRO_SCALE_REF;
        let got: f64 = s.gyro[i].to_num();
        approx(got, want, Q_TOL);
    }

    // Temp: (0 + 12420) / 340 = 36 centidegrees.
    assert_eq!(s.temp_centi_degc, TEMP_OFFSET / TEMP_DIV);
    assert_eq!(s.temp_centi_degc, 36);
}

#[test]
fn sign_map_flips_axes() {
    // Identity sign map vs the default flipping map on the same payload proves the flip.
    let payload = vec![
        0x00, 0x0A, // AX = 10
        0x00, 0x14, // AY = 20
        0x00, 0x1E, // AZ = 30
        0x00, 0x00, // TEMP
        0x00, 0x28, // GX = 40
        0x00, 0x32, // GY = 50
        0x00, 0x3C, // GZ = 60
    ];

    let id_cfg = Config {
        sign: [1, 1, 1, 1, 1, 1],
        gyro_bias: [0, 0, 0],
    };
    let mut i2c_id = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x3B], payload.clone())]);
    let mut imu_id = Imu::new(MPU6050, id_cfg);
    let s_id = imu_id.read(&mut i2c_id).unwrap();
    i2c_id.done();
    assert_eq!(s_id.accel_raw, [10, 20, 30]);
    assert_eq!(s_id.gyro_raw, [40, 50, 60]);

    // Default flipping map (-1,+1,-1,-1,+1,-1).
    let mut i2c_d = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x3B], payload)]);
    let mut imu_d = Imu::new(MPU6050, Config::default());
    let s_d = imu_d.read(&mut i2c_d).unwrap();
    i2c_d.done();
    assert_eq!(s_d.accel_raw, [-10, 20, -30]);
    assert_eq!(s_d.gyro_raw, [-40, 50, -60]);
}

#[test]
fn gyro_bias_subtraction_works() {
    // corrected = sign*raw - bias. Use identity sign so the bias is the only effect.
    let payload = vec![
        0, 0, 0, 0, 0, 0, // accel
        0, 0, // temp
        0x03, 0xE8, // GX = 1000
        0x07, 0xD0, // GY = 2000
        0x0B, 0xB8, // GZ = 3000
    ];
    let cfg = Config {
        sign: [1, 1, 1, 1, 1, 1],
        gyro_bias: [100, -50, 250],
    };
    let mut i2c = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x3B], payload)]);
    let mut imu = Imu::new(MPU6050, cfg);
    let s = imu.read(&mut i2c).unwrap();
    i2c.done();

    // 1000-100=900, 2000-(-50)=2050, 3000-250=2750.
    assert_eq!(s.gyro_raw, [900, 2050, 2750]);
    for (i, &c) in [900i32, 2050, 2750].iter().enumerate() {
        let want = c as f64 * GYRO_SCALE_REF;
        let got: f64 = s.gyro[i].to_num();
        approx(got, want, Q_TOL);
    }
}

#[test]
fn sign_then_bias_order_with_default_map() {
    // With the default map a negative-signed axis still subtracts the bias AFTER the sign:
    // corrected = (sign*raw) - bias. GX: -1*1000 - 100 = -1100.
    let payload = vec![
        0, 0, 0, 0, 0, 0, 0, 0, // accel + temp
        0x03, 0xE8, // GX = 1000
        0x00, 0x00, // GY = 0
        0x00, 0x00, // GZ = 0
    ];
    let cfg = Config {
        sign: [-1, 1, -1, -1, 1, -1],
        gyro_bias: [100, 0, 0],
    };
    let mut i2c = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x3B], payload)]);
    let mut imu = Imu::new(MPU6050, cfg);
    let s = imu.read(&mut i2c).unwrap();
    i2c.done();
    assert_eq!(s.gyro_raw[0], -1100);
}

#[test]
fn saturation_clamp_is_symmetric() {
    // accel raw -32768 with +1 sign clamps to -32767 (the symmetric guard, spec 7.4).
    let payload = vec![
        0x80, 0x00, // AX = -32768
        0x7F, 0xFF, // AY = 32767
        0x00, 0x00, // AZ = 0
        0x00, 0x00, // temp
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // gyro
    ];
    let cfg = Config {
        sign: [1, 1, 1, 1, 1, 1],
        gyro_bias: [0, 0, 0],
    };
    let mut i2c = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x3B], payload)]);
    let mut imu = Imu::new(MPU6050, cfg);
    let s = imu.read(&mut i2c).unwrap();
    i2c.done();
    assert_eq!(
        s.accel_raw[0], -32767,
        "low side floored at -32767, not -32768"
    );
    assert_eq!(s.accel_raw[1], 32767);
}

#[test]
fn temperature_transfer_function() {
    // (raw + 12420) / 340. raw=21000 -> (21000+12420)/340 = 33420/340 = 98.
    let mut payload = vec![0u8; BURST_LEN];
    let raw: i16 = 21000;
    payload[6] = (raw >> 8) as u8;
    payload[7] = (raw & 0xFF) as u8;
    let mut i2c = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x3B], payload)]);
    let mut imu = Imu::new(MPU6050, Config::default());
    let s = imu.read(&mut i2c).unwrap();
    i2c.done();
    assert_eq!(s.temp_centi_degc, (21000 + 12420) / 340);
    assert_eq!(s.temp_centi_degc, 98);
}

#[test]
fn iir_prefilter_applies_exact_coefficients() {
    // Fast pair 0.02/0.98 (spec section 10). First sample primes to itself, then
    // y <- 0.02*x + 0.98*y.
    let mut f = Iir::fast();
    let y0: f64 = f.step(Fix::from_num(100.0)).to_num();
    approx(y0, 100.0, 1e-9); // primed

    let y1: f64 = f.step(Fix::from_num(0.0)).to_num();
    // 0.02*0 + 0.98*100 = 98.0
    approx(y1, 0.02 * 0.0 + 0.98 * 100.0, 1e-6);

    let y2: f64 = f.step(Fix::from_num(0.0)).to_num();
    // 0.02*0 + 0.98*98 = 96.04
    approx(y2, 0.02 * 0.0 + 0.98 * 98.0, 1e-6);

    // Slow pair 0.01/0.99.
    let mut s = Iir::slow();
    let _ = s.step(Fix::from_num(10.0)); // prime to 10
    let s1: f64 = s.step(Fix::from_num(20.0)).to_num();
    // 0.01*20 + 0.99*10 = 0.2 + 9.9 = 10.1
    approx(s1, 0.01 * 20.0 + 0.99 * 10.0, 1e-6);

    // Coefficient constants are exactly the spec values.
    assert_eq!(IIR_FAST_NEW, 0.02);
    assert_eq!(IIR_FAST_OLD, 0.98);
    assert_eq!(IIR_SLOW_NEW, 0.01);
    assert_eq!(IIR_SLOW_OLD, 0.99);
}

#[test]
fn still_detection_asserts_after_threshold() {
    // Feed the same buffer repeatedly via decode; the still flag asserts once the counter exceeds
    // 50 (spec section 9). decode is the pure half of read, used here to avoid scripting 60 bus
    // transactions.
    let buf = [
        0x01, 0x00, 0x02, 0x00, 0x03, 0x00, // accel
        0x00, 0x00, // temp
        0x00, 0x10, 0x00, 0x20, 0x00, 0x30, // gyro
    ];
    let mut imu = Imu::new(MPU6050, Config::default());

    // First sample: no previous, counter 0, not still.
    let s = imu.decode(&buf);
    assert!(!s.still);
    assert_eq!(imu.still_count(), 0);

    // Identical samples increment the counter. After the (STILL_THRESHOLD+1)th increment it asserts.
    let mut last = s;
    for _ in 0..STILL_THRESHOLD as usize {
        last = imu.decode(&buf);
    }
    // counter now == STILL_THRESHOLD (50); flag is count > 50, so still false.
    assert_eq!(imu.still_count(), STILL_THRESHOLD);
    assert!(!last.still);

    // One more identical sample: counter 51 > 50, flag asserts.
    let s = imu.decode(&buf);
    assert_eq!(imu.still_count(), STILL_THRESHOLD + 1);
    assert!(s.still);

    // A different sample resets the counter and clears the flag.
    let mut other = buf;
    other[0] = 0x7F;
    let s = imu.decode(&other);
    assert_eq!(imu.still_count(), 0);
    assert!(!s.still);
}

#[test]
fn config_default_sign_map() {
    let cfg = Config::default();
    assert_eq!(cfg.sign, [-1, 1, -1, -1, 1, -1]);
    assert_eq!(cfg.gyro_bias, [0, 0, 0]);
}

#[test]
fn accel_g_uses_shared_scale() {
    // 8192 counts / 8192 LSB/g = 1.0 g.
    let payload = vec![
        0x20, 0x00, // AX = 8192
        0x00, 0x00, 0x00, 0x00, // AY, AZ = 0
        0x00, 0x00, // temp
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // gyro
    ];
    let cfg = Config {
        sign: [1, 1, 1, 1, 1, 1],
        gyro_bias: [0, 0, 0],
    };
    let mut i2c = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x3B], payload)]);
    let mut imu = Imu::new(MPU6050, cfg);
    let s = imu.read(&mut i2c).unwrap();
    i2c.done();
    let g: f64 = s.accel_g()[0].to_num();
    approx(g, 1.0, 1e-9);
}

// ---------------------------------------------------------------------------------------------
// Model table / identity probe (spec "Models"). The two entries differ only in WHO_AM_I.
// ---------------------------------------------------------------------------------------------

#[test]
fn probe_passes_when_who_am_i_matches_the_model() {
    // Genuine part: WHO_AM_I (0x75) reads 0x68.
    let mut i2c = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x75], vec![0x68])]);
    let mut imu = Imu::new(MPU6050, Config::default());
    imu.probe(&mut i2c).unwrap();
    i2c.done();

    // The bench clone: WHO_AM_I reads 0x2E.
    let mut i2c = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x75], vec![0x2E])]);
    let mut imu = Imu::new(CLONE_2E, Config::default());
    imu.probe(&mut i2c).unwrap();
    i2c.done();
}

#[test]
fn probe_fails_identity_on_model_mismatch_both_ways() {
    // A clone probed as a genuine MPU6050 is caught...
    let mut i2c = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x75], vec![0x2E])]);
    let mut imu = Imu::new(MPU6050, Config::default());
    assert_eq!(
        imu.probe(&mut i2c),
        Err(Error::Identity {
            expected: 0x68,
            got: 0x2E
        })
    );
    i2c.done();

    // ...and a genuine part probed as the clone is caught too.
    let mut i2c = I2cMock::new(&[Transaction::write_read(ADDR, vec![0x75], vec![0x68])]);
    let mut imu = Imu::new(CLONE_2E, Config::default());
    assert_eq!(
        imu.probe(&mut i2c),
        Err(Error::Identity {
            expected: 0x2E,
            got: 0x68
        })
    );
    i2c.done();
}

#[test]
fn model_table_entries_differ_only_in_identity() {
    assert_eq!(MPU6050.who_am_i, 0x68);
    assert_eq!(CLONE_2E.who_am_i, 0x2E);
    assert_ne!(MPU6050, CLONE_2E);
}
