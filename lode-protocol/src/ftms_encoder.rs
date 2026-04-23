//! Encoders for FTMS (Fitness Machine Service) BLE characteristic payloads.
//!
//! Ported from `ftms_encoder.cpp`. Tight input types replace the C++ version's
//! buffer-pointer + length: `i16` for watts matches the FTMS wire format, and
//! `u16` for RPM forces the caller to acknowledge the "RPM can't be negative"
//! clamp rather than leaving it implicit.

/// Length of an FTMS Indoor Bike Data (0x2AD2) payload in bytes.
pub const FTMS_INDOOR_BIKE_DATA_SIZE: usize = 8;

/// Flag bits per Bluetooth SIG GATT spec for Indoor Bike Data.
pub const FTMS_IBD_FLAG_CADENCE: u16 = 1 << 2;
pub const FTMS_IBD_FLAG_POWER: u16 = 1 << 6;

/// Encode an Indoor Bike Data notification payload.
///
/// Layout (all little-endian):
///
/// | bytes | field                  | units           |
/// |-------|------------------------|-----------------|
/// | 0..2  | Flags (cadence+power)  |                 |
/// | 2..4  | Instantaneous Speed    | 0.01 km/h       |
/// | 4..6  | Instantaneous Cadence  | 0.5 rpm         |
/// | 6..8  | Instantaneous Power    | watts (int16)   |
///
/// Speed is derived from RPM assuming a ~2.0 m wheel circumference:
/// `speed_kmh = rpm * 0.12`, so the raw 0.01 km/h value is `rpm * 12`.
///
/// Overflow on the `rpm * 12` and `rpm * 2` multiplications uses
/// wrapping semantics, matching the C++ implicit truncation. RPM values
/// that overflow a `u16` aren't physically meaningful anyway.
#[must_use]
pub fn encode_indoor_bike_data(watts: i16, rpm: u16) -> [u8; FTMS_INDOOR_BIKE_DATA_SIZE] {
    let flags = FTMS_IBD_FLAG_CADENCE | FTMS_IBD_FLAG_POWER;
    let speed_raw = rpm.wrapping_mul(12);
    let cadence_raw = rpm.wrapping_mul(2);

    let mut out = [0u8; FTMS_INDOOR_BIKE_DATA_SIZE];
    out[0..2].copy_from_slice(&flags.to_le_bytes());
    out[2..4].copy_from_slice(&speed_raw.to_le_bytes());
    out[4..6].copy_from_slice(&cadence_raw.to_le_bytes());
    out[6..8].copy_from_slice(&watts.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_u16(bytes: &[u8]) -> u16 {
        u16::from_le_bytes([bytes[0], bytes[1]])
    }
    fn read_i16(bytes: &[u8]) -> i16 {
        i16::from_le_bytes([bytes[0], bytes[1]])
    }

    #[test]
    fn flags_byte_is_cadence_plus_power() {
        let buf = encode_indoor_bike_data(100, 75);
        assert_eq!(buf[0], 0x44); // bit 2 + bit 6
        assert_eq!(buf[1], 0x00);
        assert_eq!(read_u16(&buf[0..2]), FTMS_IBD_FLAG_CADENCE | FTMS_IBD_FLAG_POWER);
    }

    #[test]
    fn typical_values() {
        let buf = encode_indoor_bike_data(100, 75);
        // Speed: 75 * 12 = 900 => 9.00 km/h
        assert_eq!(read_u16(&buf[2..4]), 900);
        // Cadence: 75 * 2 = 150 => 75 rpm in 0.5 rpm units
        assert_eq!(read_u16(&buf[4..6]), 150);
        // Power: 100 W
        assert_eq!(read_i16(&buf[6..8]), 100);
    }

    #[test]
    fn zero_values() {
        let buf = encode_indoor_bike_data(0, 0);
        assert_eq!(buf[0], 0x44); // flags still set
        assert_eq!(buf[1], 0x00);
        assert_eq!(read_u16(&buf[2..4]), 0);
        assert_eq!(read_u16(&buf[4..6]), 0);
        assert_eq!(read_i16(&buf[6..8]), 0);
    }

    #[test]
    fn negative_power_is_signed_correctly() {
        let buf = encode_indoor_bike_data(-50, 60);
        assert_eq!(read_i16(&buf[6..8]), -50);

        let buf = encode_indoor_bike_data(-1, 0);
        assert_eq!(read_i16(&buf[6..8]), -1);
        // -1 as int16 LE is 0xFF 0xFF
        assert_eq!(buf[6], 0xFF);
        assert_eq!(buf[7], 0xFF);
    }

    #[test]
    fn max_power_values() {
        let buf = encode_indoor_bike_data(1000, 100);
        assert_eq!(read_i16(&buf[6..8]), 1000);

        let buf = encode_indoor_bike_data(i16::MAX, 0);
        assert_eq!(read_i16(&buf[6..8]), i16::MAX);
    }

    #[test]
    fn high_rpm_values() {
        let buf = encode_indoor_bike_data(200, 120);
        assert_eq!(read_u16(&buf[2..4]), 1440); // 120 * 12
        assert_eq!(read_u16(&buf[4..6]), 240); // 120 * 2
    }
}
