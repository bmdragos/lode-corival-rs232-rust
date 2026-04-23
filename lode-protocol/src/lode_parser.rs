//! Parser for Lode RS-232 numeric responses of the form `"<device>,<value>"`.
//!
//! Ported from `lode_parser.cpp` in the C++ firmware. The C++ version returned
//! `-1` as an error sentinel, which collided with the legitimate value `-1`.
//! This port returns a typed [`Result`] so errors and valid negative readings
//! are unambiguously distinct.

/// Why a Lode response failed to parse.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ParseError {
    /// Input doesn't have the expected `"<device>,..."` framing, or the
    /// device number doesn't match what we asked for.
    MalformedFrame,
    /// The value portion after the comma is empty or contains non-digit
    /// characters (a leading `-` is allowed, but not `+`, spaces, decimals,
    /// or garbage).
    InvalidNumber,
    /// Value is outside the plausible range for a Lode ergometer reading.
    /// Sanity bounds are `-10..=2000`; a reading outside that is either a
    /// bike malfunction or RS-232 line noise.
    OutOfRange,
}

/// Lower/upper sanity bounds on any numeric Lode reading (watts, RPM, status).
/// A real bike can't report values outside this window under normal operation;
/// anything out here is treated as corruption.
const MIN_VALUE: i32 = -10;
const MAX_VALUE: i32 = 2000;

/// Parse a Lode RS-232 response.
///
/// # Examples
///
/// ```
/// use lode_protocol::lode_parser::{parse_numeric_response, ParseError};
///
/// assert_eq!(parse_numeric_response("0,150", 0), Ok(150));
/// assert_eq!(parse_numeric_response("1,60", 0), Err(ParseError::MalformedFrame));
/// assert_eq!(parse_numeric_response("0,1ABC", 0), Err(ParseError::InvalidNumber));
/// assert_eq!(parse_numeric_response("0,9999", 0), Err(ParseError::OutOfRange));
/// ```
pub fn parse_numeric_response(response: &str, expected_device: u8) -> Result<i32, ParseError> {
    let (device_str, value_str) = response
        .split_once(',')
        .ok_or(ParseError::MalformedFrame)?;

    // Device number must be canonical: non-empty, all digits, no leading zero
    // on a multi-digit number. This keeps "01,5" != "1,5" so the prefix check
    // is unambiguous regardless of how expected_device is formatted.
    if device_str.is_empty()
        || (device_str.len() > 1 && device_str.starts_with('0'))
        || !device_str.chars().all(|c| c.is_ascii_digit())
    {
        return Err(ParseError::MalformedFrame);
    }
    let device: u8 = device_str.parse().map_err(|_| ParseError::MalformedFrame)?;
    if device != expected_device {
        return Err(ParseError::MalformedFrame);
    }

    // Value must be non-empty, one optional leading '-', then one or more
    // ASCII digits. Reject '+', whitespace, decimals, anything else.
    if value_str.is_empty() {
        return Err(ParseError::InvalidNumber);
    }
    let digits = value_str.strip_prefix('-').unwrap_or(value_str);
    if digits.is_empty() || !digits.chars().all(|c| c.is_ascii_digit()) {
        return Err(ParseError::InvalidNumber);
    }

    // Wide parse, then range-check. i32 overflow on a digits-only value gets
    // reported as OutOfRange, which is what it effectively is.
    let value: i32 = value_str.parse().map_err(|_| ParseError::OutOfRange)?;
    if !(MIN_VALUE..=MAX_VALUE).contains(&value) {
        return Err(ParseError::OutOfRange);
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- valid inputs --------------------------------------------------

    #[test]
    fn valid_responses() {
        assert_eq!(parse_numeric_response("0,100", 0), Ok(100));
        assert_eq!(parse_numeric_response("0,0", 0), Ok(0));
        assert_eq!(parse_numeric_response("0,7", 0), Ok(7));     // MIN_POWER_WATTS
        assert_eq!(parse_numeric_response("0,999", 0), Ok(999)); // MAX_POWER_WATTS
        assert_eq!(parse_numeric_response("0,2000", 0), Ok(2000));
    }

    #[test]
    fn different_device_numbers() {
        assert_eq!(parse_numeric_response("1,60", 1), Ok(60));
        assert_eq!(parse_numeric_response("9,42", 9), Ok(42));
        assert_eq!(parse_numeric_response("99,150", 99), Ok(150));
    }

    // ---- device number mismatches --------------------------------------

    #[test]
    fn wrong_device_number_is_rejected() {
        assert_eq!(parse_numeric_response("5,100", 0), Err(ParseError::MalformedFrame));
        assert_eq!(parse_numeric_response("1,100", 0), Err(ParseError::MalformedFrame));
        assert_eq!(parse_numeric_response("10,100", 1), Err(ParseError::MalformedFrame));
        assert_eq!(parse_numeric_response("0,100", 1), Err(ParseError::MalformedFrame));
    }

    #[test]
    fn prefix_collision_guards() {
        // "10,5" must not match expected_device=1 via loose prefix matching
        assert_eq!(parse_numeric_response("10,5", 1), Err(ParseError::MalformedFrame));
        // "01,5" is not canonical form for device 0
        assert_eq!(parse_numeric_response("01,5", 0), Err(ParseError::MalformedFrame));
    }

    // ---- framing problems ----------------------------------------------

    #[test]
    fn empty_input() {
        assert_eq!(parse_numeric_response("", 0), Err(ParseError::MalformedFrame));
    }

    #[test]
    fn malformed_frames() {
        assert_eq!(parse_numeric_response("0", 0), Err(ParseError::MalformedFrame));
        assert_eq!(parse_numeric_response(",100", 0), Err(ParseError::MalformedFrame));
        assert_eq!(parse_numeric_response("0100", 0), Err(ParseError::MalformedFrame));
        assert_eq!(parse_numeric_response("0 100", 0), Err(ParseError::MalformedFrame));
    }

    // ---- value problems ------------------------------------------------

    #[test]
    fn empty_value_after_comma() {
        assert_eq!(parse_numeric_response("0,", 0), Err(ParseError::InvalidNumber));
    }

    #[test]
    fn lone_minus_sign() {
        assert_eq!(parse_numeric_response("0,-", 0), Err(ParseError::InvalidNumber));
    }

    #[test]
    fn garbage_characters_in_value() {
        assert_eq!(parse_numeric_response("0,1ABC", 0), Err(ParseError::InvalidNumber));
        assert_eq!(parse_numeric_response("0,ABC", 0), Err(ParseError::InvalidNumber));
        assert_eq!(parse_numeric_response("0,1.5", 0), Err(ParseError::InvalidNumber));
        assert_eq!(parse_numeric_response("0,1 2", 0), Err(ParseError::InvalidNumber));
        assert_eq!(parse_numeric_response("0,X%#@", 0), Err(ParseError::InvalidNumber));
        assert_eq!(parse_numeric_response("0,+100", 0), Err(ParseError::InvalidNumber));
    }

    // ---- bounds --------------------------------------------------------

    #[test]
    fn out_of_range_values() {
        assert_eq!(parse_numeric_response("0,2001", 0), Err(ParseError::OutOfRange));
        assert_eq!(parse_numeric_response("0,999999", 0), Err(ParseError::OutOfRange));
        assert_eq!(parse_numeric_response("0,-11", 0), Err(ParseError::OutOfRange));
        assert_eq!(parse_numeric_response("0,-100", 0), Err(ParseError::OutOfRange));
    }

    #[test]
    fn negative_values_within_tolerance() {
        // This is the key win over the C++ version: -1 is unambiguously a
        // valid reading, not the error sentinel it used to collide with.
        assert_eq!(parse_numeric_response("0,-1", 0), Ok(-1));
        assert_eq!(parse_numeric_response("0,-5", 0), Ok(-5));
        assert_eq!(parse_numeric_response("0,-10", 0), Ok(-10));
    }

    #[test]
    fn rpm_zero_is_valid() {
        // Bike stopped, rider not pedaling.
        assert_eq!(parse_numeric_response("0,0", 0), Ok(0));
    }
}
