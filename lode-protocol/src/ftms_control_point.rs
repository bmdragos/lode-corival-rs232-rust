//! Pure handler for FTMS Control Point (0x2AD9) writes.
//!
//! Ported from `ftms_control_point.cpp`. The Rust version collapses the
//! separate `action` enum + `newTargetWatts` field from the C++ result struct
//! into a single sum type: `FtmsCpAction::SetTargetPower(i16)` carries its
//! data inline, and the type system enforces that `newTargetWatts` only
//! exists when that variant is selected.

// ---- Opcodes (per Bluetooth SIG Fitness Machine Service spec) -----------

pub const FTMS_CP_REQUEST_CONTROL: u8 = 0x00;
pub const FTMS_CP_RESET: u8 = 0x01;
pub const FTMS_CP_SET_TARGET_POWER: u8 = 0x05;
pub const FTMS_CP_START_RESUME: u8 = 0x07;
pub const FTMS_CP_STOP_PAUSE: u8 = 0x08;
pub const FTMS_CP_RESPONSE_CODE: u8 = 0x80;

// ---- Result codes -------------------------------------------------------

pub const FTMS_RESULT_SUCCESS: u8 = 0x01;
pub const FTMS_RESULT_NOT_SUPPORTED: u8 = 0x02;
pub const FTMS_RESULT_INVALID_PARAM: u8 = 0x03;
pub const FTMS_RESULT_OPERATION_FAILED: u8 = 0x04;
pub const FTMS_RESULT_CONTROL_NOT_PERMITTED: u8 = 0x05;

/// Size of a Control Point response: `[0x80, opcode, result]`.
pub const FTMS_CP_RESPONSE_SIZE: usize = 3;

/// Action the caller should perform after the handler returns.
/// The handler itself never touches shared state; the caller is responsible
/// for applying the action (e.g. writing `target_watts` under its lock).
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum FtmsCpAction {
    /// No state change; just send the response.
    /// (Emitted for unknown opcodes and malformed `SET_TARGET_POWER` payloads.)
    Noop,
    /// Host requested control - may trigger an observable state change.
    RequestControl,
    /// Reset target power to zero.
    Reset,
    /// Apply the given (already-clamped) target watts.
    SetTargetPower(i16),
    /// Start or resume the workout.
    StartResume,
    /// Stop or pause the workout.
    StopPause,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct FtmsCpResult {
    /// Bytes to write back on the Control Point characteristic.
    pub response: [u8; FTMS_CP_RESPONSE_SIZE],
    pub action: FtmsCpAction,
}

/// Handle an incoming FTMS Control Point write.
///
/// Pure: no IO, no shared state, no locks. Returns `None` when the input is
/// empty (caller should do nothing). Otherwise returns a response payload
/// to write back plus an action describing any state change the caller
/// should apply.
///
/// Power values in `SET_TARGET_POWER` are clamped silently to
/// `[min_power_watts, max_power_watts]`; the response still reports SUCCESS
/// in that case (matches legacy firmware behavior).
#[must_use]
pub fn handle_ftms_control_point(
    data: &[u8],
    min_power_watts: i16,
    max_power_watts: i16,
) -> Option<FtmsCpResult> {
    let (&op_code, rest) = data.split_first()?;

    let (result_code, action) = match op_code {
        FTMS_CP_REQUEST_CONTROL => (FTMS_RESULT_SUCCESS, FtmsCpAction::RequestControl),
        FTMS_CP_RESET => (FTMS_RESULT_SUCCESS, FtmsCpAction::Reset),
        FTMS_CP_SET_TARGET_POWER => {
            if rest.len() >= 2 {
                let requested = i16::from_le_bytes([rest[0], rest[1]]);
                let clamped = requested.clamp(min_power_watts, max_power_watts);
                (FTMS_RESULT_SUCCESS, FtmsCpAction::SetTargetPower(clamped))
            } else {
                (FTMS_RESULT_INVALID_PARAM, FtmsCpAction::Noop)
            }
        }
        FTMS_CP_START_RESUME => (FTMS_RESULT_SUCCESS, FtmsCpAction::StartResume),
        FTMS_CP_STOP_PAUSE => (FTMS_RESULT_SUCCESS, FtmsCpAction::StopPause),
        _ => (FTMS_RESULT_NOT_SUPPORTED, FtmsCpAction::Noop),
    };

    Some(FtmsCpResult {
        response: [FTMS_CP_RESPONSE_CODE, op_code, result_code],
        action,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIN_W: i16 = 7;
    const MAX_W: i16 = 1000;

    fn call(data: &[u8]) -> Option<FtmsCpResult> {
        handle_ftms_control_point(data, MIN_W, MAX_W)
    }

    // ---- Individual opcodes -----------------------------------------------

    #[test]
    fn request_control() {
        let r = call(&[FTMS_CP_REQUEST_CONTROL]).unwrap();
        assert_eq!(
            r.response,
            [0x80, FTMS_CP_REQUEST_CONTROL, FTMS_RESULT_SUCCESS]
        );
        assert_eq!(r.action, FtmsCpAction::RequestControl);
    }

    #[test]
    fn reset() {
        let r = call(&[FTMS_CP_RESET]).unwrap();
        assert_eq!(r.response[2], FTMS_RESULT_SUCCESS);
        assert_eq!(r.action, FtmsCpAction::Reset);
    }

    #[test]
    fn start_resume() {
        let r = call(&[FTMS_CP_START_RESUME]).unwrap();
        assert_eq!(r.response[2], FTMS_RESULT_SUCCESS);
        assert_eq!(r.action, FtmsCpAction::StartResume);
    }

    #[test]
    fn stop_pause() {
        let r = call(&[FTMS_CP_STOP_PAUSE]).unwrap();
        assert_eq!(r.response[2], FTMS_RESULT_SUCCESS);
        assert_eq!(r.action, FtmsCpAction::StopPause);
    }

    // ---- Set Target Power --------------------------------------------------

    #[test]
    fn set_target_power_in_range() {
        // 150 = 0x0096 LE => 0x96 0x00
        let r = call(&[FTMS_CP_SET_TARGET_POWER, 0x96, 0x00]).unwrap();
        assert_eq!(r.response[2], FTMS_RESULT_SUCCESS);
        assert_eq!(r.action, FtmsCpAction::SetTargetPower(150));
    }

    #[test]
    fn set_target_power_clamps_below_min() {
        let r = call(&[FTMS_CP_SET_TARGET_POWER, 0x01, 0x00]).unwrap(); // 1 W
        assert_eq!(r.action, FtmsCpAction::SetTargetPower(MIN_W));
        // Clamping is silent - still SUCCESS.
        assert_eq!(r.response[2], FTMS_RESULT_SUCCESS);
    }

    #[test]
    fn set_target_power_clamps_above_max() {
        // 2000W = 0x07D0 LE => 0xD0 0x07
        let r = call(&[FTMS_CP_SET_TARGET_POWER, 0xD0, 0x07]).unwrap();
        assert_eq!(r.action, FtmsCpAction::SetTargetPower(MAX_W));
    }

    #[test]
    fn set_target_power_negative_clamps_to_min() {
        // -50 as int16 LE = 0xFFCE => 0xCE 0xFF
        let r = call(&[FTMS_CP_SET_TARGET_POWER, 0xCE, 0xFF]).unwrap();
        assert_eq!(r.action, FtmsCpAction::SetTargetPower(MIN_W));
    }

    #[test]
    fn set_target_power_short_payload_is_invalid() {
        let r = call(&[FTMS_CP_SET_TARGET_POWER]).unwrap();
        assert_eq!(r.response[2], FTMS_RESULT_INVALID_PARAM);
        assert_eq!(r.action, FtmsCpAction::Noop);

        let r = call(&[FTMS_CP_SET_TARGET_POWER, 0x64]).unwrap();
        assert_eq!(r.response[2], FTMS_RESULT_INVALID_PARAM);
    }

    #[test]
    fn set_target_power_extra_trailing_bytes_are_ignored() {
        let r = call(&[FTMS_CP_SET_TARGET_POWER, 0x96, 0x00, 0xFF, 0xFF]).unwrap();
        assert_eq!(r.action, FtmsCpAction::SetTargetPower(150));
        assert_eq!(r.response[2], FTMS_RESULT_SUCCESS);
    }

    /// Build a `SET_TARGET_POWER` payload with the given `i16` watts.
    fn set_power_frame(watts: i16) -> [u8; 3] {
        let bytes = watts.to_le_bytes();
        [FTMS_CP_SET_TARGET_POWER, bytes[0], bytes[1]]
    }

    #[test]
    fn set_target_power_boundary_values() {
        let r = call(&set_power_frame(MIN_W)).unwrap();
        assert_eq!(r.action, FtmsCpAction::SetTargetPower(MIN_W));

        let r = call(&set_power_frame(MAX_W)).unwrap();
        assert_eq!(r.action, FtmsCpAction::SetTargetPower(MAX_W));
    }

    #[test]
    fn custom_power_range_is_respected() {
        // 500 W requested, tighter range [50..200] clamps it down.
        let r =
            handle_ftms_control_point(&[FTMS_CP_SET_TARGET_POWER, 0xF4, 0x01], 50, 200).unwrap();
        assert_eq!(r.action, FtmsCpAction::SetTargetPower(200));

        let r =
            handle_ftms_control_point(&[FTMS_CP_SET_TARGET_POWER, 0x32, 0x00], 50, 200).unwrap();
        assert_eq!(r.action, FtmsCpAction::SetTargetPower(50));
    }

    // ---- Unknown / malformed ----------------------------------------------

    #[test]
    fn unknown_opcode_reports_not_supported_and_echoes_opcode() {
        let r = call(&[0xFE]).unwrap();
        assert_eq!(r.response, [0x80, 0xFE, FTMS_RESULT_NOT_SUPPORTED]);
        assert_eq!(r.action, FtmsCpAction::Noop);
    }

    #[test]
    fn empty_input_returns_none() {
        assert!(call(&[]).is_none());
    }

    #[test]
    fn response_always_starts_with_response_code_and_echoes_opcode() {
        for &op in &[
            FTMS_CP_REQUEST_CONTROL,
            FTMS_CP_RESET,
            FTMS_CP_START_RESUME,
            FTMS_CP_STOP_PAUSE,
            0xAA, // unknown
        ] {
            let r = call(&[op]).unwrap();
            assert_eq!(r.response[0], FTMS_CP_RESPONSE_CODE);
            assert_eq!(r.response[1], op);
        }
    }
}
