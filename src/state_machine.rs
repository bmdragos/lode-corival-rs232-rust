//! Lode connection-lifecycle state machine.
//!
//! Ported from `lode_state_machine.{h,cpp}`. Two notable improvements over
//! the C++ version:
//!
//! 1. Poll-result outcomes use `Option<i32>` (`None` = command failed,
//!    `Some(v)` = bike returned `v`). Cleaner than the C++ "`-1` means
//!    failure" sentinel, which collided with valid negative readings.
//!
//! 2. Timestamps are `u32` to match ESP32 `millis()` exactly (the C++
//!    version used `unsigned long`, which is 64-bit on the host test
//!    machine but 32-bit on the MCU). Wrap behavior at the 49.7-day
//!    boundary is tested against `u32::MAX`.

/// The three states the RS-232 connection can be in.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum LodeState {
    /// No communication with bike; caller attempts reconnect periodically.
    Disconnected,
    /// Actively reading PM/RM.
    Polling,
    /// Transient - one tick of the state machine resets to Disconnected.
    Error,
}

/// Derive "is the bike responsive" from state. Single source of truth.
#[must_use]
pub fn is_bike_connected(s: LodeState) -> bool {
    matches!(s, LodeState::Polling)
}

/// FTMS status notifications the caller should emit after applying a
/// transition. The handler itself never performs IO.
#[derive(Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct LodeNotify {
    pub started: bool,
    pub stopped: bool,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct LodeTransition {
    pub new_state: LodeState,
    pub new_error_count: u32,
    pub notify: LodeNotify,
}

impl LodeTransition {
    const fn no_change(state: LodeState, error_count: u32) -> Self {
        Self {
            new_state: state,
            new_error_count: error_count,
            notify: LodeNotify {
                started: false,
                stopped: false,
            },
        }
    }
}

/// Unsigned-modular time-elapsed guard. `now.wrapping_sub(last)` handles
/// the `millis()` wrap correctly for any interval shorter than `u32::MAX / 2`.
#[must_use]
pub fn should_reconnect(last_attempt_ms: u32, now_ms: u32, interval_ms: u32) -> bool {
    now_ms.wrapping_sub(last_attempt_ms) >= interval_ms
}

#[must_use]
pub fn should_poll(last_poll_ms: u32, now_ms: u32, interval_ms: u32) -> bool {
    now_ms.wrapping_sub(last_poll_ms) >= interval_ms
}

/// The `VR` (version) command responded with data.
///
/// From `Disconnected` - transition to `Polling`, reset errors, notify started.
/// From any other state - no-op.
#[must_use]
pub fn on_version_ok(current: LodeState, error_count: u32) -> LodeTransition {
    if !matches!(current, LodeState::Disconnected) {
        return LodeTransition::no_change(current, error_count);
    }
    LodeTransition {
        new_state: LodeState::Polling,
        new_error_count: 0,
        notify: LodeNotify {
            started: true,
            stopped: false,
        },
    }
}

/// A poll cycle just completed.
///
/// `watts` and `rpm` are `None` when that command failed (or timed out) and
/// `Some(value)` on success. Counting rule:
/// - Any success resets `error_count` (bike is responsive).
/// - Both failing increments by 1 (lost poll cycle).
/// - Hitting `error_threshold` transitions to `Error`.
#[must_use]
pub fn on_poll_result(
    current: LodeState,
    error_count: u32,
    watts: Option<i32>,
    rpm: Option<i32>,
    error_threshold: u32,
) -> LodeTransition {
    if !matches!(current, LodeState::Polling) {
        return LodeTransition::no_change(current, error_count);
    }

    let any_success = watts.is_some() || rpm.is_some();
    if any_success {
        return LodeTransition::no_change(current, 0);
    }

    let new_count = error_count + 1;
    let new_state = if new_count >= error_threshold {
        LodeState::Error
    } else {
        current
    };
    LodeTransition {
        new_state,
        new_error_count: new_count,
        notify: LodeNotify::default(),
    }
}

/// Tick while in `Error`: reset to `Disconnected`, clear error count,
/// notify stopped. From any other state: no-op.
#[must_use]
pub fn on_error_tick(current: LodeState) -> LodeTransition {
    if !matches!(current, LodeState::Error) {
        return LodeTransition::no_change(current, 0);
    }
    LodeTransition {
        new_state: LodeState::Disconnected,
        new_error_count: 0,
        notify: LodeNotify {
            started: false,
            stopped: true,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const THRESH: u32 = 3;

    // ---- is_bike_connected --------------------------------------------------

    #[test]
    fn is_bike_connected_only_polling() {
        assert!(!is_bike_connected(LodeState::Disconnected));
        assert!(is_bike_connected(LodeState::Polling));
        assert!(!is_bike_connected(LodeState::Error));
    }

    // ---- timing guards ------------------------------------------------------

    #[test]
    fn should_poll_after_interval() {
        assert!(!should_poll(1000, 1000, 500));
        assert!(!should_poll(1000, 1499, 500));
        assert!(should_poll(1000, 1500, 500));
        assert!(should_poll(1000, 9999, 500));
    }

    #[test]
    fn should_poll_handles_millis_wrap() {
        // 49.7-day boundary: last near u32::MAX, now at 0 post-wrap.
        // Unsigned wrapping_sub means elapsed = 101.
        let near_max = u32::MAX - 100;
        assert!(should_poll(near_max, 0, 100));
        assert!(should_poll(near_max, 0, 101));
        assert!(!should_poll(near_max, 0, 102));
    }

    #[test]
    fn should_reconnect_basic() {
        assert!(!should_reconnect(0, 1999, 2000));
        assert!(should_reconnect(0, 2000, 2000));
    }

    // ---- on_version_ok -------------------------------------------------------

    #[test]
    fn version_ok_from_disconnected() {
        let t = on_version_ok(LodeState::Disconnected, 5);
        assert_eq!(t.new_state, LodeState::Polling);
        assert_eq!(t.new_error_count, 0);
        assert!(t.notify.started);
        assert!(!t.notify.stopped);
    }

    #[test]
    fn version_ok_from_polling_noop() {
        let t = on_version_ok(LodeState::Polling, 0);
        assert_eq!(t.new_state, LodeState::Polling);
        assert_eq!(t.new_error_count, 0);
        assert!(!t.notify.started);
    }

    #[test]
    fn version_ok_from_error_noop() {
        // Must tick ERROR → DISCONNECTED first before any version check.
        let t = on_version_ok(LodeState::Error, 2);
        assert_eq!(t.new_state, LodeState::Error);
        assert_eq!(t.new_error_count, 2);
        assert!(!t.notify.started);
    }

    // ---- on_poll_result: error counting --------------------------------------

    #[test]
    fn both_succeed_resets_error_count() {
        let t = on_poll_result(LodeState::Polling, 2, Some(150), Some(75), THRESH);
        assert_eq!(t.new_state, LodeState::Polling);
        assert_eq!(t.new_error_count, 0);
    }

    #[test]
    fn only_watts_succeeds_resets() {
        let t = on_poll_result(LodeState::Polling, 2, Some(150), None, THRESH);
        assert_eq!(t.new_error_count, 0);
    }

    #[test]
    fn only_rpm_succeeds_resets() {
        let t = on_poll_result(LodeState::Polling, 2, None, Some(75), THRESH);
        assert_eq!(t.new_error_count, 0);
    }

    #[test]
    fn both_fail_increments_by_one() {
        let t = on_poll_result(LodeState::Polling, 0, None, None, THRESH);
        assert_eq!(t.new_state, LodeState::Polling);
        assert_eq!(t.new_error_count, 1);

        let t = on_poll_result(LodeState::Polling, 1, None, None, THRESH);
        assert_eq!(t.new_error_count, 2);
    }

    #[test]
    fn both_fail_at_threshold_transitions_to_error() {
        let t = on_poll_result(LodeState::Polling, 2, None, None, THRESH);
        assert_eq!(t.new_state, LodeState::Error);
        assert_eq!(t.new_error_count, 3);
    }

    #[test]
    fn rpm_zero_is_a_valid_reading() {
        let t = on_poll_result(LodeState::Polling, 1, Some(100), Some(0), THRESH);
        assert_eq!(t.new_error_count, 0);
    }

    #[test]
    fn watts_zero_is_a_valid_reading() {
        let t = on_poll_result(LodeState::Polling, 1, Some(0), Some(60), THRESH);
        assert_eq!(t.new_error_count, 0);
    }

    #[test]
    fn poll_result_in_non_polling_state_is_noop() {
        let t = on_poll_result(LodeState::Disconnected, 5, None, None, THRESH);
        assert_eq!(t.new_state, LodeState::Disconnected);
        assert_eq!(t.new_error_count, 5);

        let t = on_poll_result(LodeState::Error, 5, Some(100), Some(60), THRESH);
        assert_eq!(t.new_state, LodeState::Error);
        assert_eq!(t.new_error_count, 5);
    }

    #[test]
    fn near_threshold_single_success_resets() {
        let t = on_poll_result(LodeState::Polling, THRESH - 1, Some(100), None, THRESH);
        assert_eq!(t.new_state, LodeState::Polling);
        assert_eq!(t.new_error_count, 0);
    }

    #[test]
    fn consecutive_full_failures_climb_to_threshold() {
        let mut s = LodeState::Polling;
        let mut ec = 0u32;
        for i in 1..=THRESH {
            let t = on_poll_result(s, ec, None, None, THRESH);
            s = t.new_state;
            ec = t.new_error_count;
            assert_eq!(ec, i);
        }
        assert_eq!(s, LodeState::Error);
    }

    // ---- on_error_tick -------------------------------------------------------

    #[test]
    fn error_tick_from_error_resets_and_notifies_stopped() {
        let t = on_error_tick(LodeState::Error);
        assert_eq!(t.new_state, LodeState::Disconnected);
        assert_eq!(t.new_error_count, 0);
        assert!(t.notify.stopped);
        assert!(!t.notify.started);
    }

    #[test]
    fn error_tick_from_other_states_noop() {
        assert_eq!(
            on_error_tick(LodeState::Disconnected).new_state,
            LodeState::Disconnected
        );
        assert_eq!(
            on_error_tick(LodeState::Polling).new_state,
            LodeState::Polling
        );
    }

    // ---- End-to-end lifecycle ------------------------------------------------

    #[test]
    fn full_lifecycle_connect_degrade_recover() {
        let mut s = LodeState::Disconnected;
        let mut ec = 0u32;

        let t = on_version_ok(s, ec);
        s = t.new_state;
        ec = t.new_error_count;
        assert_eq!(s, LodeState::Polling);
        assert!(t.notify.started);

        let t = on_poll_result(s, ec, Some(150), Some(75), THRESH);
        s = t.new_state;
        ec = t.new_error_count;
        assert_eq!(ec, 0);

        for _ in 0..THRESH {
            let t = on_poll_result(s, ec, None, None, THRESH);
            s = t.new_state;
            ec = t.new_error_count;
        }
        assert_eq!(s, LodeState::Error);

        let t = on_error_tick(s);
        s = t.new_state;
        ec = t.new_error_count;
        assert_eq!(s, LodeState::Disconnected);
        assert_eq!(ec, 0);
        assert!(t.notify.stopped);

        let t = on_version_ok(s, ec);
        s = t.new_state;
        assert_eq!(s, LodeState::Polling);
        assert!(t.notify.started);
    }
}
