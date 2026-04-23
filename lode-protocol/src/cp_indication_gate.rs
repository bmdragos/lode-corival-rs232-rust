//! One-in-flight gate for FTMS Control Point indication responses.
//!
//! The server sends at most one indication on 0x2AD9 at a time, holding
//! that slot until the client confirms via ATT Handle Value Confirmation.
//! For spec-compliant clients (iOS, Android, nRF Connect) the confirm
//! arrives within milliseconds and `on_confirm` releases the gate. For
//! non-compliant peers (notably macOS CoreBluetooth as central, which
//! simply never sends HVC back to a peripheral) the gate would stay
//! locked indefinitely, silently dropping every subsequent CP response.
//! The gate self-releases after `confirm_timeout` to stay responsive.
//!
//! The gate is pure state + a monotonic clock supplied by the caller.
//! It holds no locks, no `BLECharacteristic`, no `Instant` - so it
//! lives in `lode-protocol` alongside the other host-testable logic.

use core::time::Duration;

use crate::ftms_control_point::FTMS_CP_RESPONSE_SIZE;

pub type CpResponse = [u8; FTMS_CP_RESPONSE_SIZE];

/// Return value of [`CpIndicationGate::poll`].
#[derive(Debug, PartialEq, Eq)]
pub struct PollResult {
    /// The bytes the caller should dispatch as a new indication, if any.
    pub send: Option<CpResponse>,
    /// `true` if this poll released an in-flight gate that had exceeded
    /// `confirm_timeout`. The caller may want to log this — it means a
    /// client failed to confirm the prior indication within the window.
    pub timed_out: bool,
}

pub struct CpIndicationGate {
    pending: Option<CpResponse>,
    in_flight_at_ms: Option<u64>,
    confirm_timeout: Duration,
}

impl CpIndicationGate {
    #[must_use]
    pub const fn new(confirm_timeout: Duration) -> Self {
        Self {
            pending: None,
            in_flight_at_ms: None,
            confirm_timeout,
        }
    }

    /// Queue a response to send. Called from the CP `on_write` callback.
    /// Any prior unsent response is overwritten — FTMS CP is strict
    /// request/response and a client that writes again before receiving
    /// the prior response's indication has forfeited that response.
    pub fn enqueue(&mut self, response: CpResponse) {
        self.pending = Some(response);
    }

    /// Called from `on_notify_tx` when any terminal status fires on the
    /// CP characteristic (success / timeout / failure). Releases the
    /// one-in-flight gate.
    pub fn on_confirm(&mut self) {
        self.in_flight_at_ms = None;
    }

    /// Called from the BLE `on_disconnect` callback. Clears all state
    /// so the next client starts with a clean gate.
    pub fn on_disconnect(&mut self) {
        self.pending = None;
        self.in_flight_at_ms = None;
    }

    /// Discard any queued response without occupying the gate. Use when
    /// the caller knows sending would be a no-op (e.g., no subscribers,
    /// so `.notify()` wouldn't fire `on_notify_tx` and the gate would
    /// stay locked forever). Returns `true` if something was dropped.
    pub fn drop_pending(&mut self) -> bool {
        self.pending.take().is_some()
    }

    /// Should be called every main-loop tick. `now_ms` is a monotonic
    /// millisecond clock (e.g. `Instant::elapsed().as_millis() as u64`).
    pub fn poll(&mut self, now_ms: u64) -> PollResult {
        let mut timed_out = false;

        if let Some(sent_at) = self.in_flight_at_ms {
            let elapsed = now_ms.saturating_sub(sent_at);
            let limit = self.confirm_timeout.as_millis() as u64;
            if elapsed < limit {
                return PollResult {
                    send: None,
                    timed_out: false,
                };
            }
            self.in_flight_at_ms = None;
            timed_out = true;
        }

        match self.pending.take() {
            Some(response) => {
                self.in_flight_at_ms = Some(now_ms);
                PollResult {
                    send: Some(response),
                    timed_out,
                }
            }
            None => PollResult {
                send: None,
                timed_out,
            },
        }
    }

    #[cfg(test)]
    fn is_in_flight(&self) -> bool {
        self.in_flight_at_ms.is_some()
    }

    #[cfg(test)]
    fn has_pending(&self) -> bool {
        self.pending.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TIMEOUT: Duration = Duration::from_millis(2000);
    const RESP_A: CpResponse = [0x80, 0x00, 0x01];
    const RESP_B: CpResponse = [0x80, 0x05, 0x01];

    fn gate() -> CpIndicationGate {
        CpIndicationGate::new(TIMEOUT)
    }

    #[test]
    fn fresh_gate_polls_idle() {
        let mut g = gate();
        assert_eq!(
            g.poll(0),
            PollResult {
                send: None,
                timed_out: false
            }
        );
        assert!(!g.is_in_flight());
    }

    #[test]
    fn enqueue_then_poll_dispatches() {
        let mut g = gate();
        g.enqueue(RESP_A);
        let r = g.poll(100);
        assert_eq!(r.send, Some(RESP_A));
        assert!(!r.timed_out);
        assert!(g.is_in_flight());
        assert!(!g.has_pending());
    }

    #[test]
    fn poll_while_in_flight_returns_idle() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.poll(0);
        let r = g.poll(500);
        assert_eq!(r.send, None);
        assert!(!r.timed_out);
        assert!(g.is_in_flight());
    }

    #[test]
    fn confirm_releases_gate() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.poll(0);
        g.on_confirm();
        assert!(!g.is_in_flight());

        g.enqueue(RESP_B);
        let r = g.poll(100);
        assert_eq!(r.send, Some(RESP_B));
    }

    #[test]
    fn timeout_releases_gate_empty_queue() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.poll(0);
        let r = g.poll(2000);
        assert_eq!(r.send, None);
        assert!(r.timed_out);
        assert!(!g.is_in_flight());
    }

    #[test]
    fn timeout_releases_gate_with_queued_response() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.poll(0);
        // Client sends another write while gate is stuck.
        g.enqueue(RESP_B);
        let r = g.poll(2500);
        assert_eq!(r.send, Some(RESP_B));
        assert!(r.timed_out);
        assert!(g.is_in_flight());
    }

    #[test]
    fn poll_just_under_timeout_still_in_flight() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.poll(1000);
        let r = g.poll(1000 + 1999);
        assert_eq!(r.send, None);
        assert!(!r.timed_out);
        assert!(g.is_in_flight());
    }

    #[test]
    fn poll_exactly_at_timeout_releases() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.poll(0);
        let r = g.poll(TIMEOUT.as_millis() as u64);
        assert!(r.timed_out);
        assert!(!g.is_in_flight());
    }

    #[test]
    fn enqueue_twice_last_write_wins() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.enqueue(RESP_B);
        let r = g.poll(0);
        assert_eq!(r.send, Some(RESP_B));
    }

    #[test]
    fn disconnect_clears_all_state() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.poll(0);
        assert!(g.is_in_flight());

        g.on_disconnect();
        assert!(!g.is_in_flight());
        assert!(!g.has_pending());

        // Fresh state: new enqueue dispatches immediately.
        g.enqueue(RESP_B);
        assert_eq!(g.poll(100).send, Some(RESP_B));
    }

    #[test]
    fn disconnect_with_queued_unsent_drops_it() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.on_disconnect();
        assert!(!g.has_pending());
        assert_eq!(g.poll(0).send, None);
    }

    #[test]
    fn drop_pending_returns_true_when_queued() {
        let mut g = gate();
        g.enqueue(RESP_A);
        assert!(g.drop_pending());
        assert!(!g.has_pending());
        assert_eq!(g.poll(0).send, None);
    }

    #[test]
    fn drop_pending_returns_false_when_empty() {
        let mut g = gate();
        assert!(!g.drop_pending());
    }

    #[test]
    fn drop_pending_does_not_affect_in_flight() {
        let mut g = gate();
        g.enqueue(RESP_A);
        g.poll(0);
        assert!(g.is_in_flight());

        // Arrive a new write, then caller discovers no subscribers.
        g.enqueue(RESP_B);
        assert!(g.drop_pending());

        // In-flight gate still locked until confirm or timeout.
        assert!(g.is_in_flight());
        assert_eq!(g.poll(100).send, None);
    }

    #[test]
    fn saturating_clock_handles_backwards_time() {
        // If a caller passes a now_ms lower than a previous in_flight_at_ms
        // (shouldn't happen with a monotonic clock, but let's not panic),
        // the gate should just treat elapsed as 0.
        let mut g = gate();
        g.enqueue(RESP_A);
        g.poll(10_000);
        let r = g.poll(5_000);
        assert_eq!(r.send, None);
        assert!(!r.timed_out);
    }

    #[test]
    fn write_before_first_poll_then_timeout_still_works() {
        // Caller enqueues and polls after a significant delay; t=0 for
        // in_flight = poll time. Timeout should fire 2s after that.
        let mut g = gate();
        g.enqueue(RESP_A);
        let r = g.poll(5_000_000); // start of in-flight
        assert_eq!(r.send, Some(RESP_A));
        let r = g.poll(5_002_000);
        assert!(r.timed_out);
    }
}
