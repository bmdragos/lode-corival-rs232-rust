//! Lode FTMS Bridge firmware for ESP32-C6.
//!
//! Main task wires together:
//! - `bike_serial::BikeSerial`  - RS-232 driver on UART1
//! - `ble_server::BleServer`    - NimBLE FTMS peripheral
//! - `lode_protocol::state_machine` - pure transition helpers
//!
//! The BLE callback thread pushes target-power requests into a shared
//! `Option<i16>`; the main loop picks them up, applies them over RS-232,
//! and confirms back via FTMS Status notifications. Poll outcomes feed
//! `on_poll_result` to decide DISCONNECTED/POLLING/ERROR transitions
//! and emit FTMS started/stopped events.

mod bike_serial;
mod ble_server;

use std::{
    thread,
    time::{Duration, Instant},
};

use esp_idf_svc::hal::peripherals::Peripherals;
use lode_protocol::state_machine::{on_error_tick, on_poll_result, on_version_ok, LodeState};

use bike_serial::BikeSerial;
use ble_server::BleServer;

/// How often to poll PM/RM when connected.
const POLL_INTERVAL: Duration = Duration::from_millis(500);
/// How often to attempt reconnect when DISCONNECTED.
const RECONNECT_INTERVAL: Duration = Duration::from_secs(2);
/// Main-loop idle sleep between work checks.
const TICK_INTERVAL: Duration = Duration::from_millis(50);
/// Consecutive full-failure poll cycles before transitioning to ERROR.
const MAX_RETRIES: u32 = 3;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Lode FTMS Bridge v{}", env!("CARGO_PKG_VERSION"));
    log::info!("Target: ESP32-C6 (riscv32imac)");

    let peripherals = Peripherals::take()?;

    // XIAO ESP32-C6: D6 = GPIO16 (TX to bike), D7 = GPIO17 (RX from bike).
    let mut bike = BikeSerial::new(
        peripherals.uart1,
        peripherals.pins.gpio16,
        peripherals.pins.gpio17,
    )
    .map_err(|e| anyhow::anyhow!("UART init failed: {e:?}"))?;
    log::info!("UART1 open @ 9600 8N1 on D6(GPIO16) TX / D7(GPIO17) RX");

    let ble = BleServer::new()?;

    // State machine state. u32 error_count matches the pure-module API.
    let mut state = LodeState::Disconnected;
    let mut error_count = 0u32;

    // Timers. Initialized to "now" so the first poll happens one interval
    // after boot - gives the bike a moment to settle and the BLE stack a
    // moment to get advertising up.
    let mut last_poll = Instant::now();
    let mut last_reconnect = Instant::now();

    loop {
        match state {
            LodeState::Disconnected => {
                if last_reconnect.elapsed() >= RECONNECT_INTERVAL {
                    last_reconnect = Instant::now();
                    log::debug!("Attempting bike connection...");
                    match bike.request_version() {
                        Ok(v) => {
                            log::info!("Bike online, version: {}", v.trim());
                            let t = on_version_ok(state, error_count);
                            state = t.new_state;
                            error_count = t.new_error_count;
                            if t.notify.started {
                                ble.notify_started();
                            }
                        }
                        Err(e) => log::debug!("VR failed: {e:?}"),
                    }
                }
            }

            LodeState::Polling => {
                if last_poll.elapsed() >= POLL_INTERVAL {
                    last_poll = Instant::now();

                    // Apply any pending target, retry on failure.
                    if let Some(target) = ble.take_target() {
                        // Target is clamped by the Control Point handler;
                        // convert to unsigned for the set_load API (bike
                        // doesn't accept negative watts).
                        let watts_u16 = target.max(0) as u16;
                        match bike.set_load(watts_u16) {
                            Ok(()) => {
                                log::info!("SP applied: {target} W");
                                ble.notify_target_confirmed(target);
                            }
                            Err(e) => {
                                log::warn!("SP failed ({e:?}); will retry next tick");
                                ble.requeue_if_empty(target);
                            }
                        }
                    }

                    // Poll watts + rpm. Option<i32> feeds the state machine.
                    let watts = bike.request_load().ok();
                    let rpm = bike.request_rpm().ok();

                    // Push notification to the app for any reading we got.
                    // Use the last-known value for a missing channel by
                    // picking 0 - fine for a single tick, and still
                    // conveys "one channel returned" to the iOS side.
                    if watts.is_some() || rpm.is_some() {
                        let w = watts.unwrap_or(0) as i16;
                        let r = rpm.unwrap_or(0).max(0) as u16;
                        ble.notify_bike_data(w, r);
                    }

                    let t = on_poll_result(state, error_count, watts, rpm, MAX_RETRIES);
                    state = t.new_state;
                    error_count = t.new_error_count;
                }
            }

            LodeState::Error => {
                log::warn!("Bike ERROR state, resetting to DISCONNECTED");
                let t = on_error_tick(state);
                state = t.new_state;
                error_count = t.new_error_count;
                if t.notify.stopped {
                    ble.notify_stopped();
                }
                // Reset the reconnect clock so we try immediately after
                // the error tick, not 2s later.
                last_reconnect = Instant::now() - RECONNECT_INTERVAL;
            }
        }

        thread::sleep(TICK_INTERVAL);
    }
}
