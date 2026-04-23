//! Lode FTMS Bridge firmware for ESP32-C6.
//!
//! Main task wires together:
//! - `bike_serial::BikeSerial`   - RS-232 driver on UART1
//! - `ble_server::BleServer`     - NimBLE FTMS peripheral
//! - `status_led::StatusLed`     - GPIO15 user LED
//! - `lode_protocol::state_machine` - pure transition helpers
//!
//! The BLE callback thread pushes target-power requests into a shared
//! `Option<i16>`; the main loop picks them up, applies them over RS-232,
//! and confirms back via FTMS Status notifications. Poll outcomes feed
//! `on_poll_result` to decide DISCONNECTED/POLLING/ERROR transitions
//! and emit FTMS started/stopped events.
//!
//! Build `--features simulation` to skip UART and drive synthetic bike
//! data instead, matching the C++ firmware's SIMULATION_MODE.

mod ble_server;
mod status_led;

#[cfg(not(feature = "simulation"))]
mod bike_serial;

use std::{
    thread,
    time::{Duration, Instant},
};

use esp_idf_svc::hal::peripherals::Peripherals;

use ble_server::BleServer;
use status_led::{LedState, StatusLed};

/// How often to poll PM/RM when connected (or emit synthetic data in sim).
const POLL_INTERVAL: Duration = Duration::from_millis(500);
/// How often to attempt reconnect when DISCONNECTED.
#[cfg(not(feature = "simulation"))]
const RECONNECT_INTERVAL: Duration = Duration::from_secs(2);
/// Main-loop idle sleep between work checks.
const TICK_INTERVAL: Duration = Duration::from_millis(50);
/// Consecutive full-failure poll cycles before transitioning to ERROR.
#[cfg(not(feature = "simulation"))]
const MAX_RETRIES: u32 = 3;

/// LED blink periods matching the C++ firmware's semantics.
const LED_BLINK_ADVERTISING: Duration = Duration::from_millis(1000);
#[cfg(not(feature = "simulation"))]
const LED_BLINK_BIKE_DISCONNECTED: Duration = Duration::from_millis(200);

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Lode FTMS Bridge v{}", env!("CARGO_PKG_VERSION"));
    log::info!("Target: ESP32-C6 (riscv32imac)");

    #[cfg(feature = "simulation")]
    log::warn!("SIMULATION MODE: RS-232 layer skipped, synthetic bike data");

    let peripherals = Peripherals::take()?;

    // XIAO ESP32-C6 user LED - GPIO15, active-low.
    let led = StatusLed::new(peripherals.pins.gpio15)?;

    let ble = BleServer::new()?;

    #[cfg(feature = "simulation")]
    run_sim_loop(&ble, led);

    #[cfg(not(feature = "simulation"))]
    run_real_loop(ble, led, peripherals.uart1, peripherals.pins.gpio16, peripherals.pins.gpio17)
}

// ---------------------------------------------------------------------------
// Real-bike loop
// ---------------------------------------------------------------------------

#[cfg(not(feature = "simulation"))]
fn run_real_loop<TxPin, RxPin, Uart1>(
    ble: BleServer,
    mut led: StatusLed<'_>,
    uart: Uart1,
    tx: TxPin,
    rx: RxPin,
) -> anyhow::Result<()>
where
    TxPin: esp_idf_svc::hal::gpio::OutputPin + 'static,
    RxPin: esp_idf_svc::hal::gpio::InputPin + 'static,
    Uart1: esp_idf_svc::hal::uart::Uart + 'static,
{
    use bike_serial::BikeSerial;
    use lode_protocol::state_machine::{on_error_tick, on_poll_result, on_version_ok, LodeState};

    let mut bike = BikeSerial::new(uart, tx, rx)
        .map_err(|e| anyhow::anyhow!("UART init failed: {e:?}"))?;
    log::info!("UART1 open @ 9600 8N1 on D6(GPIO16) TX / D7(GPIO17) RX");

    let mut state = LodeState::Disconnected;
    let mut error_count = 0u32;
    let mut last_poll = Instant::now();
    let mut last_reconnect = Instant::now();

    loop {
        match state {
            LodeState::Disconnected => {
                if last_reconnect.elapsed() >= RECONNECT_INTERVAL {
                    last_reconnect = Instant::now();
                    log::info!("Attempting bike connection (VR)...");
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
                        Err(e) => log::info!("VR failed: {e:?}"),
                    }
                }
            }

            LodeState::Polling => {
                if last_poll.elapsed() >= POLL_INTERVAL {
                    last_poll = Instant::now();

                    if let Some(target) = ble.take_target() {
                        let watts_u16 =
                            u16::try_from(target.max(0)).expect("i16 >= 0 fits in u16");

                        let apply = bike
                            .ensure_terminal_mode()
                            .and_then(|()| bike.set_load(watts_u16));

                        match apply {
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

                    let watts = bike.request_load().ok();
                    let rpm = bike.request_rpm().ok();

                    if watts.is_some() || rpm.is_some() {
                        let w = i16::try_from(watts.unwrap_or(0)).unwrap_or_else(|_| {
                            log::warn!("watts out of i16 range: {watts:?}");
                            0
                        });
                        let r = u16::try_from(rpm.unwrap_or(0).max(0)).unwrap_or_else(|_| {
                            log::warn!("rpm out of u16 range: {rpm:?}");
                            0
                        });
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
                last_reconnect = Instant::now()
                    .checked_sub(RECONNECT_INTERVAL)
                    .unwrap_or_else(Instant::now);
            }
        }

        // LED state: connected > advertising > bike-disconnected
        led.set(if ble.is_client_connected() {
            LedState::SolidOn
        } else if matches!(state, LodeState::Polling) {
            LedState::Blink(LED_BLINK_ADVERTISING)
        } else {
            LedState::Blink(LED_BLINK_BIKE_DISCONNECTED)
        });
        led.tick();

        thread::sleep(TICK_INTERVAL);
    }
}

// ---------------------------------------------------------------------------
// Simulation loop
// ---------------------------------------------------------------------------

#[cfg(feature = "simulation")]
fn run_sim_loop(ble: &BleServer, mut led: StatusLed<'_>) -> ! {
    use ble_server::{MAX_POWER_WATTS, MIN_POWER_WATTS};

    // Starting values mirror the C++ firmware's simWatts=100, simRPM=75.
    let mut sim_watts: i16 = 100;
    let sim_rpm: u16 = 75;
    let mut tick: u32 = 0;

    ble.notify_started();
    log::info!("SIM: emitting bike data every {} ms", POLL_INTERVAL.as_millis());

    let mut last_emit = Instant::now();
    loop {
        if last_emit.elapsed() >= POLL_INTERVAL {
            last_emit = Instant::now();
            tick = tick.wrapping_add(1);

            if let Some(target) = ble.take_target() {
                sim_watts = target.clamp(MIN_POWER_WATTS, MAX_POWER_WATTS);
                log::info!("SIM: target applied: {sim_watts} W");
                ble.notify_target_confirmed(sim_watts);
            }

            // Tiny deterministic pseudo-noise so app graphs look alive.
            // Range: -4..=3 W via a cheap LCG-like hash of the tick counter.
            let noise = (tick.wrapping_mul(2_654_435_769) >> 29) as i16 - 4;
            let displayed_watts = sim_watts.saturating_add(noise);

            ble.notify_bike_data(displayed_watts, sim_rpm);
            log::debug!("SIM: notified watts={displayed_watts} rpm={sim_rpm}");
        }

        // LED: SolidOn when a client is attached, slow-blink while we wait.
        led.set(if ble.is_client_connected() {
            LedState::SolidOn
        } else {
            LedState::Blink(LED_BLINK_ADVERTISING)
        });
        led.tick();

        thread::sleep(TICK_INTERVAL);
    }
}
