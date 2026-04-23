//! Lode FTMS Bridge firmware for ESP32-C6.
//!
//! Phase 6: RS-232 driver integrated. The main task opens UART1 on
//! D6 (TX) / D7 (RX), sends Lode commands at `POLL_INTERVAL`, and logs
//! each round trip. Without a bike plugged in, requests will time out -
//! that's the expected behavior and what the state machine will key on
//! in the next phase.

mod bike_serial;

use std::{thread, time::Duration};

use esp_idf_svc::hal::peripherals::Peripherals;

use bike_serial::{BikeError, BikeSerial};

const POLL_INTERVAL: Duration = Duration::from_millis(500);

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Lode FTMS Bridge v{}", env!("CARGO_PKG_VERSION"));
    log::info!("Target: ESP32-C6 (riscv32imac)");

    let peripherals = Peripherals::take()?;

    // XIAO ESP32-C6: D6 = GPIO16 (TX to bike), D7 = GPIO17 (RX from bike).
    // Matches the wiring documented in the .ino (MAX3232 TX <- D7, RX <- D6).
    let mut bike = BikeSerial::new(
        peripherals.uart1,
        peripherals.pins.gpio16,
        peripherals.pins.gpio17,
    )
    .map_err(|e| anyhow::anyhow!("UART init failed: {e:?}"))?;

    log::info!("UART1 open @ 9600 8N1 on D6(GPIO16) TX / D7(GPIO17) RX");

    // Attempt version exchange once at startup.
    match bike.request_version() {
        Ok(v) => log::info!("Bike version: {v}"),
        Err(e) => log::warn!("VR request failed ({e:?}) - bike may be offline"),
    }

    loop {
        match poll_once(&mut bike) {
            Ok((watts, rpm)) => log::info!("PM = {watts}W, RM = {rpm} rpm"),
            Err(e) => log::warn!("poll failed: {e:?}"),
        }
        thread::sleep(POLL_INTERVAL);
    }
}

fn poll_once(bike: &mut BikeSerial) -> Result<(i32, i32), BikeError> {
    let watts = bike.request_load()?;
    let rpm = bike.request_rpm()?;
    Ok((watts, rpm))
}
