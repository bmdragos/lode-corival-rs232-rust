//! Lode FTMS Bridge firmware for ESP32-C6.
//!
//! Phase: hello-world linking check. Calls into `lode-protocol` from the
//! main task to prove the pure-logic crate links correctly into an
//! ESP-IDF binary for target.

use std::{thread, time::Duration};

use lode_protocol::{
    ftms_control_point::{handle_ftms_control_point, FtmsCpAction, FTMS_CP_SET_TARGET_POWER},
    ftms_encoder::encode_indoor_bike_data,
    lode_parser::parse_numeric_response,
    state_machine::{is_bike_connected, LodeState},
};

fn main() -> anyhow::Result<()> {
    // esp-idf-svc's default runtime patches. Required even for std main.
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Lode FTMS Bridge v{}", env!("CARGO_PKG_VERSION"));
    log::info!("Target: ESP32-C6 (riscv32imac)");

    // Smoke-test each pure module by exercising one representative call.
    // If any of these fail to link, the build would have already failed -
    // this just produces visible evidence in the serial log that the
    // logic is reachable from the ESP32 task context.
    let parsed = parse_numeric_response("0,150", 0);
    log::info!("parse_numeric_response(\"0,150\", 0) = {parsed:?}");

    let ibd = encode_indoor_bike_data(150, 75);
    log::info!("encode_indoor_bike_data(150, 75) = {ibd:02X?}");

    let cp = handle_ftms_control_point(&[FTMS_CP_SET_TARGET_POWER, 0x96, 0x00], 7, 1000);
    log::info!("handle_ftms_control_point(set_target=150W) = {cp:?}");
    assert!(matches!(
        cp.map(|r| r.action),
        Some(FtmsCpAction::SetTargetPower(150))
    ));

    log::info!(
        "is_bike_connected(Polling) = {}",
        is_bike_connected(LodeState::Polling)
    );

    log::info!("All pure modules reachable. Idling main task.");
    loop {
        thread::sleep(Duration::from_secs(5));
        log::info!("tick");
    }
}
