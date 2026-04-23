//! Pure protocol logic for the Lode Corival RS-232 / BLE FTMS bridge.
//!
//! This crate is `no_std` when compiled normally, so the same code links into
//! the ESP32-C6 firmware and into host-side unit tests (which get `std`
//! automatically in the `test` profile).

#![cfg_attr(not(test), no_std)]

pub mod ftms_control_point;
pub mod ftms_encoder;
pub mod lode_parser;
pub mod state_machine;
