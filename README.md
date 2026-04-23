# lode-corival-rs232-rust

Rust port of [lode-corival-rs232](https://github.com/brandonj/lode-corival-rs232),
an ESP32-C6 firmware that bridges a Lode cycle ergometer (RS-232) to BLE FTMS for
iOS apps.

## Layout

```
lode-corival-rs232-rust/
├── lode-protocol/              pure logic (no_std, host-testable)
│   └── src/
│       ├── lode_parser.rs          RS-232 response parsing
│       ├── ftms_encoder.rs         Indoor Bike Data 0x2AD2 packing
│       ├── ftms_control_point.rs   Control Point 0x2AD9 dispatch
│       └── state_machine.rs        LodeState transitions + timing
└── firmware/                   ESP32-C6 binary
    ├── Cargo.toml              depends on lode-protocol + esp-idf-svc + esp32-nimble
    ├── .cargo/config.toml      target + ESP-IDF env (see Setup below)
    ├── sdkconfig.defaults      NimBLE peripheral config
    └── src/
        ├── main.rs
        └── bike_serial.rs      UART driver wrapping Lode protocol
```

## Testing

```bash
cargo test                                      # host-side, <1s
cargo clippy --all-targets -- -D warnings       # pedantic lint clean
```

All four pure modules have inline `#[cfg(test)]` blocks that cover the same
cases as the origin repo's C++ doctest suite. No ESP32 required for tests.

## Firmware build

```bash
cargo build --release                           # ~4 min cold, 2-3s incremental
cargo run --release                             # build + espflash + serial monitor
```

## Setup

### One-time prerequisites

```bash
brew install cmake ninja
cargo install espup ldproxy espflash cargo-generate
espup install --targets esp32c6 --std
```

Rosetta 2 is required on Apple Silicon for some arduino-cli tools (not for
Rust itself, but often useful in neighboring ESP tooling):
`softwareupdate --install-rosetta --agree-to-license`

### Known footguns

The `firmware/` crate is a workspace member and talks to ESP-IDF through
`esp-idf-svc`, which imposes three non-obvious setup constraints. All three
are documented in the `firmware/.cargo/config.toml` and `firmware/sdkconfig.defaults`
comments, but summarized here:

1. **`ESP_IDF_VERSION` must be pinned to v5.5.1.** `esp32-nimble` 0.12 tracks
   that specific release; v5.5.3 renamed ~650 NimBLE platform-layer
   functions (e.g. `npl_freertos_hw_exit_critical`) and breaks the build.
   Revisit when esp32-nimble ships a v5.5.3-compatible release.

2. **Wrong-version gcc toolchains linger in `.embuild/`.** If you previously
   built against a different `ESP_IDF_VERSION`, the newer gcc directory may
   be present in `.embuild/espressif/tools/riscv32-esp-elf/` and chosen
   preferentially, triggering a "Tool doesn't match supported version"
   cmake error. Delete the wrong-version dir and wipe
   `target/riscv32imac-esp-espidf/release/build/esp-idf-sys-*/out/build/`
   to clear the cached cmake toolchain paths.

3. **`sdkconfig.defaults` is silently ignored in workspace-member builds.**
   embuild resolves the default path relative to the Cargo workspace root
   (not the firmware crate), so `firmware/sdkconfig.defaults` is never
   picked up unless explicitly specified. The fix lives in
   `firmware/.cargo/config.toml`:
   ```toml
   [env]
   ESP_IDF_SDKCONFIG_DEFAULTS = { value = "sdkconfig.defaults", relative = true }
   ESP_IDF_SYS_ROOT_CRATE = "firmware"
   ```
   Symptom if this is broken: esp32-nimble fails with ~650
   "cannot find function in this scope" errors, because BT is not enabled
   in the generated ESP-IDF config and so bindgen never sees the NimBLE
   headers.

## Progress

| Phase | Status |
| --- | --- |
| Pure-logic modules | done |
| Cargo workspace | done |
| ESP-IDF toolchain + firmware scaffold | done |
| Firmware links pure modules | done |
| `bike_serial` (RS-232 driver) | done |
| NimBLE dep + BT sdkconfig | done |
| `ble_server` (FTMS GATT server) | next |
| State machine integration | pending |
| Flash + bench test against real bike | pending |
