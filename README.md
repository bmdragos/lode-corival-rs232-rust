# lode-corival-rs232-rust

Rust port of [lode-corival-rs232](https://github.com/brandonj/lode-corival-rs232),
an ESP32-C6 firmware that bridges a Lode cycle ergometer (RS-232) to BLE FTMS for
iOS apps.

## Status

Pure-logic port in progress. Hardware layer (ESP-IDF, NimBLE) will follow once
the protocol modules are translated and tested.

## Pure modules (host-testable)

| Module | C++ source | Status |
| --- | --- | --- |
| `lode_parser` | `lode_parser.{h,cpp}` | ported |
| `ftms_encoder` | `ftms_encoder.{h,cpp}` | ported |
| `ftms_control_point` | `ftms_control_point.{h,cpp}` | ported |
| `state_machine` | `lode_state_machine.{h,cpp}` | ported |

## Testing

```bash
cargo test
cargo clippy --all-targets -- -D warnings
```

Tests run on the host — no ESP32 required. All four pure modules have
inline `#[cfg(test)]` blocks that cover the same cases as the origin repo's
C++ doctest suite. Crate is `no_std` when not compiled for tests.

## Hardware layer (not yet started)

Target stack:
- `esp-idf-svc` (base: UART, NVS, logging)
- `esp-idf-hal` (pin configuration, `UartDriver` for RS-232)
- `esp32-nimble` (BLE peripheral, FTMS GATT server)
