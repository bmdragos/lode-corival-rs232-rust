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
| `lode_parser` | `lode_parser.{h,cpp}` | in progress |
| `ftms_encoder` | `ftms_encoder.{h,cpp}` | pending |
| `ftms_control_point` | `ftms_control_point.{h,cpp}` | pending |
| `state_machine` | `lode_state_machine.{h,cpp}` | pending |

## Testing

```bash
cargo test
```

Tests run on the host — no ESP32 required. Target is feature parity with the
206-assertion C++ test suite in the origin repo.
