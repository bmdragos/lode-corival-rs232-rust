//! Lode RS-232 protocol driver over ESP-IDF UART.
//!
//! Wraps `esp-idf-svc`'s UART driver with the Lode-specific framing:
//! `"<device>,<command>\r"`. Uses [`lode_protocol`] for response parsing
//! so the byte-level validation logic is shared with the C++ port's
//! test suite (and is host-testable via `cargo test -p lode-protocol`).
//!
//! Ported from the `.ino`'s `requestLoadInternal` / `requestRPMInternal` /
//! `setLoadInternal` / `ensureTerminalMode` family.

use std::{
    thread,
    time::{Duration, Instant},
};

use esp_idf_svc::{
    hal::{
        gpio::{AnyIOPin, InputPin, OutputPin},
        uart::{config, Uart, UartDriver},
        units::Hertz,
    },
    sys::EspError,
};

use lode_protocol::lode_parser::{parse_numeric_response, ParseError};

/// Device number in the Lode address. Configurable 0-99 per spec.
/// The hardware in hand responds as device 1 on VR, so we address and
/// parse for 1. If a future unit uses a different address, update here
/// (or lift to a runtime config / sdkconfig entry).
pub const LODE_DEVICE_NUM: u8 = 1;

/// Bike status code for "Terminal mode" (the only mode that accepts SP).
/// Values 0-7 are menu screens on the programmable control units.
pub const LODE_STATUS_TERMINAL: i32 = 8;

const MIN_COMMAND_INTERVAL: Duration = Duration::from_millis(60);
const RESPONSE_START_TIMEOUT: Duration = Duration::from_millis(50);
const RESPONSE_COMPLETE_TIMEOUT: Duration = Duration::from_millis(100);

const RESPONSE_BUFFER_SIZE: usize = 64;

/// ACK from bike on `SP` / `TR` (matches Lode protocol spec).
/// Any other byte (including 0x15 NAK) is treated as failure.
const ACK: u8 = 0x06;

// The inner fields of Io(EspError) and Parse(ParseError) are only
// observed through BikeError's Debug impl in the logging paths; the
// dead-code lint doesn't consider Debug a "read". Allow the lint
// rather than renaming fields.
#[allow(dead_code)]
#[derive(Debug)]
pub enum BikeError {
    /// Low-level ESP-IDF UART error.
    Io(EspError),
    /// No response (or no CR) within the deadline.
    Timeout,
    /// Response parsing failed (device prefix mismatch, non-numeric, etc).
    Parse(ParseError),
    /// Bike returned NAK or an unexpected byte where ACK was expected.
    Nak,
    /// Response exceeded [`RESPONSE_BUFFER_SIZE`] without a CR.
    BufferOverflow,
    /// Response bytes were not valid UTF-8 (Lode protocol is ASCII).
    InvalidString,
}

impl From<EspError> for BikeError {
    fn from(e: EspError) -> Self {
        Self::Io(e)
    }
}

impl From<ParseError> for BikeError {
    fn from(e: ParseError) -> Self {
        Self::Parse(e)
    }
}

pub struct BikeSerial<'d> {
    uart: UartDriver<'d>,
    last_command: Option<Instant>,
    buf: [u8; RESPONSE_BUFFER_SIZE],
}

impl<'d> BikeSerial<'d> {
    /// Open UART at 9600 8N1 on the given pins. Pass `peripherals.uart1`
    /// and the XIAO ESP32-C6 D6/D7 pins (= GPIO16 TX, GPIO17 RX) for
    /// the intended wiring.
    pub fn new<UART: Uart + 'd>(
        uart: UART,
        tx: impl OutputPin + 'd,
        rx: impl InputPin + 'd,
    ) -> Result<Self, BikeError> {
        let config = config::Config::new()
            .baudrate(Hertz(9600))
            .data_bits(config::DataBits::DataBits8)
            .parity_none()
            .stop_bits(config::StopBits::STOP1);

        let uart = UartDriver::new(
            uart,
            tx,
            rx,
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &config,
        )?;

        Ok(Self {
            uart,
            last_command: None,
            buf: [0; RESPONSE_BUFFER_SIZE],
        })
    }

    // ---- public command surface --------------------------------------

    pub fn request_version(&mut self) -> Result<String, BikeError> {
        self.send_command("VR")?;
        let bytes = self.read_response()?;
        core::str::from_utf8(bytes)
            .map(String::from)
            .map_err(|_| BikeError::InvalidString)
    }

    pub fn request_load(&mut self) -> Result<i32, BikeError> {
        self.send_command("PM")?;
        self.read_parsed_response()
    }

    pub fn request_rpm(&mut self) -> Result<i32, BikeError> {
        self.send_command("RM")?;
        self.read_parsed_response()
    }

    pub fn request_status(&mut self) -> Result<i32, BikeError> {
        self.send_command("RS")?;
        self.read_parsed_response()
    }

    pub fn set_load(&mut self, watts: u16) -> Result<(), BikeError> {
        let cmd = format!("{LODE_DEVICE_NUM},SP{watts}\r");
        self.send_raw(cmd.as_bytes())?;
        self.read_ack()
    }

    /// Query the bike's status; if not already in terminal mode, send
    /// `TR` to switch. Idempotent - safe to call before every `set_load`.
    ///
    /// The Standard Control Unit (type 20, the common Corival config)
    /// is always in terminal mode, so this short-circuits. Programmable
    /// units (types 21-22) may be in a menu screen, in which case `TR`
    /// brings them back.
    pub fn ensure_terminal_mode(&mut self) -> Result<(), BikeError> {
        match self.request_status() {
            Ok(s) if s == LODE_STATUS_TERMINAL => Ok(()),
            Ok(s) => {
                log::info!("Bike in status {s}, switching to terminal mode");
                self.send_command("TR")?;
                match self.read_ack() {
                    Ok(()) => Ok(()),
                    Err(e) => {
                        log::warn!("TR NAK/err: {e:?}");
                        Err(e)
                    }
                }
            }
            Err(e) => {
                log::warn!("RS request failed: {e:?}");
                Err(e)
            }
        }
    }

    // ---- internals ---------------------------------------------------

    /// Respect the 60ms min-command-interval per Lode protocol spec.
    /// Sleeps the current task if the last command was too recent.
    fn pace(&self) {
        if let Some(last) = self.last_command {
            // checked_sub returns None when enough time has already passed.
            if let Some(remaining) = MIN_COMMAND_INTERVAL.checked_sub(last.elapsed()) {
                thread::sleep(remaining);
            }
        }
    }

    /// Drain any stale bytes from the RX FIFO before sending a new command.
    /// Matches the C++ `while (LodeSerial.available()) LodeSerial.read();`.
    fn drain_rx(&mut self) {
        let mut discard = [0u8; 32];
        // Non-blocking reads (0 tick timeout) until the FIFO is empty.
        while matches!(self.uart.read(&mut discard, 0), Ok(n) if n > 0) {}
    }

    fn send_raw(&mut self, bytes: &[u8]) -> Result<(), BikeError> {
        self.pace();
        self.drain_rx();
        self.uart.write(bytes)?;
        self.uart.wait_tx_done(esp_idf_svc::hal::delay::BLOCK)?;
        self.last_command = Some(Instant::now());
        Ok(())
    }

    /// Send a Lode command by framing it as `"<device>,<cmd>\r"`.
    fn send_command(&mut self, cmd: &str) -> Result<(), BikeError> {
        let framed = format!("{LODE_DEVICE_NUM},{cmd}\r");
        self.send_raw(framed.as_bytes())
    }

    /// Read until CR or timeout. Returns the response body without the CR.
    fn read_response(&mut self) -> Result<&[u8], BikeError> {
        let start = Instant::now();
        let mut len = 0usize;
        let mut saw_first = false;

        loop {
            let elapsed = start.elapsed();
            if !saw_first && elapsed > RESPONSE_START_TIMEOUT {
                return Err(BikeError::Timeout);
            }
            if elapsed > RESPONSE_COMPLETE_TIMEOUT {
                return Err(BikeError::Timeout);
            }

            let mut byte = [0u8; 1];
            let n = self.uart.read(&mut byte, 1).unwrap_or(0);
            if n == 0 {
                // No data yet - yield briefly rather than spin.
                thread::sleep(Duration::from_millis(1));
                continue;
            }
            saw_first = true;

            if byte[0] == b'\r' {
                return Ok(&self.buf[..len]);
            }
            if len >= self.buf.len() {
                return Err(BikeError::BufferOverflow);
            }
            self.buf[len] = byte[0];
            len += 1;
        }
    }

    /// Read a response and parse it as a numeric value via `lode_protocol`.
    fn read_parsed_response(&mut self) -> Result<i32, BikeError> {
        let bytes = self.read_response()?;
        let s = core::str::from_utf8(bytes).map_err(|_| BikeError::InvalidString)?;
        Ok(parse_numeric_response(s, LODE_DEVICE_NUM)?)
    }

    /// Read a framed ACK response per the Lode protocol.
    ///
    /// Format: `"<device>,<ACK_byte>\r"` (CR is stripped by read_response).
    /// Check the byte after the comma - 0x06 means success, anything else
    /// (including 0x15 NAK and stray bytes) is treated as failure.
    ///
    /// Note: set commands (SP, ST, TR, MM) all use this framing; the bare
    /// 0x06/0x15 single-byte form some older docs suggest is not what the
    /// Corival V6.02 firmware actually emits.
    fn read_ack(&mut self) -> Result<(), BikeError> {
        let bytes = self.read_response()?;
        match bytes
            .iter()
            .position(|&b| b == b',')
            .and_then(|i| bytes.get(i + 1))
        {
            Some(&ACK) => Ok(()),
            _ => Err(BikeError::Nak),
        }
    }
}
