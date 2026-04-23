//! User LED on the XIAO ESP32-C6 (GPIO15, active-low).
//!
//! Three display states, matching the C++ firmware:
//! - `SolidOn`   - BLE client connected
//! - `Blink(period)` - with period=1s for advertising, 200ms for error
//! - `Off`       - inactive

use std::time::{Duration, Instant};

use esp_idf_svc::{
    hal::gpio::{Output, OutputPin, PinDriver},
    sys::EspError,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LedState {
    SolidOn,
    Blink(Duration),
    Off,
}

// PinDriver's type parameter is just the MODE (Output here); the pin
// itself is erased to AnyIOPin internally, so StatusLed doesn't need to
// carry a pin-type generic.
pub struct StatusLed<'d> {
    pin: PinDriver<'d, Output>,
    last_toggle: Instant,
    is_on: bool,
    current: LedState,
}

impl<'d> StatusLed<'d> {
    /// Take the LED pin and initialize it as off. Active-low wiring means
    /// HIGH = off, LOW = on.
    pub fn new<P: OutputPin + 'd>(pin: P) -> Result<Self, EspError> {
        let mut driver = PinDriver::output(pin)?;
        driver.set_high()?;
        Ok(Self {
            pin: driver,
            last_toggle: Instant::now(),
            is_on: false,
            current: LedState::Off,
        })
    }

    /// Set the target state. The actual pin update happens on the next
    /// `tick()`. Idempotent when the state is unchanged.
    pub fn set(&mut self, state: LedState) {
        if self.current != state {
            self.current = state;
            // Reset the blink phase so the new state lights up immediately.
            self.last_toggle = Instant::now();
        }
    }

    /// Drive the pin according to the target state. Call this from the
    /// main loop at tick cadence (roughly 50ms). For SolidOn / Off the
    /// pin is set directly; for Blink we toggle every period/2.
    pub fn tick(&mut self) {
        let want_on = match self.current {
            LedState::SolidOn => true,
            LedState::Off => false,
            LedState::Blink(period) => {
                let half = period / 2;
                if self.last_toggle.elapsed() >= half {
                    self.last_toggle = Instant::now();
                    !self.is_on
                } else {
                    // Not time to toggle yet; keep the pin where it is.
                    return;
                }
            }
        };
        if want_on != self.is_on {
            self.is_on = want_on;
            // active-low: LOW = LED on
            let _ = if want_on {
                self.pin.set_low()
            } else {
                self.pin.set_high()
            };
        }
    }
}
