//! NimBLE FTMS (Fitness Machine Service) peripheral.
//!
//! Registers the 0x1826 service with its four characteristics and
//! translates the Control Point write traffic through
//! `lode_protocol::ftms_control_point::handle_ftms_control_point`, so
//! the byte-level dispatch logic is shared with the host-tested C++
//! port suite.
//!
//! Thread-safety: esp32-nimble calls characteristic write callbacks from
//! the BLE host task (not the main task). The `Arc<Mutex<Option<i16>>>`
//! target-power channel is the handoff: the callback stores a requested
//! watts value; the main poll loop reads it via [`BleServer::take_target`]
//! and applies it to the bike under its own lock.

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

use esp32_nimble::{
    utilities::{mutex::Mutex as NimbleMutex, BleUuid},
    BLEAdvertisementData, BLECharacteristic, BLEDevice, NimbleProperties, NotifyTxStatus,
};

use lode_protocol::{
    cp_indication_gate::CpIndicationGate,
    ftms_control_point::{handle_ftms_control_point, FtmsCpAction, FTMS_CP_RESPONSE_SIZE},
    ftms_encoder::encode_indoor_bike_data,
};

// ---- FTMS UUIDs (Bluetooth SIG assigned) --------------------------------

const FTMS_SERVICE_UUID: BleUuid = BleUuid::from_uuid16(0x1826);
const FTMS_FEATURE_UUID: BleUuid = BleUuid::from_uuid16(0x2ACC);
const FTMS_INDOOR_BIKE_DATA_UUID: BleUuid = BleUuid::from_uuid16(0x2AD2);
const FTMS_CONTROL_POINT_UUID: BleUuid = BleUuid::from_uuid16(0x2AD9);
const FTMS_STATUS_UUID: BleUuid = BleUuid::from_uuid16(0x2ADA);

// ---- FTMS Feature characteristic value ----------------------------------

/// Advertised feature flags, 8 bytes.
///
/// Byte layout per Bluetooth SIG Fitness Machine Feature characteristic:
///   [0..4]  uint32 LE - Fitness Machine Features bitmap
///   [4..8]  uint32 LE - Target Setting Features bitmap
///
/// Bits we set:
///   - Fitness Machine Features bit 2  (0x04): Cadence Supported
///   - Fitness Machine Features bit 14 (0x4000): Power Measurement Supported
///   - Target Setting Features  bit 3  (0x08): Power Target Setting Supported
const FEATURE_BYTES: [u8; 8] = [
    0x04, 0x40, 0x00, 0x00, // cadence + power measurement
    0x08, 0x00, 0x00, 0x00, // power target setting
];

// ---- FTMS Status opcodes (for 0x2ADA notifications) ---------------------

const FTMS_STATUS_STOPPED: u8 = 0x02; // param: 0x01 stop, 0x02 pause
const FTMS_STATUS_STARTED: u8 = 0x04; // no param
const FTMS_STATUS_TARGET_POWER: u8 = 0x08; // param: int16 watts LE

// ---- Power limits (mirror the .ino) ------------------------------------

pub const MIN_POWER_WATTS: i16 = 7;
pub const MAX_POWER_WATTS: i16 = 1000;

/// How long we wait for a client to confirm a CP indication before
/// giving up and releasing the in-flight gate. iOS clients confirm
/// within milliseconds; macOS CoreBluetooth acting as central never
/// confirms at all (see CLAUDE.md). The timeout keeps us responsive
/// when talking to the latter.
const CP_INDICATE_CONFIRM_TIMEOUT: Duration = Duration::from_secs(2);

// ---- Public surface -----------------------------------------------------

pub struct BleServer {
    bike_data_char: Arc<NimbleMutex<BLECharacteristic>>,
    status_char: Arc<NimbleMutex<BLECharacteristic>>,
    cp_char: Arc<NimbleMutex<BLECharacteristic>>,
    /// Queues CP responses from the on_write callback and serializes
    /// their dispatch from the main loop. Pure logic - all state + the
    /// one-in-flight + self-timeout rule lives in `lode-protocol`.
    /// Never touched from within `on_write` directly on `cp_char`
    /// (esp32-nimble 0.12 ble_characteristic.rs:229 forbids re-locking
    /// the characteristic from its own callback).
    cp_gate: Arc<Mutex<CpIndicationGate>>,
    /// Monotonic reference point for the gate's millisecond clock.
    cp_gate_epoch: Instant,
    target: Arc<Mutex<Option<i16>>>,
    client_connected: Arc<AtomicBool>,
}

impl BleServer {
    /// Initialize NimBLE, register the FTMS service, and start advertising
    /// as "Lode Bike". Blocks only briefly - advertising happens on the
    /// NimBLE host task.
    pub fn new() -> anyhow::Result<Self> {
        let device = BLEDevice::take();

        // Set the GAP device name. Without this, esp32-nimble falls back
        // to "nimble" whenever advertising auto-restarts after disconnect
        // (the BLEAdvertisementData.name() we set later only governs the
        // first advertising cycle).
        BLEDevice::set_device_name("Lode Bike")
            .map_err(|e| anyhow::anyhow!("set_device_name: {e:?}"))?;

        // Tracked connection state so the main loop (and status LED) can
        // see whether a BLE client is attached without calling into
        // NimBLE from arbitrary threads.
        let client_connected: Arc<AtomicBool> = Arc::new(AtomicBool::new(false));

        let server = device.get_server();
        let connected_on_connect = Arc::clone(&client_connected);
        server.on_connect(move |server, desc| {
            log::info!("BLE client connected: {desc:?}");
            connected_on_connect.store(true, Ordering::Release);

            // Tighten connection params. NimBLE's defaults leave the
            // supervision timeout long enough (up to ~30s) that macOS/iOS
            // abrupt teardowns don't surface as on_disconnect until well
            // after the peer stopped listening. During that window we
            // stay "connected" from the device's POV and can't advertise
            // again. These values are iOS-friendly (20-40ms interval,
            // 0 latency, 2s supervision timeout).
            //   interval_min/max: 1.25ms units
            //   latency:          intervals skipped
            //   timeout:          10ms units
            if let Err(e) = server.update_conn_params(desc.conn_handle(), 16, 32, 0, 200) {
                log::warn!("update_conn_params failed: {e:?}");
            }
        });

        // CP indication gate - queues responses, serializes dispatch,
        // self-releases on timeout. Pure logic tested in lode-protocol.
        let cp_gate = Arc::new(Mutex::new(CpIndicationGate::new(
            CP_INDICATE_CONFIRM_TIMEOUT,
        )));
        let cp_gate_epoch = Instant::now();

        // Auto-resume advertising on disconnect. Without this, the first
        // client that connects and drops takes advertising down until reboot.
        let advertising_on_disconnect = device.get_advertising();
        let connected_on_disconnect = Arc::clone(&client_connected);
        let cp_gate_on_disconnect = Arc::clone(&cp_gate);
        server.on_disconnect(move |_, reason| {
            log::info!("BLE client disconnected ({reason:?}) - resuming advertising");
            connected_on_disconnect.store(false, Ordering::Release);
            // Any pending/in-flight indication is now moot - reset the
            // gate so the next client starts with a clean slate.
            cp_gate_on_disconnect.lock().unwrap().on_disconnect();
            if let Err(e) = advertising_on_disconnect.lock().start() {
                log::warn!("Failed to restart advertising: {e:?}");
            }
        });

        // Service + characteristics.
        let service = server.create_service(FTMS_SERVICE_UUID);

        let feature_char = service
            .lock()
            .create_characteristic(FTMS_FEATURE_UUID, NimbleProperties::READ);
        feature_char.lock().set_value(&FEATURE_BYTES);

        let bike_data_char = service
            .lock()
            .create_characteristic(FTMS_INDOOR_BIKE_DATA_UUID, NimbleProperties::NOTIFY);

        let cp_char = service.lock().create_characteristic(
            FTMS_CONTROL_POINT_UUID,
            NimbleProperties::WRITE | NimbleProperties::INDICATE,
        );

        let status_char = service
            .lock()
            .create_characteristic(FTMS_STATUS_UUID, NimbleProperties::NOTIFY);

        // Shared target-power channel: BLE callback writes, main loop reads.
        let target: Arc<Mutex<Option<i16>>> = Arc::new(Mutex::new(None));

        // Control Point write handler - queues the framed response into
        // the gate. Does NOT touch cp_char: the crate holds cp_char's
        // mutex across this callback, so any re-lock here deadlocks.
        let target_for_cb = Arc::clone(&target);
        let cp_gate_for_cb = Arc::clone(&cp_gate);
        cp_char.lock().on_write(move |args| {
            let data = args.recv_data();
            log::debug!("CP write: {data:02X?}");

            let Some(result) = handle_ftms_control_point(data, MIN_POWER_WATTS, MAX_POWER_WATTS)
            else {
                return;
            };

            match result.action {
                FtmsCpAction::SetTargetPower(w) => {
                    *target_for_cb.lock().unwrap() = Some(w);
                    log::info!("Target power queued: {w} W");
                }
                FtmsCpAction::Reset => {
                    *target_for_cb.lock().unwrap() = Some(0);
                    log::info!("Reset queued (target power -> 0)");
                }
                FtmsCpAction::RequestControl => log::info!("Client requested control"),
                FtmsCpAction::StartResume => log::info!("Start/Resume requested"),
                FtmsCpAction::StopPause => log::info!("Stop/Pause requested"),
                FtmsCpAction::Noop => {}
            }

            debug_assert_eq!(result.response.len(), FTMS_CP_RESPONSE_SIZE);
            cp_gate_for_cb.lock().unwrap().enqueue(result.response);
        });

        // Notification-TX callback: fires on indication completion. On
        // spec-compliant clients (iOS, Android) this fires with
        // SuccessIndicate within ms. On macOS CoreBluetooth it may never
        // fire until disconnect - the gate's self-timeout covers that.
        let cp_gate_on_tx = Arc::clone(&cp_gate);
        cp_char.lock().on_notify_tx(move |tx| {
            let release = match tx.status() {
                NotifyTxStatus::SuccessIndicate => {
                    log::debug!("CP indication confirmed");
                    true
                }
                NotifyTxStatus::ErrorIndicateTimeout => {
                    log::warn!("CP indication timed out");
                    true
                }
                NotifyTxStatus::ErrorIndicateFailure | NotifyTxStatus::ErrorGatt => {
                    log::warn!("CP indication failed ({:?})", tx.status());
                    true
                }
                other => {
                    log::debug!("CP notify_tx: {other:?}");
                    false
                }
            };
            if release {
                cp_gate_on_tx.lock().unwrap().on_confirm();
            }
        });

        // Advertise as "Lode Bike" with the FTMS service UUID visible in
        // scan results so apps filtering on 0x1826 see us immediately.
        let advertising = device.get_advertising();
        advertising.lock().set_data(
            BLEAdvertisementData::new()
                .name("Lode Bike")
                .add_service_uuid(FTMS_SERVICE_UUID),
        )?;
        advertising.lock().start()?;

        log::info!("BLE advertising as \"Lode Bike\" with FTMS service");

        Ok(Self {
            bike_data_char,
            status_char,
            cp_char,
            cp_gate,
            cp_gate_epoch,
            target,
            client_connected,
        })
    }

    /// Is a BLE client currently attached? Driven by the on_connect /
    /// on_disconnect callbacks. Consumed by the status LED.
    #[must_use]
    pub fn is_client_connected(&self) -> bool {
        self.client_connected.load(Ordering::Acquire)
    }

    /// Take any pending target-power request written by the Control Point
    /// callback since the last call. Returns `None` if no new request.
    #[must_use]
    pub fn take_target(&self) -> Option<i16> {
        self.target.lock().unwrap().take()
    }

    /// Re-queue a target that the main loop just tried to apply and the
    /// bike rejected (RS-232 NAK, timeout, etc.). Does nothing if a newer
    /// request has already been written by the BLE callback - the newer
    /// value wins, which matches the C++ firmware's intent.
    #[cfg_attr(feature = "simulation", allow(dead_code))]
    pub fn requeue_if_empty(&self, watts: i16) {
        let mut guard = self.target.lock().unwrap();
        if guard.is_none() {
            *guard = Some(watts);
        }
    }

    /// Dispatch a queued Control Point response as an indication. The
    /// main loop should call this every tick; it's cheap when the queue
    /// is empty or an indication is still in flight.
    pub fn drain_cp_response(&self) {
        let mut cp = self.cp_char.lock();
        let mut gate = self.cp_gate.lock().unwrap();

        // No subscribers means .notify() is a no-op and on_notify_tx
        // would never fire - which would pin the gate forever. Drop
        // any pending response without locking the gate.
        if cp.subscribed_count() == 0 {
            if gate.drop_pending() {
                log::debug!("CP response dropped: no subscribers");
            }
            return;
        }

        let now_ms = self.cp_gate_epoch.elapsed().as_millis() as u64;
        let result = gate.poll(now_ms);
        if result.timed_out {
            log::warn!(
                "CP indication unconfirmed after {:?}; releasing gate",
                CP_INDICATE_CONFIRM_TIMEOUT
            );
        }
        if let Some(bytes) = result.send {
            cp.set_value(&bytes).notify();
        }
    }

    /// Push a fresh Indoor Bike Data notification. Called once per poll
    /// cycle from the main loop.
    pub fn notify_bike_data(&self, watts: i16, rpm: u16) {
        let bytes = encode_indoor_bike_data(watts, rpm);
        self.bike_data_char.lock().set_value(&bytes).notify();
    }

    /// FTMS Status: bike connected.
    pub fn notify_started(&self) {
        self.status_char
            .lock()
            .set_value(&[FTMS_STATUS_STARTED])
            .notify();
    }

    /// FTMS Status: bike disconnected / stopped. Param 0x01 = stop.
    #[cfg_attr(feature = "simulation", allow(dead_code))]
    pub fn notify_stopped(&self) {
        self.status_char
            .lock()
            .set_value(&[FTMS_STATUS_STOPPED, 0x01])
            .notify();
    }

    /// FTMS Status: target power was applied on the bike (acknowledgement
    /// to the app that the SP command succeeded).
    pub fn notify_target_confirmed(&self, watts: i16) {
        let w = watts.to_le_bytes();
        self.status_char
            .lock()
            .set_value(&[FTMS_STATUS_TARGET_POWER, w[0], w[1]])
            .notify();
    }
}
