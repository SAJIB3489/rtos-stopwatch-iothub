# RTOS Stopwatch System / RTOS Timer System


ESP32‑S3 stopwatch with ultrasonic start/stop trigger, SSD1306 OLED, tri‑color LEDs, and push button. Built with FreeRTOS (tasks, queues, mutexes, ISR). Each completed “job” (timing run) is published to Azure IoT Hub and optionally appended to Google Sheets in near real time.

Demo‑friendly: plug in, watch the OLED and LEDs, see events in Azure, and rows appear in a spreadsheet.

---

## 0. Prerequisite
- Hardware
  - ESP32‑S3 development board (3.3 V logic)
  - HC‑SR04 (or compatible) ultrasonic distance sensor
  - SSD1306 128×64 I2C OLED
  - 3x LEDs (RED, YELLOW, GREEN) + 3 resistors (~220–1kΩ)
  - 1x momentary push button
  - Breadboard and jumper wires
  - Optional: 5V→3.3V level shifting for ultrasonic ECHO (if sensor is 5V)
- Software
  - Arduino IDE 2.x (or PlatformIO) with ESP32 board support
  - Libraries: Adafruit SSD1306, Adafruit GFX, PubSubClient
  - Azure CLI (for monitoring): `az extension add -n azure-iot`
  - curl (for quick Google Apps Script test)
- Accounts/Cloud
  - Azure subscription + IoT Hub
  - Google account (Google Sheets + Apps Script)

---

## 1. Hardware list
- ESP32‑S3 dev board
- Ultrasonic sensor (HC‑SR04)
- SSD1306 OLED 128×64, I2C
- LEDs: RED, YELLOW, GREEN
- Resistors: 3 × 220–1kΩ for LEDs
- Push button
- Optional: 2 resistors for ECHO level divider (e.g., 1 kΩ + 2 kΩ) or level shifter
- USB cable

---

## 2. Hardware connection
Pin map (as used by the firmware):
- OLED (I2C)
  - SDA → GPIO 14
  - SCL → GPIO 13
  - VCC → 3.3V
  - GND → GND
- Ultrasonic
  - TRIG → GPIO 17
  - ECHO → GPIO 18  (CAUTION: shift to 3.3 V if your module outputs 5 V)
  - VCC → 5V (typical for HC‑SR04)
  - GND → GND
- LEDs (via resistors to GND)
  - RED anode → GPIO 4
  - YELLOW anode → GPIO 5
  - GREEN anode → GPIO 6
- Push button
  - One side → GPIO 8
  - Other side → GND
  - Pin mode uses `INPUT_PULLUP` (active LOW)

Electrical notes:
- Level shift ECHO to 3.3 V. If powering HC‑SR04 at 5 V, its ECHO is 5 V; use a divider or level shifter.
- Keep ultrasonic wires short and away from the ESP32 antenna. Add 100 nF + 10 µF decoupling near the sensor VCC.

---

## 3. Software and library installed
- Arduino IDE:
  - Boards Manager → Install “esp32” by Espressif Systems (latest)
  - Tools → Board → ESP32S3 Dev Module (or your exact S3 variant)
- Libraries (Library Manager):
  - Adafruit GFX Library
  - Adafruit SSD1306
  - PubSubClient
- Select:
  - Tools → Upload Speed (921600 or stable default)
  - Tools → Port → your ESP32‑S3
- Optional (PlatformIO):
  - Framework Arduino; add the above libraries to `lib_deps`.

---

## 4. RTOS Architecture and core task and synchronization
Core tasks (priorities in parentheses; higher = higher priority):
- Manager (3): State machine (READY → RUNNING → PAUSED). Consumes events from `eventQueue`. Starts/stops stopwatch, controls long PAUSED hold, queues Azure telemetry (and sends to Sheets if enabled).
- Stopwatch (2): Increments elapsed milliseconds when running using `vTaskDelayUntil(1 ms)`. Protected by `stopwatchMutex`.
- Azure (2): Ensures Wi‑Fi, NTP, MQTT SAS auth, and publishes telemetry from `azureQueue`.
- Ultrasonic (1): Samples distance ~20 Hz, applies hysteresis + consecutive‑sample debounce, emits `EV_OBJ_NEAR` on debounced far→near edges.
- Display (1): Renders state + HH:MM:SS to OLED.
- LED (1): Drives tri‑color LEDs; YELLOW blinks on 100 ms buckets aligned with stopwatch ms.

Synchronization and data flow:
- `eventQueue`: Carries `EV_OBJ_NEAR` (ultrasonic) and `EV_BUTTON` (ISR) to Manager.
- `azureQueue`: Manager enqueues telemetry on RUNNING→PAUSED; Azure task publishes over MQTT/TLS.
- `stopwatchMutex`: Protects shared stopwatch struct (`ms`, `running`).
- Button ISR: Debounced in ISR using RTOS ticks; posts `EV_BUTTON` to `eventQueue`.
- Ultrasonic debounce: Requires N consecutive “near” or “far” samples to change debounced state; only far→near raises event to avoid spam.

State machine (simplified):
- READY: GREEN ON, timer 00:00:00. On EV_OBJ_NEAR → RUNNING.
- RUNNING: Start stopwatch, GREEN OFF, YELLOW blinks. On EV_OBJ_NEAR or EV_BUTTON → PAUSED (capture start/end/duration, queue telemetry).
- PAUSED: RED ON, hold for `PAUSE_HOLD_MS` (config), then reset stopwatch and return to READY.

---

## 5. How to use (clone the project)
- Place these files in an Arduino sketch folder named like the main `.ino` (e.g., `rtos_stopwatch_iothub`):
  - `rtos_stopwatch_iothub.ino` (main firmware)
  - `azure_config.h` (Wi‑Fi + IoT Hub credentials)
  - `azure_root_ca.h` (root CA PEM for your IoT Hub TLS)
- Open the sketch in Arduino IDE.
- Edit `azure_config.h` (Wi‑Fi SSID/PASS, IoT Hub host, deviceId, deviceKey).
- Edit `azure_root_ca.h` (paste the correct root certificate PEM).
- Optional (Google Sheets): ensure your `SHEETS_WEBHOOK_URL` and `SHEETS_TOKEN` are set in the `.ino`.
- Select the correct board/port and Upload.
- Open Serial Monitor @ 115200 to observe Wi‑Fi/NTP/MQTT/Publish logs.

---

## 6. Make changes in the code (from all files)
Main tuning points in `rtos_stopwatch_iothub.ino`:
- Pinout:
  - `OLED_SDA`, `OLED_SCL`, `TRIG_PIN`, `ECHO_PIN`, `LED_RED`, `LED_YEL`, `LED_GRN`, `BTN_PIN`
- Stopwatch and UI:
  - `BLINK_MS_BUCKET` (default 100 ms), `PAUSE_HOLD_MS` (e.g., 10000 ms)
- Ultrasonic thresholds and debounce:
  - `NEAR_THRESH_CM` (10.0), `FAR_THRESH_CM` (12.0)
  - `ULTRASONIC_SAMPLE_MS` (50), `NEAR_CONSEC_REQUIRED` (3), `FAR_CONSEC_REQUIRED` (3)
- Google Sheets webhook:
  - `SHEETS_WEBHOOK_URL`, `SHEETS_TOKEN` (search for these constants)
- Azure connectivity:
  - `mqttClient.setBufferSize(1024)`, keep‑alive, socket timeouts

`azure_config.h`:
```c
#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASSWORD  "YOUR_WIFI_PASSWORD"
#define IOT_HUB_HOST   "your-hub.azure-devices.net"
#define DEVICE_ID      "your-device-id"
#define DEVICE_KEY     "BASE64_DEVICE_PRIMARY_KEY"
#define SAS_TTL_SECS   (60 * 60)
#define IOTHUB_API_VERSION "2021-04-12"
```

`azure_root_ca.h`:
- Paste the appropriate root PEM (often DigiCert Global Root G2; some hubs still chain to Baltimore CyberTrust Root). You can include multiple certs concatenated.

Advanced:
- Persist job number across reboots (NVS/Preferences).
- Offload Google Sheets HTTP POST to a dedicated low‑priority task + queue to avoid blocking Manager.

---

## 7. Azure IoT hub setup

1) Create IoT Hub and Device
- Azure Portal → Create “IoT Hub”.
- In the hub: IoT devices → Add device → Note `deviceId`. Copy the device primary key (base64 SharedAccessKey).

2) Fill firmware configs
- `azure_config.h`:
  - `IOT_HUB_HOST`: e.g., `your-hub.azure-devices.net`
  - `DEVICE_ID`: your device id
  - `DEVICE_KEY`: device primary key (base64), not the entire connection string
- `azure_root_ca.h`: paste root CA PEM.

3) Time and network
- Device relies on NTP; ensure internet access. Port 8883 (MQTT over TLS) must be open.

4) Test
- Upload and open Serial Monitor. You should see:
  - Wi‑Fi connected
  - NTP time sync
  - “[Azure] MQTT connected”
  - “[Azure] Publish OK” after a job completes
- Monitor:
  - 1. Open Azure IoT Hub and check messages.
![azure iot hub](/images/azure_iot_hub.png)

  - 2. Fetch the data using terminal
  ```bash
  az extension add -n azure-iot
  az iot hub monitor-events -n <YOUR_HUB_NAME> -d <DEVICE_ID>
  ```
  ![termina](/images/azure_data_fetch.png)

Troubleshooting MQTT:
- rc = -2 (socket/TLS): fix root CA, ensure correct host, try `wifiClient.setInsecure()` only for diagnosis.
- rc = 5 (unauthorized): check DEVICE_KEY, device exists/enabled, username format.

---

## 8. Google Sheets setup and app script
1) Create a Google Sheet
- Add header row: `jobNumber | startTime | endTime | durationMs | insertedAt`
- Note the spreadsheet ID from the URL (`/d/<SPREADSHEET_ID>/`).

2) Apps Script (Web App)
- Extensions → Apps Script. Paste a handler similar to:
```javascript
const EXPECTED_TOKEN = "YOUR_SECRET";
const SHEET_NAME = "Sheet1";
const SPREADSHEET_ID = "YOUR_SPREADSHEET_ID";

function doPost(e) {
  try {
    const token = (e.parameter && e.parameter.token) ? e.parameter.token : "";
    if (token !== EXPECTED_TOKEN) {
      return ContentService.createTextOutput(JSON.stringify({ status: "unauthorized" }))
        .setMimeType(ContentService.MimeType.JSON);
    }
    if (!e.postData || !e.postData.contents) {
      return ContentService.createTextOutput(JSON.stringify({ status: "bad_request" }))
        .setMimeType(ContentService.MimeType.JSON);
    }
    const p = JSON.parse(e.postData.contents);
    const ss = SpreadsheetApp.openById(SPREADSHEET_ID);
    const sheet = ss.getSheetByName(SHEET_NAME);
    sheet.appendRow([p.jobNumber, p.startTime, p.endTime, p.durationMs, new Date().toISOString()]);
    return ContentService.createTextOutput(JSON.stringify({ status: "ok" }))
      .setMimeType(ContentService.MimeType.JSON);
  } catch (err) {
    return ContentService.createTextOutput(JSON.stringify({ status: "error", message: err.toString() }))
      .setMimeType(ContentService.MimeType.JSON);
  }
}
```
- Deploy → New deployment → Web app
  - Execute as: Me
  - Who has access: Anyone (even anonymous)
  - Copy the Web App URL (ends with `/exec`).

3) Firmware configuration
- In `rtos_stopwatch_iothub.ino`, set:
  - `SHEETS_WEBHOOK_URL` to your `/exec` URL
  - `SHEETS_TOKEN` to match `EXPECTED_TOKEN`
- For a quick demo, the helper uses `client.setInsecure()`; for production, validate TLS.

4) Test from PC
```bash
curl -X POST "WEB_APP_URL?token=YOUR_SECRET" \
  -H "Content-Type: application/json" \
  -d '{"jobNumber":1,"startTime":"2025-11-11T21:05:51Z","endTime":"2025-11-11T21:06:32Z","durationMs":41436}'
```
- Expect: `{"status":"ok"}` and a new row in the sheet.

![google sheet](/images/google_sheet_data.png)
---

## 9. Demonstration
- Power the ESP32‑S3; open Serial Monitor (115200).
- READY: GREEN ON, OLED shows `00:00:00`.
- Bring an object within ~10 cm (debounced) → RUNNING:
  - GREEN OFF, YELLOW blinks ~10 Hz, stopwatch runs.
- Trigger stop (object near again or press button) → PAUSED:
  - RED ON for `PAUSE_HOLD_MS` (default 10 s), display frozen, telemetry queued/sent.
- Show cloud and sheet:
  - Azure: `az iot hub monitor-events -n <hub> -d <device>` to view each job JSON.
  - Google Sheet: rows appear live (use Freeze header; optionally insert charts).
- Repeat to accumulate rows for averages/charts.

---

## 10. Troubleshooting
Hardware
- Ultrasonic false triggers:
  - Keep wiring short; add decoupling (100 nF + 10 µF).
  - Level‑shift ECHO to 3.3 V if sensor is 5 V.
  - Increase debounce: `NEAR_CONSEC_REQUIRED`, `FAR_CONSEC_REQUIRED`.
  - Widen hysteresis: lower `NEAR_THRESH_CM` or raise `FAR_THRESH_CM`.
- OLED blank:
  - Confirm I2C address (0x3C). If needed run an I2C scanner.
  - Check SDA/SCL pins and pull‑ups; verify 3.3 V power.

Networking/Azure
- Wi‑Fi not connecting: SSID/PASS, 2.4 GHz band, antenna clearance, router distance.
- NTP time not syncing: ensure internet access (UDP 123). SAS requires valid time.
- MQTT rc = -2: TLS handshake failure; fix CA (`azure_root_ca.h`), host spelling, SNI, port 8883 open.
- MQTT rc = 5: Unauthorized; verify `DEVICE_KEY` (base64 key only), device enabled.
- SAS expiry: default 1 hour; device reconnects with fresh token as needed.

Google Sheets / Apps Script
- 403/HTML response:
  - Use Web App URL (`/exec`), not editor URL.
  - Web app deployment: Execute as “Me”; access “Anyone (even anonymous)”.
  - Authorize script (run a function in editor once to grant permissions).
- No rows but 200 OK:
  - Wrong `SPREADSHEET_ID` or `SHEET_NAME`; check exact values.
  - Check Apps Script Executions log for errors.
- Manager blocking on HTTP:
  - For a smoother demo, offload to a `sheetsQueue` and a low‑priority “Sheets” task.

Performance/Robustness
- If bursts occur, buffer messages in a queue (or NVS) and retry in a background task.
- Consider replacing `client.setInsecure()` with proper CA pinning for public demos.

---

## File structure (reference)
- `rtos_stopwatch_iothub.ino` — main FreeRTOS firmware with Azure MQTT and optional Google Sheets send
- `azure_config.h` — Wi‑Fi and IoT Hub credentials and settings
- `azure_root_ca.h` — Root CA certificate(s) for IoT Hub TLS

---

## License
Demo/educational use. Review licenses for third‑party libraries (Adafruit GFX/SSD1306, PubSubClient).
