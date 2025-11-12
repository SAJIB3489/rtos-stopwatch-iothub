/*
  RTOS Stopwatch System + Azure IoT Hub Publisher (ESP32-S3) by Md Sajib Pramanic

  Pins:
    OLED: SDA=14, SCL=13
    Ultrasonic: TRIG=17, ECHO=18
    LEDs: RED=4, YELLOW=5, GREEN=6
    Button: 8 (active LOW, PULLUP)

  Behavior:
    - READY: GREEN ON, display "00:00:00"
    - RUNNING: start when object < 10cm, GREEN OFF, YELLOW blinks per 100ms bucket
    - PAUSED: stop via object <10cm (edge) or button press; RED ON, pause 20s, then reset to READY
    - On each RUNNING → PAUSED (job done): send telemetry to Azure IoT Hub:
      { jobNumber, startTime(UTC), endTime(UTC), durationMs }

  Dependencies:
    - Adafruit SSD1306, Adafruit GFX
    - PubSubClient (MQTT)
    - ESP32 Arduino core (with mbedTLS)
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include <ctype.h>
#include <mbedtls/base64.h>
#include <mbedtls/md.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "azure_config.h"
#include "azure_root_ca.h"

// Add these webhook constants near your other #define constants (e.g., after the Azure config section)
#include <HTTPClient.h> /*new*/
const char* SHEETS_WEBHOOK_URL = "https://script.google.com/macros/s/AKfycbw6KojmC1_1A7EqYO1T5qTt3Y9WoiYSaTVq17-M91BFmeDM_A2qseGJselLF3d7xwrEmw/exec"; // replace with your Web App URL /*new*/
const char* SHEETS_TOKEN       = "hkdshfshfTUYSD7SAKSDAL8"; // must match EXPECTED_TOKEN in Apps Script /*new*/

// ==== OLED ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 14
#define OLED_SCL 13
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==== Ultrasonic ====
#define TRIG_PIN 17
#define ECHO_PIN 18
#define PULSE_TIMEOUT_US 30000UL // 30ms (~5m max)

// ==== LEDs ====
#define LED_RED   4
#define LED_YEL   5
#define LED_GRN   6

// ==== Button ====
#define BTN_PIN   8

// ==== Distance thresholds ====
#define NEAR_THRESH_CM 10.0f
#define FAR_THRESH_CM  12.0f

// ==== Stopwatch/LED ====
#define BLINK_MS_BUCKET 100
#define PAUSE_HOLD_MS   10000 // 10 seconds hold

// ====NEW DEFINE TO SOLVE ULTRASONIC FALSE TRIGGER
// Debounce the ultrasonic edge: require N consecutive samples
#define ULTRASONIC_SAMPLE_MS     50   // sampling interval (ms)
#define NEAR_CONSEC_REQUIRED     3    // require 3 consecutive "near" samples to confirm NEAR
#define FAR_CONSEC_REQUIRED      3    // require 3 consecutive "far" samples to confirm FAR

// ==== RTOS handles ====
TaskHandle_t taskManagerHandle    = nullptr;
TaskHandle_t taskUltrasonicHandle = nullptr;
TaskHandle_t taskStopwatchHandle  = nullptr;
TaskHandle_t taskDisplayHandle    = nullptr;
TaskHandle_t taskLedHandle        = nullptr;
TaskHandle_t taskAzureHandle      = nullptr;

QueueHandle_t eventQueue = nullptr;
QueueHandle_t azureQueue = nullptr;
SemaphoreHandle_t stopwatchMutex = nullptr;

enum SystemState : uint8_t { STATE_READY = 0, STATE_RUNNING, STATE_PAUSED };
enum EventType : uint8_t { EV_OBJ_NEAR = 1, EV_BUTTON = 2 };

struct Event { EventType type; };

struct Stopwatch {
  uint32_t ms = 0;
  bool running = false;
};

struct Telemetry {
  uint32_t jobId;
  time_t startUtc;
  time_t endUtc;
  uint32_t durationMs;
};

// Globals
volatile SystemState gState = STATE_READY;
Stopwatch gStopwatch;

volatile uint32_t gCurrentJobId = 0;
volatile time_t   gCurrentJobStart = 0;

static volatile uint32_t gLastButtonTickISR = 0;

// WiFi/MQTT
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// ==== Forward declarations ====
void taskManager(void* arg);
void taskUltrasonic(void* arg);
void taskStopwatch(void* arg);
void taskDisplay(void* arg);
void taskLed(void* arg);
void taskAzure(void* arg);
void IRAM_ATTR isrButton();

static inline unsigned long measureEchoDuration();
static inline float durationToCm(unsigned long duration_us);
static inline void setStopwatchRunning(bool r);
static inline void resetStopwatch();
static inline uint32_t getStopwatchMs();
static inline bool isStopwatchRunning();

static void formatHMS(char* out, size_t outLen, uint32_t msTotal);

// Azure helpers
static bool ensureWiFi();
static bool ensureTime();
static String urlEncode(const String& s);
static bool generateSasToken(String& outToken, time_t expiry);
static bool ensureMqtt();
static bool mqttPublishTelemetry(const Telemetry& t);
static String iso8601(time_t ts);


// ==== Setup ====
void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 init failed"));
    while (true) delay(100);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  display.display();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YEL, OUTPUT);
  pinMode(LED_GRN, OUTPUT);

  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), isrButton, FALLING);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YEL, LOW);
  digitalWrite(LED_GRN, HIGH);

  eventQueue = xQueueCreate(10, sizeof(Event));
  azureQueue = xQueueCreate(10, sizeof(Telemetry));
  stopwatchMutex = xSemaphoreCreateMutex();

  // MQTT setup
  wifiClient.setCACert(AZURE_ROOT_CA);   // For testing only: you may use wifiClient.setInsecure();
  mqttClient.setServer(IOT_HUB_HOST, 8883);
  mqttClient.setBufferSize(1024); // Allow JSON payload comfortably

//start
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(20);
  wifiClient.setTimeout(15000);
  
  #ifndef AZURE_TLS_INSECURE_TEST
  #define AZURE_TLS_INSECURE_TEST 0
  #endif

  #if AZURE_TLS_INSECURE_TEST
    wifiClient.setInsecure(); // DIAGNOSTIC ONLY
    Serial.println(F("[Azure] WARNING: TLS validation disabled (diagnostic)."));
  #else
    wifiClient.setCACert(AZURE_ROOT_CA);
  #endif
  

  // Create tasks
  xTaskCreate(taskManager,   "Manager",   4096, nullptr, 3, &taskManagerHandle);
  xTaskCreate(taskStopwatch, "Stopwatch", 4096, nullptr, 2, &taskStopwatchHandle);
  xTaskCreate(taskUltrasonic,"Ultrasonic",4096, nullptr, 1, &taskUltrasonicHandle);
  xTaskCreate(taskDisplay,   "Display",   4096, nullptr, 1, &taskDisplayHandle);
  xTaskCreate(taskLed,       "LED",       2048, nullptr, 1, &taskLedHandle);
  xTaskCreate(taskAzure,     "Azure",     8192, nullptr, 2, &taskAzureHandle);

  // Initial screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("RTOS Stopwatch READY"));
  display.setTextSize(2);
  display.setCursor(0, 24);
  display.print(F("00:00:00"));
  display.display();
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

// ==== ISR ====
void IRAM_ATTR isrButton() {
  BaseType_t hpTaskWoken = pdFALSE;
  uint32_t now = xTaskGetTickCountFromISR();
  if ((now - gLastButtonTickISR) > pdMS_TO_TICKS(200)) {
    Event ev{ EV_BUTTON };
    if (eventQueue) xQueueSendFromISR(eventQueue, &ev, &hpTaskWoken);
    gLastButtonTickISR = now;
  }
  if (hpTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

// ==== Utilities ====
static inline unsigned long measureEchoDuration() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);
}

static inline float durationToCm(unsigned long duration_us) {
  return 0.017f * (float)duration_us;
}

static inline void setStopwatchRunning(bool r) {
  xSemaphoreTake(stopwatchMutex, portMAX_DELAY);
  gStopwatch.running = r;
  xSemaphoreGive(stopwatchMutex);
}

static inline void resetStopwatch() {
  xSemaphoreTake(stopwatchMutex, portMAX_DELAY);
  gStopwatch.ms = 0;
  gStopwatch.running = false;
  xSemaphoreGive(stopwatchMutex);
}

static inline uint32_t getStopwatchMs() {
  xSemaphoreTake(stopwatchMutex, portMAX_DELAY);
  uint32_t v = gStopwatch.ms;
  xSemaphoreGive(stopwatchMutex);
  return v;
}

static inline bool isStopwatchRunning() {
  xSemaphoreTake(stopwatchMutex, portMAX_DELAY);
  bool r = gStopwatch.running;
  xSemaphoreGive(stopwatchMutex);
  return r;
}

static void formatHMS(char* out, size_t outLen, uint32_t msTotal) {
  uint32_t totalSec = msTotal / 1000UL;
  uint32_t s = totalSec % 60UL;
  uint32_t m = (totalSec / 60UL) % 60UL;
  uint32_t h = (totalSec / 3600UL) % 100UL;
  snprintf(out, outLen, "%02u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)s);
}

// ==== Tasks ====
void taskManager(void* arg) {
  gState = STATE_READY;
  resetStopwatch();
  digitalWrite(LED_GRN, HIGH);
  digitalWrite(LED_YEL, LOW);
  digitalWrite(LED_RED, LOW);

  Event ev{};
  for (;;) {
    if (xQueueReceive(eventQueue, &ev, portMAX_DELAY) == pdTRUE) {
      switch (gState) {
        case STATE_READY:
          if (ev.type == EV_OBJ_NEAR) {
            gState = STATE_RUNNING;
            // New job starts
            gCurrentJobId += 1;
            gCurrentJobStart = time(nullptr); // UTC (requires NTP)
            setStopwatchRunning(true);
            digitalWrite(LED_GRN, LOW);
          }
          break;

        case STATE_RUNNING:
          if (ev.type == EV_OBJ_NEAR || ev.type == EV_BUTTON) {
            // End job
            setStopwatchRunning(false);
            uint32_t durMs = getStopwatchMs();
            time_t endUtc = time(nullptr);

            Telemetry t;
            t.jobId = gCurrentJobId;
            t.startUtc = gCurrentJobStart;
            t.endUtc = endUtc;
            t.durationMs = durMs;
            // Send to Azure queue (non-blocking best-effort)
            xQueueSend(azureQueue, &t, 0);

                        // ---------- INSERT #3: send same job to Google Sheets (best-effort) new start----------
            {
              String startIso = iso8601(gCurrentJobStart);
              String endIso   = iso8601(endUtc);
              bool sheetOk = sendJobToSheet((uint32_t)t.jobId, startIso, endIso, t.durationMs);
              if (!sheetOk) {
                Serial.println("[Sheets] failed to send job (no retry in this path)");
                // optional: enqueue locally for retry or write to NVS
              }
            }
            // -------------------------------end------------------------------------------------

            // Enter paused state (20s)
            gState = STATE_PAUSED;
            digitalWrite(LED_YEL, LOW);
            digitalWrite(LED_RED, HIGH);

            vTaskDelay(pdMS_TO_TICKS(PAUSE_HOLD_MS));

            // Reset to READY
            gState = STATE_READY;
            resetStopwatch();
            digitalWrite(LED_RED, LOW);
            digitalWrite(LED_GRN, HIGH);
          }
          break;

        case STATE_PAUSED:
          // Ignore events during the hold
          break;
      }
    }
  }
}

void taskUltrasonic(void* arg) {
  bool debouncedNear = false;   // debounced state
  uint8_t nearCnt = 0;
  uint8_t farCnt  = 0;

  for (;;) {
    // Measure distance
    unsigned long dur = measureEchoDuration();
    float dist = (dur == 0) ? 9999.0f : durationToCm(dur);

    // Raw classification with hysteresis bands
    bool rawNear = (dist < NEAR_THRESH_CM);
    bool rawFar  = (dist > FAR_THRESH_CM);

    // Update consecutive counters
    if (rawNear) {
      nearCnt = (nearCnt < 255) ? (nearCnt + 1) : 255;
      farCnt = 0;
    } else if (rawFar) {
      farCnt = (farCnt < 255) ? (farCnt + 1) : 255;
      nearCnt = 0;
    } else {
      // In hysteresis band: don't accumulate either; slowly decay counters
      if (nearCnt > 0) nearCnt--;
      if (farCnt  > 0) farCnt--;
    }

    // Debounced transitions
    if (!debouncedNear && nearCnt >= NEAR_CONSEC_REQUIRED) {
      debouncedNear = true;
      nearCnt = 0;           // reset counter after transition

      // Far -> Near debounced edge: generate event
      Event ev{ EV_OBJ_NEAR };
      xQueueSend(eventQueue, &ev, 0);
    } else if (debouncedNear && farCnt >= FAR_CONSEC_REQUIRED) {
      // Near -> Far debounced transition (no event on this edge)
      debouncedNear = false;
      farCnt = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SAMPLE_MS));
  }
}

void taskStopwatch(void* arg) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    xSemaphoreTake(stopwatchMutex, portMAX_DELAY);
    if (gStopwatch.running) {
      gStopwatch.ms += 1;
    }
    xSemaphoreGive(stopwatchMutex);
  }
}

void taskDisplay(void* arg) {
  char buf[16];
  SystemState lastState = (SystemState)255;
  uint32_t lastSec = UINT32_MAX;

  for (;;) {
    SystemState s = gState;
    uint32_t ms = getStopwatchMs();
    uint32_t sec = ms / 1000UL;

    if (s != lastState || sec != lastSec) {
      lastState = s;
      lastSec = sec;

      formatHMS(buf, sizeof(buf), (s == STATE_READY) ? 0 : ms);

      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      if (s == STATE_READY) display.print(F("READY"));
      else if (s == STATE_RUNNING) display.print(F("RUNNING"));
      else display.print(F("PAUSED"));

      display.setTextSize(2);
      display.setCursor(0, 24);
      display.print(buf);
      display.display();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void taskLed(void* arg) {
  uint32_t lastBucket = UINT32_MAX;
  for (;;) {
    SystemState s = gState;

    if (s == STATE_READY) {
      digitalWrite(LED_GRN, HIGH);
      digitalWrite(LED_YEL, LOW);
      digitalWrite(LED_RED, LOW);
      lastBucket = UINT32_MAX;
    } else if (s == STATE_RUNNING) {
      digitalWrite(LED_GRN, LOW);
      digitalWrite(LED_RED, LOW);
      uint32_t ms = getStopwatchMs();
      uint32_t bucket = ms / BLINK_MS_BUCKET;
      if (bucket != lastBucket) {
        digitalWrite(LED_YEL, !digitalRead(LED_YEL));
        lastBucket = bucket;
      }
    } else { // PAUSED
      digitalWrite(LED_GRN, LOW);
      digitalWrite(LED_YEL, LOW);
      digitalWrite(LED_RED, HIGH);
      lastBucket = UINT32_MAX;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==== Azure IoT Hub task ====
void taskAzure(void* arg) {
  // Bring up Wi-Fi and time; then maintain MQTT and publish telemetry from queue
  for (;;) {
    if (!ensureWiFi()) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    if (!ensureTime()) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    if (!ensureMqtt()) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    // Process telemetry messages
    Telemetry t;
    if (xQueueReceive(azureQueue, &t, pdMS_TO_TICKS(250))) {
      // Ensure MQTT connection and valid SAS (ensureMqtt does both)
      if (!ensureMqtt()) {
        // Will retry next loop
        continue;
      }

      if (!mqttPublishTelemetry(t)) {
        Serial.println(F("[Azure] Publish failed, will retry connection."));
        mqttClient.disconnect();
      }
    } else {
      // Keep MQTT alive
      mqttClient.loop();
    }
  }
}

// ==== Wi-Fi / Time / MQTT helpers ====
static bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  Serial.print(F("[WiFi] Connecting to "));
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
    vTaskDelay(pdMS_TO_TICKS(250));
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[WiFi] Connected. IP: "));
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println(F("[WiFi] Failed to connect"));
    return false;
  }
}

static bool ensureTime() {
  time_t now = time(nullptr);
  if (now > 1700000000) return true; // already valid time

  Serial.println(F("[NTP] Syncing time..."));
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  uint32_t start = millis();
  while ((now = time(nullptr)) < 1700000000 && (millis() - start) < 15000) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  if (now >= 1700000000) {
    Serial.print(F("[NTP] Time: "));
    Serial.println(iso8601(now));
    return true;
  }
  Serial.println(F("[NTP] Failed to sync time"));
  return false;
}

static String toLower(const String& s) {
  String r = s;
  for (size_t i = 0; i < r.length(); ++i) r.setCharAt(i, (char)tolower(r[i]));
  return r;
}

static String urlEncode(const String& s) {
  String out;
  const char* hex = "0123456789ABCDEF";
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (('a' <= c && c <= 'z') ||
        ('A' <= c && c <= 'Z') ||
        ('0' <= c && c <= '9') ||
        c == '-' || c == '_' || c == '.' || c == '~') {
      out += c;
    } else {
      out += '%';
      out += hex[(c >> 4) & 0xF];
      out += hex[c & 0xF];
    }
  }
  return out;
}

static bool base64Decode(const String& inB64, std::vector<uint8_t>& out) {
  size_t olen = 0;
  int rc = mbedtls_base64_decode(nullptr, 0, &olen,
                                 (const unsigned char*)inB64.c_str(), inB64.length());
  if (rc != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL && rc != 0) return false;
  out.resize(olen);
  rc = mbedtls_base64_decode(out.data(), out.size(), &olen,
                             (const unsigned char*)inB64.c_str(), inB64.length());
  if (rc != 0) return false;
  out.resize(olen);
  return true;
}

static bool hmacSha256(const uint8_t* key, size_t keyLen,
                       const uint8_t* msg, size_t msgLen,
                       std::vector<uint8_t>& mac) {
  mac.resize(32);
  const mbedtls_md_info_t* mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!mdInfo) return false;

  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  if (mbedtls_md_setup(&ctx, mdInfo, 1) != 0) { mbedtls_md_free(&ctx); return false; }
  if (mbedtls_md_hmac_starts(&ctx, key, keyLen) != 0) { mbedtls_md_free(&ctx); return false; }
  if (mbedtls_md_hmac_update(&ctx, msg, msgLen) != 0) { mbedtls_md_free(&ctx); return false; }
  if (mbedtls_md_hmac_finish(&ctx, mac.data()) != 0) { mbedtls_md_free(&ctx); return false; }
  mbedtls_md_free(&ctx);
  return true;
}

static bool base64Encode(const uint8_t* data, size_t len, String& outB64) {
  size_t olen = 0;
  int rc = mbedtls_base64_encode(nullptr, 0, &olen, data, len);
  if (rc != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL && rc != 0) return false;
  std::vector<uint8_t> buf(olen);
  rc = mbedtls_base64_encode(buf.data(), buf.size(), &olen, data, len);
  if (rc != 0) return false;
  outB64 = String((const char*)buf.data(), olen);
  return true;
}

static String resourceUriLowerEncoded() {
  // Must be lowercase and URL-encoded per Azure SAS spec
  String sr = toLower(String(IOT_HUB_HOST)) + "/devices/" + toLower(String(DEVICE_ID));
  return urlEncode(sr);
}

static bool generateSasToken(String& outToken, time_t expiry) {
  // String to sign: <lower+urlenc resourceUri>\n<expiry>
  String srEncLower = resourceUriLowerEncoded();
  String toSign = srEncLower + "\n" + String((unsigned long)expiry);

  // Decode device key (base64) to bytes
  std::vector<uint8_t> keyBytes;
  if (!base64Decode(String(DEVICE_KEY), keyBytes)) {
    Serial.println(F("[Azure] Invalid DEVICE_KEY base64"));
    return false;
  }

  // HMAC-SHA256
  std::vector<uint8_t> mac;
  if (!hmacSha256(keyBytes.data(), keyBytes.size(),
                  (const uint8_t*)toSign.c_str(), toSign.length(), mac)) {
    Serial.println(F("[Azure] HMAC failed"));
    return false;
  }

  // Base64 signature, then URL-encode
  String sigB64;
  if (!base64Encode(mac.data(), mac.size(), sigB64)) {
    Serial.println(F("[Azure] Base64 encode failed"));
    return false;
  }
  String sigEnc = urlEncode(sigB64);

  // sr (NOT re-encoded), per Azure expects same lower+urlencoded string used in signature
  String sas = "SharedAccessSignature sr=" + srEncLower +
               "&sig=" + sigEnc +
               "&se=" + String((unsigned long)expiry);
  outToken = sas;
  return true;
}

static bool ensureMqtt() {
  // If connected, make sure token not expired soon. For simplicity, reconnect periodically.
  if (mqttClient.connected()) {
    mqttClient.loop();
    return true;
  }

  // Build connect params
  String username = String(IOT_HUB_HOST) + "/" + DEVICE_ID + "/?api-version=" + IOTHUB_API_VERSION;
  time_t now = time(nullptr);
  time_t expiry = now + SAS_TTL_SECS;
  String sas;
  if (!generateSasToken(sas, expiry)) {
    return false;
  }

  // Prepare TLS (SNI handled by WiFiClientSecure)
  // wifiClient.setCACert(AZURE_ROOT_CA); // already set in setup

  Serial.println(F("[Azure] Connecting MQTT..."));
  if (!mqttClient.connect(DEVICE_ID, username.c_str(), sas.c_str())) {
    Serial.print(F("[Azure] MQTT connect failed, rc="));
    Serial.println(mqttClient.state());
    return false;
  }
  Serial.println(F("[Azure] MQTT connected"));
  return true;
}

static String iso8601(time_t ts) {
  if (ts <= 0) return String("1970-01-01T00:00:00Z");
  struct tm tmUtc;
  gmtime_r(&ts, &tmUtc);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmUtc);
  return String(buf);
}

static bool mqttPublishTelemetry(const Telemetry& t) {
  // Topic for telemetry
  String topic = "devices/";
  topic += DEVICE_ID;
  topic += "/messages/events/";

  // JSON payload
  // Example:
  // {"jobNumber":1,"startTime":"2025-11-01T19:45:00Z","endTime":"2025-11-01T19:45:12Z","durationMs":12034}
  char payload[256];
  String startStr = iso8601(t.startUtc);
  String endStr   = iso8601(t.endUtc);
  int n = snprintf(payload, sizeof(payload),
                   "{\"jobNumber\":%lu,\"startTime\":\"%s\",\"endTime\":\"%s\",\"durationMs\":%lu}",
                   (unsigned long)t.jobId,
                   startStr.c_str(),
                   endStr.c_str(),
                   (unsigned long)t.durationMs);
  if (n <= 0 || n >= (int)sizeof(payload)) {
    Serial.println(F("[Azure] Payload build error"));
    return false;
  }

  bool ok = mqttClient.publish(topic.c_str(), payload);
  Serial.print(F("[Azure] Publish "));
  Serial.println(ok ? F("OK") : F("FAILED"));
  return ok;
}

//new start

// Place this function after your helper functions (e.g., after iso8601(...) or mqttPublishTelemetry(...))

// Sends one job entry to the Google Apps Script Web App (Sheets). Blocking HTTP POST.
bool sendJobToSheet(uint32_t jobNumber, const String& startIso, const String& endIso, uint32_t durationMs) {
  // Build JSON payload
  String payload = "{";
  payload += "\"jobNumber\":" + String(jobNumber) + ",";
  payload += "\"startTime\":\"" + startIso + "\",";
  payload += "\"endTime\":\"" + endIso + "\",";
  payload += "\"durationMs\":" + String(durationMs);
  payload += "}";

  // Use a fresh WiFiClientSecure for the HTTP request
  WiFiClientSecure client;
  client.setInsecure(); // DEMO ONLY: skip cert verification. For production, use setCACert(...).

  HTTPClient http;
  // Append token as query param (matches Apps Script check ?token=...)
  String url = String(SHEETS_WEBHOOK_URL) + "?token=" + String(SHEETS_TOKEN);

  if (!http.begin(client, url)) {
    Serial.println("[Sheets] HTTP begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);
  if (code <= 0) {
    Serial.printf("[Sheets] HTTP POST failed, err=%s\n", http.errorToString(code).c_str());
    http.end();
    return false;
  }

  String resp = http.getString();
  Serial.printf("[Sheets] resp (%d): %s\n", code, resp.c_str());
  http.end();
  return (code >= 200 && code < 300);
}

//end