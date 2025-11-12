#pragma once

// ====== Wi-Fi ======
#define WIFI_SSID      "FINNISH_PROBLEM"
#define WIFI_PASSWORD  "FINNISH_PROBLEM"

// ====== Azure IoT Hub ======
// Hostname like: "your-hub.azure-devices.net"
#define IOT_HUB_HOST   "SajibIoTHub.azure-devices.net"
// Device ID you created in IoT Hub
#define DEVICE_ID      "SajibDevID"
// Device key (primary key) from IoT Hub device; this is BASE64 string
#define DEVICE_KEY     "ZXrKybmvOEkbzCTQ3YWnav9EZ5GnzKzLX6gufeWlNzk="

// SAS token lifetime (seconds)
#define SAS_TTL_SECS   (60 * 60) // 1 hour

// Optional: API version
#define IOTHUB_API_VERSION "2021-04-12"

