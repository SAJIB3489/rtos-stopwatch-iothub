#pragma once

// ====== Wi-Fi ======
#define WIFI_SSID      "SSID_Name"
#define WIFI_PASSWORD  "Password"

// ====== Azure IoT Hub ======
// Hostname like: "your-hub.azure-devices.net"
#define IOT_HUB_HOST   "Host_Name"
// Device ID you created in IoT Hub
#define DEVICE_ID      "Device_ID"
// Device key (primary key) from IoT Hub device; this is BASE64 string
#define DEVICE_KEY     ""

// SAS token lifetime (seconds)
#define SAS_TTL_SECS   (60 * 60) // 1 hour

// Optional: API version
#define IOTHUB_API_VERSION "2021-04-12"

