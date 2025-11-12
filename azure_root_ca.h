#pragma once

// Replace this with the correct Azure IoT Hub Root CA for your region/tenant.
// As of recent Azure updates, DigiCert Global Root G2 is commonly required.
// You may also need Baltimore CyberTrust Root for legacy hubs.
// Concatenate multiple certs if necessary.

static const char AZURE_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";