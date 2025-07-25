#pragma once

#include "web_ota.h"

// Options for Wi-Fi networks to connect to
Credentials btnCredentials[] = {
    {"some-ssid1", "some-password1"},  // E.g. the RPi used in the Utecht public library
    {"some-ssid2", "some-password2"}   // E.g. your home WiFi network
};

// Fallback option for the Interactive Pole to function as an access point
Credentials apCredentials = {"hahahopole", "some-password3"};

// Account to execute OTA firmware updates
Credentials otaCredentials = {"admin", "some-password4"};
