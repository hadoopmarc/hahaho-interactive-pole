#pragma once

#define OTA_USERNAME "admin"
#define OTA_PASSWORD "some-password0"

typedef struct wifiCredentials
{
  char ssid[17];
  char token[17];
} wifiCredentials;

// Options for Wi-Fi networks to connect to
wifiCredentials btnCredentials[2] = {
    {"some-ssid1", "some-password1"},  // E.g. the RPi used in the Utecht public library
    {"some-ssid2", "some-password2"}   // E.g. your home WiFi network
};

// Fallback option for the Interactive Pole to function as access point
wifiCredentials apCredentials = {"hahahopole", "some-password3"};
