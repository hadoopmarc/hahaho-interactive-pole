/*
General utility to connect to the best of specified WiFi networks and fallback to
serve as an access point if not network is available.
*/
#include <WiFi.h>
#include <WiFiMulti.h>
#include <pole_wifi.h>

WiFiMulti wifiMulti;

void wifi_setup(Credentials *btnCreds, int nbtn, Credentials *apCreds) {
  IPAddress ip;
  WiFi.mode(WIFI_STA);
  Serial.println("\nInitializing WiFi");
  for (int i = 0; i < nbtn; i++) {
    wifiMulti.addAP(btnCreds[i].id, btnCreds[i].password);
  }
  if (wifiMulti.run() == WL_CONNECTED) {
    ip = WiFi.localIP();
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apCreds->id, apCreds->password);
    ip = WiFi.softAPIP();
  }
  Serial.printf("ESP32 available at: %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("No WiFi connection; ESP32 is AP with SSID: %s, PASS: %s\n", apCreds->id, apCreds->password);
  }
}
