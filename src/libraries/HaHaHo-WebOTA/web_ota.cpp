/*
Uploading new binaries to the ESP32 over Wi-Fi will speed up the development

Based on:
https://github.com/IPdotSetAF/ESPAsyncHTTPUpdateServer/blob/master/examples/simple_server/simple_server.ino
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPAsyncWebServer.h>  // Install "ESP Aync WebServer" and "Async TCP" by ESP32Async
#include <ESPAsyncHTTPUpdateServer.h>  // Install ESPAsyncHTTPUpdateServer by ipdotsetaf
#include <ESPmDNS.h>
#include <Update.h>
#include <web_ota.h>

WiFiMulti wifiMulti;  // Selects the best of defined possible WiFi networks
AsyncWebServer server(80);
ESPAsyncHTTPUpdateServer updateServer;

void wifiInit(Credentials *btnCreds, int nbtn, Credentials *apCreds) {
  WiFi.mode(WIFI_STA);
  Serial.println("\nInitializing WiFi");
  for (int i = 0; i < nbtn; i++) {
    wifiMulti.addAP(btnCreds[i].id, btnCreds[i].password);
  }
  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.printf("ESP32 available for OTA updates at: ");
    Serial.print(WiFi.localIP());
    Serial.println("/update");
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apCreds->id, apCreds->password);
    MDNS.begin("esp32");
    Serial.printf("ESP32 available for OTA updates at: ");
    Serial.print(WiFi.softAPIP());
    Serial.println("/update");
    Serial.printf("No WiFi connection; ESP32 is AP with SSID: %s, PASS: %s\n", apCreds->id, apCreds->password);
  }
}

void web_ota_setup(Credentials *btnCreds, int nbtn, Credentials *apCreds, Credentials *otaCreds) {
  wifiInit(btnCreds, nbtn, apCreds);
  updateServer.setup(&server, "/update", otaCreds->id, otaCreds->password);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", "Use /update to enter the update page!");
  });
  server.onNotFound(
    [](AsyncWebServerRequest *request) {
      request->send(404, "text/plain", "Not found");
    }
  );
  updateServer.onUpdateBegin = [](const UpdateType type, int &result)
  {
      Serial.println("Update started : " + String(type));
  };
  updateServer.onUpdateEnd = [](const UpdateType type, int &result)
  {
      Serial.println("Update finished : " + String(type) + " result: " + String(result));
  };
  server.begin();
}
