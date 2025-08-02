/*
Uploading new binaries to the ESP32 over Wi-Fi will speed up the development

Based on:
https://github.com/IPdotSetAF/ESPAsyncHTTPUpdateServer/blob/master/examples/simple_server/simple_server.ino
*/
#include <Arduino.h>
#include <ESPAsyncWebServer.h>  // Install "ESP Aync WebServer" and "Async TCP" by ESP32Async
#include <ESPAsyncHTTPUpdateServer.h>  // Install ESPAsyncHTTPUpdateServer by ipdotsetaf
#include <Update.h>
#include <web_ota.h>

ESPAsyncHTTPUpdateServer updateServer;

void web_ota_setup(AsyncWebServer& server, Credentials *otaCreds) {
  updateServer.setup(&server, "/update", otaCreds->id, otaCreds->password);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", "Use /dash, /webserial or /update");
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
