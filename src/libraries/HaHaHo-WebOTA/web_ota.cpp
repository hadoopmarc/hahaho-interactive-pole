/*
Uploading new binaries to the ESP32 over Wi-Fi will speed up the development
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <Ticker.h>
#include <web_ota.h>
#include <html.h>

WiFiMulti wifiMulti;  // Selects the best of defined possible WiFi networks
WebServer server(80);
Ticker tkSecond;
Credentials *otaCreds;  // Global, so that they can be captured by lambda functions
uint8_t otaDone = 0;

const char *csrfHeaders[2] = { "Origin", "Host" };
static bool authenticated = false;


void wifiInit(Credentials *btnCreds, int nbtn, Credentials *apCreds) {
  WiFi.mode(WIFI_STA);
  Serial.println("\nInitializing WiFi");
  for (int i = 0; i < nbtn; i++) {
    wifiMulti.addAP(btnCreds[i].id, btnCreds[i].password);
  }
  Serial.printf("ESP32 available for OTA updates at: ");
  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apCreds->id, apCreds->password);
    MDNS.begin("esp32");
    Serial.println(WiFi.softAPIP());
    Serial.printf("No WiFi connection; ESP32 is AP with SSID: %s, PASS: %s\n", apCreds->id, apCreds->password);
  }
}

void handleUpdateEnd() {
  if (!authenticated) {
    return server.requestAuthentication();
  }
  server.sendHeader("Connection", "close");
  if (Update.hasError()) {
    server.send(502, "text/plain", Update.errorString());
  } else {
    server.sendHeader("Refresh", "10");
    server.sendHeader("Location", "/");
    server.send(307);
    delay(500);
    ESP.restart();
  }
}

void handleUpdate() {
  size_t fsize = UPDATE_SIZE_UNKNOWN;
  if (server.hasArg("size")) {
    fsize = server.arg("size").toInt();
  }
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    authenticated = server.authenticate(otaCreds->id, otaCreds->password);
    if (!authenticated) {
      Serial.println("Authentication fail!");
      otaDone = 0;
      return;
    }
    String origin = server.header(String(csrfHeaders[0]));
    String host = server.header(String(csrfHeaders[1]));
    String expectedOrigin = String("http://") + host;
    if (origin != expectedOrigin) {
      Serial.printf("Wrong origin received! Expected: %s, Received: %s\n", expectedOrigin.c_str(), origin.c_str());
      authenticated = false;
      otaDone = 0;
      return;
    }

    Serial.printf("Receiving Update: %s, Size: %d\n", upload.filename.c_str(), fsize);
    if (!Update.begin(fsize)) {
      otaDone = 0;
      Update.printError(Serial);
    }
  } else if (authenticated && upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    } else {
      otaDone = 100 * Update.progress() / Update.size();
    }
  } else if (authenticated && upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("Update Success: %u bytes\nRebooting...\n", upload.totalSize);
    } else {
      Serial.printf("%s\n", Update.errorString());
      otaDone = 0;
    }
  }
}

void webServerInit() {
  server.collectHeaders(csrfHeaders, 2);
  server.on(
    "/update", HTTP_POST,
    []() {
      handleUpdateEnd();
    },
    []() {
      handleUpdate();
    });
  server.on("/favicon.ico", HTTP_GET, []() {
    server.sendHeader("Content-Encoding", "gzip");
    server.send_P(200, "image/x-icon", favicon_ico_gz, favicon_ico_gz_len);
  });
  server.onNotFound([]() {
    if (!server.authenticate(otaCreds->id, otaCreds->password)) {
      return server.requestAuthentication();
    }
    server.send(200, "text/html", indexHtml);
  });
  server.begin();
}

void everySecond() {
  if (otaDone > 1) {
    Serial.printf("ota: %d%%\n", otaDone);
  }
}

void ota_setup(Credentials *btnCreds, int nbtn, Credentials *apCreds, Credentials *otaCredsPtr) {
  otaCreds = otaCredsPtr;
  wifiInit(btnCreds, nbtn, apCreds);
  webServerInit();
  tkSecond.attach(1, everySecond);
}

void ota_loop() {
  server.handleClient();
}
