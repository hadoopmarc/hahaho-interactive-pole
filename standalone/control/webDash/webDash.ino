/*
  ------------------------
  ESP-DASH - Basic Example
  ------------------------

  Basic dashboard with some cards, filled with random data in real time.

  Github: https://github.com/ayushsharma82/ESP-DASH
  WiKi: https://docs.espdash.pro

  Works with ESP32, RP2040+W and RP2350+W based devices / projects.
  -------------------------------

  This can be used for the Interactive Pole to monitor sensor readings.
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>  // Install "ESP Aync WebServer" and "Async TCP" by ESP32Async
#include <ESPDash.h>
#include "secrets.h"

/* Start Webserver */
AsyncWebServer server(80);

/* Attach ESP-DASH to AsyncWebServer */
ESPDash dashboard(server);

/*
  Dashboard Cards
  Format - (Dashboard Instance, Card Type, Card Name, Card Symbol(optional) )
*/
dash::TemperatureCard temperature(dashboard, "Temperature");
dash::HumidityCard humidity(dashboard, "Humidity");

void setup() {
  Serial.begin(115200);

  /* Connect WiFi */
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.printf("WiFi Failed!\n");
      return;
  }
  Serial.print("\nIP Address: ");
  Serial.println(WiFi.localIP());

  /* Start AsyncWebServer */
  server.begin();
}

void loop() {
  /* Update Card Values */
  temperature.setValue((int)random(0, 50));
  humidity.setValue((int)random(0, 100));

  /* Send Updates to our Dashboard (realtime) */
  dashboard.sendUpdates();

  /*
    Delay is just for demonstration purposes in this example,
    Replace this code with 'millis interval' in your final project.
  */
  delay(3000);
}
