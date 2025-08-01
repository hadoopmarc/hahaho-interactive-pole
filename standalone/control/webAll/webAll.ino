/*
Simple demo how webOTAupdate, webSerial and webDash can be combined into a single webserver
with handlers for multiple pages.
*/
#include <arduino.h>                       // Only relevant for VSCode/PlatformIO Intellisense
#include <AsyncWebSerial.h>                // Install "AsyncWebSerial" by Sessa
#include <ESPAsyncWebServer.h>             // Install "ESP Aync WebServer" and "Async TCP" by ESP32Async
#include <ESPDash.h>
#include <web_ota.h>                       // From src/libraries installed in Arduino/Platform libraries
#include "secrets.h"

char dashURI[] = "/dash";
char serialURI[] = "/serial";
char updateURI[] = "/update";

AsyncWebServer server(80);
AsyncWebSerial webSerial;

// Dashboard Cards
using namespace dash;
ESPDash dashboard = ESPDash(server, dashURI, true);
TemperatureCard temperature(dashboard, "Temperature");
HumidityCard humidity(dashboard, "Humidity");

// Variables for example funtionality
const int led = 2;                         // ESP32 Pin to which onboard LED is connected
int ledState = LOW;                        // State of the LED to be negated in each loop 
int webSerialCounter = 0;                  // Example output for webSerial

// Timing control
const long ledPeriod = 200;                // Interval at which to blink (milliseconds)
unsigned long ledMillis = 0;               // Millis at the last led state change
const long serialPeriod = 1000;            // Interval at which to update the webSerial page
unsigned long serialMillis = 0;            // Millis at the last webSerial update
const long dashPeriod = 3000;              // Interval at which to update the dashboard page
unsigned long dashMillis = 0;              // Millis at the last dashboard update

void setup() {
  Serial.begin(115200);                    // For use by both the script and the libraries

  // Hardware setup (do not change. ToDo: make into a library that reflects the pole hardware wiring)
  pinMode(led, OUTPUT);

  // Library setup voor OTA updates and webSerial monitoring
  web_ota_setup(server, btnCredentials, sizeof(btnCredentials) / sizeof(Credentials), &apCredentials, &otaCredentials);
  webSerial.begin(&server);

  server.begin();
}

// All function calls should be non-blocking (do not use the delay() function)
void loop() {
  unsigned long currentMillis = millis();

  // blink internal LED
  if (currentMillis - ledMillis >= ledPeriod) {
    digitalWrite(led, ledState);
    ledState = not(ledState);
    ledMillis = currentMillis;
  }

  // Update webSerial page
  if (currentMillis - serialMillis >= serialPeriod) {
    webSerial.print(webSerialCounter++);
    webSerial.loop();
    serialMillis = currentMillis;
  }

  // Update dashboard cards
  if (currentMillis - dashMillis >= dashPeriod) {
    temperature.setValue((int)random(0, 50));
    humidity.setValue((int)random(0, 100));
    dashboard.sendUpdates();
    dashMillis = currentMillis;
  }
}