/*
Uploading new binaries to the ESP32 over Wi-Fi will speed up the development
*/
#include <arduino.h>                       // Only relevant for VSCode/PlatformIO Intellisense
#include <ESPAsyncWebServer.h>             // Install "ESP Aync WebServer" and "Async TCP" by 
// See src/libraries/README.md for use in Arduino IDE
#include <secrets.h>
#include <pole_wifi.h>
#include <web_ota.h>

AsyncWebServer server(80);

const int led = 2;                         // ESP32 Pin to which onboard LED is connected
const long interval = 500;                 // interval at which to blink (milliseconds)
int ledState = LOW;                        // state of the LED to be negated in each loop cycle

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  wifi_setup(btnCredentials, btnCredsSize, &apCredentials);
  Serial.println("Open webpage at /update");

  // This includes setup a webserver with an /update page
  web_ota_setup(server, &otaCredentials);
}

void loop() {
  // blink internal LED
  digitalWrite(led, ledState);
  ledState = not(ledState);
  delay(interval);
}
