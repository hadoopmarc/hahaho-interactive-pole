/*
Uploading new binaries to the ESP32 over Wi-Fi will speed up the development
*/
#include <arduino.h>                             // Only relevant for VSCode/PlatformIO Intellisense
#include <web_ota.h>                             // From src/libraries installed in Arduino/Platform libraries
#include "secrets.h"

const int led = 2;                               // ESP32 Pin to which onboard LED is connected
const long interval = 500;                       // interval at which to blink (milliseconds)
int ledState = LOW;                              // state of the LED to be negated in each loop cycle


void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  // This includes setup of a Wi-Fi connection and a webserver with an /update page
  web_ota_setup(btnCredentials, sizeof(btnCredentials) / sizeof(Credentials), &apCredentials, &otaCredentials);
}

void loop() {
  // blink internal LED
  digitalWrite(led, ledState);
  ledState = not(ledState);
  delay(interval);
}
