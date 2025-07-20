/*
Uploading new binaries to the ESP32 over Wi-Fi will speed up the development
*/
#include "secrets.h"
#include "web_ota.h"

const int led = 2;                               // ESP32 Pin to which onboard LED is connected
const long interval = 200;                       // interval at which to blink (milliseconds)
int ledState = LOW;                              // state of the LED to be negated in each loop cycle


void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  ota_setup(btnCredentials, &apCredentials, &otaCredentials);
}

void loop() {
  ota_loop();

  // blink internal LED
  digitalWrite(led, ledState);
  ledState = not(ledState);
  delay(interval);
}
