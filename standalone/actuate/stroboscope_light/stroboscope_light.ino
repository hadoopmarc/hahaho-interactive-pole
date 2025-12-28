 /*
  * This script drives the orange stroboscope light of the Interactive Pole, see e.g.
  * https://nl.aliexpress.com/item/1005008893148458.html
  * The script has the MCU drive a small electronic circuit including the stroboscope light, see:
  * kicad/driver-12V-1A-inverting/driver-12V-1A-inverting.png
  * The stroboscope light can handle 12V towards the collector of the BD137 (10V is fine too).
  * Do not forget to also connect the GND of the Arduino/ESP to the GND of the external 12V power supply.
  */
# include "esp32_wiring.h"

void setup() {
  Serial.begin(9600);
  Serial.println("Stroboscope light demo");
  esp32_wiring_setup();  // Implies setting internalLED and stroboscope GPIO pins as output
}

void loop() {
  digitalWrite(internalLED, !digitalRead(internalLED));
  digitalWrite(stroboscope, !digitalRead(stroboscope));
  delay(5000);
}
