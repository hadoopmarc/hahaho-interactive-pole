 /*
  * This script drives the orange stroboscope light of the Interactive Pole, see e.g.
  * https://nl.aliexpress.com/item/1005008893148458.html
  * The script has the MCU drive a small electronic circuit including the stroboscope light, see:
  * kicad/driver-12V-1A-inverting/driver-12V-1A-inverting.png
  * The stroboscope light can handle 12V towards the collector of the BD137 (10V is fine too).
  * Do not forget to also connect the GND of the Arduino/ESP to the GND of the external 12V power supply.
  */

int ledPin = 13;
int drivePin = 2;

void setup() {
  Serial.begin(9600);
  Serial.println("Stroboscope light demo");
  pinMode(ledPin, OUTPUT);
  pinMode(drivePin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, !digitalRead(ledPin));
  digitalWrite(drivePin, !digitalRead(drivePin));
  delay(5000);
}
