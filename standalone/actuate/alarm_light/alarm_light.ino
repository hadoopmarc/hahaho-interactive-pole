 /*
  * This script drives the orange alarm light of the Interactive Pole
  * It assumes it drives a BD137 NPN transistor via a 100 Ohm resistor (giving 5 mA of drive current)
  * The alarm light can handle 12V towards the collector of the BD137 (10V is fine to)
  * The emitter of the BD137 can be connected to GND
  * Do not forget to also connect the GND of the Arduino/ESP to the GND
  * of the external 12V power supply.
  * The BD137 has a limited current gain of about 50. To give maximum light one should get a
  * transistor with higher current gain or use a smaller base transistor.
  */
 
 int ledPin = 13;
 int drivePin = 2;
 
 void setup() {
  Serial.begin(9600);
  Serial.println("Alarm light demo");
  pinMode(ledPin, OUTPUT);
  pinMode(drivePin, OUTPUT);
 }
 
 void loop() {
  digitalWrite(ledPin, !digitalRead(ledPin));
  digitalWrite(drivePin, !digitalRead(drivePin));
  delay(2000);
 }
 