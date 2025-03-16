int ledPin = 2;
 
 void setup() {
  Serial.begin(9600);
  Serial.println("\n\nHello, HaHaHo Interactive Pole");
  pinMode(ledPin, OUTPUT);
 }
 
 void loop() {
  digitalWrite(ledPin, !digitalRead(ledPin));
  delay(500);
 }
 