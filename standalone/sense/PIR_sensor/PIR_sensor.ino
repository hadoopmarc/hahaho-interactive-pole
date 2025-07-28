/*
HC-SR501 PIR sensor module
https://nl.aliexpress.com/item/1005006834678128.html
https://nl.aliexpress.com/item/1005006227628663.html)

Looking at the header pins, with the white cap under:
-left pin: 3v3
-center pin: HIGH/LOW signal
-right pin: GND

Jumper position:
- at the edge = non-repeatable mode (sensor always returns to LOW after delay time)
- from the edge = repeatable mode (sensor can be retriggered and stay high)

Looking at the orange variable resistors, with the white cap under:
- left resistor: distance adjustment (best setting still to be determined)
- right resistor: delay adjustment (best setting to be determined)
*/
int ledPin = 2;  // internal LED of esp32-dev-module
int sensorPin = 35;
int sensorState = LOW;
int sensorValue = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(sensorPin, INPUT);
}

void loop(){
  sensorValue = digitalRead(sensorPin);
  if (sensorValue == HIGH) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    
    if (sensorState == LOW) {
      Serial.println("Motion detected!");
      sensorState = HIGH;
    }
  } 
  else {
      digitalWrite(ledPin, LOW);
      delay(200);
      
      if (sensorState == HIGH){
        Serial.println("Motion stopped!");
        sensorState = LOW;
    }
  }
}
