/*
This version of the HaHaHo Interactive Pole combines the following standalone scripts:
https://github.com/hadoopmarc/hahaho-interactive-pole/tree/main/standalone/actuate/buzzer
https://github.com/hadoopmarc/hahaho-interactive-pole/tree/main/standalone/sense/red_button

See src/libraries/README.md for building this sketch in the Arduinio IDE instead of in PlatformIO
*/
#include <esp32_wiring.h>

void setup() {
  Serial.begin(115200);
  esp32_wiring_setup();
  sleep(1);
  Serial.println("\n\nHello, HaHaHo Interactive Pole");
}

void loop() {
  bool buttonActive = !digitalRead(redButton);
  Serial.println(buttonActive);
  if (buttonActive) {
    digitalWrite(internalLED, HIGH);
    tone(buzzer, 1000); // 1KHz sound signal
    delay(1000);
  } else {
    digitalWrite(internalLED, LOW);
    noTone(buzzer);
    delay(1000);
  }
}
