/* Arduino tutorial - Buzzer / Piezo Speaker
   More info and circuit: http://www.ardumotive.com/how-to-use-a-buzzer-en.html
   Dev: Michalis Vasilakis // Date: 9/6/2015 // www.ardumotive.com
*/

#include <esp32_wiring.h>  // Implies setting buzzer - GPIO D4 as an output

void setup() {
}

void loop() {
  tone(buzzer, 440);  // Send 440 Hz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(1000);        // ...for 1sec
}
