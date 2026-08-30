/*
This is an unimaginative demo script for the HaHaHo Interactive Pole that just demonstrates
all available functionalities. These functionalities are:
 - auto login to a WiFi access point with preconfigured credentials (src/libraries/Interactive-Pole-Utils/secrets.cpp)
 - a web page for OTA firmware updates
 - a web dashboard with sensor measurements and states
 - a web page with a serial monitor
 - a red button used to toggle states for one separate push, two successive pushes and three successive pushes
 - a front and a back PIR sensor
 - a buzzer
 - playing small mp3 files with the esp32
 - playing large mp3 files with a built-in mp3 player module
 - displaying texts on a vertical neopixel matrix display
 - a stroboscope alarm light

After powering on the HaHaHo Interactive Pole and the RPi access point for the publicroam WiFi one can do the following:
 - see all sensor states at http://192.168.?.?/sensors
 - see all monitor output at http://192.168.?.?/serial
 - see the OTA update option at http://192.168.?.?/update
 - push the button once, hear the buzzer inside the pole (very softly) and see the display "once:on twice:off thrice:off"
 - push the button once again and the buzzer stops
 - push the button twice in succession, hear a short mp3 file playing and see the display "once:off twice:on thrice:off"
 - push the button thrice in succession and hear a long mp3 file playin and see the display "once:off twice:on thrice:on"
 - try and see how many sounds you can get playing in parallel...
 
See src/libraries/README.md for building this sketch in the Arduinio IDE instead of in PlatformIO
*/

#include <arduino.h>                       // Only relevant for VSCode/PlatformIO Intellisense
#include <PoleFirmwareBase.h>

PoleFirmwareBase pf;

void setup() {
  Serial.begin(115200);                    // For use by both the script and the libraries
  pf.setup();
}

// All function calls should be non-blocking (do not use the delay() function)
void loop() {
  pf.sense_loop();

  // Add demo logic

  pf.act_loop();
}
