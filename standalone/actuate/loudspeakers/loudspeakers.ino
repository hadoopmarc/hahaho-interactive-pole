/*
This script uses the SPIFFS file system (https://github.com/espressif/arduino-esp32/tree/master/libraries/SPIFFS)
and assumes that the data folder in the script folder is built as image and uploaded
to the ESP32. In the PlatformIO IDE the following esp32dev Platform project tasks are available
for that:
1. Build Filesystem Image
2. Upload Filesystem Image

Arduino IDE 2.x does not support SPIFFS, only LittleFS, see:
      https://randomnerdtutorials.com/arduino-ide-2-install-esp32-littlefs/
ToDo: migrate to LittleFS so that it works with both IDE's, see:
      https://randomnerdtutorials.com/esp32-vs-code-platformio-littlefs/
*/
#include <string.h>
#include <SPIFFS.h>
#include "esp32_wiring.h"  // Implies setting internalLED and I2S GPIO ports
#include "mp3_player.h"

char buffer[100];  // Used for formatted print


void setup() {
  pinMode(internalLED, OUTPUT);
  Serial.begin(115200);
  if(!SPIFFS.begin(true)) {
    Serial.println("An Error occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
  showAllFiles();
  setup_mp3();
}


void loop() {
  File root = SPIFFS.open("/");                       // Open root directory
  File file;                                          // File in directory

  while((file = root.openNextFile()))                 // Iterate through files
  {
    Serial.println(file.path());
    Serial.println(sizeof(file.path()));
    if (strstr(file.path(), ".mp3")) {
      play_mp3(file.path());
      delay(3000);
    }
  }
}


void showAllFiles() {
  File root = SPIFFS.open("/");                       // Open root directory
  File file;                                          // File in directory
  int n = 0;

  Serial.println("SPIFFS contents:");
  while((file = root.openNextFile()))                 // Iterate through files
  {
    snprintf(buffer, sizeof(buffer), "%5d %s", file.size(), file.path());
    Serial.println(buffer);
    n++;
  }
  if (n == 0) {
    Serial.println("No files found; be sure to also upload the filesystem image!");
  }
}
