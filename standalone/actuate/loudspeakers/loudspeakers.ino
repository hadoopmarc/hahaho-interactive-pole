/*
This script uses the SPIFFS file system (https://github.com/espressif/arduino-esp32/tree/master/libraries/SPIFFS)
and assumes that the data folder in the script folder is built as image and uploaded
to the ESP32. In the PlatformIO IDE the following esp32dev Platform project tasks are available
for that:
1. Build Filesystem Image
2. Upload Filesystem Image
*/
#include "SPIFFS.h"

char buffer[100];  // Used for formatted print


void setup() {
  Serial.begin(115200);
  Serial.println("Hello!");
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
  showAllFiles();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
}

void showAllFiles()
{
  File root = SPIFFS.open ( "/" );                       // Open root directory
  File file;                                             // File in directory

  Serial.println("SPIFFS contents:");
  while (file = root.openNextFile())                    // Iterate through files
  {
    snprintf(buffer, sizeof(buffer), "%5d %s", file.size(), file.path());
    Serial.println(buffer);
  }
}
