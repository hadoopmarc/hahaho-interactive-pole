
/*
Simple demo how to use Webserial on ESP32:
- copy secrets-example.h to secrets.h and fill in SSID and PASSWORD
- upload the script to your ESP32 and note the web url printed to the Arduino IDE serial monitor
- the actual IP address in the web url depends on the configured Wi-Fi network
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>  // Install "ESP Aync WebServer" and "Async TCP" by ESP32Async
#include <AsyncWebSerial.h>     // Install "AsyncWebSerial" by Sessa
// See src/libraries/README.md for use in Arduino IDE
#include <secrets.h>
#include <pole_wifi.h>

AsyncWebSerial webSerial;
AsyncWebServer server(80);

int i = 0;  // Used for progressing output


void setup() {
  Serial.begin(115200);
  wifi_setup(btnCredentials, btnCredsSize, &apCredentials);
  Serial.println("Open webpage at /webserial");
  webSerial.begin(&server);
  server.begin();
}


void loop()
{
    webSerial.print(i++);
    webSerial.loop();
    delay(1000);
}