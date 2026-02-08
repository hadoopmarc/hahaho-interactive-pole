/*
This script wraps the neopixel_vertical library for the HaHaHo Interactive Pole
in a standalalone webapp to experiment with the NeoPixel matrix display.

The script keeps restarting until an SSID + password to a WiFi access point is
entered at the web page at http://192.168.4.1 (reachable via the esp32 acting
as access point itself).

When using the Arduino IDE, see the lib_deps in platformio.ini for the
the libraries to be installed with the library manager.
*/

#include <WiFi.h>
#include <WiFiManager.h>
#define WEBSERVER_H  // Prevent redefinitions done by WiFiManager, see:
                     // https://stackoverflow.com/questions/75043892/i-am-facing-http-get-conflicts-with-a-previous-declaration-error-with-the-wifi
#include <ESPAsyncWebServer.h>
#include "neopixel_vertical.h"

// -- OBJECTEN & CONFIGURATIE --
AsyncWebServer server(80);

const uint16_t matrix_width = 64; 
const uint16_t matrix_height = 8;
const uint16_t NUM_LEDS = matrix_width * matrix_height; 

String displayText = "WiFi Connected!";

uint8_t currentBrightness = 100; // Helderheid iets verhoogd voor zichtbaarheid
CRGB currentColor = CRGB::OrangeRed;

// Functies om de web requests af te handelen
void handleRoot(AsyncWebServerRequest *request) {
    request->send(200, "text/html", generateHTML());
}

void handleSet(AsyncWebServerRequest *request) {
    if (request->hasArg("text")) {
        displayText = request->arg("text");
        if (displayText == "") displayText = " ";
        setNeoPixelText(displayText);
    }
    if (request->hasArg("brightness")) {
        currentBrightness = request->arg("brightness").toInt();
        setNeoPixelBrightness(currentBrightness);
    }
    if (request->hasArg("color")) {
        String colorStr = request->arg("color");
        long number = (long)strtol(&colorStr[1], NULL, 16);
        setNeoPixelColor(number);
    }
    request->redirect("/", 303);
}

void setup() {
    Serial.begin(115200);

    // FastLED setup
    setupNeoPixel();

    // Start WiFiManager
    WiFiManager wm;
    if (!wm.autoConnect("ESP32-Matrix-Setup")) {
        Serial.println("Failed to connect and hit timeout");
        ESP.restart();
    }
    
    Serial.println("\nWiFi verbonden!");
    Serial.print("IP-adres: ");
    Serial.println(WiFi.localIP());

    // Start de webserver
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        handleRoot(request);
    });
    server.on("/set", HTTP_POST, [](AsyncWebServerRequest *request) {
        handleSet(request);
    });
    server.begin();
}

void loop() {
    drawScrollingText(); 
}

// Genereert de HTML voor de webpagina
String generateHTML() {
    String html = R"rawliteral(
    <!DOCTYPE html><html><head><title>ESP32 Matrix Editor</title><meta name="viewport" content="width=device-width, initial-scale=1"><style>body{font-family:Arial,sans-serif;background-color:#f0f2f5;margin:0;padding:20px;display:flex;justify-content:center}.container{background-color:white;padding:30px;border-radius:10px;box-shadow:0 4px 8px rgba(0,0,0,0.1);max-width:500px;width:100%}h1{color:#333;text-align:center}form{display:flex;flex-direction:column;gap:20px}label{font-weight:bold;color:#555}input[type=text],input[type=color],input[type=range]{width:100%;padding:12px;border-radius:5px;border:1px solid #ccc;box-sizing:border-box}input[type=color]{height:50px;padding:5px}input[type=submit]{background-color:#007bff;color:white;padding:15px;border:none;border-radius:5px;cursor:pointer;font-size:16px;transition:background-color .3s}input[type=submit]:hover{background-color:#0056b3}.range-container{display:flex;align-items:center;gap:15px}#brightness-value{font-weight:bold}</style></head><body><div class="container"><h1>LED Matrix Editor</h1><form action="/set" method="POST"><div><label for="text">Tekst:</label><input type="text" id="text" name="text" value="%TEXT%"></div><div><label for="color">Kleur:</label><input type="color" id="color" name="color" value="%COLOR%"></div><div class="range-container"><label for="brightness">Helderheid:</label><input type="range" id="brightness" name="brightness" min="5" max="200" value="%BRIGHTNESS%" oninput="document.getElementById('brightness-value').innerText=this.value"><span id="brightness-value">%BRIGHTNESS%</span></div><input type="submit" value="Update Display"></form></div></body></html>
    )rawliteral";

    html.replace("%TEXT%", displayText);
    html.replace("%BRIGHTNESS%", String(currentBrightness));
    char hexColor[8];
    sprintf(hexColor, "#%06X", (currentColor.r << 16) | (currentColor.g << 8) | currentColor.b);
    html.replace("%COLOR%", String(hexColor));
    return html;
}
