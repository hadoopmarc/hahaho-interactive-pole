/*
This is the version provided by Herman. For inclusion in the Interactive Pole, the following
still has to be done:
 - rotate the moving characters by 90 degrees for a vertical display
 - port the webserver to the ESPAsyncWebServer used by the rest of the project
*/
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <FastLED.h>
#include <FastLED_NeoMatrix.h>

// -- OBJECTEN & CONFIGURATIE --
WebServer server(80); 

#define LED_PIN 13
// << WIJZIGING 1: De totale breedte is nu 64 pixels >>
const uint16_t matrix_width = 64; 
const uint16_t matrix_height = 8;
const uint16_t NUM_LEDS = matrix_width * matrix_height; // Dit wordt nu automatisch 512

// -- GLOBALE VARIABELEN --
String displayText = "WiFi Connected!";
int scrollSpeed = 60;
int x_pos;
unsigned long previousScrollTime = 0;

uint8_t currentBrightness = 30;
CRGB currentColor = CRGB::OrangeRed;

CRGB leds[NUM_LEDS]; // Deze array is nu automatisch 512 LEDs groot

// << WIJZIGING 2: Vertel de matrix zijn NIEUWE totale afmetingen >>
// We gebruiken nu de variabelen, wat netter is.
FastLED_NeoMatrix matrix(leds, matrix_width, matrix_height, 1, 1, 
                        NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG);

// Functie om 24-bit CRGB kleur om te zetten naar 16-bit RGB565
uint16_t convertCRGBtoRGB565(const CRGB &c) {
    return ((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3);
}

// Genereert de HTML voor de webpagina
String generateHTML() {
    // Deze functie hoeft NIET aangepast te worden.
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

// Functies om de web requests af te handelen
void handleRoot() {
    server.send(200, "text/html", generateHTML());
}

void handleSet() {
    if (server.hasArg("text")) {
        displayText = server.arg("text");
        if (displayText == "") displayText = " ";
        // Deze regel pakt nu automatisch de nieuwe breedte van 64
        x_pos = matrix.width(); 
    }
    if (server.hasArg("brightness")) {
        currentBrightness = server.arg("brightness").toInt();
        FastLED.setBrightness(currentBrightness);
    }
    if (server.hasArg("color")) {
        String colorStr = server.arg("color");
        long number = (long)strtol(&colorStr[1], NULL, 16);
        currentColor = CRGB(number);
        matrix.setTextColor(convertCRGBtoRGB565(currentColor));
    }
    server.sendHeader("Location", "/");
    server.send(303);
}

// Non-blocking scrolling text function
void drawScrollingText() {
    unsigned long currentTime = millis();
    if (currentTime - previousScrollTime >= scrollSpeed) {
        previousScrollTime = currentTime;
        
        matrix.fillScreen(0);
        // Deze regel is aangepast (van 7 naar 0) in ons vorige gesprek
        matrix.setCursor(x_pos, 0); 
        matrix.print(displayText);
        
        // Deze 'if' statement werkt nu automatisch met de nieuwe breedte van 64
        if (--x_pos < -(int)(displayText.length() * 6)) {
            x_pos = matrix.width(); // matrix.width() is nu 64
        }
        
        matrix.show();
        FastLED.show();
    }
}

// -- SETUP & LOOP --
void setup() {
    Serial.begin(115200);

    // FastLED is nu geconfigureerd voor 512 LEDs
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(currentBrightness);
    FastLED.clear();

    // Matrix is nu geconfigureerd voor 64x8
    matrix.begin();
    matrix.setTextWrap(false);
    matrix.setTextColor(convertCRGBtoRGB565(currentColor));
    matrix.setFont();
    x_pos = matrix.width(); // x_pos start nu op 64

    // Start WiFiManager
    WiFiManager wm;
    // wm.resetSettings(); 
    if (!wm.autoConnect("ESP32-Matrix-Setup")) {
        Serial.println("Failed to connect and hit timeout");
        ESP.restart();
    }
    
    Serial.println("\nWiFi verbonden!");
    Serial.print("IP-adres: ");
    Serial.println(WiFi.localIP());

    // Start de webserver
    server.on("/", HTTP_GET, handleRoot);
    server.on("/set", HTTP_POST, handleSet);
    server.begin();
}

void loop() {
    server.handleClient(); 
    drawScrollingText();   
}