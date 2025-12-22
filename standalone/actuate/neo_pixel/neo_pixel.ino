// When using the Arduino IDE, see the lib_deps in platformio.ini for the
// the libraries to be installed with the library manager.
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <FastLED.h>
#include <FastLED_NeoMatrix.h>
#include <esp32_wiring.h>  // Implies setting neoPixel GPIO port

// -- OBJECTEN & CONFIGURATIE --
WebServer server(80); 

const uint16_t matrix_width = 64; 
const uint16_t matrix_height = 8;
const uint16_t NUM_LEDS = matrix_width * matrix_height; 

// Constante font hoogte gedefinieerd om de compiler fout op te lossen (standaard 8 pixels)
const int FONT_HEIGHT = 8; 
// Verticale afstand tussen de karakters (5 pixels letter + 3 pixels spatie = 8)
const int VERTICAL_SPACING = 8; 
// X-positie voor centrering: (8 - 5) / 2 = 1.
const int CENTER_X = 1; 

// -- GLOBALE VARIABELEN --
String displayText = "WiFi Connected!";
int scrollSpeed = 60;
int y_pos; // Gebruik y_pos voor verticale scrolling
unsigned long previousScrollTime = 0;

uint8_t currentBrightness = 100; // Helderheid iets verhoogd voor zichtbaarheid
CRGB currentColor = CRGB::OrangeRed;

CRGB leds[NUM_LEDS]; 

// Werkende mapper-configuratie voor geroteerd scherm (BOTTOM/RIGHT/COLUMNS/ZIGZAG)
FastLED_NeoMatrix matrix(leds, matrix_width, matrix_height, 1, 1, 
                         NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG);

// Functie om 24-bit CRGB kleur om te zetten naar 16-bit RGB565
uint16_t convertCRGBtoRGB565(const CRGB &c) {
    return ((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3);
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

// Functies om de web requests af te handelen
void handleRoot() {
    server.send(200, "text/html", generateHTML());
}

void handleSet() {
    if (server.hasArg("text")) {
        displayText = server.arg("text");
        if (displayText == "") displayText = " ";
        // Reset y_pos naar de hoogte (8 pixels)
        y_pos = matrix.height(); 
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
        
        // Gebruik de constante X-positie voor centrering
        int char_x = CENTER_X; 
        int char_y = y_pos; 
        
        // Loop door elk karakter in de string
        for (int i = 0; i < displayText.length(); i++) {
            matrix.setCursor(char_x, char_y);
            matrix.print(displayText[i]);
            
            // Verschuif de Y-positie met de gedefinieerde VERTICAL_SPACING
            char_y += VERTICAL_SPACING; 
        }

        // Decrement y_pos. Reset als de hele kolom karakters van het scherm is
        if (--y_pos < -(int)(VERTICAL_SPACING * displayText.length())) {
            y_pos = matrix.height(); 
        }
        
        // Deze show-functies moeten de LED's data sturen
        matrix.show();
        FastLED.show();
    }
}

// -- SETUP & LOOP --
void setup() {
    Serial.begin(115200);

    // FastLED setup
    FastLED.addLeds<NEOPIXEL, neoPixel>(leds, NUM_LEDS);
    FastLED.setBrightness(currentBrightness);
    FastLED.clear();

    // Matrix setup
    matrix.begin();
    matrix.setRotation(1); // 90 graden rotatie
    matrix.setTextWrap(false);
    matrix.setTextColor(convertCRGBtoRGB565(currentColor));
    matrix.setFont();
    y_pos = matrix.height(); 

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
    server.on("/", HTTP_GET, handleRoot);
    server.on("/set", HTTP_POST, handleSet);
    server.begin();
}

void loop() {
    server.handleClient(); 
    drawScrollingText(); 
}