#include <Arduino.h>
#include <FastLED.h>
#include <FastLED_NeoMatrix.h>
#include <DFRobotDFPlayerMini.h>

// -- PIN CONFIGURATIE --
#define LED_PIN 18          // Data pin voor de LED matrix
#define BUTTON_PIN 0        // BOOT-knop op de ESP32 (doorlopen teksten)
#define TRIGGER_PIN_4 5     // Externe schakelaar naar GND voor tekst 4 / 0004.mp3
#define RELAY_OUTPUT_PIN 19 // Uitgang: Standaard HIGH, 10s LOW bij trigger op pin 5
#define PIR_FRONT_PIN 34    // PIR sensor voorkant -> Tekst 2 / 0002.mp3 (Input-only pin)
#define PIR_BACK_PIN 35     // PIR sensor achterkant -> Tekst 3 / 0003.mp3 (Input-only pin)

#define DFPLAYER_RX 16      // ESP32 RX2 -> Verbinden met TX van DFPlayer
#define DFPLAYER_TX 17      // ESP32 TX2 -> Verbinden met RX van DFPlayer (via 1k weerstand)

// -- OBJECTEN --
HardwareSerial dfSerial(2);
DFRobotDFPlayerMini myDFPlayer;

const uint16_t matrix_width = 64; 
const uint16_t matrix_height = 8;
const uint16_t NUM_LEDS = matrix_width * matrix_height; 

const int FONT_HEIGHT = 8; 
const int VERTICAL_SPACING = 8; 
const int CENTER_X = 1; 

// -- KEUZE UIT 4 TEKSTEN --
const String messages[4] = {
    "Hallo, is daar iemand?",
    "Hallo, ik zie iemand aan de voorkant, kom verder en druk op de knop",
    "Hallo, ik zie iemand aan de achterkant, kom verder en druk op de knop",
    "Wat goed, je hebt op de knop gedrukt, je bent de eerste vandaag",
};

int currentMessageIndex = 0;
String displayText = messages[0];

// -- GLOBALE VARIABELEN --
int scrollSpeed = 60;
int y_pos;
unsigned long previousScrollTime = 0;

// Timer voor inactiviteit (30 seconden)
unsigned long lastActivityTime = 0;
const unsigned long idleTimeout = 30000; 

// Timer voor GPIO 19 puls (10 seconden LOW)
unsigned long pin19LowStartTime = 0;
bool isPin19ActiveLow = false;
const unsigned long OUTPUT_PULSE_DURATION = 10000; // 10 seconden

uint8_t currentBrightness = 100;
uint8_t mp3Volume = 15; // Bereik: 0 t/m 30

CRGB currentColor = CRGB::OrangeRed;
CRGB leds[NUM_LEDS]; 

FastLED_NeoMatrix matrix(leds, matrix_width, matrix_height, 1, 1, 
                         NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG);

uint16_t convertCRGBtoRGB565(const CRGB &c) {
    return ((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3);
}

void selectText(int index) {
    if (index >= 0 && index < 4) {
        currentMessageIndex = index;
        displayText = messages[currentMessageIndex];
        y_pos = matrix.height(); 
        
        int trackNumber = currentMessageIndex + 1; // 1 t/m 4
        myDFPlayer.playMp3Folder(trackNumber);

        // Reset inactiviteitstimer bij elke activiteit
        lastActivityTime = millis();

        Serial.print(">>> GESELECTEERD: Tekst [");
        Serial.print(trackNumber);
        Serial.print("] -> \"");
        Serial.print(displayText);
        Serial.println("\" (Track /mp3/000X.mp3 gestart)");
    }
}

void drawScrollingText() {
    unsigned long currentTime = millis();
    if (currentTime - previousScrollTime >= scrollSpeed) {
        previousScrollTime = currentTime;
        
        matrix.fillScreen(0);
        
        int char_x = CENTER_X; 
        int char_y = y_pos; 
        
        for (int i = 0; i < displayText.length(); i++) {
            matrix.setCursor(char_x, char_y);
            matrix.print(displayText[i]);
            char_y += VERTICAL_SPACING; 
        }

        if (--y_pos < -(int)(VERTICAL_SPACING * displayText.length())) {
            y_pos = matrix.height(); 
        }
        
        matrix.show();
        FastLED.show();
    }
}

// Bewaakt GPIO 19 en zet deze na 10 seconden weer op HIGH
void handleOutputPin() {
    if (isPin19ActiveLow) {
        if (millis() - pin19LowStartTime >= OUTPUT_PULSE_DURATION) {
            digitalWrite(RELAY_OUTPUT_PIN, HIGH);
            isPin19ActiveLow = false;
            Serial.println("GPIO 19 hersteld naar -> HIGH");
        }
    }
}

// Controleert of er 30 seconden verstreken zijn zonder triggers
void handleIdleTimer() {
    if (millis() - lastActivityTime >= idleTimeout) {
        Serial.println("Geen activiteit gedurende 30s -> Automatisch terug naar Tekst 1 (0001.mp3)");
        selectText(0); // Index 0 = Tekst 1 / 0001.mp3
    }
}

// Ingangen uitlezen (Knoppen & PIR Sensoren)
void handleInputs() {
    const unsigned long debounceDelay = 50;

    // --- 1. BOOT knop (GPIO 0 - Actief LOW) ---
    static int lastBootState = HIGH;
    static unsigned long lastBootDebounce = 0;
    static int bootState = HIGH;

    int readingBoot = digitalRead(BUTTON_PIN);
    if (readingBoot != lastBootState) {
        lastBootDebounce = millis();
    }
    if ((millis() - lastBootDebounce) > debounceDelay) {
        if (readingBoot != bootState) {
            bootState = readingBoot;
            if (bootState == LOW) {
                int nextIndex = (currentMessageIndex + 1) % 4;
                selectText(nextIndex);
            }
        }
    }
    lastBootState = readingBoot;

    // --- 2. Schakelaar GPIO 5 (Actief LOW -> Tekst 4 / 0004.mp3 & GPIO 19 LOW) ---
    static int lastPin5State = HIGH;
    static unsigned long lastPin5Debounce = 0;
    static int pin5State = HIGH;

    int readingPin5 = digitalRead(TRIGGER_PIN_4);
    if (readingPin5 != lastPin5State) {
        lastPin5Debounce = millis();
    }
    if ((millis() - lastPin5Debounce) > debounceDelay) {
        if (readingPin5 != pin5State) {
            pin5State = readingPin5;
            if (pin5State == LOW) {
                Serial.println("GPIO 5 ingedrukt -> Start Tekst 4 en zet GPIO 19 LOW voor 10s");
                selectText(3); // Index 3 = Tekst 4 / 0004.mp3
                
                // GPIO 19 direct LOW schakelen en timer starten
                digitalWrite(RELAY_OUTPUT_PIN, LOW);
                pin19LowStartTime = millis();
                isPin19ActiveLow = true;
            }
        }
    }
    lastPin5State = readingPin5;

    // --- 3. PIR Voorkant (GPIO 34 - Actief HIGH -> Tekst 2 / 0002.mp3) ---
    static int lastPirFrontState = LOW;
    int readingPirFront = digitalRead(PIR_FRONT_PIN);
    if (readingPirFront == HIGH && lastPirFrontState == LOW) {
        Serial.println("PIR Voorkant beweging gedetecteerd (GPIO 34) -> Start Tekst 2");
        selectText(1); // Index 1 = Tekst 2 / 0002.mp3
    }
    lastPirFrontState = readingPirFront;

    // --- 4. PIR Achterkant (GPIO 35 - Actief HIGH -> Tekst 3 / 0003.mp3) ---
    static int lastPirBackState = LOW;
    int readingPirBack = digitalRead(PIR_BACK_PIN);
    if (readingPirBack == HIGH && lastPirBackState == LOW) {
        Serial.println("PIR Achterkant beweging gedetecteerd (GPIO 35) -> Start Tekst 3");
        selectText(2); // Index 2 = Tekst 3 / 0003.mp3
    }
    lastPirBackState = readingPirBack;
}

void handleSerial() {
    while (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming == '\r' || incoming == '\n' || incoming == ' ') {
            continue;
        }
        if (incoming >= '1' && incoming <= '4') {
            int index = incoming - '1';
            selectText(index);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // GPIO 19 als uitgang configureren en direct HIGH zetten
    pinMode(RELAY_OUTPUT_PIN, OUTPUT);
    digitalWrite(RELAY_OUTPUT_PIN, HIGH);

    // Knoppen (met interne pull-up)
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(TRIGGER_PIN_4, INPUT_PULLUP);

    // PIR sensoren
    pinMode(PIR_FRONT_PIN, INPUT);
    pinMode(PIR_BACK_PIN, INPUT);

    // DFPlayer UART
    dfSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);

    Serial.println("\n--- Systeem Start ---");
    Serial.println("GPIO 19 ingesteld op HIGH");

    if (!myDFPlayer.begin(dfSerial, false, false)) {
        Serial.println("Let op: DFPlayer reageert niet direct.");
    } else {
        Serial.println("DFPlayer gereed.");
    }
    
    myDFPlayer.setTimeOut(200);
    myDFPlayer.volume(mp3Volume);

    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(currentBrightness);
    FastLED.clear();

    matrix.begin();
    matrix.setRotation(1);
    matrix.setTextWrap(false);
    matrix.setTextColor(convertCRGBtoRGB565(currentColor));
    matrix.setFont();
    y_pos = matrix.height(); 

    Serial.println("Klaar voor sensoren en knoppen.");
    selectText(0); // Start direct met tekst 1 / 0001.mp3
}

void loop() {
    handleInputs();
    handleOutputPin();
    handleSerial();
    handleIdleTimer();
    drawScrollingText(); 
}