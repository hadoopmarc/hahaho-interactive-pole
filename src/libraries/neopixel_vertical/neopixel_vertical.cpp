// When using the Arduino IDE, see the lib_deps in platformio.ini for the
// the libraries to be installed with the library manager.
#include <FastLED.h>
#include <FastLED_NeoMatrix.h>
#include <esp32_wiring.h>  // Implies setting neoPixel GPIO port

const uint16_t matrix_width = 64;
const uint16_t matrix_height = 8;
const uint16_t NUM_LEDS = matrix_width * matrix_height;

const int FONT_HEIGHT = 8;
// Vertical distance between characters (5 pixels for the characters + 3 pixels spacing = 8)
const int VERTICAL_SPACING = 8;
// X-position for centering: (8 - 5) / 2 = 1
const int CENTER_X = 1;

// -- GLOBAL VARIABLES --
String neoText = "HaHaHo Interactive Pole started!";
int scrollSpeed = 60;
int y_pos;// To be used for vertical scrolling
unsigned long previousScrollTime = 0;

CRGB leds[NUM_LEDS];

// Mapper configuration for vertically oriented screen (BOTTOM/RIGHT/COLUMNS/ZIGZAG)
FastLED_NeoMatrix matrix(
    leds, matrix_width, matrix_height, 1, 1,
    NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG
);


// Convert 24-bit CRGB color to 16-bit RGB565
uint16_t convertCRGBtoRGB565(const CRGB &c) {
    return ((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3);
}


void setNeoPixelText(String text) {
    neoText = text;
    if (neoText == "") neoText = " ";
    // Reset y_pos to matrix height (8 pixels)
    y_pos = matrix.height();
}


void setNeoPixelBrightness(int brightness) {
    FastLED.setBrightness(brightness);
}


void setNeoPixelColor(long number) {
    CRGB color = CRGB(number);
    matrix.setTextColor(convertCRGBtoRGB565(color));
}

// Non-blocking scrolling text function
void drawScrollingText() {
    unsigned long currentTime = millis();
    if (currentTime - previousScrollTime >= scrollSpeed) {
        previousScrollTime = currentTime;

        matrix.fillScreen(0);

        int char_x = CENTER_X; // text centering
        int char_y = y_pos;

        // Run through all characters
        for (int i = 0;i < neoText.length();i++) {
            matrix.setCursor(char_x, char_y);
            matrix.print(neoText[i]);
            // Shift the vertical position with the VERTICAL_SPACING
            char_y += VERTICAL_SPACING;
        }
        // Reset y_pos when entire text has shifted from the display
        if (--y_pos < -(int)(VERTICAL_SPACING * neoText.length())) {
            y_pos = matrix.height();
        }
        // Effectuate the printed characters
        matrix.show();
        FastLED.show();
    }
}


void setupNeoPixel() {
    // FastLED setup
    FastLED.addLeds<NEOPIXEL, neoPixel>(leds, NUM_LEDS);
    FastLED.setBrightness(100);
    FastLED.clear();

    // Matrix setup
    matrix.begin();
    matrix.setRotation(1);// 90 degrees rotation
    matrix.setTextWrap(false);
    matrix.setTextColor(convertCRGBtoRGB565(CRGB::OrangeRed));
    matrix.setFont();
    y_pos = matrix.height();
}
