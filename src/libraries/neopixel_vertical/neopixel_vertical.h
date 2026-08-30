#include <FastLED.h>

#ifndef NEOPIXEL_VERTICAL_H
#define NEOPIXEL_VERTICAL_H

void setupNeoPixel();

void setNeoPixelText(String text);
void setNeoPixelBrightness(int brightness);
void setNeoPixelColor(long number);
void drawScrollingText();  // Non-blocking scrolling text function

#endif // NEOPIXEL_VERTICAL_H
