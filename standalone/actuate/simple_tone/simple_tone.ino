/*
 This example generates a square wave based tone at a specified frequency
 and sample rate. Then outputs the data using the I2S interface to a DAC
 breakout board.
 created 17 November 2016
 by Sandeep Mistry
 For ESP extended
 Tomas Pilny
 2nd September 2021
 Lucas Saavedra Vaz (lucasssvaz)
 22nd December 2023
 anon
 10nd February 2025
 */

// Arduino IDE installs esp-arduino v3.x, see:
// AppData/Local/Arduino15/packages/esp32/hardware/esp32/3.3.4/libraries/ESP_I2S/src/ESP_I2S.h
// PlatformIO still uses esp-arduino v2.x, see:
// https://github.com/platformio/platform-espressif32/releases
#include <ESP_I2S.h>  // arduino-core 3.x
// #include <I2S.h>         // arduino-core 2.x

// The GPIO pins are not fixed, most other pins could be used for the I2S function
// This works on standalone ESP32-dev with only DAC attached and powerd by USB
#define I2S_LRC  33
#define I2S_BCLK 14
#define I2S_DIN  27

// On fietspaal hardwired. Checked does not work, because GPIO input only
// #define I2S_LRC  33
// #define I2S_BCLK 2
// #define I2S_DIN  35

const int frequency = 440;    // frequency of square wave in Hz
const int amplitude = 1000;   // amplitude of square wave
const int sampleRate = 8000;  // sample rate in Hz

i2s_data_bit_width_t bps = I2S_DATA_BIT_WIDTH_16BIT;
i2s_mode_t mode = I2S_MODE_STD;
i2s_slot_mode_t slot = I2S_SLOT_MODE_STEREO;

const unsigned int halfWavelength = sampleRate / frequency / 2;  // half wavelength of square wave

int32_t sample = amplitude;  // current sample value
unsigned int count = 0;

I2SClass i2s;

void setup() {
  Serial.begin(115200);
  Serial.println("I2S simple tone");

  i2s.setPins(I2S_BCLK, I2S_LRC, I2S_DIN);

  // start I2S at the sample rate with 16-bits per sample
  if (!i2s.begin(mode, sampleRate, bps, slot)) {
    Serial.println("Failed to initialize I2S!");
    while (1);  // do nothing
  }
}

void loop() {
  if (count % halfWavelength == 0) {
    // invert the sample every half wavelength count multiple to generate square wave
    sample = -1 * sample;
  }

  // Left channel, the low 8 bits then high 8 bits
  i2s.write(sample);
  i2s.write(sample >> 8);

  // Right channel, the low 8 bits then high 8 bits
  i2s.write(sample);
  i2s.write(sample >> 8);

  // increment the counter for the next sample
  count++;
}
