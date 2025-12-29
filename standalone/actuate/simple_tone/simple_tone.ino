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

// This script with ESP_I2S.h needs arduino-esp32 v3.x with ESP_I2S.h
// This can be installed with the Arduino IDE, see:
// AppData/Local/Arduino15/packages/esp32/hardware/esp32/3.3.4/libraries/ESP_I2S/src/ESP_I2S.h
// or with the pioarduino clone of PlatformIO, see:
// https://github.com/pioarduino/platform-espressif32
// PlatformIO itself still uses arduino-esp v2.x with I2S.h, see:
// https://github.com/platformio/platform-espressif32/releases

// The PCM5102 breakout board may need solder bridges,
// e.g. see https://github.com/pschatzmann/arduino-audio-tools/issues/773
// The one tested showed stereo output on the LOUT and ROUT pins, but only
// had output on one of the channels of the jack connector only.

#include <ESP_I2S.h>          // arduino-core 3.x
#include <esp32_wiring.h>

const int frequency = 55;     // frequency of square wave in Hz
const int amplitude = 5000;   // amplitude of square wave
const int sampleRate = 8000;  // sample rate in Hz

i2s_data_bit_width_t bps = I2S_DATA_BIT_WIDTH_16BIT;
i2s_mode_t mode = I2S_MODE_STD;
i2s_slot_mode_t slot = I2S_SLOT_MODE_STEREO;

const unsigned int halfWavelength = sampleRate / frequency / 2;  // half wavelength of square wave

int32_t sample = amplitude;   // current sample value
unsigned int count = 0;

I2SClass i2s;

void setup() {
  Serial.begin(115200);
  Serial.println("I2S simple tone");

  esp32_wiring_setup();
  i2s.setPins(bclkI2S, lrcI2S, dinI2S);

  // start I2S at the sample rate with 16-bits per sample
  if (!i2s.begin(mode, sampleRate, bps, slot)) {
    Serial.println("Failed to initialize I2S!");
    while (1);  // do nothing
  }
  Serial.print("sample: ");
  Serial.println(sample);
  Serial.print("sample>>8: ");
  Serial.println(sample>>8);
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
  // Get them 180 degrees out of phase to demonstrate stereo on an oscilloscope
  i2s.write(-sample);
  i2s.write((-sample) >> 8);

  // increment the counter for the next sample
  count++;
}
