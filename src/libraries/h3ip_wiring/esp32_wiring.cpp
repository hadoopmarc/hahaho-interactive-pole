
#include <Arduino.h>
#include <esp32_wiring.h>

// Explicit pinmodes for all available GPIO pins
void esp32_wiring_setup() {

  // System GPIO pins
  pinMode(internalLED, OUTPUT);
  // pinMode(serial0RX, ...);  This is handled by Serial.begin(115200)
  // pinMode(serial0TX, ...);  This is handled by Serial.begin(115200)

  // Special purpose GPIO ports
  pinMode(serial2RX, INPUT);
  pinMode(serial2TX, OUTPUT);
  pinMode(DAC1, OUTPUT);
  pinMode(DAC2, OUTPUT);

  // General use GPIO ports
  pinMode(redButton, INPUT_PULLUP);
  pinMode(stroboscope, OUTPUT);
  pinMode(freePin, INPUT);

  // Sensor GPIO pins (input only)
  pinMode(frontPIR, INPUT);
  pinMode(backPIR, INPUT);
  pinMode(ADC1_0, INPUT);
  pinMode(ADC1_3, INPUT);

  // Output GPIO pins
  pinMode(buzzer, OUTPUT);
  pinMode(neoPixel, OUTPUT);

  // Shared bus default GPIO pins (can be remapped for other uses when necessary)
  pinMode(sckHSPI, OUTPUT);
  pinMode(misoHSPI, INPUT);
  pinMode(mosiHSPI, OUTPUT);
  pinMode(csHSPI, OUTPUT);
  pinMode(sclI2C, OUTPUT);
  pinMode(sdaI2C, INPUT);
  pinMode(bclkI2S, OUTPUT);
  pinMode(lrcI2S, OUTPUT);
  pinMode(dinI2S, OUTPUT);
}
