
#include <Arduino.h>
#include <esp32_wiring.h>

// Explicit pimodes for all available GPIO pins
void esp32_wiring_setup() {
  // Special GPIO pins
  pinMode(internalLED, OUTPUT);
  pinMode(serial0RX, INPUT);
  pinMode(serial0TX, OUTPUT);
  pinMode(free0, INPUT);
  pinMode(free1, INPUT);
  pinMode(free2, INPUT);
  // Sensor GPIO pins
  pinMode(redButton, INPUT_PULLUP);
  pinMode(frontPIR, INPUT);
  pinMode(backPIR, INPUT);
  pinMode(gpsSerial, INPUT);
  // Output GPIO pins
  pinMode(buzzer, OUTPUT);
  pinMode(audioEnable, OUTPUT);
  pinMode(audioLeft, OUTPUT);
  pinMode(audioRight, OUTPUT);
  pinMode(neoPixel, OUTPUT);
  // Interface GPIO pins
  pinMode(serial2RX, INPUT);
  pinMode(serial2TX, OUTPUT);
  pinMode(sckSPI, OUTPUT);
  pinMode(misoSPI, INPUT);
  pinMode(mosiSPI, OUTPUT);
  pinMode(sclI2C, OUTPUT);
  pinMode(sdaI2C, INPUT);
  pinMode(bclkI2S, OUTPUT);
  pinMode(lrcI2S, OUTPUT);
  pinMode(dinI2S, OUTPUT);
}
