/**********************************************************************
Wiring of the 30-pin ESP32dev module for the HaHaHo Interactive Pole  *
https://www.electronicshub.org/esp32-pinout/                          *
----------------------------------------------------------------------*
Pin Name     Connected to                                             *
--  ------   --------------------------                               *
14  GND      Power supply GND                                         *
17  GND      Power supply GND                                         *
15  Vin      Power supply 5V                                          *
16  3.3 V    GPS Vcc                                                  *
 1  EN       Enable (connected to onboard reset)                      *
**********************************************************************/
#ifndef ESP32_WIRING_H
#define ESP32_WIRING_H

// Special GPIO pins
#define internalLED  2   // pin19   Built-in LED
#define serial0RX    3   // pin27   Reserved serial input RXD0
#define serial0TX    1   // pin28   Reserved serial output TXD0
#define free0       13   // pin13   Unused
#define free1       15   // pin18   Unused
#define free2       39   // pin03   Unused          (can function as ADC VN pin, input only pin)

// Sensor GPIO pins
#define redButton    5   // pin23   Wire big button directly to GND (CS0 active low)
#define frontPIR    35   // pin05   PIR sensor                                  (input only pin)
#define backPIR     36   // pin02   PIR sensor      (can function as ADC VP pin, input only pin)
#define gpsSerial   34   // pin04   Reserved for outdoor applications with GPS  (input only pin)

// Output GPIO pins
#define buzzer       4   // pin20   Wire buzzer to GND via 100 Ohm
#define audioEnable 32   // pin06   Enable VIN of PCM5102
#define audioLeft   25   // pin08   audio output van interne DAC                
#define audioRight  26   // pin09   audio output van interne DAC                
#define neoPixel    12   // pin12   Neopixel panel DIN

// Interface GPIO pins
#define serial2RX   16   // pin21   Unused serial input RXD2             
#define serial2TX   17   // pin22   Unused serial output TXD2
#define sckSPI      18   // pin24   Keep available for SPI sensors
#define misoSPI     19   // pin25   Keep available for SPI sensors
#define mosiSPI     23   // pin30   Keep available for SPI sensors    
#define sclI2C      22   // pin29   SCL for I2C (DS3132)
#define sdaI2C      21   // pin26   SDA for I2C (DS3132)    
#define bclkI2S     14   // pin11   Inter-IC-Sound bitclock to MAX98357A / PCM5102
#define lrcI2S      33   // pin07   Inter-IC-Sound left/right clock to MAX98357A / PCM5102
#define dinI2S      27   // pin10   Inter-IC-Sound data input to MAX98357A / PCM5102

void esp32_wiring_setup();

#endif // ESP32_WIRING_H
