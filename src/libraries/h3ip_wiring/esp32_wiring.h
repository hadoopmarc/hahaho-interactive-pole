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
***********************************************************************
For details on GPIO assignment, see:
    https://www.studiopieters.nl/esp32-pinout/
For analog input use ADC1_x ports, because ADC2_y shares resources
with the WiFi hardware.
*/

#ifndef ESP32_WIRING_H
#define ESP32_WIRING_H

// System GPIO pins
#define internalLED  02   // pin19   Reserved built-in LED
#define serial0RX    03   // pin27   Reserved input RXD0 for USB serial
#define serial0TX    01   // pin28   Reserved output TXD0 for USB serial

// Special purpose GPIO ports
#define serial2RX    16   // pin21   In use as serial for mp3 DfPlayer
#define serial2TX    17   // pin22   In use as serial for mp3 DfPlayer
#define DAC1         25   // pin08   Free as DAC1
#define DAC2         26   // pin09   Free as DAC2

// General use GPIO ports
#define redButton    05   // pin23   In use for big button (wire it directly to GND; CS inactive high by default)
#define stroboscope  19   // pin25   In use for stroboscope light
#define free         23   // pin30   Free for general use

// Sensor GPIO pins (input only)
#define frontPIR     34   // pin04   In use for PIR sensor
#define backPIR      35   // pin05   In use for PIR sensor
#define ADC1_0       36   // pin02   Free as ADC1_0
#define ADC1_3       39   // pin03   Free as ADC1_3

// Output GPIO pins
#define buzzer       04   // pin20   In use for buzzer (wire it to GND via 100 Ohm)
#define neoPixel     18   // pin24   In use for data line of neopixel panel

// Shared bus default GPIO pins (can be remapped for other uses when necessary)
#define sckHSPI      14   // pin11   Free for SPI peripherals
#define misoHSPI     12   // pin12   Free for SPI peripherals
#define mosiHSPI     13   // pin13   Free for SPI peripherals
#define csHSPI       15   // pin18   Free for SPI peripherals
#define sclI2C       22   // pin29   Free for I2C peripherals
#define sdaI2C       21   // pin26   Free for I2c peripherals
#define bclkI2S      32   // pin06   In use for Inter-IC-Sound bitclock to PCM5102
#define lrcI2S       33   // pin07   In use for Inter-IC-Sound left/right clock to PCM5102
#define dinI2S       27   // pin10   In use for Inter-IC-Sound data to PCM5102

void esp32_wiring_setup();     // Including esp32_wiring.h implies doing the setuo

#endif // ESP32_WIRING_H
