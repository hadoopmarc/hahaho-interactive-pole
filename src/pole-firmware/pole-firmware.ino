/*
This version of the HaHaHo Interactive Pole combines the following standalone scripts:
https://github.com/hadoopmarc/hahaho-interactive-pole/tree/main/standalone/actuate/buzzer
https://github.com/hadoopmarc/hahaho-interactive-pole/tree/main/standalone/sense/red_button

**************************************************************************************************
Wiring of 30-pin ESP32dev module                                                                 *
https://www.electronicshub.org/esp32-pinout/                                                     *
------------------------------------------------------------------------------------------------ *
Pin Name     Signal  Connected to                Remarks                                         *
--  ------   ------  --------------------------  ----------------------------------------------- *
28  GPIO1    TXD0    Reserved serial output                                                      *
19  GPIO2            Built-in LED                                                                *
27  GPIO3    RXD0    Reserved serial input                                                       *
20  GPIO4            Buzzer via 100 Ohm                                                          *
23  GPIO5    CS0     Big button                  (CS0 active low)                                *
12  GPIO12           Neopixel panel DIN                                                          *
13  GPIO13                                                                                       *
11  GPIO14           I2S BCLK naar bijv. MAX98357A / PCM5102                                     *
18  GPIO15           -                                                                           *
21  GPIO16   RXD2    -                                                                           *
22  GPIO17   TXD2    -                                                                           *
24  GPIO18   SCK     -                                                                           *
25  GPIO19   MISO    -                                                                           *
26  GPIO21           SDA for I2C (DS3132)                                                        *
29  GPIO22           SCL for I2C (DS3132)                                                        *
30  GPIO23   MOSI    -                                                                           *
 8  GPIO25   -       audio output van interne DAC                                                *
 9  GPIO26   -       audio output van interne DAC                                                *
10  GPIO27   -       I2S DIN  naar bijv. MAX98357A / PCM5102                                     *
 6  GPIO32   -       Enable VIN of PCM5102                                                       *
 7  GPIO33   -       I2S LRC  naar bijv. MAX98357A / PCM5102                                     *
 4  GPIO34   -       GPS TX                      (input only pin)                                *
 5  GPIO35   -       -                           (input only pin)                                *
 2  GPIO36   VP      -                           (input only pin)                                *
 3  GPIO39   VN      -                           (input only pin)                                *
-------  ------  ----------------                                                                *
14  GND      -       Power supply GND                                                            *
17  GND      -       Power supply GND                                                            *
15  Vin      -       Power supply 5V                                                             *
16  3.3 V    -       GPS Vcc                                                                     *
 1  EN       -       Enable (connected to onboard reset)                                         *
*************************************************************************************************/

#define internalLED  2     // GPIO2, do not connect
#define buzzer       4     // GPIO4, D4, pin 20
#define redButton    5     // GPIO5, D5, pin 23
 
void setup() {
  Serial.begin(115200);
  sleep(1);
  Serial.println("\n\nHello, HaHaHo Interactive Pole");
  pinMode(internalLED, OUTPUT);
  pinMode(buzzer, OUTPUT);           // Wire button to GND via 100 Ohm
  pinMode(redButton, INPUT_PULLUP);  // Wire button directly to GND
}
 
void loop() {
  bool buttonActive = !digitalRead(redButton);
  Serial.println(buttonActive);
  if (buttonActive) {
    digitalWrite(internalLED, HIGH);
    tone(buzzer, 1000); // 1KHz sound signal
    delay(1000);
  } else {
    digitalWrite(internalLED, LOW);
    noTone(buzzer);
    delay(1000);
  }
}
