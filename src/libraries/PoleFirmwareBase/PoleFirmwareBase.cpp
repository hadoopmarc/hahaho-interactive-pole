/*
This class is an overall abstraction layer to all functions of the HaHaHo Interactive pole.
The intention of the abstraction is to lower the barrier for beginning programmers to make
creative use of the interactive pole.

The user documentation for this class is included in the demo script located in:
  hahaho-interactive-pole/src/pole-firmware-demo
*/

/* code below is the original demo to be refactored into the base class*/

/*
The redButton module uses a hardware interrupt on the input GPIO to reliably detect
pushing of the red button. After detection of a red button hit, the following
state machine applies:
START: no red button interaction is being handled
HIT_ONCE: the first interaction is detected. After hitDelayDuring has passed
  the hitOnceOn switch is toggled and the state progresses to PAUSE.
HIT_TWICE: an interaction is detected while in the HIT_ONCE state. After
  hitDelayDuring has passed, the hitTwiceOn switch is toggled and the state
  progresses to PAUSE. If another interaction is detected before
  hitDelayDuring has passed, the hitThriceOn switch is toggled and the state
  progresses to PAUSE immediately.
PAUSE: the interaction is finished. If hitDelayAfter has passed, the state
  returns to START.
*/

#include <arduino.h>                       // Only relevant for VSCode/PlatformIO Intellisense
#include <PoleFirmwareBase.h>
#include <AsyncWebSerial.h>                // Install "AsyncWebSerial" by Sessa
#include <ESPAsyncWebServer.h>             // Install "ESP Aync WebServer" and "Async TCP" by ESP32Async
#include <ESPDash.h>
// See src/libraries/README.md for use in Arduino IDE
#include <esp32_wiring.h>
#include <secrets.h>
#include <pole_wifi.h>
#include <web_ota.h>
#include "neopixel_vertical.h"
#include "PoleFirmwareBase.h"

using namespace dash;  //For components in ESPash.h

// NeoPixel
const uint16_t matrix_width = 64; 
const uint16_t matrix_height = 8;
const uint16_t NUM_LEDS = matrix_width * matrix_height; 
uint8_t currentBrightness = 100; // Helderheid iets verhoogd voor zichtbaarheid
CRGB currentColor = CRGB::OrangeRed;
String displayText = "Pole firmware started!";

// Variables for example funtionality
int ledState = LOW;                        // State of the LED to be negated in each loop 
int webSerialCounter = 0;                  // Example output for webSerial

// Timing control
const long ledPeriod = 200;                // Interval at which to blink (milliseconds)
unsigned long ledMillis = 0;               // Millis at the last led state change
const long serialPeriod = 1000;            // Interval at which to update the webSerial page
unsigned long serialMillis = 0;            // Millis at the last webSerial update
const long dashPeriod = 3000;              // Interval at which to update the dashboard page
unsigned long dashMillis = 0;              // Millis at the last dashboard update

// Red button
bool hitOnceOn = false;                     // Toggle state for a single hit, to be read by other modules
bool hitTwiceOn = false;                    // Toggle state for a dual hit, to be read by other modules
bool hitThriceOn = false;                   // Toggle state for a triple hit, to be read by other modules
unsigned long int hitDelayBefore = 100000;  // Delay in micros before finish a detection (debouncing)
unsigned long int hitDelayDuring = 500000;  // Delay in micros before completing an interaction
unsigned long int hitDelayAfter = 1500000;  // Delay in micross before accepting a new interaction
volatile bool hitButton = false;            // Flag to be set by the interrupt and cleared in loop()
volatile unsigned long hitMicros;           // Set by the interrupt only
enum HitState {START, HIT_ONCE, HIT_TWICE, HIT_THRICE, PAUSE};
HitState hitState = START;

// Function prototypes implemented below
void printToggleSwitches();
void handlePush();


PoleFirmwareBase::PoleFirmwareBase()
{
}

void PoleFirmwareBase::setup()
{
    esp32_wiring_setup();
    // redButton is configured as INPUT_PULLUP and wired to ground, so FALLING is the right trigger
    attachInterrupt(digitalPinToInterrupt(redButton), handlePush, FALLING);

    // Library setup voor OTA updates and webSerial monitoring
    wifi_setup(btnCredentials, btnCredsSize, &apCredentials);
    web_ota_setup(*server, &otaCredentials);
    webSerial.begin(server);

    // FastLED setup
    setupNeoPixel();
    setNeoPixelText(displayText);

    // Start de webserver
    (*server).begin();
}

// All function calls should be non-blocking (do not use the delay() function)
void PoleFirmwareBase::sense_loop() {

  unsigned long currentMillis = millis();

  // blink internal LED
  if (currentMillis - ledMillis >= ledPeriod) {
    digitalWrite(internalLED, ledState);
    ledState = not(ledState);
    ledMillis = currentMillis;
  }

  // Update webSerial page
  if (currentMillis - serialMillis >= serialPeriod) {
    webSerial.print(webSerialCounter++);
    webSerial.loop();
    serialMillis = currentMillis;
  }

  // Update dashboard cards
  if (currentMillis - dashMillis >= dashPeriod) {
    temperature.setValue((int)random(0, 50));
    humidity.setValue((int)random(0, 100));
    dashboard.sendUpdates();
    dashMillis = currentMillis;
  }

  unsigned long int diffMicros = micros() - hitMicros;
  if (hitState == START) {
    if (hitButton && diffMicros >= hitDelayBefore) {
      hitButton = false;
      hitState = HIT_ONCE;
      Serial.println("--> HIT_ONCE");
    }
  } else if (hitState == HIT_ONCE) {
    if (hitButton && diffMicros >= hitDelayBefore) {
      hitButton = false;
      hitState = HIT_TWICE;
      Serial.println("--> HIT_TWICE");
    } else if (diffMicros >= hitDelayDuring) {
      hitOnceOn = !hitOnceOn;
      hitState = PAUSE;
      printToggleSwitches();
      displayText = "HIT ONCE!";
      webSerial.println(displayText);
    }
  } else if (hitState == HIT_TWICE) {
    if (hitButton && diffMicros >= hitDelayBefore) {
      hitButton = false;
      hitState = HIT_THRICE;
      Serial.println("--> HIT_THRICE");
    } else if (diffMicros >= hitDelayDuring) {
      hitButton = false;
      hitTwiceOn = !hitTwiceOn;
      hitState = PAUSE;
      printToggleSwitches();
      displayText = "HIT TWICE!";
      webSerial.println(displayText);
    }
  } else if (hitState == HIT_THRICE) {
    if (diffMicros >= hitDelayDuring) {
      hitButton = false;
      hitThriceOn = !hitThriceOn;
      hitState = PAUSE;
      printToggleSwitches();
      displayText = "HIT THRICE!";
      webSerial.println(displayText);
    }
  } else {  // hitState == PAUSE
    if (hitButton) {
      Serial.println("\nTake it easy...");
      hitButton = false;
    } else if (diffMicros >= hitDelayAfter) {
      hitState = START;
    }
  }
}

// All function calls should be non-blocking (do not use the delay() function)
void PoleFirmwareBase::act_loop() {
  drawScrollingText(); 
}

void printToggleSwitches() {
  Serial.print("hitOnce:    ");
  Serial.println((hitOnceOn) ? "on" : "off");
  Serial.print("hitTwice:   ");
  Serial.println((hitTwiceOn) ? "on" : "off");
  Serial.print("hitThrice:  ");
  Serial.println((hitThriceOn) ? "on" : "off");
}

void handlePush() {
  cli();                            // disable interrupts for making the value changes below in an atomic way
  if (!hitButton) {
    hitMicros = micros();           // the micros() value can be reliably read on entry of an ISR
    hitButton = true;
  }
  sei();
}
