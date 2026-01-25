/*
This module uses a hardware interrupt on the input GPIO to reliably detect
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

F
*/

#include "esp32_wiring.h"

bool hitOnceOn = false;                     // Toggle state for a single hit, to be read by other modules
bool hitTwiceOn = false;                    // Toggle state for a dual hit, to be read by other modules
bool hitThriceOn = false;                   // Toggle state for a triple hit, to be read by other modules
unsigned long int hitDelayBefore = 100000;  // Delay in micros before finish a detection (debouncing)
unsigned long int hitDelayDuring = 500000;  // Delay in micros before completing an interaction
unsigned long int hitDelayAfter = 1500000;  // Delay in micross before accepting a new interaction

volatile bool hitButton = false;            // Flag to be set by the interrupt and cleared in loop()
volatile unsigned long hitMicros;           // Set by the interrupt only

enum HitState { 
    START, HIT_ONCE, HIT_TWICE, PAUSE
} hitState = START;


void handlePush() {
  cli();                            // disable interrupts for making the value changes below in an atomic way
  if (!hitButton) {
    hitMicros = micros();           // the micros() value can be reliably read on entry of an ISR
    hitButton = true;
  }
  sei();
}


void setup() {
  Serial.begin(115200);
  Serial.println("Hit the red button once, twice or thrice and the states are toggled!");
  esp32_wiring_setup();
  // redButton is configured as INPUT_PULLUP and wired to ground, so FALLING is the right trigger
  attachInterrupt(digitalPinToInterrupt(redButton), handlePush, FALLING);
}


void loop() {
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
    }
  } else if (hitState == HIT_TWICE) {
    if (hitButton && diffMicros >= hitDelayBefore) {
      hitThriceOn = !hitThriceOn;
      hitButton = false;
      hitState = PAUSE;
      Serial.println("--> HIT_THRICE");
      printToggleSwitches();
    } else if (diffMicros >= hitDelayDuring) {
      hitButton = false;
      hitTwiceOn = !hitTwiceOn;
      hitState = PAUSE;
      printToggleSwitches();
    }
  } else {  // hitState == PAUSE
    if (hitButton) {
      Serial.println("\nTake it easy...");
      hitButton = false;
    } else if (diffMicros >= hitDelayAfter) {
      hitState = START;
    }
  }
  delay(50);
}


void printToggleSwitches() {
  Serial.print("\nhitOnce:    ");
  Serial.println((hitOnceOn) ? "on" : "off");
  Serial.print("hitTwice:   ");
  Serial.println((hitTwiceOn) ? "on" : "off");
  Serial.print("hitThrice:  ");
  Serial.println((hitThriceOn) ? "on" : "off");
}
