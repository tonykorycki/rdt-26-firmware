#include <Arduino.h>
#include "stepper_driver.h"
#include "config.h"

static bool          STEPPER_enable = false;
static unsigned long lastStepTime   = 0;
static bool          isStepPinHigh  = false;

void STEPPER_Init() {
    pinMode(STEPPER_DIR_PIN,    OUTPUT);
    pinMode(STEPPER_STEP_PIN,   OUTPUT);
    pinMode(STEPPER_ENABLE_PIN, OUTPUT);
    pinMode(STEPPER_M0_PIN,     OUTPUT);
    pinMode(STEPPER_M1_PIN,     OUTPUT);
    pinMode(STEPPER_M2_PIN,     OUTPUT);
    setMicrostep(MICROSTEPPING_FACTOR);
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);  // DRV8825 active-low enable; start disabled
}

void setMicrostep(int mode) {
    switch (mode) {
        case 1:  digitalWrite(STEPPER_M0_PIN, LOW);  digitalWrite(STEPPER_M1_PIN, LOW);  digitalWrite(STEPPER_M2_PIN, LOW);  break;
        case 2:  digitalWrite(STEPPER_M0_PIN, HIGH); digitalWrite(STEPPER_M1_PIN, LOW);  digitalWrite(STEPPER_M2_PIN, LOW);  break;
        case 4:  digitalWrite(STEPPER_M0_PIN, LOW);  digitalWrite(STEPPER_M1_PIN, HIGH); digitalWrite(STEPPER_M2_PIN, LOW);  break;
        case 8:  digitalWrite(STEPPER_M0_PIN, HIGH); digitalWrite(STEPPER_M1_PIN, HIGH); digitalWrite(STEPPER_M2_PIN, LOW);  break;
        case 16: digitalWrite(STEPPER_M0_PIN, LOW);  digitalWrite(STEPPER_M1_PIN, LOW);  digitalWrite(STEPPER_M2_PIN, HIGH); break;
        case 32: digitalWrite(STEPPER_M0_PIN, HIGH); digitalWrite(STEPPER_M1_PIN, LOW);  digitalWrite(STEPPER_M2_PIN, HIGH); break;
    }
}

// direction: +1 = up, -1 = down, 0 = stop
void STEPPER_SetDirection(int direction) {
    STEPPER_enable = (direction != 0);
    digitalWrite(STEPPER_ENABLE_PIN, STEPPER_enable ? LOW : HIGH);  // active-low
    if      (direction == 1)  digitalWrite(STEPPER_DIR_PIN, HIGH);
    else if (direction == -1) digitalWrite(STEPPER_DIR_PIN, LOW);
}

/*
  Toggles STEP at half the requested period so one full rising edge is produced
  every periodMicroseconds. Uses micros() for non-blocking timing; the rest
  of the loop continues between steps.
 */
void STEPPER_Update(unsigned long periodMicroseconds) {
    if (!STEPPER_enable) return;
    unsigned long now = micros();
    if (now - lastStepTime >= periodMicroseconds / 2) {
        isStepPinHigh = !isStepPinHigh;
        digitalWrite(STEPPER_STEP_PIN, isStepPinHigh ? HIGH : LOW);
        lastStepTime = now;
    }
}
