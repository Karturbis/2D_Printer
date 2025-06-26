#include "config.h"

void setup() {
        pinMode(X_AXIS_END_SWITCH_0_PIN, INPUT_PULLUP);
        pinMode(X_AXIS_END_SWITCH_1_PIN, INPUT_PULLUP);
        pinMode(Y_AXIS_END_SWITCH_0_PIN, INPUT_PULLUP);
        pinMode(Y_AXIS_END_SWITCH_1_PIN, INPUT_PULLUP);
        Serial.begin(BAUD_RATE);
}

void loop() {
  if(!digitalRead(X_AXIS_END_SWITCH_0_PIN)){
    Serial.println("X-Axis switch 0 triggered");
  }
  if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
    Serial.println("X-Axis switch 1 triggered");
  }
  if(!digitalRead(Y_AXIS_END_SWITCH_0_PIN)){
    Serial.println("Y-Axis switch 0 triggered");
  }
  if(!digitalRead(Y_AXIS_END_SWITCH_1_PIN)){
    Serial.println("Y-Axis switch 1 triggered");
  }

}
