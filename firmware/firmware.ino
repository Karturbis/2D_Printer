#include <AccelStepper.h>

#include "config.h"


bool STATUS_LED_TOP = false;
bool STATUS_LED_MID = false;
bool STATUS_LED_BOT = false;
int[] position;


void setup() {
    // Setup Pins:

        // Led Pins:
        pinMode(TRX_LED_PIN, OUTPUT);
        pinMode(STATUS_LED_TOP_PIN, OUTPUT);
        pinMode(STATUS_LED_MID_PIN, OUTPUT);
        pinMode(STATUS_LED_BOT_PIN, OUTPUT);

        // Button Pins:
        pinMode(HOMING_BUTTON_PIN, INPUT);
        pinMode(WORKLIGHT_BUTTON_PIN, INPUT);

        // Motor Pins:
        pinMode(MOTOR_A_EN_PIN, OUTPUT);
        pinMode(MOTOR_A_DIR_PIN, OUTPUT);
        pinMode(MOTOR_A_STEP_PIN, OUTPUT);
        pinMode(MOTOR_B_EN_PIN, OUTPUT);
        pinMode(MOTOR_B_DIR_PIN, OUTPUT);
        pinMode(MOTOR_B_STEP_PIN, OUTPUT);

        // Axis Endswitch Pins:
        pinMode(X_AXIS_END_SWITCH_0_PIN, INPUT);
        pinMode(X_AXIS_END_SWITCH_1_PIN, INPUT);
        pinMode(Y_AXIS_END_SWITCH_0_PIN, INPUT);
        pinMode(Y_AXIS_END_SWITCH_1_PIN, INPUT);

        // Toolhead Pins:
        pinMode(SERVO_PIN, OUTPUT);
        pinMode(WORKLIGHT_PIN, OUTPUT);
  Serial.begin(BAUD_RATE);

}

void loop() {

}

void move(int[2] target_position) {
  int steps = (int)(FULLSTEP_TO_MICROMETER_RATIO*micrometer);
}

int homeing() {
  // drive to x-axis stop
  // drive to y-axis stop
  //if error return 1, else 0
}