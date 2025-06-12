#include "config.h"

void setup() {
    // Setup Pins:

        // Led Pins:
        pinMode(RX_LED_PIN, OUTPUT);
        pinMode(TX_LED_PIN, OUTPUT);
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
        pinMode(MOTOR_A_CS_PIN, OUTPUT);
        pinMode(MOTOR_B_EN_PIN, OUTPUT);
        pinMode(MOTOR_B_DIR_PIN, OUTPUT);
        pinMode(MOTOR_B_STEP_PIN, OUTPUT);
        pinMode(MOTOR_B_CS_PIN, OUTPUT);

        // Axis Endswitch Pins:
        pinMode(X_AXIS_END_SWITCH_PIN, INPUT);
        pinMode(Y_AXIS_END_SWITCH_PIN, OUTPUT);

        // Toolhead Pins:
        pinMode(SERVO_PIN, OUTPUT);
        pinMode(WORKLIGHT_PIN, OUTPUT);

  Serial.begin(BAUD_RATE);

}

void loop() {

}
