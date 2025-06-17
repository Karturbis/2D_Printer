/**
 * This is the firmware for the 2D Printer.
 */

 // include libraries:
#include <AccelStepper.h>
#include <TMC2208Stepper.h>

#include "config.h" // include configuration

// initialzize varables
bool status_led_top = false;
bool status_led_mid = false;
bool status_led_bot = false;
int position[2];

// initialize TMC2208 class, use Hardware Serial Port for communication
TMC2208Stepper driver = TMC2208Stepper(&Serial1);

AccelStepper stepper_a(AccelStepper::DRIVER, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN)

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

        // Axis Endswitch Pins:
        pinMode(X_AXIS_END_SWITCH_0_PIN, INPUT);
        pinMode(X_AXIS_END_SWITCH_1_PIN, INPUT);
        pinMode(Y_AXIS_END_SWITCH_0_PIN, INPUT);
        pinMode(Y_AXIS_END_SWITCH_1_PIN, INPUT);

        // Toolhead Pins:
        pinMode(SERVO_PIN, OUTPUT);
        pinMode(WORKLIGHT_PIN, OUTPUT);
  Serial1.begin(STEPPER_BAUD_RATE);
  Serial.begin(BAUD_RATE);
  status_led_top = true;
  digitalWrite(STATUS_LED_TOP_PIN, HIGH);
  int connection_error = connect();
  if(connection_error) {
  // the connecting process returned an error, the device has to be rebooted:
    while(true){
      status_led_top = !status_led_top;
      digitalWrite(STATUS_LED_TOP_PIN, status_led_top);
      delay(500);
    }
  }
  homeing();
}

void loop() {

}

void moveto(int target_position[2]) {
  int target_pos_motor[2];
  int target_pos_x = target_position[0]*STEP_TO_MICROMETER_RATIO;
  int target_pos_y = target_position[1]*STEP_TO_MICROMETER_RATIO;
  target_pos_motor[0] = target_pos_x + target_pos_y;
  target_pos_motor[1] = target_pos_x - target_pos_y;
  // use moveto from stepper library
}

void move(float direction, int micrometers) {
  // direction in radians
  int steps[2];
  int delta_x = (int)sin(direction)*micrometers;
  int delta_y = (int)cos(direction)*micrometers;
  steps[0] = (delta_x + delta_y)*STEP_TO_MICROMETER_RATIO;
  steps[1] = (delta_x - delta_y)*STEP_TO_MICROMETER_RATIO;
  // move motor a steps[0] and motor b steps[1]

}

void homeing() {
  // drive to x-axis stop using move:
  bool stop = digitalRead(X_AXIS_END_SWITCH_0_PIN);
  while(!stop) {
    move(HALF_PI, INVERSE_STEP_TO_MICROMETER_RATIO); // move one step
    stop = digitalRead(X_AXIS_END_SWITCH_0_PIN)
  }
  // drive to y-axis stop using move
}

void init_driver() {
  // Disable the Drivers:
  digitalWrite(MOTOR_A_EN_PIN, HIGH);
  digitalWrite(MOTOR_B_EN_PIN, HIGH);
  driver.pdn_disable(true);
  driver.I_scale_analog(false);

}

int connect() {
  send(SOFTWARE_VERSION);
  String reply = Serial.readString();
  if(reply == "connect"){
    return 0;
  }
  else {
    return 1;
  }
}

void send(String data) {
  digitalWrite(TRX_LED_PIN, HIGH);
  Serial.println(data);
  digitalWrite(TRX_LED_PIN, LOW);
}
