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
TMC2208Stepper driver_a = TMC2208Stepper(MOTOR_A_RX_PIN, MOTOR_A_TX_PIN);
TMC2208Stepper driver_b = TMC2208Stepper(MOTOR_B_RX_PIN, MOTOR_B_TX_PIN);

AccelStepper stepper_a(AccelStepper::DRIVER, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);
AccelStepper stepper_b(AccelStepper::DRIVER, MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN);

void setup() {
    // Setup Pins:
        /*
        // Led Pins:
        pinMode(TRX_LED_PIN, OUTPUT);
        pinMode(STATUS_LED_TOP_PIN, OUTPUT);
        pinMode(STATUS_LED_MID_PIN, OUTPUT);
        pinMode(STATUS_LED_BOT_PIN, OUTPUT);

        // Button Pins:
        pinMode(HOMING_BUTTON_PIN, INPUT);
        pinMode(WORKLIGHT_BUTTON_PIN, INPUT);
        */
        // Axis Endswitch Pins:
        pinMode(X_AXIS_END_SWITCH_0_PIN, INPUT);
        pinMode(X_AXIS_END_SWITCH_1_PIN, INPUT);
        pinMode(Y_AXIS_END_SWITCH_0_PIN, INPUT);
        pinMode(Y_AXIS_END_SWITCH_1_PIN, INPUT);
        
        // Toolhead Pins:
        pinMode(SERVO_PIN, OUTPUT);
        pinMode(STATUS_LED_TOP_PIN, OUTPUT);

        // Motor Setup:
        // init motr pins:
        pinMode(MOTOR_A_EN_PIN, OUTPUT);
        pinMode(MOTOR_B_EN_PIN, OUTPUT);
        pinMode(MOTOR_A_STEP_PIN, OUTPUT);
        pinMode(MOTOR_B_STEP_PIN, OUTPUT);
        pinMode(MOTOR_A_DIR_PIN, OUTPUT);
        pinMode(MOTOR_B_DIR_PIN, OUTPUT);
        // disable drivers in hardware:
        digitalWrite(MOTOR_A_EN_PIN, HIGH);
        digitalWrite(MOTOR_B_EN_PIN, HIGH);
        // initialize the Serial connection to the drivers:
        driver_a.beginSerial(STEPPER_BAUD_RATE);
        driver_b.beginSerial(STEPPER_BAUD_RATE);
        // reset driver registers:
        driver_a.push();
        driver_b.push();

        driver_a.pdn_disable(true);     // Use PDN/UART pin for communication
        driver_b.pdn_disable(true);     // Use PDN/UART pin for communication
        driver_a.I_scale_analog(false); // Use internal voltage reference
        driver_b.I_scale_analog(false); // Use internal voltage reference
        driver_a.rms_current(MAX_MOTOR_CURRENT);      // Set driver current in mA
        driver_b.rms_current(MAX_MOTOR_CURRENT);      // Set driver current in mA
        driver_a.microsteps(MICROSTEPPING);     // set microstepping driver a
        driver_b.microsteps(MICROSTEPPING);     // set microstepping driver b
        driver_a.toff(2);               // Enable driver in software
        driver_b.toff(2);               // Enable driver in software

        digitalWrite(MOTOR_A_EN_PIN, LOW);    // Enable driver in hardware
        digitalWrite(MOTOR_B_EN_PIN, LOW);    // Enable driver in hardware

  Serial.begin(BAUD_RATE);
  Serial.setTimeout(100);
  //status_led_top = true;
  //digitalWrite(STATUS_LED_TOP_PIN, HIGH);

  //Serial.println(SOFTWARE_VERSION);
  //String reply = Serial.readString();

  /*if(connection_error) {
  // the connecting process returned an error, the device has to be rebooted:
    while(true){
      status_led_top = !status_led_top;
      digitalWrite(STATUS_LED_TOP_PIN, status_led_top);
      delay(500);
    }
    
  }*/
//  homeing();

}

void loop() {
  digitalWrite(STATUS_LED_TOP_PIN, HIGH);
  while (!Serial.available()) {
  }
  digitalWrite(STATUS_LED_TOP_PIN, LOW);
  while (Serial.available() == 0) {}     //wait for data available
  String command = Serial.readStringUntil('Q');  //read until timeout
  command.trim();
  if(command.startsWith("G")){
    Serial.print("Command: ");
    Serial.println(command);
    String direction = command.substring(1, command.indexOf('S')); // string until the seperator, ignoring first char
    String distance = command.substring(command.indexOf('S')+1); // string until terminator, not include seperator
    Serial.print("Direction: ");
    Serial.println(direction);
    Serial.print("Distance: ");
    Serial.println(distance);
    move(direction.toFloat(), distance.toInt()); // go to
  }
  else if (command.startsWith("HOMING")) {
  //homeing();
  }
  else if (command.startsWith("DISENGAGE_TOOLHEAD")) {
    disengage_toolhead();
  }
  else if (command.startsWith("ENGAGE_TOOLHEAD")) {
    engage_toolhead();
  }
  Serial.println(0);
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
  Serial.println("Starting to move ...");
  // direction in radians
  int steps[2];
  int pos_a = 0;
  int pos_b = 0;
  long delta_x = (long)(sin(direction)*micrometers);
  long delta_y = (long)(cos(direction)*micrometers);
  steps[0] = (int)(delta_x + delta_y)*STEP_TO_MICROMETER_RATIO;
  steps[1] = (int)(delta_x - delta_y)*STEP_TO_MICROMETER_RATIO;
  stepper_a.move(steps[0]);
  stepper_b.move(steps[1]);
  Serial.println("-----------------------------------");
  Serial.println("######## Start moving #############");
  Serial.println("-----------------------------------");
  Serial.print("Delty_X: ");
  Serial.println(delta_x);
  Serial.print("Delty_y: ");
  Serial.println(delta_y);
  Serial.println("About to start the while loop ...");
  Serial.print("Steps[0]: ");
  Serial.println(steps[0]);
  Serial.print("Steps[1]: ");
  Serial.println(steps[1]);
  while (abs(pos_a) < abs(steps[0]) || abs(pos_b) < abs(steps[1])) {
    Serial.println("Started While loop ...");
    if(pos_a <= steps[0]) {
      digitalWrite(MOTOR_A_DIR_PIN, LOW);
      digitalWrite(MOTOR_A_STEP_PIN, HIGH);
      pos_a ++;
    }
    else if(pos_a > steps[0]) {
      digitalWrite(MOTOR_A_DIR_PIN, HIGH);
      digitalWrite(MOTOR_A_STEP_PIN, HIGH);
      pos_a --;
    }
    if(pos_b < steps[1]) {
      digitalWrite(MOTOR_B_DIR_PIN, LOW);
      digitalWrite(MOTOR_B_STEP_PIN, HIGH);
      pos_b ++;
    }
    else if(pos_b > steps[1]) {
      digitalWrite(MOTOR_B_DIR_PIN, HIGH);
      digitalWrite(MOTOR_B_STEP_PIN, HIGH);
      pos_b --;
    }
    delayMicroseconds(WORKING_SPEED_DELAY);
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.print("pos_a: ");
    Serial.println(pos_a);
    Serial.print("pos_b: ");
    Serial.println(pos_b);
  }
}

void homeing() {
  // drive to x-axis stop using move:
  Serial.println("Start Homing ...");
  Serial.println("Homing X-Axis ...");
  bool stop = digitalRead(X_AXIS_END_SWITCH_0_PIN);
  while(!stop) {
    move(3*HALF_PI, 1); // move one step
    stop = digitalRead(X_AXIS_END_SWITCH_0_PIN);
  }
  Serial.println("Hit the trigger, moving back");
  // driving until the switch is not triggered anymore:
  while (stop) {
  move(HALF_PI, 1);
  }
  Serial.println("Finished Homing X-Axis");
  // drive to y-axis stop using move
}

void test_motor(){
  while (true) {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    Serial.println("Direction HIGH");
    for (int i = 0; i < 500; i++) {
        digitalWrite(MOTOR_A_STEP_PIN, !digitalRead(MOTOR_A_STEP_PIN));
        digitalWrite(MOTOR_B_STEP_PIN, !digitalRead(MOTOR_B_STEP_PIN));
        delay(1);
    }
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
    Serial.println("Direction LOW");
    for (int i = 0; i < 500; i++) {
        digitalWrite(MOTOR_A_STEP_PIN, !digitalRead(MOTOR_A_STEP_PIN));
        digitalWrite(MOTOR_B_STEP_PIN, !digitalRead(MOTOR_B_STEP_PIN));
        delay(1);
    }
  }

}

void send(String data) {
  digitalWrite(TRX_LED_PIN, HIGH);
  Serial.println(data);
  digitalWrite(TRX_LED_PIN, LOW);
}

void engage_toolhead(){
  //not implemented yet
}

void disengage_toolhead(){
  //no implemented yet
}
