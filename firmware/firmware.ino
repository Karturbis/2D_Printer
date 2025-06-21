/**
 * This is the firmware for the 2D Printer.
 */

 // include libraries:
#include <AccelStepper.h>
#include <TMC2208Stepper.h>
#include <Servo.h>

#include "config.h" // include configuration

// Function prototypes:
int move_steps(int steps[2], int working_speed_delay=WORKING_SPEED_DELAY, bool ignore_endswitches=false);
int move_steps_diagonal_support(int steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false);


// initialzize varables
bool status_led_top = false;
bool status_led_mid = false;
bool status_led_bot = false;
bool manual_mode = false;
bool expert_mode = false;
int position[2];

// initialize TMC2208 class, use Hardware Serial Port for communication
TMC2208Stepper driver_a = TMC2208Stepper(MOTOR_A_RX_PIN, MOTOR_A_TX_PIN);
TMC2208Stepper driver_b = TMC2208Stepper(MOTOR_B_RX_PIN, MOTOR_B_TX_PIN);

// initialize Accel steppers:
AccelStepper stepper_a(AccelStepper::DRIVER, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);
AccelStepper stepper_b(AccelStepper::DRIVER, MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN);

// initialize toolhead servo:
Servo toolhead_servo;

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
        pinMode(X_AXIS_END_SWITCH_0_PIN, INPUT_PULLUP);
        pinMode(X_AXIS_END_SWITCH_1_PIN, INPUT_PULLUP);
        pinMode(Y_AXIS_END_SWITCH_0_PIN, INPUT_PULLUP);
        pinMode(Y_AXIS_END_SWITCH_1_PIN, INPUT_PULLUP);
        
        // Toolhead Pins:
        pinMode(SERVO_PIN, OUTPUT);
        pinMode(STATUS_LED_TOP_PIN, OUTPUT);

        // Servo Setup:
        toolhead_servo.attach(SERVO_PIN);
        toolhead_servo.write((SERVO_UP_POSITION));

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

        // UART Setup:
        driver_a.pdn_disable(true);     // Use PDN/UART pin for communication
        driver_b.pdn_disable(true);     // Use PDN/UART pin for communication
        driver_a.I_scale_analog(false); // Use internal voltage reference
        driver_b.I_scale_analog(false); // Use internal voltage reference
        driver_a.rms_current(MAX_MOTOR_CURRENT);      // Set driver current in mA
        driver_b.rms_current(MAX_MOTOR_CURRENT);      // Set driver current in mA
        driver_a.pwm_autoscale(1);
        driver_b.pwm_autoscale(1);
        driver_a.microsteps(MICROSTEPPING);     // set microstepping driver a
        driver_b.microsteps(MICROSTEPPING);     // set microstepping driver b
        driver_a.toff(2);               // Enable driver in software
        driver_b.toff(2);               // Enable driver in software

        digitalWrite(MOTOR_A_EN_PIN, LOW);    // Enable driver in hardware
        digitalWrite(MOTOR_B_EN_PIN, LOW);    // Enable driver in hardware

        // Accelstepper Setup:
        stepper_a.setMaxSpeed(MAX_SPEED);

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
}

void loop() {
  digitalWrite(STATUS_LED_TOP_PIN, HIGH);
  while (!Serial.available()) // wait for data available
  digitalWrite(STATUS_LED_TOP_PIN, LOW);
  String command = Serial.readStringUntil(TERMINATOR);  //read until terminator character
  command.trim();
  if(command.startsWith(GO_TO)){
    Serial.print("Command: ");
    Serial.println(command);
    String direction = command.substring(1, command.indexOf(SEPERATOR)); // string until the seperator, ignoring first char
    String distance = command.substring(command.indexOf(SEPERATOR)+1); // string until terminator, not include seperator
    Serial.print("Direction: ");
    Serial.println(direction);
    Serial.print("Distance: ");
    Serial.println(distance);
    int error = move(direction.toFloat(), distance.toInt()); // go to
    if(error == 1){
      Serial.println("Toolhead ran into wall, x-axis 0 triggered!");
      Serial.println("Home the Printer to move again!");
    }
    else if(error == 2){
      Serial.println("Toolhead ran into wall, x-axis 1 triggered!");
      Serial.println("Home the Printer to move again!");
    }
    else if(error == 3){
      Serial.println("Toolhead ran into wall, y-axis 0 triggered!");
      Serial.println("Home the Printer to move again!");
    }
    else if(error == 4){
      Serial.println("Toolhead ran into wall, y-axis 1 triggered!");
      Serial.println("Home the Printer to move again!");
    }
    Serial.println(error);
  }
  else if (command.startsWith(HOMING)) {
  homeing();
  homeing();
  homeing();
  Serial.println(0);
  }
  else if (command.startsWith(DISENGAGE_TOOLHEAD)) {
    disengage_toolhead();
    Serial.println(0);
  }
  else if (command.startsWith(ENGAGE_TOOLHEAD)) {
    engage_toolhead();
    Serial.println(0);
  }
}

int move(float direction, int micrometers) {
  Serial.println("Starting to move ...");
  // direction in radians
  int steps[2];
  long delta_x = (long)(sin(direction)*micrometers);
  long delta_y = (long)(cos(direction)*micrometers);
  if(TOGGLE_X_Y_AXIS){
    int new_delta_x = delta_y;
    delta_y = delta_x;
    delta_x = delta_y;
  }
  if(TOGGLE_X_AXIS){
    delta_x = -delta_x;
  }
  if(TOGGLE_Y_AXIS) {
    delta_y = -delta_y;
  }
  steps[0] = (int)(delta_x + delta_y)*STEP_TO_MICROMETER_RATIO;
  steps[1] = (int)(delta_x - delta_y)*STEP_TO_MICROMETER_RATIO;
  stepper_a.move(steps[0]);
  stepper_b.move(steps[1]);
  Serial.println("-----------------------------------");
  Serial.println("######## Start moving #############");
  Serial.println("-----------------------------------");
  Serial.print("Delta_X: ");
  Serial.println(delta_x);
  Serial.print("Delta_y: ");
  Serial.println(delta_y);
  Serial.print("Steps[0]: ");
  Serial.println(steps[0]);
  Serial.print("Steps[1]: ");
  Serial.println(steps[1]);
  // move:
  int error;
  error = STEPS_ALGORITHM(steps);
  // Update Position:
  position[0] = position[0] + delta_x;
  position[1] = position[1] + delta_y;
  Serial.print("New Position: ");
  Serial.print(position[0]);
  Serial.print(" ");
  Serial.println(position[1]);
  return error;
}

int move_steps(int steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false){
  int pos_a = 0;
  int pos_b = 0;
  while (abs(pos_a) < abs(steps[0]) || abs(pos_b) < abs(steps[1])) {
    if(pos_a < steps[0]) {
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
    if(!ignore_endswitches){
      if(!digitalRead(X_AXIS_END_SWITCH_0_PIN)){
        return 1;
      }
      if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
        return 2;
      }
      if(!digitalRead(Y_AXIS_END_SWITCH_0_PIN)){
        return 3;
      }
      if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
        return 4;
      }
    }
    delayMicroseconds(WORKING_SPEED_DELAY);
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
  }
  return 0;

}

void homeing() {
  int steps[2];
  disengage_toolhead();
  // drive to x-axis stop using move:
  Serial.println("Start Homing ...");
  Serial.println("Homing X-Axis ...");
  bool stop = !digitalRead(X_AXIS_END_SWITCH_0_PIN);
  steps[0] = 1;
  steps[1] = 1;
  while(!stop) {
    move_steps(steps, HOMING_SPEED_DELAY, true);
    stop = !digitalRead(X_AXIS_END_SWITCH_0_PIN);
  }
  Serial.println("Hit the trigger, moving back");
  // driving until the switch is not triggered anymore:
  steps[0] = -1;
  steps[1] = -1;
  while (stop) {
    move_steps(steps, HOMING_SPEED_DELAY, true);
    stop = !digitalRead(X_AXIS_END_SWITCH_0_PIN);
  }
  position[0] = 0;
  Serial.println("Finished Homing X-Axis");
  Serial.println("Backing up on X-Axis ...");
  move(HALF_PI, 10000);
  // drive to y-axis stop using move
  Serial.println("Homing Y-Axis ...");
  stop = !digitalRead(Y_AXIS_END_SWITCH_0_PIN);
  steps[0] = 1;
  steps[1] = -1;
  while(!stop) {
    move_steps(steps, HOMING_SPEED_DELAY, true);
    stop = !digitalRead(Y_AXIS_END_SWITCH_0_PIN);
  }
  Serial.println("Hit the trigger, moving back");
  // driving until the switch is not triggered anymore:
  steps[0] = -1;
  steps[1] = 1;
  while (stop) {
    move_steps(steps, HOMING_SPEED_DELAY, true);
    stop = !digitalRead(Y_AXIS_END_SWITCH_0_PIN);
  }
  position[1] = 0;
  Serial.println("Finished Homing Y-Axis");
  Serial.println("Backing up on Y-Axis ...");
  move(PI, 10000);
  Serial.println("Finished Homing");
  //move(1.5, 1000);
}

// Toolhead:
void engage_toolhead(){
  toolhead_servo.write(SERVO_DOWN_POSITION);
}

void disengage_toolhead(){
  toolhead_servo.write(SERVO_UP_POSITION);
}

// tests:
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

int move_steps_accelstepper(int steps[2]) {
  Serial.println("Started move_steps_accelstepper");
  bool done_a = false;
  bool done_b = false;
  stepper_a.move(steps[0]);
  stepper_b.move(steps[1]);
  while (!(done_a && done_b)) {
    Serial.println("started while loop");
  done_a = !stepper_a.run();
  done_b = !stepper_b.run();
  if(!digitalRead(X_AXIS_END_SWITCH_0_PIN)){
    return 1;
  }
  if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
    return 2;
  }
  if(!digitalRead(Y_AXIS_END_SWITCH_0_PIN)){
    return 3;
  }
  if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
    return 4;
  }
  }
}

int move_steps_diagonal_support(int steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false){
  Serial.println("-------------------------------------------------------");
  Serial.println("####### move_steps_diagonal_support starting... ######");
  Serial.println("-------------------------------------------------------");
  bool motor_a_state = false;
  bool motor_b_state = false;
  int pos_a = 0;
  int pos_b = 0;
  bool done_a = false;
  bool done_b = false;
  // init counters:
  unsigned long current_millis;
  unsigned long previous_millis_a = 0;
  unsigned long previous_millis_b = 0;
  // set the directions of the steppers:
  if(steps[0] < 0) {
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
  }
  else {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
  }
  if(steps[1] < 0) {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
  }
  else {
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
  }
  // calculate time intervals vor the steppers:
  steps[0] = abs(steps[0]);
  steps[1] = abs(steps[1]);
  float ratio;
  unsigned int interval_a;
  unsigned int interval_b;
  if(steps[0] > steps[1]){
    Serial.println("A has more steps than b");
    ratio = steps[0]/steps[1];
    interval_a = working_speed_delay;
    interval_b = (int)ratio*working_speed_delay;
  }
  else {
    Serial.println(" has more steps than A");
    ratio = steps[1]/steps[0];
    interval_b = working_speed_delay;
    interval_a = (int)ratio*working_speed_delay;
  }
  Serial.print("Ratio: ");
  Serial.println(ratio);
  Serial.print("Interval A: ");
  Serial.println(interval_a);
  Serial.print("Interval B: ");
  Serial.println(interval_b);
  //return 0;
  while (!(done_a && done_b)) {
    current_millis = millis();
    if(current_millis - previous_millis_a >= interval_a) {
      previous_millis_a = current_millis;
      motor_a_state = !motor_a_state;
      digitalWrite(MOTOR_A_STEP_PIN, motor_a_state);
      Serial.print("Setting motor A ");
      Serial.println(digitalRead(motor_a_state));
      pos_a ++;
      Serial.print("Overall steps to go A: ");
      Serial.println(steps[0]);
      Serial.print("Steps gone A: ");
      Serial.println(pos_a);
      if(pos_a > steps[0]){
        done_a = true;
        Serial.println("A is done");
      }
    }
    if(current_millis - previous_millis_b >= interval_b) {
      previous_millis_b = current_millis;
      motor_b_state = !motor_b_state;
      digitalWrite(MOTOR_B_STEP_PIN, motor_b_state);
      Serial.print("Setting motor B ");
      Serial.println(motor_b_state);
      pos_b ++;
      Serial.print("Overall steps to go B: ");
      Serial.println(steps[1]);
      Serial.print("Steps gone B: ");
      Serial.println(pos_b);
      if(pos_b > steps[1]){
        done_b = true;
        Serial.println("B is done");
      }
    }
    if(!ignore_endswitches){
      if(!digitalRead(X_AXIS_END_SWITCH_0_PIN)){
        digitalWrite(MOTOR_A_STEP_PIN, LOW);
        digitalWrite(MOTOR_B_STEP_PIN, LOW);
        Serial.println("motor a LOW");
        Serial.println("motor B LOW");
        return 1;
      }
      if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
        digitalWrite(MOTOR_A_STEP_PIN, LOW);
        digitalWrite(MOTOR_B_STEP_PIN, LOW);
        Serial.println("motor a LOW");
        Serial.println("motor B LOW");
        return 2;
      }
      if(!digitalRead(Y_AXIS_END_SWITCH_0_PIN)){
        digitalWrite(MOTOR_A_STEP_PIN, LOW);
        digitalWrite(MOTOR_B_STEP_PIN, LOW);
        Serial.println("motor a LOW");
        Serial.println("motor B LOW");
        return 3;
      }
      if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
        digitalWrite(MOTOR_A_STEP_PIN, LOW);
        digitalWrite(MOTOR_B_STEP_PIN, LOW);
        Serial.println("motor a LOW");
        Serial.println("motor B LOW");
        return 4;
      }
    }
  }
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println("motor a LOW");
    Serial.println("motor B LOW");
  return 0;

}