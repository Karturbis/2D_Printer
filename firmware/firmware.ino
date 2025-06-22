/**
 * This is the firmware for the 2D Printer.
 */

 // include libraries:
#include <AccelStepper.h>
#include <TMC2208Stepper.h>
#include <Servo.h>

#include "config.h" // include configuration

// Function prototypes:
int move_steps(int steps[2], int working_speed_delay=WORKING_SPEED_DELAY, bool ignore_endswitches=false, bool ignore_direction=false);
int move_steps_diagonal_micros(int steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false);
int move_steps_diagonal_slope(int steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false);


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
        stepper_b.setMaxSpeed(MAX_SPEED);
        stepper_a.setAcceleration(ACCELERATION);
        stepper_b.setAcceleration(ACCELERATION);
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
  Serial.print("LOG:Command: ");
  Serial.println(command);
  if(command.startsWith(GO_TO)){
    String direction = command.substring(1, command.indexOf(SEPERATOR)); // string until the seperator, ignoring first char
    String distance = command.substring(command.indexOf(SEPERATOR)+1); // string until terminator, not include seperator
    Serial.print("DEBUG:Direction: ");
    Serial.println(direction);
    Serial.print("DEBUG:Distance: ");
    Serial.println(distance);
    int error = move(direction.toFloat(), distance.toInt()); // go to
    if(error == 1){
      Serial.println("CRITICAL:Toolhead ran into wall, x-axis 0 triggered!");
      Serial.println("CRITICAL:Home the Printer to move again!");
    }
    else if(error == 2){
      Serial.println("CRITICAL:Toolhead ran into wall, x-axis 1 triggered!");
      Serial.println("CRITICAL:Home the Printer to move again!");
    }
    else if(error == 3){
      Serial.println("CRITICAL:Toolhead ran into wall, y-axis 0 triggered!");
      Serial.println("CRITICAL:Home the Printer to move again!");
    }
    else if(error == 4){
      Serial.println("CRITICAL:Toolhead ran into wall, y-axis 1 triggered!");
      Serial.println("CRITICAL:Home the Printer to move again!");
    }
    Serial.println(error);
  }
  else if (command.startsWith(HOMING)) {
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
  Serial.println("DEBUG:Starting to move ...");
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
  Serial.println("LOG:-----------------------------------");
  Serial.println("LOG:######## Start moving #############");
  Serial.println("LOG:-----------------------------------");
  Serial.print("DEBUG:Delta_X: ");
  Serial.println(delta_x);
  Serial.print("DEBUG:Delta_y: ");
  Serial.println(delta_y);
  Serial.print("DEBUG:Steps[0]: ");
  Serial.println(steps[0]);
  Serial.print("DEBUG:Steps[1]: ");
  Serial.println(steps[1]);
  // move:
  int error;
  error = STEPS_ALGORITHM(steps);
  // Update Position:
  position[0] = position[0] + delta_x;
  position[1] = position[1] + delta_y;
  Serial.print("LOG:New Position: ");
  Serial.print(position[0]);
  Serial.print(" ");
  Serial.println(position[1]);
  return error;
}

int move_steps(int steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false, bool ignore_direction=false){
  int pos_a = 0;
  int pos_b = 0;
  if(!ignore_direction){
    if(steps[0] < 0){
      digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    }
    else{
      digitalWrite(MOTOR_A_DIR_PIN, LOW);
    }
    if(steps[1] < 0){
      digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    }
    else {
      digitalWrite(MOTOR_B_DIR_PIN, LOW);    
    }
  }
  while (abs(pos_a) < abs(steps[0]) || abs(pos_b) < abs(steps[1])) {
    if(pos_a < steps[0]) {
      digitalWrite(MOTOR_A_STEP_PIN, HIGH);
      pos_a ++;
    }
    else if(pos_a > steps[0]) {
      digitalWrite(MOTOR_A_STEP_PIN, HIGH);
      pos_a --;
    }
    if(pos_b < steps[1]) {
      digitalWrite(MOTOR_B_STEP_PIN, HIGH);
      pos_b ++;
    }
    else if(pos_b > steps[1]) {
      digitalWrite(MOTOR_B_STEP_PIN, HIGH);
      pos_b --;
    }
    if(!ignore_endswitches){
      uint8_t collision = check_collision();
      if(collision){
        return collision;
      }
    }
    delayMicroseconds(working_speed_delay);
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
  }
  return 0;

}

int check_collision() {
  if(!digitalRead(X_AXIS_END_SWITCH_0_PIN)){
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println("DEBUG:motor a LOW");
    Serial.println("DEBUG:motor B LOW");
    return 1;
  }
  if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println("DEBUG:motor a LOW");
    Serial.println("DEBUG:motor B LOW");
    return 2;
  }
  if(!digitalRead(Y_AXIS_END_SWITCH_0_PIN)){
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println("DEBUG:motor a LOW");
    Serial.println("DEBUG:motor B LOW");
    return 3;
  }
  if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println("DEBUG:motor a LOW");
    Serial.println("DEBUG:motor B LOW");
    return 4;
  }
  // no collision:
  return 0;
  
}

void _homeing_x(int speed_delay) {
  int steps[2];
  // drive to x-axis stop using move:
  Serial.println("LOG:Homing X-Axis ...");
  bool stop = !digitalRead(X_AXIS_END_SWITCH_0_PIN);
  steps[0] = 1;
  steps[1] = 1;
  while(!stop) {
    move_steps(steps, speed_delay, true);
    stop = !digitalRead(X_AXIS_END_SWITCH_0_PIN);
  }
  Serial.println("LOG:Hit the trigger, moving back");
  // driving until the switch is not triggered anymore:
  steps[0] = -1;
  steps[1] = -1;
  while (stop) {
    move_steps(steps, speed_delay, true);
    stop = !digitalRead(X_AXIS_END_SWITCH_0_PIN);
  }
  position[0] = 0;
  Serial.println("LOG:Finished Homing X-Axis");
  Serial.println("LOG:Backing up on X-Axis ...");
  move(HALF_PI, 10000);
}

void _homeing_y(int speed_delay){
  int steps[2];
  // drive to y-axis stop using move
  Serial.println("LOG:Homing Y-Axis ...");
  bool stop = !digitalRead(Y_AXIS_END_SWITCH_0_PIN);
  steps[0] = 1;
  steps[1] = -1;
  while(!stop) {
    move_steps(steps, speed_delay, true);
    stop = !digitalRead(Y_AXIS_END_SWITCH_0_PIN);
  }
  Serial.println("LOG:Hit the trigger, moving back");
  // driving until the switch is not triggered anymore:
  steps[0] = -1;
  steps[1] = 1;
  while (stop) {
    move_steps(steps, speed_delay, true);
    stop = !digitalRead(Y_AXIS_END_SWITCH_0_PIN);
  }
  position[1] = 0;
  Serial.println("LOG:Finished Homing Y-Axis");
  Serial.println("LOG:Backing up on Y-Axis ...");
  move(PI, 10000);
  Serial.println("LOG:Finished Homing");
}

void homeing() {
  // make sure toolhead is up:
  Serial.println("LOG:Start Homing ...");
  disengage_toolhead();
  // homing x two times, fast than slow:
  _homeing_x(WORKING_SPEED_DELAY);
  _homeing_x(HOMING_SPEED_DELAY);
  // homing y two times, fast than slow:
  _homeing_y(WORKING_SPEED_DELAY);
  _homeing_y(HOMING_SPEED_DELAY);
  // move to sofware home position:
  uint8_t sign_x = (HOMING_OFFSET_X > 0) - (HOMING_OFFSET_X < 0);
  uint8_t sign_y = (HOMING_OFFSET_Y > 0) - (HOMING_OFFSET_Y < 0);
  for(int offset_x = 0; offset_x < abs(HOMING_OFFSET_X); offset_x ++){
    move(0.0, sign_x*10000);
  }
  for(int offset_y = 0; offset_y < abs(HOMING_OFFSET_Y); offset_y ++){
    move(HALF_PI, sign_y*10000);
  }
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
    Serial.println("DEBUG:Direction HIGH");
    for (int i = 0; i < 500; i++) {
        digitalWrite(MOTOR_A_STEP_PIN, !digitalRead(MOTOR_A_STEP_PIN));
        digitalWrite(MOTOR_B_STEP_PIN, !digitalRead(MOTOR_B_STEP_PIN));
        delay(1);
    }
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
    Serial.println("DEBUG:Direction LOW");
    for (int i = 0; i < 500; i++) {
        digitalWrite(MOTOR_A_STEP_PIN, !digitalRead(MOTOR_A_STEP_PIN));
        digitalWrite(MOTOR_B_STEP_PIN, !digitalRead(MOTOR_B_STEP_PIN));
        delay(1);
    }
  }

}

int move_steps_accelstepper(int steps[2]) {
  Serial.println("LOG:Started move_steps_accelstepper");
  bool done_a = false;
  bool done_b = false;
  stepper_a.move(steps[0]);
  stepper_b.move(steps[1]);
  uint8_t collision;
  while (!(done_a && done_b)) {
    Serial.println("LOOPDEBUG:started while loop");
    done_a = !stepper_a.run();
    done_b = !stepper_b.run();
    collision = check_collision();
    if(collision) {
      return collision;
    }
  }
}

int move_steps_diagonal_micros(int steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false){
  Serial.println("LOG:-------------------------------------------------------");
  Serial.println("LOG:####### move_steps_diagonal_micros starting... ########");
  Serial.println("LOG:-------------------------------------------------------");
  bool motor_a_state = false;
  bool motor_b_state = false;
  int pos_a = 0;
  int pos_b = 0;
  bool done_a = false;
  bool done_b = false;
  // init counters:
  unsigned long current_micros;
  unsigned long previous_micros_a = 0;
  unsigned long previous_micros_b = 0;
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
  if (steps[0] == steps[1]){
    Serial.println("DEBUG:Steps A == B");
    interval_a = working_speed_delay;
    interval_b = working_speed_delay;
  }
  else {
    if(!(steps[0] && steps[1])){
      if(steps[0] > steps[1]){
        Serial.println("DEBUG:A has more steps than b");
        ratio = steps[0]/steps[1];
        interval_a = working_speed_delay;
        interval_b = (int)(ratio*working_speed_delay) >> 1;  // divide by two, to compensate raising edge every two iterations
      }
      else {
        Serial.println("DEBUG:B has more steps than A");
        ratio = steps[1]/steps[0];
        interval_b = working_speed_delay;
        interval_a = (int)(ratio*working_speed_delay) >> 1;  // divide by two, to compensate raising edge every two iterations
      }
      Serial.print("DEBUG:Ratio: ");
      Serial.println(ratio);
    }
    else {
      if (!steps[0]){
        done_a = true;
        interval_b = working_speed_delay;
        Serial.println("DEBUG:No steps on motor A");
      }
      if(!steps[1]){
        done_b = true;
        interval_a = working_speed_delay;
        Serial.println("DEBUG:No steps on motor B");
      }
    }
  }
  Serial.print("DEBUG:Interval A: ");
  Serial.println(interval_a);
  Serial.print("DEBUG:Interval B: ");
  Serial.println(interval_b);
  //return 0;

  while (!(done_a && done_b)) {
    current_micros = micros();
    if((current_micros - previous_micros_a >= interval_a) && !done_a) {
      previous_micros_a = current_micros;
      motor_a_state = !motor_a_state;
      digitalWrite(MOTOR_A_STEP_PIN, motor_a_state);
      Serial.print("LOOPDEBUG:Setting motor A ");
      Serial.println(digitalRead(motor_a_state));
      pos_a ++;
      Serial.print("LOOPDEBUG:Overall steps to go A: ");
      Serial.println(steps[0]);
      Serial.print("LOOPDEBUG:Steps gone A: ");
      Serial.println(pos_a);
      if(pos_a > steps[0]){
        done_a = true;
        Serial.println("LOOPDEBUG:A is done");
      }
    }
    if((current_micros - previous_micros_b >= interval_b) && !done_b) {
      previous_micros_b = current_micros;
      motor_b_state = !motor_b_state;
      digitalWrite(MOTOR_B_STEP_PIN, motor_b_state);
      Serial.print("LOOPDEBUG:Setting motor B ");
      Serial.println(motor_b_state);
      pos_b ++;
      Serial.print("LOOPDEBUG:Overall steps to go B: ");
      Serial.println(steps[1]);
      Serial.print("LOOPDEBUG:Steps gone B: ");
      Serial.println(pos_b);
      if(pos_b > steps[1]){
        done_b = true;
        Serial.println("LOOPDEBUG:B is done");
      }
    }
    if(!ignore_endswitches){
      uint8_t collision = check_collision();
      if(collision){
        return collision;
      }
    }
  }
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println("DEBUG:motor a LOW");
    Serial.println("DEBUG:motor B LOW");
  return 0;

}

int move_steps_diagonal_slope(int steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false){
  Serial.println("LOG:-------------------------------------------------------");
  Serial.println("LOG:######## move_steps_diagonal_slope starting... ########");
  Serial.println("LOG:-------------------------------------------------------");
  bool done = false;
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
  if(!(steps[0] && steps[1]) || abs(steps[0]) == abs(steps[1])){
    Serial.println("DEBUG:Using move_steps to move");
    Serial.print("DEBUG:steps A are: ");
    Serial.println(steps[0]);
    Serial.print("DEBUG:steps B are: ");
    Serial.println(steps[1]);
    return move_steps(steps);
  }
  steps[0] = abs(steps[0]);
  steps[1] = abs(steps[1]);
  // calculate slope and long / short side:
  uint8_t short_side;
  uint8_t long_side;
  uint8_t step_pin_short_side;
  uint8_t step_pin_long_side;
  long steps_short_position = 0;
  long steps_long_position = 0;
  float slope;
  if(steps[0] < steps[1]){
    Serial.println("DEBUG:Steps: A < B");
    long_side = 1;
    short_side = 0;
    step_pin_short_side = MOTOR_A_STEP_PIN;
    step_pin_long_side = MOTOR_B_STEP_PIN;
    slope = (float) steps[short_side] / (float) steps[long_side];
  }
  else {
    Serial.println("DEBUG:Steps: B < A");
    long_side = 0;
    short_side = 1;
    step_pin_short_side = MOTOR_B_STEP_PIN;
    step_pin_long_side = MOTOR_A_STEP_PIN;
    Serial.print("DEBUG:short side: ");
    Serial.println(short_side);
    Serial.print("DEBUG:steps short side: ");
    Serial.println(steps[short_side]);
    Serial.print("DEBUG:long side: ");
    Serial.println(long_side);
    Serial.print("DEBUG:steps long side: ");
    Serial.println(steps[long_side]);
    slope = (float) steps[short_side] / (float) steps[long_side];
  }
  Serial.print("DEBUG:Slope: ");
  Serial.println(slope);
  long diff;
  for(Serial.println("DEBUG:started for loop"); steps_long_position <= steps[long_side]; steps_long_position++) {
    digitalWrite(step_pin_long_side, HIGH);
    Serial.print("LOOPDEBUG:Set Motor ");
    Serial.print(long_side);
    Serial.println(" HIGH");
    diff = round(steps_long_position * slope - steps_short_position);
    Serial.print("LOOPDEBUG:diff == ");
    Serial.println(diff);
    if(diff >= 1){ // check difference between calculated and actual position
      digitalWrite(step_pin_short_side, HIGH);
      steps_short_position ++;
      Serial.print("LOOPDEBUG:Set Motor ");
      Serial.print(short_side);
      Serial.println(" HIGH");
    }

    delayMicroseconds(HIGH_DELAY);
    digitalWrite(step_pin_short_side, LOW);
    digitalWrite(step_pin_long_side, LOW);
    Serial.println("LOOPDEBUG:Setting both motors LOW");
    delayMicroseconds(working_speed_delay);
    if(!ignore_endswitches){
      uint8_t collision = check_collision();
      if(collision){
        return collision;
      }
    }
  }
  digitalWrite(MOTOR_A_STEP_PIN, LOW);
  digitalWrite(MOTOR_B_STEP_PIN, LOW);
  Serial.println("DEBUG:motor a LOW");
  Serial.println("DEBUG:motor B LOW");
  return 0;
}