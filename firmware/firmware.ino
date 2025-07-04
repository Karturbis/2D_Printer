/**
 * This is the firmware for the 2D Printer.
 */

 // include libraries:
#include <TMC2208Stepper.h>  // for UART communication with the TMC2208 stepper drivers
#include <Servo.h>  // for controlling the servo

#include "config.h" // include configuration

// Function prototypes (needed, to use default arguments in the functions):
uint8_t move_steps(int_fast64_t steps[2], int working_speed_delay=WORKING_SPEED_DELAY, bool ignore_endswitches=false, bool ignore_direction=false);
uint8_t move_steps_linear_interpolation_time(int_fast64_t steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false);
uint8_t move_steps_linear_interpolation_slope(int_fast64_t steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false);

// initialzize varables
byte leds = 11100111;

// initialize TMC2208 class, use Software Serial Port for communication
TMC2208Stepper driver_a = TMC2208Stepper(MOTOR_A_RX_PIN, MOTOR_A_TX_PIN);
TMC2208Stepper driver_b = TMC2208Stepper(MOTOR_B_RX_PIN, MOTOR_B_TX_PIN);

// initialize toolhead servo:
Servo toolhead_servo;

void setup() {
  // Setup Pins:

    // bitshift register:
    pinMode(SER, OUTPUT);
    pinMode(SRCLK, OUTPUT);
    pinMode(RCLK, OUTPUT);

    // Axis Endswitch Pins:
    pinMode(X_AXIS_END_SWITCH_0_PIN, INPUT_PULLUP);
    pinMode(X_AXIS_END_SWITCH_1_PIN, INPUT_PULLUP);
    pinMode(Y_AXIS_END_SWITCH_0_PIN, INPUT_PULLUP);
    pinMode(Y_AXIS_END_SWITCH_1_PIN, INPUT_PULLUP);

    // Servo:
    pinMode(SERVO_PIN, OUTPUT);

    // Motors:
    pinMode(MOTOR_A_EN_PIN, OUTPUT);
    pinMode(MOTOR_B_EN_PIN, OUTPUT);
    pinMode(MOTOR_A_STEP_PIN, OUTPUT);
    pinMode(MOTOR_B_STEP_PIN, OUTPUT);
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);

  // Motor Setup:

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
    if(!DISABLE_MOTORS){
      digitalWrite(MOTOR_A_EN_PIN, LOW);    // Enable driver in hardware
      digitalWrite(MOTOR_B_EN_PIN, LOW);    // Enable driver in hardware
    }

  // connection with computer:
    Serial.begin(BAUD_RATE);
    Serial.setTimeout(100);
    Serial.flush(); // make sure the output buffer is empty
  
  // LEDs:
    update_bitshift_register();

  // Servo Setup:
    toolhead_servo.attach(SERVO_PIN);
    // Go to two different positions with the servo to show Setup is done: 
    toolhead_servo.write(SERVO_TEST_POSITION);
    delay(400);
    toolhead_servo.write((SERVO_UP_POSITION));
}

void loop() {
  /*
    Sends a 0 back, if everything was ok,
    if an error occured, the error code is sent
  */
  while (!Serial.available()) {} // wait until data is available
  String command = Serial.readString();  // read the command from the computer
  command.trim();  // remove unwanted characters from the command
  Serial.print(F("LOG:Command: "));
  Serial.println(command);
  if(command.startsWith(GO_TO)){  // a move command was received
    if(command.indexOf(".") > 0){ // check if command contains a ".", which hints a float
      Serial.println(F("A '.' was found, please use Integers, not Floats"));
      Serial.println(6);
    }
    int x_distance = command.substring(1, command.indexOf(SEPERATOR)).toInt(); // string until the seperator, ignoring first char
    int y_distance = command.substring(command.indexOf(SEPERATOR)+1).toInt(); // string until terminator, not include seperator
    Serial.print(F("DEBUG:X distance: "));
    Serial.println(x_distance);
    Serial.print(F("DEBUG:Y distance: "));
    Serial.println(y_distance);
    int exit_code = move(x_distance, y_distance); // move the toolhead
    // make whatever the exitcodes demands:
    if(exit_code == 1){
      Serial.println(F("CRITICAL:Toolhead ran into x-negativ wall, x-axis 0 triggered!"));
      Serial.println(F("CRITICAL:Home the Printer to move again!"));
    }
    else if(exit_code == 2){
      Serial.println(F("CRITICAL:Toolhead ran into x-positiv wall, x-axis 1 triggered!"));
      Serial.println("CRITICAL:Home the Printer to move again!");
    }
    else if(exit_code == 3){
      Serial.println(F("CRITICAL:Toolhead ran into y-positve wall, y-axis 0 triggered!"));
      Serial.println(F("CRITICAL:Home the Printer to move again!"));
    }
    else if(exit_code == 4){
      Serial.println(F("CRITICAL:Toolhead ran into y-negative wall, y-axis 1 triggered!"));
      Serial.println(F("CRITICAL:Home the Printer to move again!"));
    }
    Serial.println(exit_code);
  }
  else if (command.startsWith(HOMING)) {
    homing();
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
  else if (command.startsWith(CHANGE_TOOL)) {
    change_tool();
    Serial.println(0);
  }
  else {
    Serial.println(F("WARNING:Unknown command"));
    Serial.println(5);
  }
}

uint8_t move(int x_distance, int y_distance) {
  Serial.println(F("DEBUG:Starting to move ..."));
  int_fast64_t steps[2];
  steps[0] = (-x_distance - y_distance)*STEP_TO_MICROMETER_RATIO;
  steps[1] = (-x_distance + y_distance)*STEP_TO_MICROMETER_RATIO;
  Serial.println(F("LOG:-----------------------------------"));
  Serial.println(F("LOG:######## Start moving #############"));
  Serial.println(F("LOG:-----------------------------------"));
  // move the calculated steps with the step algorithm, that is defined in config.h:
  uint8_t exit_code;
  exit_code = STEPS_ALGORITHM(steps);
  Serial.println(F("Finished moving"));
  return exit_code;
}

uint8_t move_steps(int_fast64_t steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false, bool ignore_direction=false){
  // This algorithm works for all directions, where the angle to the x-axis is a miltiple of pi/4.
  // init positions:
  int_fast64_t pos_a = 0;
  int_fast64_t pos_b = 0;
  // set direction pins of the steppermotor drivers:
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
  // run the stepper motors:
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

uint8_t move_steps_linear_interpolation_time(int_fast64_t steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false){
  // has the same problem as move_steps() at the moment
  Serial.println(F("LOG:-------------------------------------------------------"));
  Serial.println(F("LOG:####### move_steps_diagonal_time starting... ########"));
  Serial.println(F("LOG:-------------------------------------------------------"));
  // init positions:
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
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
  }
  else {
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
  }
  if(steps[1] < 0) {
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
  }
  else {
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
  }
  // calculate time intervals vor the steppers:
  steps[0] = abs(steps[0]);
  steps[1] = abs(steps[1]);
  float ratio;
  unsigned int interval_a;
  unsigned int interval_b;
  if (steps[0] == steps[1]){
    // if the steps are equal, both intervals are the minimum interval
    Serial.println(F("DEBUG:Steps A == B"));
    interval_a = working_speed_delay;
    interval_b = working_speed_delay;
  }
  else {
    if(!(steps[0] && steps[1])){
      if(steps[0] > steps[1]){
        Serial.println(F("DEBUG:A has more steps than b"));
        ratio = steps[0]/steps[1];
        interval_a = working_speed_delay;
        interval_b = (int)(ratio*working_speed_delay);
      }
      else {
        Serial.println(F("DEBUG:B has more steps than A"));
        ratio = steps[1]/steps[0];
        interval_b = working_speed_delay;
        interval_a = (int)(ratio*working_speed_delay);
      }
      Serial.print(F("DEBUG:Ratio: "));
      Serial.println(ratio);
    }
    else {
      // only one motor has to move:
      if (!steps[0]){
        done_a = true;
        interval_b = working_speed_delay;
        Serial.println(F("DEBUG:No steps on motor A"));
      }
      if(!steps[1]){
        done_b = true;
        interval_a = working_speed_delay;
        Serial.println(F("DEBUG:No steps on motor B"));
      }
    }
  }
  Serial.print(F("DEBUG:Interval A: "));
  Serial.println(interval_a);
  Serial.print(F("DEBUG:Interval B: "));
  Serial.println(interval_b);
  // move the steps:
  while (!(done_a && done_b)) {
    current_micros = micros();
    if((current_micros - previous_micros_a >= HIGH_DELAY) && !done_a){
      digitalWrite(MOTOR_A_STEP_PIN, LOW);
    }
    if((current_micros - previous_micros_b >= HIGH_DELAY) && !done_b){
      digitalWrite(MOTOR_B_STEP_PIN, LOW);
    }
    if((current_micros - previous_micros_a >= interval_a) && !done_a) {
      previous_micros_a = current_micros;
      digitalWrite(MOTOR_A_STEP_PIN, HIGH);
      pos_a ++;
      if(pos_a > steps[0]){
        done_a = true;
      }
    }
    if((current_micros - previous_micros_b >= interval_b) && !done_b) {
      previous_micros_b = current_micros;
      digitalWrite(MOTOR_B_STEP_PIN, HIGH);
      pos_b ++;
      if(pos_b > steps[1]){
        done_b = true;
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
    Serial.println(F("DEBUG:motor a LOW"));
    Serial.println(F("DEBUG:motor B LOW"));
  return 0;
}

uint8_t move_steps_linear_interpolation_slope(int_fast64_t steps[2], int working_speed_delay = WORKING_SPEED_DELAY, bool ignore_endswitches=false){
  Serial.println(F("LOG:-------------------------------------------------------"));
  Serial.println(F("LOG:######## move_steps_diagonal_slope starting... ########"));
  Serial.println(F("LOG:-------------------------------------------------------"));
  bool done = false;
  // set the directions of the steppers:
  if(steps[0] < 0) {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
  }
  else {
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
  }
  if(steps[1] < 0) {
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
  }
  else {
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
  }
  // use move_steps, if the angle to the x-axis is a multiple of pi/4:
  if(!(steps[0] && steps[1]) || abs(steps[0]) == abs(steps[1])){
    return move_steps(steps);
  }
  /*
    use the absolute of steps, to simplify calculations,
    the sign is not needed anymore, because direction is
    already set
  */
  steps[0] = abs(steps[0]);
  steps[1] = abs(steps[1]);
  // calculate slope and long / short side:
  // init variables:
  uint8_t short_side;
  uint8_t long_side;
  uint8_t step_pin_short_side;
  uint8_t step_pin_long_side;
  int_fast64_t steps_short_position = 0;
  int_fast64_t steps_long_position = 0;
  float slope;
  // check, which side is bigger:
  if(steps[0] < steps[1]){
    Serial.println(F("DEBUG:Steps: A < B"));
    long_side = 1;
    short_side = 0;
    step_pin_short_side = MOTOR_A_STEP_PIN;
    step_pin_long_side = MOTOR_B_STEP_PIN;
    // calculate slope:
    slope = (float) steps[short_side] / (float) steps[long_side];
  }
  else {
    Serial.println(F("DEBUG:Steps: B < A"));
    long_side = 0;
    short_side = 1;
    step_pin_short_side = MOTOR_B_STEP_PIN;
    step_pin_long_side = MOTOR_A_STEP_PIN;
    // calculate slope:
    slope = (float) steps[short_side] / (float) steps[long_side];
  }
  Serial.print(F("DEBUG:Slope: "));
  Serial.println(slope);
  int_fast64_t diff;
  // move the steps:
  for(Serial.println(F("DEBUG:started for loop")); steps_long_position <= steps[long_side]; steps_long_position++) {
    digitalWrite(step_pin_long_side, HIGH);  // move the long side every iteration
    // calculate the difference between perfect and actal position:
    diff = round(steps_long_position * slope - steps_short_position);
    if(diff >= 1){ // check difference between calculated and actual position
      digitalWrite(step_pin_short_side, HIGH);
      steps_short_position ++;
    }
    // delay, so the drivers have time to receive the signal:
    delayMicroseconds(HIGH_DELAY);
    digitalWrite(step_pin_short_side, LOW);
    digitalWrite(step_pin_long_side, LOW);
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
  Serial.println(F("DEBUG:motor a LOW"));
  Serial.println(F("DEBUG:motor B LOW"));
  return 0;
}

uint8_t check_collision() {
  // check for collisions and return the appropriate exit code
  if(!digitalRead(X_AXIS_END_SWITCH_0_PIN)){
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println(F("DEBUG:motor a LOW"));
    Serial.println(F("DEBUG:motor B LOW"));
    return 1;
  }
  if(!digitalRead(X_AXIS_END_SWITCH_1_PIN)){
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println(F("DEBUG:motor a LOW"));
    Serial.println(F("DEBUG:motor B LOW"));
    return 2;
  }
  if(!digitalRead(Y_AXIS_END_SWITCH_0_PIN)){
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println(F("DEBUG:motor a LOW"));
    Serial.println(F("DEBUG:motor B LOW"));
    return 3;
  }
  if(!digitalRead(Y_AXIS_END_SWITCH_1_PIN)){
    digitalWrite(MOTOR_A_STEP_PIN, LOW);
    digitalWrite(MOTOR_B_STEP_PIN, LOW);
    Serial.println(F("DEBUG:motor a LOW"));
    Serial.println(F("DEBUG:motor B LOW"));
    return 4;
  }
  // no collision:
  return 0;
}

void _homing_x(int speed_delay) {
  int_fast64_t steps[2];
  // drive to x-axis stop using move:
  Serial.println(F("LOG:Homing X-Axis ..."));
  bool stop = !digitalRead(X_AXIS_END_SWITCH_0_PIN);
  steps[0] = 1;
  steps[1] = 1;
  while(!stop) {
    move_steps(steps, speed_delay, true);
    stop = !digitalRead(X_AXIS_END_SWITCH_0_PIN);
  }
  Serial.println(F("LOG:Hit the trigger, moving back"));
  // driving until the switch is not triggered anymore:
  steps[0] = -1;
  steps[1] = -1;
  while (stop) {
    move_steps(steps, speed_delay, true);
    stop = !digitalRead(X_AXIS_END_SWITCH_0_PIN);
  }
  Serial.println(F("LOG:Finished Homing X-Axis"));
  Serial.println(F("LOG:Backing up on X-Axis ..."));
  move(HOMING_MOVEBACK, 0);
}

void _homing_y(int speed_delay){
  int_fast64_t steps[2];
  // drive to y-axis stop using move
  Serial.println(F("LOG:Homing Y-Axis ..."));
  bool stop = !digitalRead(Y_AXIS_END_SWITCH_1_PIN);
  steps[0] = 1;
  steps[1] = -1;
  while(!stop) {
    move_steps(steps, speed_delay, true);
    stop = !digitalRead(Y_AXIS_END_SWITCH_1_PIN);
  }
  Serial.println(F("LOG:Hit the trigger, moving back"));
  // driving until the switch is not triggered anymore:
  steps[0] = -1;
  steps[1] = 1;
  while (stop) {
    move_steps(steps, speed_delay, true);
    stop = !digitalRead(Y_AXIS_END_SWITCH_1_PIN);
  }
  Serial.println(F("LOG:Finished Homing Y-Axis"));
  Serial.println(F("LOG:Backing up on Y-Axis ..."));
  move(0, HOMING_MOVEBACK);
}

void homing() {
  Serial.println(F("LOG:Start Homing ..."));
  // make sure toolhead is up:
  change_tool();
  // homing y two times, fast than slow:
  _homing_y(WORKING_SPEED_DELAY);
  _homing_y(HOMING_SPEED_DELAY);
  // homing x two times, fast than slow:
  _homing_x(WORKING_SPEED_DELAY);
  _homing_x(HOMING_SPEED_DELAY);
  // move to sofware home position:
  uint8_t sign_x = (HOMING_OFFSET_X > 0) - (HOMING_OFFSET_X < 0);
  uint8_t sign_y = (HOMING_OFFSET_Y > 0) - (HOMING_OFFSET_Y < 0);
  for(int offset_x = 0; offset_x < abs(HOMING_OFFSET_X); offset_x ++){
    move(sign_x*1000, 0);
  }
  for(int offset_y = 0; offset_y < abs(HOMING_OFFSET_Y); offset_y ++){
    move(0, sign_y*1000);
  }
  Serial.println(F("LOG:Finished Homing"));
}


// Toolhead:

void engage_toolhead(){
  toolhead_servo.write(SERVO_DOWN_POSITION);
}

void disengage_toolhead(){
  toolhead_servo.write(SERVO_UP_POSITION);
}

void change_tool(){
  toolhead_servo.write(SERVO_CHANGE_TOOL_POSITION);
}


// LEDs:

void enable_led(uint8_t led){
  bitSet(leds, led);
  update_bitshift_register();
}

void disable_led(uint8_t led){
  bitClear(leds, led);
  update_bitshift_register();
}

void update_bitshift_register(){
  digitalWrite(RCLK, LOW);  // disconnect input and output registers
  shiftOut(SER, SRCLK, LSBFIRST, leds);  // write LED data to input register
  digitalWrite(RCLK, HIGH);  // connect input and output registers, to update the LEDs
}
