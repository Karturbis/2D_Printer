// Author Teemu Mäntykallio, 2017-04-07
// with modifications from karturbis

// Define pins
#define EN_PIN_A    2  // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_A  3  // Step on rising edge
#define DIR_PIN_A   4
#define TX_PIN_A    5
#define RX_PIN_A    6
#define EN_PIN_B    7  // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_B  8  // Step on rising edge
#define DIR_PIN_B   9
#define TX_PIN_B    10
#define RX_PIN_B    11

#include <TMC2208Stepper.h>                       // Include library
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <math.h>
TMC2208Stepper drivera = TMC2208Stepper(RX_PIN_A, TX_PIN_A);
TMC2208Stepper driverb = TMC2208Stepper(RX_PIN_B, TX_PIN_B);

AccelStepper steppera(AccelStepper::DRIVER, STEP_PIN_A, DIR_PIN_A);
AccelStepper stepperb(AccelStepper::DRIVER, STEP_PIN_B, DIR_PIN_B);

int posa = 900;
int posb = 1100;

void setup() {
  Serial.begin(9600);           // connect to computer
  drivera.beginSerial(115200);
  driverb.beginSerial(115200);
  drivera.push();                // Reset registers
  driverb.push();                // Reset registers

  // Prepare pins
  pinMode(EN_PIN_A, OUTPUT);
  pinMode(EN_PIN_B, OUTPUT);
  pinMode(STEP_PIN_A, OUTPUT);
  pinMode(STEP_PIN_B, OUTPUT);
  digitalWrite(EN_PIN_A, HIGH);   // Disable driver in hardware
  digitalWrite(EN_PIN_B, HIGH);   // Disable driver in hardware

  drivera.pdn_disable(true);     // Use PDN/UART pin for communication
  driverb.pdn_disable(true);     // Use PDN/UART pin for communication
  drivera.I_scale_analog(false); // Use internal voltage reference
  driverb.I_scale_analog(false); // Use internal voltage reference
  drivera.rms_current(400);      // Set driver current 400mA
  driverb.rms_current(400);      // Set driver current 400mA
  drivera.toff(2);               // Enable driver in software
  driverb.toff(2);               // Enable driver in software

  digitalWrite(EN_PIN_A, LOW);    // Enable driver in hardware
  digitalWrite(EN_PIN_B, LOW);    // Enable driver in hardware

  steppera.setMaxSpeed(40000.0);
  steppera.setAcceleration(3000.0);
  stepperb.setMaxSpeed(40000.0);
  stepperb.setAcceleration(3000.0);
  

  steppera.moveTo(posa);
  stepperb.moveTo(posb);
}
void loop() {
  //move(2.0, 800);
  /*
  move(M_PI, 500);
  delay(500);
  move(2.4, 500);
  delay(500);
  move(-M_PI, 500);
  move(2*M_PI-2.4, 500);
  */
  bool runninga = steppera.run();
  if (!runninga) {
    steppera.moveTo(- steppera.currentPosition());
  }
  bool runningb = stepperb.run();
  if (!runningb) {
    steppera.moveTo(- stepperb.currentPosition());
  }
}
void move(float direction, int dist) {
  // direction in radians
  int steps[2];
  int delta_x = (int)sin(direction)*dist;
  int delta_y = (int)cos(direction)*dist;
  steps[0] = (delta_x + delta_y);
  steps[1] = (delta_x - delta_y);
  steppera.moveTo(steps[0]);
  stepperb.moveTo(steps[1]);
  while(steppera.distanceToGo() > 0 && stepperb.distanceToGo() > 0) {
    steppera.run();
    stepperb.run();
  }
    
  }

