#include <Servo.h>

Servo toolhead;

int pos = 5;    // variable to store the servo position

/*Modes:
  0: Manual over Serial Port
  1: Automatic, going up and down*/
const int mode = 0;

void setup() {
  toolhead.attach(9);
  Serial.begin(9600);
}

void loop() {
  if(mode==0){
    manual_mode();
  }
  if(mode==1){
    automatic_mode();
  }
}

void automatic_mode() {
  int counter = 0;
  while (counter < 500) {
    toolhead.write(5);
    Serial.println(toolhead.read());
    delay(250);
    toolhead.write(85);
    Serial.println(toolhead.read());
    delay(250);
    Serial.println(counter);
    counter ++;
  }
}

void manual_mode() {
  while (true) {
    if(Serial.available() > 0){
        pos = Serial.parseInt();
        toolhead.write(pos);
        Serial.println(pos);
        delay(100);
      }
  }
}