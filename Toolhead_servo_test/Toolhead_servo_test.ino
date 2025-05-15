#include <Servo.h>

Servo toolhead;

int pos = 0;    // variable to store the servo position

void setup() {
  toolhead.attach(9);
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() > 0){
    pos = Serial.parseInt();
    toolhead.write(pos);
    delay(100);
  }
}