#define SERVO_PIN 13
#define SERVO_UP_POSITION 10
#define SERVO_DOWN_POSITION 0
#define SPEED 60

#include <Servo.h>

Servo toolhead_servo;
int pos;

void setup() {
Serial.begin((115200));
toolhead_servo.attach(SERVO_PIN);

}

void loop() {
  Serial.println("waiting for instructions ...");
  while (!Serial.available()) {}
  String com = Serial.readString();
  if(com.startsWith("S")){
    stresstest();
  }
  pos = com.toInt();
  toolhead_servo.write(pos);

}

void stresstest(){
  while (true) {
    toolhead_servo.write(SERVO_UP_POSITION);
    delay(SPEED);
    toolhead_servo.write(SERVO_DOWN_POSITION);
    delay(SPEED);
  }
}
