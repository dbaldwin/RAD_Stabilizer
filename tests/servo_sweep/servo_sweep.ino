// This example sketch tests CH2 (left elevon) and CH3 (right elevon) servos

#include "Servo.h"

Servo leftElevon;
Servo rightElevon;

void setup() {
  leftElevon.attach(9);
  rightElevon.attach(10);

}

void loop() {

  leftElevon.right(0);

}
