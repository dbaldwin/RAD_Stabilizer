// This example sketch tests CH2 (left elevon) and CH3 (right elevon) servos
// Pitch - positive nose up (this means both elevons towards the sky)
// Roll - positive right wing down (this means right elevon up an left elevon down)

#include "Servo.h"

Servo leftElevon;
Servo rightElevon;
int counter = 0;

int direction = 1;

void setup() {

  
  leftElevon.attach(5);
  rightElevon.attach(6);

  leftElevon.write(0);
  rightElevon.write(0);

}

void loop() {

  leftElevon.write(counter);
  rightElevon.write(counter);


  if (counter == 180) {
    direction = 0;
  } else if (counter == 0) {
    direction = 1;
  }

  if (direction) {
    counter++;
  } else {
    counter--;
  }

  Serial.println(counter);

  delay(100);

}
