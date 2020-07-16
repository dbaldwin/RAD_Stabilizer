// This example sketch tests CH2 (left elevon) and CH3 (right elevon) servos
// by sweeping forward and backward
// Pitch - positive nose up (this means both elevons towards the sky)
// Roll - positive right wing down (this means right elevon up an left elevon down)

#include "Servo.h"

Servo leftElevon;
Servo rightElevon;
int counter = 0;

// 1 is counting up
// 0 is counting down
int direction = 1;

void setup() {

  Serial.begin(9600);

  // Setup servos on digital pins 5 and 6
  leftElevon.attach(5);
  rightElevon.attach(6);

  // Reset the servos
  leftElevon.write(counter);
  rightElevon.write(counter);

}

void loop() {

  // Move the servos 1 degree at a time from 0 to 180 and back down
  leftElevon.write(counter);
  rightElevon.write(counter);

  // Change the direction when we get to 180
  if (counter == 180) {
    direction = 0;
  // Change the direction when we get to 0
  } else if (counter == 0) {
    direction = 1;
  }

  // Count up
  if (direction) {
    counter++;
  // Count down
  } else {
    counter--;
  }

  // Print the counter to the serial monitor
  Serial.println(counter);

  // Delay 100ms
  delay(100);

}
