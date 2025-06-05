#include "EyeDisplay.h"
#include "HeadControl.h"

// Create the EyeDisplay object
EyeDisplay eye;
HeadControl headServos;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  
  eye.begin();
  headServos.begin();

  startTime = millis() / 1000;  
}

void loop() {
  unsigned long time = millis()/1000 - startTime;

  eye.update();
  
  // Check if serial data is available
  if (Serial.available() >= 2) {
    // Read 2 bytes from serial
    int leftTiltValue = Serial.read();  // First byte for left tilt servo
    int rightTiltValue = Serial.read(); // Second byte for right tilt servo
    
    // Constrain values to valid servo range (0-180)
    leftTiltValue = constrain(leftTiltValue, 1, 60);
    rightTiltValue = constrain(rightTiltValue, 120, 180);
    
    // Control the tilt servos directly
    headServos.setTiltPosition(leftTiltValue, rightTiltValue);
  }
  else {
    if (time < 2) {
      headServos.centerHead();
    } else if (time < 4) {
      headServos.spinLeft();
    } else if (time < 6) {
      headServos.spinRight();
    } else if (time < 8) {
      headServos.centerHead();
    } else if (time < 10) {
      headServos.lookLeft();
    } else if (time < 12) {
      headServos.lookRight();
    } else if (time < 14) {
      headServos.lookUp();
    } else if (time < 16) {
      headServos.lookDown();
    } else if (time < 18) {
      headServos.centerHead();
    } else if (time < 20) {
      headServos.openJaw();
    } else if (time < 22){
      headServos.closeJaw();
    }
  }
  
  delay(20); // Small delay
}