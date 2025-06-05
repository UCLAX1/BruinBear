#include "EyeDisplay.h"
#include "HeadControl.h"

// Create the EyeDisplay object
EyeDisplay eye;
HeadControl headServos;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  
  eye.begin();
  headServos.begin();
}

void loop() {
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
  
  delay(20); // Small delay
}