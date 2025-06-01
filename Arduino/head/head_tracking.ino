#include <Servo.h>

// Define pins
const int pwmPin1 = 2;
const int pwmPin2 = 3;
const int servoPin1 = 9;
const int servoPin2 = 10;

// Create servo objects
Servo servo1;
Servo servo2;

// Map PWM pulse width (µs) to value in a custom range
int pulseToValue(int pulseWidth, int outMin, int outMax, bool reverse = false) {
  pulseWidth = constrain(pulseWidth, 1000, 2000);
  if (reverse) {
    return map(pulseWidth, 1000, 2000, outMax, outMin);
  } else {
    return map(pulseWidth, 1000, 2000, outMin, outMax);
  }
}

void setup() {
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);

  pinMode(pwmPin1, INPUT);
  pinMode(pwmPin2, INPUT);
}

void loop() {
  // Read PWM pulse widths (in microseconds)
  unsigned long pwm1 = pulseIn(pwmPin1, HIGH, 25000); // Timeout: 25ms
  unsigned long pwm2 = pulseIn(pwmPin2, HIGH, 25000);

  if (pwm1 > 0 && pwm2 > 0) {
    // Byte 1: from 60 to 1 (reverse range)
    int val1 = pulseToValue(pwm1, 1, 60, true);
    // Byte 2: from 120 to 180 (normal range)
    int val2 = pulseToValue(pwm2, 120, 180);

    // Write to servos
    servo1.write(val1);
    servo2.write(val2);
  }

  delay(20); // Small delay
}