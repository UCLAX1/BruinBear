// HeadControl.h

#ifndef HEADCONTROL_H
#define HEADCONTROL_H

#include <Servo.h>

class HeadControl {
  private:
    Servo spinServo;        // Servo for spinning (turning left/right)
    Servo leftTiltServo;    // Servo for left tilt
    Servo rightTiltServo;   // Servo for right tilt
    Servo jawServo;         // Servo for opening/closing jaw

    // Default spin positions
    const int spinCenterPos = 60;
    const int spinLeftPos = 10;
    const int spinRightPos = 110;

    // Default tilt positions
    const int tiltCenter[2] = {60, 120};  // {leftTilt, rightTilt}
    const int tiltUp[2]    = {1, 180};
    const int tiltDown[2]  = {60, 120};
    const int tiltLeft[2]  = {1, 120};
    const int tiltRight[2] = {60, 180};

    // Jaw positions
    const int jawClosedPos = 60;
    const int jawOpenPos = jawClosedPos + 60;

    // Servo pins
    const int SPIN_SERVO_PIN = 3;
    const int LEFT_TILT_SERVO_PIN = 6;
    const int RIGHT_TILT_SERVO_PIN = 5;
    const int JAW_SERVO_PIN = 10;

  public:
    void begin() {
      spinServo.attach(SPIN_SERVO_PIN);
      leftTiltServo.attach(LEFT_TILT_SERVO_PIN);
      rightTiltServo.attach(RIGHT_TILT_SERVO_PIN);
      jawServo.attach(JAW_SERVO_PIN);

      centerHead();
      closeJaw();
    }

    void centerHead() {
      spinServo.write(spinCenterPos);
      leftTiltServo.write(tiltCenter[0]);
      rightTiltServo.write(tiltCenter[1]);
    }

    void spinLeft() {
      spinServo.write(spinLeftPos);
    }

    void spinRight() {
      spinServo.write(spinRightPos);
    }

    void lookUp() {
      leftTiltServo.write(tiltUp[0]);
      rightTiltServo.write(tiltUp[1]);
    }

    void lookDown() {
      leftTiltServo.write(tiltDown[0]);
      rightTiltServo.write(tiltDown[1]);
    }

    void lookLeft() {
      leftTiltServo.write(tiltLeft[0]);
      rightTiltServo.write(tiltLeft[1]);
    }

    void lookRight() {
      leftTiltServo.write(tiltRight[0]);
      rightTiltServo.write(tiltRight[1]);
    }

    void openJaw() {
      jawServo.write(jawOpenPos);
    }

    void closeJaw() {
      jawServo.write(jawClosedPos);
    }
};

#endif
