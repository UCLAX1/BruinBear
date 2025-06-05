// #include "EyeDisplay.h"
// #include "HeadControl.h"
// #include <Servo.h>

// // Create the EyeDisplay object
// EyeDisplay eye;
// HeadControl headServos;
// unsigned long startTime;

// void setup() {
//   eye.begin();

//   headServos.begin();

//   startTime = millis() / 1000;  
// }


// void loop() {
//   unsigned long time = millis()/1000 - startTime;


//   eye.update();

//   if (time < 2) {
//     headServos.centerHead();
//   } else if (time < 4) {
//     headServos.spinLeft();
//   } else if (time < 6) {
//     headServos.spinRight();
//   } else if (time < 8) {
//     headServos.centerHead();
//   } else if (time < 10) {
//     headServos.lookLeft();
//   } else if (time < 12) {
//     headServos.lookRight();
//   } else if (time < 14) {
//     headServos.lookUp();
//   } else if (time < 16) {
//     headServos.lookDown();
//   } else if (time < 18) {
//     headServos.centerHead();
//   } else if (time < 20) {
//     headServos.openJaw();
//   } else if (time < 22){
//     headServos.closeJaw();
//   }

// }
