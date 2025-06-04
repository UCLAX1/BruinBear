// led_control.ino

#define LED_PIN 13  // built-in LED on most Arduino boards

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (Serial.available() >= 2) {
    // String command = Serial.readStringUntil('\n');
    // command.trim(); // remove any whitespace or newline characters

    byte value1 = Serial.read();  // Read first byte
    byte value2 = Serial.read();  // Read second byte

    if (value1 == 60 && value2 == 120) {
      digitalWrite(LED_PIN, HIGH);
    } else if (value1 == 1 && value2 == 180) {
      digitalWrite(LED_PIN, LOW);
    }
  }

    // void setup() {
//   Serial.begin(9600);  // Set baud rate to match sender
//   pinMode(13, OUTPUT);
// }

// void loop() {
//   if (Serial.available() >= 2) {  // Wait until two bytes are available
//     byte value1 = Serial.read();  // Read first byte
//     byte value2 = Serial.read();  // Read second byte

//     // Print the values
//     Serial.print("Value 1: ");
//     Serial.print(value1);
//     Serial.print(" | Value 2: ");
//     Serial.println(value2);

//     if (value1 == 60) {
//       digitalWrite(13, HIGH);
//     }

//   }
// }
  }
