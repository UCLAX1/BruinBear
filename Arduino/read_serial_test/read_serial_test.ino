// led_control.ino

#define LED_PIN 13  // built-in LED on most Arduino boards

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // remove any whitespace or newline characters

    if (command == "ON") {
      digitalWrite(LED_PIN, HIGH);
    } else if (command == "OFF") {
      digitalWrite(LED_PIN, LOW);
    }
  }
}
