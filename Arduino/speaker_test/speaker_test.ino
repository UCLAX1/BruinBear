// Define the buzzer pin
int buzzerPin = 8;
#include "pitches.h"

// Define the "Happy Birthday" melody
int melody[] = {
  NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_G4, NOTE_G4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C4
};


int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 2, 4, 4, 2, 4, 4, 2, 4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 4, 2
};

// Note definitions for the melody


void setup() {
  // Iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 24; thisNote++) {
    // To calculate the note duration, take one second divided by the note type.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(buzzerPin, melody[thisNote], noteDuration);

    // To distinguish the notes, set a minimum time between them.
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);

    // Stop the tone playing:
    noTone(buzzerPin);
  }
}

void loop() {
  // No need to repeat the melody in the loop for this example.
  // The setup() is enough to play it once.
}