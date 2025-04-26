#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h> // Pins: CS, DC, RST

#define TFT_CS    4
#define TFT_DC    7
#define TFT_RST   8  // Or -1 if you tie it to 3.3V

#define BLACK   0x0000
#define WHITE   0xFFFF
#define BLUE    0x001F

// Initialize display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Center position (make this variable)
int centerX = 150;  // Middle of 240px screen
int centerY = 160;  // Middle of 320px screen

void drawOpenEye() {
  tft.fillScreen(BLACK);

  // Sizes
  int eyeballSize = 60;  // Total area of the eyeball
  int irisSize = 60;
  int pupilSize = 30;
  int gap = 15;  // Gap size in pixels between the squares

  int quarterEyeball = eyeballSize / 2;
  int squareSize = quarterEyeball - gap/2; // Shrink each square slightly to leave gaps

  // Top-left square
  tft.fillRect(centerX - quarterEyeball, centerY - quarterEyeball, squareSize, squareSize, WHITE);

  // Top-right square
  tft.fillRect(centerX + gap/2, centerY - quarterEyeball, squareSize, squareSize, WHITE);

  // Bottom-left square
  tft.fillRect(centerX - quarterEyeball, centerY + gap/2, squareSize, squareSize, WHITE);

  // Bottom-right square
  tft.fillRect(centerX + gap/2, centerY + gap/2, squareSize, squareSize, WHITE);

  // // Draw blue iris (small centered square)
  // tft.fillRect(centerX - irisSize/2, centerY - irisSize/2, irisSize, irisSize, BLUE);

  // // Draw black pupil (tiny centered square)
  // tft.fillRect(centerX - pupilSize/2, centerY - pupilSize/2, pupilSize, pupilSize, BLACK);
}



void drawClosedEye() {
  tft.fillScreen(BLACK);
}

void setup() {
  tft.init(240, 320);
  tft.setRotation(1);
  drawOpenEye();
}

unsigned long blinkDuration = 300;
unsigned long blinkTime = 1000;
unsigned long lastBlinkTime = 0;
bool isEyeOpen = true;
bool isBlinking = false;

void loop() {
  unsigned long currentTime = millis();

  // Blinking
  if (isEyeOpen && currentTime - lastBlinkTime >= blinkTime) {
    drawClosedEye();
    isEyeOpen = false;
    isBlinking = true;
    lastBlinkTime = currentTime;
  }

  if (isBlinking && currentTime - lastBlinkTime >= blinkDuration) {
    drawOpenEye();
    isEyeOpen = true;
    isBlinking = false;
    lastBlinkTime = currentTime;
  }
}
