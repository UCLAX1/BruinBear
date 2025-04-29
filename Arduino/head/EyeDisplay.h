// EyeDisplay.h

#ifndef EYEDISPLAY_H
#define EYEDISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define BLACK   0x0000
#define WHITE   0xFFFF
#define BLUE    0x001F

// Hardcoded pins for display
#define TFT_CS    4
#define TFT_DC    7
#define TFT_RST   8

class EyeDisplay {
  private:
    Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

    // Hardcoded eye center
    const int centerX = 150;
    const int centerY = 160;

    // Timing parameters
    unsigned long blinkDuration = 300;
    unsigned long blinkTime = 1000;
    unsigned long lastBlinkTime = 0;
    bool isEyeOpen = true;
    bool isBlinking = false;

  public:
    EyeDisplay() {} // No parameters constructor

    void begin() {
      tft.init(240, 320);
      tft.setRotation(1);
      drawOpenEye();
    }

    void update() {
      unsigned long currentTime = millis();
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

    void drawOpenEye() {
      tft.fillScreen(BLACK);

      int eyeballSize = 60;
      int gap = 15;
      int quarterEyeball = eyeballSize / 2;
      int squareSize = quarterEyeball - gap/2;

      // Draw 4 white squares
      tft.fillRect(centerX - quarterEyeball, centerY - quarterEyeball, squareSize, squareSize, WHITE);
      tft.fillRect(centerX + gap/2, centerY - quarterEyeball, squareSize, squareSize, WHITE);
      tft.fillRect(centerX - quarterEyeball, centerY + gap/2, squareSize, squareSize, WHITE);
      tft.fillRect(centerX + gap/2, centerY + gap/2, squareSize, squareSize, WHITE);
    }

    void drawClosedEye() {
      tft.fillScreen(BLACK);
    }
};

#endif
