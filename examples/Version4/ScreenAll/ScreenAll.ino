#include "PKS_ICON.h"
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void drawCompassHeading() {
  // Store the previous heading for clearing the display
  static int prevHeading = 0;
  static uint8_t x = 50, y = 52;
  
  // Read the compass heading from the robot's compass module
  int heading = robot.compassRead();

  // Clear previous heading display
  robot.tft.setTextColor(ST7735_BLACK);
  robot.tft.setCursor(x, y);
  robot.tft.print(prevHeading);

  // Display current heading
  robot.tft.setTextColor(ST7735_GREEN);
  robot.tft.setCursor(x, y);
  robot.tft.print(heading);

  // Update previous heading for next loop iteration
  prevHeading = heading;
}

void drawUltrasoundDistances() {
  // Store the previous distances for clearing the display
  static uint16_t prevDist[4] = {0, 0, 0, 0};
  static uint8_t x[4] = {10, 43, 75, 100}, y = 82;
  // Read and display distances from the ultrasound sensors
  for (uint8_t pos = Front; pos <= Left; pos++) {
    uint16_t dist = robot.ultrasoundGetDist(pos);
    if (dist != prevDist[pos]) {
      // Clear previous distance display
      robot.tft.setTextColor(ST7735_BLACK);
      robot.tft.setCursor(x[pos], y);
      robot.tft.print(prevDist[pos]);

      // Display current distance
      robot.tft.setTextColor(ST7735_GREEN);
      robot.tft.setCursor(x[pos], y);
      robot.tft.print(dist);

      // Update previous distance for next loop iteration
      prevDist[pos] = dist;
    }
  }
}

void drawCompoundEyeValues() {
  static uint8_t prevMaxIdx = 0, prevMaxVal = 0;
  static uint16_t prevAngle = 0;

  uint8_t  maxIdx = (uint8_t)robot.compoundEye.readMaxEye();
  uint8_t  maxVal = robot.compoundEye.readMaxEyeVal();
  uint16_t angle  = robot.compoundEyeAngleRead();

  // Clear previous values
  robot.tft.setTextColor(ST7735_BLACK);
  robot.tft.setCursor(23, 62); robot.tft.print(prevMaxIdx);
  robot.tft.setCursor(67, 62); robot.tft.print(prevMaxVal);
  robot.tft.setCursor(110, 62); robot.tft.print(prevAngle);
  // Display current values
  robot.tft.setTextColor(ST7735_GREEN);
  robot.tft.setCursor(23, 62); robot.tft.print(maxIdx);
  robot.tft.setCursor(67, 62); robot.tft.print(maxVal);
  robot.tft.setCursor(110, 62); robot.tft.print(angle);
  
  prevMaxIdx = maxIdx;
  prevMaxVal = maxVal;
  prevAngle = angle;
}

void drawColorSensorValues(void) {
  static uint8_t x[4] = {15, 43, 75, 100};
  static uint8_t yR = 92, yG = 102, yB = 112;
  static uint8_t yH = 122, yS = 132, yL = 142, yW = 152;
  // Store previous values for clearing the display
  static RGB  prevRgb[4] = {};
  static HSL  prevHsl[4] = {};
  static bool prevW[4]   = {false, false, false, false};

  for (uint8_t pos = Front; pos <= Left; pos++) {
    RGB  rgb  = robot.colorSensorReadRGB((SensorPos)pos);
    HSL  hsl  = robot.colorSensorReadHSL((SensorPos)pos);
    bool w    = robot.isWhiteLine((SensorPos)pos);

    // Clear previous values
    robot.tft.setTextColor(ST7735_BLACK);
    robot.tft.setCursor(x[pos], yR); robot.tft.print(prevRgb[pos].r);
    robot.tft.setCursor(x[pos], yG); robot.tft.print(prevRgb[pos].g);
    robot.tft.setCursor(x[pos], yB); robot.tft.print(prevRgb[pos].b);
    robot.tft.setCursor(x[pos], yH); robot.tft.print(prevHsl[pos].h);
    robot.tft.setCursor(x[pos], yS); robot.tft.print(prevHsl[pos].s);
    robot.tft.setCursor(x[pos], yL); robot.tft.print(prevHsl[pos].l);
    robot.tft.setCursor(x[pos], yW); robot.tft.print(prevW[pos] ? "T" : "F");

    // Display current values
    robot.tft.setTextColor(ST7735_GREEN);
    robot.tft.setCursor(x[pos], yR); robot.tft.print(rgb.r);
    robot.tft.setCursor(x[pos], yG); robot.tft.print(rgb.g);
    robot.tft.setCursor(x[pos], yB); robot.tft.print(rgb.b);
    robot.tft.setCursor(x[pos], yH); robot.tft.print(hsl.h);
    robot.tft.setCursor(x[pos], yS); robot.tft.print(hsl.s);
    robot.tft.setCursor(x[pos], yL); robot.tft.print(hsl.l);
    robot.tft.setTextColor(w ? ST7735_WHITE : ST7735_GREEN);
    robot.tft.setCursor(x[pos], yW); robot.tft.print(w ? "T" : "F");

    // Update previous values
    prevRgb[pos] = rgb;
    prevHsl[pos] = hsl;
    prevW[pos]   = w;
  }
}

void setup() {
  robot.init();
  // draw company logo
  robot.tft.drawBitmap(23, 0, PKS_ICONPKS_White_LOGO, 73, 49, ST7735_WHITE);
  // set text color and size
  robot.tft.setTextColor(ST7735_WHITE); robot.tft.setTextSize(1);
  // draw staic labels
  robot.tft.setCursor(0, 52);   robot.tft.print("Compass:");
  robot.tft.setCursor(0, 62);   robot.tft.print("max:");
  robot.tft.setCursor(43, 62);  robot.tft.print("Val:");
  robot.tft.setCursor(87, 62);  robot.tft.print("Ang:");
  robot.tft.setCursor(10, 72);  robot.tft.print("Front");
  robot.tft.setCursor(43, 72);  robot.tft.print("Right");
  robot.tft.setCursor(75, 72);  robot.tft.print("Back");
  robot.tft.setCursor(100, 72); robot.tft.print("Left");
  robot.tft.setCursor(0, 82);   robot.tft.print("U:");
  robot.tft.setCursor(0, 92);   robot.tft.print("R:");
  robot.tft.setCursor(0, 102);  robot.tft.print("G:");
  robot.tft.setCursor(0, 112);  robot.tft.print("B:");
  robot.tft.setCursor(0, 122);  robot.tft.print("H:");
  robot.tft.setCursor(0, 132);  robot.tft.print("S:");
  robot.tft.setCursor(0, 142);  robot.tft.print("L:");
  robot.tft.setCursor(0, 152);  robot.tft.print("W:");  
}

void loop() {
  drawCompassHeading();
  drawUltrasoundDistances();
  drawCompoundEyeValues();
  drawColorSensorValues();
  delay(10);
}