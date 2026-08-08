/**
 * This example demonstrates how to read IR compound eye sensors and display
 * the individual IR readings, the ball angle indicator, and the maximum eye
 * value on the TFT screen.
 */

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

// Global variables to store previous values for clearing the display
int prevEye[12] = {0};
int prevMaxEye = 0;
int prevMaxEyeVal = 0;
int prevAngle = 0;
int prevMode = 0; // 0=single, 1=double

void displayStaticLabels() {
  // Display static labels for the IR sensor readings
  robot.setTextSize(1);
  
  // Display the title
  robot.setTextColor(ST7735_WHITE);
  robot.setScreen(0, 0, "Comp Eye");

  // Display sensor labels (E0-E11) in a 6x2 grid
  robot.setTextColor(ST7735_CYAN);
  for (int i = 0; i < 6; i++) {
    robot.setScreen(0, 2 + i, "E");
    robot.setScreen(1, 2 + i, i);
    robot.setScreen(8, 2 + i, "E");
    robot.setScreen(9, 2 + i, i + 6);
  }
  // Display static label for ball angle
  robot.setTextColor(ST7735_WHITE);
  robot.setScreen(0, 9, "Angle:");

  // Display static labels for max eye info
  robot.setScreen(0, 11, "Max:");
  robot.setScreen(0, 13, "Val:");

  // Display static label for detection mode
  robot.setScreen(0, 15, "Mode:");
}

void setup() {
  robot.init();

  // Initialize TFT display
  robot.setTextSize(2);
  robot.setTextColor(ST7735_WHITE);
  robot.clearScreen();

  displayStaticLabels();
}

void loop() {
  // Read all 12 IR sensor values
  uint8_t* ir = robot.compoundEyeReadAll();

  // Read maximum eye and its reading
  int maxEye = robot.compoundMaxEyeRead();
  int maxEyeVal = robot.compoundMaxEyeValueRead();

  // Read ball angle
  int ballAngle = robot.compoundEyeAngleRead();
  // Read detection mode
  int detectionMode = robot.compoundEyeModeRead();

  // Display IR readings in a 6x2 grid
  robot.setTextSize(1);
  for (int i = 0; i < 6; i++) {
    // Clear previous values
    robot.setTextColor(ST7735_BLACK);
    robot.setScreen(3, 2 + i, prevEye[i]);
    robot.setScreen(12, 2 + i, prevEye[i + 6]);

    // Display current values
    robot.setTextColor(ST7735_GREEN);
    robot.setScreen(3, 2 + i, ir[i]);
    robot.setScreen(12, 2 + i, ir[i + 6]);

    // Update previous values
    prevEye[i] = ir[i];
    prevEye[i + 6] = ir[i + 6];
  }

  // Clear previous max eye values
  robot.setTextColor(ST7735_BLACK);
  robot.setScreen(6, 9, prevAngle);
  robot.setScreen(4, 11, prevMaxEye);
  robot.setScreen(4, 13, prevMaxEyeVal);
  robot.setScreen(5, 15, prevMode);

  // Display current max eye values
  robot.setTextColor(ST7735_YELLOW);
  robot.setScreen(6, 9, ballAngle);
  robot.setScreen(4, 11, maxEye);
  robot.setScreen(4, 13, maxEyeVal);
  robot.setScreen(5, 15, detectionMode);

  // Update previous values
  prevAngle = ballAngle;
  prevMaxEye = maxEye;
  prevMaxEyeVal = maxEyeVal;
  prevMode = detectionMode;

  // Draw ball angle pointer, Center at (90, 100), radius 30
  robot.drawAnglePointer(90, 120, 30, ballAngle);

  delay(100);
}