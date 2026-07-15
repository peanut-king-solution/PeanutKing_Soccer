/**
 * This example demonstrates how to read IR compound eye sensors and display
 * the individual IR readings and the maximum eye value on the TFT screen.
 */

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

// Global variables to store previous values for clearing the display
uint16_t prevEye[12] = {0};
uint16_t prevMaxEye = 0;
uint16_t prevMaxEyeVal = 0;

void setup() {
  robot.init();

  // Initialize TFT display
  robot.setTextSize(2);
  robot.setTextColor(ST7735_WHITE);
  robot.clearScreen();

  // Display title
  robot.setScreen(0, 0, "Comp Eye");
}

void loop() {
  // Read all 12 IR sensor values
  uint8_t* ir = robot.compoundEyeRead();

  // Read maximum eye and its reading
  uint8_t maxEye = robot.compoundMaxEye();
  uint8_t maxEyeVal = robot.compoundMaxEyeVal();

  // Sensor labels (drawn once, no need to clear)
  robot.setTextSize(1);
  robot.setTextColor(ST7735_CYAN);

  // Display IR readings in a 6x2 grid
  for (uint8_t i = 0; i < 6; i++) {
    robot.setScreen(0, 2 + i, "E");
    robot.setScreen(1, 2 + i, (int16_t)i);
    robot.setScreen(8, 2 + i, "E");
    robot.setScreen(9, 2 + i, (int16_t)(i + 6));
  }

  // Display IR values with clear of previous values
  robot.setTextSize(1);
  for (uint8_t i = 0; i < 6; i++) {
    // Clear previous values
    robot.setTextColor(ST7735_BLACK);
    robot.setScreen(3, 2 + i, (int16_t)prevEye[i]);
    robot.setScreen(11, 2 + i, (int16_t)prevEye[i + 6]);

    // Display current values
    robot.setTextColor(ST7735_GREEN);
    robot.setScreen(3, 2 + i, (int16_t)ir[i]);
    robot.setScreen(11, 2 + i, (int16_t)ir[i + 6]);

    // Update previous values
    prevEye[i] = ir[i];
    prevEye[i + 6] = ir[i + 6];
  }

  // Display max eye info
  robot.setTextSize(1);
  robot.setTextColor(ST7735_WHITE);
  robot.setScreen(0, 9, "Max:");
  robot.setScreen(7, 9, "Val:");

  // Clear previous max eye values
  robot.setTextColor(ST7735_BLACK);
  robot.setScreen(4, 9, (int16_t)prevMaxEye);
  robot.setScreen(11, 9, (int16_t)prevMaxEyeVal);

  // Display current max eye values
  robot.setTextColor(ST7735_YELLOW);
  robot.setScreen(4, 9, (int16_t)maxEye);
  robot.setScreen(11, 9, (int16_t)maxEyeVal);

  // Update previous values
  prevMaxEye = maxEye;
  prevMaxEyeVal = maxEyeVal;

  delay(100);
}