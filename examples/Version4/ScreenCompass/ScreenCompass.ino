/**
 * This example demonstrates how to read compass heading and display it
 * with a direction arrow on the TFT screen.
 */

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // Initialize TFT display
  robot.setTextSize(2);
  robot.setTextColor(ST7735_WHITE);
  robot.clearScreen();

  // Display title
  robot.setScreen(0, 0, "Compass");
}

// Global variable to store the previous heading for clearing the display
uint16_t prevHeading = 0;

void loop() {
  // Read compass heading
  uint16_t heading = robot.compass.read();

  // Set text size for heading display
  robot.setTextSize(3);

  // Clear previous heading display
  robot.setTextColor(ST7735_BLACK);
  robot.setScreen(2, 2, (int16_t)prevHeading);

  // Display current heading
  robot.setTextColor(ST7735_YELLOW);
  robot.setScreen(2, 2, (int16_t)heading);

  // Update previous heading for next loop iteration
  prevHeading = heading;

  // Draw heading direction indicator
  robot.drawAnglePointer(64, 120, 25, heading);

  delay(50);
}