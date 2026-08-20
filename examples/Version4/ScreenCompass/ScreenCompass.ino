/**
 * This example demonstrates how to read compass heading and display it
 * with a direction arrow on the TFT screen.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // Initialize TFT display
  robot.screenClear();

  // Display title
  robot.screenSetTextSize(2);
  robot.screenSetTextColor(ST7735_WHITE);
  robot.screenPrintText(0, 0, "Compass");
}

// Global variable to store the previous heading for clearing the display
int prevHeading = 0;

void loop() {
  robot.buttonUpdate(); // Update button states
  if (robot.buttonStateRead(Button1) == ButtonPressed) {
    robot.compassUpdateNorth(); // Set current heading as new north reference
  }

  // Read compass heading
  int heading = robot.compassRead();

  // Set text size for heading display
  robot.screenSetTextSize(3);

  // Clear previous heading display
  robot.screenSetTextColor(ST7735_BLACK);
  robot.screenPrintNumber(2, 2, prevHeading);

  // Display current heading
  robot.screenSetTextColor(ST7735_YELLOW);
  robot.screenPrintNumber(2, 2, heading);

  // Update previous heading for next loop iteration
  prevHeading = heading;

  // Draw heading direction indicator
  robot.screenDrawAnglePointer(64, 120, 25, heading);

  delay(50);
}