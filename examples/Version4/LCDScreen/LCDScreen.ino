/**
 * This example demonstrates how to use the TFT display on the PeanutKingSoccerV4 robot.
 * It shows how to display text, numbers, shapes, and update the screen in a loop.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // Set the screen to black
  robot.screenClear();

  // Set text size and color for the display
  robot.screenSetTextSize(1);
  robot.screenSetTextColor(ST7735_WHITE);
  
  // Display a welcome message on the screen
  robot.screenPrintText(0, 0, "Hello World.");
}

// Global variable to store the current tick count
uint32_t curr_tick = 0;
float angle = 0.0f;

void loop() {
  // Clear the previous tick count from the screen
	robot.screenSetTextColor(ST7735_BLACK);
  robot.screenPrintNumber(0, 1, (int16_t)curr_tick);
  
  // Update the current tick count
  curr_tick = millis();
  
  // Display the current tick count on the screen
  robot.screenSetTextColor(ST7735_WHITE);
  robot.screenPrintNumber(0, 1, (int16_t)curr_tick);

  // Display compass heading on the screen
  robot.screenDrawAnglePointer(90, 60, 20, angle, ST7735_YELLOW);
  angle += 1.0f; // Increment the angle for demonstration
  if (angle >= 360.0f) { angle = 0.0f; } // Reset angle after a full rotation

  // Draw some shapes on the screen
  robot.tft.fillCircle(40, 30, 10, ST7735_BLUE);
  robot.tft.drawRect(40, 100, 50, 20, ST7735_YELLOW);
  robot.tft.fillTriangle(45, 60, 10, 80, 60, 80, ST7735_MAGENTA);
}
