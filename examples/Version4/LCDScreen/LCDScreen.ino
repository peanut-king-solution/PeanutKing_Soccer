#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // Set the screen to black
  robot.clearScreen();

  // Set text size and color for the display
  robot.setTextSize(1);
  robot.setTextColor(ST7735_WHITE);
  
  // Display a welcome message on the screen
  robot.setScreen(0, 0, "Hello World.");
}

// Global variable to store the current tick count
uint32_t curr_tick = 0;

void loop() {
  // Clear the previous tick count from the screen
	robot.setTextColor(ST7735_BLACK);
  robot.setScreen(0, 1, (int16_t)curr_tick);
  
  // Update the current tick count
  curr_tick = millis();
  
  // Display the current tick count on the screen
  robot.setTextColor(ST7735_WHITE);
  robot.setScreen(0, 1, (int16_t)curr_tick);
  
  // Draw some shapes
  robot.tft.fillCircle(40, 30, 10, ST7735_BLUE);
  robot.tft.drawRect(40, 100, 50, 20, ST7735_YELLOW);
  robot.tft.fillTriangle(45, 60, 10, 80, 60, 80, ST7735_MAGENTA);
}
