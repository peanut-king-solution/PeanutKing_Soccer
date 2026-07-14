/**
 * This example demonstrates how to read 4 ultrasonic sensors and display
 * the distances on the TFT screen.
 */

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

// Global variables to store previous distances for clearing the display
uint16_t prevDist[4] = {0};

void setup() {
  robot.init();

  // Initialize TFT display
  robot.setTextSize(2);
  robot.setTextColor(ST7735_WHITE);
  robot.clearScreen();

  // Display title
  robot.setScreen(0, 0, "Xsound");
}

void loop() {
  // Read distances from all four ultrasonic sensors
  uint16_t dist[4];
  dist[0] = robot.ultrasonicRead(U1);     // Front
  dist[1] = robot.ultrasonicRead(U2);     // Right
  dist[2] = robot.ultrasonicRead(U3);     // Back
  dist[3] = robot.ultrasonicRead(U4);     // Left

  // Sensor labels (drawn once, no need to clear)
  robot.setTextSize(1);
  robot.setTextColor(ST7735_CYAN);
  robot.setScreen(0, 2, "U1(F):");
  robot.setScreen(0, 4, "U2(R):");
  robot.setScreen(0, 6, "U3(B):");
  robot.setScreen(0, 8, "U4(L):");

  // Display distance values (mm) with clear of previous values
  robot.setTextSize(2);
  for (uint8_t i = 0; i < 4; i++) {
    // Clear previous distance
    robot.setTextColor(ST7735_BLACK);
    robot.setScreen(7, 2 + i * 2, (int16_t)prevDist[i]);

    // Display current distance
    robot.setTextColor(ST7735_GREEN);
    robot.setScreen(7, 2 + i * 2, (int16_t)dist[i]);

    // Update previous distance for next loop iteration
    prevDist[i] = dist[i];
  }

  // Display unit (drawn once, no need to clear)
  robot.setTextSize(1);
  robot.setTextColor(ST7735_WHITE);
  robot.setScreen(15, 2, "mm");
  robot.setScreen(15, 4, "mm");
  robot.setScreen(15, 6, "mm");
  robot.setScreen(15, 8, "mm");

  delay(100);
}