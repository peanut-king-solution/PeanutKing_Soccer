/**
 * This example demonstrates how to read 4 ultrasonic sensors 
 * and display the distances on the TFT screen.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

// Constants for screen layout
const int dataColStart = 0;   // starting column for distance values on the screen
const int dataRowStart = 2;   // starting row for distance values on the screen
const int dataRowSpacing = 2; // row spacing between distance values

const int dataCol = dataColStart + 7; // skip 7 columns for the label, then display the distance value
const int unitCol = dataCol + 8;      // skip 8 columns for the distance value, then display the unit "mm"

// Global variables to store previous distances for clearing the display
int prevDist[4] = {0, 0, 0, 0}; // Previous distances for Front, Right, Back, Left

void displayStaticLabels() {
  // Title
  robot.screenSetTextSize(2);
  robot.screenSetTextColor(ST7735_WHITE);
  robot.screenPrintText(0, 0, "Ultrasound");

  // Sensor labels
  robot.screenSetTextSize(1);
  robot.screenSetTextColor(ST7735_CYAN);
  robot.screenPrintText(dataColStart, dataRowStart + 0 * dataRowSpacing, "Front:");
  robot.screenPrintText(dataColStart, dataRowStart + 1 * dataRowSpacing, "Right:");
  robot.screenPrintText(dataColStart, dataRowStart + 2 * dataRowSpacing, "Back:");
  robot.screenPrintText(dataColStart, dataRowStart + 3 * dataRowSpacing, "Left:");

  // Display unit (drawn once, no need to clear)
  robot.screenSetTextSize(1);
  robot.screenSetTextColor(ST7735_WHITE);
  robot.screenPrintText(unitCol, dataRowStart + 0 * dataRowSpacing, "mm");
  robot.screenPrintText(unitCol, dataRowStart + 1 * dataRowSpacing, "mm");
  robot.screenPrintText(unitCol, dataRowStart + 2 * dataRowSpacing, "mm");
  robot.screenPrintText(unitCol, dataRowStart + 3 * dataRowSpacing, "mm");
}

void setup() {
  robot.init();

  // Initialize TFT display
  robot.screenClear();
  
  // Display static labels for the ultrasonic sensors
  displayStaticLabels();

  // Set text size for distance values
  robot.screenSetTextSize(2);
}

void loop() {
  // Read distances from all four ultrasonic sensors
  int dist[4];
  dist[0] = robot.ultrasoundGetDist(Front);  // Front
  dist[1] = robot.ultrasoundGetDist(Right);  // Right
  dist[2] = robot.ultrasoundGetDist(Back);   // Back
  dist[3] = robot.ultrasoundGetDist(Left);   // Left

  // Display distance values (mm) with clear of previous values
  // Loop through each sensor position (Front, Right, Back, Left)
  for (int pos = 0; pos < PositionCount; pos++) {
    // Clear previous distance
    robot.screenSetTextColor(ST7735_BLACK);
    robot.screenPrintNumber(
      dataCol,
      dataRowStart + pos * dataRowSpacing,
      prevDist[pos]
    );

    // Display current distance
    robot.screenSetTextColor(ST7735_GREEN);
    robot.screenPrintNumber(
      dataCol, 
      dataRowStart + pos * dataRowSpacing, 
      dist[pos]
    );

    // Update previous distance for next loop iteration
    prevDist[pos] = dist[pos];
  }

  delay(100);
}