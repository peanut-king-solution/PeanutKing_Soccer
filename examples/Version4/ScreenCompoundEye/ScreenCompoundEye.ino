/**
 * This example demonstrates how to read IR compound eye sensors and display
 * the individual IR readings, the ball angle indicator, and the maximum eye
 * value on the TFT screen.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

const int eyeRowStart = 2;      // Starting row for displaying eye readings
const int eyelabelColLeft = 0;  // Column for displaying eye labels (E0-E5)
const int eyeValueColright = 8; // Column for displaying eye labels (E6-E11)

const int infoCol = 0;        // Column for displaying ball angle, max eye, max value, and mode
const int angleRow = 9;       // Row for displaying ball angle
const int maxEyeRow = 11;     // Row for displaying max eye index
const int maxEyeValRow = 13;  // Row for displaying max eye value
const int modeRow = 15;       // Row for displaying detection mode (single/double)

// Global variables to store previous values for clearing the display
int prevEye[12] = {0};
int prevMaxEye = 0;
int prevMaxEyeVal = 0;
int prevAngle = 0;
int prevMode = 0; // 0=single, 1=double

// Display static labels for the IR sensor readings
void displayStaticLabels() {
  // Display the title
  robot.screenSetTextSize(2);
  robot.screenSetTextColor(ST7735_WHITE);
  robot.screenPrintText(0, 0, "Comp Eye");

  // Display sensor labels (E0-E11) in a 6x2 grid
  robot.screenSetTextSize(1);
  robot.screenSetTextColor(ST7735_CYAN);
  for (int i = 0; i < 6; i++) {
    // Left column (E0-E5)
    robot.screenPrintText(eyelabelColLeft, eyeRowStart + i, "E");
    robot.screenPrintNumber(eyelabelColLeft + 1, eyeRowStart + i, i);
    // Right column (E6-E11)
    robot.screenPrintText(eyeValueColright, eyeRowStart + i, "E");
    robot.screenPrintNumber(eyeValueColright + 1, eyeRowStart + i, i + 6);
  }

  // Display static label for ball angle
  robot.screenSetTextColor(ST7735_WHITE);
  robot.screenPrintText(infoCol, angleRow, "Angle:");

  // Display static labels for max eye info
  robot.screenPrintText(infoCol, maxEyeRow, "Max:");
  robot.screenPrintText(infoCol, maxEyeValRow, "Val:");

  // Display static label for detection mode
  robot.screenPrintText(infoCol, modeRow, "Mode:");
}

void setup() {
  robot.init();

  // Initialize TFT display
  robot.screenClear();

  // Draw static labels for the IR sensor readings
  displayStaticLabels();

  // Set text size for dynamic values
  robot.screenSetTextSize(1);
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
  for (int i = 0; i < 6; i++) {
    // Clear previous values
    robot.screenSetTextColor(ST7735_BLACK);
    robot.screenPrintNumber(eyelabelColLeft + 3, 2 + i, prevEye[i]);      // Left column (E0-E5)
    robot.screenPrintNumber(eyeValueColright + 4, 2 + i, prevEye[i + 6]); // Right column (E6-E11)

    // Display current values
    robot.screenSetTextColor(ST7735_GREEN);
    robot.screenPrintNumber(eyelabelColLeft + 3, 2 + i, ir[i]);       // Left column (E0-E5)
    robot.screenPrintNumber(eyeValueColright + 4, 2 + i, ir[i + 6]);  // Right column (E6-E11)

    // Update previous values
    prevEye[i] = ir[i];           // Left column (E0-E5)
    prevEye[i + 6] = ir[i + 6];   // Right column (E6-E11)
  }

  // Clear previous max eye values
  robot.screenSetTextColor(ST7735_BLACK);
  robot.screenPrintNumber(infoCol + 6, angleRow, prevAngle);
  robot.screenPrintNumber(infoCol + 4, maxEyeRow, prevMaxEye);
  robot.screenPrintNumber(infoCol + 4, maxEyeValRow, prevMaxEyeVal);
  robot.screenPrintNumber(infoCol + 5, modeRow, prevMode);

  // Display current max eye values
  robot.screenSetTextColor(ST7735_YELLOW);
  robot.screenPrintNumber(infoCol + 6, angleRow, ballAngle);
  robot.screenPrintNumber(infoCol + 4, maxEyeRow, maxEye);
  robot.screenPrintNumber(infoCol + 4, maxEyeValRow, maxEyeVal);
  robot.screenPrintNumber(infoCol + 5, modeRow, detectionMode);

  // Update previous values
  prevAngle = ballAngle;
  prevMaxEye = maxEye;
  prevMaxEyeVal = maxEyeVal;
  prevMode = detectionMode;

  // Draw ball angle pointer, Center at (90, 100), radius 30
  robot.screenDrawAnglePointer(90, 120, 30, ballAngle);

  delay(100);
}