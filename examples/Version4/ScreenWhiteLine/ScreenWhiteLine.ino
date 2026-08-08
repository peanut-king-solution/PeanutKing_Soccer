/*
 * ScreenWhiteLine - TFT display of 4 color sensors + white line detection
 *
 * Layout:
 *   Row 0: WhiteLine
 *   Row 2: F:W H:126 S:45 L:46
 *   Row 4: R:G H:130 S:55 L:45
 *   Row 6: B:G H:128 S:50 L:44
 *   Row 8: L:W H:167 S:12 L:62
 *   Row10: baseline:
 *   Row11: F-> H:126 S:50 L:45
 *   Row12: R-> H:130 S:55 L:48
 *   Row13: B-> H:125 S:52 L:44
 *   Row14: L-> H:128 S:48 L:46
 *
 * Calibration: runs at startup — ensure robot is on green field.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

const char* sensorLabels[4] = {"F", "R", "B", "L"};
const uint8_t sensorRows[4] = {2, 4, 6, 8};

// Previous values for clearing only what changes
char prevSensor[4][25];
char prevBase[4][25];

void drawStaticLabels() {
  // Title
  robot.setTextColor(ST7735_CYAN);
  robot.setScreen(0, 0, "WhiteLine");

  // baseline label
  robot.setTextColor(ST7735_YELLOW);
  robot.setScreen(0, 10, "baseline:");
}

// Display sensor HSL and white line checking data (Front, Right, Back, Left) — clear old, draw new
void displayColorSensorData() {
  for (int pos = Front; pos < PositionCount; pos++) {
    // Read HSL values and white line detection status
    HSL hsl = robot.colorSensorReadHSL(pos);
    bool isWhite = robot.isWhiteLine(pos);

    // Format line string
    char line[25];
    snprintf(line, sizeof(line), "%s:%c H:%3d S:%3d L:%3d",
             sensorLabels[pos], isWhite ? 'W' : 'G', hsl.h, hsl.s, hsl.l);

    // Clear previous
    robot.setTextColor(ST7735_BLACK);
    robot.setScreen(0, sensorRows[pos], prevSensor[pos]);

    // Draw new
    robot.setTextColor(isWhite ? ST7735_WHITE : ST7735_GREEN);
    robot.setScreen(0, sensorRows[pos], line);

    // Store current line for next iteration
    strncpy(prevSensor[pos], line, sizeof(prevSensor[pos]));
  }
}

// Display the green baseline values (Front, Right, Back, Left) — clear old, draw new
void displayGreenBaselineInfo() {
  for (int pos = Front; pos < PositionCount; pos++) {
    // Get the baseline values for the current sensor position
    GreenBaseline baseline = robot.colorSensorGetBaseline(pos);
    // Only display baseline if calibrated
    if (baseline.calibrated) {
      // Format baseline string
      char base[25];
      snprintf(base, sizeof(base), "%s-> H:%3d S:%3d L:%3d",
               sensorLabels[pos], baseline.greenHue, baseline.greenSat, baseline.greenLight);

      // Clear previous baseline
      robot.setTextColor(ST7735_BLACK);
      robot.setScreen(0, 11 + pos, prevBase[pos]);

      // Draw new baseline
      robot.setTextColor(ST7735_GREEN);
      robot.setScreen(0, 11 + pos, base);

      // Store current baseline for next iteration
      strncpy(prevBase[pos], base, sizeof(prevBase[pos]));
    }
  }
}

void displayWhiteLine() {
  // Display HSL values and white line detection
  displayColorSensorData();

  // Display green baseline values
  displayGreenBaselineInfo();
}

void clearPrevData() {
  // Clear previous sensor and baseline data for display
  for (int pos = Front; pos < PositionCount; pos++) {
    prevSensor[pos][0] = '\0';
    prevBase[pos][0] = '\0';
  }
}

void ReCalibrateBaseline() {
  // Clear previous data and display "Calibrating..." message
  clearPrevData();
  robot.clearScreen();
  robot.setTextColor(ST7735_WHITE);
  robot.setScreen(0, 0, "Calibrating...");

  // Recalibrate baseline for all 4 sensors
  for (int pos = Front; pos < PositionCount; pos++) {
    robot.colorSensorCalBaseline(pos);
  }

  // After calibration, clear previous data and redraw static labels
  clearPrevData();
  robot.clearScreen();
  robot.setTextColor(ST7735_GREEN);
  robot.setScreen(0, 0, "Calibration Done!");
  delay(200);
  
  robot.clearScreen();
  clearPrevData();
  drawStaticLabels();
}

void setup() {
  robot.init();

  // Configure color sensor ports if needed (detail description see ColorSensor.ino example). 
  // Default mapping is CL1=Front, CL2=Right, CL3=Back, CL4=Left.
  // robot.colorSensorConfiguration(CL1, CL2, CL3, CL4);
  
  // Clear previous data and draw static labels
  robot.clearScreen();
  clearPrevData();
  drawStaticLabels();
  // Recalibrate baseline at startup
  ReCalibrateBaseline();
}

void loop() {
  // Update button state machine
  robot.buttonUpdate();

  // If button 1 is pressed, recalibrate baseline;
  if (robot.buttonStateRead(Button1) == ButtonPressed) {
    ReCalibrateBaseline();
  }
  // Otherwise, display sensor readings and white line detection
  else {
    displayWhiteLine();
  }
  delay(100);
}