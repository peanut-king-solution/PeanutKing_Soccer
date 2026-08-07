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
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

static const char* sensorLabels[4] = {"F", "R", "B", "L"};
static const uint8_t sensorRows[4] = {2, 4, 6, 8};

// Previous values for clearing only what changes
static char prevSensor[4][25];
static char prevBase[4][25];

void drawStaticLabels() {
  // Title
  robot.setTextColor(ST7735_CYAN);
  robot.setScreen(0, 0, (char*)"WhiteLine");

  // baseline label
  robot.setTextColor(ST7735_YELLOW);
  robot.setScreen(0, 10, (char*)"baseline:");
}

void displayWhiteLine() {
  // Display sensor data — clear old, draw new
  for (uint8_t i = 0; i < 4; i++) {
    CLR_SENSOR_ID sid = (CLR_SENSOR_ID)i;

    // Read HSL values and white line detection status
    hsl_t hsl = robot.colorSensor.readHSL(sid);
    bool  isW = robot.colorSensor.isWhiteLine(sid);

    // Format line string
    char line[25];
    snprintf(line, sizeof(line), "%s:%c H:%3d S:%3d L:%3d",
             sensorLabels[i], isW ? 'W' : 'G', hsl.h, hsl.s, hsl.l);

    // Clear previous
    robot.setTextColor(ST7735_BLACK);
    robot.setScreen(0, sensorRows[i], prevSensor[i]);

    // Draw new
    robot.setTextColor(isW ? ST7735_WHITE : ST7735_GREEN);
    robot.setScreen(0, sensorRows[i], line);

    // Store current line for next iteration
    strncpy(prevSensor[i], line, sizeof(prevSensor[i]));
  }

  // Display baseline values — clear old, draw new
  for (uint8_t i = 0; i < 4; i++) {
    CLR_SENSOR_ID sid = (CLR_SENSOR_ID)i;
    // Only display baseline if calibrated
    if (robot.colorSensor.isCalibrated(sid)) {
      GreenBaseLine bl = robot.colorSensor.getBaseline(sid);
      
      // Format baseline string
      char base[25];
      snprintf(base, sizeof(base), "%s-> H:%3d S:%3d L:%3d",
               sensorLabels[i], bl.greenHue, bl.greenSat, bl.greenLight);

      // Clear previous baseline
      robot.setTextColor(ST7735_BLACK);
      robot.setScreen(0, 11 + i, prevBase[i]);

      // Draw new baseline
      robot.setTextColor(ST7735_GREEN);
      robot.setScreen(0, 11 + i, base);

      // Store current baseline for next iteration
      strncpy(prevBase[i], base, sizeof(prevBase[i]));
    }
  }
}

void clearPrevData() {
  // Clear previous sensor and baseline data for display
  for (uint8_t i = 0; i < 4; i++) {
    prevSensor[i][0] = '\0';
    prevBase[i][0] = '\0';
  }
}

void ReCalibrateBaseline() {
  // Clear previous data and display "Calibrating..." message
  clearPrevData();
  robot.clearScreen();
  robot.setTextColor(ST7735_WHITE);
  robot.setScreen(0, 0, (char*)"Calibrating...");

  // Recalibrate baseline for all 4 sensors
  for (uint8_t i = 0; i < 4; i++) robot.colorSensor.calBaseline((CLR_SENSOR_ID)i);

  // After calibration, clear previous data and redraw static labels
  clearPrevData();
  robot.clearScreen();
  robot.setTextColor(ST7735_GREEN);
  robot.setScreen(0, 0, (char*)"Cal Done!");
  delay(200);
  
  robot.clearScreen();
  clearPrevData();
  drawStaticLabels();
}

void setup() {
  robot.init();
  
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