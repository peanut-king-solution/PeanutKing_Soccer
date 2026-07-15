/**
 * This example demonstrates how to read color sensors and display
 * the detected color name, RGB, and HSL values on the TFT screen.
 */

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

char* color_name[] = {
  "BLACK",
  "WHITE",
  "GREY",
  "RED",
  "GREEN",
  "BLUE",
  "YELLOW",
  "CYAN"
};

// Global variables to store previous values for clearing the display
uint8_t prevColorIdx[4] = {0};
uint16_t prevR[4] = {0}, prevG[4] = {0}, prevB[4] = {0};

void setup() {
  robot.init();

  // Initialize TFT display
  robot.setTextSize(2);
  robot.setTextColor(ST7735_WHITE);
  robot.clearScreen();

  // Display title
  robot.setScreen(0, 0, "Color");

  // Display column headers
  robot.setTextSize(1);
  robot.setTextColor(ST7735_CYAN);

  uint8_t title_row = 2;
  robot.setScreen(4, title_row, "Color");
  robot.setScreen(12, title_row, "R");
  robot.setScreen(15, title_row, "G");
  robot.setScreen(18, title_row, "B");
}

void loop() {
  // Read color values from all four sensors
  uint8_t colorIdx[4]; rgb_t rgb[4]; hsl_t hsl[4];

  for (uint8_t i = 0; i < 4; i++) {
    colorIdx[i] = robot.colorSensor.readColor((CLR_SENSOR_ID)i);
    rgb[i] = robot.colorSensor.readRGB((CLR_SENSOR_ID)i);
    hsl[i] = robot.colorSensor.readHSL((CLR_SENSOR_ID)i);
  }

  // Display data for each sensor
  robot.setTextSize(1);
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t row = 3 + i + i;

    // Clear previous color name
    robot.setTextColor(ST7735_BLACK);
    robot.setScreen(4, row, color_name[prevColorIdx[i]]);

    // Display sensor ID
    robot.setTextColor(ST7735_YELLOW);
    robot.setScreen(0, row, "CL");
    robot.setScreen(2, row, (int16_t)(i + 1));

    // Display current color name
    robot.setTextColor(ST7735_GREEN);
    robot.setScreen(4, row, color_name[colorIdx[i] <= 7 ? colorIdx[i] : 0]);

    // Clear previous RGB values
    robot.setTextColor(ST7735_BLACK);
    robot.setScreen(12, row, (int16_t)prevR[i]);
    robot.setScreen(15, row, (int16_t)prevG[i]);
    robot.setScreen(18, row, (int16_t)prevB[i]);

    // Display current RGB values
    robot.setTextColor(ST7735_WHITE);
    robot.setScreen(12, row, (int16_t)rgb[i].r);
    robot.setScreen(15, row, (int16_t)rgb[i].g);
    robot.setScreen(18, row, (int16_t)rgb[i].b);

    // Update previous values for next loop iteration
    prevColorIdx[i] = colorIdx[i];
    prevR[i] = rgb[i].r;
    prevG[i] = rgb[i].g;
    prevB[i] = rgb[i].b;
  }

  delay(100);
}