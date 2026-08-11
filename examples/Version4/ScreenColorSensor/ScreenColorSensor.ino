/**
 * This example demonstrates how to read color sensors and display
 * the detected color name, RGB, and HSL values on the TFT screen.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

// You can define your own color index enumeration for easier reference
enum ColorIndex
{
  ColorUnknown = 0,
  ColorBlack = 1,
  ColorWhite = 2,
  ColorGrey = 3,
  ColorRed = 4,
  ColorGreen = 5,
  ColorBlue = 6,
  ColorYellow = 7,
  ColorCyan = 8,
  ColorMagenta = 9
};

// Array of color names corresponding to the ColorIndex enumeration
const char *color_name[] = {
  "?",    // UNKNOWN
  "BLK", "WHT", "GRY", "RED",
  "GRN", "BLU", "YEL", "CYN",
  "MGT"   // MAGENTA
};

// Display color for each ColorIndex
const int color_disp[] = {
  ST7735_WHITE,  // ColorUnknown -> white
  ST7735_WHITE,  // ColorBlack
  ST7735_WHITE,  // ColorWhite
  ST7735_WHITE,  // ColorGrey
  ST7735_RED,    // ColorRed
  ST7735_GREEN,  // ColorGreen
  ST7735_BLUE,   // ColorBlue
  ST7735_YELLOW, // ColorYellow
  ST7735_CYAN,   // ColorCyan
  ST7735_MAGENTA // ColorMagenta
};

// Global variables to store previous values for clearing the display
int prevColorIdx[4] = {0, 0, 0, 0};
int prevR[4] = {0}, prevG[4] = {0}, prevB[4] = {0};
int prevH[4] = {0}, prevS[4] = {0}, prevL[4] = {0};

// Data columns
const int dataCol[4] = {2, 7, 12, 17};
// Data rows: R=3, G=4, B=5, H=6, S=7, L=8, C=9
const int dataRow[7] = {3, 4, 5, 6, 7, 8, 9};

// Approximate a color index from HSL values (the thresholds should be adjusted with your own findings)
int determineColor(const HSL hsl)
{
  // Extract HSL values
  int hue = hsl.h, saturation = hsl.s, lightness = hsl.l;

  // define your own thresholds for color detection based on HSL values
  const int satThreshold = 20; // Below this, consider it achromatic (black, white, grey)
  const int lightThresholdLow = 15; // Below this, consider it black
  const int lightThresholdHigh = 80; // Above this, consider it white
  const int lightThresholdUltraLow = 10; // Below this, consider it black regardless of saturation
  const int redHueLow = 0, redHueHigh = 10; // Red hue range
  const int yellowHueLow = 10,yellowHueHigh = 71; // Yellow hue range
  const int greenHueLow = 71, greenHueHigh = 151; // Green hue range
  const int cyanHueLow = 151, cyanHueHigh = 201; // Cyan hue range
  const int blueHueLow = 201, blueHueHigh = 261; // Blue hue range
  const int magentaHueLow = 261, magentaHueHigh = 341; // Magenta hue range
  const int redHueWrapLow = 341, redHueWrapHigh = 360; // Red hue wrap-around range

  // 1. Handle Achromatic Colors (Black, White, Grey) based on Saturation and Lightness
  if (saturation < satThreshold) {
    if (lightness < lightThresholdLow) { return ColorBlack; }
    if (lightness > lightThresholdHigh) { return ColorWhite; }
    return ColorGrey;
  }

  // Also catch ultra-low brightness as black regardless of saturation
  if (lightness < lightThresholdUltraLow) { return ColorBlack; }

  // 2. Handle Chromatic Colors using the Hue angle (0 to 360 degrees)
  if (hue >= redHueLow     && hue < redHueHigh)      { return ColorRed; }
  if (hue >= redHueWrapLow && hue <= redHueWrapHigh) { return ColorRed; }
  if (hue >= yellowHueLow  && hue < yellowHueHigh)   { return ColorYellow; }
  if (hue >= greenHueLow   && hue < greenHueHigh)    { return ColorGreen; }
  if (hue >= cyanHueLow    && hue < cyanHueHigh)     { return ColorCyan; }
  if (hue >= blueHueLow    && hue < blueHueHigh)     { return ColorBlue; }
  if (hue >= magentaHueLow && hue < magentaHueHigh)  { return ColorMagenta; }
  if (hue >= redHueWrapLow && hue <= redHueWrapHigh) { return ColorRed; }

  // Cannot determine color, return unknown
  return ColorUnknown;
}

void displayStaticLabels()
{
  // Display title
  robot.screenSetTextSize(2);
  robot.screenSetTextColor(ST7735_WHITE);
  robot.screenPrintText(0, 0, "Color"); // Title

  // Column headers: Front, Right, Back, Left
  robot.screenSetTextSize(1);
  robot.screenSetTextColor(ST7735_CYAN);
  robot.screenPrintText(dataCol[0] + 1, 2, "F");  // Front
  robot.screenPrintText(dataCol[1] + 1, 2, "R");  // Right
  robot.screenPrintText(dataCol[2] + 1, 2, "B");  // Back
  robot.screenPrintText(dataCol[3] + 1, 2, "L");  // Left

  // Row labels (left column)
  robot.screenSetTextColor(ST7735_CYAN);
  robot.screenPrintText(0, dataRow[0], "R");  // Red
  robot.screenPrintText(0, dataRow[1], "G");  // Green
  robot.screenPrintText(0, dataRow[2], "B");  // Blue
  robot.screenPrintText(0, dataRow[3], "H");  // Hue
  robot.screenPrintText(0, dataRow[4], "S");  // Saturation
  robot.screenPrintText(0, dataRow[5], "L");  // Lightness
  robot.screenPrintText(0, dataRow[6], "C");  // Color
}

void setup()
{
  // Initialize the robot's modules
  robot.init();

  // Initialize TFT display
  robot.screenClear();

  // Display static labels for the table
  displayStaticLabels();
  
  // All text will be printed in size 1 in loop
  robot.screenSetTextSize(1);
}

void loop()
{
  // Read RGB values from all four sensors
  RGB rgb[4];  HSL hsl[4];
  int colorIdx[4];

  // Read color data from each sensor (Front, Right, Back, Left)
  for (int pos = 0; pos < 4; pos++)
  {
    rgb[pos] = robot.colorSensorReadRGB(pos);
    hsl[pos] = robot.colorSensorReadHSL(pos);
    colorIdx[pos] = determineColor(hsl[pos]); // Approximate color name
  }

  // Display data: vertical table, one metric per row
  for (int i = 0; i < 4; i++)
  {
    // Column for each sensor (Front, Right, Back, Left)
    int col = dataCol[i];
    // Row index for each metric (R, G, B, H, S, L, Color)
    int rowIndex = 0;

    // R row - red value
    robot.screenSetTextColor(ST7735_BLACK); // Clear previous value
    robot.screenPrintNumber(col, dataRow[rowIndex], prevR[i]);
    robot.screenSetTextColor(ST7735_RED); // Display new value in red
    robot.screenPrintNumber(col, dataRow[rowIndex], rgb[i].r);
    rowIndex++; // Next row

    // G row - green value
    robot.screenSetTextColor(ST7735_BLACK); // Clear previous value
    robot.screenPrintNumber(col, dataRow[rowIndex], prevG[i]);
    robot.screenSetTextColor(ST7735_GREEN); // Display new value in green
    robot.screenPrintNumber(col, dataRow[rowIndex], rgb[i].g);
    rowIndex++; // Next row

    // B row - blue value
    robot.screenSetTextColor(ST7735_BLACK); // Clear previous value
    robot.screenPrintNumber(col, dataRow[rowIndex], prevB[i]);
    robot.screenSetTextColor(ST7735_BLUE);  // Display new value in blue
    robot.screenPrintNumber(col, dataRow[rowIndex], rgb[i].b);
    rowIndex++; // Next row

    // H row
    robot.screenSetTextColor(ST7735_BLACK); // Clear previous value
    robot.screenPrintNumber(col, dataRow[rowIndex], prevH[i]);
    robot.screenSetTextColor(ST7735_WHITE); // Display new value in white
    robot.screenPrintNumber(col, dataRow[rowIndex], hsl[i].h);
    rowIndex++; // Next row

    // S row
    robot.screenSetTextColor(ST7735_BLACK); // Clear previous value
    robot.screenPrintNumber(col, dataRow[rowIndex], prevS[i]);
    robot.screenSetTextColor(ST7735_WHITE); // Display new value in white
    robot.screenPrintNumber(col, dataRow[rowIndex], hsl[i].s);
    rowIndex++; // Next row

    // L row
    robot.screenSetTextColor(ST7735_BLACK); // Clear previous value
    robot.screenPrintNumber(col, dataRow[rowIndex], prevL[i]);
    robot.screenSetTextColor(ST7735_WHITE); // Display new value in white
    robot.screenPrintNumber(col, dataRow[rowIndex], hsl[i].l);
    rowIndex++; // Next row

    // Color name row, color follows detected color
    robot.screenSetTextColor(ST7735_BLACK); // Clear previous color name
    robot.screenPrintText(col, dataRow[rowIndex], color_name[prevColorIdx[i]]);
    robot.screenSetTextColor(color_disp[colorIdx[i]]);  // Set text color based on detected color
    robot.screenPrintText(col, dataRow[rowIndex], color_name[colorIdx[i]]); // Display new color name

    // Update previous values for next loop iteration
    prevColorIdx[i] = colorIdx[i];
    prevR[i] = rgb[i].r;
    prevG[i] = rgb[i].g;
    prevB[i] = rgb[i].b;
    prevH[i] = hsl[i].h;
    prevS[i] = hsl[i].s;
    prevL[i] = hsl[i].l;
  }

  delay(10);
}