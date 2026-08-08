/**
 * This example demonstrates how to use the ColorSensor module of the PeanutKingSoccerV4 robot.
 * It reads RGB, HSL, and RGBC values from the color sensor.
 * It also checks if the color sensor detects a white line.
 * 
 * Important: To make the white line detection work correctly and more accurate, 
 * you should place the robot (especially the color sensor) on a green surface 
 * (like a green field), it will auto calculate the threshold value for white line detection.
 * If the robot is placed on a different color surface, 
 * the white line detection may not work as expected.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

const char* positionNames[4] = {"Front", "Right", "Back", "Left"};

void setup() {
  robot.init();
  
  /**
   * Assign color sensor ports to physical positions with default mapping:
   *  - Front color sensor is connected to CL1
   *  - Right color sensor is connected to CL2
   *  - Back color sensor is connected to CL3
   *  - Left color sensor is connected to CL4
   *  - Other sensors' ports (CL5~CL8) are not used and will be disabled
   * 
   * If you connect the color sensors differently, you can change the mapping here.
   */
  // robot.colorSensorConfiguration(CL1, CL2, CL3, CL4);

  // Display the calibrated baseline values for each color sensor on the Serial Monitor
  Serial.println("===================================================================");
  Serial.println("Calibrated baseline values for each color sensor:");
  for (int pos = Front; pos < PositionCount; pos++) {
    // Get the baseline values for the color sensor at the specified position
    GreenBaseline baseline = robot.colorSensorGetBaseline(pos);
    // Print the baseline values to the Serial Monitor
    if (baseline.calibrated) {
      Serial.print(positionNames[pos]);
      Serial.print(" - Green Avg Hue: ");
      Serial.print(baseline.greenHue);
      Serial.print(", Green Avg Sat: ");
      Serial.print(baseline.greenSat);
      Serial.print(", Green Avg Light: ");
      Serial.println(baseline.greenLight);
    } else {
      Serial.print(positionNames[pos]);
      Serial.println(" - Not calibrated");
    }
  }
  Serial.println("===================================================================");
  delay(3000); // Wait for 3 seconds before starting the loop
}

void loop() {
  // Select a position of the color sensor to read
  SensorPos colorSensor = Front;

  // Read RGB values from sensor in position selected
  RGB rgb = robot.colorSensorReadRGB(colorSensor);

  // Read HSL values from sensor in position selected
  HSL hsl = robot.colorSensorReadHSL(colorSensor);

  // Read RGBC raw values from sensor in position selected
  RGBC rgbc = robot.colorSensorReadRGBC(colorSensor);

  // Check if the color sensor in position selected detects a white line
  bool isWhiteDetected = robot.isWhiteLine(colorSensor);

  // Or you can read all sensors at once using dataFetch()
  // robot.dataFetch();
  // RGB rgb = robot.colorRGB[colorSensor];
  // HSL hsl = robot.colorHSL[colorSensor];
  // RGBC rgbc = robot.colorRGBC[colorSensor];
  // bool isWhiteDetected = robot.isWhite[colorSensor];

  // Print the readings to the Serial Monitor
  Serial.print(positionNames[colorSensor]);
  Serial.print(" color sensor");

  // Print RGB values
  Serial.print(", RGB: ");
  Serial.print(rgb.r);   // Print red value (0-255)
  Serial.print(", ");
  Serial.print(rgb.g);   // Print green value (0-255)
  Serial.print(", ");
  Serial.print(rgb.b);   // Print blue value (0-255)
  // Print HSL values
  Serial.print(", HSL: ");
  Serial.print(hsl.h);   // Print hue value (0-360)
  Serial.print(", ");
  Serial.print(hsl.s);   // Print saturation value (0-100)
  Serial.print(", ");
  Serial.print(hsl.l);   // Print lightness value (0-100)
  // Print RGBC raw values
  Serial.print(", RGBC: ");
  Serial.print(rgbc.r);  // Print red raw value (0-65535)
  Serial.print(", ");
  Serial.print(rgbc.g);  // Print green raw value (0-65535)
  Serial.print(", ");
  Serial.print(rgbc.b);  // Print blue raw value (0-65535)
  Serial.print(", ");
  Serial.print(rgbc.c);  // Print clear raw value (0-65535)
  // Print white line detection
  Serial.print(", White Detected: ");
  Serial.print(isWhiteDetected ? "YES" : "NO");
  Serial.println();

  delay(100);
}