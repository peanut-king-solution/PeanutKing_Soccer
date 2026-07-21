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

void setup() {
  robot.init();

  /*
  By default, only CL1-CL4 sensors are enabled (mask 0x0F).
  CL5-CL8 are disabled and will return 0 when read.

  If you have fewer sensors connected, you can disable unused ones:

  Method 1: Use setEnabled() with a bitmask
  */
  // robot.colorSensor.setEnabled(0b00001111);  // Enable CL1-CL4 only

  /*
  Method 2: Use enableSensor() to control individual sensors
  */
  // robot.colorSensor.enableSensor(CL1, true);   // Enable CL1
  // robot.colorSensor.enableSensor(CL2, false);  // Disable CL2
  // robot.colorSensor.enableSensor(CL3, true);   // Enable CL3
  // robot.colorSensor.enableSensor(CL4, false);  // Disable CL4
}

void loop() {
  // Read color index from CL1 sensor
  CLR_SENSOR_ID colorSensor = CL1;

  // Read color index from CL1 sensor
  uint8_t colorIdx = robot.colorSensor.readColor(colorSensor);

  // Alternatively, use the wrapper function for compatibility:
  // uint8_t colorIdx = robot.getColorSensor(colorSensor);

  // Read RGB values from CL1 sensor
  rgb_t rgb = robot.colorSensor.readRGB(colorSensor);

  // Alternatively, use the wrapper function:
  // rgb_t rgb = robot.getColorSensorRGB(colorSensor);

  // Read HSL values from CL1 sensor
  hsl_t hsl = robot.colorSensor.readHSL(colorSensor);

  // Alternatively, use the wrapper function:
  // hsl_t hsl = robot.getColorSensorHSL(colorSensor);

  // Read RGBC raw values from CL1 sensor
  rgbc_t rgbc = robot.colorSensor.readRGBRaw(colorSensor);

  // Or you can read all sensors at once using dataFetch()
  // robot.dataFetch();
  // uint8_t colorIdx = robot.colorRGB[colorSensor];
  // hsl_t hsl = robot.colorHSL[colorSensor];

  // Print the readings to the Serial Monitor
  Serial.print("CL1 - Color: ");
  if (colorIdx <= 7) {
    Serial.print(color_name[colorIdx]);
  } else {
    Serial.print("UNKNOWN");
  }

  // Print RGB values
  Serial.print(", RGB: ");
  Serial.print(rgb.r);
  Serial.print(", ");
  Serial.print(rgb.g);
  Serial.print(", ");
  Serial.print(rgb.b);
  // Print HSL values
  Serial.print(", HSL: ");
  Serial.print(hsl.h);
  Serial.print(", ");
  Serial.print(hsl.s);
  Serial.print(", ");
  Serial.print(hsl.l);
  // Print RGBC raw values
  Serial.print(", RGBC: ");
  Serial.print(rgbc.r);
  Serial.print(", ");
  Serial.print(rgbc.g);
  Serial.print(", ");
  Serial.print(rgbc.b);
  Serial.print(", ");
  Serial.print(rgbc.c);
  // Print white line detection
  Serial.print(", White: ");
  Serial.print(robot.colorSensor.isWhiteLine(colorSensor) ? "YES" : "NO");
  Serial.println();

  delay(100);
}