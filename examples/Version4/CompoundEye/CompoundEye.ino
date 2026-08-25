/**
 * This example demonstrates how to read the compound eye sensor values
 * from the PeanutKingSoccerV4 robot.
 *
 * Before running this example, you should ensure you have checked the following:
 *  - The compound eye module is properly connected via I2C.
 *
 * The compound eye provides:
 *  - 12 individual IR sensor readings (Eye0-Eye11)
 *  - Maximum eye index and value
 *  - Ball angle (0-360°) with coordinate conversion
 *
 * If the ball angle seems incorrect for your setup, you can adjust
 * the coordinate system using the coordinate converter functions.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  /*
  The compound eye coordinate system can be adjusted if the sensor is
  mounted at an angle or if you need to flip the direction.

  For example, if the sensor is rotated 90° clockwise on the robot,
  you should adjust the coordinate system by uncommenting the below line:

  robot.compoundEyeCoordinateRotate(90, CW);

  Or if the angle direction is reversed:

  robot.compoundEyeCoordinateFlip();
  */
}

void loop() {
  // Read all 12 IR sensor values
  uint8_t* ir = robot.compoundEyeReadAll();

  // Print the readings to the Serial Monitor
  for(int i = 0;i<12;i++){
    Serial.print("eye");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(ir[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Maximum Eye - The infrared senor with highest reading
  int maxEye = robot.compoundMaxEyeRead();
  // Maximum Eye Reading - The reading from the Maximum Eye
  int maxEyeReading = robot.compoundMaxEyeValueRead();
  // Ball Angle - The angle of the detected object based on the IR sensor readings
  // (automatically converted via the coordinate system)
  int ballAngle = robot.compoundEyeAngleRead();

  // Print the maximum eye and its reading to the Serial Monitor
  Serial.print("MaxEye: ");
  Serial.print(maxEye);
  Serial.print(", EyeVal:" );
  Serial.print(maxEyeReading);
  Serial.print(", BallAngle: ");
  Serial.println(ballAngle);
}
