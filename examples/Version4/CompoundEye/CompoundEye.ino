/**
 * This example demonstrates how to read the compound eye sensor values
 * from the PeanutKingSoccerV4 robot.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
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
  int ballAngle = robot.compoundEyeAngleRead();

  // Print the maximum eye and its reading to the Serial Monitor
  Serial.print("MaxEye: ");
  Serial.print(maxEye);
  Serial.print(", EyeVal:" );
  Serial.print(maxEyeReading);
  Serial.print(", BallAngle: ");
  Serial.println(ballAngle);
}
