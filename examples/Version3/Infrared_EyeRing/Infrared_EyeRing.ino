#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();

void setup() {
  robot.init();
}

void loop() {
  // print all Eyes (Infrared eyering has 12 infrared sensor)
  robot.compoundEyeRead();
  
  // print maxEye, maxEyeReading
  int maxEye = robot.compoundEyeRead(13);             // Maximum Eye         - The infrared senor with highest reading
  int maxEyeReading = robot.compoundEyeRead(14);      // Maximum Eye Reading - The reading from the Maximum Eye

  robot.setScreen(0, 0, maxEye, 4);               // Show data on LCD monitor (Column, row, number, digit)
  robot.setScreen(7, 0, maxEyeReading,4);         // Show data on LCD monitor (Column, row, number, digit)
}
