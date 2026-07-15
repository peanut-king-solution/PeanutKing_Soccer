#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // Read all 12 IR sensor values
  uint8_t* ir = robot.compoundEyeRead();
  
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
  int maxEye = robot.compoundMaxEye();
  // Maximum Eye Reading - The reading from the Maximum Eye
  int maxEyeReading = robot.compoundMaxEyeVal();

  // Print the maximum eye and its reading to the Serial Monitor
  Serial.print("MaxEye:");
  Serial.print(maxEye);
  Serial.print("EyeVal:");
  Serial.println(maxEyeReading);
}
