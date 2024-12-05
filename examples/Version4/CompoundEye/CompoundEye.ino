#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // print all Eyes (Infrared eyering has 12 infrared sensor)
  
    uint8_t* ir = robot.compoundEyeRead();
    Serial.print("Eye:");
    for(int i = 0;i<12;i++){
      Serial.print(ir[i]);
      Serial.print(" ");
    }
  
  // print maxEye, maxEyeReading
  int maxEye = robot.compoundMaxEye();             // Maximum Eye         - The infrared senor with highest reading
  int maxEyeReading = robot.compoundMaxEyeVal();      // Maximum Eye Reading - The reading from the Maximum Eye

  Serial.print("MaxEye:");
  Serial.print(maxEye);               // Show data on LCD monitor (Column, row, number, digit)
  Serial.print("EyeVal:");
  Serial.println(maxEyeReading);         // Show data on LCD monitor (Column, row, number, digit)
}
