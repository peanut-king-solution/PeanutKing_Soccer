#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();


void setup() {
  robot.init();
}

void loop() {
  Serial.print("Angle:");
  Serial.println(robot.compassRead());
  
  delay(10);
}
