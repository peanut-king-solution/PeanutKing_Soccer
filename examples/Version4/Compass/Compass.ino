#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  Serial.print("Heading:");
  Serial.println(robot.compass.read());
  delay(100);
}