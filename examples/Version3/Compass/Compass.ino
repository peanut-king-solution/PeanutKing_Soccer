#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();


void setup() {
  robot.init();
}

void loop() {
  int angle = robot.compassRead(); // Read data from compass
  robot.setScreen(0,0,angle,3);    // Show data on LCD monitor (Column, row, number, digit)
  delay(10);
}
