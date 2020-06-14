#include <PeanutKing_Soccer_V2.h>

PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

void setup() {
  robot.init();
  robot.enableScanning(false, ALLSENSORS, false);
  delay(1000);
}

void loop() {
  int frontDistance = robot.ultrasonicRead(front);

  if (frontDistance>15) {
    robot.motorSet(0, 70);
    robot.motorSet(1, 70);
    robot.motorSet(2, -70);
    robot.motorSet(3, -70);
  }
  else {
    robot.motorSet(0, 0);
    robot.motorSet(1, 0);
    robot.motorSet(2, 0);
    robot.motorSet(3, 0);
  }
  delay(10);
}
