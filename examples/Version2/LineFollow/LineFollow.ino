#include <PeanutKing_Soccer_V2.h>

PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

void setup() {
  robot.init();
  robot.enableScanning(false, ALLSENSORS, false);
  delay(1000);
}

void loop() {
  bool iswhite[4];
  for (int i=0; i<4; i++) {
    iswhite[i] = robot.whiteLineCheck(i);
  }
  
  if (!iswhite[0]) {
    robot.motorSet(0, 0);
    robot.motorSet(1, 60);
    robot.motorSet(2, 0);
    robot.motorSet(3, 0);
  }
  else if (!iswhite[1]) {
    robot.motorSet(0, 0);
    robot.motorSet(1, 0);
    robot.motorSet(2, 0);
    robot.motorSet(3, -60);
  }
  else {
    robot.motorSet(0, 0);
    robot.motorSet(1, 60);
    robot.motorSet(2, 0);
    robot.motorSet(3, -60);
  }
}
