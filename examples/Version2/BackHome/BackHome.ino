#include <PeanutKing_Soccer_V2.h>

PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

void setup() {
  robot.init();
  robot.enableScanning(false, ALLSENSORS, false);
  delay(1000);
}

void loop() {
  int16_t 
    moveSpeed = 80,
    MaxEye  = robot.compoundEyeRead(13),
    reading = robot.compoundEyeRead(14),
    XL = robot.ultrasonicRead(left),
    XR = robot.ultrasonicRead(right),
    XB = robot.ultrasonicRead(back),
    angle = robot.compassRead(),
    y = XB - 12,
    x = (XL-XR)/2;
    
  if (angle<350 && angle>180) {
    robot.motorSet(0, 35);
    robot.motorSet(1, 35);
    robot.motorSet(2, 35);
    robot.motorSet(3, 35);
  }
  else if (angle>10 && angle<=180) {
    robot.motorSet(0, -35);
    robot.motorSet(1, -35);
    robot.motorSet(2, -35);
    robot.motorSet(3, -35);
  }
  else if ( x > 5 ) {
    robot.motorSet(0, -70);
    robot.motorSet(1, 70);
    robot.motorSet(2, 70);
    robot.motorSet(3, -70);
  }
  else if ( x < -5 ) {
    robot.motorSet(0, 70);
    robot.motorSet(1, -70);
    robot.motorSet(2, -70);
    robot.motorSet(3, 70);
  }
  else if ( y > 4 ) {
    robot.motorSet(0, -70);
    robot.motorSet(1, -70);
    robot.motorSet(2, 70);
    robot.motorSet(3, 70);
  }
  else if ( y < -4 ) {
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
}
